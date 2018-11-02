//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
// 
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
// 
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see http://www.gnu.org/licenses/.
// 

#include "VenerisServer.h"


#include "messages/BaseTypes_generated.h"
#include "messages/Header_generated.h"
#include "messages/UseOpal_generated.h"
#include "messages/StaticMesh_generated.h"
#include "messages/CreateVehicle_generated.h"
#include "messages/DestroyVehicle_generated.h"
#include "messages/VehicleState_generated.h"
#include "messages/AddDynamicMeshGroup_generated.h"
#include "messages/RemoveDynamicMeshGroup_generated.h"
#include "messages/FinishDynamicMeshGroup_generated.h"
#include "messages/DynamicMesh_generated.h"
#include "messages/UpdateTransformInGroup_generated.h"




using namespace Veneris::Communications;
namespace inet {

Define_Module(VenerisServer);

#define CHKOPAL() if (!useOpal) {return;} if (opalRadio==nullptr) {throw cRuntimeError("OpalRadio Module not created before adding static mesh");return;}




void VenerisServer::createOpal() {

    //Remove default radioMedium
    getParentModule()->getModuleByPath(".radioMedium")->deleteModule();

    useOpal=true;
    auto msg = readMsg<UseOpal>(msg_size);
    uint32_t ad=msg->azimuthDelta();
    uint32_t ed=msg->elevationDelta();
    bool useDecimal=msg->useDecimalDegrees();
    uint32_t maxReflections=msg->maxReflections();
    float f=msg->frequency();
    cModuleType *moduleType = cModuleType::get(opalModule.c_str());
    cModule *module = moduleType->create("radioMedium", this->getParentModule());
    module->par("azimuthDelta").setIntValue(ad);
    module->par("elevationDelta").setIntValue(ed);
    module->par("carrierFrequency").setDoubleValue((double)f);
    module->par("loadMeshesFromFile").setBoolValue(false);
    module->par("maxNumberOfReflections").setIntValue(maxReflections);
    module->par("useDecimalDegreeDelta").setBoolValue(useDecimal);
    opalRadio=check_and_cast<inet::physicallayer::OpalRadioMedium*>(module);
    module->finalizeParameters();
    module->buildInside();
    module->scheduleStart(simTime());
    module->callInitialize();
    std::cout<<simTime()<<"Creating Opal context. azimuthDelta="<<ad<<"elevationDelta="<<ed<<"carrierFrequency="<<f<<"maxNumberOfReflections="<<maxReflections<<"useDecimalDegreeDelta="<<useDecimal<<endl;

}
VenerisServer::VenerisServer() {
    opalRadio=nullptr;
    useOpal=false;
}
VenerisServer::~VenerisServer() {

}

void VenerisServer::initialize(int stage) {
    ExecutionServer::initialize(stage);
    if (stage==INITSTAGE_LOCAL) {
        opalModule=par("opalModule").stdstringValue();
        vehicleModule=par("vehicleModule").stdstringValue();
        mobilityModule=par("mobilityModule").stdstringValue();

    }
}

void VenerisServer::addStaticMesh() {

    CHKOPAL();
    std::cout<<"Adding static mesh"<<std::endl;
    auto msg = readMsg<StaticMesh>(msg_size);
    auto vertices=msg->vertices();
    auto indices=msg->indexes();
    //We are copying a lot...
    int vsize=vertices->size();

    std::vector<optix::float3> vo(vsize);
    for (int i=0; i<vsize; i++){

        vo[i]=optix::make_float3(vertices->Get(i)->x(),vertices->Get(i)->y(),vertices->Get(i)->z());
        //std::cout<<"v["<<i<<"]="<<vo[i]<<std::endl;
    }
    int isize=indices->size();
    std::vector<int> io(isize);
    for (int i=0; i<isize; i++){
        io[i]=indices->Get(i);
        //std::cout<<"i["<<i<<"]="<<io[i]<<std::endl;

    }
    auto tm=msg->transform();
    optix::Matrix4x4 tmo;
    tmo.setRow(0, optix::make_float4(tm->r0().x(),tm->r0().y(),tm->r0().z(),tm->r0().w()));
    tmo.setRow(1, optix::make_float4(tm->r1().x(),tm->r1().y(),tm->r1().z(),tm->r1().w()));
    tmo.setRow(2, optix::make_float4(tm->r2().x(),tm->r2().y(),tm->r2().z(),tm->r2().w()));
    tmo.setRow(3, optix::make_float4(tm->r3().x(),tm->r3().y(),tm->r3().z(),tm->r3().w()));
    std::cout<<"transformation matrix "<<tmo<<std::endl;

    auto em=msg->material();

    MaterialEMProperties emo=opalRadio->ITUparametersToMaterial(em->a(),em->b(),em->c(),em->d());
    std::cout<<"MaterialEMProperties"<<emo.dielectricConstant<<std::endl;
    opalRadio->addStaticMesh(vsize, vo.data(), isize,io.data(), tmo, emo);

    //opalRadio->addStaticMesh(vsize, vo.data(), isize, io.data(), tmo, emo);


}

void VenerisServer::finishOpalContext() {

    CHKOPAL();
    opalRadio->finishOpalContext();
}

void VenerisServer::createVehicle() {
    auto msg = readMsg<CreateVehicle>(msg_size);
    uint id=msg->id();
    auto pos=msg->position();
    auto radius=msg->radius();
    cModuleType *moduleType = cModuleType::get(vehicleModule.c_str());
    cModule *module = moduleType->create("node", this->getParentModule(), activeVehicles.size()+1,activeVehicles.size());
    module->par("id").setIntValue(id);
    if (useOpal) {
        module->par("receptionRadius").setDoubleValue(radius);
    }
    module->par("mobilityType").setStringValue(mobilityModule);
    module->finalizeParameters();
    module->buildInside();
    module->scheduleStart(simTime());

    //Init position
    cModule *mob = module->getModuleByPath(".mobility");
    VenerisExternalMobility* ven= check_and_cast<VenerisExternalMobility*>(mob);
    ven->setPosition(pos->x(),pos->y(), pos->z());
    VehicleInfo info;
    info.module=module;
    info.mob=ven;
    std::cout<<simTime()<<": Creating  Vehicle with module="<<vehicleModule<<", id="<<id<<". Calling initialize. "<< std::endl;
    cModule *gen = module->getModuleByPath(".generator");
    if (id==0) {
        //Disable transmission

        gen->par("beaconFrequency").setDoubleValue(-1.0);
    } else {
        gen->par("transmitWhenPositionChanges").setBoolValue(true);
    }
    module->getModuleByPath(".wlan.radio.transmitter")->par("power").setDoubleValue(1.0);
    module->callInitialize();
    activeVehicles.insert(std::pair<int,VehicleInfo>(id, info));
    std::cout<<simTime()<<"Creating  Vehicle with module="<<vehicleModule<<", id="<<id<<" at "<< std::endl;

}

void VenerisServer::updateVehicleState() {
    auto msg = readMsg<VehicleState>(msg_size);
    uint id=msg->id();
    auto upos=msg->position();
    //Just update the position, the receiver in Opal is updated with the emitted signal
    try {
        VehicleInfo info=activeVehicles.at(id);
        info.mob->setPosition(upos->x(), upos->y(), upos->z());
        std::cout<<"Updating position of vehicle "<< id<< " to "<<info.mob->getCurrentPosition()<<endl;
    } catch (std::out_of_range e) {
        throw cRuntimeError("Vehicle is not registered with opal");
        return;
    }

}

void VenerisServer::addDynamicMeshGroup() {
    CHKOPAL();
    auto msg = readMsg<AddDynamicMeshGroup>(msg_size);
    const int id=msg->id();
    opalRadio->addDynamicMeshGroup(id);
}

void VenerisServer::addDynamicMesh() {
    CHKOPAL();
    auto msg = readMsg<DynamicMesh>(msg_size);
    auto vertices=msg->vertices();
    auto indices=msg->indexes();
    //We are copying a lot...
    int vsize=vertices->size();

    std::vector<optix::float3> vo(vsize);
    for (int i=0; i<vsize; i++){

        vo[i]=optix::make_float3(vertices->Get(i)->x(),vertices->Get(i)->y(),vertices->Get(i)->z());
       // std::cout<<"v["<<i<<"]="<<vo[i]<<std::endl;
    }
    int isize=indices->size();
    std::vector<int> io(isize);
    for (int i=0; i<isize; i++){
        io[i]=indices->Get(i);
       // std::cout<<"i["<<i<<"]="<<io[i]<<std::endl;

    }
    auto id=msg->id();
    auto em=msg->material();

    MaterialEMProperties emo=opalRadio->ITUparametersToMaterial(em->a(),em->b(),em->c(),em->d());
    std::cout<<"MaterialEMProperties"<<emo.dielectricConstant<<std::endl;
    opalRadio->addMeshToGroup(id, vsize, vo.data(), isize, io.data(), emo);
}

void VenerisServer::finishDynamicMeshGroup() {
    CHKOPAL();
    auto msg = readMsg<FinishDynamicMeshGroup>(msg_size);
    auto id=msg->id();
    opalRadio->finishDynamicMeshGroup(id);
}

void VenerisServer::removeDynamicMeshGroup() {
    CHKOPAL();
    auto msg = readMsg<RemoveDynamicMeshGroup>(msg_size);
    auto id=msg->id();
    opalRadio->removeDynamicMeshGroup(id);
}

void VenerisServer::updateTransformInGroup() {
    CHKOPAL();
    auto msg = readMsg<UpdateTransformInGroup>(msg_size);
    auto tm=msg->transform();
    optix::Matrix4x4 tmo;
    tmo.setRow(0, optix::make_float4(tm->r0().x(),tm->r0().y(),tm->r0().z(),tm->r0().w()));
    tmo.setRow(1, optix::make_float4(tm->r1().x(),tm->r1().y(),tm->r1().z(),tm->r1().w()));
    tmo.setRow(2, optix::make_float4(tm->r2().x(),tm->r2().y(),tm->r2().z(),tm->r2().w()));
    tmo.setRow(3, optix::make_float4(tm->r3().x(),tm->r3().y(),tm->r3().z(),tm->r3().w()));
    auto id=msg->id();
    std::cout<<"transformation matrix "<<tmo<<" in group "<<id<<std::endl;
    opalRadio->updateTransformInGroup(id, tmo);

}

void VenerisServer::destroyVehicle() {

    auto msg = readMsg<DestroyVehicle>(msg_size);
    auto id =msg->id();

    //Find module
    cModule* nodes=this->getParentModule()->getSubmodule("node");
    int s=nodes->getVectorSize();
    std::cout<<"nodes size"<<s<<endl;
    for (int i=0; i<s;i++) {
        cModule* mod=this->getParentModule()->getSubmodule("node",i);
        int modid= mod->par("id");
        if (modid=id) {
            mod->callFinish();
            //Opal remove radio is called when removing the radio from radio medium in OpalRadioMedium
            mod->deleteModule();
            break;
        }
    }
    std::cout<<"nodes size"<<nodes->getVectorSize()<<endl;
}

void VenerisServer::processMessage(Veneris::Communications::VenerisMessageTypes type, uint32_t size) {
    ExecutionServer::processMessage(type,size);
    if (type==Veneris::Communications::VenerisMessageTypes::VenerisMessageTypes_ExternalTime) {
        return;
    }
    switch (type) {
    case Veneris::Communications::VenerisMessageTypes::VenerisMessageTypes_UseOpal:
        createOpal();
        break;
    case Veneris::Communications::VenerisMessageTypes::VenerisMessageTypes_StaticMesh:
        addStaticMesh();
        break;
    case Veneris::Communications::VenerisMessageTypes::VenerisMessageTypes_FinishOpalContext:
        finishOpalContext();
        break;
    case Veneris::Communications::VenerisMessageTypes::VenerisMessageTypes_End:
        endServer();
        break;
    case Veneris::Communications::VenerisMessageTypes::VenerisMessageTypes_Create:
        createVehicle();
        break;
    case Veneris::Communications::VenerisMessageTypes::VenerisMessageTypes_Destroy:
            destroyVehicle();
            break;
    case Veneris::Communications::VenerisMessageTypes::VenerisMessageTypes_VehicleState:
        updateVehicleState();
        break;
    case Veneris::Communications::VenerisMessageTypes::VenerisMessageTypes_AddDynamicMeshGroup:
        addDynamicMeshGroup();
        break;
    case Veneris::Communications::VenerisMessageTypes::VenerisMessageTypes_DynamicMesh:
        addDynamicMesh();
        break;
    case Veneris::Communications::VenerisMessageTypes::VenerisMessageTypes_UpdateTransformInGroup:
        updateTransformInGroup();
        break;
    case Veneris::Communications::VenerisMessageTypes::VenerisMessageTypes_RemoveDynamicMeshGroup:
        removeDynamicMeshGroup();
        break;
    case Veneris::Communications::VenerisMessageTypes::VenerisMessageTypes_FinishDynamicMeshGroup:
           finishDynamicMeshGroup();
           break;
    default:
        std::cout<<"VenerisServer::processMessage(): Unknown message type "<<type<<std::endl;

        EV_ERROR<<"VenerisServer::processMessage(): Unknown message type "<<type<<std::endl;
        endServer();
    }
}

} //namespace
