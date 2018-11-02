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

#include "OpalTestPacketGenerator.h"
#include "inet/linklayer/common/MacAddressTag_m.h"
#include "inet/linklayer/common/Ieee802SapTag_m.h"
#include "inet/linklayer/common/MacAddress.h"
#include "inet/common/packet/chunk/ByteCountChunk.h"
#include "inet/common/Ptr.h"

namespace inet {

Define_Module(OpalTestPacketGenerator);

simsignal_t OpalTestPacketGenerator::mobilityStateChangedSignal = cComponent::registerSignal("mobilityStateChanged");
OpalTestPacketGenerator::OpalTestPacketGenerator() :    generationTimer(nullptr), beaconFrequency(0),transmitWhenPositionChanges(false)
{
}
OpalTestPacketGenerator::~OpalTestPacketGenerator() {
    if (generationTimer!=nullptr) {
        cancelAndDelete(generationTimer);
    }
}
void OpalTestPacketGenerator::initialize()
{
    // TODO - Generated method body
    beaconFrequency= par("beaconFrequency");
    packetBytes= par("packetBytes");
    generationTimer = new cMessage("New packet");
    lowerLayerOutId=gate("lowerLayerOutput")->getBaseId();
    transmitWhenPositionChanges = par("transmitWhenPositionChanges");
    EV_INFO<<simTime()<<" beaconFrequency="<<beaconFrequency<<endl;
    if (transmitWhenPositionChanges) {
        getParentModule()->subscribe(mobilityStateChangedSignal,this);

    } else {
        if (beaconFrequency>0) {
            scheduleAt(simTime() + 1.0/beaconFrequency,generationTimer);
        }
    }
    id=getParentModule()->par("id");
}

void OpalTestPacketGenerator::handleMessage(cMessage *msg)
{
    if (msg->isSelfMessage()) {
        processGenerationTime();
    } else {
        EV_INFO<<id<<":Received CAM"<< endl;
        std::cout<<id<<":Received CAM"<< endl;
        delete msg;
    }
}
void OpalTestPacketGenerator::processGenerationTime() {
    if (beaconFrequency>0) {
        scheduleAt(simTime() + 1.0/beaconFrequency,generationTimer);
    }
    auto data = makeShared<ByteCountChunk>(B(packetBytes));
    Packet* cam = new Packet("cam", data);
    sendDown(cam);

}
void OpalTestPacketGenerator::sendDown(Packet* p) {
    //Add SAP
    p->addTagIfAbsent<Ieee802SapReq>()->setDsap(SapCode::SAP_IP);
    p->addTagIfAbsent<MacAddressReq>()->setDestAddress(MacAddress::BROADCAST_ADDRESS);

    send(p,lowerLayerOutId);
}

void OpalTestPacketGenerator::receiveSignal(cComponent* source,
        simsignal_t signal, cObject* value, cObject* details) {
    if (signal==mobilityStateChangedSignal) {
        if (beaconFrequency>0) {
        //MobilityBase* mob=check_and_cast<MobilityBase*>(value);
            Enter_Method_Silent();
            auto data = makeShared<ByteCountChunk>(B(packetBytes));
               Packet* cam = new Packet("cam", data);
               sendDown(cam);
        }
    }
}

} //namespace
