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

#ifndef __INET_VENERISSERVER_H_
#define __INET_VENERISSERVER_H_

#include <omnetpp.h>
#include "ExecutionServer.h"
#include "inet/physicallayer/common/packetlevel/OpalRadioMedium.h"
#include "VenerisExternalMobility.h"
using namespace omnetpp;

namespace inet {

/**
 * TODO - Generated class
 */
class VenerisServer : public ExecutionServer
{

  protected:
    struct VehicleInfo {
        cModule* module;
        VenerisExternalMobility* mob;
    };

    bool useOpal;
    std::string opalModule;
    std::string vehicleModule;
    std::string mobilityModule;

    std::map<int,VehicleInfo> activeVehicles;
    inet::physicallayer::OpalRadioMedium* opalRadio;
    virtual int numInitStages() const override {return NUM_INIT_STAGES;}
    virtual void initialize(int stage) override;

    //Server message handlers
    virtual void createOpal();
    virtual void addStaticMesh();
    virtual void finishOpalContext();
    virtual void createVehicle();
    virtual void destroyVehicle();
    virtual void updateVehicleState();
    virtual void addDynamicMeshGroup();
    virtual void addDynamicMesh();
    virtual void finishDynamicMeshGroup();
    virtual void removeDynamicMeshGroup();
    virtual void updateTransformInGroup();
  public:
    VenerisServer();
    virtual ~VenerisServer();
    virtual void processMessage( Veneris::Communications::VenerisMessageTypes type, uint32_t size) override;
};

} //namespace

#endif
