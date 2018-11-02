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

#ifndef __INET_OPALTESTPACKETGENERATOR_H_
#define __INET_OPALTESTPACKETGENERATOR_H_

#include <omnetpp.h>
#include "inet/common/packet/Packet.h"
using namespace omnetpp;

namespace inet {

/**
 * TODO - Generated class
 */
class OpalTestPacketGenerator : public cSimpleModule, cListener
{
  protected:
    //Subscribe to position changes for the receivers and transmitters
        static simsignal_t mobilityStateChangedSignal;
    int id;
    cMessage* generationTimer;
    double beaconFrequency;
    bool transmitWhenPositionChanges;
    virtual void initialize();
    virtual void handleMessage(cMessage *msg);
    void processGenerationTime();
    void sendDown(Packet* o);
    int lowerLayerOutId;
    int packetBytes;
  public:
    OpalTestPacketGenerator();
    virtual ~OpalTestPacketGenerator();
    virtual void receiveSignal(cComponent *source, simsignal_t signal, cObject *value, cObject *details) override;
};

} //namespace

#endif
