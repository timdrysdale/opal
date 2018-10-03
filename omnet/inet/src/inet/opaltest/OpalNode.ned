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

package inet.opaltest;


import inet.linklayer.contract.IWirelessInterface;
import inet.mobility.contract.IMobility;
import inet.networklayer.common.InterfaceTable;

module OpalNode
{
    parameters:
        @networkNode;
        wlan.mgmt.typename = default("Ieee80211MgmtAdhoc");
        wlan.agent.typename = default("");
        wlan.opMode = "p";
        mobility.typename = default("StationaryMobility");
        wlan.**.mobilityModule = default(exists(mobility) ? absPath(".mobility") : "");
        //wlan.interfaceTableModule = default("InterfaceTable");
        
        *.interfaceTableModule = default(absPath(".interfaceTable"));
       // wlan.radio.antenna.mobilityModule = default("StationaryMobility");
      

    gates:
        input radioIn @directIn;
    submodules:

        interfaceTable: InterfaceTable {
            parameters:
              @display("p=125,240;is=s");
        }
 		mobility: <default("")> like IMobility if typename != "" {
            parameters:
                @display("p=125,160;is=s");
        }
        generator: OpalTestPacketGenerator {
            parameters:
                @display("p=366,422;is=s");
        }
        // TODO: move wlan interfaces after eth interfaces, but it changes IP address assignment and breaks examples/inet/configurator/complex.ini
        wlan: <default("Ieee80211Interface")> like IWirelessInterface {
            parameters:
                @display("p=375,616,row,150;q=queue");
        }


    connections allowunconnected:

        generator.lowerLayerOutput --> { @display("m=s"); } --> wlan.upperLayerIn;
        wlan.upperLayerOut --> { @display("m=s"); } --> generator.lowerLayerIn;
        radioIn --> { @display("m=s"); } --> wlan.radioIn;




}
