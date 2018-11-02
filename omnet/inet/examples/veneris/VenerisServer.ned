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

package inet.examples.veneris;
import inet.common.veneris.VenerisServer;
import inet.physicallayer.common.packetlevel.RadioMedium;
import inet.physicallayer.contract.packetlevel.IRadioMedium;
import inet.physicallayer.ieee80211.packetlevel.Ieee80211ScalarRadioMedium;
network VenerisServer {
   	@statistic[receptionPower](source="receptionMinSignalPower(signalArrivalStarted)"; record=last,vector);
   // @statistic[position](source="transmitterPosition(nodes.signalArrivalStarted)"; record=last,vector);
    submodules:
        server:VenerisServer;
        //Create a default radioMedium in case we do not use OpalRadioMedium
        radioMedium: <default("Ieee80211ScalarRadioMedium")> like IRadioMedium {
            @display("p=49.704147,85.20711");
        }
}