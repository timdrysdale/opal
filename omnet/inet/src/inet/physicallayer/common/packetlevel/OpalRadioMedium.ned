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

package inet.physicallayer.common.packetlevel;

//
// TODO auto-generated module
//
module OpalRadioMedium extends RadioMedium
{
    parameters:
    @class(OpalRadioMedium);
    double carrierFrequency @unit(Hz) = default(5.9GHz);
    int azimuthDelta = default(1);
    int elevationDelta = default(1);
    bool useDecimalDegreeDelta= default(false);
   	double receptionRadius @unit(m) = default(1m);
   	//bool useTransmitInGroup = default(false);
   	bool loadMeshesFromFile = default(false);
   	string meshesPath = default("");
   	string meshes = default("");
   	int maxNumberOfReflections = default(10);
   	analogModel.typename = default("ScalarAnalogModel");
    backgroundNoise.typename = default("IsotropicScalarBackgroundNoise");
    propagation.typename = default("ConstantSpeedPropagation");
    pathLoss.typename = default("FreeSpacePathLoss"); //Not actually using path loss, propagation is computed by Opal, but we need to provide some module
    backgroundNoise.power = default(-110dBm);
    mediumLimitCache.carrierFrequency = default(5.9GHz);
    mediumLimitCache.minReceptionPower = default(-85dBm);
    mediumLimitCache.minInterferencePower = default(-110dBm);
}
