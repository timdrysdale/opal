/***************************************************************/
//
//Copyright (c) 2019 Esteban Egea-Lopez http://girtel.upct.es/~eegea
//
/**************************************************************/

#include "flatSimulation.h"
namespace opal {
	LPFlatMeshReflectionSimulation::LPFlatMeshReflectionSimulation(OpalSceneManager* m) : BasicFlatMeshReflectionSimulation(m) {
		this->withPolarization=true;
		this->simType =OpalSimulationTypes::FLATWALLS;
	}
	std::string LPFlatMeshReflectionSimulation::printConfigInfo() const  {
		std::ostringstream stream;
		stream<<"--- Simulation Type---"<<std::endl;
		stream<<"\tUsing LPFlatMeshReflectionSimulation"<<std::endl;
		stream<< "\t - Using depolarization. Transmitter and receiver can have any linear polarizations (linear antenna may be oriented in any direction) and can be different. "<<std::endl;
		return stream.str();
	}
	void LPFlatMeshReflectionSimulation::setOtherDirectory()  {
		this->currentDir=(cudaProgramsDir+"/polarization");
	}
	void LPFlatMeshReflectionSimulation::addReceiver(int id, float3  position, float3 polarization, float radius, std::function<void(float, int)>  callback, std::vector<optix::Material>& materials) {
		OpalSimulation::addReceiver(id,  position,polarization,radius,callback,materials);
	}
	void LPFlatMeshReflectionSimulation::updateReceiver(int id, float3 position, float3 polarization, float radius) {
		OpalSimulation::updateReceiver(id,position,polarization,radius);
	}

}
