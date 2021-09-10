/***************************************************************/
//
//Copyright (c) 2019 Esteban Egea-Lopez http://girtel.upct.es/~eegea
//
/**************************************************************/

#include "basicSimulation.h"
#include "timer.h"
#include <algorithm>
#include <iomanip>

namespace opal {
	BasicFlatMeshReflectionSimulation::BasicFlatMeshReflectionSimulation(OpalSceneManager* m) : OpalSimulation(m) {
		//#ifdef OPAL_EXTENDED_HITINFO
			if (mode!=ComputeMode::VOLTAGE) {
				throw opal::Exception("BasicFlatMeshReflectionSimulation(): for  Basic simulation only induced voltage on the antenna can be computed");
			}
		//#endif
		this->partialLaunchState = new opalthrustutils::PartialLaunchState<HitInfo>();
		this->optixProgramsDir ="optix";
		this->cudaProgramsDir=(m->getBaseDirectory()+"/"+optixProgramsDir);
		this->acceptCurved = false;
		this->withPolarization = false;
		this->simType=OpalSimulationTypes::BASIC;
	}
	BasicFlatMeshReflectionSimulation::~BasicFlatMeshReflectionSimulation() {
		if (partialLaunchState) {	
			delete this->partialLaunchState;
		}
		partialLaunchState=nullptr;
	}
	void BasicFlatMeshReflectionSimulation::setOtherDirectory() {
			this->currentDir=(cudaProgramsDir+"/basic");
	}
	std::string BasicFlatMeshReflectionSimulation::printConfigInfo() const  {
		std::ostringstream stream;
		stream<<"--- Simulation Type---"<<std::endl;
		stream<<"\tUsing BasicFlatMeshReflectionSimulation (No depolarization)"<<std::endl;
		stream<<"\t - You are assuming that: (1)  both transmitters and receivers have the same polarization and (2) the polarization is either purely vertical (0, 1, 0) or horizontal (1,0,0), but not (0,0,1). " <<std::endl; 
		stream<<"\t   If this is not the intended behaviour, check the use of depolarization, though it has a performance cost. " <<std::endl; 
		return stream.str();
	}
	void BasicFlatMeshReflectionSimulation::executeTransmitLaunch(uint numTransmitters,bool partial) {
		//Transmission launch
		Timer timer;
		timer.start();
		//	std::cout<<"BasicFlatMeshReflectionSimulation::executeTransmitLaunch(): getRaySphere "<<numTransmitters<<" transmitters"<<std::endl;
		RaySphere raySphere= myManager->getRaySphere();
		checkLaunchSize(raySphere.elevationSteps, raySphere.azimuthSteps, numTransmitters);

		//	std::cout<<"BasicFlatMeshReflectionSimulation::executeTransmitLaunch(): launching with "<<numTransmitters<<" transmitters"<<std::endl;
		myManager->getContext()->launch(reflectionEntryIndex, raySphere.elevationSteps, raySphere.azimuthSteps,numTransmitters); //Launch 3D (elevation, azimuth, transmitters);
		timer.stop();
		const double launchTime=timer.getTime();
		std::vector<int> enabledDevices= myManager->getEnabledDevices();
		if (partial) {
			timer.restart();

			//Filter with thrust multiple hits coming from the same face. Do not transfer the filtered vector
			uint hits=opalthrustutils::filterHitsMultiGPU(globalHitInfoBuffer,  atomicIndexBuffer, enabledDevices, partialLaunchState,maxGlobalBufferSize);
			partialLaunchState->setIndex(hits);
			uint vhits=partialLaunchState->getDeviceHitsBufferSize();
			std::cout<<"hits="<<hits<<"vector size="<<vhits<<std::endl;
			//Log times for performance tests
			timer.stop();
			uint numReceivers = myManager->getNumberOfReceivers();
			const double filterTime=timer.getTime();
			std::cout<<"#"<<numReceivers<<"\t"<<hits<<"\t"<<launchTime<<"\t"<<filterTime<<std::endl;

		} else {
			timer.restart();
			//std::cout<<"Finished launch. Filtering hits..."<<std::endl;
			//Filter with thrust multiple hits comin from the same face. Directly returns the filtered vector
			//thrust::host_vector<HitInfo> host_hits=opalthrustutils::filterHitsAndTransferMultiGPU(globalHitInfoBuffer,  atomicIndexBuffer, enabledDevices, maxGlobalBufferSize );
			thrust::host_vector<HitInfo> host_hits=opalthrustutils::filterHitsAndTransfer(globalHitInfoBuffer,  atomicIndexBuffer, enabledDevices, maxGlobalBufferSize );

			//Log times for performance tests
			timer.stop();
			const double filterTime=timer.getTime();
			uint numReceivers = myManager->getNumberOfReceivers();
			uint hits=host_hits.size();
			std::cout<<"#"<<numReceivers<<"\t"<<hits<<"\t"<<launchTime<<"\t"<<filterTime<<std::endl;
			processLaunch(host_hits.data(), hits,numTransmitters);

		}

	}
	void BasicFlatMeshReflectionSimulation::endPartialLaunch(uint numTransmitters) {
		std::vector<HitInfo> host_hits = partialLaunchState->getHits();
		uint hits=host_hits.size();
		processLaunch(host_hits.data(), hits, numTransmitters);
		//partialLaunchState->reset();
		//To make sure the memory in the device is freed...
		delete partialLaunchState;
		this->partialLaunchState = new opalthrustutils::PartialLaunchState<HitInfo>();
	}
	void BasicFlatMeshReflectionSimulation::updateReceiver(int id, float3 position, float3 polarization, float radius) {
		if ((polarization.x==1.0) && (polarization.y==0.0) &&(polarization.z==0.0)) {
			OpalSimulation::updateReceiver(id,position,polarization,radius);
			return;
		} else  if ((polarization.x==0.0) && (polarization.y==1.0) &&(polarization.z==0.0)) {
			OpalSimulation::updateReceiver(id,position,polarization,radius);
			return;
		} else {
			throw opal::Exception("BasicFlatMeshReflectionSimulation::updateReceiver() for  Basic simulation only (1,0,0) and (0,1,0) polarizations are valid");
		}
	}
	void BasicFlatMeshReflectionSimulation::addReceiver(int id, float3  position, float3 polarization, float radius, std::function<void(float, int)>  callback, std::vector<optix::Material>& materials) {
		if ((polarization.x==1.0) && (polarization.y==0.0) &&(polarization.z==0.0)) {
			OpalSimulation::addReceiver(id,  position,polarization,radius,callback,materials);
			return;
		} else  if ((polarization.x==0.0) && (polarization.y==1.0) &&(polarization.z==0.0)) {
			OpalSimulation::addReceiver(id,  position,polarization,radius,callback,materials);
			return;
		} else {
			throw opal::Exception("BasicFlatMeshReflectionSimulation::addReceiver() for  Basic simulation only (1,0,0) and (0,1,0) polarizations are valid");
		}
	}

}
