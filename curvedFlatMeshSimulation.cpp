/***************************************************************/
//
//Copyright (c) 2019 Esteban Egea-Lopez http://girtel.upct.es/~eegea
//
/**************************************************************/

#include "curvedFlatMeshSimulation.h"
#include "timer.h"
#include <algorithm>
namespace opal {
	LPCurvedFlatMeshReflectionSimulation::LPCurvedFlatMeshReflectionSimulation(OpalSceneManager* m) : LPCurvedMeshReflectionSimulation(m) {
		this->printHits=false;	
	}
	std::string LPCurvedFlatMeshReflectionSimulation::printConfigInfo() const  {
		std::ostringstream stream;
		stream<<"--- Simulation Type ---"<<std::endl;
		stream<<"\tUsing LPCurvedFlatMeshReflectionSimulation with depolarization"<<std::endl;
		if (useAngleDiscrimination) {
			stream<<"\tUsing angle discrimination" <<std::endl;
		}
		stream << "-----" << std::endl;
		return stream.str();
	}
	void LPCurvedFlatMeshReflectionSimulation::executeTransmitLaunch(uint numTransmitters,bool partial) {

		//Transmission launch
		Timer timer;
		timer.start();
		RaySphere raySphere= myManager->getRaySphere();
		checkLaunchSize(raySphere.elevationSteps, raySphere.azimuthSteps, numTransmitters);
		myManager->getContext()->launch(reflectionEntryIndex, raySphere.elevationSteps, raySphere.azimuthSteps,numTransmitters); //Launch 3D (elevation, azimuth, transmitters);
		timer.stop();
		const double launchTime=timer.getTime();
		std::vector<int> enabledDevices= myManager->getEnabledDevices();
		if (partial) {
			timer.restart();

			if (useAngleDiscrimination) {	
				//Separate hits on curved surfaces from the rest and transfer to partialstate the non-curved ones
				uint hits=opalthrustutils::getMixedHitsMultiGPU(globalHitInfoBuffer,  atomicIndexBuffer, enabledDevices, partialLaunchState,maxGlobalBufferSize);
				partialLaunchState->setIndex(hits);
				uint vhits=partialLaunchState->getDeviceHitsBufferSize();
				std::cout<<"unique non-curved hits="<<hits<<"vector size="<<vhits<<std::endl;
			} else {
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
			}

		} else {
			timer.restart();

			if (useAngleDiscrimination) {	
				uint curvedHits=0;
				std::cout<<"LPCurvedFlatMeshReflectionSimulation launch angle discrimination ended"<<std::endl;
				thrust::host_vector<HitInfo> hits=opalthrustutils::getMixedHitsMultiGPU(globalHitInfoBuffer,  atomicIndexBuffer, enabledDevices,maxGlobalBufferSize, curvedHits);
				//std::cout<<"Launch finished. total hits="<<hits.size()<<"\tcurved="<<curvedHits<<"\tnon-curved="<<(hits.size()-curvedHits)<<std::endl;
				std::vector<HitInfo> curved(hits.begin(),hits.begin()+curvedHits);
				std::vector<HitInfo> non_curved(hits.begin()+curvedHits,hits.end());	
				std::vector<HitInfo> filtered=filterByAngle(curved.data(),curved.size());
				std::vector<HitInfo> all(filtered.size()+non_curved.size());
				std::merge(filtered.begin(),filtered.end(),non_curved.begin(), non_curved.end(), all.begin());
				processLaunch(all.data(), all.size(), numTransmitters);

			} else {

				//Filter with thrust multiple hits coming from the same face. Directly returns the filtered vector
				std::cout<<"LPCurvedFlatMeshReflectionSimulation launch no angle discrimination ended"<<std::endl;
				thrust::host_vector<HitInfo> host_hits=opalthrustutils::filterHitsAndTransfer(globalHitInfoBuffer,  atomicIndexBuffer, enabledDevices, maxGlobalBufferSize );
				//Log times for performance tests
				timer.stop();
				const double filterTime=timer.getTime();
				uint numReceivers = myManager->getNumberOfReceivers();
				uint hits=host_hits.size();
				std::cout<<"#"<<numReceivers<<"\t"<<host_hits.size()<<"\t"<<launchTime<<"\t"<<filterTime<<std::endl;
				processLaunch(host_hits.data(), hits,numTransmitters);

			}

		}
	}
	void LPCurvedFlatMeshReflectionSimulation::endPartialLaunch(uint numTransmitters) {
		//Now we get all the hits on curved surfaces.
		std::vector<int> enabledDevices= myManager->getEnabledDevices();
		//The hits on non-curved have already been transferred to the partialState vector. If no angle discrimination is used, all the hits are already there
		std::vector<HitInfo> non_curved=partialLaunchState->getHits();
		//std::cout<<"non_curved="<<non_curved.size()<<std::endl;
		if (useAngleDiscrimination) {
			//Get all hits on curved and filter by Angle
			thrust::host_vector<HitInfo> curved=opalthrustutils::getAllHitsOrderedMultiGPU(globalHitInfoBuffer,  atomicIndexBuffer, enabledDevices, maxGlobalBufferSize);
			//std::cout<<"curved="<<curved.size()<<std::endl;
			std::vector<HitInfo> filtered=filterByAngle(curved.data(),curved.size());
			std::vector<HitInfo> all(filtered.size()+non_curved.size());
			std::merge(filtered.begin(),filtered.end(),non_curved.begin(), non_curved.end(), all.begin());
			std::cout<<"endPartial:\tc="<<curved.size()<<"\tf="<<filtered.size()<<"\tnc="<<non_curved.size()<<"\ta="<<all.size()<<std::endl;
			processLaunch(all.data(), all.size(), numTransmitters);
		} else {
			processLaunch(non_curved.data(), non_curved.size(), numTransmitters);
		}
		//partialLaunchState->reset();
		delete partialLaunchState;
		this->partialLaunchState = new opalthrustutils::PartialLaunchState<HitInfo>();


	}

}
