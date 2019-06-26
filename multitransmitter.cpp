/***************************************************************/
//
//Copyright (c) 2019 Esteban Egea-Lopez http://ait.upct.es/eegea
//
/**************************************************************/
#include "multitransmitter.h"
#include "timer.h"
#include <iostream>
#include <sstream>
#include "sutil.h"

using namespace opal;
using namespace optix;
OpalSceneManagerMultiTransmitter::OpalSceneManagerMultiTransmitter() {
	std::cout<<"OpalSceneManagerMultiTransmitter() called"<<std::endl;
	initMembers();
	
}
OpalSceneManagerMultiTransmitter::OpalSceneManagerMultiTransmitter(float f, bool useExactSpeedOfLight)
{
	initMembers();
	initContext(f,useExactSpeedOfLight);

}

void OpalSceneManagerMultiTransmitter::initMembers() {
	OpalSceneManager::initMembers();        
	this->activeTransmitters.clear();

}
void OpalSceneManagerMultiTransmitter::registerTransmitter(int txId, optix::float3 origin, optix::float3 polarization, float transmitPower) {
	//Register transmitter in Opal. Add to map
	BaseTransmitter* tb = new BaseTransmitter();
	tb->origin=origin;
	tb->polarization = polarization;
	tb->externalId=txId;
	tb->transmitPower=transmitPower;
	transmitterExtToBase.insert(std::pair<int,BaseTransmitter*> (txId,tb));

}

void OpalSceneManagerMultiTransmitter::removeTransmitter(int txId) {
	//Find transmitter and remove from Opal. Cannot transmit anymore
	BaseTransmitter* tb;
	try {
		tb = transmitterExtToBase.at(txId); //Throw exception if not found
	}
	catch (std::out_of_range e) {
		std::cout << "Not found transmitter " << txId << std::endl;
		throw  opal::Exception("Transmitter is not registered with Opal");
		return;

	}
	std::cout << "Opal:: Removing transmitter " << txId << std::endl;
	for (unsigned int i=0; i<activeTransmitters.size(); ++i) {
		if (activeTransmitters[i]->externalId==txId) {
			activeTransmitters.erase(activeTransmitters.begin()+i);
			break;
		}
	}
	delete tb;
	transmitterExtToBase.erase(txId);
}
void OpalSceneManagerMultiTransmitter::addTransmitterToGroup(int txId, float transmitPower, optix::float3 origin, optix::float3 polarization) {
	//Find transmitter 
	BaseTransmitter* tb;
	try {
		tb = transmitterExtToBase.at(txId); //Throw exception if not found
	}
	catch (std::out_of_range e) {
		std::cout << "OpalSceneManager::addTransmitterToGroup: Not found transmitter" << txId << std::endl;
		throw  opal::Exception("OpalSceneManager::addTransmitterToGroup: Transmitter is not registered with Opal");
		return;

	}
	tb->origin=origin;
	tb->polarization = polarization;
	tb->transmitPower=transmitPower;
	for (int i=0; i<activeTransmitters.size(); i++) {
		if (activeTransmitters[i]->externalId==txId) {
			return;
		}
	}
	activeTransmitters.push_back(tb);
}
void OpalSceneManagerMultiTransmitter::addTransmitterToGroup(int txId,float transmitPower, optix::float3 origin ) {
	//Find transmitter 
	BaseTransmitter* tb;
	try {
		tb = transmitterExtToBase.at(txId); //Throw exception if not found
	}
	catch (std::out_of_range e) {
		std::cout << "OpalSceneManager::addTransmitterToBatch: Not found transmitter" << txId << std::endl;
		throw  opal::Exception("OpalSceneManager::addTransmitterToBatch: Transmitter is not registered with Opal");
		return;

	}
	tb->origin=origin;
	tb->transmitPower=transmitPower;
	for (int i=0; i<activeTransmitters.size(); i++) {
		if (activeTransmitters[i]->externalId==txId) {
			return;
		}
	}
	activeTransmitters.push_back(tb);
}
void OpalSceneManagerMultiTransmitter::clearGroup() {
	activeTransmitters.clear();

}
void OpalSceneManagerMultiTransmitter::groupTransmit() {

	uint numReceivers = static_cast<uint>(receivers.size());
	if (numReceivers == 0) {
		//For debug or testing reflections or any other thing, at least one receiver should be added
		return;

	}

	uint numTransmitters=static_cast<uint>(activeTransmitters.size());
	if (numTransmitters==0) {
		return;
	}
	if (numReceivers == 1 && numTransmitters==1) {
		if (receivers[0]->externalId == activeTransmitters[0]->externalId) {
			//No power will be received, since we are not assuming   dual transceiver
			return;
		}
	}


	checkInternalBuffers();	

	Transmitter* host_tx = reinterpret_cast<Transmitter*>  (txOriginBuffer->map());
	Transmitter trI;
	for (uint i=0; i<numTransmitters; ++i) {
		trI.origin = activeTransmitters[i]->origin;
		trI.polarization =activeTransmitters[i]-> polarization;
		trI.externalId =activeTransmitters[i]-> externalId;
		host_tx[i]=trI;	
	}

	txOriginBuffer->unmap();





	//Adaptive radius just to avoid a transmitter being inside a receiver sphere
	bool applyDirty=false;
	for (int i=0; i<numReceivers; i++) {
		for (int j=0; j<numTransmitters; j++) {
			if (receivers[i]->externalId!=activeTransmitters[j]->externalId) {
				float distance=length(receivers[i]->position-activeTransmitters[j]->origin);
				if (distance<=receivers[i]->radius+0.001f) {
					receivers[i]->geomInstance["sphere"]->setFloat(make_float4(receivers[i]->position.x, receivers[i]->position.y, receivers[i]->position.z,distance*radioReductionFraction ));
					receivers[i]->dirty=true;
					applyDirty=true;
				} else if (receivers[i]->dirty) {
					receivers[i]->geomInstance["sphere"]->setFloat(make_float4(receivers[i]->position.x, receivers[i]->position.y, receivers[i]->position.z,receivers[i]->radius ));
					receivers[i]->dirty=false;
					applyDirty=true;
				}
			}
		}
	}
	if (applyDirty)  {
		receiversGroup->getAcceleration()->markDirty();
		rootGroup->getAcceleration()->markDirty();
	}
	
	std::cout<<"Transmitting. Number of transmitters= "<<numTransmitters<<std::endl;	
	if (useMultiGPU) {
		executeTransmitLaunchMultiGPU(numTransmitters);
	} else {
		executeTransmitLaunch(numTransmitters);
	}


	clearGroup();

}
void OpalSceneManagerMultiTransmitter::executeTransmitLaunch(uint numTransmitters) {
	//Initialize index for global buffer	
	uint* aib=reinterpret_cast<uint*>(atomicIndexBuffer->map());
	(*aib)=0u;
	atomicIndexBuffer->unmap();


	//Transmission launch
	//std::cout<<"Transmitting["<<txId<<"]["<<txPower<<"]"<<origin<<std::endl;	
	Timer timer;
	timer.start();
	context->launch(0, raySphere.elevationSteps, raySphere.azimuthSteps,numTransmitters); //Launch 3D (elevation, azimuth, transmitters);
	timer.stop();
	const double launchTime=timer.getTime();
	transmissionLaunches++;

	//Get total number of hits (last global buffer index used)

	aib=reinterpret_cast<uint*>(atomicIndexBuffer->map());
	uint lastHitIndex= (*aib);
	atomicIndexBuffer->unmap();


	//std::cout<<"lastHitIndex="<<lastHitIndex<<std::endl;

	timer.restart();
	//Filter with thrust multiple hits coming from the same face
	uint hits=opalthrustutils::filterHitsWithCopyResize(globalHitInfoBuffer, resultHitInfoBuffer, lastHitIndex);
	
	//Performance test
	timer.stop();
	const double filterTime=timer.getTime();
	timer.restart();

	//Transfer the filtered hits to the host
	HitInfo* host_hits=reinterpret_cast<HitInfo*>  (resultHitInfoBuffer->map());
	
	//Just for performance
	timer.stop();
	const double transferTime=timer.getTime();
	float2 E=make_float2(0.0f,0.0f);
	RTsize gsize;
	globalHitInfoBuffer->getSize(gsize);
	//Log times for performance tests
	uint numReceivers = static_cast<uint>(receivers.size());
	std::cout<<"#"<<numReceivers<<"\t"<<numTransmitters<<"\t"<<gsize<<"\t"<<lastHitIndex<<"\t"<<hits<<"\t"<<launchTime<<"\t"<<filterTime<<"\t"<<transferTime<<std::endl;

	//Compute received power by adding EM waves of all hits. Global computation. Not done with thrust because thrust::reduce does not seem to allow a different type as output of the sum

	uint index=0u;
	uint raysHit=0u;

	uint currentTx=0u;
	for (uint i=0; i<hits; i++) {
		if (i==0) {
			//Get first transmitter 			
			currentTx=host_hits->thrd.x;
			//Get first receiver
			index=host_hits->thrd.z;

		} else {

			if (host_hits->thrd.x!=currentTx) {
				if (raysHit!=0) {
					computeReceivedPower(E,index,activeTransmitters[currentTx]->externalId,activeTransmitters[currentTx]->transmitPower,activeTransmitters[currentTx]->origin); 			
				} 				
				//New transmitter 				
				currentTx=host_hits->thrd.x; 				
				//New receiver,  start new accumulation 				
				index=host_hits->thrd.z; 				
				E=make_float2(0.0f,0.0f); 				
				raysHit=0u; 				
				//std::cout<<"New transmitter tx="<<currentTx<<";rx="<<index<<std::endl; 			
			} else { 				
				if (host_hits->thrd.z!=index) { 					
					if (raysHit!=0u) { 						
						//At least one hit, callback 						
						computeReceivedPower(E,index,activeTransmitters[currentTx]->externalId,activeTransmitters[currentTx]->transmitPower,activeTransmitters[currentTx]->origin);
					} 			
					//New receiver, same transmitter, start new accumulation
					//std::cout<<"New receiver tx="<<currentTx<<";rx="<<index<<std::endl; 					
					index=host_hits->thrd.z;
					E=make_float2(0.0f,0.0f);
					raysHit=0u;
				} 			
			}


		}

		++raysHit;
		E += host_hits->E;

	// Log hits received
	//	std::cout<<"E["<<i<<"]="<<(host_hits)->E<<std::endl;
	//	std::cout<<"\t rxBufferIndex="<<(host_hits)->thrd.z<<std::endl;
	//	std::cout<<"\t written="<<(host_hits)->thrd.x<<std::endl;
	//	std::cout<<"\t refhash="<<(host_hits)->thrd.y<<std::endl;
	//	std::cout<<"\t dist="<<(host_hits)->thrd.w<<std::endl;


		++host_hits;

	}
	//Last one
	if (raysHit!=0u) {
		computeReceivedPower(E,index,activeTransmitters[currentTx]->externalId,activeTransmitters[currentTx]->transmitPower,activeTransmitters[currentTx]->origin); 
	}	
	//timer.stop();
	resultHitInfoBuffer->unmap();
}
void OpalSceneManagerMultiTransmitter::executeTransmitLaunchMultiGPU(uint numTransmitters) {
	//Transmission launch
	Timer timer;
	timer.start();
	context->launch(0, raySphere.elevationSteps, raySphere.azimuthSteps,numTransmitters); //Launch 3D (elevation, azimuth, transmitters);
	timer.stop();
	const double launchTime=timer.getTime();
	transmissionLaunches++;


	timer.restart();
	//Filter with thrust multiple hits coming from the same face. Directly returns the filtered vector
	thrust::host_vector<HitInfo> host_hits=opalthrustutils::filterHitsMultiGPU(globalHitInfoBuffer,  atomicIndexBuffer, enabledDevices );
	
	//Log times for performance tests
	timer.stop();
	const double filterTime=timer.getTime();
	uint numReceivers = static_cast<uint>(receivers.size());
	std::cout<<"#"<<numReceivers<<"\t"<<numTransmitters<<"\t"<<host_hits.size()<<"\t"<<launchTime<<"\t"<<filterTime<<std::endl;

	//Compute received power by adding EM waves of all hits. Global computation. Not done with thrust because thrust::reduce does not seem to allow a different type as output of the sum

	float2 E=make_float2(0.0f,0.0f);
	uint index=0u;
	uint raysHit=0u;
	uint currentTx=0u;
	for (uint i=0; i<host_hits.size(); i++) {
		if (i==0) {
			//Get first transmitter 			
			currentTx=host_hits[i].thrd.x;
			//Get first receiver
			index=host_hits[i].thrd.z;

		} else {

			if (host_hits[i].thrd.x!=currentTx) {
				if (raysHit!=0) {
					computeReceivedPower(E,index,activeTransmitters[currentTx]->externalId,activeTransmitters[currentTx]->transmitPower,activeTransmitters[currentTx]->origin); 			
				} 				
				//New transmitter 				
				currentTx=host_hits[i].thrd.x; 				
				//New receiver,  start new accumulation 				
				index=host_hits[i].thrd.z; 				
				E=make_float2(0.0f,0.0f); 				
				raysHit=0u; 				
				//std::cout<<"New transmitter tx="<<currentTx<<";rx="<<index<<std::endl; 			
			} else { 				
				if (host_hits[i].thrd.z!=index) { 					
					if (raysHit!=0u) { 						
						//At least one hit, callback 						
						computeReceivedPower(E,index,activeTransmitters[currentTx]->externalId,activeTransmitters[currentTx]->transmitPower,activeTransmitters[currentTx]->origin);
					} 			
					//New receiver, same transmitter, start new accumulation
					//std::cout<<"New receiver tx="<<currentTx<<";rx="<<index<<std::endl; 					
					index=host_hits[i].thrd.z;
					E=make_float2(0.0f,0.0f);
					raysHit=0u;
				} 			
			}


		}

		++raysHit;
		E += host_hits[i].E;

		std::cout<<"E["<<i<<"]="<<host_hits[i].E<<std::endl;
		std::cout<<"\t rxBufferIndex="<<host_hits[i].thrd.z<<std::endl;
		std::cout<<"\t written="<<host_hits[i].thrd.x<<std::endl;
		std::cout<<"\t refhash="<<host_hits[i].thrd.y<<std::endl;
		std::cout<<"\t dist="<<host_hits[i].thrd.w<<std::endl;




	}
	//Last one
	if (raysHit!=0u) {
		computeReceivedPower(E,index,activeTransmitters[currentTx]->externalId,activeTransmitters[currentTx]->transmitPower,activeTransmitters[currentTx]->origin); 
	}	
	//timer.stop();

}
void OpalSceneManagerMultiTransmitter::checkInternalBuffers() {
	OpalSceneManager::checkInternalBuffers();
	uint numTransmitters=static_cast<uint>(activeTransmitters.size());
	if (currentInternalBuffersState.tx==numTransmitters) {
		return;
	} else {
		std::cout<<"Resizing transmitter buffer to "<<numTransmitters<<std::endl;
		resizeTransmitterBuffer(numTransmitters);
	}

	//Store current state
	currentInternalBuffersState.tx=numTransmitters;

}
void OpalSceneManagerMultiTransmitter::resizeTransmitterBuffer(uint tx) {
	uint tra=1u;
	if (tx>0) {
		tra=tx;
	}
	txOriginBuffer->setSize(tra);
}
