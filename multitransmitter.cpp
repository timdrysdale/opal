/***************************************************************/
//
//Copyright (c) 2019 Esteban Egea-Lopez http://ait.upct.es/eegea
//
/**************************************************************/
#include "multitransmitter.h"
#include <iostream>
#include <sstream>
#include "sutil.h"

using namespace opal;
using namespace optix;
OpalSceneManagerMultiTransmitter::OpalSceneManagerMultiTransmitter(float f, bool useExactSpeedOfLight)
{
	initMembers();
	initContext(f,useExactSpeedOfLight);

}

void OpalSceneManagerMultiTransmitter::initMembers() {
	OpalSceneManager::initMembers();        
	this->txOriginBuffer = nullptr;
	this->activeTransmitters.clear();
	this->cudaProgramsDir="multitransmitter";

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

	//Initialize index for global buffer	
	uint* aib=reinterpret_cast<uint*>(atomicIndexBuffer->map());
	(*aib)=0u;
	atomicIndexBuffer->unmap();


	//Transmission launch
	std::cout<<"Transmitting. Number of transmitters= "<<numTransmitters<<std::endl;	
	Timer timer;
	timer.start();
	context->launch(0, raySphere.elevationSteps, raySphere.azimuthSteps,numTransmitters); //Launch 3D (elevation, azimuth, numTransmitters);
	timer.stop();
	const double launchTime=timer.getTime();
	transmissionLaunches++;

	//Get total number of hits (last global buffer index used)

	aib=reinterpret_cast<uint*>(atomicIndexBuffer->map());
	uint lastHitIndex= (*aib);
	atomicIndexBuffer->unmap();


	//std::cout<<"lastHitIndex="<<lastHitIndex<<std::endl;

	//Filter with thrust multiple hits coming from the same face
	timer.restart();
	uint hits=opalthrustutils::filterHitsWithCopyResize(globalHitInfoBuffer, resultHitInfoBuffer, lastHitIndex);
	timer.stop();
	const double filterTime=timer.getTime();
	timer.restart();
	HitInfo* host_hits=reinterpret_cast<HitInfo*>  (resultHitInfoBuffer->map());
	timer.stop();
	const double transferTime=timer.getTime();
	float2 E=make_float2(0.0f,0.0f);
	RTsize gsize;
	globalHitInfoBuffer->getSize(gsize);
	//Log times for performance tests
	std::cout<<"#"<<numReceivers<<"\t"<<numTransmitters<<"\t"<<gsize<<"\t"<<lastHitIndex<<"\t"<<hits<<"\t"<<launchTime<<"\t"<<filterTime<<"\t"<<transferTime<<std::endl;

	//Compute received power by adding EM waves of all hits. Global computation. Not done with thrust because reduce does not seem to allow a different type as output of the sum

	uint index=0u;
	uint raysHit=0u;
	uint currentTx=0u;

	for (uint i=0; i<hits; i++) {
		if (i==0) {
			//Get first transmitter
			currentTx=host_hits->thrd.x;
			//Get first receiver
			index=host_hits->thrd.z;
			//std::cout<<"First tx="<<currentTx<<";rx="<<index<<std::endl;
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
		//if (host_hits->whrd.x!=0) {

		++raysHit;
		//}
		E += host_hits->E;

		//	std::cout<<"E["<<i<<"]="<<(host_hits)->E<<std::endl;
		//	 std::cout<<"\t rxBufferIndex="<<(host_hits)->thrd.z<<std::endl;
		//	 std::cout<<"\t txBufferIndex="<<(host_hits)->thrd.x<<std::endl;
		//	 std::cout<<"\t refhash="<<(host_hits)->thrd.y<<std::endl;
		//	 std::cout<<"\t dist="<<(host_hits)->thrd.w<<std::endl;
		++host_hits;

	}
	//Last one
	if (raysHit!=0u) {
		computeReceivedPower(E,index,activeTransmitters[currentTx]->externalId,activeTransmitters[currentTx]->transmitPower,activeTransmitters[currentTx]->origin);

	}	
	//timer.stop();
	resultHitInfoBuffer->unmap();

	clearGroup();

}

optix::Buffer OpalSceneManagerMultiTransmitter::setTransmitterBuffer(optix::uint tx) {
	uint tra=1u;
	if (tx>0) {
		tra=tx;
	}
	optix::Buffer b = context->createBuffer(RT_BUFFER_INPUT, RT_FORMAT_USER, tra);
	b->setElementSize(sizeof(Transmitter));
	context["tx_origin"]->set(b);
	return b;
}
std::string opal::OpalSceneManagerMultiTransmitter::printInternalBuffersState()
{

	std::ostringstream stream;
	unsigned long long totalBytes = 0;
	unsigned long long sb;
	stream << "--Internal buffers--" << std::endl;
	RTsize w;
	RTsize h;
	RTsize d;
	//std::cout<<"Available device memory: "<<context->getAvailableDeviceMemory(0)<<std::endl;
	std::cout<<"Receivers="<<currentInternalBuffersState.rx<<std::endl;
	if (globalHitInfoBuffer) {
		globalHitInfoBuffer->getSize(w);
		sb = sizeof(HitInfo)*w;
		totalBytes += sb;
		stream << "\t globalHitInfoBuffers=(" << w <<"). size=" << (sb / 1024.f) << " KiB" << std::endl;
	}
	if (resultHitInfoBuffer) {
		resultHitInfoBuffer->getSize(w);
		sb = sizeof(HitInfo)*w;
		totalBytes += sb;
		stream << "\t resulHitInfoBuffers=(" << w <<"). size=" << (sb / 1024.f) << " KiB" << std::endl;
	}
	if (txOriginBuffer) {
		txOriginBuffer->getSize(w);
		sb = sizeof(Transmitter)*w;
		totalBytes += sb;
		stream << "\t txOriginBuffer=(" << w <<"). size=" << (sb / 1024.f) << " KiB" << std::endl;
	}
	//Check memory usage
	stream << "Total memory in internal buffers:  " << (totalBytes / (1024.f*1024.f)) << " MiB" << std::endl;
	return stream.str();

}
void OpalSceneManagerMultiTransmitter::setInternalBuffers() {
	OpalSceneManager::setInternalBuffers();
	optix::uint txSize=static_cast<uint>(activeTransmitters.size());
	//Store current state
	currentInternalBuffersState.tx=txSize;
	txOriginBuffer = setTransmitterBuffer(txSize);
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
optix::Program OpalSceneManagerMultiTransmitter::createClosestHitReceiver()
{

	optix::Program chrx;
	std::string ptx=sutil::getPtxString(cudaProgramsDir.c_str(), "multiTransmitter.cu");
	chrx = context->createProgramFromPTXString(ptx, "closestHitReceiver");

	//Add programs for complex arithmetic
	chrx["complex_exp_only_imaginary"]->setProgramId(defaultPrograms.at("complex_exp_only_imaginary"));
	chrx["sca_complex_prod"]->setProgramId(defaultPrograms.at("sca_complex_prod"));
	chrx["complex_prod"]->setProgramId(defaultPrograms.at("complex_prod"));

	//Program variables: common value for all receiver instances, since they all share the program. 
	chrx["k"]->setFloat(defaultChannel.k); //If multichannel is used, this should be set per transmission

	return chrx;
}
optix::Program OpalSceneManagerMultiTransmitter::createClosestHitInternalRay()
{



	std::cout<<"Creating closestHitReceiveInternal" <<std::endl;
	optix::Program chrx = context->createProgramFromPTXString(sutil::getPtxString(cudaProgramsDir.c_str(), "multiTransmitter.cu"), "closestHitReceiverInternalRay");

	//Add programs for complex arithmetic
	chrx["complex_exp_only_imaginary"]->setProgramId(defaultPrograms.at("complex_exp_only_imaginary"));
	chrx["sca_complex_prod"]->setProgramId(defaultPrograms.at("sca_complex_prod"));
	chrx["complex_prod"]->setProgramId(defaultPrograms.at("complex_prod"));

	//Program variable: common value for all receiver instances, since they all share the program. If multichannel is used, this should be set per transmission
	chrx["k"]->setFloat(defaultChannel.k);
	return chrx;
}
optix::Program  OpalSceneManagerMultiTransmitter::createRayGenerationProgram()
{


	return context->createProgramFromPTXString(sutil::getPtxString(cudaProgramsDir.c_str(), "generationMultiTransmitter.cu"), "genRayAndReflectionsFromSphereIndex");

}
optix::Program OpalSceneManagerMultiTransmitter::createMissProgram() 
{
	return context->createProgramFromPTXString(sutil::getPtxString(cudaProgramsDir.c_str(), "multiTransmitter.cu"), "miss");

}
