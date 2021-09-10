/***************************************************************/
//
//Copyright (c) 2019 Esteban Egea-Lopez http://girtel.upct.es/~eegea
//
/**************************************************************/
#include "transmitterManager.h"
using namespace opal;
using namespace optix;

TransmitterManager::TransmitterManager(OpalSceneManager* m) : activeTransmitters(), transmitterExtToBase() {
	std::cout<<"TransmitterManager() called"<<std::endl;
	this->myManager=m;	
}

void TransmitterManager::registerTransmitter(int txId, optix::float3 origin, optix::float3 polarization, float transmitPower) {
	//Register transmitter in Opal. Add to map
	Transmitter* tb = new Transmitter();
	tb->origin_p=make_float4(origin,transmitPower);
	tb->polarization_k =make_float4(polarization,this->myManager->getChannelParameters().k);
	tb->externalId=txId;
	tb->gainId=RT_BUFFER_ID_NULL;
//	tb->txPower=transmitPower;
	transmitterExtToBase.insert(std::pair<int,Transmitter*> (txId,tb));

}
Transmitter* TransmitterManager::getTransmitter(int externalId) {
	Transmitter* tb;
	try {
		tb = transmitterExtToBase.at(externalId); //Throw exception if not found
	}
	catch (std::out_of_range e) {
		std::cout << "TransmitterManager()::Not found transmitter " << externalId << std::endl;
		throw  opal::Exception("TransmitterManager()::Transmitter is not registered with Opal");
		return nullptr;

	}
	return tb;
}
void TransmitterManager::removeTransmitter(int txId) {
	//Find transmitter and remove from Opal. Cannot transmit anymore
	Transmitter* tb= getTransmitter(txId);
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
void TransmitterManager::addTransmitterToGroup(int txId, float transmitPower, optix::float3 origin, optix::float3 polarization) {
	//Find transmitter 
	Transmitter* tb= getTransmitter(txId);
	tb->origin_p=make_float4(origin,transmitPower);
	tb->polarization_k =make_float4(polarization, this->myManager->getChannelParameters().k);
//	tb->txPower=transmitPower;
	for (int i=0; i<activeTransmitters.size(); i++) {
		if (activeTransmitters[i]->externalId==txId) {
			return;
		}
	}
	activeTransmitters.push_back(tb);
}
void TransmitterManager::addTransmitterToGroup(int txId,float transmitPower, optix::float3 origin ) {
	//Find transmitter 
	Transmitter* tb= getTransmitter(txId);
	tb->origin_p=make_float4(origin, transmitPower);
	//tb->txPower=transmitPower;
	for (int i=0; i<activeTransmitters.size(); i++) {
		if (activeTransmitters[i]->externalId==txId) {
			return;
		}
	}
	activeTransmitters.push_back(tb);
}
void TransmitterManager::clearGroup() {
	activeTransmitters.clear();

}

const std::vector<Transmitter*>& TransmitterManager::getActiveTransmitters() {
	return activeTransmitters; 
}
unsigned int TransmitterManager::getTransmitterIndex(int externalId) {
	for (unsigned int i=0; i<activeTransmitters.size();++i) {
		if (activeTransmitters[i]->externalId==externalId) {
			return i;
		}
	}
	std::cout<<"TransmitterManager::getTransmitterIndex(), Transmitter "<<externalId<<" is not an active transmitter with Opal";
	throw  opal::Exception("TransmitterManager::getTransmitterIndex(), Transmitter is not an active transmitter with Opal");
	return 0;
}
void TransmitterManager::registerTransmitterGain(int txId, int gainId) {
	Transmitter* t=getTransmitter(txId);
	optix::Buffer b = myManager->getAntennaGainBuffer(gainId);
	t->gainId=b->getId();
}

