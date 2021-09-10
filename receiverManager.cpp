/***************************************************************/
//
//Copyright (c) 2019 Esteban Egea-Lopez http://girtel.upct.es/~eegea
//
/**************************************************************/
#include "transmitterManager.h"
using namespace opal;
using namespace optix;

ReceiverManager::ReceiverManager(OpalSceneManager* m) : receivers(), receiverExtToBufferId() {
	std::cout<<"ReceiverManager() called"<<std::endl;
	this->myManager=m;	
}

uint  ReceiverManager::registerReceiver(SphereReceiver* rx) {
	//Add to map
	uint nextId = static_cast<uint>(receivers.size());
	receiverExtToBufferId.insert(std::pair<int, unsigned int>(id, nextId));
	receivers.push_back(rx);
	return nextId; 
}
Receiver* ReceiverManager::getReceiver(int externalId) {
}
void ReceiverManager::removeReceiver(int externalId) {
	unsigned int index=getReceiverIndex(externalId);
	
	delete getReceiver(extern;
	
	//Swap contents in the buffers if not last
	if (index != (receivers.size() - 1)) {
		receivers[index] = receivers[receivers.size() - 1];
		receiverExtToBufferId.at(receivers[index]->externalId) = index;
		receivers[index]->geomInstance["receiverBufferIndex"]->setUint(index);
		std::cout << "Swapping with " << (receivers.size() -1) << " with externalId= "<<receivers[index]->externalId<<std::endl;
		std::cout << "New index for " << receivers[index]->externalId<<" is "<<receiverExtToBufferId.at(receivers[index]->externalId)<<std::endl;
		std::cout << "Receiver buffer index for  " << receivers[index]->externalId<<" is "<<receivers[index]->geomInstance["receiverBufferIndex"]->getUint()<<std::endl;
	}
	receivers.pop_back();
	receiverExtToBufferId.erase(id);


}
void ReceiverManager::addReceiverToGroup(int txId, float transmitPower, optix::float3 origin, optix::float3 polarization) {
	//Find transmitter 
	Receiver* tb;
	try {
		tb = transmitterExtToBase.at(txId); //Throw exception if not found
	}
	catch (std::out_of_range e) {
		std::cout << "OpalSceneManager::addReceiverToGroup: Not found transmitter" << txId << std::endl;
		throw  opal::Exception("OpalSceneManager::addReceiverToGroup: Receiver is not registered with Opal");
		return;

	}
	tb->origin=origin;
	tb->polarization = polarization;
	tb->txPower=transmitPower;
	for (int i=0; i<activeReceivers.size(); i++) {
		if (activeReceivers[i]->externalId==txId) {
			return;
		}
	}
	activeReceivers.push_back(tb);
}
void ReceiverManager::addReceiverToGroup(int txId,float transmitPower, optix::float3 origin ) {
	//Find transmitter 
	Receiver* tb;
	try {
		tb = transmitterExtToBase.at(txId); //Throw exception if not found
	}
	catch (std::out_of_range e) {
		std::cout << "OpalSceneManager::addReceiverToBatch: Not found transmitter" << txId << std::endl;
		throw  opal::Exception("OpalSceneManager::addReceiverToBatch: Receiver is not registered with Opal");
		return;

	}
	tb->origin=origin;
	tb->txPower=transmitPower;
	for (int i=0; i<activeReceivers.size(); i++) {
		if (activeReceivers[i]->externalId==txId) {
			return;
		}
	}
	activeReceivers.push_back(tb);
}
void ReceiverManager::clearGroup() {
	activeReceivers.clear();

}

const std::vector<Receiver*>& ReceiverManager::getActiveTransmitters() {
	return activeReceivers; 
}

