/***************************************************************/
//
//Copyright (c) 2019 Esteban Egea-Lopez http://girtel.upct.es/~eegea
//
/**************************************************************/

#ifndef RECEIVERMANAGER_H
#define RECEIVERMANAGER_H

#include "Opal.h"

//Manager for  multi-transmitter and multi-receiver operation. Supports simultaneous parallel transmission of multiple nodes
namespace opal {
	class OpalSceneManager;  //Forward declaration

	class ReceiverManager {
		protected:
			OpalSceneManager* myManager;
			//External to internal info
			std::map<int, unsigned int> receiverExtToBufferId; //External id to receptionInfo buffer Id
			std::vector<SphereReceiver*> receivers; //receivers info (mapped to receptionInfo buffer)
		public:
			ReceiverManager(OpalSceneManager* m);

			//Register transmitter in Opal. Add to transmitters map
			void registerReceiver(SphereReceiver* rx) ;
			//Find receiver and remove from Opal
			void removeReceiver(int txId) ;
			void clearReceivers();
			std::vector<SphereReceiver*> getReceivers();
			Receiver* getReceiver(int externalId);
			unsigned int getNumberOfReceivers() const;

	};
} //namespace opal
#endif


