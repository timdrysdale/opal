/***************************************************************/
//
//Copyright (c) 2019 Esteban Egea-Lopez http://girtel.upct.es/~eegea
//
/**************************************************************/

#ifndef TRANSMITTERMANAGER_H
#define TRANSMITTERMANAGER_H

#include "Opal.h"

//Manager for  multi-transmitter and multi-receiver operation. Supports simultaneous parallel transmission of multiple nodes
namespace opal {
	class OpalSceneManager;  //Forward declaration

	class TransmitterManager {
		protected:
			OpalSceneManager* myManager;
			std::vector<Transmitter*> activeTransmitters; //Map from internal buffer index to external Id. Used for grouping transmissions (batches)
			std::map<int, Transmitter*> transmitterExtToBase; //Map from externalId to transmitterBase
		public:
			TransmitterManager(OpalSceneManager* m);

			//Register transmitter in Opal. Add to transmitters map
			void registerTransmitter(int txId, optix::float3 origin, optix::float3 polarization, float transmitPower) ;
			//Find transmitter and remove from Opal. Cannot transmit anymore 
			void removeTransmitter(int txId) ;
			//Add transmitter to next parallel transmission
			void addTransmitterToGroup(int txId,float transmitPower, optix::float3 origin,optix::float3 polarization);
			//Add transmitter to next parallel transmission
			void addTransmitterToGroup(int txId,float transmitPower, optix::float3 origin);
			//Clear current transmit group
			void clearGroup();
			//Transmit simultaneously all transmitters in group
			void groupTransmit(bool partial=false) ;
			const std::vector<Transmitter*>& getActiveTransmitters();
			Transmitter* getTransmitter(int externalId);
			unsigned int getTransmitterIndex(int externalId);
			void registerTransmitterGain(int txId, int gainId); 

	};
} //namespace opal
#endif


