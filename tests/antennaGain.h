/***************************************************************/
//
//Copyright (c) 2021 Esteban Egea-Lopez http://girtel.upct.es/~eegea
//
/**************************************************************/
#ifndef ANTENNAGAIN_H
#define ANTENNAGAIN_H
#include "../Opal.h"
namespace opal {
	class AntennaGainTests {
		protected:
			OpalSceneManager* sceneManager;
			float sphereRadius;

		public:
			AntennaGainTests(OpalSceneManager* sceneManager, float sphereRadius);
			void freeSpaceRDN(); 
			void freeSpace(bool useDepolarization); 
	};
}
#endif



