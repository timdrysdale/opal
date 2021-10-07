/***************************************************************/
//
//Copyright (c) 2019 Esteban Egea-Lopez http://girtel.upct.es/~eegea
//
/**************************************************************/


#ifndef ROUX_H
#define ROUX_H
#include "tunnelsBase.h"
#include <string>
namespace opal {
	class RouxTests: TunnelsBase {
		protected:
			void loadRouxTunnel(float radius,  float length, float height);	
		public:
			RouxTests(OpalSceneManager* sceneManager, float sphereRadius, bool useDepolarization); 
			void runTunnel();
	};
}
#endif



