/***************************************************************/
//
//Copyright (c) 2019 Esteban Egea-Lopez http://girtel.upct.es/~eegea
//
/**************************************************************/


#ifndef DUDLEY_H
#define DUDLEY_H
#include "tunnelsBase.h"
#include <string>
namespace opal {
	class DudleyTests: TunnelsBase {
		protected:
		MaterialEMProperties emProp1;
		float freq;
		float3 polarizationTx;
		float3 rx;
		float3 postx;
		public:
			DudleyTests(OpalSceneManager* sceneManager, float sphereRadius, bool useDepolarization); 
			void runDudleyRDN(); 
			void runDudleyRDNIsotropic(bool half);
			void runTest(std::string test);
	};
}
#endif



