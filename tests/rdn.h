/***************************************************************/
//
//Copyright (c) 2019 Esteban Egea-Lopez http://girtel.upct.es/~eegea
//
/**************************************************************/


#ifndef RDN_H
#define RDN_H
#include "tunnelsBase.h"
#include <string>
#include "../rayDensityNormalizationSimulation.h"
namespace opal {
	class RDNTests: TunnelsBase {
		protected:
			RayDensityNormalizationSimulation* sim;
			void didascalouDielectricIsotropic(bool half,  float3 polarization, float3 postx, float3 polarizationTx);
			void didascalouConductorIsotropic(bool half,  float3 polarization, float3 postx, float3 polarizationTx);
		public:
			RDNTests(OpalSceneManager* sceneManager, float sphereRadius, bool useDepolarization); 
			void rdnPlaneTestMultiReceiver() ;
			void rdnPlaneTest() ;
			void freeSpace() ;
			void runDidascalouDielectricRDN(int mode);
			void runDidascalouDielectricMultipleReceivers(int mode);
			void runDidascalouDielectric(int mode) ;
			void runDidascalouConductor(int mode) ;
			void runDidascalouConductorRDN(int mode);
			void runTest(std::string test);
	};
}
#endif


