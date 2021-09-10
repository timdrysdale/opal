/***************************************************************/
//
//Copyright (c) 2019 Esteban Egea-Lopez http://girtel.upct.es/~eegea
//
/**************************************************************/
#ifndef CURVATURE_H
#define CURVATURE_H
#include "../Opal.h"
namespace opal {
	class CurvatureTests {
		protected:
			OpalSceneManager* sceneManager;
			float sphereRadius;

			void loadCylinder(float radius,  float length, MaterialEMProperties emProp1, bool outside); 
		public:
			CurvatureTests(OpalSceneManager* sceneManager, float sphereRadius); 
			void cylinderTest();
			void symmetricDivergenceTest();
	};
}
#endif

