/***************************************************************/
//
//Copyright (c) 2019 Esteban Egea-Lopez http://girtel.upct.es/~eegea
//
/**************************************************************/


#ifndef TESTS_H
#define TEST_H
#include "../Opal.h"
#include <iostream>
using namespace opal;
namespace opal {
	class BasicTests {
		protected:
			OpalSceneManager* sceneManager;
			float sphereRadius;
			bool useDepolarization;
		public:
			//Horizontal plane test. To validate against a two-ray model
			BasicTests(OpalSceneManager* sceneManager, float sphereRadius, bool useDepolarization); 
			void planeTest(int mode);
			void freeSpace();
			//Adding compund dynamic meshes
			void addCompoundDynamicMeshes(); 
			void quadTest( bool print, bool subSteps) ;
			void loadScenario(); 

	};

}
//Basic tests

//Old versions. Uncomment if necessary to try
//std::unique_ptr<OpalSceneManager> addCompoundDynamicMeshes(std::unique_ptr<OpalSceneManager> sceneManager); 
////Adding, moving and removing dynamic meshes
//std::unique_ptr<OpalSceneManager> addRemoveDynamicMeshes(std::unique_ptr<OpalSceneManager> sceneManager, bool print, bool subSteps) ;
////Adding and removing dynamic meshes
//std::unique_ptr<OpalSceneManager> addRemoveReceivers(std::unique_ptr<OpalSceneManager> sceneManager); 
////moving receivers
//std::unique_ptr<OpalSceneManager> moveReceivers(std::unique_ptr<OpalSceneManager> sceneManager, bool useDepolarization) ;
////Street crossing test. Cubes are intended to be buildings and a plane is the floor
//std::unique_ptr<OpalSceneManager> crossingTest(std::unique_ptr<OpalSceneManager> sceneManager, bool print, bool subSteps, bool useDepolarization, float radius) ;
////Street crossing test. Cubes are intended to be buildings and a plane is the floor
//std::unique_ptr<OpalSceneManager> crossingTestMulti(std::unique_ptr<OpalSceneManager> sceneManager, bool print, bool subStepsi, bool useDepolarization, float radius) ;
////Two quads as walls and two overlapping receivers
//std::unique_ptr<OpalSceneManager> quadTest(std::unique_ptr<OpalSceneManager> sceneManager, bool print, bool subSteps) ;
////Street crossing with vehicle mesh 
//std::unique_ptr<OpalSceneManager> crossingTestAndVehicle(std::unique_ptr<OpalSceneManager> sceneManager) ;
////Penetration tests
////Penetration test. One cube, transmitter and receiver
//std::unique_ptr<OpalSceneManager> penetrationTest(std::unique_ptr<OpalSceneManager> sceneManager, bool print, bool subSteps) ;
////Penetration test. Plane 
//std::unique_ptr<OpalSceneManager> penetrationPlane(std::unique_ptr<OpalSceneManager> sceneManager, bool print, bool subSteps) ;
////Free space: to validate filtering
//std::unique_ptr<OpalSceneManager> freeSpaceRDN(std::unique_ptr<OpalSceneManager> sceneManager,  bool useDepolarization, float radius);
//std::unique_ptr<OpalSceneManager> freeSpace(std::unique_ptr<OpalSceneManager> sceneManager,  bool useDepolarization, float radius);
////Horizontal plane test. To validate against a two-ray model
//std::unique_ptr<OpalSceneManager> planeTest(std::unique_ptr<OpalSceneManager> sceneManager, float radius, bool useDepolarization );
//std::unique_ptr<OpalSceneManager> planeTestProgressive(std::unique_ptr<OpalSceneManager> sceneManager, float radius, bool useDepolarization) ;

#endif

