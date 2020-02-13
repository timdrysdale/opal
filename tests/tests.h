/***************************************************************/
//
//Copyright (c) 2019 Esteban Egea-Lopez http://ait.upct.es/eegea
//
/**************************************************************/


#ifndef TESTS_H
#define TEST_H
#include "../Opal.h"
#include "../util.h"
#include <memory>

namespace opal {
	//Basic tests
	//Adding compund dynamic meshes
	std::unique_ptr<OpalSceneManager> addCompoundDynamicMeshes(std::unique_ptr<OpalSceneManager> sceneManager);
	//Adding, moving and removing dynamic meshes
	std::unique_ptr<OpalSceneManager> addRemoveDynamicMeshes(std::unique_ptr<OpalSceneManager> sceneManager, bool print, bool subSteps);
	//Adding and removing dynamic meshes
	std::unique_ptr<OpalSceneManager> addRemoveReceivers(std::unique_ptr<OpalSceneManager> sceneManager);
	//moving receivers
	std::unique_ptr<OpalSceneManager> moveReceivers(std::unique_ptr<OpalSceneManager> sceneManager);
	//Street crossing test. Cubes are intended to be buildings and a plane is the floor
	std::unique_ptr<OpalSceneManager> crossingTest(std::unique_ptr<OpalSceneManager> sceneManager, bool print, bool subSteps);
	//Horizontal plane test. To validate against a two-ray model
	std::unique_ptr<OpalSceneManager> planeTest(std::unique_ptr<OpalSceneManager> sceneManager, bool print, bool subSteps);
	//Two quads as walls and two overlapping receivers
	std::unique_ptr<OpalSceneManager> quadTest(std::unique_ptr<OpalSceneManager> sceneManager, bool print, bool subSteps);
	//Street crossing with vehicle mesh 
	std::unique_ptr<OpalSceneManager> crossingTestAndVehicle(std::unique_ptr<OpalSceneManager> sceneManager);
	//Penetration tests
	//Penetration test. One cube, transmitter and receiver
	std::unique_ptr<OpalSceneManager> penetrationTest(std::unique_ptr<OpalSceneManager> sceneManager, bool print, bool subSteps);
	//Penetration test. Plane 
	std::unique_ptr<OpalSceneManager> penetrationPlane(std::unique_ptr<OpalSceneManager> sceneManager, bool print, bool subSteps);
}
#endif

