/***************************************************************/
//
//Copyright (c) 2019 Esteban Egea-Lopez http://girtel.upct.es/~eegea
//
/**************************************************************/

#ifndef DEPOLARIZATION_H
#define DEPOLARIZATION_H
#include "../Opal.h"
#include "../util.h"
using namespace opal;
	//Depolarization tests
	//Polarization test. Horizontal plane test but with arbitrary polarizations. To validate against a two-ray model
std::unique_ptr<OpalSceneManager> polarizationPlaneTest(std::unique_ptr<OpalSceneManager> sceneManager, bool print, bool subSteps) ;
	//Street crossing test with arbitray polarizations. Cubes are intended to be buildings and a plane is the floor
std::unique_ptr<OpalSceneManager> crossingTestDepolarization(std::unique_ptr<OpalSceneManager> sceneManager, bool print, bool subSteps) ;
#endif

