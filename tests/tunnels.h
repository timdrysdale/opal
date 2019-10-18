/***************************************************************/
//
//Copyright (c) 2019 Esteban Egea-Lopez http://ait.upct.es/eegea
//
/**************************************************************/


#ifndef TUNNELS_H
#define TUNNELS_H
#include "../Opal.h"
#include <memory>
using namespace opal;

//Simulate propagation inside a tunnel. Used to exemplify partial launches with low angular separation (precise angular sampling)
std::unique_ptr<OpalSceneManager> cubeTunnel(std::unique_ptr<OpalSceneManager> sceneManager, bool print, bool subSteps) ;

#endif

