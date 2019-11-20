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

//Simulate propagation inside a tunnel. Used to exemplify partial launches with low angular separation (precise angular sampling). To be run with a high number of reflections, such as 20 or 30
std::unique_ptr<OpalSceneManager> cubeTunnel(std::unique_ptr<OpalSceneManager> sceneManager, float sphereRadius) ;
void xcut(OpalSceneManager* sceneManager, float width, float height, float y, float distance, float3 polarization, float sphereRadius); 
void ycut(OpalSceneManager* sceneManager, float width, float height, float x, float distance, float3 polarization, float sphereRadius) ;
void zrun(OpalSceneManager* sceneManager, float zinit, float deltad, float x, float y, float length, float3 polarization, float sphereRadius) ;

#endif

