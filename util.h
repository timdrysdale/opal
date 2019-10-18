/***************************************************************/
//
//Copyright (c) 2019 Esteban Egea-Lopez http://ait.upct.es/eegea
//
/**************************************************************/
#ifndef UTIL_H
#define UTIL_H
#include <vector>
#include <optix_world.h>
using namespace optix;
void printPower(float power, int txId ); 
std::vector<float3>  loadVerticesFromFile(const char* file); 
std::vector<int>  loadTrianglesFromFile(const char* file) ;
#endif

