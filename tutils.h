/***************************************************************/
//
//Copyright (c) 2019 Esteban Egea-Lopez http://ait.upct.es/eegea
//
/**************************************************************/



#pragma once
#include "cuda_runtime.h"
#include "device_launch_parameters.h"
#include <optix_world.h>
#include <optixu/optixu_math_namespace.h>
#include <optixu/optixu_matrix_namespace.h> 
#include <optixu/optixpp_namespace.h>

//Functions to filter duplicate hits on device with the Thrust library

namespace opalthrustutils {
	uint filterHitsWithCopy(optix::Buffer hitBuffer, optix::Buffer resultBuffer, unsigned int bsize); 
	uint filterHitsWithCopyResize(optix::Buffer hitBuffer, optix::Buffer resultBuffer, unsigned int bsize); 
}

