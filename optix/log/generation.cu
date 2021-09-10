/***************************************************************/
//
//Copyright (c) 2019 Esteban Egea-Lopez http://girtel.upct.es/~eegea
//
/**************************************************************/


#include "../../Common.h"
#include "../traceFunctions.h"
#include <optix_world.h>
#include <optixu/optixu_math_namespace.h>
#include <optixu/optixu_aabb_namespace.h>
using namespace optix;


//Generation of ray sphere


//Ray Sphere buffer
rtBuffer<float3, 1> hitRays;

//Transmitter buffer
rtBuffer<Transmitter, 1> txBuffer;

//Launch variables
rtDeclareVariable(uint3, launchIndex, rtLaunchIndex, );

rtDeclareVariable(uint, rayTypeIndex, , );


//Configuration variables
rtDeclareVariable(uint2, raySphereSize, , );

RT_PROGRAM void genRayTracesFromHits()
{


	//2D kernel launch [rayIndex, transmitters]	


	const Transmitter tx = txBuffer[launchIndex.y];
	
	float3 origin = make_float3(tx.origin_p);
	
	float3 ray_direction = hitRays[launchIndex.x];

	BaseReflectionPayload rayPayload;
	rayPayload.ndtd = optix::make_float4(0.0f);
	rayPayload.hitPointAtt = make_float4(origin);
	rayPayload.rhfr=make_uint4(0u,0u,FLAG_NONE,0u);
		
	//Print all rays generated
	//rtPrintf("A\t%u\t%u\t%f\t%f\t%f\n", launchIndex.x, launchIndex.y, ray_direction.x, ray_direction.y, ray_direction.z);

	traceReflection<BaseReflectionPayload>(rayPayload, rayTypeIndex, origin, ray_direction, launchIndex.x,launchIndex.y, true);


}


