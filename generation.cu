/***************************************************************/
//
//Copyright (c) 2019 Esteban Egea-Lopez http://ait.upct.es/eegea
//
/**************************************************************/


#include "Common.h"
#include "traceFunctions.h"
#include <optix_world.h>
#include <optixu/optixu_math_namespace.h>
#include <optixu/optixu_aabb_namespace.h>
using namespace optix;


//Generation of ray sphere


//Ray Sphere buffer
rtBuffer<float3, 2> raySphere2D;

//Transmitter buffer
rtBuffer<Transmitter, 1> txBuffer;

//Launch variables
rtDeclareVariable(uint3, launchIndex, rtLaunchIndex, );



//Configuration variables
rtDeclareVariable(uint2, raySphereSize, , );
rtDeclareVariable(uint, usePenetration, , );
rtDeclareVariable(uint, standardSphere, , );

RT_PROGRAM void genRayAndReflectionsFromSphereIndex()
{


	//3D kernel launch [elevation, azimuth, transmitters]	

	uint2 idx = make_uint2(launchIndex.x, launchIndex.y); //[elevation, azimuth]
	if (standardSphere==1u) {
		//index goes from 0 to raySphereSize.x-1 //The last elevation step corresponds to 180 degrees elevation
		if ((idx.x == 0 ||idx.x==  raySphereSize.x-1  ) && idx.y != 0) {
			//These rays are all the same (0,1,0) or (0,-1,0). Only trace  (0,0) and (last,0) corresponding to 0 and 180 elevation degrees
			return;
		}
	}

	const Transmitter tx = txBuffer[launchIndex.z];
	
	float3 origin = tx.origin;
	
	float3 ray_direction = raySphere2D[idx];

	HVWavePayload rayPayload;
	rayPayload.ndtd = optix::make_float4(0.0f);
	//rayPayload.nextDirection = optix::make_float3(0, 0, 0);
	//rayPayload.totalDistance = 0.0f;
	rayPayload.hitPoint = origin;
	rayPayload.polarization = tx.polarization;
	rayPayload.lrhpd = make_float4(origin);
	rayPayload.lrhpd.w = 0.0f; //totalDistanceTillLastReflection
	rayPayload.electricFieldAmplitude = 1.0f; //Normalized Eo=1. Antenna Gain = 1. TODO: Implement antenna gain with buffer dependent on the ray direction and txId : initialEFAmplitude[txId] * antennaGain[txId]);
	rayPayload.accumulatedAttenuation=0.0f;
	rayPayload.reflections = 0;
	rayPayload.hits = 0;
	rayPayload.flags = FLAG_NONE;	
	rayPayload.prodReflectionCoefficient = make_float2(1.0f, 0.0f);
	rayPayload.refhash=0;
	
	//Print all rays generated
	//rtPrintf("A\t%u\t%u\t%f\t%f\t%f\n", launchIndex.x, launchIndex.y, ray_direction.x, ray_direction.y, ray_direction.z);

	traceReflection(rayPayload, origin, ray_direction);

}


