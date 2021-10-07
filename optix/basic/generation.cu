/***************************************************************/
//
//Copyright (c) 2019 Esteban Egea-Lopez http://girtel.upct.es/~eegea
//
/**************************************************************/


#include "../../Common.h"
#include "../traceFunctions.h"
#include "../configuration.h"
#include "../receiverFunctions.h" //For antenna gain
#include <optix_world.h>
#include <optixu/optixu_math_namespace.h>
#include <optixu/optixu_aabb_namespace.h>
using namespace optix;


//Generation of ray sphere


//Ray Sphere buffer
rtBuffer<float3, 2> raySphere2D;

//Transmitter buffer
rtBuffer<Transmitter, 1> txBuffer;

//Sphere parameters buffer
rtBuffer<uint4, 1> raySphereParametersBuffer;
rtDeclareVariable(uint, rayTypeIndex, , );

//Launch variables
rtDeclareVariable(uint3, launchIndex, rtLaunchIndex, );



//Configuration variables
//rtDeclareVariable(uint2, raySphereSize, , );
//rtDeclareVariable(uint, standardSphere, , );
rtDeclareVariable(uint, initialHash, , );

RT_PROGRAM void genRayAndReflectionsFromSphereIndex()
{


	//3D kernel launch [elevation, azimuth, transmitters]	

	uint2 idx = make_uint2(launchIndex.x, launchIndex.y); //[elevation, azimuth]
	const uint standardSphere= raySphereParametersBuffer[0].z;
	const uint2 raySphereSize = make_uint2(raySphereParametersBuffer[0].x,raySphereParametersBuffer[0].y);
	if (standardSphere==1u) {
		//index goes from 0 to raySphereSize.x-1 //The last elevation step corresponds to 180 degrees elevation
		if ((idx.x == 0 ||idx.x==  raySphereSize.x-1  ) && idx.y != 0) {
			//These rays are all the same (0,1,0) or (0,-1,0). Only trace  (0,0) and (last,0) corresponding to 0 and 180 elevation degrees
			return;
		}
	}

	const Transmitter tx = txBuffer[launchIndex.z];
	
	float3 origin = make_float3(tx.origin_p);
	
	float3 ray_direction = raySphere2D[idx];

	HVWavePayload rayPayload;
	rayPayload.ndtd = optix::make_float4(0.0f); //rayPayload.nextDirection = optix::make_float3(0, 0, 0) //rayPayload.totalDistance = 0.0f;
	rayPayload.hitPointAtt =make_float4(origin);
	rayPayload.hitPointAtt.w=0.0f;
	
	rayPayload.lrhpd = make_float4(origin);
	rayPayload.lrhpd.w = 0.0f; //totalDistanceTillLastReflection
	
	rayPayload.polarization_k = tx.polarization_k;
	//TODO: Add possibilty of differentInitialFieldAmplitude;	
	if (useAntennaGain) {
		rtBufferId<float,2> bid=tx.gainId;
		const Matrix<4,4> tp=tx.transformToPolarization;
		float g=getAntennaGain(ray_direction,bid,tp) ;	
		rayPayload.electricFieldAmplitude = g; //Gain is already in electric field units, no need to convert from dB or take sqrt 
	} else {
		rayPayload.electricFieldAmplitude = 1.0f; //Normalized Eo=1. Antenna Gain = 1. 
	}
	rayPayload.prodReflectionCoefficient = make_float2(1.0f, 0.0f);
	
	
	rayPayload.rhfr=make_uint4(0u,0u,FLAG_NONE,initialHash);

	rayPayload.initialRayDir=make_float4(ray_direction);

	
	//Print all rays generated
	//rtPrintf("A\t%u\t%u\t%f\t%f\t%f\n", launchIndex.x, launchIndex.y, ray_direction.x, ray_direction.y, ray_direction.z);

	traceReflection<HVWavePayload>(rayPayload,rayTypeIndex, origin, ray_direction, launchIndex.x,launchIndex.y,false);

}


