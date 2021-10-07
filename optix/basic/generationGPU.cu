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


//Generation of rays directly on GPU


//Ray range buffer
rtBuffer<float4, 1> rayRangeBuffer;



//Transmitter buffer
rtBuffer<Transmitter, 1> txBuffer;

//Launch variables
rtDeclareVariable(uint3, launchIndex, rtLaunchIndex, );


rtDeclareVariable(uint, rayTypeIndex, , );


//Configuration variables
rtDeclareVariable(uint, initialHash, , );

RT_PROGRAM void genRaysOnLaunch()
{


	//3D kernel launch [elevation, azimuth, transmitters]	
	const float4 range=rayRangeBuffer[0];
	const uint2 idx = make_uint2(launchIndex.x, launchIndex.y); //[elevation, azimuth]
	float3 ray_direction = equispacedRayDirection(idx,range);

	const Transmitter tx = txBuffer[launchIndex.z];
	
	float3 origin = make_float3(tx.origin_p);
	

	HVWavePayload rayPayload;
	rayPayload.ndtd = optix::make_float4(0.0f); //rayPayload.nextDirection = optix::make_float3(0, 0, 0) //rayPayload.totalDistance = 0.0f;
	rayPayload.hitPointAtt =make_float4(origin);
	rayPayload.hitPointAtt.w=0.0f;
	
	rayPayload.lrhpd = make_float4(origin);
	rayPayload.lrhpd.w = 0.0f; //totalDistanceTillLastReflection
	
	rayPayload.polarization_k=tx.polarization_k;
	
	rayPayload.prodReflectionCoefficient = make_float2(1.0f, 0.0f);
	//TODO: Add possibilty of differentInitialFieldAmplitude;	
	if (useAntennaGain) {
		rtBufferId<float,2> bid=tx.gainId;
		const Matrix<4,4> tp=tx.transformToPolarization;
		float g=getAntennaGain(ray_direction,bid,tp) ;	
		rayPayload.electricFieldAmplitude = g; //Gain is already in electric field units, no need to convert from dB or take sqrt 
	} else {
		rayPayload.electricFieldAmplitude = 1.0f; //Normalized Eo=1. Antenna Gain = 1. 
	}
	
	
	rayPayload.rhfr=make_uint4(0u,0u,FLAG_NONE,initialHash);

	rayPayload.initialRayDir=make_float4(ray_direction);

	
	//Print all rays generated
	//rtPrintf("A\t%u\t%u\t%f\t%f\t%f\n", launchIndex.x, launchIndex.y, ray_direction.x, ray_direction.y, ray_direction.z);

	traceReflection<HVWavePayload>(rayPayload,rayTypeIndex, origin, ray_direction, launchIndex.x,launchIndex.y,false);

}


