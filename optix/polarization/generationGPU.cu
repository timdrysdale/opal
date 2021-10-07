/***************************************************************/
//
//Copyright (c) 2019 Esteban Egea-Lopez http://girtel.upct.es/~eegea
//
/**************************************************************/


#include "../../Common.h"
#include "../traceFunctions.h"
#include "../configuration.h"
#include "../receiverFunctions.h" //For antenna gain
#include "linearPolarizationFunctions.h"
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
//rtDeclareVariable(uint, usePenetration, , );
rtDeclareVariable(uint, initialHash, , );

RT_PROGRAM void genRaysOnLaunch()
{


	//3D kernel launch [elevation, azimuth, transmitters]	
	const float4 range=rayRangeBuffer[0];
	const uint2 idx = make_uint2(launchIndex.x, launchIndex.y); //[elevation, azimuth]
	float3 ray_direction = equispacedRayDirection(idx,range);

	const Transmitter tx = txBuffer[launchIndex.z];
	
	float3 origin = make_float3(tx.origin_p);
	

	LPWavePayload rayPayload;
	rayPayload.ndtd = optix::make_float4(0.0f);
	//rayPayload.hitPoint = origin;
	rayPayload.hitPointAtt =make_float4(origin);
	rayPayload.hitPointAtt.w=0.0f;

	rayPayload.hor_coeff=make_float2(1.0f,0.0f);	
	rayPayload.ver_coeff=make_float2(1.0f,0.0f);	

	fillPolarization(rayPayload,make_float3(tx.polarization_k), ray_direction);
	
	//rtPrintf("\t%u\t%u\tray=(%f,%f,%f),pol=(%f,%f,%f), polt=(%f,%f,%f)\n",launchIndex.x, launchIndex.y,ray_direction.x,ray_direction.y,ray_direction.z,tx.polarization.x,tx.polarization.y,tx.polarization.z,rayPayload.E.x,rayPayload.E.y,rayPayload.E.z);
	//rtPrintf("G\t%u\t%u\tray=(%f,%f,%f),pol=(%f,%f,%f),\n",launchIndex.x, launchIndex.y,ray_direction.x,ray_direction.y,ray_direction.z,tx.polarization.x,tx.polarization.y,tx.polarization.z);
	//rtPrintf("G\t%u\t%u\thor_v=(%.6e,%.6e,%.6e),ver_v=(%.6e,%.6e,%.6e), hor_coeff=(%.6e,%.6e), ver_coeff(%.6e,%.6e)\n",launchIndex.x, launchIndex.y,rayPayload.hor_v.x,rayPayload.hor_v.y,rayPayload.hor_v.z,rayPayload.ver_v.x,rayPayload.ver_v.y,rayPayload.ver_v.z, rayPayload.hor_coeff.x,rayPayload.hor_coeff.y,rayPayload.ver_coeff.x,rayPayload.ver_coeff.y);
	
	rayPayload.lrhpd = make_float4(origin);
	rayPayload.lrhpd.w = 0.0f; //totalDistanceTillLastReflection
	rayPayload.polarization_k=tx.polarization_k;
	if (useAntennaGain) {
		rtBufferId<float,2> bid=tx.gainId;
		float g=getAntennaGain(ray_direction,bid,tx.transformToPolarization) ;	
		rayPayload.electricFieldAmplitude = g; //Gain is already in electric field units, no need to convert from dB or take sqrt 
	} else {
		rayPayload.electricFieldAmplitude = 1.0f; //Normalized Eo=1. Antenna Gain = 1. 
	}
	//rayPayload.accumulatedAttenuation=0.0f;
	rayPayload.rhfr=make_uint4(0u,0u,FLAG_NONE,initialHash);

	rayPayload.initialRayDir=make_float4(ray_direction);

	
//	float2 ang= getAngles(ray_direction);
	//Print all rays generated
//	rtPrintf("A\t%u\t%u\t%f\t%f\t%f\t%f\t%f\n", launchIndex.x, launchIndex.y, (ang.x*180.f/M_PIf), (ang.y*180.f/M_PIf), ray_direction.x, ray_direction.y, ray_direction.z);

	//trace ray
	traceReflection<LPWavePayload>(rayPayload, rayTypeIndex, origin, ray_direction, launchIndex.x,launchIndex.y,false);


}


