/***************************************************************/
//
//Copyright (c) 2019 Esteban Egea-Lopez http://ait.upct.es/eegea
//
/**************************************************************/


#include "../../Common.h"
#include "../../traceFunctions.h"
#include "linearPolarizationFunctions.h"
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

RT_PROGRAM void genRayAndReflectionsFromSphereIndex()
{


	//3D kernel launch [elevation, azimuth, transmitters]	

	uint2 idx = make_uint2(launchIndex.x, launchIndex.y); //[elevation, azimuth]
	//index goes from 0 to raySphereSize.x-1 //The last elevation step corresponds to 180 degrees elevation
	if ((idx.x == 0 ||idx.x==  raySphereSize.x-1  ) && idx.y != 0) {
		//These rays are all the same (0,1,0) or (0,-1,0). Only trace  (0,0) and (last,0) corresponding to 0 and 180 elevation degrees
		return;
	}

	const Transmitter tx = txBuffer[launchIndex.z];
	
	float3 origin = tx.origin;
	
	float3 ray_direction = raySphere2D[idx];

	LPWavePayload rayPayload;
	rayPayload.ndtd = optix::make_float4(0.0f);
	//rayPayload.hitPoint = origin;
	rayPayload.hitPointAtt =make_float4(origin);
	rayPayload.hitPointAtt.w=0.0f;

	rayPayload.hor_coeff=make_float2(1.0f,0.0f);	
	rayPayload.ver_coeff=make_float2(1.0f,0.0f);	

	fillPolarization(rayPayload,tx.polarization, ray_direction);
	
	//rtPrintf("\t%u\t%u\tray=(%f,%f,%f),pol=(%f,%f,%f), polt=(%f,%f,%f)\n",launchIndex.x, launchIndex.y,ray_direction.x,ray_direction.y,ray_direction.z,tx.polarization.x,tx.polarization.y,tx.polarization.z,rayPayload.E.x,rayPayload.E.y,rayPayload.E.z);
	//rtPrintf("G\t%u\t%u\tray=(%f,%f,%f),pol=(%f,%f,%f),\n",launchIndex.x, launchIndex.y,ray_direction.x,ray_direction.y,ray_direction.z,tx.polarization.x,tx.polarization.y,tx.polarization.z);
	//rtPrintf("G\t%u\t%u\thor_v=(%.6e,%.6e,%.6e),ver_v=(%.6e,%.6e,%.6e), hor_coeff=(%.6e,%.6e), ver_coeff(%.6e,%.6e)\n",launchIndex.x, launchIndex.y,rayPayload.hor_v.x,rayPayload.hor_v.y,rayPayload.hor_v.z,rayPayload.ver_v.x,rayPayload.ver_v.y,rayPayload.ver_v.z, rayPayload.hor_coeff.x,rayPayload.hor_coeff.y,rayPayload.ver_coeff.x,rayPayload.ver_coeff.y);
	
	rayPayload.lrhpd = make_float4(origin);
	rayPayload.lrhpd.w = 0.0f; //totalDistanceTillLastReflection
	rayPayload.electricFieldAmplitude = 1.0f; //Normalized Eo=1. Antenna Gain = 1. TODO: Implement antenna gain with buffer dependent on the ray direction and txId : initialEFAmplitude[txId] * antennaGain[txId]);
	//rayPayload.accumulatedAttenuation=0.0f;
	rayPayload.rhfr=make_uint4(0u,0u,FLAG_NONE,0u);
//	rayPayload.reflections = 0;
//	rayPayload.hits = 0;
//	rayPayload.flags= FLAG_NONE;
//
//	rayPayload.refhash=0;
#ifdef OPAL_LOG_TRACE
	rayPayload.initialRayDir=ray_direction;
#endif
	//Print all rays generated
	//rtPrintf("A\t%u\t%u\t%f\t%f\t%f\n", launchIndex.x, launchIndex.y, ray_direction.x, ray_direction.y, ray_direction.z);

	traceReflection<LPWavePayload>(rayPayload, OPAL_RAY_REFLECTION, origin, ray_direction, launchIndex.x,launchIndex.y);
	//traceLPReflection(rayPayload, origin, ray_direction, launchIndex.x,launchIndex.y);


}


