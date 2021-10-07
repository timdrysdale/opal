/***************************************************************/
//
//Copyright (c) 2019 Esteban Egea-Lopez http://girtel.upct.es/~eegea
//
/**************************************************************/


#include "../../../Common.h"
#include "../../traceFunctions.h"
#include "../linearPolarizationFunctions.h"
#include "../../configuration.h"
#include "../../receiverFunctions.h"
#include <optix_world.h>
#include <optixu/optixu_math_namespace.h>
#include <optixu/optixu_aabb_namespace.h>
using namespace optix;


//Generation of ray sphere


//Ray Sphere buffer
rtBuffer<float3, 2> raySphere2D;

//Transmitter buffer
rtBuffer<Transmitter, 1> txBuffer;

//RDN config buffer
rtBuffer<RDNParameters, 1> rdnParametersBuffer;

//Launch variables
rtDeclareVariable(uint3, launchIndex, rtLaunchIndex, );
rtDeclareVariable(uint, rayTypeIndex, , );


//Sphere parameters buffer
rtBuffer<uint4, 1> raySphereParametersBuffer;

//Configuration variables
//rtDeclareVariable(uint2, raySphereSize, , );
//rtDeclareVariable(uint, usePenetration, , );
rtDeclareVariable(uint, initialHash, , );
//rtDeclareVariable(uint, standardSphere, , );
//rtDeclareVariable(float, initialDensity, ,);
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

	const float initialDensity = rdnParametersBuffer[0].initialDensity;
	
	RDNLPWavePayload rayPayload;
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
	//TODO: Add possibilty of differentInitialFieldAmplitude;	
	if (useAntennaGain) {
		rtBufferId<float,2> bid=tx.gainId;
		const Matrix<4,4> tp=tx.transformToPolarization;
		float g=getAntennaGain(ray_direction,bid,tp) ;	
		rayPayload.electricFieldAmplitude = g; //Gain is already in electric field units, no need to convert from dB or take sqrt 
	} else {
		rayPayload.electricFieldAmplitude = 1.0f; //Normalized Eo=1. Antenna Gain = 1. 
	}
	//rayPayload.accumulatedAttenuation=0.0f;
	rayPayload.polarization_k=tx.polarization_k;
	rayPayload.rhfr=make_uint4(0u,0u,FLAG_NONE,initialHash);
	//rayPayload.initialRayDir=ray_direction;

	//Curvature data and divergence
	rayPayload.radii=make_float2(0.0f,0.0f);
	rayPayload.divergence=make_float2(1.0f,0.0); //Keep it real 
	//rayPayload.updateDivergence=0u;
	//Initial directions correspond to the vectors normal to the ray. Need to be normalized
	//Even though the field is zero, the principal planes cannot be
	float3 p1;
	float3 p2;
	//Check if the field vectors are zero, dot=squared norm
	if (dot(rayPayload.hor_v, rayPayload.hor_v)<1e-12) {
		p1=normalize(rayPayload.ver_v);
	} else {
		p1=normalize(rayPayload.hor_v);
	}
	p2=normalize(cross(ray_direction,p1));
	rayPayload.p1 = p1; 
	rayPayload.p2 = p2;
	rayPayload.rayDensity = initialDensity;
	rayPayload.phaseJumps=0;
	//rtPrintf("%u\t%u p1=(%f,%f,%f) p2=(%f,%f,%f)\n",launchIndex.x,launchIndex.y,rayPayload.p1.x,rayPayload.p1.y,rayPayload.p1.z,rayPayload.p2.x,rayPayload.p2.y,rayPayload.p2.z);
	
	
	//Print all rays generated
	//rtPrintf("A\t%u\t%u\t%f\t%f\t%f\n", launchIndex.x, launchIndex.y, ray_direction.x, ray_direction.y, ray_direction.z);

	//trace ray
	traceReflection<RDNLPWavePayload>(rayPayload, rayTypeIndex, origin, ray_direction, launchIndex.x,launchIndex.y,false);


}


