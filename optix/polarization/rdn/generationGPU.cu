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
#include "rdnrandom.h"
#include <optix_world.h>
#include <optixu/optixu_math_namespace.h>
#include <optixu/optixu_aabb_namespace.h>
using namespace optix;


//Generation of rays directly on GPU


//Ray range buffer
rtBuffer<float4, 1> rayRangeBuffer;

//Transmitter buffer
rtBuffer<Transmitter, 1> txBuffer;

//rtBuffer<float2, 3> frBuffer; //Buffer to store all the hits

//RDN config buffer
rtBuffer<RDNParameters, 1> rdnParametersBuffer;

//Launch variables
rtDeclareVariable(uint3, launchIndex, rtLaunchIndex, );
rtDeclareVariable(uint2, launchDim,   rtLaunchDim, );

rtDeclareVariable(uint, rayTypeIndex, , );

//Configuration variables
//rtDeclareVariable(uint2, raySphereSize, , );
rtDeclareVariable(uint, initialHash, , );
//rtDeclareVariable(uint, standardSphere, , );
//rtDeclareVariable(float, initialDensity, ,);
RT_PROGRAM void genRaysOnLaunch()
{


	//3D kernel launch [elevation, azimuth, transmitters]	
       
//	const unsigned int nrx = rdnParametersBuffer[0].filter_rc_nrx.z;
//	for (unsigned int i=0; i<nrx; ++i) {
//		uint3 bindex=make_uint3(i,launchIndex.x,launchIndex.y);
//		frBuffer[bindex]=make_float2(0.0f,0.0f);
//		//frBuffer[bindex]=make_float3(0.0f,0.0f,0.0f);
//	}
	const float4 range=rayRangeBuffer[0];
	const float eli = range.x; //initElevation
	const float elf = range.y; //endElevation
	const float azi = range.z; //initAzimuth
	const float aze = range.w; //endAzimuth
	const uint2 idx = make_uint2(launchIndex.x, launchIndex.y); //[elevation, azimuth]
	unsigned int s1=tea<8>(idx.y*launchDim.x+idx.x,1);
	unsigned int s2=tea<8>(idx.y*launchDim.x+idx.x,2);
	float rng1=rng(s1);	
	float rng2=rng(s2);	
	
	//rtPrintf("TEA idx=%u idy=%u lx=%u ly=%u s1=%u s2=%u rng1=%f rng2=%f\n", idx.x, idx.y,launchDim.x, launchDim.y, s1,s2,rng1,rng2);	
	
	float cosEl=cosf(eli)+ rng1*(cosf(elf)-cosf(eli) );
	float sinEl=sqrt(1.0 -(cosEl*cosEl));
	float azr=(aze-azi)*rng2  + azi;

	float3 ray_direction=  make_float3(sinEl*sinf(azr), cosEl, sinEl*cosf(azr) );        
				//  make_float3(sinEl*cosf(azr),  sinEl*sinf(azr), cosEl );   //Left-handed with elevation from Z and azimuth clockwise from X to Y 
	//rtPrintf("eli=%f elf=%f azi=%f aze=%f ray_direction =(%f,%f,%f) \n", eli, elf, azi, aze, ray_direction.x, ray_direction.y, ray_direction.z);	
	const Transmitter tx = txBuffer[launchIndex.z];
	
	float3 origin = make_float3(tx.origin_p);
	

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
	rayPayload.polarization_k=tx.polarization_k;
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
	rayPayload.rhfr=make_uint4(0u,0u,FLAG_NONE,initialHash);
	rayPayload.initialRayDir=make_float4(ray_direction);

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
	rayPayload.phaseJumps=0;;
	//rtPrintf("%u\t%u p1=(%f,%f,%f) p2=(%f,%f,%f)\n",launchIndex.x,launchIndex.y,rayPayload.p1.x,rayPayload.p1.y,rayPayload.p1.z,rayPayload.p2.x,rayPayload.p2.y,rayPayload.p2.z);
	
	
	//Print all rays generated
	//rtPrintf("A\t%u\t%u\t%f\t%f\t%f\n", launchIndex.x, launchIndex.y, ray_direction.x, ray_direction.y, ray_direction.z);

	//trace ray
	traceReflection<RDNLPWavePayload>(rayPayload, rayTypeIndex, origin, ray_direction, launchIndex.x,launchIndex.y,false);


}


