/***************************************************************/
//
//Copyright (c) 2019 Esteban Egea-Lopez http://girtel.upct.es/~eegea
//
/**************************************************************/


#include "../../Common.h"
#include "../Complex.h"
#include "../configuration.h"
#include "../penetrationFunctions.h"
#include "../receiverFunctions.h"
#include <optix_world.h>
#include <optixu/optixu_math_namespace.h>
#include <optixu/optixu_aabb_namespace.h>
using namespace optix;



//Receiver global buffers
rtBuffer<HitInfo, 1> globalHitInfoBuffer; //Buffer to store all the hits
rtBuffer<uint, 1> atomicIndex; //Buffer to store the current global buffer index 
rtDeclareVariable(uint, global_info_buffer_maxsize, ,);

//Transmitter buffer
rtBuffer<Transmitter, 1> txBuffer;


//Receiver local variables
rtDeclareVariable(uint, receiverBufferIndex, , ); //Buffer id
rtDeclareVariable(int, externalId, , ); //External id  used to identify receivers 
rtDeclareVariable(SphereHit, hit_attr, attribute hit_attr, );
//rtDeclareVariable(float, k, , );
rtDeclareVariable(float4, sphere, , );

//Launch variables
rtDeclareVariable(uint3, receiverLaunchIndex, rtLaunchIndex, );
rtDeclareVariable(HVWavePayload, hitPayload, rtPayload, );
rtDeclareVariable(optix::Ray, ray_receiver, rtCurrentRay, );

//Global variables
rtDeclareVariable(float, asRadiusConstant, ,);

//Penetration configuration
//rtDeclareVariable(uint, usePenetration, , );

//Antenna gain
typedef rtBufferId<float,2> AGB;
rtDeclareVariable(AGB, gainBufferId, ,);


//Closest hit program for receivers
RT_PROGRAM void closestHitReceiver()
{


	//Do not end the ray, it can pass through the reception sphere and reflect on a wall, inside or outside the receiver sphere

	//Update ray data
	const float rayLength = hit_attr.geom_normal_t.w;
	const float3 hitPoint = updateReceiverPayload<HVWavePayload>(hitPayload,rayLength,ray_receiver);
	//const float3 hitPoint = ray_receiver.origin + rayLength*ray_receiver.direction;
	////hitPayload.hitPoint=hitPoint;
	//hitPayload.hitPointAtt.x=hitPoint.x;
	//hitPayload.hitPointAtt.y=hitPoint.y;
	//hitPayload.hitPointAtt.z=hitPoint.z;
	//const float aux= hitPayload.ndtd.w; //Previous total distance
	//hitPayload.ndtd = make_float4(ray_receiver.direction); //Next direction only
	//hitPayload.ndtd.w = aux+ rayLength; //totalDistance 

	const uint txBufferIndex=receiverLaunchIndex.z;
	const Transmitter current_tx=txBuffer[txBufferIndex];
	float3 prx = make_float3(sphere.x, sphere.y, sphere.z);
#ifdef OPAL_AVOID_SI

	hitPayload.lastNormal=make_float3(hit_attr.geom_normal_t.x,hit_attr.geom_normal_t.y,hit_attr.geom_normal_t.z);
	//hitPayload.lastNormal=normalize(hitPoint-prx);
#endif

	//Check if ray is hitting his own tx (transmitter are also receivers usually) A transmitter cannot receive while it is transmitting, unless other channel is used.
	if (externalId == current_tx.externalId) {
		//My own outgoing ray
		//rtPrintf("External. txId=%d i.el=%u i.az=%u, ray=(%f,%f,%f) origin=(%f,%f,%f) t=%f rId[%u]=%d\n", txBuffer.externalId, receiverLaunchIndex.x, receiverLaunchIndex.y, ray_receiver.direction.x, ray_receiver.direction.y, ray_receiver.direction.z, ray_receiver.origin.x, ray_receiver.origin.y, ray_receiver.origin.z, hit_attr.t, receiverBufferIndex,externalId);
		return;
	}


	//uint reflections = hitPayload.rhfr.x;

	//HitInfo values
	float d;
	uint hash=0u;
	//uint dtrx=0u;

	//Distance from ray line to receiver position. To keep only the closest hit later
	//Line is defined by ray
	//float3 pd = prx - hitPayload.hitPoint;
	float3 pd = prx - hitPoint;
	float u = dot(pd, ray_receiver.direction);
	float3 p3 = hitPoint + u*ray_receiver.direction;

	float3 rxtoh=prx-p3;
	float drxtohitsq=dot(rxtoh,rxtoh);


	//Unfolded path distance		
	const float3 lastReflectionHitPoint = make_float3(hitPayload.lrhpd.x,hitPayload.lrhpd.y,hitPayload.lrhpd.z);


	//d=length(prx-lastReflectionHitPoint);
	d=length(p3-lastReflectionHitPoint);
	//rtPrintf("HE\t%u\t%u\t%u\t%6e\t%6e\t%6e \n", receiverLaunchIndex.x, receiverLaunchIndex.y, hitPayload.rhfr.x, d  E.x, E.y,d, hitPayload.prodReflectionCoefficient.x, hitPayload.prodReflectionCoefficient.y,Rzexp.x,Rzexp.y,-k*d);


	d+=hitPayload.lrhpd.w; //totalDistanceTillLastReflection

	//TODO: make this check optional. It works well anyway for large distances and low accuracy
	//Take into account radius and angular separation: ignore hit if distance from hit to receiver is > unfolded path distance * angular separation
	//This is not necessary if the radius is properly set
	//float drxtohit=length(prx - p3);
		//	float vrsq=d*d*asRadiusConstant*asRadiusConstant/3; //sqr(d*as/sqrt(3));
		//	if (drxtohitsq>vrsq) {
		//		return;
		//	}
	//	float dm = drxtohitsq*10000000000.0f;  //Multiply by 1000 0000000 to truncate later take 10 digits
	//	int dmt = __float2int_rz(dm);   //Truncate
	//	//HitInfo values
	//	dtrx=static_cast<uint>(dmt);
	//hash=hitPayload.refhash;
	hash=hitPayload.rhfr.w;



	//Compute electric field
	float k = hitPayload.polarization_k.w;	
	float2 z = make_float2(0.0f, -k*d);
	float2 zexp = complex_exp_only_imaginary(z);
	float2 Rzexp = complex_prod(hitPayload.prodReflectionCoefficient, zexp);

	//rtPrintf("RZ\t%u\t%u\t%u\tk= %6e zexp=(%6e,%6e) d=%6ez=(%6e,%6e)Rzexp=(%6e,%6e)-k*d=%6e \n", receiverLaunchIndex.x, receiverLaunchIndex.y, hitPayload.rhfr.x, k, zexp.x, zexp.y,d, z.x, z.y,Rzexp.x,Rzexp.y,-k*d);

	float2 E = sca_complex_prod((hitPayload.electricFieldAmplitude / d), Rzexp);


	if (usePenetration==1u) {

		E=applyPenetration<HVWavePayload>(hitPayload,E);			

	}

	if (useAntennaGain) {
		
		float g=getAntennaGain(-ray_receiver.direction, gainBufferId);	
		E=sca_complex_prod(g,E);
	}
	//Store hit information on buffer:
	HitInfo aHit;
	//Update rdud=[rayDir (float3), squared distance from sphere hit point to receiver (float)],
	aHit.rdud.w=drxtohitsq; //Keep the squared distance from p3 to receiver in order to sort and get the closest hit to the receiver
	//Update thrd=[txBufferIndex,refhash,rxBufferIndex,hitOnCurved] 
	aHit.thrd=make_uint4(txBufferIndex,hash,receiverBufferIndex,hitPayload.rhfr.z);
	
	aHit.EEx=make_float4(E, make_float2(0.0f,0.0f));
	aHit.EyEz=make_float4(0.0f,0.f,0.0f,0.0f);	
	//initial ray direction is only updated if we actually compute later a log trace
	aHit.rdud.x=hitPayload.initialRayDir.x; 
	aHit.rdud.y=hitPayload.initialRayDir.y; 
	aHit.rdud.z=hitPayload.initialRayDir.z; 


	//********** Debug ***************
	//	const float3 lastReflectionHitPoint = make_float3(hitPayload.lrhpd.x,hitPayload.lrhpd.y,hitPayload.lrhpd.z);
	//	aHit.lh=lastReflectionHitPoint;
	//	//aHit.r=reflections;
	//	aHit.in=receiverLaunchIndex;
	//	aHit.Ex=E;
	//	aHit.Ey=E;
	//	aHit.Rp=hitPayload.prodReflectionCoefficient;
	//	aHit.Rn=hitPayload.prodReflectionCoefficient;
	//*********************************

	//Check if global buffer is full
	uint hitIndex=atomicAdd(&atomicIndex[0u],1u);
	if (hitIndex>=global_info_buffer_maxsize) {
		rtThrow(GLOBALHITBUFFEROVERFLOW);
		//if exceptions are disabled, let it crash...?
		hitIndex=global_info_buffer_maxsize-1;
	}
	//Store hit in global buffer
	globalHitInfoBuffer[hitIndex]=aHit;
	//Log hit
	//rtPrintf("H\t%u\t%u\t%u\t%u\t%f\t%f\t%f\t%f\t%u\t%u\t%u\t%d\n", receiverLaunchIndex.x, receiverLaunchIndex.y,receiverBufferIndex, hitPayload.reflections, attE,  E.x, E.y,d, aHit.thrd.x,aHit.thrd.y,aHit.thrd.w,externalId);
}








rtDeclareVariable(HVWavePayload, missPayload, rtPayload, );
//Miss program. End ray
RT_PROGRAM void miss()
{
	//rtPrintf("miss i.x=%u. iy=%u \n", receiverLaunchIndex.x, receiverLaunchIndex.y);
	//missPayload.flags = FLAG_END;
	//Set the end flag
	missPayload.rhfr.z |= 1u<<FLAG_END_POSITION;
	//missPayload.rhfr.z = FLAG_END;
}

