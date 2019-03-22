/***************************************************************/
//
//Copyright (c) 2019 Esteban Egea-Lopez http://ait.upct.es/eegea
//
/**************************************************************/


#include "Common.h"
#include <optix_world.h>
#include <optixu/optixu_math_namespace.h>
#include <optixu/optixu_aabb_namespace.h>
using namespace optix;


//Propagation kernels for single transmitter/ multiple receivers



//Receivers buffers
rtBuffer<HitInfo, 1> resultHitInfoBuffer; //Results buffer to be used  by thrust to store filtering resutls 
rtBuffer<HitInfo, 1> globalHitInfoBuffer; //Buffer to store all the hits
rtBuffer<uint, 1> atomicIndex; //Buffer to store the current global buffer index 
rtDeclareVariable(uint, global_info_buffer_maxsize, ,);

//Transmitter buffer
rtBuffer<Transmitter, 1> tx_origin;



rtDeclareVariable(SphereHit, hit_attr, attribute hit_attr, );
rtDeclareVariable(EMWavePayload, hitPayload, rtPayload, );
rtDeclareVariable(float, k, , );
rtDeclareVariable(uint3, receiverLaunchIndex, rtLaunchIndex, );

rtDeclareVariable(uint, receiverBufferIndex, , ); //Buffer id
rtDeclareVariable(int, externalId, , ); //External id  used to identify receivers 

rtDeclareVariable(float, asRadiusConstant, ,);

rtDeclareVariable(float4, sphere, , );
rtDeclareVariable(optix::Ray, ray_receiver, rtCurrentRay, );




rtDeclareVariable(rtCallableProgramId<float2(float2)>, complex_exp_only_imaginary, , );
rtDeclareVariable(rtCallableProgramId<float2(float, float2)>, sca_complex_prod, , );
rtDeclareVariable(rtCallableProgramId<float2(float2, float2)>, complex_prod, , );


RT_PROGRAM void closestHitReceiverInternalRay()
{

	//Log internal ray
	//	rtPrintf("IR hit\t%u\t%u\t%u\t%d\t%d\t%d\n", receiverLaunchIndex.x, receiverLaunchIndex.y, receiverBufferIndex,hitPayload.rxId,receiverBufferIndex );


	//Update ray data

	const float rayLength = hit_attr.geom_normal_t.w;
	hitPayload.totalDistance += rayLength;
	hitPayload.hitPoint = ray_receiver.origin + rayLength*ray_receiver.direction;
	hitPayload.nextDirection = ray_receiver.direction;
	uint c_tx_index=receiverLaunchIndex.z;
	const Transmitter current_tx=tx_origin[c_tx_index];
	//const Transmitter current_tx = {make_float3(0.0f,0.0f,0.0f), make_float3(0.0f,0.0f,0.0f),0};
	//Check if we are hitting the receiver for this internal ray
	if (hitPayload.rxBufferIndex!=receiverBufferIndex) {
		//rtPrintf("IR not receiver\t%u\t%u\t%u\t%d\t%d\t%d\n", receiverLaunchIndex.x, receiverLaunchIndex.y, receiverBufferIndex,hitPayload.rxBufferIndex,receiverBufferIndex );
		return;
	} else {
		//We finish the internal ray always once we hit the sphere again
		hitPayload.end=true; 
	}


	//TODO: We do not check polarization between tx and rx. Can be done comparing payload polarization and receiver polarization

	//Do not end the ray, it can pass through the reception sphere and reflect on a wall, inside or outside the receiver sphere



	//Check if ray is hitting his own tx (transmitter are also receivers usually) A transmitter cannot receive while it is transmitting, unless other channel is used.
	if (externalId == current_tx.externalId) {
		//My own outgoing ray
		return;
	}


	int reflections = hitPayload.reflections;
	if (reflections==hitPayload.internalRayInitialReflections) {
		//Not reflected, do not do anything else
		//rtPrintf("Not reflected\t%u\t%u\t%u\t%d\n", receiverLaunchIndex.x, receiverLaunchIndex.y, receiverBufferIndex,hitPayload.rxBufferIndex);
		return;
	}


	//Reflected ray

	if (reflections>0) {
		//Distance from ray line to receiver position
		//Line is defined by ray
		float3 prx = make_float3(sphere.x, sphere.y, sphere.z);
		float3 pd = prx - hitPayload.hitPoint;
		float u = dot(pd, ray_receiver.direction);
		float3 p3 = hitPayload.hitPoint + u*ray_receiver.direction;


		float dm = length(prx - p3)*1000000.0f;  //Multiply by 1000 000 to truncate later take 6 digits
		int dmt = __float2int_rz(dm);   //Truncate
		float d=length(prx-hitPayload.lastReflectionHitPoint);


		//Compute electric field
		d+=hitPayload.totalDistanceTillLastReflection;

		float2 z = make_float2(0.0f, -k*d);
		float2 zexp = complex_exp_only_imaginary(z);
		float2 Rzexp = complex_prod(hitPayload.prodReflectionCoefficient, zexp);

		uint hitIndex=atomicAdd(&atomicIndex[0u],1u);
		//Check if global buffer is full
		if (hitIndex>=global_info_buffer_maxsize) {
			rtThrow(GLOBALHITBUFFEROVERFLOW);
			//if exceptions are disabled, let it crash...?
			hitIndex=global_info_buffer_maxsize-1;
		}

		float2 E = sca_complex_prod((hitPayload.electricFieldAmplitude / d), Rzexp);
		HitInfo internalHit;
		internalHit.thrd=make_uint4(c_tx_index,hitPayload.refhash,receiverBufferIndex,static_cast<uint>(dmt));
		internalHit.E=E;

		//Store hit in global buffer
		globalHitInfoBuffer[hitIndex]=internalHit;

		//Log hit
		//rtPrintf("CI\t%u\t%u\t%u\t%f\t%f\t%f\t%f\t%f\t%u\t%u\t%u\t%d\n", receiverLaunchIndex.x, receiverLaunchIndex.y,receiverBufferIndex,  directHit.E.x, directHit.E.y, E.x, E.y,d, directHit.thrd.x,directHit.thrd.y,directHit.thrd.w,externalId);


	}
}




RT_PROGRAM void closestHitReceiver()
{

	//TODO: We do not check polarization between tx and rx. Can be done comparing payload polarization and receiver polarization

	//Do not end the ray, it can pass through the reception sphere and reflect on a wall, inside or outside the receiver sphere

	//Update ray data
	const float rayLength = hit_attr.geom_normal_t.w;
	hitPayload.totalDistance += rayLength; 
	hitPayload.hitPoint = ray_receiver.origin + rayLength*ray_receiver.direction;
	hitPayload.nextDirection = ray_receiver.direction;

	uint c_tx_index=receiverLaunchIndex.z;
	const Transmitter current_tx=tx_origin[c_tx_index];
	//const Transmitter current_tx = {make_float3(0.0f,0.0f,0.0f), make_float3(0.0f,0.0f,0.0f),0};

	//Check if ray is hitting his own tx (transmitter are also receivers usually) A transmitter cannot receive while it is transmitting, unless other channel is used.
	if (externalId == current_tx.externalId) {
		//My own outgoing ray
		//rtPrintf("External. txId=%d i.el=%u i.az=%u, ray=(%f,%f,%f) origin=(%f,%f,%f) t=%f rId[%u]=%d\n", tx_origin.externalId, receiverLaunchIndex.x, receiverLaunchIndex.y, ray_receiver.direction.x, ray_receiver.direction.y, ray_receiver.direction.z, ray_receiver.origin.x, ray_receiver.origin.y, ray_receiver.origin.z, hit_attr.t, receiverBufferIndex,externalId);
		return;
	}


	int reflections = hitPayload.reflections;
	float3 prx = make_float3(sphere.x, sphere.y, sphere.z);

	//Check if ray originated inside the reception radius of this receiver

	//Distance from ray origin to receiver
	//float dor=length(ray_receiver.origin-prx);
	float3 raytorx=ray_receiver.origin-prx;	
	float dorsquare=dot(raytorx,raytorx);


	if (dorsquare<=((sphere.w*sphere.w)+0.0001)) { //Give some epsilon, otherwise numerical inaccuracies may make it fail the check

		// Origin is inside the reception radius: This ray has hit us before, ignore it. Internal ray may compute additional contributions from potential reflections inside the reception sphere

		//rtPrintf("Ignored. txId=%d i.x=%u i.y=%u, ray=(%f,%f,%f) origin=(%f,%f,%f) t=%f rId[%u]=%d\n", tx_origin.externalId, receiverLaunchIndex.x, receiverLaunchIndex.y, ray_receiver.direction.x, ray_receiver.direction.y, ray_receiver.direction.z, ray_receiver.origin.x, ray_receiver.origin.y, ray_receiver.origin.z, hit_attr.t, receiverBufferIndex,externalId);
		return;

	} else {
		//If ray origin is outside the reception radius an internal ray is always created to keep track of internal reflections 
		//Mark to  trace the internal ray to check if it collides with another thing and hits the receiver again

		hitPayload.rxBufferIndex=receiverBufferIndex;	
	}


	//HitInfo values
	float d;
	uint hash=0u;
	uint dtrx=0u;
	if (reflections == 0) {
		//This is a direct ray
		//Compute electric field. For direct rays, the distance is always between tx and rx
		float3 ptx = current_tx.origin;
		d = length(prx - ptx);
	} else {
		//Reflected ray


		//Distance from ray line to receiver position. To keep only the closest hit later
		//Line is defined by ray
		float3 pd = prx - hitPayload.hitPoint;
		float u = dot(pd, ray_receiver.direction);
		float3 p3 = hitPayload.hitPoint + u*ray_receiver.direction;


		float3 rxtoh=prx-p3;
		float drxtohitsq=dot(rxtoh,rxtoh);
		//float drxtohit=length(prx - p3);

		//Unfolded path distance		
		d=length(prx-hitPayload.lastReflectionHitPoint);

		d+=hitPayload.totalDistanceTillLastReflection;

		//Take into account radius and angular separation: ignore hit if distance from hit to receiver is > unfolded path distance * angular separation

		float vrsq=d*d*asRadiusConstant*asRadiusConstant/3; //sqr(d*as/sqrt(3));
		if (drxtohitsq>vrsq) {
			return;
		}
		float dm = drxtohitsq*1000000.0f;  //Multiply by 1000 000 to truncate later take 6 digits
		int dmt = __float2int_rz(dm);   //Truncate
		//HitInfo values
		dtrx=static_cast<uint>(dmt);
		hash=hitPayload.refhash;
	}



	//Compute electric field	
	float2 z = make_float2(0.0f, -k*d);
	float2 zexp = complex_exp_only_imaginary(z);
	float2 Rzexp = complex_prod(hitPayload.prodReflectionCoefficient, zexp);


	float2 E = sca_complex_prod((hitPayload.electricFieldAmplitude / d), Rzexp);

	HitInfo aHit;
	aHit.thrd=make_uint4(c_tx_index,hash,receiverBufferIndex,dtrx);
	aHit.E=E;
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
	//if (reflections==0) {
	//rtPrintf("DH\t%u\t%u\t%u\t%f\t%f\t%f\t%f\t%f\t%u\t%u\t%u\t%d\n", receiverLaunchIndex.x, receiverLaunchIndex.y,receiverBufferIndex,  directHit.E.x, directHit.E.y, E.x, E.y,d, directHit.thrd.x,directHit.thrd.y,directHit.thrd.w,externalId);
	// } else {
	//rtPrintf("C\t%u\t%u\t%u\t%f\t%f\t%f\t%f\t%f\t%u\t%u\t%u\t%d\n", receiverLaunchIndex.x, receiverLaunchIndex.y,receiverBufferIndex,  reflectedHit.E.x, reflectedHit.E.y, E.x, E.y,d, reflectedHit.thrd.x,reflectedHit.thrd.y,reflectedHit.thrd.w,externalId);
	//}	

}









rtDeclareVariable(EMWavePayload, missPayload, rtPayload, );
RT_PROGRAM void miss()
{
	//rtPrintf("miss i.x=%u. iy=%u \n", receiverLaunchIndex.x, receiverLaunchIndex.y);
	missPayload.end = true;
}

