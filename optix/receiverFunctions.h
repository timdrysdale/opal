/***************************************************************/
//
//Copyright (c) 2020 Esteban Egea-Lopez http://girtel.upct.es/~eegea
//
/**************************************************************/
#ifndef RECEIVERFUNCTIONS_H 
#define RECEIVERFUNCTIONS_H
#include <optix_world.h>
#include "polarization/linearPolarizationFunctions.h"
using namespace optix;
//Computes the reception point and returns the distance from that point to the last physical interaction and from that point to receiver (squared) [dToInt,dToRxSquared]
	template<class T>
__forceinline__ __device__  float2 distancesToReceptionPoint(T& hitPayload, float3 hitPoint, float3 prx, float3  rayDirection) {
	//We compute the "reception point": since the ray does not hit the receiver but some point on the sphere, we
	//have to decide what is the reception point.

	//Distance from ray line to receiver position. To keep only the closest hit later
	float3 pd = prx - hitPoint; //Vector from  hit point to receiver
	float u = dot(pd, rayDirection); //Projection of  previous vector on ray
	float3 p3 = hitPoint + u*rayDirection; //Point on ray closest to receiver: p3 is on the ray line, if you draw from p3 a line normal to ray it passes through recevier point. 

	//Vector from p3 to receiver
	float3 rxtoh=prx-p3;
	//Squared length of previous vector. This distance is stored in the HitInfo and used to filter rays later with Thrust
	float drxtohitsq=dot(rxtoh,rxtoh);
	//float drxtohit=length(rxtoh);


	//Now we compute the path length of the ray from the last physical interaction to the reception point

	//We get the point of last interaction with a real physical element (wall, etc)
	const float3 lastReflectionHitPoint = make_float3(hitPayload.lrhpd.x,hitPayload.lrhpd.y,hitPayload.lrhpd.z);

	float d=length(p3-lastReflectionHitPoint); //Distance from last physical point to reception point on ray
	//float d=length(prx-lastReflectionHitPoint); //Alternative, distance directly to the receiver point (not on the real trajectory of the ray)


	
	//Take into account radius and angular separation: ignore hit if distance from hit to receiver is > unfolded path distance * angular separation

	//		float vrsq=d*d*asRadiusConstant*asRadiusConstant/3; //sqr(d*as/sqrt(3));
	//		if (drxtohitsq>vrsq) {
	//			//	rtPrintf("DS\t%u\t%u\tRE\tp3=(%.6e,%.6e,%.6e) drxtohitsq=%.6e vrsq=%.6e hash=%u\n",receiverLaunchIndex.x,receiverLaunchIndex.y,p3.x,p3.y,p3.z,drxtohitsq,vrsq,hitPayload.refhash);
	//			return;
	//		}



	//rtPrintf("DM drxtohitsq=%.6e dm=%.6e dtxt=%u\n",drxtohitsq,dm, dtrx);

	return make_float2(d,drxtohitsq);
}
	template<class T>
__forceinline__ __device__  float distancesProjectedToReceptionPoint(T& hitPayload,  float3 prx, float3  rayDirection) {


	//We get the point of last interaction with a real physical element (wall, etc)
	const float3 lastReflectionHitPoint = make_float3(hitPayload.lrhpd.x,hitPayload.lrhpd.y,hitPayload.lrhpd.z);
	const float3 pi=prx-lastReflectionHitPoint; //Vector from point on last element (last reflection) to receiver
  	const float3 n=hitPayload.lastInteractionNormal;
        //float d=dot(n,pi)/fabs(dot(n,normalize(rayDirection))); //distance from receiver to plane defined by normal / cos angle (absolute value to account for inverted normal) and assuming both ray and normal are normalized 
        float d=fabs(dot(n,pi));

	return d;
}
//Updates ray payload after a hit and returns the hit point on the sphere 
	template<class T>
__forceinline__ __device__  float3 updateReceiverPayload(T& hitPayload, float rayLength, optix::Ray& ray_receiver) {
	//Update ray data
	const float3 hitPoint = ray_receiver.origin + rayLength*ray_receiver.direction;
	//hitPayload.hitPoint=hitPoint;
	hitPayload.hitPointAtt.x=hitPoint.x;
	hitPayload.hitPointAtt.y=hitPoint.y;
	hitPayload.hitPointAtt.z=hitPoint.z;
	const float aux= hitPayload.ndtd.w; //Previous total distance
	hitPayload.ndtd = make_float4(ray_receiver.direction); //Next direction only
	hitPayload.ndtd.w = aux+ rayLength; // update totalDistance 

#ifdef OPAL_AVOID_SI
	hitPayload.lastNormal=make_float3(hit_attr.geom_normal_t.x,hit_attr.geom_normal_t.y,hit_attr.geom_normal_t.z);
	//hitPayload.lastNormal=normalize(hitPoint-prx);
#endif

	return hitPoint;
}
	template<class T>
__forceinline__ __device__  void fillHitInfo(T& hitPayload, HitInfo& aHit, uint txBufferIndex, uint receiverBufferIndex, float sqrDistanceReceiverPointToReceiver) {
	//Fill common part of hitinfo

	//Get previous hash
	uint hash=hitPayload.rhfr.w;
	//Update thrd=[txBufferIndex,refhash,rxBufferIndex,hitOnCurved] 
	aHit.thrd=make_uint4(txBufferIndex,hash,receiverBufferIndex,hitPayload.rhfr.z);
	
	//Update rdud=[rayDir (float3), squared distance from reception point to receiver (float)],
	aHit.rdud.w=sqrDistanceReceiverPointToReceiver; //Keep the squared distance from reception point to receiver in order to sort and get the closest hit to the receiver
	
	aHit.rdud.x=hitPayload.initialRayDir.x; 
	aHit.rdud.y=hitPayload.initialRayDir.y; 
	aHit.rdud.z=hitPayload.initialRayDir.z; 
}
__forceinline__ __device__ float getAntennaGain(float3 const  ray, rtBufferId<float, 2>& gains) {
	if (gains==RT_BUFFER_ID_NULL) {
		//TODO: return a provided function. At the moment return 1 as isotropic for this receiver. 
	//	rtPrintf("null gain bufferId\n");
		return 1.0f;
	}
	float2 angles=getAngles(ray);
	float elevation=angles.x; 
	float azimuth = angles.y;
	size_t2 gsize=gains.size();
	uint aa=gsize.x;
	uint ee=gsize.y;
	const float azd=2*M_PIf/aa;
	const float eld=M_PIf/ee;
	if (azimuth<0) {
		azimuth=2*M_PIf+azimuth;
	} 
	if (elevation<0) {
		elevation=2*M_PIf+elevation;
	} 
	const uint azi=floorf(azimuth/azd);
	const uint eli=floorf(elevation/eld);
	float g = gains[make_uint2(azi,eli)];	
	
	//rtPrintf("e=%f a=%f azd=%f eld=%f azi=%d eli=%d  as=%d es=%d g=%6e \n", elevation, azimuth,azd, eld,azi, eli, aa, ee,g);
	//return gains[make_uint2(azi,eli)];	
	return g;
} 

#endif //RECEIVERFUNCTIONS_H
