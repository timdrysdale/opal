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


//Ray Sphere buffer
rtBuffer<float3> raySphere;
rtBuffer<float3, 2> raySphere2D;


//Receivers buffers




rtBuffer<HitInfo, 1> resultHitInfoBuffer; //Results buffer to be used  by thrust to store filtering resutls 
rtBuffer<HitInfo, 1> globalHitInfoBuffer; //Buffer to store all the hits
rtBuffer<uint, 1> atomicIndex; //Buffer to store the current global buffer index 
rtDeclareVariable(uint, global_info_buffer_maxsize, ,);
rtDeclareVariable(Transmitter, tx_origin, ,);

rtDeclareVariable(uint2, launchIndex, rtLaunchIndex, );

rtDeclareVariable(rtObject, root, , );
rtDeclareVariable(float, min_t_epsilon, , );
rtDeclareVariable(unsigned int, max_interactions, , );

rtDeclareVariable(uint2, raySphereSize, , );
RT_PROGRAM void genRayAndReflectionsFromSphereIndex()
{


	//2D kernel launch [elevation, azimuth]	

	uint2 idx = make_uint2(launchIndex.x, launchIndex.y); //[elevation, azimuth]
	//index goes from 0 to raySphereSize.x-1 //The last elevation step corresponds to 180 degrees elevation
	if ((idx.x == 0 ||idx.x==  raySphereSize.x-1  ) && idx.y != 0) {
		//These rays are all the same (0,1,0) or (0,-1,0). Only trace  (0,0) and (last,0) corresponding to 0 and 180 elevation degrees
		return;
	}
	float3 origin = tx_origin.origin;
	float3 ray_direction = raySphere2D[idx];

	EMWavePayload rayPayload;
	rayPayload.geomNormal = optix::make_float3(0, 0, 0);
	rayPayload.nextDirection = optix::make_float3(0, 0, 0);
	rayPayload.hitPoint = origin;
	rayPayload.polarization = tx_origin.polarization;
	rayPayload.lastReflectionHitPoint = origin;
	rayPayload.electricFieldAmplitude = 1.0f; //Normalized Eo=1. Antenna Gain = 1. TODO: Implement antenna gain with buffer dependent on the ray direction and txId : initialEFAmplitude[txId] * antennaGain[txId]);
	rayPayload.t = -1.0f;
	rayPayload.reflections = 0;
	rayPayload.internalRayInitialReflections=0;
	rayPayload.hits = 0;
	rayPayload.totalDistance = 0.0f;
	rayPayload.end = false;

	rayPayload.prodReflectionCoefficient = make_float2(1.0f, 0.0f);
	//rayPayload.faceId = 0u;
	rayPayload.rxBufferIndex=-1;
	rayPayload.refhash=0;


	// Each iteration is a segment (due to reflections) of the ray path.  The closest hit will
	// return new segments to be traced here. Additionally, the closest hit at receiver will generate another ray to continue the propagation through the recption sphere
	//Print all rays generated
	//rtPrintf("A\t%u\t%u\t%f\t%f\t%f\n", launchIndex.x, launchIndex.y, ray_direction.x, ray_direction.y, ray_direction.z);
	while (true) {
		optix::Ray myRay(origin, ray_direction, 0, min_t_epsilon, RT_DEFAULT_MAX);

		rtTrace(root, myRay, rayPayload);

		//rtPrintf("AB\t%u\t%u\t%f\t%f\t%f\n", launchIndex.x, launchIndex.y, ray_direction.x, ray_direction.y, ray_direction.z, rayPayload.rxBufferIndex);
		if (rayPayload.rxBufferIndex>=0) {	
			//Hit a receiver. Trace internal ray	
			EMWavePayload internalRayPayload;
			internalRayPayload.geomNormal = optix::make_float3(0, 0, 0);
			internalRayPayload.nextDirection = optix::make_float3(0, 0, 0);
			internalRayPayload.hitPoint = rayPayload.hitPoint;
			internalRayPayload.lastReflectionHitPoint = rayPayload.lastReflectionHitPoint;
			internalRayPayload.polarization = rayPayload.polarization;
			internalRayPayload.electricFieldAmplitude = 1.0f; //Normalized Eo=1. Antenna Gain = 1. Implement antenna gain with antennaBuffer dependent on the ray direction and txId : initialEFAmplitude[txId] * antennaGain[txId]);
			internalRayPayload.t = -1.0f;
			internalRayPayload.reflections = rayPayload.reflections;
			internalRayPayload.internalRayInitialReflections = rayPayload.reflections;

			internalRayPayload.hits = rayPayload.hits;
			internalRayPayload.totalDistance = rayPayload.totalDistance;
			internalRayPayload.end = false;
			internalRayPayload.refhash = rayPayload.refhash;

			internalRayPayload.prodReflectionCoefficient = rayPayload.prodReflectionCoefficient; 
			//internalRayPayload.faceId = rayPayload.faceId;
			internalRayPayload.rxBufferIndex=rayPayload.rxBufferIndex;
			float3 internal_ray_direction = rayPayload.nextDirection;
			float3 internal_origin=rayPayload.hitPoint;
			while (true) {
				optix::Ray internalRay(internal_origin, internal_ray_direction, 1u, min_t_epsilon, RT_DEFAULT_MAX); //Internal ray type =1
				//rtPrintf("IR\t%u\t%u\t%d\tinternal_origin=(%f,%f,%f)internal_direction=(%f,%f,%f)\t%d\t%d\n", launchIndex.x, launchIndex.y, internalRayPayload.rxBufferIndex, internal_origin.x,internal_origin.y,internal_origin.z,internal_ray_direction.x,internal_ray_direction.y,internal_ray_direction.z,internalRayPayload.reflections,internalRayPayload.end);
				rtTrace(root, internalRay, internalRayPayload);
				//Miss or too much attenuation
				if (internalRayPayload.end) {
					//rtPrintf("IR end\t%u\t%u\t%u\t%d\t%d\t%d\n", launchIndex.x, launchIndex.y, internalRayPayload.rxBufferIndex,internalRayPayload.reflections,internalRayPayload.end );
					break;
				}
				//Max number of reflections
				if (internalRayPayload.reflections > max_interactions) {
					//rtPrintf("IR max\t%u\t%u\t%u\t%d\t%d\t%d\n", launchIndex.x, launchIndex.y, internalRayPayload.rxBufferIndex,internalRayPayload.reflections,internalRayPayload.end );
					break;
				}

				//Reflection or going through receiver
				// Update ray data for the next path segment
				internal_ray_direction = internalRayPayload.nextDirection;
				internal_origin = internalRayPayload.hitPoint;
			}
		}


		//Miss or too much attenuation
		if (rayPayload.end) {
			break;
		}
		//Max number of reflections
		if (rayPayload.reflections > max_interactions) {
			break;
		}

		//Reflection or going through receiver
		// Update ray data for the next path segment
		ray_direction = rayPayload.nextDirection;
		origin = rayPayload.hitPoint;
		rayPayload.rxBufferIndex=-1;	


		//Reflection info log (to be used in external programs)
		rtPrintf("R\t%u\t%u\t%d\t%d\t%f\t%f\t%f\t%f\t%f\t%f\t%f\n", launchIndex.x, launchIndex.y,  rayPayload.reflections, rayPayload.hits, ray_direction.x, ray_direction.y, ray_direction.z, rayPayload.hitPoint.x, rayPayload.hitPoint.y, rayPayload.hitPoint.z, rayPayload.totalDistance);
		//Verbose log
		//rtPrintf("Reflecting ray i.x=%u i.y=%u, inter=%d hits=%d rd=(%f,%f,%f) origin=(%f,%f,%f) end=%d \n", launchIndex.x, launchIndex.y, rayPayload.reflections, rayPayload.hits, rayPayload.reflectionDirection.x, rayPayload.reflectionDirection.y, rayPayload.reflectionDirection.z, rayPayload.hitPoint.x, rayPayload.hitPoint.y, rayPayload.hitPoint.z, rayPayload.end);
	}


}












rtDeclareVariable(SphereHit, hit_attr, attribute hit_attr, );
rtDeclareVariable(EMWavePayload, hitPayload, rtPayload, );
rtDeclareVariable(float, k, , );
rtDeclareVariable(uint2, receiverLaunchIndex, rtLaunchIndex, );

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
	if (externalId == tx_origin.externalId) {
		//My own outgoing ray
		//rtPrintf("External hit for internal ray. txId=%d i.x=%u i.y=%u, ray=(%f,%f,%f) origin=(%f,%f,%f) t=%f rId[%u]=%d\n", tx_origin.externalId, receiverLaunchIndex.x, receiverLaunchIndex.y, ray_receiver.direction.x, ray_receiver.direction.y, ray_receiver.direction.z, ray_receiver.origin.x, ray_receiver.origin.y, ray_receiver.origin.z, hit_attr.t, receiverBufferIndex,externalId);
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
		HitInfo directHit;
		directHit.whrd=make_uint4(1u,hitPayload.refhash,receiverBufferIndex,static_cast<uint>(dmt));
		directHit.E=E;

		//Store hit in global buffer
		globalHitInfoBuffer[hitIndex]=directHit;

		//Log hit
		//rtPrintf("CI\t%u\t%u\t%u\t%f\t%f\t%f\t%f\t%f\t%u\t%u\t%u\t%d\n", receiverLaunchIndex.x, receiverLaunchIndex.y,receiverBufferIndex,  directHit.E.x, directHit.E.y, E.x, E.y,d, directHit.whrd.x,directHit.whrd.y,directHit.whrd.w,externalId);


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


	//Check if ray is hitting his own tx (transmitter are also receivers usually) A transmitter cannot receive while it is transmitting, unless other channel is used.
	if (externalId == tx_origin.externalId) {
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
		float3 ptx = tx_origin.origin;
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
	aHit.whrd=make_uint4(1u,hash,receiverBufferIndex,dtrx);
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
	//rtPrintf("DH\t%u\t%u\t%u\t%f\t%f\t%f\t%f\t%f\t%u\t%u\t%u\t%d\n", receiverLaunchIndex.x, receiverLaunchIndex.y,receiverBufferIndex,  directHit.E.x, directHit.E.y, E.x, E.y,d, directHit.whrd.x,directHit.whrd.y,directHit.whrd.w,externalId);
	// } else {
	//rtPrintf("C\t%u\t%u\t%u\t%f\t%f\t%f\t%f\t%f\t%u\t%u\t%u\t%d\n", receiverLaunchIndex.x, receiverLaunchIndex.y,receiverBufferIndex,  reflectedHit.E.x, reflectedHit.E.y, E.x, E.y,d, reflectedHit.whrd.x,reflectedHit.whrd.y,reflectedHit.whrd.w,externalId);
	//}	

}









rtDeclareVariable(EMWavePayload, missPayload, rtPayload, );
RT_PROGRAM void miss()
{
	//rtPrintf("miss i.x=%u. iy=%u \n", receiverLaunchIndex.x, receiverLaunchIndex.y);
	missPayload.end = true;
}

