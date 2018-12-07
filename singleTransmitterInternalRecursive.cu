

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



//TODO: these buffers consume a lot of memory if there are many receivers. This might be replaced by recursive tracing inside the sphere or any other solution. 
//This is a possible way to do this: create a new ray at the first hit with the sphere and trace it recursively, this is the internal ray that will take care of additional hits with the receiver. The original 
//ray just go on and is never considered again when hitting the sphere (just check if ray origin is within sphere radius and ignore it). The internal ray created, passes along the sphere, 
//if it simply hits the sphere again without any reflection, it is just ignored and the tracing stops, otherwise it will hit something and will be reflected and when it finally hits again the sphere, the 
//electric field is added.
//rtBuffer<int, 3> internalRaysBuffer; //Filter internal rays buffer  [[elevationSteps, azimuthSteps, receiver]

//TODO: These buffers consume a lot of memory also if we are adding new dynamic meshes with a lot of faces (e.g, vehicle models) since the faceIds are not being reused. Consider how to reuse faceIds

rtBuffer<int, 3> bufferMinD; //minDistance ray to receiver [reflections, faceId,receiver];


//Buffer with the Electric field of the minimum distance ray [reflections, faceId,receiver];
rtBuffer<float2, 3> bufferMinE; 

//TODO: I am not sure if the previous declaration together with the AtomicExch() done later may create trouble. In fact, the multi-transmitter version has been separated from this because  it actually caused 
//problems when used in a rtBufferId. 
//My main concern is that the AtomicExc should be done simultaneously for both the x and y components of the float2. But that function cannot be passed a float2*.
//A workaround is the following. Use a long long (64 bits) to store both floats (32 bits) of the electric field. The buffer is of long long, that can be used in AtomicExch, then 
//effectively exchanging both values in one call.
//A working example  below:
//rtBuffer<unsigned long long, 3> test; //Buffer with the Electric field stored as long long of the minimum distance ray [reflections, faceId,receiver];
		
//	Create the long long by merging both floats:
//		uint Exi=__float_as_uint(E.x);
//		uint Eyi=__float_as_uint(E.y);
//		unsigned long long El =Exi;
//		El=((El<<32) | Eyi);
//		
//Atomic Exchange in buffer test and later recover the floats:

//		uint iExi=(El>>32);
//		uint iEyi=(El<<32)>>32;
//		float Exf=__uint_as_float(iExi);
//		float Eyf=__uint_as_float(iEyi);
//		rtPrintf("x=%f y=%f ex=%f,ey=%f dx=%f dy=%f\n",E.x,E.y,Exf,Eyf, E.x-Exf,E.y-Eyf);



rtBuffer<ReceptionInfo, 1> receptionInfoBuffer; //Results buffer [receiver]


rtDeclareVariable(Transmitter, tx_origin, ,);

rtDeclareVariable(uint3, initializationLaunchIndex, rtLaunchIndex, );
rtDeclareVariable(uint2, launchIndex, rtLaunchIndex, );

rtDeclareVariable(rtObject, staticMeshes, , );
rtDeclareVariable(rtObject, root, , );
rtDeclareVariable(float, min_t_epsilon, , );
rtDeclareVariable(unsigned int, max_interactions, , );
//rtDeclareVariable(uint2, tx_rx, , ); //[transmitters, receivers]
//rtDeclareVariable(uint, number_of_receivers, , ); // receivers
rtDeclareVariable(unsigned int, number_of_faces, , );

rtDeclareVariable(uint2, raySphereSize, , );
//rtDeclareVariable(uint, initialize, , );


RT_PROGRAM void initializeBuffersFaceBased() {
	//rtPrintf("Initializing reception buff   index=(%u,%u,%u)\n",  launchIndex.x, launchIndex.y, launchIndex.z);




	//3D kernel launch
	//launchIndex = [reflections, faces, receivers]	

	//One per ray and for all receivers 

	//internalRaysBuffer[launchIndex]=-1;
	//rtPrintf("ib[%u,%u,%u]=%d \n",  launchIndex.x, launchIndex.y, launchIndex.z,  ib[launchIndex]);

	//Only once per receiver, does not depend on elevation or azimut	

		uint receiverIndex = initializationLaunchIndex.z; 
		receptionInfoBuffer[receiverIndex].sumRxElectricField = make_float2(0.0f, 0.0f);
		receptionInfoBuffer[receiverIndex].directHits = 0;
		receptionInfoBuffer[receiverIndex].reflections = 0;
		//rtPrintf("Initializing reception buff  Ep0[%u]=(%f,%f)  \n", receiverIndex, receptionInfoBuffer[receiverIndex].sumRxElectricField.x, receptionInfoBuffer[receiverIndex].sumRxElectricField.y);
		//rtPrintf("Initializing reception buff  Ep0[%u,%u]=(%f,%f)  \n", receiverIndex.x, receiverIndex.y, receptionInfoBuffer[receiverIndex].sumRxElectricField.x, receptionInfoBuffer[receiverIndex].sumRxElectricField.y);
			//uint2 idf = make_uint2(k, i);
			//	facesBuffer[idf] = 0u;
				//uint3 idmd = make_uint3(launchIndex.x, launchIndex.y, launchIndex.z);
				bufferMinD[initializationLaunchIndex] = 2147483647;
				bufferMinE[initializationLaunchIndex] = make_float2(0.0f, 0.0f);
				//ife[idmd].E = make_float2(0.0f, 0.0f);
				//ife[idmd].r = 0;
				//rtPrintf("Initializing faces bufferMinD buff idmd=(%u,%u,%u)=%d \n", initializationLaunchIndex.x,initializationLaunchIndex.y,initializationLaunchIndex.z, bufferMinD[initializationLaunchIndex]);
				//rtPrintf("Initializing faces ifm bufferMinE =(%u,%u,%u)=(%f,%f) \n", initializationLaunchIndex.x,initializationLaunchIndex.y,initializationLaunchIndex.z, bufferMinE[initializationLaunchIndex].x,bufferMinE[initializationLaunchIndex].y);

	
	
	
	



}
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
		rayPayload.electricFieldAmplitude = 1.0f; //Normalized Eo=1. Antenna Gain = 1. Implement antenna gain with antennaBuffer dependent on the ray direction and txId : initialEFAmplitude[txId] * antennaGain[txId]);
		rayPayload.t = -1.0f;
		rayPayload.reflections = 0;
		rayPayload.internalRayInitialReflections=0;
		rayPayload.hits = 0;
		rayPayload.totalDistance = 0.0f;
		rayPayload.end = false;

		rayPayload.prodReflectionCoefficient = make_float2(1.0f, 0.0f);
		rayPayload.faceId = 0u;
		rayPayload.rxBufferIndex=-1;

		//rtPrintf("Lsphere2D el=%u az=%u  ray=(%f,%f,%f)\n", launchIndex.x, launchIndex.y, ray_direction.x, ray_direction.y, ray_direction.z );


		// Each iteration is a segment (due to reflections) of the ray path.  The closest hit will
		// return new segments to be traced here. Additionally, the closest hit at receiver will generate another ray to continue the propagation through the recption sphere
		//rtPrintf("Generating ray i.x=%u i.y=%u, ray=(%f,%f,%f) inter=%d end=%d \n", launchIndex.x, launchIndex.y, ray_direction.x, ray_direction.y, ray_direction.z, rayPayload.reflections, rayPayload.end);
			//rtPrintf("A\t%u\t%u\t%f\t%f\t%f\n", launchIndex.x, launchIndex.y, ray_direction.x, ray_direction.y, ray_direction.z);
		while (true) {
			optix::Ray myRay(origin, ray_direction, 0, min_t_epsilon, RT_DEFAULT_MAX);

			rtTrace(root, myRay, rayPayload);
			
			//rtPrintf("A\t%u\t%u\t%f\t%f\t%f\n", launchIndex.x, launchIndex.y, ray_direction.x, ray_direction.y, ray_direction.z, rayPayload.rxBufferIndex);
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

				internalRayPayload.prodReflectionCoefficient = rayPayload.prodReflectionCoefficient; 
				internalRayPayload.faceId = rayPayload.faceId;
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
			
			//rtPrintf("Continuing or reflecting ray i.x=%u i.y=%u, reflections=%d hits=%d rd=(%f,%f,%f) origin=(%f,%f,%f) end=%d \n", launchIndex.x, launchIndex.y, rayPayload.reflections, rayPayload.hits, ray_direction.x, ray_direction.y, ray_direction.z, rayPayload.hitPoint.x, rayPayload.hitPoint.y, rayPayload.hitPoint.z, rayPayload.end);
			
			//Reflection info log (to be used in external programs)
			//rtPrintf("R\t%u\t%u\t%d\t%d\t%f\t%f\t%f\t%f\t%f\t%f\t%f\n", launchIndex.x, launchIndex.y,  rayPayload.reflections, rayPayload.hits, ray_direction.x, ray_direction.y, ray_direction.z, rayPayload.hitPoint.x, rayPayload.hitPoint.y, rayPayload.hitPoint.z, rayPayload.totalDistance);
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
rtDeclareVariable(uint, holdReflections, , ); 


rtDeclareVariable(float4, sphere, , );
rtDeclareVariable(optix::Ray, ray_receiver, rtCurrentRay, );




rtDeclareVariable(rtCallableProgramId<float2(float2)>, complex_exp_only_imaginary, , );
rtDeclareVariable(rtCallableProgramId<float2(float, float2)>, sca_complex_prod, , );
rtDeclareVariable(rtCallableProgramId<float2(float2, float2)>, complex_prod, , );

RT_PROGRAM void closestHitReceiverInternalRay()
{

//	rtPrintf("IR hit\t%u\t%u\t%u\t%d\t%d\t%d\n", receiverLaunchIndex.x, receiverLaunchIndex.y, receiverBufferIndex,hitPayload.rxId,receiverBufferIndex );
	//rtPrintf("Hit\t%u\t%u\t%u\torigin=(%f,%f,%f)\t%d\t%d\t%u\n", receiverLaunchIndex.x, receiverLaunchIndex.y, receiverBufferIndex, ray_receiver.origin.x,ray_receiver.origin.y,ray_receiver.origin.z,hitPayload.reflections,hitPayload.rxBufferIndex,ray_receiver.ray_type );
	//Update ray data


	hitPayload.totalDistance += hit_attr.t;
	hitPayload.hitPoint = ray_receiver.origin + hit_attr.t*ray_receiver.direction;
	hitPayload.nextDirection = ray_receiver.direction;
	
	//Check if we are hitting the receiver for this internal ray
	if (hitPayload.rxBufferIndex!=receiverBufferIndex) {
		//rtPrintf("IR not receiver\t%u\t%u\t%u\t%d\t%d\t%d\n", receiverLaunchIndex.x, receiverLaunchIndex.y, receiverBufferIndex,hitPayload.rxBufferIndex,receiverBufferIndex );
		return;
	} else {
		hitPayload.end=true; //We finish the internal ray always once hit the sphere again
	}


	//TODO: We do not check polarization between tx and rx. Can be done comparing payload polarization and receiver polarization

	//Do not end the ray, it can pass through the reception sphere and reflect on a wall, inside or outside the receiver sphere
	//Store for use later
	float prevTd = hitPayload.totalDistance;



	//Check if ray is hitting his own tx (transmitter are also receivers usually) A transmitter cannot receive while it is transmitting, unless other channel is used.
	//const uint transmitterId=receiverLaunchIndex.z; //Internal transmitter id is always 0 for single transmitter
	//	uint2 index = make_uint2(receiverId, transmitterId);
	if (externalId == tx_origin.externalId) {
		//Outgoing ray
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
	//Check for duplicates: keep the ray closest to the center of the receiver

	//		rtBufferId<int, 3>&  min_d_transmitter =bufferMinD [transmitterId];

	if (reflections>0) {
	uint3 idmd = make_uint3(reflections-1, hitPayload.faceId, receiverBufferIndex);

	//Distance from ray line to receiver position
	//Line is defined by ray
	float3 prx = make_float3(sphere.x, sphere.y, sphere.z);
	float3 pd = prx - hitPayload.hitPoint;
	float u = dot(pd, ray_receiver.direction);
	float3 p3 = hitPayload.hitPoint + u*ray_receiver.direction;


	float dm = length(prx - p3)*1000000.0f;  //Multiply by 1000 000 to truncate later take 6 digits
	int dmt = __float2int_rz(dm);   //Truncate
	//int oldd = atomicMin(&bufferMinD[idmd], dmt);
	int oldd = atomicMin(&bufferMinD[idmd], dmt);

	if (oldd < dmt) {
		//our distance is greater,return
		//rtPrintf("FRF\t%u\t%u\t%u\t%f\t%d\t%d\t%f\t%f\t%f\n", receiverLaunchIndex.x, receiverLaunchIndex.y, reflections, dm, dmt, oldd, hitPayload.hitPoint.x, hitPayload.hitPoint.y, hitPayload.hitPoint.z);

		return;
	}
	//rtPrintf("T\t%u\t%u\t%u\t%u\t%f\t%d\t%d\t%f\t%f\t%f\n", receiverLaunchIndex.x, receiverLaunchIndex.y, reflections, hitPayload.faceId,dm, dmt, oldd, hitPayload.hitPoint.x, hitPayload.hitPoint.y, hitPayload.hitPoint.z);


	//rtPrintf("Reflected hit reflections=%d i.x=%u i.y=%u db=%d with idb.x=%u idb.y=%u oldR=%u bit=%u rId=%d \n", hitPayload.reflections, receiverLaunchIndex.x, receiverLaunchIndex.y, db[receiverLaunchIndex], idx.x, idx.y, oldR, bit, receiverBufferIndex);
	atomicAdd(&receptionInfoBuffer[receiverBufferIndex].reflections, 1);


//	float3 ptx = ray_receiver.origin;
//	float d = length(prx - ptx);
	float d=length(prx-hitPayload.lastReflectionHitPoint);
	//Compute electric field
	//rtPrintf("ref totalDistance=%f d=%f reflections=%d i.x=%u i.y=%u \n", hitPayload.totalDistance, d, hitPayload.reflections, receiverLaunchIndex.x, receiverLaunchIndex.y);
//	d += prevTd; //totalDistance
	d+=hitPayload.totalDistanceTillLastReflection;

	float2 z = make_float2(0.0f, -k*d);
	float2 zexp = complex_exp_only_imaginary(z);
	float2 Rzexp = complex_prod(hitPayload.prodReflectionCoefficient, zexp);


	float2 E = sca_complex_prod((hitPayload.electricFieldAmplitude / d), Rzexp);
	/*
	   if (receiverLaunchIndex.x==1118 && receiverLaunchIndex.y==900) {
	   rtPrintf("ref R=(%f,%f) z=(%f,%f) zepx(%f,%f) Rzexp=(%f,%f), E=(%f,%f) i.x=%u i.y=%u \n", hitPayload.prodReflectionCoefficient.x, hitPayload.prodReflectionCoefficient.y, z.x, z.y, zexp.x, zexp.y, Rzexp.x, Rzexp.y, E.x, E.y, receiverLaunchIndex.x, receiverLaunchIndex.y);
	   }
	 */


	//float2 Eprev = bufferMinE[idmd].E;
	//Update min buffer
	//bufferMinE[idmd].E = E;
	//Update min buffer
	//		rtBufferId<DuplicateReflection,3>&  min_e_transmitter =bufferMinE [receiverLaunchIndex.z];
	//		DuplicateReflection dref=min_e_transmitter[idmd];
	//		float*  drx = &(min_e_transmitter[idmd].E.x);
	//		float*  dry = &(min_e_transmitter[idmd].E.y);
	//		float2 Ebuffer=dref.E;
	//		float ebx=Ebuffer.x;
	//		float eby=Ebuffer.y;
	//float*  drx = &Ebuffer.x;
	//float*  dry = &dref.E.y;


	//float*  drx = &bufferMinE[idmd].E.x;
	//float*  dry = &bufferMinE[idmd].E.y;
	//float Eprevx = atomicExch(drx, E.x);
	//float Eprevy = atomicExch(dry, E.y);
	//		float Eprevx = atomicExch(&ebx, E.x);
	//		float Eprevy = atomicExch(&eby, E.y);
	//	rtPrintf("access index=(%u,%u) idmd=(%u,%u,%u) bmi=(%f,%f)\n",index.x,index.y,idmd.x,idmd.y,idmd.z,bufferMinE[transmitterId][idmd].x,bufferMinE[transmitterId][idmd].y);


	float Eprevx = atomicExch(&bufferMinE[idmd].x, E.x);
	float Eprevy = atomicExch(&bufferMinE[idmd].y, E.y);
	float2 Eprev = make_float2(Eprevx, Eprevy);
	//float2 Eprev = make_float2(0.f,0.f);
	//rtPrintf("C\t%u\t%u\t%u\t%u\t%d\t%f\t%f\t%f\t%f\t%f\n", receiverLaunchIndex.x, receiverLaunchIndex.y, reflections, hitPayload.faceId,  dmt,  E.x, E.y, hitPayload.prodReflectionCoefficient.x, hitPayload.prodReflectionCoefficient.y, d);

	//rtPrintf("II\t%u\t%u\t%u\t%u\t%f\t%d\t%d\t%f\t%f\t%f\t%f\t%f\n", receiverLaunchIndex.x, receiverLaunchIndex.y, reflections, hitPayload.faceId, dm, dmt, oldd, E.x, E.y, Eprev.x, Eprev.y, d);
		if (holdReflections==1) {
			return;
		}

	//Remove Electric field from previous minimum distance hit
	E -= Eprev; 

	//Update the receiver
	float oldEx = atomicAdd(&receptionInfoBuffer[receiverBufferIndex].sumRxElectricField.x, E.x);
	float oldEy = atomicAdd(&receptionInfoBuffer[receiverBufferIndex].sumRxElectricField.y, E.y);
	}
	//rtPrintf("IHR. i.x=%u i.y=%u  Reflected hit  reflections=%d Ep=(%f,%f) E=(%f,%f) En=(%f,%f) rId=%d \n", receiverLaunchIndex.x, receiverLaunchIndex.y, reflections, oldEx, oldEy, E.x, E.y, receptionInfoBuffer[receiverBufferIndex].sumRxElectricField.x, receptionInfoBuffer[receiverBufferIndex].sumRxElectricField.y, receiverBufferIndex);

//	rtPrintf("Old E=(%f.%f) New E=(%f,%f) i.x=%u i.y=%u \n", oldx, oldy, receptionInfoBuffer[receiverBufferIndex].sumRxElectricField.x, receptionInfoBuffer[receiverBufferIndex].sumRxElectricField.y, receiverLaunchIndex.x, receiverLaunchIndex.y);
	//rtPrintf("%f\t%f\n", E.x, E.y);
	//Reflected hit info log (to be used in external programs)
	//	rtPrintf("IF\t%u\t%u\t%u\t%u\t%f\t%f\t%f\t%f\t%f\t%f\t%d\n", receiverLaunchIndex.x, receiverLaunchIndex.y, reflections, hitPayload.faceId, oldEx, oldEy, E.x, E.y, receptionInfoBuffer[receiverBufferIndex].sumRxElectricField.x, receptionInfoBuffer[receiverBufferIndex].sumRxElectricField.y, receiverBufferIndex);





}





RT_PROGRAM void closestHitReceiver()
{

	//TODO: We do not check polarization between tx and rx. Can be done comparing payload polarization and receiver polarization

	//Do not end the ray, it can pass through the reception sphere and reflect on a wall, inside or outside the receiver sphere
	//Store for use later
	float prevTd = hitPayload.totalDistance;

	//Update ray data
	hitPayload.totalDistance += hit_attr.t;
	hitPayload.hitPoint = ray_receiver.origin + hit_attr.t*ray_receiver.direction;
	hitPayload.nextDirection = ray_receiver.direction;

	
	//Check if ray is hitting his own tx (transmitter are also receivers usually) A transmitter cannot receive while it is transmitting, unless other channel is used.
	//const uint transmitterId=receiverLaunchIndex.z; //Internal transmitter id is always 0 for single transmitter
//	uint2 index = make_uint2(receiverId, transmitterId);
	if (externalId == tx_origin.externalId) {
		//Outgoing ray
		//rtPrintf("External. txId=%d i.el=%u i.az=%u, ray=(%f,%f,%f) origin=(%f,%f,%f) t=%f rId[%u]=%d\n", tx_origin.externalId, receiverLaunchIndex.x, receiverLaunchIndex.y, ray_receiver.direction.x, ray_receiver.direction.y, ray_receiver.direction.z, ray_receiver.origin.x, ray_receiver.origin.y, ray_receiver.origin.z, hit_attr.t, receiverBufferIndex,externalId);
		//rtPrintf("External. txId=%d i.el=%u i.az=%u, ray=(%f,%f,%f) origin=(%f,%f,%f) t=%f ref=%d\n", tx_origin.externalId, receiverLaunchIndex.x, receiverLaunchIndex.y, ray_receiver.direction.x, ray_receiver.direction.y, ray_receiver.direction.z, ray_receiver.origin.x, ray_receiver.origin.y, ray_receiver.origin.z, hit_attr.t, hitPayload.reflections);
		return;
	}


	int reflections = hitPayload.reflections;
	float3 prx = make_float3(sphere.x, sphere.y, sphere.z);
	float dor=length(ray_receiver.origin-prx);
//	float dtxrx=length(tx_origin.origin-prx);
	//rtPrintf("Rx\t%u\t%u\t%u\torigin=(%f,%f,%f)\t%f\t%u\n", receiverLaunchIndex.x, receiverLaunchIndex.y, receiverBufferIndex, ray_receiver.origin.x,ray_receiver.origin.y,ray_receiver.origin.z,dor,ray_receiver.ray_type );
	if (dor<=(sphere.w+0.0001)) { //Give some epsilon, otherwise numerical inaccuracies may make it fail the check
		//Ray originated inside the reception radius
		
		//if (dtxrx>=sphere.w+0.001) {
			//Transmitter is outside the reception radius: This ray has hit us before, ignore it. Internal ray may compute additional contributions
			//rtPrintf("Ignored. txId=%d i.x=%u i.y=%u, ray=(%f,%f,%f) origin=(%f,%f,%f) t=%f rId[%u]=%d\n", tx_origin.externalId, receiverLaunchIndex.x, receiverLaunchIndex.y, ray_receiver.direction.x, ray_receiver.direction.y, ray_receiver.direction.z, ray_receiver.origin.x, ray_receiver.origin.y, ray_receiver.origin.z, hit_attr.t, receiverBufferIndex,externalId);
			return;
		//} //the transmitter is inside the reception radius, so this ray has not hit us before. No internal ray is created

	} else {
		//If ray origin is outside the reception radius an internal ray is always created 
		//Mark to  trace the internal ray to check if it collides with another thing and hits the receiver again

		hitPayload.rxBufferIndex=receiverBufferIndex;	
	}
	

	if (reflections == 0) {
		
		//This is a direct ray

		int old = atomicAdd(&receptionInfoBuffer[receiverBufferIndex].directHits, 1);
		//rtPrintf("before dh=%d,id=%u tx=%u receptionInfoBuffer[index].directHits=%d  i.x=%u i.y=%u, ray=(%f,%f,%f) t=%f \n", old, index.x, index.y, receptionInfoBuffer[index].directHits, receiverLaunchIndex.x, receiverLaunchIndex.y, ray_receiver.direction.x, ray_receiver.direction.y, ray_receiver.direction.z, hit_attr.t);
		if (old > 0) {
			//rtPrintf("HR. already direct \n");
			//Already has a direct ray counted, just ignore
			return;
		}
		//Compute electric field. For direct rays, the distance is always between tx and rx
		//float3 prx = make_float3(sphere.x, sphere.y, sphere.z);
		float3 ptx = tx_origin.origin;
		float d = length(prx - ptx);
		//rtPrintf("DHd\t%u\t%u\t%f\t%f\t%f\t%f\t%f\t%d\t%d\n", receiverLaunchIndex.x, receiverLaunchIndex.y, sphere.x, sphere.y, sphere.z, sphere.w,d, receiverBufferIndex,externalId);
		//rtPrintf("DR. Direct hit   i.x=%u i.y=%u ray=(%f,%f,%f) prx=(%f,%f,%f) ptx=(%f,%f,%f) rId[%u] \n",  receiverLaunchIndex.x, receiverLaunchIndex.y, ray_receiver.direction.x, ray_receiver.direction.y, ray_receiver.direction.z, prx.x, prx.y, prx.z, ptx.x, ptx.y, ptx.z, receiverBufferIndex);
		//rtPrintf("HR. Ep.x=%f Ep.y=%f Eo=%f\n", receptionInfoBuffer[receiverBufferIndex].sumRxElectricField.x, receptionInfoBuffer[receiverBufferIndex].sumRxElectricField.y, hitPayload.electricFieldAmplitude);
		float2 z = make_float2(0.0f, -k*d);
		//rtPrintf("HR. prx=(%f,%f,%f) k=%f d=%f prevTd=%f hitPayload.totalDistance=%f Eo=%f\n", prx.x, prx.y, prx.z, k, d, prevTd, hitPayload.totalDistance, hitPayload.electricFieldAmplitude);

		float2 zexp = complex_exp_only_imaginary(z);
		float2 E = sca_complex_prod((hitPayload.electricFieldAmplitude / d), zexp);
		

		float oldEx = atomicAdd(&receptionInfoBuffer[receiverBufferIndex].sumRxElectricField.x, E.x);
		float oldEy = atomicAdd(&receptionInfoBuffer[receiverBufferIndex].sumRxElectricField.y, E.y);
		//rtPrintf("DHd\t%u\t%u\t%f\t%f\t%f\t%d\n", receiverLaunchIndex.x, receiverLaunchIndex.y, oldEx, oldEy,d, receiverId);

		//rtPrintf("DR. Direct hit   i.x=%u i.y=%u  Ep=(%f,%f) E=(%f,%f) En=(%f,%f) rId[%u] \n", receiverLaunchIndex.x, receiverLaunchIndex.y, oldEx,oldEy, E.x, E.y, receptionInfoBuffer[receiverBufferIndex].sumRxElectricField.x, receptionInfoBuffer[receiverBufferIndex].sumRxElectricField.y, receiverBufferIndex);
		//Direct hit info log (to be used in external programs)
		//rtPrintf("DH\t%u\t%u\t%f\t%f\t%f\t%f\t%f\t%f\t%d\t%d\n", receiverLaunchIndex.x, receiverLaunchIndex.y,  oldEx, oldEy, E.x, E.y, receptionInfoBuffer[receiverBufferIndex].sumRxElectricField.x, receptionInfoBuffer[receiverBufferIndex].sumRxElectricField.y, receiverBufferIndex,externalId);

		//receptionInfoBuffer[receiverBufferIndex].hasDirectContribution = true;
		//rtPrintf("%f\t%f\n", E.x, E.y);
	}
	else {

		//Reflected ray
		//Check for duplicates: keep the ray closest to the center of the receiver
		
//		rtBufferId<int, 3>&  min_d_transmitter =bufferMinD [transmitterId];
			
		uint3 idmd = make_uint3(reflections-1, hitPayload.faceId, receiverBufferIndex);
		
		//Distance from ray line to receiver position
		//Line is defined by ray
		float3 prx = make_float3(sphere.x, sphere.y, sphere.z);
		float3 pd = prx - hitPayload.hitPoint;
		float u = dot(pd, ray_receiver.direction);
		float3 p3 = hitPayload.hitPoint + u*ray_receiver.direction;


		float dm = length(prx - p3)*1000000.0f;  //Multiply by 1000 000 to truncate later take 6 digits
		int dmt = __float2int_rz(dm);   //Truncate
		//int oldd = atomicMin(&bufferMinD[idmd], dmt);
		int oldd = atomicMin(&bufferMinD[idmd], dmt);
		
		if (oldd < dmt) {
			//our distance is greater,return
			//rtPrintf("FRF\t%u\t%u\t%u\t%f\t%d\t%d\t%f\t%f\t%f\n", receiverLaunchIndex.x, receiverLaunchIndex.y, reflections, dm, dmt, oldd, hitPayload.hitPoint.x, hitPayload.hitPoint.y, hitPayload.hitPoint.z);
			
			return;
		}
		//rtPrintf("T\t%u\t%u\t%u\t%u\t%f\t%d\t%d\t%f\t%f\t%f\n", receiverLaunchIndex.x, receiverLaunchIndex.y, reflections, hitPayload.faceId,dm, dmt, oldd, hitPayload.hitPoint.x, hitPayload.hitPoint.y, hitPayload.hitPoint.z);


		//rtPrintf("Reflected hit reflections=%d i.x=%u i.y=%u db=%d with idb.x=%u idb.y=%u oldR=%u bit=%u rId=%d \n", hitPayload.reflections, receiverLaunchIndex.x, receiverLaunchIndex.y, db[receiverLaunchIndex], idx.x, idx.y, oldR, bit, receiverBufferIndex);
		atomicAdd(&receptionInfoBuffer[receiverBufferIndex].reflections, 1);

		
		//float3 ptx = ray_receiver.origin;
		//float d = length(prx - ptx);
		float d=length(prx-hitPayload.lastReflectionHitPoint);
		//Compute electric field
		//rtPrintf("ref totalDistance=%f d=%f reflections=%d i.x=%u i.y=%u \n", hitPayload.totalDistance, d, hitPayload.reflections, receiverLaunchIndex.x, receiverLaunchIndex.y);
		//d += prevTd; //totalDistance
		d+=hitPayload.totalDistanceTillLastReflection;
		float2 z = make_float2(0.0f, -k*d);
		float2 zexp = complex_exp_only_imaginary(z);
		float2 Rzexp = complex_prod(hitPayload.prodReflectionCoefficient, zexp);


		float2 E = sca_complex_prod((hitPayload.electricFieldAmplitude / d), Rzexp);
		/*
		if (receiverLaunchIndex.x==1118 && receiverLaunchIndex.y==900) {
			rtPrintf("ref R=(%f,%f) z=(%f,%f) zepx(%f,%f) Rzexp=(%f,%f), E=(%f,%f) i.x=%u i.y=%u \n", hitPayload.prodReflectionCoefficient.x, hitPayload.prodReflectionCoefficient.y, z.x, z.y, zexp.x, zexp.y, Rzexp.x, Rzexp.y, E.x, E.y, receiverLaunchIndex.x, receiverLaunchIndex.y);
		}
		*/
		
		
		//float2 Eprev = bufferMinE[idmd].E;
		//Update min buffer
		//bufferMinE[idmd].E = E;
		//Update min buffer
//		rtBufferId<DuplicateReflection,3>&  min_e_transmitter =bufferMinE [receiverLaunchIndex.z];
//		DuplicateReflection dref=min_e_transmitter[idmd];
//		float*  drx = &(min_e_transmitter[idmd].E.x);
//		float*  dry = &(min_e_transmitter[idmd].E.y);
//		float2 Ebuffer=dref.E;
//		float ebx=Ebuffer.x;
//		float eby=Ebuffer.y;
		//float*  drx = &Ebuffer.x;
		//float*  dry = &dref.E.y;
		
		
		//float*  drx = &bufferMinE[idmd].E.x;
		//float*  dry = &bufferMinE[idmd].E.y;
		//float Eprevx = atomicExch(drx, E.x);
		//float Eprevy = atomicExch(dry, E.y);
//		float Eprevx = atomicExch(&ebx, E.x);
//		float Eprevy = atomicExch(&eby, E.y);
	//	rtPrintf("access index=(%u,%u) idmd=(%u,%u,%u) bmi=(%f,%f)\n",index.x,index.y,idmd.x,idmd.y,idmd.z,bufferMinE[transmitterId][idmd].x,bufferMinE[transmitterId][idmd].y);
		
			
		float Eprevx = atomicExch(&bufferMinE[idmd].x, E.x);
		float Eprevy = atomicExch(&bufferMinE[idmd].y, E.y);
		float2 Eprev = make_float2(Eprevx, Eprevy);
		//rtPrintf("C\t%u\t%u\t%u\t%u\t%d\t%f\t%f\t%f\t%f\t%f\n", receiverLaunchIndex.x, receiverLaunchIndex.y, reflections, hitPayload.faceId,  dmt,  E.x, E.y, hitPayload.prodReflectionCoefficient.x, hitPayload.prodReflectionCoefficient.y, d);

		//rtPrintf("FF\t%u\t%u\t%u\t%u\t%f\t%d\t%d\t%f\t%f\t%f\t%f\t%f\n", receiverLaunchIndex.x, receiverLaunchIndex.y, reflections, hitPayload.faceId, dm, dmt, oldd, E.x, E.y, Eprev.x, Eprev.y, d);
		if (holdReflections==1) {
			return;
		}

		//Remove Electric field from previous minimum distance hit
		E -= Eprev; 

		//Update the receiver
		float oldEx = atomicAdd(&receptionInfoBuffer[receiverBufferIndex].sumRxElectricField.x, E.x);
		float oldEy = atomicAdd(&receptionInfoBuffer[receiverBufferIndex].sumRxElectricField.y, E.y);
		//rtPrintf("HR. i.x=%u i.y=%u  Reflected hit  reflections=%d Ep=(%f,%f) E=(%f,%f) En=(%f,%f) rId=%d \n", receiverLaunchIndex.x, receiverLaunchIndex.y, reflections, oldEx, oldEy, E.x, E.y, receptionInfoBuffer[receiverBufferIndex].sumRxElectricField.x, receptionInfoBuffer[receiverBufferIndex].sumRxElectricField.y, receiverBufferIndex);

		//rtPrintf("Old E=(%f.%f) New E=(%f,%f) i.x=%u i.y=%u \n", oldx, oldy, receptionInfoBuffer[receiverBufferIndex].sumRxElectricField.x, receptionInfoBuffer[receiverBufferIndex].sumRxElectricField.y, receiverLaunchIndex.x, receiverLaunchIndex.y);
		//rtPrintf("%f\t%f\n", E.x, E.y);
		//Reflected hit info log (to be used in external programs)
		//rtPrintf("F\t%u\t%u\t%u\t%u\t%f\t%f\t%f\t%f\t%f\t%f\t%d\n", receiverLaunchIndex.x, receiverLaunchIndex.y, reflections, hitPayload.faceId, oldEx, oldEy, E.x, E.y, receptionInfoBuffer[receiverBufferIndex].sumRxElectricField.x, receptionInfoBuffer[receiverBufferIndex].sumRxElectricField.y, receiverBufferIndex);


	}

}







//Mainly for debug purposes: The electric field values of all reflected hits are kept in a buffer and can be summed after launch.


rtDeclareVariable(EMWavePayload, missPayload, rtPayload, );
RT_PROGRAM void miss()
{
	//rtPrintf("miss i.x=%u. iy=%u \n", receiverLaunchIndex.x, receiverLaunchIndex.y);
	missPayload.end = true;
}

