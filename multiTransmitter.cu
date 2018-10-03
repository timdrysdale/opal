

#include "Common.h"
#include <optix_world.h>
#include <optixu/optixu_math_namespace.h>
#include <optixu/optixu_aabb_namespace.h>
using namespace optix;



//Ray Sphere buffer
rtBuffer<float3> raySphere;
rtBuffer<float3, 2> raySphere2D;

rtBuffer<Transmitter,1> tx_origin;


//Receiver buffers



//TODO: these buffers consume a lot of memory if there are many receivers. This could be replaced by recursive tracing inside the sphere or any other solution that should be tried
rtBuffer<rtBufferId<int, 3>, 1> internalRaysBuffer; //Filter internal rays buffer  [[elevationSteps, azimuthSteps, receiver],transmitter]

rtBuffer<rtBufferId<int, 3>,1> bufferMinD; //minDistance ray to receiver [[reflections, faceId,receiver],transmitter];

//TODO: using rtBufferId and float2 plus the AtomicExch() seems to cause problems: segmentation faults from device and WARNING: Raw pointer access. Separating into two buffers seems to solve it
rtBuffer<rtBufferId<float, 3>,1> bufferMinEx; //Buffer with the real part of the Electric field of the minimum distance ray [[reflections, faceId,receiver], transmitter];
rtBuffer<rtBufferId<float, 3>,1> bufferMinEy; //Buffer with the imaginary part of the Electric field of the minimum distance ray [[reflections, faceId,receiver], transmitter];


rtBuffer<ReceptionInfo, 2> receptionInfoBuffer; //Results buffer [receiver,transmitter]



//rtDeclareVariable(uint2, duplicateBlockSize, , );

rtDeclareVariable(uint3, launchIndex, rtLaunchIndex, );

rtDeclareVariable(rtObject, staticMeshes, , );
rtDeclareVariable(rtObject, root, , );
rtDeclareVariable(float, min_t_epsilon, , );
rtDeclareVariable(unsigned int, max_interactions, , );
rtDeclareVariable(uint2, tx_rx, , ); //[transmitters, receivers]
rtDeclareVariable(unsigned int, number_of_faces, , );

rtDeclareVariable(uint2, raySphereSize, , );
//rtDeclareVariable(uint, initialize, , );


RT_PROGRAM void initializeBuffersFaceBased() {
	//rtPrintf("Initializing reception buff  tx_rx=(%u,%u) index=(%u,%u,%u)\n", tx_rx.x, tx_rx.y, launchIndex.x, launchIndex.y, launchIndex.z);


	//TODO: all done by index=(0,0,0) for a high number of faces and receivers it takes a lot of time. Parallelize it
/*	if (launchIndex == make_uint3(0, 0, 0)) {
		
		for (unsigned int i = 0; i < tx_rx.y; ++i)
		{
			
			for (unsigned int j = 0; j < tx_rx.x; ++j)
			{
				uint2 index = make_uint2(i, j);
				//rtPrintf("Initializing reception buff  index=(%u,%u)\n", index.x, index.y);
				//rtPrintf("Initializing reception buff  Ep0[%u,%u]=(%f,%f)  \n", index.x, index.y, receptionInfoBuffer[index].sumRxElectricField.x, receptionInfoBuffer[index].sumRxElectricField.y);
				receptionInfoBuffer[index].sumRxElectricField = make_float2(0.0f, 0.0f);
				receptionInfoBuffer[index].directHits = 0;
				receptionInfoBuffer[index].reflections = 0;
				//rtPrintf("Initializing reception buff  Ep0[%u,%u]=(%f,%f)  \n", index.x, index.y, receptionInfoBuffer[index].sumRxElectricField.x, receptionInfoBuffer[index].sumRxElectricField.y);



			}
			for (unsigned int k = 0; k < number_of_faces; ++k)
			{
				//uint2 idf = make_uint2(k, i);
			//	facesBuffer[idf] = 0u;
				for (unsigned int  l = 0; l < max_interactions; ++l) 
				{
					
					uint3 idmd = make_uint3(l, k, i);
					bufferMinD[idmd] = 2147483647;
					bufferMinE[idmd].E = make_float2(0.0f, 0.0f);
					bufferMinE[idmd].r = 0;
					//rtPrintf("Initializing faces buff idmd=(%u,%u,%u)=%d \n", idmd.x,idmd.y,idmd.z, bufferMinD[idmd]);

					
				}
			}

		}
	}
	*/
	
	//One per ray and for all receivers 

	unsigned int transmitter=launchIndex.z;

	rtBufferId<int, 3>& ib = internalRaysBuffer[transmitter];
	for (unsigned int r=0; r<tx_rx.y; ++r) {
		uint3 irBufferId=make_uint3(launchIndex.x,launchIndex.y,r);
		ib[irBufferId] = -1;
		//rtPrintf("tx=%u ib[%u,%u,%u]=%d \n", transmitter, launchIndex.x, launchIndex.y, r,  ib[irBufferId]);
		//Only once per transmitter, does not depend on elevation or azimut	

		if (launchIndex.x==0 && launchIndex.y==0) {
			uint2 index = make_uint2(r, transmitter); 
			//rtPrintf("Initializing reception buff  index=(%u,%u)\n", index.x, index.y);
			//rtPrintf("Initializing reception buff  Ep0[%u,%u]=(%f,%f)  \n", index.x, index.y, receptionInfoBuffer[index].sumRxElectricField.x, receptionInfoBuffer[index].sumRxElectricField.y);
			receptionInfoBuffer[index].sumRxElectricField = make_float2(0.0f, 0.0f);
			receptionInfoBuffer[index].directHits = 0;
			receptionInfoBuffer[index].reflections = 0;
			//rtPrintf("Initializing reception buff  Ep0[%u,%u]=(%f,%f)  \n", index.x, index.y, receptionInfoBuffer[index].sumRxElectricField.x, receptionInfoBuffer[index].sumRxElectricField.y);
			rtBufferId<int, 3>& ifm =bufferMinD[transmitter];
			//rtBufferId<DuplicateReflection, 3>& ife=bufferMinE[transmitter]; 
			rtBufferId<float, 3>& ifex=bufferMinEx[transmitter]; 
			rtBufferId<float, 3>& ifey=bufferMinEy[transmitter]; 
			for (unsigned int k = 0; k < number_of_faces; ++k)
			{
				//uint2 idf = make_uint2(k, i);
				//	facesBuffer[idf] = 0u;
				for (unsigned int  l = 0; l < max_interactions; ++l) 
				{
					//		for (unsigned int r=0; r<tx_rx.y; ++r) {
					uint3 idmd = make_uint3(l, k, r);
					ifm[idmd] = 2147483647;
					ifex[idmd] = 0.0f;
					ifey[idmd] = 0.0f;
					//ife[idmd] = make_float2(0.0f, 0.0f);
					//ife[idmd].E = make_float2(0.0f, 0.0f);
					//ife[idmd].r = 0;
					//rtPrintf("Initializing faces ifm buff idmd=(%u,%u,%u)%u=%d \n", idmd.x,idmd.y,idmd.z,transmitter, ifm[idmd]);
					//rtPrintf("Initializing faces ife buff idmd=(%u,%u,%u)%u=%d \n", idmd.x,idmd.y,idmd.z,transmitter, ife[idmd].r);
					//		}

				}
			}
		}
	}
	
	
	



	//Each launch initializes its element in the receivers buffer
	//rtPrintf("Initializing buffers tx_rx=(%u,%u)\n", tx_rx.x, tx_rx.y);


/*	for (unsigned int i = 0; i < tx_rx.y; ++i)
	{
		
		rtBufferId<int, 3>& ib = internalRaysBuffer[i];
		ib[launchIndex] = -1;
		//rtPrintf("i=%u db[%u,%u,%u]=%u ib=%d \n", i, launchIndex.x, launchIndex.y, launchIndex.z, ib[launchIndex]);

	}
*/
}
RT_PROGRAM void genRayAndReflectionsFromSphereIndex()
{

	

		uint2 idx = make_uint2(launchIndex.x, launchIndex.y); //[elevation, azimuth]
		//index goes from 0 to raySphereSize.x-1 //The last elevation step corresponds to 180 degrees elevation
		if ((idx.x == 0 ||idx.x==  raySphereSize.x-1  ) && idx.y != 0) {
			//These rays are all the same (0,1,0) or (0,-1,0). Only trace  (0,0) and (last,0) corresponding to 0 and 180 elevation degrees
			return;
		}
		float3 origin = tx_origin[launchIndex.z].origin;
		float3 ray_direction = raySphere2D[idx];

		EMWavePayload rayPayload;
		rayPayload.geomNormal = optix::make_float3(0, 0, 0);
		rayPayload.nextDirection = optix::make_float3(0, 0, 0);
		rayPayload.hitPoint = origin;
		rayPayload.polarization = tx_origin[launchIndex.z].polarization;
		rayPayload.electricFieldAmplitude = 1.0f; //Normalized Eo=1. Antenna Gain = 1. Implement antenna gain with antennaBuffer dependent on the ray direction and txId(launchIndex) : initialEFAmplitude[launchIndex.z] * antennaGain[launchIndex]);
		rayPayload.t = -1.0f;
		rayPayload.reflections = 0;

		rayPayload.hits = 0;
		rayPayload.totalDistance = 0.0f;
		rayPayload.end = false;

		rayPayload.prodReflectionCoefficient = make_float2(1.0f, 0.0f);
		rayPayload.faceId = 0u;



		//rtPrintf("Lsphere2D el=%u az=%u tx=%u ray=(%f,%f,%f)\n", launchIndex.x, launchIndex.y, launchIndex.z,ray_direction.x, ray_direction.y, ray_direction.z );


		// Each iteration is a segment (due to reflections) of the ray path.  The closest hit will
		// return new segments to be traced here. Additionally, the closest hit at receiver will generate another ray to continue the propagation
		int i = 0;
		//int lastReflectionNumber = 0;
		//rtPrintf("Generating ray i.x=%u i.y=%u, ray=(%f,%f,%f) inter=%d end=%d \n", launchIndex.x, launchIndex.y, ray_direction.x, ray_direction.y, ray_direction.z, rayPayload.reflections, rayPayload.end);
		//rtPrintf("A\t%u\t%u\t%f\t%f\t%f\n", launchIndex.x, launchIndex.y, ray_direction.x, ray_direction.y, ray_direction.z);
		
		while (true) {
			optix::Ray myRay(origin, ray_direction, 0, min_t_epsilon, RT_DEFAULT_MAX);

			rtTrace(root, myRay, rayPayload);
			i++;
			//Miss or too much attenuation
			if (rayPayload.end) {
				break;
			}
			//Max number of reflections
			if (rayPayload.reflections > max_interactions) {
				break;
			}
			//Hit one or more receivers and no meshes
			/*if (rayPayload.reflections == 0 && rayPayload.hits > 0) {
				//Continue the ray
				origin = rayPayload.hitPoint;
				ray_direction = rayPayload.nextDirection;
				rtPrintf("1 Continuing ray i.x=%u i.y=%u, reflections=%d hits=%d rd=(%f,%f,%f) origin=(%f,%f,%f) end=%d \n", launchIndex.x, launchIndex.y, rayPayload.reflections, rayPayload.hits, ray_direction.x, ray_direction.y, ray_direction.z, rayPayload.hitPoint.x, rayPayload.hitPoint.y, rayPayload.hitPoint.z, rayPayload.end);
				continue;
			}
			//No additional reflection in the last trace, it may have hit a receiver, otherwise it would have been killed by miss program. Now kill
			if (rayPayload.reflections == lastReflectionNumber  && rayPayload.hits > 0) {
				//Continue the ray
				origin = rayPayload.hitPoint;
				ray_direction = rayPayload.nextDirection;
				rtPrintf("2 Continuing ray i.x=%u i.y=%u, reflections=%d hits=%d rd=(%f,%f,%f) origin=(%f,%f,%f) end=%d \n", launchIndex.x, launchIndex.y, rayPayload.reflections, rayPayload.hits, ray_direction.x, ray_direction.y, ray_direction.z, rayPayload.hitPoint.x, rayPayload.hitPoint.y, rayPayload.hitPoint.z, rayPayload.end);

				continue;

			}*/
			//Reflection or going through receiver
			//lastReflectionNumber = rayPayload.reflections;
			// Update ray data for the next path segment
			ray_direction = rayPayload.nextDirection;
			origin = rayPayload.hitPoint;
			//rtPrintf("Continuing or reflecting ray i.x=%u i.y=%u, reflections=%d hits=%d rd=(%f,%f,%f) origin=(%f,%f,%f) end=%d \n", launchIndex.x, launchIndex.y, rayPayload.reflections, rayPayload.hits, ray_direction.x, ray_direction.y, ray_direction.z, rayPayload.hitPoint.x, rayPayload.hitPoint.y, rayPayload.hitPoint.z, rayPayload.end);
			
			//Reflection info log (to be used in external programs)
			//rtPrintf("R\t%u\t%u\t%u\t%d\t%d\t%f\t%f\t%f\t%f\t%f\t%f\t%f\n", launchIndex.x, launchIndex.y, launchIndex.z, rayPayload.reflections, rayPayload.hits, ray_direction.x, ray_direction.y, ray_direction.z, rayPayload.hitPoint.x, rayPayload.hitPoint.y, rayPayload.hitPoint.z, rayPayload.totalDistance);

			//if (rayPayload.reflections == 0) {
				//rtPrintf("Generating reflection with 0 reflection i.x=%u i.y=%u, inter=%d end=%d \n", launchIndex.x, launchIndex.y, rayPayload.reflections, rayPayload.end);
				//break;
			//}
			//rtPrintf("Reflecting ray i.x=%u i.y=%u, inter=%d hits=%d rd=(%f,%f,%f) origin=(%f,%f,%f) end=%d \n", launchIndex.x, launchIndex.y, rayPayload.reflections, rayPayload.hits, rayPayload.reflectionDirection.x, rayPayload.reflectionDirection.y, rayPayload.reflectionDirection.z, rayPayload.hitPoint.x, rayPayload.hitPoint.y, rayPayload.hitPoint.z, rayPayload.end);
		}

	
}









//rtDeclareVariable(TriangleHit, hit_attr, attribute hit_attr, );



rtDeclareVariable(SphereHit, hit_attr, attribute hit_attr, );
rtDeclareVariable(EMWavePayload, hitPayload, rtPayload, );
rtDeclareVariable(float, k, , );
rtDeclareVariable(uint3, receiverLaunchIndex, rtLaunchIndex, );

rtDeclareVariable(uint, receiverId, , ); //Buffer id
rtDeclareVariable(uint, externalId, , ); //External id  used to identify receivers 


rtDeclareVariable(float4, sphere, , );
rtDeclareVariable(optix::Ray, ray_receiver, rtCurrentRay, );

//rtDeclareVariable(int, duplicateStep, , );



rtDeclareVariable(rtCallableProgramId<float2(float2)>, complex_exp_only_imaginary, , );
rtDeclareVariable(rtCallableProgramId<float2(float, float2)>, sca_complex_prod, , );
rtDeclareVariable(rtCallableProgramId<float2(float2, float2)>, complex_prod, , );



RT_PROGRAM void closestHitReceiverFaceMin()
{

	//TODO: We do not check polarization between tx and rx. Can be done comparing payload polarization and receiver polarization

	//Do not end the ray, it can pass the reception sphere and reflect on a wall, inside or outside the receiver sphere
	//Store for use later
	float prevTd = hitPayload.totalDistance;

	//Update ray data
	hitPayload.totalDistance += hit_attr.t;
	hitPayload.hitPoint = ray_receiver.origin + hit_attr.t*ray_receiver.direction;
	hitPayload.nextDirection = ray_receiver.direction;

	
	//Check if ray is hitting his own tx (transmitter are also receivers usually) A transmitter cannot receive while it is transmitting, unless other channel is used.
	const uint transmitterId=receiverLaunchIndex.z;
	uint2 index = make_uint2(receiverId, transmitterId);
	if (externalId == tx_origin[transmitterId].externalId) {
		//Outgoing ray
		rtPrintf("External. id=%u tx=%u i.x=%u i.y=%u, ray=(%f,%f,%f) origin=(%f,%f,%f) t=%f rId=%d\n", index.x, index.y, receiverLaunchIndex.x, receiverLaunchIndex.y, ray_receiver.direction.x, ray_receiver.direction.y, ray_receiver.direction.z, ray_receiver.origin.x, ray_receiver.origin.y, ray_receiver.origin.z, hit_attr.t, receiverId);

		return;
	}


	//uint2 index = make_uint2(receiverId, receiverLaunchIndex.z);//Index is given by transmitter and receiver ids
	int reflections = hitPayload.reflections;

	//Check incoming or outgoing ray
	//rtBufferId<int, 3>& ib = internalRaysBuffer[receiverId];
	const rtBufferId<int, 3>& ib = internalRaysBuffer[transmitterId];
	uint3 irBufferId=make_uint3(receiverLaunchIndex.x,receiverLaunchIndex.y,receiverId);
	int prevRef = ib[irBufferId];
	if (prevRef < 0) {
		//Incoming ray. Store number of reflections
		ib[irBufferId] = reflections;
		++hitPayload.hits;
		
		//rtPrintf("IR\t%u\t%u\t%u\t%d\t%d\t%d\n", receiverLaunchIndex.x, receiverLaunchIndex.y, receiverLaunchIndex.z, prevRef, reflections, ib[receiverLaunchIndex]);
		
	}
	else {
		//Outgoing ray. Check reflections
		//Reinit the buffer
		ib[irBufferId] = -1;
		

		
		if (prevRef == reflections) {
			//Ray has not been reflected within the receiver sphere, ignore it
			//rtPrintf("  rId=%d----> outgoing ray \n", receiverId);
			
			//rtPrintf("OR\t%u\t%u\t%u\t%d\t%d\n", receiverLaunchIndex.x, receiverLaunchIndex.y, receiverLaunchIndex.z, prevRef, reflections);
			
			return;

		}
	}
	//float r = hit_attr.t; //Distance to origin
	//rtPrintf("HR. rx=(%f,%f,%f) radius=%f id=%u tx=%u\n", sphere.x,sphere.y,sphere.z, sphere.w, index.x, index.y);
	//rtPrintf("HR. HitPayload inte=%d \n", hitPayload.reflections);



	if (reflections == 0) {
		
		//This is a direct ray

		int old = atomicAdd(&receptionInfoBuffer[index].directHits, 1);
		//rtPrintf("before dh=%d,id=%u tx=%u receptionInfoBuffer[index].directHits=%d  i.x=%u i.y=%u, ray=(%f,%f,%f) t=%f \n", old, index.x, index.y, receptionInfoBuffer[index].directHits, receiverLaunchIndex.x, receiverLaunchIndex.y, ray_receiver.direction.x, ray_receiver.direction.y, ray_receiver.direction.z, hit_attr.t);
		if (old > 0) {
			//rtPrintf("HR. already direct \n");
			//Already has a direct ray counted, just ignore
			return;
		}
		//Compute electric field. For direct rays, the distance is always between tx and rx
		float3 prx = make_float3(sphere.x, sphere.y, sphere.z);
		float3 ptx = tx_origin[transmitterId].origin;
		float d = length(prx - ptx);
		//rtPrintf("DHd\t%u\t%u\t%u\t%f\t%f\t%f\t%f\t%f\t%d\t%d\n", receiverLaunchIndex.x, receiverLaunchIndex.y, receiverLaunchIndex.z, sphere.x, sphere.y, sphere.z, sphere.w,d, receiverId,externalId);
		//rtPrintf("DR. Direct hit   i.x=%u i.y=%u ray=(%f,%f,%f) prx=(%f,%f,%f) ptx=(%f,%f,%f) rId=%d \n",  receiverLaunchIndex.x, receiverLaunchIndex.y, ray_receiver.direction.x, ray_receiver.direction.y, ray_receiver.direction.z, prx.x, prx.y, prx.z, ptx.x, ptx.y, ptx.z, receiverId);
		//rtPrintf("HR. Ep.x=%f Ep.y=%f Eo=%f\n", receptionInfoBuffer[index].sumRxElectricField.x, receptionInfoBuffer[index].sumRxElectricField.y, hitPayload.electricFieldAmplitude);
		float2 z = make_float2(0.0f, -k*d);
		//rtPrintf("HR. prx=(%f,%f,%f) k=%f d=%f prevTd=%f hitPayload.totalDistance=%f Eo=%f\n", prx.x, prx.y, prx.z, k, d, prevTd, hitPayload.totalDistance, hitPayload.electricFieldAmplitude);

		float2 zexp = complex_exp_only_imaginary(z);
		float2 E = sca_complex_prod((hitPayload.electricFieldAmplitude / d), zexp);

		float oldEx = atomicAdd(&receptionInfoBuffer[index].sumRxElectricField.x, E.x);
		float oldEy = atomicAdd(&receptionInfoBuffer[index].sumRxElectricField.y, E.y);
		//rtPrintf("DHd\t%u\t%u\t%u\t%f\t%f\t%f\t%d\n", receiverLaunchIndex.x, receiverLaunchIndex.y, receiverLaunchIndex.z, oldEx, oldEy,d, receiverId);

		//rtPrintf("DR. Direct hit   i.x=%u i.y=%u  Ep=(%f,%f) E=(%f,%f) En=(%f,%f) rId=%d \n", receiverLaunchIndex.x, receiverLaunchIndex.y, oldEx,oldEy, E.x, E.y, receptionInfoBuffer[index].sumRxElectricField.x, receptionInfoBuffer[index].sumRxElectricField.y, receiverId);
		//Direct hit info log (to be used in external programs)
		//rtPrintf("DH\t%u\t%u\t%u\t%f\t%f\t%f\t%f\t%f\t%f\t%d\t%d\n", receiverLaunchIndex.x, receiverLaunchIndex.y, receiverLaunchIndex.z, oldEx, oldEy, E.x, E.y, receptionInfoBuffer[index].sumRxElectricField.x, receptionInfoBuffer[index].sumRxElectricField.y, receiverId,externalId);

		//receptionInfoBuffer[index].hasDirectContribution = true;
		//rtPrintf("%f\t%f\n", E.x, E.y);
	}
	else {

		//Reflected ray
		//Check for duplicates: keep the ray closest to the center of the receiver
		
		rtBufferId<int, 3>&  min_d_transmitter =bufferMinD [transmitterId];
			
		
		uint3 idmd = make_uint3(reflections-1, hitPayload.faceId, receiverId);
		
		//Distance from ray line to receiver position
		//Line is defined by ray
		float3 prx = make_float3(sphere.x, sphere.y, sphere.z);
		float3 pd = prx - hitPayload.hitPoint;
		float u = dot(pd, ray_receiver.direction);
		float3 p3 = hitPayload.hitPoint + u*ray_receiver.direction;


		float dm = length(prx - p3)*1000000.0f;  //Multiply by 1000 000 to truncate later take 6 digits
		int dmt = __float2int_rz(dm);   //Truncate
		int oldd = atomicMin(&bufferMinD[transmitterId][idmd], dmt);
		//int oldd = atomicMin(&min_d_transmitter[idmd], dmt);
		
		if (oldd < dmt) {
			//our distance is greater,return
			//rtPrintf("FRF\t%u\t%u\t%u\t%f\t%d\t%d\t%f\t%f\t%f\n", receiverLaunchIndex.x, receiverLaunchIndex.y, reflections, dm, dmt, oldd, hitPayload.hitPoint.x, hitPayload.hitPoint.y, hitPayload.hitPoint.z);
			
			return;
		}
		rtPrintf("T\t%u\t%u\t%u\t%u\t%f\t%d\t%d\t%f\t%f\t%f\n", receiverLaunchIndex.x, receiverLaunchIndex.y, reflections, hitPayload.faceId,dm, dmt, oldd, hitPayload.hitPoint.x, hitPayload.hitPoint.y, hitPayload.hitPoint.z);


		//rtPrintf("Reflected hit reflections=%d i.x=%u i.y=%u db=%d with idb.x=%u idb.y=%u oldR=%u bit=%u rId=%d \n", hitPayload.reflections, receiverLaunchIndex.x, receiverLaunchIndex.y, db[receiverLaunchIndex], idx.x, idx.y, oldR, bit, receiverId);
		atomicAdd(&receptionInfoBuffer[index].reflections, 1);

		
		float3 ptx = ray_receiver.origin;
		float d = length(prx - ptx);
		//Compute electric field
		//rtPrintf("ref totalDistance=%f d=%f reflections=%d i.x=%u i.y=%u \n", hitPayload.totalDistance, d, hitPayload.reflections, receiverLaunchIndex.x, receiverLaunchIndex.y);
		d += prevTd; //totalDistance

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
		//rtBufferId<float2,3>&  min_e_transmitter =bufferMinE [transmitterId];
		//float*  drx = &(min_e_transmitter[idmd].x);
		//float*  dry = &(min_e_transmitter[idmd].y);
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
		
		//float Eprevx = atomicExch(&min_e_transmitter[idmd].x, E.x);
		//float Eprevy = atomicExch(&min_e_transmitter[idmd].y, E.y);
		float Eprevx = atomicExch(&bufferMinEx[transmitterId][idmd], E.x);
		float Eprevy = atomicExch(&bufferMinEy[transmitterId][idmd], E.y);
		float2 Eprev = make_float2(Eprevx, Eprevy);
		//float2 Eprev = make_float2(Eprevx,0.f);
		//rtPrintf("C\t%u\t%u\t%u\t%u\t%d\t%f\t%f\t%f\t%f\t%f\n", receiverLaunchIndex.x, receiverLaunchIndex.y, reflections, hitPayload.faceId,  dmt,  E.x, E.y, hitPayload.prodReflectionCoefficient.x, hitPayload.prodReflectionCoefficient.y, d);

		rtPrintf("FF\t%u\t%u\t%u\t%u\t%f\t%d\t%d\t%f\t%f\t%f\t%f\t%f\n", receiverLaunchIndex.x, receiverLaunchIndex.y, reflections, hitPayload.faceId, dm, dmt, oldd, E.x, E.y, Eprev.x, Eprev.y, d);

		//Remove Electric field from previous minimum distance hit
		E -= Eprev; 

		//Update the receiver
		float oldEx = atomicAdd(&receptionInfoBuffer[index].sumRxElectricField.x, E.x);
		float oldEy = atomicAdd(&receptionInfoBuffer[index].sumRxElectricField.y, E.y);
		//rtPrintf("HR. i.x=%u i.y=%u  Reflected hit  reflections=%d Ep=(%f,%f) E=(%f,%f) En=(%f,%f) rId=%d \n", receiverLaunchIndex.x, receiverLaunchIndex.y, reflections, oldEx, oldEy, E.x, E.y, receptionInfoBuffer[index].sumRxElectricField.x, receptionInfoBuffer[index].sumRxElectricField.y, receiverId);

		//rtPrintf("Old E=(%f.%f) New E=(%f,%f) i.x=%u i.y=%u \n", oldx, oldy, receptionInfoBuffer[index].sumRxElectricField.x, receptionInfoBuffer[index].sumRxElectricField.y, receiverLaunchIndex.x, receiverLaunchIndex.y);
		//rtPrintf("%f\t%f\n", E.x, E.y);
		//Reflected hit info log (to be used in external programs)
	//	rtPrintf("F\t%u\t%u\t%u\t%u\t%f\t%f\t%f\t%f\t%f\t%f\t%d\t%u\n", receiverLaunchIndex.x, receiverLaunchIndex.y, reflections, hitPayload.faceId, oldEx, oldEy, E.x, E.y, receptionInfoBuffer[index].sumRxElectricField.x, receptionInfoBuffer[index].sumRxElectricField.y, receiverId, transmitterId);



	}

}



//Mainly for debug purposes: The electric field values of all reflected hits are kept in a buffer and can be summed after launch.
RT_PROGRAM void closestHitReceiverFaceMinHoldReflections()
{

	//TODO: We do not check polarization between tx and rx. Can be done comparing payload polarization and receiver polarization

	//Do not end the ray, it can pass the reception sphere and reflect on a wall, inside or outside the receiver sphere
	//Store for use later
	float prevTd = hitPayload.totalDistance;

	//Update ray data
	hitPayload.totalDistance += hit_attr.t;
	hitPayload.hitPoint = ray_receiver.origin + hit_attr.t*ray_receiver.direction;
	hitPayload.nextDirection = ray_receiver.direction;

	//Check if ray is hitting his own tx (transmitter are also receiver usually) A transmitter cannot receive while it is transmitting, unless other channel is used.
	uint2 index = make_uint2(receiverId, receiverLaunchIndex.z);
	if (externalId == receiverLaunchIndex.z) {
		//Outgoing ray
		//rtPrintf("External. id=%u tx=%u i.x=%u i.y=%u, ray=(%f,%f,%f) origin=(%f,%f,%f) t=%f rId=%d\n", index.x, index.y, receiverLaunchIndex.x, receiverLaunchIndex.y, ray_receiver.direction.x, ray_receiver.direction.y, ray_receiver.direction.z, ray_receiver.origin.x, ray_receiver.origin.y, ray_receiver.origin.z, hit_attr.t, receiverId);

		return;
	}


	//uint2 index = make_uint2(receiverId, receiverLaunchIndex.z);//Index is given by transmitter and receiver ids
	int reflections = hitPayload.reflections;

	//Check incoming or outgoing ray
	rtBufferId<int, 3>& ib = internalRaysBuffer[receiverId];
	int prevRef = ib[receiverLaunchIndex];
	if (prevRef < 0) {
		//Incoming ray. Store number of reflections
		ib[receiverLaunchIndex] = reflections;
		++hitPayload.hits;
	}
	else {
		//Outgoing ray. Check reflections
		//Reinit the buffer
		ib[receiverLaunchIndex] = -1;
		if (prevRef == reflections) {
			//Ray has not been reflected within the receiver sphere, ignore it
			//rtPrintf("  rId=%d----> outgoing ray \n", receiverId);

			return;

		}
	}
	//float r = hit_attr.t; //Distance to origin
	//rtPrintf("HR. rx=(%f,%f,%f) radius=%f id=%u tx=%u\n", sphere.x,sphere.y,sphere.z, sphere.w, index.x, index.y);
	//rtPrintf("HR. HitPayload inte=%d \n", hitPayload.reflections);



	if (reflections == 0) {
		//This is a direct ray

		int old = atomicAdd(&receptionInfoBuffer[index].directHits, 1);
		//rtPrintf("before dh=%d,id=%u tx=%u receptionInfoBuffer[index].directHits=%d  i.x=%u i.y=%u, ray=(%f,%f,%f) t=%f \n", old, index.x, index.y, receptionInfoBuffer[index].directHits, receiverLaunchIndex.x, receiverLaunchIndex.y, ray_receiver.direction.x, ray_receiver.direction.y, ray_receiver.direction.z, hit_attr.t);
		if (old > 0) {
			//rtPrintf("HR. already direct \n");
			//Already has a direct ray counted, just ignore
			return;
		}
		//Compute electric field. For direct rays, the distance is always between tx and rx
		float3 prx = make_float3(sphere.x, sphere.y, sphere.z);
		float3 ptx = tx_origin[launchIndex.z].origin;
		float d = length(prx - ptx);
		//rtPrintf("DR. Direct hit   i.x=%u i.y=%u ray=(%f,%f,%f) prx=(%f,%f,%f) ptx=(%f,%f,%f) rId=%d \n",  receiverLaunchIndex.x, receiverLaunchIndex.y, ray_receiver.direction.x, ray_receiver.direction.y, ray_receiver.direction.z, prx.x, prx.y, prx.z, ptx.x, ptx.y, ptx.z, receiverId);
		//rtPrintf("HR. Ep.x=%f Ep.y=%f Eo=%f\n", receptionInfoBuffer[index].sumRxElectricField.x, receptionInfoBuffer[index].sumRxElectricField.y, hitPayload.electricFieldAmplitude);
		float2 z = make_float2(0.0f, -k*d);
		//rtPrintf("HR. prx=(%f,%f,%f) k=%f d=%f prevTd=%f hitPayload.totalDistance=%f Eo=%f\n", prx.x, prx.y, prx.z, k, d, prevTd, hitPayload.totalDistance, hitPayload.electricFieldAmplitude);

		float2 zexp = complex_exp_only_imaginary(z);
		float2 E = sca_complex_prod((hitPayload.electricFieldAmplitude / d), zexp);

		float oldEx = atomicAdd(&receptionInfoBuffer[index].sumRxElectricField.x, E.x);
		float oldEy = atomicAdd(&receptionInfoBuffer[index].sumRxElectricField.y, E.y);
		//rtPrintf("DR. Direct hit   i.x=%u i.y=%u  Ep=(%f,%f) E=(%f,%f) En=(%f,%f) rId=%d \n", receiverLaunchIndex.x, receiverLaunchIndex.y, oldEx,oldEy, E.x, E.y, receptionInfoBuffer[index].sumRxElectricField.x, receptionInfoBuffer[index].sumRxElectricField.y, receiverId);
		//Direct hit info log (to be used in external programs)
		//rtPrintf("DH\t%u\t%u\t%u\t%f\t%f\t%f\t%f\t%f\t%f\t%d\n", receiverLaunchIndex.x, receiverLaunchIndex.y, receiverLaunchIndex.z, oldEx, oldEy, E.x, E.y, receptionInfoBuffer[index].sumRxElectricField.x, receptionInfoBuffer[index].sumRxElectricField.y, receiverId);

		//receptionInfoBuffer[index].hasDirectContribution = true;
		//rtPrintf("%f\t%f\n", E.x, E.y);
	}
	else {

		//Reflected ray
		//Check for duplicates
		//Index of buffer is receiverLaunchIndex= receiverLaunchIndex.x, receiverLaunchIndex.y,txId
		rtBufferId<int, 3>&  min_d_transmitter =bufferMinD [receiverLaunchIndex.z];

		uint3 idmd = make_uint3(reflections-1, hitPayload.faceId, receiverId);

		//Distance from ray line to receiver position
		//Line is defined by ray
		float3 prx = make_float3(sphere.x, sphere.y, sphere.z);
		float3 pd = prx - hitPayload.hitPoint;
		float u = dot(pd, ray_receiver.direction);
		float3 p3 = hitPayload.hitPoint + u*ray_receiver.direction;


		float dm = length(prx - p3)*1000000.0f;  //Multiply by 1000 000 to truncate later take 6 digits
		int dmt = __float2int_rz(dm);   //Truncate
		
		//int oldd = atomicMin(&bufferMinD[idmd], dmt);
		int oldd = atomicMin(&min_d_transmitter[idmd], dmt);
		if (oldd < dmt) {
			//our distance is greater,return
			//rtPrintf("FRF\t%u\t%u\t%u\t%f\t%d\t%d\t%f\t%f\t%f\n", receiverLaunchIndex.x, receiverLaunchIndex.y, reflections, dm, dmt, oldd, hitPayload.hitPoint.x, hitPayload.hitPoint.y, hitPayload.hitPoint.z);

			return;
		}
		//rtPrintf("FT\t%u\t%u\t%u\t%f\t%d\t%d\t%f\t%f\t%f\n", receiverLaunchIndex.x, receiverLaunchIndex.y, reflections, dm, dmt, oldd, hitPayload.hitPoint.x, hitPayload.hitPoint.y, hitPayload.hitPoint.z);


		//rtPrintf("Reflected hit reflections=%d i.x=%u i.y=%u db=%d with idb.x=%u idb.y=%u oldR=%u bit=%u rId=%d \n", hitPayload.reflections, receiverLaunchIndex.x, receiverLaunchIndex.y, db[receiverLaunchIndex], idx.x, idx.y, oldR, bit, receiverId);
		atomicAdd(&receptionInfoBuffer[index].reflections, 1);


		float3 ptx = ray_receiver.origin;
		float d = length(prx - ptx);
		//Compute electric field
		//rtPrintf("ref totalDistance=%f d=%f reflections=%d i.x=%u i.y=%u \n", hitPayload.totalDistance, d, hitPayload.reflections, receiverLaunchIndex.x, receiverLaunchIndex.y);
		d += prevTd; //totalDistance

		float2 z = make_float2(0.0f, -k*d);
		float2 zexp = complex_exp_only_imaginary(z);
		float2 Rzexp = complex_prod(hitPayload.prodReflectionCoefficient, zexp);


		float2 E = sca_complex_prod((hitPayload.electricFieldAmplitude / d), Rzexp);
		//rtPrintf("ref R=(%f,%f) z=(%f,%f) zepx(%f,%f) Rzexp=(%f,%f), E=(%f,%f) i.x=%u i.y=%u \n", hitPayload.prodReflectionCoefficient.x, hitPayload.prodReflectionCoefficient.y, z.x, z.y, zexp.x, zexp.y, Rzexp.x, Rzexp.y, E.x, E.y, receiverLaunchIndex.x, receiverLaunchIndex.y);



		//float2 Eprev = bufferMinE[idmd].E;
		//Update min buffer
		//bufferMinE[idmd].E = E;
		//Update min buffer
	//	rtBufferId<DuplicateReflection,3>&  min_e_transmitter =bufferMinE [receiverLaunchIndex.z];
		
	
		//rtBufferId<float2,3>&  min_e_transmitter =bufferMinE [receiverLaunchIndex.z];
		//float*  drx = &min_e_transmitter[idmd].x;
		//float*  dry = &min_e_transmitter[idmd].y;
		
		
		//float*  drx = &min_e_transmitter[idmd].E.x;
		//float*  dry = &min_e_transmitter[idmd].E.y;
		//float*  drx = &bufferMinE[idmd].E.x;
		//float*  dry = &bufferMinE[idmd].E.y;
		
		
		//float Eprevx = atomicExch(drx, E.x);
		//float Eprevy = atomicExch(dry, E.y);
		//float2 Eprev = make_float2(Eprevx, Eprevy);
		//rtPrintf("FR\t%u\t%u\t%u\t%u\t%f\t%f\n", receiverLaunchIndex.x, receiverLaunchIndex.y, hitPayload.faceId, reflections, hitPayload.prodReflectionCoefficient.x, hitPayload.prodReflectionCoefficient.y);

		//rtPrintf("FF\t%u\t%u\t%u\t%u\t%d\t%d\t%f\t%f\t%f\t%f\t%f\t%f\n", receiverLaunchIndex.x, receiverLaunchIndex.y, hitPayload.faceId, reflections,  dmt, oldd, E.x, E.y, Eprev.x, Eprev.y, prevTd, length(prx - ptx));

		//Remove Electric field from previous minimum distance hit
		/*E -= Eprev;

		//Update the receiver
		float oldEx = atomicAdd(&receptionInfoBuffer[index].sumRxElectricField.x, E.x);
		float oldEy = atomicAdd(&receptionInfoBuffer[index].sumRxElectricField.y, E.y);
		//rtPrintf("HR. i.x=%u i.y=%u  Reflected hit  reflections=%d Ep=(%f,%f) E=(%f,%f) En=(%f,%f) rId=%d \n", receiverLaunchIndex.x, receiverLaunchIndex.y, reflections, oldEx, oldEy, E.x, E.y, receptionInfoBuffer[index].sumRxElectricField.x, receptionInfoBuffer[index].sumRxElectricField.y, receiverId);

		//rtPrintf("Old E=(%f.%f) New E=(%f,%f) i.x=%u i.y=%u \n", oldx, oldy, receptionInfoBuffer[index].sumRxElectricField.x, receptionInfoBuffer[index].sumRxElectricField.y, receiverLaunchIndex.x, receiverLaunchIndex.y);
		//rtPrintf("%f\t%f\n", E.x, E.y);
		//Reflected hit info log (to be used in external programs)
		rtPrintf("F\t%u\t%u\t%u\t%f\t%f\t%f\t%f\t%f\t%f\t%d\t%f\t%f\n", receiverLaunchIndex.x, receiverLaunchIndex.y, reflections, oldEx, oldEy, E.x, E.y, receptionInfoBuffer[index].sumRxElectricField.x, receptionInfoBuffer[index].sumRxElectricField.y, receiverId, prevTd, length(prx - ptx));

		*/
		rtPrintf("F\t%u\t%u\t%u\t%f\t%f\t%f\t%f\t%d\t%f\t%f\n", receiverLaunchIndex.x, receiverLaunchIndex.y, reflections,  E.x, E.y, receptionInfoBuffer[index].sumRxElectricField.x, receptionInfoBuffer[index].sumRxElectricField.y, receiverId, prevTd, length(prx - ptx));
	}

}



rtDeclareVariable(EMWavePayload, missPayload, rtPayload, );
RT_PROGRAM void miss()
{
	//rtPrintf("miss i.x=%u. iy=%u \n", receiverLaunchIndex.x, receiverLaunchIndex.y);
	missPayload.end = true;
}

