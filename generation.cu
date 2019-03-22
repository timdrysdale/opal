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


//Generation of ray sphere


//Ray Sphere buffer
rtBuffer<float3, 2> raySphere2D;
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


