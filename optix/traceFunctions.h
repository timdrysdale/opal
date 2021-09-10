/***************************************************************/
//
//Copyright (c) 2019 Esteban Egea-Lopez http://girtel.upct.es/~eegea
//
/**************************************************************/
#ifndef TRACEFUNCTIONS_H
#define TRACEFUNCTIONS_H


#include "../Common.h"
#include "Complex.h"
#include <optix_world.h>
#include  <optixu/optixu_matrix_namespace.h>
using namespace optix;
rtDeclareVariable(rtObject, root, , );
rtDeclareVariable(float, min_t_epsilon, , );
rtDeclareVariable(unsigned int, max_interactions, , );
//trace buffer
rtBuffer<LogTraceHitInfo, 1> traceBuffer;
rtBuffer<uint, 1> traceAtomicIndex; //Buffer to store the current log trace buffer index 


	template<class T>
__forceinline__ __device__ void traceReflection(T& rayPayload, unsigned int rayType, float3 ro, float3 rd, unsigned int x, unsigned int y, bool log) 
{
	float3 origin=ro;
	float3 ray_direction=rd;
	// Each iteration is a segment (due to reflections) of the ray path.  The closest hit will
	// return new segments to be traced here. Additionally, the closest hit at receiver will generate another ray to continue the propagation through the recption sphere
	while (true) {
#ifdef OPAL_AVOID_SI
		optix::Ray myRay(origin, ray_direction, rayType, 0.0f, RT_DEFAULT_MAX);
#else
		optix::Ray myRay(origin, ray_direction, rayType, min_t_epsilon, RT_DEFAULT_MAX);
#endif
		rtTrace(root, myRay, rayPayload, RT_VISIBILITY_ALL, RT_RAY_FLAG_DISABLE_ANYHIT);



		const uint flags=rayPayload.rhfr.z;
		const uint reflections = rayPayload.rhfr.x;
		//Miss or too much attenuation
		//Extract flag
		uint end = (flags >> FLAG_END_POSITION) & 1u;
		if (end) {
			break;
		}
		//Max number of reflections
		if (reflections > max_interactions) {
			break;
		}

		//Reflection or going through receiver
		// Update ray data for the next path segment
		ray_direction =make_float3(rayPayload.ndtd.x,rayPayload.ndtd.y,rayPayload.ndtd.z);
		const float3 hitPoint = make_float3(rayPayload.hitPointAtt.x,rayPayload.hitPointAtt.y,rayPayload.hitPointAtt.z);	
#ifdef OPAL_AVOID_SI
		//Shift along the normal to avoid self-intersection
		float pr=dot(ray_direction,rayPayload.lastNormal);
		float s=((pr>=0)?1.0f:-1.0f);
		origin = offset_ray(hitPoint,s*rayPayload.lastNormal);
#else
		origin = hitPoint;
#endif

		//Reflection info log (to be used in external programs)
		//Here we print the reflection. However, it seems that the print buffer cannot be synchronized and some threads may overwrite others, or so I think.
		//To do it properly, we would need to store  the trace information in its own buffer and print it after the launch..
		if (log) {
			//With printf instead of rtPrintf we force printing even if it is disabled
			//printf("RL\t%u\t%u\t%d\t%f\t%f\t%f\t%f\t%f\t%f\t%f\n", x, y,  reflections,  ray_direction.x, ray_direction.y, ray_direction.z, rayPayload.hitPointAtt.x, rayPayload.hitPointAtt.y, rayPayload.hitPointAtt.z, rayPayload.ndtd.w);
			LogTraceHitInfo aHit;
			aHit.hitp = rayPayload.hitPointAtt;
			aHit.cdata=make_uint4(x,reflections,0,0);
			uint hitIndex=atomicAdd(&traceAtomicIndex[0u],1u);
			traceBuffer[hitIndex]=aHit;
			//traceBuffer[aHit.ray_ref]=aHit;
		}

		//	rtPrintf("RL\t%u\t%u\t%d\t%f\t%f\t%f\t%f\t%f\t%f\t%f\n", x, y,  reflections,  ray_direction.x, ray_direction.y, ray_direction.z, rayPayload.hitPointAtt.x, rayPayload.hitPointAtt.y, rayPayload.hitPointAtt.z, rayPayload.ndtd.w);


		//Verbose log
	}

}


//TODO: to generate rays directly from a dedicated launch or during generation. Improves performance when the ray sphere changes from launch to launch. Otherwise, generate a ray sphere once
__forceinline__ __device__  float3 equispacedRayDirection( uint2 index, float4 range ) {
	//range [elevationInit, elevationDelta, azimuthInit, azimuthDelta]
	const unsigned int ie=index.x; //Elevation index
	const unsigned int ia=index.y; //Azimuth index
	float elr=range.x +ie*range.y; //elevation angle in radians (range in radians)
	float azr=range.z +ia*range.w; //azimuth angle in radians

			//Spherical to cartesian coordinates with r=1, ISO (physics) convention: elevation (inclination) from z-axis, azimuth on the XY plane from x counterclockwise to Y
			//All rays with elevation 0 or 180 and any azimuth  are repeated, because they map to (0,1,0) or (0,-1,0). They will not be traced, but we generate them to keep the symmetry  and use 
			//Unity left-handed coordinate system: Y=up, Z=forward, X=right; elevation from Y to Z (around X) clockwise, azimuth from Z to X (around Y) clockwise 
	return 	 make_float3(sinf(elr)*sinf(azr), cosf(elr), sinf(elr)*cosf(azr) );


} 
#endif //TRACEFUNCTIONS_H
