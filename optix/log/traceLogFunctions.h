/***************************************************************/
//
//Copyright (c) 2019 Esteban Egea-Lopez http://ait.upct.es/eegea
//
/**************************************************************/


//Replicate the trace functions for polarization. We could have adopted more elegant approaches with inheritance or templates and so on, but I am not sure about the alignment of inherited structs

#include "../../Common.h"
#include <optix_world.h>
#include  <optixu/optixu_matrix_namespace.h>
using namespace optix;
rtDeclareVariable(rtObject, root, , );
rtDeclareVariable(float, min_t_epsilon, , );
rtDeclareVariable(unsigned int, max_interactions, , );



__forceinline__ __device__ void traceLogTrace(LogTracePayload& rayPayload, float3 ro, float3 rd, unsigned int x, unsigned int y) 
{
	float3 origin=ro;
	float3 ray_direction=rd;
	// Each iteration is a segment (due to reflections) of the ray path.  The closest hit will
	// return new segments to be traced here. Additionally, the closest hit at receiver will generate another ray to continue the propagation through the recption sphere
	while (true) {
		#ifdef OPAL_AVOID_SI
			optix::Ray myRay(origin, ray_direction, OPAL_RAY_LOG_TRACE_RAY, 0.0f, RT_DEFAULT_MAX);
		#else
			optix::Ray myRay(origin, ray_direction, OPAL_RAY_LOG_TRACE_RAY, min_t_epsilon, RT_DEFAULT_MAX);
 		#endif

		rtTrace(root, myRay, rayPayload, RT_VISIBILITY_ALL, RT_RAY_FLAG_DISABLE_ANYHIT);



		//Miss or too much attenuation
		if (rayPayload.flags==FLAG_END) {
			break;
		}
		//Max number of reflections
		if (rayPayload.reflections > max_interactions) {
			break;
		}

		//Reflection or going through receiver
		// Update ray data for the next path segment
		ray_direction =make_float3(rayPayload.ndtd.x,rayPayload.ndtd.y,rayPayload.ndtd.z);
		
		#ifdef OPAL_AVOID_SI
			//Shift along the normal to avoid self-intersection
			float pr=dot(ray_direction,rayPayload.lastNormal);
			float s=((pr>=0)?1.0f:-1.0f);
			origin = offset_ray(rayPayload.hitPoint,s*rayPayload.lastNormal);
			rtPrintf("SI\t%u\t%u\t%d\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\n", x, y,  rayPayload.reflections,  ray_direction.x, ray_direction.y, ray_direction.z, rayPayload.lastNormal.x,rayPayload.lastNormal.y,rayPayload.lastNormal.z, origin.x, origin.y, origin.z, rayPayload.ndtd.w);
		#else
			origin = rayPayload.hitPoint;
		#endif


		//Reflection info log (to be used in external programs)
		//launchIndex is not available in this scope, pass it to the function to be used in the log

		rtPrintf("RL\t%u\t%u\t%d\t%f\t%f\t%f\t%f\t%f\t%f\t%f\n", x, y,  rayPayload.reflections,  ray_direction.x, ray_direction.y, ray_direction.z, rayPayload.hitPoint.x, rayPayload.hitPoint.y, rayPayload.hitPoint.z, rayPayload.ndtd.w);

		//Verbose log
		//rtPrintf("Reflecting ray i.x=%u i.y=%u, inter=%d hits=%d rd=(%f,%f,%f) origin=(%f,%f,%f) end=%d \n", launchIndex.x, launchIndex.y, rayPayload.reflections, rayPayload.hits, rayPayload.reflectionDirection.x, rayPayload.reflectionDirection.y, rayPayload.reflectionDirection.z, rayPayload.hitPoint.x, rayPayload.hitPoint.y, rayPayload.hitPoint.z, rayPayload.end);
	}

}

