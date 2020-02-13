/***************************************************************/
//
//Copyright (c) 2019 Esteban Egea-Lopez http://ait.upct.es/eegea
//
/**************************************************************/



#include "Common.h"
#include "Complex.h"
#include <optix_world.h>
#include  <optixu/optixu_matrix_namespace.h>
using namespace optix;
rtDeclareVariable(rtObject, root, , );
rtDeclareVariable(float, min_t_epsilon, , );
rtDeclareVariable(unsigned int, max_interactions, , );

	template<class T>
__forceinline__ __device__ void traceReflection(T& rayPayload, unsigned int rayType, float3 ro, float3 rd, unsigned int x, unsigned int y) 
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
		if (flags==FLAG_END) {
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
#ifdef OPAL_LOG_TRACE
		//Here we print the reflection. However, it seems that the print buffer cannot be synchronized and some threads may overwrite others, or so I think.
		//To do it properly, we would need to store  the trace information in its own buffer and print it after the launch..
		if (rayType==OPAL_RAY_LOG_TRACE_RAY) {
			rtPrintf("RL\t%u\t%u\t%d\t%f\t%f\t%f\t%f\t%f\t%f\t%f\n", x, y,  reflections,  ray_direction.x, ray_direction.y, ray_direction.z, rayPayload.hitPointAtt.x, rayPayload.hitPointAtt.y, rayPayload.hitPointAtt.z, rayPayload.ndtd.w);
		}
#endif


		//rtPrintf("R\t%u\t%u\t%d\t%d\t%f\t%f\t%f\t%f\t%f\t%f\t%f\n", x, y,  rayPayload.reflections, rayPayload.hits, ray_direction.x, ray_direction.y, ray_direction.z, rayPayload.hitPoint.x, rayPayload.hitPoint.y, rayPayload.hitPoint.z, rayPayload.ndtd.w);

		//Verbose log
		//rtPrintf("Reflecting ray i.x=%u i.y=%u, inter=%d hits=%d rd=(%f,%f,%f) origin=(%f,%f,%f) end=%d \n", launchIndex.x, launchIndex.y, rayPayload.reflections, rayPayload.hits, rayPayload.reflectionDirection.x, rayPayload.reflectionDirection.y, rayPayload.reflectionDirection.z, rayPayload.hitPoint.x, rayPayload.hitPoint.y, rayPayload.hitPoint.z, rayPayload.end);
	}

}


