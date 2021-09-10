/***************************************************************/
//
//Copyright (c) 2019 Esteban Egea-Lopez http://girtel.upct.es/~eegea
//
/**************************************************************/


#include "../../Common.h"
#include <optix_world.h>
#include <optixu/optixu_math_namespace.h>
#include <optixu/optixu_aabb_namespace.h>
using namespace optix;




//Receiver local variables
rtDeclareVariable(SphereHit, hit_attr, attribute hit_attr, );
//Launch variables
rtDeclareVariable(uint3, receiverLaunchIndex, rtLaunchIndex, );
rtDeclareVariable(BaseReflectionPayload, hitPayload, rtPayload, );
rtDeclareVariable(optix::Ray, ray_receiver, rtCurrentRay, );






RT_PROGRAM void closestHitReceiverLogTrace()
{


	//Do not end the ray, it can pass through the reception sphere and reflect on a wall, inside or outside the receiver sphere

	//Update ray data
	const float rayLength = hit_attr.geom_normal_t.w;
	const float3 hitPoint = ray_receiver.origin + rayLength*ray_receiver.direction;
	hitPayload.hitPointAtt.x=hitPoint.x;
	hitPayload.hitPointAtt.y=hitPoint.y;
	hitPayload.hitPointAtt.z=hitPoint.z;
	const float aux= hitPayload.ndtd.w; //Previous total distance
	hitPayload.ndtd = make_float4(ray_receiver.direction); //Next direction only
	hitPayload.ndtd.w = aux+ rayLength; //totalDistance 
#ifdef OPAL_AVOID_SI

	hitPayload.lastNormal=make_float3(hit_attr.geom_normal_t.x,hit_attr.geom_normal_t.y,hit_attr.geom_normal_t.z);
	//hitPayload.lastNormal=normalize(hitPoint-prx);
#endif


}








rtDeclareVariable(BaseReflectionPayload, missPayload, rtPayload, );
//Miss program. End ray
RT_PROGRAM void missLogTrace()
{
	//rtPrintf("miss i.x=%u. iy=%u \n", receiverLaunchIndex.x, receiverLaunchIndex.y);
	//missPayload.flags = FLAG_END;
	//missPayload.rhfr.z = FLAG_END;
	missPayload.rhfr.z |= 1u<<FLAG_END_POSITION;
}

