/***************************************************************/
//
//Copyright (c) 2019 Esteban Egea-Lopez http://girtel.upct.es/~eegea
//
/**************************************************************/

//License from NVIDIA parts
/*
 * Copyright (c) 2016, NVIDIA CORPORATION. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *  * Neither the name of NVIDIA CORPORATION nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS ``AS IS'' AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY
 * OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "../../Common.h"
#include <optix.h>
#include <optixu/optixu_math_namespace.h>
#include <optixu/optixu_aabb_namespace.h>
//#include <cmath>
using namespace optix;

//Launch variables
rtDeclareVariable(uint3, launchIndexTriangle, rtLaunchIndex, );
rtDeclareVariable(BaseReflectionPayload, rayPayload, rtPayload, );
rtDeclareVariable(optix::Ray, ray, rtCurrentRay, );
rtDeclareVariable(TriangleHit, ch_triangle_data, attribute triangle_hit_data, );

//Per-mesh local variables 
rtDeclareVariable(uint, meshId, , );


//Only for reflections, penetrations are not visualized yet
RT_PROGRAM void closestHitTriangleLogTrace()
{

	//Update payload
	//const float rayLength = ch_triangle_data.geom_normal_t.w;
	//const float3 hp= ray.origin + rayLength * ray.direction ;
	//rayPayload.hitPoint =hp;
	//Get the hitpoint from the barycentric coordinates computed in the triangle hit. This should get us a point always on the surface and help avoid self-intersection
	//See https://www.realtimerendering.com/raytracinggems/ 6.1
	const float3 lastHP=make_float3(rayPayload.hitPointAtt.x,rayPayload.hitPointAtt.y,rayPayload.hitPointAtt.z);
	//we could use t of ray, but if we shift the ray over the normal to avoid self-intersection we introduce an error in the electric field
	const float rayLength=length(ch_triangle_data.hp-lastHP);
	rayPayload.hitPointAtt.x =ch_triangle_data.hp.x;
	rayPayload.hitPointAtt.y =ch_triangle_data.hp.y;
	rayPayload.hitPointAtt.z =ch_triangle_data.hp.z;
	const float3 gn=make_float3(ch_triangle_data.geom_normal_t.x,ch_triangle_data.geom_normal_t.y,ch_triangle_data.geom_normal_t.z);	
	const float3 n = normalize(rtTransformNormal(RT_OBJECT_TO_WORLD,gn )); //Plane normal
	const float3 reflection_dir=reflect(ray.direction, n);
	const float aux=rayPayload.ndtd.w;
	rayPayload.ndtd = make_float4(reflection_dir); //initialized with float3, w is set to 0. and updated below
	rayPayload.ndtd.w = aux+ rayLength;
#ifdef OPAL_AVOID_SI
	rayPayload.lastNormal=n;
#endif	
	
	
	uint reflections=rayPayload.rhfr.x;
	rayPayload.rhfr.x=reflections+1u;
	

}



