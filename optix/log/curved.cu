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
using namespace optix;

//Launch variables
rtDeclareVariable(uint3, launchIndexTriangle, rtLaunchIndex, );
rtDeclareVariable(CurvedMeshLPWavePayload, rayPayload, rtPayload, );
rtDeclareVariable(optix::Ray, ray, rtCurrentRay, );
rtDeclareVariable(CurvedTriangleHit, ch_triangle_data, attribute curved_triangle_hit_data, );


RT_PROGRAM void closestHitCurvedLogTrace()
{

	//Update payload
	//Get the hitpoint from the barycentric coordinates computed in the triangle hit. This should get us a point always on the surface and help avoid self-intersection
	//See https://www.realtimerendering.com/raytracinggems/ 6.1
	const float3 lastHP=make_float3(rayPayload.hitPointAtt.x,rayPayload.hitPointAtt.y,rayPayload.hitPointAtt.z);
	//we could use t of ray, but if we shift the ray over the normal to avoid self-intersection we introduce an error in the electric field
       
	 //Segment length from last physical interaction to this physical interaction
	const float3 lastReflectionHitPoint = make_float3(rayPayload.lrhpd.x,rayPayload.lrhpd.y,rayPayload.lrhpd.z);

	//The rayLength is the distance from the last hit point to this hit point. Notice that, for instance, if the last hp is on a sphere receiver, there was no actual physical interaction with 
        //any element. In that case, this rayLength is different from s_prime. It is only used to update the total distance (unfolded) of this ray. 
	//TODO: it is used in the basic setup, for curved may be removed
	const float rayLength=length(ch_triangle_data.hp-lastHP);
	rayPayload.hitPointAtt.x =ch_triangle_data.hp.x;
	rayPayload.hitPointAtt.y =ch_triangle_data.hp.y;
	rayPayload.hitPointAtt.z =ch_triangle_data.hp.z;
	//rtPrintf("THP\t%u\t%u\tbary=(%.6e,%.6e,%.6e)\n",launchIndexTriangle.x,launchIndexTriangle.y,ch_triangle_data.hp.x,ch_triangle_data.hp.y,ch_triangle_data.hp.z);
	const float3 gn=make_float3(ch_triangle_data.geom_normal_t.x,ch_triangle_data.geom_normal_t.y,ch_triangle_data.geom_normal_t.z);	
	const float3 n = normalize(rtTransformNormal(RT_OBJECT_TO_WORLD,gn )); //Plane normal
	
	//Redundant
	//if (curvedMesh==1u) {
	rayPayload.rhfr.z |= 1u<<FLAG_CURVED_MESH_POSITION;
	//rayPayload.rhfr.z=rayPayload.rhfr.z+FLAG_CURVED_MESH;
	//}

#ifdef OPAL_AVOID_SI
	rayPayload.lastNormal=n;
#endif	
	const float3 reflection_dir=normalize(reflect(ray.direction, n));
	const float aux=rayPayload.ndtd.w; //previous total distance of the ray
	rayPayload.ndtd = make_float4(reflection_dir); //initialized with float3, w is set to 0. and updated below
	
	//Use reflections and hits to create hash
	
	//hash_combine_impl<uint>(rayPayload.refhash,ch_triangle_data.faceId+rayPayload.reflections+rayPayload.hits);
	rayPayload.ndtd.w = aux+ rayLength; //Update total distance of the ray
	
	
	
	//++rayPayload.reflections;
	uint reflections=rayPayload.rhfr.x;
	rayPayload.rhfr.x=reflections+1u;

}




