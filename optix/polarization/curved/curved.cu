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

#include "../../../Common.h"
#include "../../Complex.h"
#include "../../traceFunctions.h"
#include "../../reflectionFunctions.h"
#include <optix.h>
#include <optixu/optixu_math_namespace.h>
#include <optixu/optixu_aabb_namespace.h>
#include "../../curvedFunctions.h"
#include "../../triangleFunctions.h"

using namespace optix;

//Launch variables
rtDeclareVariable(uint3, launchIndexTriangle, rtLaunchIndex, );
rtDeclareVariable(CurvedMeshLPWavePayload, rayPayload, rtPayload, );
rtDeclareVariable(optix::Ray, ray, rtCurrentRay, );
rtDeclareVariable(CurvedTriangleHit, ch_triangle_data, attribute curved_triangle_hit_data, );

//Per-mesh local variables 
rtDeclareVariable(MaterialEMProperties, EMProperties, , );
rtDeclareVariable(uint, meshId, , );
rtDeclareVariable(uint, curvedMesh, , );

//Penetration configuration
//rtDeclareVariable(uint, usePenetration, , );
//rtDeclareVariable(float, attenuationLimit, , );

//Debug invalid rays
rtBuffer<int, 1> invalidRaysBuffer; //Buffer to store the number of invalid rays 
RT_PROGRAM void closestHitCurvedMesh()
{

	//Update payload
	//Get the hitpoint from the barycentric coordinates computed in the triangle hit. This should get us a point always on the surface and help avoid self-intersection
	//See https://www.realtimerendering.com/raytracinggems/ 6.1
	
	//s_prime: Segment length from last physical interaction to this physical interaction
	const float3 lastReflectionHitPoint = make_float3(rayPayload.lrhpd.x,rayPayload.lrhpd.y,rayPayload.lrhpd.z);
	const float s_prime=length(ch_triangle_data.hp-lastReflectionHitPoint);

	//Flag as hit on curved surface
	rayPayload.rhfr.z |= 1u<<FLAG_CURVED_MESH_POSITION;

	
	
	ReflectedRayBasis rrb = getReflectedRayBasis<CurvedTriangleHit>(ch_triangle_data,ray);
	uint reflections=updateTrianglePayload<CurvedMeshLPWavePayload,CurvedTriangleHit>(rayPayload,ch_triangle_data,ray,rrb.reflection_dir);        
        
        const float4 rc=getReflectionCoefficient<CurvedMeshLPWavePayload>(rayPayload,rrb.cosA, EMProperties);
	updateReflectionCoefficient<CurvedMeshLPWavePayload>(rayPayload,ray,rrb, rc);
	
	
	

	//WARNING: ASSUMING spherical wave, 
	//Curvature information. Has to be defined externally for the mesh. Cannot be deduced from the triangles.
	 float3 u1=make_float3(ch_triangle_data.principalDirection1.x,ch_triangle_data.principalDirection1.y,ch_triangle_data.principalDirection1.z);	
	 float3 u2=make_float3(ch_triangle_data.principalDirection2.x,ch_triangle_data.principalDirection2.y,ch_triangle_data.principalDirection2.z);		
	 float R1=ch_triangle_data.principalDirection1.w;  
	 float R2=ch_triangle_data.principalDirection2.w;
	
	updateCurvature<CurvedMeshLPWavePayload>(rayPayload,rrb,s_prime,reflections,u1,u2,R1,R2,false);


}




