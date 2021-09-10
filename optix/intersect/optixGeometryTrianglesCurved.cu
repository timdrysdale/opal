/***************************************************************/
//
//Copyright (c) 2019 Esteban Egea-Lopez http://girtel.upct.es/~eegea
//
/**************************************************************/


//Adapted from OptiX samples

/*
 * Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.
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


#include <optix.h>
#include <optixu/optixu_math_namespace.h>
#include "../../Common.h"
using namespace optix;

rtDeclareVariable(optix::Ray, ray, rtCurrentRay, );

//Mesh buffers
rtBuffer<float3> vertex_buffer;
rtBuffer<float4> principalDirection1_buffer;
rtBuffer<float4> principalDirection2_buffer;
rtBuffer<int3>   index_buffer;
rtBuffer<uint> faceId_buffer;

//rtDeclareVariable(CurvedTriangleHit, int_triangle_data, attribute triangle_hit_data, );
rtDeclareVariable(CurvedTriangleHit, int_triangle_data, attribute curved_triangle_hit_data, );
rtDeclareVariable(float, t,rtIntersectionDistance, );


RT_PROGRAM void triangle_attributes_curved()
{
	const unsigned int primIdx = rtGetPrimitiveIndex();
	const int3   v_idx = index_buffer[primIdx];
	const float3 p0    = vertex_buffer[v_idx.x];
	const float3 p1    = vertex_buffer[v_idx.y];
	const float3 p2    = vertex_buffer[v_idx.z];
	const float3 e0 = p1 - p0;
	const float3 e1 = p0 - p2;
	const float3 n  = cross( e1, e0 );

	//const float3 e2 = ( 1.0f / dot( n, ray.direction ) ) * ( p0 - ray.origin );
	//const float3 i  = cross( ray.direction, e2 );

	//beta  = dot( i, e1 );
	//gamma = dot( i, e0 );
	CurvedTriangleHit h;
	//h.t = dot(n,e2);
	h.triId = primIdx;

	//h.geom_normal_t=make_float4(n.x,n.y,n.z,dot(n,e2));
	h.geom_normal_t=make_float4(n.x,n.y,n.z,t);
	//h.geom_normal=n;

	h.faceId = faceId_buffer[primIdx];

	//Get hitpoint from barycentrics
	const float2 bar=rtGetTriangleBarycentrics();
	const float3 hp=(1.0f -bar.x -bar.y)*p0 + bar.x*p1 + bar.y*p2;
	h.hp=hp;
	
	h.principalDirection1=principalDirection1_buffer[primIdx];
	h.principalDirection2=principalDirection2_buffer[primIdx];
	int_triangle_data = h;

}

