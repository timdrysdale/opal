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

//For Optix 5.x
//Mesh buffers
rtBuffer<float3> vertex_buffer;
rtBuffer<float4> principalDirection1_buffer;
rtBuffer<float4> principalDirection2_buffer;
rtBuffer<int3>   index_buffer;
rtBuffer<uint> faceId_buffer;

rtDeclareVariable(CurvedTriangleHit, int_triangle_data, attribute triangle_hit_data, );

RT_PROGRAM void intersectTriangleCurved(int primIdx)
{
	const int3 v_idx = index_buffer[primIdx];

	const float3 p0 = vertex_buffer[v_idx.x];
	const float3 p1 = vertex_buffer[v_idx.y];
	const float3 p2 = vertex_buffer[v_idx.z];

	// Intersect ray with triangle
	float3 normal;
	float  t, beta, gamma;

	//rtPrintf("PreIntersection idx=%d ray=(%f,%f,%f)", primIdx, ray.direction.x, ray.direction.y, ray.direction.z);
	if (intersect_triangle(ray, p0, p1, p2, normal, t, beta, gamma))
	{
		if (rtPotentialIntersection(t))
		{
			CurvedTriangleHit h;
			h.triId = primIdx;
			h.geom_normal_t = make_float4(normal.x,normal.y,normal.z,t);
			h.faceId = faceId_buffer[primIdx];

			h.principalDirection1=principalDirection1_buffer[primIdx];
			h.principalDirection2=principalDirection2_buffer[primIdx];

			int_triangle_data = h;
			//rtPrintf("Intersection idx=%d ray=(%f,%f,%f)", primIdx, ray.direction.x, ray.direction.y, ray.direction.z);
			rtReportIntersection( /*material index*/ 0);
		}
	}
}
RT_PROGRAM void boundsTriangle(int primIdx, float result[6])
{
	const int3 v_idx = index_buffer[primIdx];

	const float3 p0 = vertex_buffer[v_idx.x];
	const float3 p1 = vertex_buffer[v_idx.y];
	const float3 p2 = vertex_buffer[v_idx.z];

	optix::Aabb* aabb = (optix::Aabb*)result;
	aabb->m_min = fminf(fminf(p0, p1), p2);
	aabb->m_max = fmaxf(fmaxf(p0, p1), p2);
}

