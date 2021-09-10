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
//#include "../../Complex.h"
//#include "../../tracefunctions.h"
#include "../reflectionFunctions.h"
#include "../triangleFunctions.h"
#include <optix.h>
#include <optixu/optixu_math_namespace.h>
#include <optixu/optixu_aabb_namespace.h>
//#include <cmath>
using namespace optix;

//Launch variables
rtDeclareVariable(uint3, launchIndexTriangle, rtLaunchIndex, );
rtDeclareVariable(LPWavePayload, rayPayload, rtPayload, );
rtDeclareVariable(optix::Ray, ray, rtCurrentRay, );
rtDeclareVariable(TriangleHit, ch_triangle_data, attribute triangle_hit_data, );

//Per-mesh local variables 
rtDeclareVariable(MaterialEMProperties, EMProperties, , );
rtDeclareVariable(uint, meshId, , );
rtDeclareVariable(uint, curvedMesh, , );

//Penetration configuration
//rtDeclareVariable(uint, usePenetration, , );
//rtDeclareVariable(float, attenuationLimit, , );

RT_PROGRAM void closestHitTriangle()
{

	
	
	ReflectedRayBasis rrb = getReflectedRayBasis<TriangleHit>(ch_triangle_data,ray);
	
	uint reflections=updateTrianglePayload<LPWavePayload,TriangleHit>(rayPayload,ch_triangle_data,ray,rrb.reflection_dir);        
        const float4 rc=getReflectionCoefficient<LPWavePayload>(rayPayload,rrb.cosA, EMProperties);
        //const float4 rc=getPerfectConductor();

	//Create transmission here to copy rayPayload before updating below the reflection coefficient
	if ((usePenetration==1u) && (reflections<max_interactions)) {
		penetrationRay<LPWavePayload>(rayPayload,ray,rrb,ch_triangle_data.hp,rc,EMProperties,launchIndexTriangle.x,launchIndexTriangle.y);
	}

	//Compute and update the payload with the reflection coefficients 
	updateReflectionCoefficient<LPWavePayload>(rayPayload,ray,rrb, rc);


}



