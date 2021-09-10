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
//#include "../../../Complex.h"
//#include "../../../traceFunctions.h"
#include "../../reflectionFunctions.h"
#include "../../triangleFunctions.h"
//#include "rdnrandom.h"
#include <optix.h>
#include <optixu/optixu_math_namespace.h>
#include <optixu/optixu_aabb_namespace.h>
#include "../../curvedFunctions.h"
using namespace optix;

//Launch variables
rtDeclareVariable(uint3, launchIndexTriangle, rtLaunchIndex, );
rtDeclareVariable(uint2, launchDim,   rtLaunchDim, );
rtDeclareVariable(RDNLPWavePayload, rayPayload, rtPayload, );
rtDeclareVariable(optix::Ray, ray, rtCurrentRay, );
rtDeclareVariable(TriangleHit, ch_triangle_data, attribute triangle_hit_data, );

//Per-mesh local variables 
rtDeclareVariable(MaterialEMProperties, EMProperties, , );
rtDeclareVariable(uint, meshId, , );
rtDeclareVariable(uint, curvedMesh, , );

//Penetration configuration
//rtDeclareVariable(uint, usePenetration, , );
//rtDeclareVariable(float, attenuationLimit, , );

//Debug invalid rays
rtBuffer<int, 1> invalidRaysBuffer; //Buffer to store the number of invalid rays 

RT_PROGRAM void closestHitFlatMesh()
{
	//	const uint2 idx = make_uint2(launchIndexTriangle.x, launchIndexTriangle.y); //[elevation, azimuth]
	//	unsigned int s1=tea<8>(idx.y*launchDim.x+idx.x,1);
	//	float rng1=rng(s1);	
	//	if (rng1>0.5) {
	//			//Kill ray at the moment
	//			rayPayload.rhfr.z |= 1u<<FLAG_END_POSITION;
	//			//rayPayload.rhfr.z= FLAG_END;
	//			return;
	//	}




	//s_prime: Segment length from last physical interaction to this physical interaction
	const float3 lastReflectionHitPoint = make_float3(rayPayload.lrhpd.x,rayPayload.lrhpd.y,rayPayload.lrhpd.z);
	const float s_prime=length(ch_triangle_data.hp-lastReflectionHitPoint);

	ReflectedRayBasis rrb = getReflectedRayBasis<TriangleHit>(ch_triangle_data,ray);

	uint reflections=updateTrianglePayload<RDNLPWavePayload,TriangleHit>(rayPayload,ch_triangle_data,ray,rrb.reflection_dir);        

	const float4 rc=getReflectionCoefficient<RDNLPWavePayload>(rayPayload,rrb.cosA, EMProperties);
	//const float4 rc=getPerfectConductor();

	updateReflectionCoefficient<RDNLPWavePayload>(rayPayload,ray,rrb, rc);

	//Curvature part
	//Copy before updating below
	float rho1=rayPayload.radii.x;
	float rho2=rayPayload.radii.y;

	float R1=OPAL_INFINITY_F ;//This should be infinity
	float R2=OPAL_INFINITY_F;//This should be infinity
	//rtPrintf("ISINF R1=%d R2=%d \n",isinf(R1), isinf(R2));

	//Get Ortonormal basis from the triangle normal
	optix::Onb onb(rrb.normal); 
	float3 u1=onb.m_binormal;
	float3 u2=onb.m_tangent;
	//rtPrintf("ONB u1=(%f,%f,%f) u2=(%f,%f,%f) \n",u1.x,u2.y,u3.z,u2.x,u2.y,u2.z);
	//	rtPrintf("CHBEF ray=(%f,%f,%f) |ray|=%f x1=(%f,%f,%f) |x1|=%f x2=(%f,%f,%f) |x2|=%f \n",ray.direction.x,ray.direction.y,ray.direction.z,length(ray.direction),x1.x,x1.y,x1.z,length(x1),x2.x,x2.y,x2.z,length(x2));
	updateCurvature<RDNLPWavePayload>(rayPayload,rrb,s_prime,reflections,u1,u2,R1,R2,true);
	//Finaly, update density (RDN)
	if (reflections==0) {
		rayPayload.rayDensity =rayPayload.rayDensity/(s_prime*s_prime); //Initial ray density already includes rayNumber/solidAngle
	} else {
		float sq1=rho1/(rho1+s_prime);
		float sq2=rho2/(rho2+s_prime);
		//What if caustics are infinity
		//If we apply the L'Hôpital rule, the fraction goes to 1...
		//if (isinf(rho1)) {
		//	sq1=1;
		//} else if (sq1<0) {
		//	rayPayload.phaseJumps++;
		//}
		//if (isinf(rho2)) {
		//	sq2=1;
		//} else if (sq2<0) {
		//	rayPayload.phaseJumps++;
		//}
		if (isinf(rho1)) {
			sq1=1;
		}
		if (isinf(rho2)) {
			sq2=1;
		}
		if ((sq1*sq2)<0) {
			rayPayload.phaseJumps++;
		}
		rayPayload.rayDensity = rayPayload.rayDensity * abs(sq1*sq2);
	}


}



