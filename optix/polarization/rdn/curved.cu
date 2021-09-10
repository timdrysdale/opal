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

	//const uint2 idx = make_uint2(launchIndexTriangle.x, launchIndexTriangle.y); //[elevation, azimuth]
	//unsigned int s1=tea<8>(idx.y*launchDim.x+idx.x,1);
	//float rng1=rng(s1);	
	//if (rng1>0.5) {
	//		//Kill ray at the moment
	//		rayPayload.rhfr.z |= 1u<<FLAG_END_POSITION;
	//		//rayPayload.rhfr.z= FLAG_END;
	//		return;
	//}
	

	//Segment length from last physical interaction to this physical interaction (this is a surface, so this is a physical interaction)
	const float3 lastReflectionHitPoint = make_float3(rayPayload.lrhpd.x,rayPayload.lrhpd.y,rayPayload.lrhpd.z);
	const float s_prime=length(ch_triangle_data.hp-lastReflectionHitPoint);
	
        //Flag as hit on curved surface (not used to filter for RDN, keep it here in case we need stats)
	rayPayload.rhfr.z |= 1u<<FLAG_CURVED_MESH_POSITION;

	//Use reflections and hits to create hash

	//uint reflections=rayPayload.rhfr.x;
	//uint hits=rayPayload.rhfr.y;
	//uint hash=rayPayload.rhfr.w;
	//hash_combine_impl<uint>(hash,ch_triangle_data.faceId+reflections+hits);
	//hash_combine_impl<uint>(rayPayload.refhash,ch_triangle_data.faceId+rayPayload.reflections+rayPayload.hits);
	//rtPrintf("HASH \t%u\t%u\t%u\t%u\n",ch_triangle_date.faceId,rayPayload.reflections,rayPayload.hits,rayPayload.refhash);

	ReflectedRayBasis rrb = getReflectedRayBasis<CurvedTriangleHit>(ch_triangle_data,ray);

	//float3 reflection_dir; //Computed in the 
	uint reflections=updateTrianglePayload<RDNLPWavePayload,CurvedTriangleHit>(rayPayload,ch_triangle_data,ray,rrb.reflection_dir);        
        //Get the previous hit point: can be physical (on surface) or virtual (on receiver sphere)
//	const float3 lastHP=make_float3(rayPayload.hitPointAtt.x,rayPayload.hitPointAtt.y,rayPayload.hitPointAtt.z);
//	
//        //The rayLength is the distance from the last hit point to this hit point. Notice that, for instance, if the last hp is on a sphere receiver, there was no actual physical interaction with 
//	//any element. In that case, this rayLength is different from s_prime. It is only used to update the total distance (unfolded) of this ray. 
//	//TODO: keeping track of the total distance of the ray (unfolded distance) is only used in the basic setup, for curved surfaces it may be removed
//	const float rayLength=length(ch_triangle_data.hp-lastHP);
//
//	//Get the hitpoint from the barycentric coordinates computed in the triangle hit. This should get us a point always on the surface and help avoid self-intersection
//	//See https://www.realtimerendering.com/raytracinggems/ 6.1
//	//we could use t of ray, but if we shift the ray over the normal to avoid self-intersection we introduce an error in the electric field
//	//Update the last hitpoint on the ray payload. Attenuation is not used in this program
//	rayPayload.hitPointAtt.x =ch_triangle_data.hp.x;
//	rayPayload.hitPointAtt.y =ch_triangle_data.hp.y;
//	rayPayload.hitPointAtt.z =ch_triangle_data.hp.z;
//	//rtPrintf("THP\t%u\t%u\tbary=(%.6e,%.6e,%.6e)\n",launchIndexTriangle.x,launchIndexTriangle.y,ch_triangle_data.hp.x,ch_triangle_data.hp.y,ch_triangle_data.hp.z);
//	
//        //Get normal
//	const float3 gn=make_float3(ch_triangle_data.geom_normal_t.x,ch_triangle_data.geom_normal_t.y,ch_triangle_data.geom_normal_t.z);	
//	const float3 n = normalize(rtTransformNormal(RT_OBJECT_TO_WORLD,gn )); //Plane normal
//
//
//#ifdef OPAL_AVOID_SI
//	rayPayload.lastNormal=n;
//#endif	
//	const float3 reflection_dir=normalize(reflect(ray.direction, n));
//	const float aux=rayPayload.ndtd.w; //previous total distance of the ray
//	rayPayload.ndtd = make_float4(reflection_dir); //initialized with float3, w is set to 0. and updated below
//
//
//
//	//Update total distance of the ray so far	
//        rayPayload.ndtd.w = aux+ rayLength; 
//	
//        //Update last reflection (physical interaction in general) as this hit point
//	rayPayload.lrhpd =make_float4(ch_triangle_data.hp); //lastReflectionHitPoint;
//	
//        //Total distance till last reflection (more properly, last physical interaction) coincides in this case with total length of the ray
//	//This should be equal to s_prime+rayPayload.lrhpd.w. assert() would be great here
//	rayPayload.lrhpd.w = rayPayload.ndtd.w; //totalDistanceTillLastReflection;
//


	//Compute reflection coefficient

	//Incidence angle (ai) is defined with respect to the surface, we use the complementary, which is 90-ai, and is the angle between ray and normal
	//WARNING: Assuming ray direction is normalized: dot(r,n)=cos(angle(r,n))
	//We use the fabs() because if a ray hits an internal face, the normal is reversed. The cos would be negative. For "closed" meshes this should not happen. However, in the borders, due to precision
	//it happens: a ray is not detected as hitting a face and gets inside the mesh, hitting an internal face later.
	//With penetration we can hit internal faces in closed meshes. This way, we also get the correct incidence angle again.
	//float cosA = fabs(dot(-ray.direction, n));

	//Perfect conductor
	//const float2 Rnorm=make_float2(-1.0f,0.0);
	//const float2 Rpar=make_float2(-1.0f,0.0);
	//float2 Rnorm;
	//float2 Rpar;
	////getPerfectConductor(Rnorm,Rpar);
	//getReflectionCoefficient(cosA, EMProperties,Rnorm,Rpar);
        const float4 rc=getReflectionCoefficient<RDNLPWavePayload>(rayPayload,rrb.cosA, EMProperties);
        //const float4 rc=getPerfectConductor();

	//float3 anorm_i;
	//float3 apar_i; 
	updateReflectionCoefficient<RDNLPWavePayload>(rayPayload,ray,rrb, rc);
        //Compute local incidence coordinate system for reflection (components parallel and normal to the incidence plane)
	//const float3 anorm_i=normalize(cross(ray.direction,n));
	//const float3 apar_i=normalize(cross(anorm_i,ray.direction)); 
	////rtPrintf("IRG\t%u\t%u\tn=(%.6e,%.6e,%.6e)|anorm_i|=(%.6e,%.6e,%.6e)=%.6e\t|apar_i|=(%.6e,%.6e,%.6e)=%.6e \n",launchIndexTriangle.x,launchIndexTriangle.y,n.x,n.y,n.z,anorm_i.x,anorm_i.y,anorm_i.z,length(anorm_i),apar_i.x,apar_i.y,apar_i.z,length(apar_i));



	////Reflected ray basis
	//const float3 anorm_r=anorm_i; 
	//const float3 apar_r=cross(anorm_r,reflection_dir); //Should change the direction with respect to the incidence parallel
	////rtPrintf("RRG\t%u\t%u\tn=(%.6e,%.6e,%.6e)|anorm_r|=(%.6e,%.6e,%.6e)=%.6e\t|apar_r|=(%.6e,%.6e,%.6e)=%.6e \n",launchIndexTriangle.x,launchIndexTriangle.y,n.x,n.y,n.z,anorm_r.x,anorm_r.y,anorm_r.z,length(anorm_r),apar_r.x,apar_r.y,apar_r.z,length(apar_r));



	////Get geometric  components, multiply by previous coefficients  and multiply by reflection coefficients computed above or transmission coefficients below

	////Geometric part normal
	//const float2 Einorm=sca_complex_prod(dot(rayPayload.hor_v,anorm_i),rayPayload.hor_coeff) + sca_complex_prod(dot(rayPayload.ver_v,anorm_i),rayPayload.ver_coeff);
	////Geometric part parallel
	//const float2 Eipar=sca_complex_prod(dot(rayPayload.hor_v,apar_i),rayPayload.hor_coeff) + sca_complex_prod(dot(rayPayload.ver_v,apar_i),rayPayload.ver_coeff);

	////Penetration is not implemented and disabled
	////	if ((usePenetration==1u) && (reflections<max_interactions)) {
	////		//Penetration in a curved surface requires a similar analysis as below, but with the corresponding curvature matrix, see for instance Deschamps,"Ray Techniques in Electromagnetics", Proceedings of the IEEE, 60(9),1972, sect. III.A
	////		//Raise an exception...
	////		rtThrow(GLOBALHITBUFFEROVERFLOW);
	////		//if they are disabled, just return printing something and hope someone notices..
	////		printf("Penetration for curved surfaces is not implemented yet");
	////		return;
	////	}
	////Update here reflection coefficient, otherwise we multiply reflection and transmission in the transmission above

	////New horizontal (normal)  coefficient
	////rtPrintf("TT\t%u\t%u\tRnorm(%.6e,%.6e)  Rpar(%.6e,%.6e) hash=%u\n",launchIndexTriangle.x,launchIndexTriangle.y,rayPayload.hor_coeff.x,rayPayload.hor_coeff.y,rayPayload.ver_coeff.x,rayPayload.ver_coeff.y, rayPayload.rhfr.w);
	//rayPayload.hor_coeff=complex_prod(Einorm,Rnorm);
	////New vertical (parallel)  coefficient
	//rayPayload.ver_coeff=complex_prod(Eipar,Rpar);


	
	//Copy before updating below
	float rho1=rayPayload.radii.x;
	float rho2=rayPayload.radii.y;

	float3 u1=make_float3(ch_triangle_data.principalDirection1.x,ch_triangle_data.principalDirection1.y,ch_triangle_data.principalDirection1.z);	
	float3 u2=make_float3(ch_triangle_data.principalDirection2.x,ch_triangle_data.principalDirection2.y,ch_triangle_data.principalDirection2.z);		
	float R1=ch_triangle_data.principalDirection1.w;  
	float R2=ch_triangle_data.principalDirection2.w;
	updateCurvature<RDNLPWavePayload>(rayPayload,rrb,s_prime,reflections,u1,u2,R1,R2,false);
	//Finaly, update density
	if (reflections==0) {
		rayPayload.rayDensity =rayPayload.rayDensity/(s_prime*s_prime); //Initial ray density already includes rayNumber/solidAngle
	} else {
		float sq1=rho1/(rho1+s_prime);
		float sq2=rho2/(rho2+s_prime);
		//What if caustics are infinity
		//If we apply the L'Hôpital rule, the fraction goes to 1...
		if (isinf(rho1)) {
			sq1=1;
		}
		if (isinf(rho2)) {
			sq2=1;
		}
		if ((sq1*sq2)<0) {
			rayPayload.phaseJumps++;
		}
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


		rayPayload.rayDensity = rayPayload.rayDensity * abs(sq1*sq2);
	}

	
	//Curvature information. Has to be defined externally for the mesh. Cannot be deduced from the triangles.
//	const float3 u1=make_float3(ch_triangle_data.principalDirection1.x,ch_triangle_data.principalDirection1.y,ch_triangle_data.principalDirection1.z);	
//	const float3 u2=make_float3(ch_triangle_data.principalDirection2.x,ch_triangle_data.principalDirection2.y,ch_triangle_data.principalDirection2.z);		
//	const float R1=ch_triangle_data.principalDirection1.w;  
//	const float R2=ch_triangle_data.principalDirection2.w;
//	float3 x1=rayPayload.p1;
//	float3 x2=rayPayload.p2;	
//	float irho1=0.0f;
//	float irho2=0.0f;
//	//TODO: there is something arbitrary here and I do not know what to do. It is as follows:
//	//Principal directions (x1 and x2) for the incident ray: for a spherical wave any pair of perpendicular vectors as well as perpendicular to the ray
//	//Then, depending on the choice of those directions, the resulting reflected principal directions may be zero (or nan). But changing the selection of the original 
//	// principal directions may result in perfectly valid reflected principal directions... what to do? This is only valid for the first segment, once the 
//	// reflected principal direction has been computed, there should be no choice
//	// See commented code below
//	if (reflections==0) {
//		//For the first hit, we should incorporate the first segment attenuation to the divergence, just 1/unfolded_path
//		rayPayload.divergence=rayPayload.divergence*(1.0f/(s_prime)); //The total distance has already been updated.
//		rayPayload.rayDensity =rayPayload.rayDensity/(s_prime*s_prime); //Initial ray density already includes rayNumber/solidAngle
//		//Just use as principal directions the field vector directions
//		x1=rrb.apar_i;
//		x2=rrb.anorm_i;
//	
//		//No need to compute this below, but the performance does not seem to be affected anyway
//		//The principal directions used in the Kouyoumjian and Pathak NASA tech report used to derive the simplification for spherical waves (see Balanis also)
//		//Vector of plane of incidence
//		//float cosAA=dot(-ray.direction,n);
//		//float3 ni=normalize(cross(ray.direction,n));
//		////Vector of principal plane n,u2
//		//float3 nu2=normalize(cross(n,u2));
//		////Angle between those two planes
//		////float3 par=normalize(reflection_dir-dot(reflection_dir,n)*n);
//		////float cosw=dot(u2,par);
//		//float cosw=dot(ni,nu2);
//		//float sinw=sqrt(1.0f-(cosw*cosw));
//		//float sinA=sqrt(1.0f-(cosAA*cosAA));
//		////x1=(-cosAA*sinw)*u1+(cosAA*cosw)*u2+sinA*n;
//		////x1=normalize(cosA*ray.direction-sinA*n);
//		//x2=cosw*u1+sinw*u2;
//		//x1=cross(ray.direction,x2);
//		//x1=normalize(rayPayload.p1-dot(rayPayload.p1,ni)*ni);
//		//x2=normalize(cross(ray.direction,x1));
//
//		//	rtPrintf("COS cosAA=%f acosAA=%f cosA=%f acosA=%f\n",cosAA,(acosf(cosAA)*57.29577),cosA,(acosf(cosA)*57.29577));
//		//	rtPrintf("CH ray=(%f,%f,%f) |ray|=%f x1=(%f,%f,%f) |x1|=%f x2=(%f,%f,%f) |x2|=%f \n",ray.direction.x,ray.direction.y,ray.direction.z,length(ray.direction),x1.x,x1.y,x1.z,length(x1),x2.x,x2.y,x2.z,length(x2));
//		//	rtPrintf("CH x1=(%f,%f,%f) |x1|=%f x2=(%f,%f,%f) |x2|=%f acosw=%f asinw=%f acosA=%f asinA=%f\n",x1.x,x1.y,x1.z,length(x1),x2.x,x2.y,x2.z,length(x2),(acosf(cosw)*57.29577),(asinf(sinw)*57.29577),(acosf(cosA)*57.29577),asinf(sinA));
//		//	rtPrintf("DOT dot(ni,x1)=%f dot(x1,x2)=%f dot(ray.direction,x1)=%f dot(ray,direction,x2)=%f dot(n,x1)=%f dot(n,x2)=%f dot(u1,n)=%f dot(u2,n)=%f\n",dot(ni,x1), dot(x1,x2), dot(-ray.direction,x1), dot(-ray.direction,x2), dot(n,x1), dot(n,x2),dot(u1,n), dot(u2,n));
//		//	rtPrintf("COS th11=%f th12=%f th21=%f th22=%f td=%f\n",-cosAA*sinw,cosAA*cosw,cosw,sinw,-cosAA);
//		//rayPayload.rayDensity =rayPayload.rayDensity/(rayLength*rayLength); //Initial ray density already includes rayNumber/solidAngle
//	}
//	bool valid= computePrincipalDirections<RDNLPWavePayload>(rayPayload,u1,u2,R1,R2,s_prime,rrb.cosA, rrb.reflection_dir,rrb.normal,x1,x2,irho1,irho2,launchIndexTriangle,reflections);
//	if (!valid) {
//		//At the moment I just ignore and kill this ray from now on
//		rtPrintf("Not valid ray=(%f,%f,%f) \n", ray.direction.x,ray.direction.y,ray.direction.z );
//		rayPayload.rhfr.z |= 1u<<FLAG_END_POSITION;
//
//		atomicAdd(&invalidRaysBuffer[0],1);
//		//	rayPayload.rhfr.z= FLAG_END;
//		return;
//
//	}
//
//	//	float2 radiir=computeRadiiSpherical<CurvedMeshLPWavePayload>(rayPayload,u1,u2,R1,R2,rayLength,cosA, ray.direction,launchIndexTriangle,reflections);
//	//rtPrintf("RADII rho1=%f rho2=%f 1/rho1=%f 1/rho2=%f\n",radiir.x,radiir.y, (1.0f/radiir.x), (1.0f/radiir.y));
//	//	rtPrintf("RAD\t%u\t%u\t%f\t%f\t%f\t%f\t%f\n",launchIndexTriangle.x,launchIndexTriangle.y, ray.direction.x,ray.direction.y,ray.direction.z, radiir.x,radiir.y);
//	if (reflections > 0u) {
//
//		//We have to incorporate the previous segment
//		float rho1=rayPayload.radii.x;
//		float rho2=rayPayload.radii.y;
//		float2 divergence=computeDivergence(rho1,rho2,s_prime,rayPayload.divergence,launchIndexTriangle.x,launchIndexTriangle.y,reflections);
//		if (isnan(divergence.x) || isnan(divergence.y)) {
//			rtPrintf("CDR \t%u\t%u\tref=%d ray=(%f,%f,%f)\n",launchIndexTriangle.x,launchIndexTriangle.y,reflections, ray.direction.x,ray.direction.y,ray.direction.z); 
//			//Kill ray at the moment
//			rayPayload.rhfr.z |= 1u<<FLAG_END_POSITION;
//			//rayPayload.rhfr.z= FLAG_END;
//			return;
//		}	
//		rayPayload.divergence=divergence;
//
//		//Update ray density
//		float sq1=rho1/(rho1+s_prime);
//		float sq2=rho2/(rho2+s_prime);
//		//What if caustics are infinity
//		//If we apply the L'Hôpital rule, the fraction goes to 1...
//		if (isinf(rho1)) {
//			sq1=1;
//		}
//		if (isinf(rho2)) {
//			sq2=1;
//		}
//		rayPayload.rayDensity = rayPayload.rayDensity * abs(sq1*sq2);
//	}
//	
//
//        //Update payload now
//	//rtPrintf("CC r=%u irho1=%f,irho2=%f\n", reflections,irho1,irho2);
//	rayPayload.radii=make_float2(1.0f/irho1,1.0f/irho2);
//	//rayPayload.radii=radiir;
//	//rtPrintf("CC r=%u rho1=%f,rho2=%f\n", reflections,radiir.x,radiir.y);
//	//rtPrintf("CC r=%u rho1=%f,rho2=%f\n", reflections,(1.0f/irho1),(1.0f/irho2));
//	//rayPayload.updateDivergence+=1u;	
//
//	rayPayload.p1=x1;
//	rayPayload.p2=x2;	
//


	//Update vectors
//	rayPayload.ver_v=apar_r;
//	rayPayload.hor_v=anorm_r;
	//++rayPayload.reflections;
	//++reflections;
	//rayPayload.rhfr=make_uint4(reflections,hits,rayPayload.rhfr.z,hash);

}




