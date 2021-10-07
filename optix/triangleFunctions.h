/***************************************************************/
//
//Copyright (c) 2021 Esteban Egea-Lopez http://girtel.upct.es/~eegea
//
/**************************************************************/
/**********************************************************
 *
 * Functions for triangle interactions. 
 * 
 * ********************************************************/
#ifndef TRIANGLEFUNCTIONS_H
#define TRIANGLEFUNCTIONS_H

#include "../Common.h"
#include <optix_world.h>

//Update rayPayload on triangle hit and returns the number of reflections so far (before increasing)
	template<class P, class T>
__forceinline__ __device__ uint updateTrianglePayload(P& rayPayload, T& ch_triangle_data, const optix::Ray& ray, const float3& reflection_dir) {
        //Get the previous hit point: can be physical (on surface) or virtual (on receiver sphere)
	const float3 lastHP=make_float3(rayPayload.hitPointAtt.x,rayPayload.hitPointAtt.y,rayPayload.hitPointAtt.z);
	//The rayLength is the distance from the last hit point to this hit point. Notice that, for instance, if the last hp is on a sphere receiver, there was no actual physical interaction with 
	//any element. In that case, this rayLength is different from s_prime. It is only used to update the total distance (unfolded) of this ray. 
	//TODO: keeping track of the total distance of the ray (unfolded distance) is only used in the basic setup, for curved surfaces it may be removed
	const float rayLength=length(ch_triangle_data.hp-lastHP);
	//Get the hitpoint from the barycentric coordinates computed in the triangle hit. This should get us a point always on the surface and help avoid self-intersection
	//See https://www.realtimerendering.com/raytracinggems/ 6.1
	//we could use t of ray, but if we shift the ray over the normal to avoid self-intersection we introduce an error in the electric field
	//Update the last hitpoint on the ray payload. Attenuation is not used in this program
	rayPayload.hitPointAtt.x =ch_triangle_data.hp.x;
	rayPayload.hitPointAtt.y =ch_triangle_data.hp.y;
	rayPayload.hitPointAtt.z =ch_triangle_data.hp.z;
	//printf("THP\t%u\t%u\tbary=(%.6e,%.6e,%.6e) %u\n",launchIndexTriangle.x,launchIndexTriangle.y,ch_triangle_data.hp.x,ch_triangle_data.hp.y,ch_triangle_data.hp.z, ch_triangle_data.faceId);
	
        //Get normal
//	const float3 gn=make_float3(ch_triangle_data.geom_normal_t.x,ch_triangle_data.geom_normal_t.y,ch_triangle_data.geom_normal_t.z);	
//	float3 n = normalize(rtTransformNormal(RT_OBJECT_TO_WORLD,gn )); //Plane normal

//#ifdef OPAL_AVOID_SI
//	rayPayload.lastNormal=n;
//#endif	
	//reflection_dir=normalize(reflect(ray.direction, n));
	const float aux=rayPayload.ndtd.w; //previous total distance of the ray
	rayPayload.ndtd = make_float4(reflection_dir); //initialized with float3, w is set to 0. and updated below



	//Update total distance of the ray so far	
        rayPayload.ndtd.w = aux+ rayLength; 
	
        //Update last reflection (physical interaction in general) as this hit point
	rayPayload.lrhpd =make_float4(ch_triangle_data.hp); //lastReflectionHitPoint;
	
        //Total distance till last reflection (more properly, last physical interaction) coincides in this case with total length of the ray
	//This should be equal to s_prime+rayPayload.lrhpd.w. assert() would be great here
	rayPayload.lrhpd.w = rayPayload.ndtd.w; //totalDistanceTillLastReflection;
	
	uint reflections=rayPayload.rhfr.x;
	uint hits=rayPayload.rhfr.y;
	uint hash=rayPayload.rhfr.w;
	hash_combine_impl<uint>(hash,ch_triangle_data.faceId+reflections+hits);
	rayPayload.rhfr=make_uint4(reflections+1,hits,rayPayload.rhfr.z,hash);
        return reflections; //We usually need it for further checks
	
//	return n;

}
//	template<class P>
//__forceinline__ __device__ void updateReflectionCoefficient(P& rayPayload, optix::Ray& ray, float2& Rnorm, float2& Rpar, float3& anorm_i, float3& apar_i ) {
//
//	//Compute local incidence coordinate system for reflection (components parallel and normal to the incidence plane)
//	anorm_i=normalize(cross(ray.direction,n));
//	apar_i=normalize(cross(anorm_i,ray.direction)); 
//	//rtPrintf("IRG\t%u\t%u\tn=(%.6e,%.6e,%.6e)|anorm_i|=(%.6e,%.6e,%.6e)=%.6e\t|apar_i|=(%.6e,%.6e,%.6e)=%.6e \n",launchIndexTriangle.x,launchIndexTriangle.y,n.x,n.y,n.z,anorm_i.x,anorm_i.y,anorm_i.z,length(anorm_i),apar_i.x,apar_i.y,apar_i.z,length(apar_i));
//
//
//
//	//Reflected ray basis
//	const float3 anorm_r=anorm_i; 
//	const float3 apar_r=cross(anorm_r,reflection_dir); //Should change the direction with respect to the incidence parallel
//	//rtPrintf("RRG\t%u\t%u\tn=(%.6e,%.6e,%.6e)|anorm_r|=(%.6e,%.6e,%.6e)=%.6e\t|apar_r|=(%.6e,%.6e,%.6e)=%.6e \n",launchIndexTriangle.x,launchIndexTriangle.y,n.x,n.y,n.z,anorm_r.x,anorm_r.y,anorm_r.z,length(anorm_r),apar_r.x,apar_r.y,apar_r.z,length(apar_r));
//
//
//
//	//Get geometric  components, multiply by previous coefficients  and multiply by reflection coefficients computed above or transmission coefficients below
//
//	//Geometric part normal
//	const float2 Einorm=sca_complex_prod(dot(rayPayload.hor_v,anorm_i),rayPayload.hor_coeff) + sca_complex_prod(dot(rayPayload.ver_v,anorm_i),rayPayload.ver_coeff);
//	//Geometric part parallel
//	const float2 Eipar=sca_complex_prod(dot(rayPayload.hor_v,apar_i),rayPayload.hor_coeff) + sca_complex_prod(dot(rayPayload.ver_v,apar_i),rayPayload.ver_coeff);
//
//
//	//New horizontal (normal)  coefficient
//	//rtPrintf("TT\t%u\t%u\tRnorm(%.6e,%.6e)  Rpar(%.6e,%.6e) hash=%u\n",launchIndexTriangle.x,launchIndexTriangle.y,rayPayload.hor_coeff.x,rayPayload.hor_coeff.y,rayPayload.ver_coeff.x,rayPayload.ver_coeff.y, rayPayload.rhfr.w);
//	rayPayload.hor_coeff=complex_prod(Einorm,Rnorm);
//	//New vertical (parallel)  coefficient
//	rayPayload.ver_coeff=complex_prod(Eipar,Rpar);
//
//	//Update vectors
//	rayPayload.ver_v=apar_r;
//	rayPayload.hor_v=anorm_r;
//}



#endif
