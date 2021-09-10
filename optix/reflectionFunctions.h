/***************************************************************/
//
//Copyright (c) 2021 Esteban Egea-Lopez http://girtel.upct.es/~eegea
//
/**************************************************************/
/**********************************************************
 *
 * Functions for reflection interactions . 
 * 
 * ********************************************************/
#ifndef REFLECTIONFUNCTIONS_H
#define REFLECTIONFUNCTIONS_H

#include "../Common.h"
#include "configuration.h"
#include "Complex.h"
#include "traceFunctions.h"
#include <optix_world.h>
#include  <optixu/optixu_matrix_namespace.h>

struct ReflectedRayBasis {
        __forceinline__ __device__ ReflectedRayBasis(const float3& n, optix::Ray& ray) {
		normal=n;
		//Compute reflection direction
		reflection_dir=normalize(reflect(ray.direction, n));

		//Incidence angle (ai) is defined with respect to the surface, we use the complementary, which is 90-ai, and is the angle between ray and normal
		//WARNING: Assuming ray direction is normalized: dot(r,n)=cos(angle(r,n))
		//We use the fabs() because if a ray hits an internal face, the normal is reversed. The cos would be negative. For "closed" meshes this should not happen. However, in the borders, due to precision
		//it happens: a ray is not detected as hitting a face and gets inside the mesh, hitting an internal face later.
		//With penetration we can hit internal faces in closed meshes. This way, we also get the correct incidence angle again.
		cosA = fabs(dot(-ray.direction, n));

		//Compute local incidence coordinate system for reflection (components parallel and normal to the incidence plane)
		anorm_i=normalize(cross(ray.direction,n));
		apar_i=normalize(cross(anorm_i,ray.direction)); 
		//rtPrintf("IRG\t%u\t%u\tn=(%.6e,%.6e,%.6e)|anorm_i|=(%.6e,%.6e,%.6e)=%.6e\t|apar_i|=(%.6e,%.6e,%.6e)=%.6e \n",launchIndexTriangle.x,launchIndexTriangle.y,n.x,n.y,n.z,anorm_i.x,anorm_i.y,anorm_i.z,length(anorm_i),apar_i.x,apar_i.y,apar_i.z,length(apar_i));



		//Reflected ray basis
		anorm_r=anorm_i; 
		apar_r=cross(anorm_r,reflection_dir); //Should change the direction with respect to the incidence parallel
		//rtPrintf("RRG\t%u\t%u\tn=(%.6e,%.6e,%.6e)|anorm_r|=(%.6e,%.6e,%.6e)=%.6e\t|apar_r|=(%.6e,%.6e,%.6e)=%.6e \n",launchIndexTriangle.x,launchIndexTriangle.y,n.x,n.y,n.z,anorm_r.x,anorm_r.y,anorm_r.z,length(anorm_r),apar_r.x,apar_r.y,apar_r.z,length(apar_r));


	}
	 float3 anorm_i;
	 float3 apar_i;
	 float3 anorm_r;
	 float3 apar_r;
	 float3 reflection_dir;
	 float3 normal;
         float cosA;

};

	template<class T>
__forceinline__ __device__  ReflectedRayBasis  getReflectedRayBasis(T& ch_triangle_data, optix::Ray& ray) {
        //Get normal
	const float3 gn=make_float3(ch_triangle_data.geom_normal_t.x,ch_triangle_data.geom_normal_t.y,ch_triangle_data.geom_normal_t.z);	
	const float3 n = normalize(rtTransformNormal(RT_OBJECT_TO_WORLD,gn )); //Plane normal
        return ReflectedRayBasis(n,ray);

}
__forceinline__ __device__ float4 getPerfectConductor() {
	float2 Rnorm=make_float2(-1.0f,0.0);
	//float2 Rpar=make_float2(-1.0f,0.0); //Didascalou uses -1 here in its perfect conductor
	float2 Rpar=make_float2(1.0f,0.0);
        return make_float4(Rnorm.x,Rnorm.y,Rpar.x,Rpar.y);

}

 __forceinline__ __device__ float2 ITUParametersToDielectricConstant(float k, MaterialEMProperties EMProperties) {
	const float m_2_pi=6.283185307179586f;
	const float waveLength = m_2_pi/k;
	const float frequency= speedOfLight/waveLength; 
	float relativePermitivity;
	const float a =EMProperties.ITUParameters.x;
	const float b =EMProperties.ITUParameters.y;
	const float c =EMProperties.ITUParameters.z;
	const float d =EMProperties.ITUParameters.w;
	if (b==0) {
		relativePermitivity = a;
	}
	else {
		relativePermitivity=a*powf((frequency / 1.0e9f), b); //Frequency in GHz
	}
	float conductivity;
	if (d == 0) {
		conductivity = c;
	}
	else {
		conductivity = c*powf((frequency / 1.0e9f), d); //Frequency in GHz
	}
	return make_float2(relativePermitivity,-60.0f*waveLength*conductivity);
}
	template<class P>
 __forceinline__ __device__ float2 getDielectricConstant(P& rayPayload, MaterialEMProperties EMProperties) {
	if (useMultichannel) {
		return ITUParametersToDielectricConstant(rayPayload.polarization_k.w,EMProperties);
	} else {
		return EMProperties.dielectricConstant;
	}
}
	//Returns the reflection coefficients as two complex numbers [Rnorm,Rpar] 
	template<class P>
__forceinline__ __device__ float4 getReflectionCoefficient(P& rayPayload, float cosA, MaterialEMProperties EMProperties) {
	//Incidence angle (ai) is  defined  in many books with respect to the surface, we use the complementary, which is 90-ai, and is the angle between ray and surface normal, so cosA is the cosine of the angle between
	//normal and ray
	//WARNING: Assuming ray direction is normalized: dot(r,n)=cos(angle(r,n))
	////Compute the reflection coefficients (only depend on incidence angle and dielectric properties)
	////Complex arithmetic: sum
	float2 dielectricConstant = getDielectricConstant<P>(rayPayload,EMProperties);
	if (isinf(dielectricConstant.y)) {
		//Perfect conductor
		return getPerfectConductor();
	}
	float2 argument = make_float2(dielectricConstant.x + (cosA*cosA) - 1.0f, dielectricConstant.y);
	float2 root = complex_sqrt(argument);
	
	
	//Soft reflection. 
	//Normal reflection coefficient (Electric field not in plane of incidence)
	float2 Rnorm = complex_div(make_float2(cosA-root.x,-root.y),make_float2(cosA+root.x,root.y));


	//Reflection info log (to be used in external programs)
	//rtPrintf("H\t%u\t%u\t%d\t%d\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\n", launchIndexTriangle.x, launchIndexTriangle.y, rayPayload.reflections, rayPayload.faceId, argument.x, argument.y, root.x, root.y, R.x, R.y, tR.x, tR.y);


	//Hard reflection.  
	//Parallel reflection coefficient (Electric field in plane of incidence)
	float2 num = sca_complex_prod(cosA, dielectricConstant);
	float2 div=num;
	num -=root;
	div +=root;
	
	
	float2 Rpar = complex_div(num, div);
        return make_float4(Rnorm.x,Rnorm.y,Rpar.x,Rpar.y);

}	

 /* Pass reflection coefficients [Rnorm,Rpar]*/
	template<class P>
 __forceinline__ __device__ void updateReflectionCoefficient(P& rayPayload, optix::Ray& ray, ReflectedRayBasis& rrb, const float4& rc) {
	//Get reflection coefficients
	const float2 Rnorm=make_float2(rc.x,rc.y);
	const float2 Rpar=make_float2(rc.z,rc.w);

	//Compute local incidence coordinate system for reflection (components parallel and normal to the incidence plane)
	const float3 anorm_i=rrb.anorm_i;
	const float3 apar_i=rrb.apar_i;
	//rtPrintf("IRG\t%u\t%u\tn=(%.6e,%.6e,%.6e)|anorm_i|=(%.6e,%.6e,%.6e)=%.6e\t|apar_i|=(%.6e,%.6e,%.6e)=%.6e \n",launchIndexTriangle.x,launchIndexTriangle.y,rrb.normal.x,rrb.normal.y,rrb.normal.z,anorm_i.x,anorm_i.y,anorm_i.z,length(anorm_i),apar_i.x,apar_i.y,apar_i.z,length(apar_i));



	//Reflected ray basis
	const float3 anorm_r=rrb.anorm_r;
	const float3 apar_r=rrb.apar_r;
	//rtPrintf("RRG\t%u\t%u\tn=(%.6e,%.6e,%.6e)|anorm_r|=(%.6e,%.6e,%.6e)=%.6e\t|apar_r|=(%.6e,%.6e,%.6e)=%.6e \n",launchIndexTriangle.x,launchIndexTriangle.y,n.x,n.y,n.z,anorm_r.x,anorm_r.y,anorm_r.z,length(anorm_r),apar_r.x,apar_r.y,apar_r.z,length(apar_r));



	//Get geometric  components, multiply by previous coefficients  and multiply by reflection coefficients computed above or transmission coefficients below

	//Geometric part normal
	const float2 Einorm=sca_complex_prod(dot(rayPayload.hor_v,anorm_i),rayPayload.hor_coeff) + sca_complex_prod(dot(rayPayload.ver_v,anorm_i),rayPayload.ver_coeff);
	//Geometric part parallel
	const float2 Eipar=sca_complex_prod(dot(rayPayload.hor_v,apar_i),rayPayload.hor_coeff) + sca_complex_prod(dot(rayPayload.ver_v,apar_i),rayPayload.ver_coeff);


	//New horizontal (normal)  coefficient
	//rtPrintf("TT\t%u\t%u\tRnorm(%.6e,%.6e)  Rpar(%.6e,%.6e) hash=%u\n",launchIndexTriangle.x,launchIndexTriangle.y,rayPayload.hor_coeff.x,rayPayload.hor_coeff.y,rayPayload.ver_coeff.x,rayPayload.ver_coeff.y, rayPayload.rhfr.w);
	rayPayload.hor_coeff=complex_prod(Einorm,Rnorm);
	//New vertical (parallel)  coefficient
	rayPayload.ver_coeff=complex_prod(Eipar,Rpar);

	//Update vectors
	rayPayload.ver_v=apar_r;
	rayPayload.hor_v=anorm_r;
}
//For penetration (transmission at the interface)

	template<class P>
 __forceinline__ __device__ void updateTransmissionCoefficient(P& rayPayload, optix::Ray& ray, ReflectedRayBasis& rrb, const float4& rc) {
	//Get reflection coefficients
	const float2 Rnorm=make_float2(rc.x,rc.y);
	const float2 Rpar=make_float2(rc.z,rc.w);

	//Compute local incidence coordinate system for reflection (components parallel and normal to the incidence plane)
	const float3 anorm_i=rrb.anorm_i;
	const float3 apar_i=rrb.apar_i;
	//rtPrintf("IRG\t%u\t%u\tn=(%.6e,%.6e,%.6e)|anorm_i|=(%.6e,%.6e,%.6e)=%.6e\t|apar_i|=(%.6e,%.6e,%.6e)=%.6e \n",launchIndexTriangle.x,launchIndexTriangle.y,n.x,n.y,n.z,anorm_i.x,anorm_i.y,anorm_i.z,length(anorm_i),apar_i.x,apar_i.y,apar_i.z,length(apar_i));



	//Reflected ray basis
	//const float3 anorm_r=rrb.anorm_r;
	//const float3 apar_r=rrb.apar_r;
	//rtPrintf("RRG\t%u\t%u\tn=(%.6e,%.6e,%.6e)|anorm_r|=(%.6e,%.6e,%.6e)=%.6e\t|apar_r|=(%.6e,%.6e,%.6e)=%.6e \n",launchIndexTriangle.x,launchIndexTriangle.y,n.x,n.y,n.z,anorm_r.x,anorm_r.y,anorm_r.z,length(anorm_r),apar_r.x,apar_r.y,apar_r.z,length(apar_r));



	//Get geometric  components, multiply by previous coefficients  and multiply by reflection coefficients computed above or transmission coefficients below

	//Geometric part normal
	const float2 Einorm=sca_complex_prod(dot(rayPayload.hor_v,anorm_i),rayPayload.hor_coeff) + sca_complex_prod(dot(rayPayload.ver_v,anorm_i),rayPayload.ver_coeff);
	//Geometric part parallel
	const float2 Eipar=sca_complex_prod(dot(rayPayload.hor_v,apar_i),rayPayload.hor_coeff) + sca_complex_prod(dot(rayPayload.ver_v,apar_i),rayPayload.ver_coeff);


	//New horizontal (normal)  coefficient
	rayPayload.hor_coeff=complex_prod(Einorm,make_float2(1.0f+Rnorm.x,Rnorm.y));
	//New vertical (parallel)  coefficient
	rayPayload.ver_coeff=complex_prod(Eipar,make_float2(1.0f+Rpar.x,Rpar.y));

	//Update vectors. Assuming they are equal to incident vectors
	rayPayload.ver_v=apar_i;
	rayPayload.hor_v=anorm_i;
}
template<class P>
__forceinline__ __device__ void penetrationRay(P& rayPayload, optix::Ray& ray, ReflectedRayBasis& rrb, float3& hp, const float4& rc, MaterialEMProperties EMProperties,  uint i_x, uint i_y) {
	//Trace a penetration ray as a new ray. Recursive tracing, check stack depth>max_interactions
	//Quickly check for attenuation in dB, if att has a very low value we do not trace. Also, we do not want to overflow the float in the operations and get a nan.
	//Apply material attenuation. Again, we assume the material is not bending the ray in any direction

	//Typical values are -15 dB for 203 mm at 5 GHz => -75 dB/m
	//Considering real distance travelled through material
	float dbAtt=(EMProperties.tattenuation.x/rrb.cosA)*(EMProperties.tattenuation.y); //Attenuation in dB (power) distance*att/m = thickness/cosA*att/m

	//Considering that material has been travelled in perpendicular
	//float dbAtt=(EMProperties.tattenuation.x)*(EMProperties.tattenuation.y);
	//float tAtt=rayPayload.accumulatedAttenuation + dbAtt; //Accumulate in log scale to avoid overflows
	float tAtt=rayPayload.hitPointAtt.w + dbAtt; //Accumulate in log scale to avoid overflows
	if (tAtt>attenuationLimit) {
		//Copy payload
		P penPayload=rayPayload;
		penPayload.ndtd  = optix::make_float4(0, 0, 0, rayPayload.ndtd.w);
		//penPayload.hits=rayPayload.hits+1;
		//penPayload.flags = FLAG_NONE;
		uint reflections=rayPayload.rhfr.x;
		uint hits=rayPayload.rhfr.y;
		uint hash=rayPayload.rhfr.w;
		hash_combine_impl<uint>(hash,ch_triangle_data.faceId+reflections+hits);
		penPayload.rhfr=make_uint4(reflections,hits+1u,FLAG_NONE,hash);
		//Assuming the media on both sides of the plane are the same (air, most likely), then the incidence angle is equal to the transmission angle, so the ray does not change trajectory
		//Otherwise, we have to rotate the ray by the transmission angle, where a_t (angle_transmission) and theta= 90-a_t, with respect to the vector ortoghonal to the normal and ray,
		//that is the normal vector of the plane defined by ray and mesh face normal.
		//We can use Rodrigues formula to rotate the vector given angle and unit vector e,  to avoid using rotating matrix
		//So, e = cross(normal,-ray), remember right hand rule and check this... already should be a unit vector since ray and normal are normalized
		//ray_rot=cos(theta)*ray + sin(theta)*(cross(e,ray)+(1-cos(theta)(dot(e,ray))e. 
		//Have to compute the cos and sin of theta=90-a_
		//Check the above
		//Assume equal media
		//Transmission coefficient (1+reflection coefficient)
		//
		updateTransmissionCoefficient<P>(penPayload,ray,rrb,rc);

		//penPayload.prodReflectionCoefficient = complex_prod(rayPayload.prodReflectionCoefficient,make_float2(1.0f+R.x, R.y)); 
		//Attenuation
		penPayload.hitPointAtt.w = tAtt;
		//penPayload.accumulatedAttenuation = tAtt;
		//rtPrintf("AT cosA=%f\tatt=%f\ttAtt=%f\nr=(%f,%f,%f)\tn=(%f,%f,%f)\n",cosA,dbAtt,tAtt,ray.direction.x,ray.direction.y,ray.direction.z,n.x,n.y,n.z);
		traceReflection<P>(penPayload, ray.ray_type, hp, ray.direction,i_x,i_y, false);
	}
}

#endif
