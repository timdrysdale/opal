/***************************************************************/
//
//Copyright (c) 2019 Esteban Egea-Lopez http://ait.upct.es/eegea
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
#include "../../Complex.h"
#include "../../traceFunctions.h"
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

//Penetration configuration
rtDeclareVariable(uint, usePenetration, , );
rtDeclareVariable(float, attenuationLimit, , );

RT_PROGRAM void closestHitTriangle()
{

	//Update payload
	//const float rayLength = ch_triangle_data.geom_normal_t.w;
	//const float3 hp= ray.origin + rayLength * ray.direction ;
	//rayPayload.hitPoint =hp;
	//Get the hitpoint from the barycentric coordinates computed in the triangle hit. This should get us a point always on the surface and help avoid self-intersection
	//See https://www.realtimerendering.com/raytracinggems/ 6.1
	//const float rayLength=length(ch_triangle_data.hp-rayPayload.hitPoint);
	//rayPayload.hitPoint =ch_triangle_data.hp;
	const float3 lastHP=make_float3(rayPayload.hitPointAtt.x,rayPayload.hitPointAtt.y,rayPayload.hitPointAtt.z);
	//we could use t of ray, but if we shift the ray over the normal to avoid self-intersection we introduce an error in the electric field
	const float rayLength=length(ch_triangle_data.hp-lastHP);
	rayPayload.hitPointAtt.x =ch_triangle_data.hp.x;
	rayPayload.hitPointAtt.y =ch_triangle_data.hp.y;
	rayPayload.hitPointAtt.z =ch_triangle_data.hp.z;
	//rtPrintf("THP\t%u\t%u\th=(%.6e,%.6e,%.6e)bary=(%.6e,%.6e,%.6e)\n",launchIndexTriangle.x,launchIndexTriangle.y,hp.x,hp.y,hp.z,ch_triangle_data.hp.x,ch_triangle_data.hp.y,ch_triangle_data.hp.z);
	const float3 gn=make_float3(ch_triangle_data.geom_normal_t.x,ch_triangle_data.geom_normal_t.y,ch_triangle_data.geom_normal_t.z);	
	const float3 n = normalize(rtTransformNormal(RT_OBJECT_TO_WORLD,gn )); //Plane normal
	
#ifdef OPAL_AVOID_SI
	rayPayload.lastNormal=n;
#endif	
	const float3 reflection_dir=reflect(ray.direction, n);
	const float aux=rayPayload.ndtd.w;
	rayPayload.ndtd = make_float4(reflection_dir); //initialized with float3, w is set to 0. and updated below
	//hash_combine_impl<uint>(rayPayload.refhash,ch_triangle_data.faceId);
	//rtPrintf("HASH \t%u\t%u\t%u\t%u\n",ch_triangle_date.faceId,rayPayload.reflections,rayPayload.hits,rayPayload.refhash);
	
	
	//Use reflections and hits to create hash
	
	uint reflections=rayPayload.rhfr.x;
	uint hits=rayPayload.rhfr.y;
	uint hash=rayPayload.rhfr.w;
	hash_combine_impl<uint>(hash,ch_triangle_data.faceId+reflections+hits);
	//hash_combine_impl<uint>(rayPayload.refhash,ch_triangle_data.faceId+rayPayload.reflections+rayPayload.hits);
	rayPayload.ndtd.w = aux+ rayLength;
	rayPayload.lrhpd =make_float4(ch_triangle_data.hp); //lastReflectionHitPoint;
	rayPayload.lrhpd.w = rayPayload.ndtd.w; //totalDistanceTillLastReflection;
	
	
	
	//Compute reflection coefficient

	//Incidence angle (ai) is defined with respect to the surface, we use the complementary, which is 90-ai, and is the angle between ray and normal
	//WARNING: Assuming ray direction is normalized: dot(r,n)=cos(angle(r,n))
	//We use the fabs() because if a ray hits an internal face, the normal is reversed. The cos would be negative. For "closed" meshes this should not happen. However, in the borders, due to precision
	//it happens: a ray is not detected as hitting a face and gets inside the mesh, hitting an internal face later.
	//With penetration we can hit internal faces in closed meshes. This way, we also get the correct incidence angle again.
	
	//Compute local incidence coordinate system for reflection (components parallel and normal to the incidence plane)
	const float3 anorm_i=normalize(cross(ray.direction,n));
	const float3 apar_i=normalize(cross(anorm_i,ray.direction)); 
	//rtPrintf("IRG\t%u\t%u\tn=(%.6e,%.6e,%.6e)|anorm_i|=(%.6e,%.6e,%.6e)=%.6e\t|apar_i|=(%.6e,%.6e,%.6e)=%.6e \n",launchIndexTriangle.x,launchIndexTriangle.y,n.x,n.y,n.z,anorm_i.x,anorm_i.y,anorm_i.z,length(anorm_i),apar_i.x,apar_i.y,apar_i.z,length(apar_i));


	
	//Reflected ray basis
	const float3 anorm_r=anorm_i; 
	const float3 apar_r=cross(anorm_r,reflection_dir); //Should change the direction with respect to the incidence parallel
	//rtPrintf("RRG\t%u\t%u\tn=(%.6e,%.6e,%.6e)|anorm_r|=(%.6e,%.6e,%.6e)=%.6e\t|apar_r|=(%.6e,%.6e,%.6e)=%.6e \n",launchIndexTriangle.x,launchIndexTriangle.y,n.x,n.y,n.z,anorm_r.x,anorm_r.y,anorm_r.z,length(anorm_r),apar_r.x,apar_r.y,apar_r.z,length(apar_r));
	
	float cosA = fabs(dot(-ray.direction, n));


	//rtPrintf("RTG\t%u\t%u\t%d\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\n", launchIndexTriangle.x, launchIndexTriangle.y, rayPayload.reflections, cosA, ray.direction.x, ray.direction.y, ray.direction.z, n.x, n.y, n.z, rayPayload.lrhpd.w);
	
	//Compute the reflection coefficients (only depend on incidence angle and dielectric properties)
	//Complex arithmetic: sum
	float2 argument = make_float2(EMProperties.dielectricConstant.x + (cosA*cosA) - 1.0f, EMProperties.dielectricConstant.y);
	float2 root = complex_sqrt(argument);
	
	
	//Soft reflection. 
	//Normal reflection coefficient (Electric field not in plane of incidence)
	const float2 Rnorm = complex_div(make_float2(cosA-root.x,-root.y),make_float2(cosA+root.x,root.y));


	//Reflection info log (to be used in external programs)
	//rtPrintf("H\t%u\t%u\t%d\t%d\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\n", launchIndexTriangle.x, launchIndexTriangle.y, rayPayload.reflections, rayPayload.faceId, argument.x, argument.y, root.x, root.y, R.x, R.y, tR.x, tR.y);


	//Hard reflection.  
	//Parallel reflection coefficient (Electric field in plane of incidence)
	float2 num = sca_complex_prod(cosA, EMProperties.dielectricConstant);
	float2 div=num;
	num -=root;
	div +=root;

//Rappaport version, depends on how you define the reflected ray geometrically
//	float2 num = sca_complex_prod(cosA, make_float2(-EMProperties.dielectricConstant.x, -EMProperties.dielectricConstant.y));
//	float2 div = sca_complex_prod(cosA, EMProperties.dielectricConstant);
//	num.x += root.x;
//	num.y += root.y;
//	div.x += root.x;
//	div.y += root.y;
//	////////////////////////////////////////
	
	
	const float2	Rpar = complex_div(num, div);
	


		//Reflection info log (to be used in external programs)
	//	float mycos = dot(-ray.direction, n);
	//	rtPrintf("S\t%u\t%u\t%d\t%d\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\n", launchIndexTriangle.x, launchIndexTriangle.y, rayPayload.reflections, rayPayload.faceId, argument.x, argument.y, root.x, root.y, R.x, R.y, tR.x, tR.y);
	//	rtPrintf("NN dot=%f angle=%f hp=(%f,%f,%f)per=(%f,%f)rayR=(%f,%f)\n",mycos,  acosf(mycos), rayPayload.hitPoint.x, rayPayload.hitPoint.y, rayPayload.hitPoint.z, EMProperties.dielectricConstant.x, EMProperties.dielectricConstant.y, rayPayload.prodReflectionCoefficient.x, rayPayload.prodReflectionCoefficient.y);

	

	//Get geometric  components, multiply by previous coefficients  and multiply by reflection coefficients computed above or transmission coefficients below
		
	//Geometric part normal
	const float2 Einorm=sca_complex_prod(dot(rayPayload.hor_v,anorm_i),rayPayload.hor_coeff) + sca_complex_prod(dot(rayPayload.ver_v,anorm_i),rayPayload.ver_coeff);
	//Geometric part parallel
	const float2 Eipar=sca_complex_prod(dot(rayPayload.hor_v,apar_i),rayPayload.hor_coeff) + sca_complex_prod(dot(rayPayload.ver_v,apar_i),rayPayload.ver_coeff);

//	rtPrintf("T\t%u\t%u\thh=%.6evh=%.6ehv=%.6evv=%.6e\n",launchIndexTriangle.x,launchIndexTriangle.y,dot(rayPayload.hor_v,anorm_i),dot(rayPayload.ver_v,anorm_i),dot(rayPayload.hor_v,apar_i),dot(rayPayload.ver_v,apar_i));
//	rtPrintf("T\t%u\t%u\tRnorm(%.6e,%.6e)*hh + Rpar(%.6e,%.6e)*vh=Einorm=(%.6e,%.6e)\n",launchIndexTriangle.x,launchIndexTriangle.y,rayPayload.hor_coeff.x,rayPayload.hor_coeff.y,rayPayload.ver_coeff.x,rayPayload.ver_coeff.y,Einorm.x,Einorm.y);
//	rtPrintf("T\t%u\t%u\tRnorm(%.6e,%.6e)*hv + Rpar(%.6e,%.6e)*vv=Eipar=(%.6e,%.6e)\n",launchIndexTriangle.x,launchIndexTriangle.y,rayPayload.hor_coeff.x,rayPayload.hor_coeff.y,rayPayload.ver_coeff.x,rayPayload.ver_coeff.y,Eipar.x,Eipar.y);
//
	if ((usePenetration==1u) && (reflections<max_interactions)) {
		//Trace a penetration ray as a new ray. Recursive tracing, check stack depth>max_interactions
		//Quickly check for attenuation in dB, if att has a very low value we do not trace. Also, we do not want to overflow the float in the operations and get a nan.
		//Apply material attenuation. Again, we assume the material is not bending the ray in any direction
		
		
		//Typical values are -15 dB for 203 mm at 5 GHz => -75 dB/m
		//Considering real distance travelled through material
		float dbAtt=(EMProperties.tattenuation.x/cosA)*(EMProperties.tattenuation.y); //Attenuation in dB (power) distance*att/m = thickness/cosA*att/m
		
		//Considering that material has been travelled in perpendicular
		//float dbAtt=(EMProperties.tattenuation.x)*(EMProperties.tattenuation.y);
		//float tAtt=rayPayload.accumulatedAttenuation + dbAtt; //Accumulate in log scale to avoid overflows
		float tAtt=rayPayload.hitPointAtt.w + dbAtt; //Accumulate in log scale to avoid overflows
		if (tAtt>attenuationLimit) {
			//Copy payload
			LPWavePayload penPayload=rayPayload;
			penPayload.ndtd  = optix::make_float4(0, 0, 0, rayPayload.ndtd.w);
			//penPayload.hits=rayPayload.hits+1;
			//penPayload.flags = FLAG_NONE;
			rayPayload.rhfr=make_uint4(reflections,hits+1u,FLAG_NONE,hash);
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
			//New horizontal (normal)  coefficient
			penPayload.hor_coeff=complex_prod(Einorm,make_float2(1.0f+Rnorm.x,Rnorm.y));
			//New vertical (parallel)  coefficient
			penPayload.ver_coeff=complex_prod(Eipar,make_float2(1.0f+Rpar.x,Rpar.y));

			//Update vectors. Assuming they are equal to incident vectors
			penPayload.ver_v=apar_i;
			penPayload.hor_v=anorm_i;


			//penPayload.prodReflectionCoefficient = complex_prod(rayPayload.prodReflectionCoefficient,make_float2(1.0f+R.x, R.y)); 
			//Attenuation
			penPayload.hitPointAtt.w = tAtt;
			//penPayload.accumulatedAttenuation = tAtt;
			//rtPrintf("AT cosA=%f\tatt=%f\ttAtt=%f\nr=(%f,%f,%f)\tn=(%f,%f,%f)\n",cosA,dbAtt,tAtt,ray.direction.x,ray.direction.y,ray.direction.z,n.x,n.y,n.z);
			traceReflection<LPWavePayload>(penPayload, OPAL_RAY_REFLECTION, ch_triangle_data.hp, ray.direction,launchIndexTriangle.x,launchIndexTriangle.y);
		}
	}
	//Update here reflection coefficient, otherwise we multiply reflection and transmission in the transmission above

	//New horizontal (normal)  coefficient
	//rtPrintf("TT\t%u\t%u\tRnorm(%.6e,%.6e)  Rpar(%.6e,%.6e) hash=%u\n",launchIndexTriangle.x,launchIndexTriangle.y,rayPayload.hor_coeff.x,rayPayload.hor_coeff.y,rayPayload.ver_coeff.x,rayPayload.ver_coeff.y, rayPayload.rhfr.w);
	rayPayload.hor_coeff=complex_prod(Einorm,Rnorm);
	//New vertical (parallel)  coefficient
	rayPayload.ver_coeff=complex_prod(Eipar,Rpar);
	
	//rtPrintf("T\t%u\t%u\tRnorm(%.6e,%.6e)  Rpar(%.6e,%.6e) hash=%u\n",launchIndexTriangle.x,launchIndexTriangle.y,rayPayload.hor_coeff.x,rayPayload.hor_coeff.y,rayPayload.ver_coeff.x,rayPayload.ver_coeff.y, rayPayload.rhfr.w);
	
	//	rtPrintf("T\t%u\t%u\tRnorm(%.6e,%.6e)  Rpar(%.6e,%.6e)\n hash=%u",launchIndexTriangle.x,launchIndexTriangle.y,rayPayload.hor_coeff.x,rayPayload.hor_coeff.y,rayPayload.ver_coeff.x,rayPayload.ver_coeff.y, rayPayload.refhash);
	//	
//	rtPrintf("T\t%u\t%u\t|anorm_r|=(%.6e,%.6e,%.6e)=%.6e\t|apar_r|=(%.6e,%.6e,%.6e)=%.6e \n",launchIndexTriangle.x,launchIndexTriangle.y,anorm_r.x,anorm_r.y,anorm_r.z,length(anorm_r),apar_r.x,apar_r.y,apar_r.z,length(apar_r));
	//}
	
	//Update vectors
	rayPayload.ver_v=apar_r;
	rayPayload.hor_v=anorm_r;
	//++rayPayload.reflections;
	++reflections;
	rayPayload.rhfr=make_uint4(reflections,hits,rayPayload.rhfr.z,hash);

}



