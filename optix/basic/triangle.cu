/***************************************************************/
//
//Copyright (c) 2019 Esteban Egea-Lopez http://girtel.upct.es/~eegea
//
/**************************************************************/


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
#include "../Complex.h"
#include "../traceFunctions.h"
#include "../reflectionFunctions.h"
#include <optix.h>
#include <optixu/optixu_math_namespace.h>
#include <optixu/optixu_aabb_namespace.h>
//#include <cmath>
using namespace optix;

//Launch variables
rtDeclareVariable(uint3, launchIndexTriangle, rtLaunchIndex, );
rtDeclareVariable(HVWavePayload, rayPayload, rtPayload, );
rtDeclareVariable(optix::Ray, ray, rtCurrentRay, );
rtDeclareVariable(TriangleHit, ch_triangle_data, attribute triangle_hit_data, );

//Per-mesh local variables 
rtDeclareVariable(MaterialEMProperties, EMProperties, , );
rtDeclareVariable(uint, meshId, , );


//WARNING: Only to be used with vertical (0,1,0) and horizontal, but just (1,0,0), polarizations. Assuming that the transmitters and receivers have the same polarization. In any other 
//case, you should use full depolarization

RT_PROGRAM void closestHitTriangle()
{

	//Update payload
	const float3 gn=make_float3(ch_triangle_data.geom_normal_t.x,ch_triangle_data.geom_normal_t.y,ch_triangle_data.geom_normal_t.z);	
	const float3 n = normalize(rtTransformNormal(RT_OBJECT_TO_WORLD,gn ));
	const float3 reflection_dir=normalize(reflect(ray.direction, n));
	const float aux=rayPayload.ndtd.w;
	//const float rayLength = ch_triangle_data.geom_normal_t.w;
	//const float3 hp= ray.origin + rayLength * ray.direction ;
	//rayPayload.hitPoint =hp;
	//Get the hitpoint from the barycentric coordinates computed in the triangle hit. This should get us a point always on the surface and help avoid self-intersection
	//See https://www.realtimerendering.com/raytracinggems/ 6.1
	//rayPayload.hitPoint =ch_triangle_data.hp;
	const float3 lastHP=make_float3(rayPayload.hitPointAtt.x,rayPayload.hitPointAtt.y,rayPayload.hitPointAtt.z);
	
	//we could use t of ray, but if we shift the ray over the normal to avoid self-intersection we introduce an error in the electric field
	const float rayLength=length(ch_triangle_data.hp-lastHP);

	rayPayload.hitPointAtt.x =ch_triangle_data.hp.x;
	rayPayload.hitPointAtt.y =ch_triangle_data.hp.y;
	rayPayload.hitPointAtt.z =ch_triangle_data.hp.z;
#ifdef OPAL_AVOID_SI
	rayPayload.lastNormal=n;
#endif	
	rayPayload.ndtd = make_float4(reflection_dir); //initialized with float3, w is set to 0. and updated below
	//hash_combine_impl<uint>(rayPayload.refhash,ch_triangle_data.faceId);
	
	//Use reflections and hits to create hash
	uint reflections=rayPayload.rhfr.x;
	uint hits=rayPayload.rhfr.y;
	uint hash=rayPayload.rhfr.w;
	hash_combine_impl<uint>(hash,ch_triangle_data.faceId+reflections+hits);
	//hash_combine_impl<uint>(rayPayload.refhash,ch_triangle_data.faceId+rayPayload.reflections+rayPayload.hits);
	//rtPrintf("HASH \t%u\t%u\t%u\t%u\n",ch_triangle_data.faceId,rayPayload.reflections,rayPayload.hits,rayPayload.refhash);
	
	rayPayload.ndtd.w = aux+ rayLength;
	rayPayload.lrhpd =make_float4(ch_triangle_data.hp); //lastReflectionHitPoint;
	rayPayload.lrhpd.w = rayPayload.ndtd.w; //totalDistanceTillLastReflection;
	
	
	
	//Compute reflection coefficient

	//Incidence angle (ai) is defined with respect to the surface, we use the complementary, which is 90-ai, and is the angle between ray and normal
	//WARNING: Assuming ray direction is normalized: dot(r,n)=cos(angle(r,n))
	//We use the fabs() because if a ray hits an internal face, the normal is reversed. The cos would be negative. For "closed" meshes this should not happen. However, in the borders, due to precision
	//it happens: a ray is not detected as hitting a face and gets inside the mesh, hitting an internal face later.
	//With penetration we can hit internal faces in closed meshes. This way, we also get the correct incidence angle again.
	float cosA = fabs(dot(-ray.direction, n));
	float2 dielectricConstant = getDielectricConstant<HVWavePayload>(rayPayload,EMProperties);

	//rtPrintf("G\t%u\t%u\t%d\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\n", launchIndexTriangle.x, launchIndexTriangle.y, rayPayload.reflections, cosA, ray.direction.x, ray.direction.y, ray.direction.z, n.x, n.y, n.z, rayPayload.lrhpd.w);

	//Complex arithmetic: sum
	float2 argument = make_float2(dielectricConstant.x + (cosA*cosA) - 1.0f, dielectricConstant.y);
	float2 root = complex_sqrt(argument);
	float2 R;
	float polarization = dot(make_float3(rayPayload.polarization_k), n); //Assuming polarization is normalized, we get the cos(angle(rayPayload.polarization, n))
	if (fabs(polarization) <= 0.7071f) {
		//Angle between tx polarization and normal in 45 and 90 degrees
		//Approximation (no depolarization)
		//Soft reflection. Electric field perpendicular to the plane of incidence. Electric field is parallel to the wall, ie, perpendicular to the triangle (wall) normal

		R = complex_div(make_float2(cosA-root.x,-root.y),make_float2(cosA+root.x,root.y));


		//Reflection info log (to be used in external programs)
		//rtPrintf("S\t%u\t%u\t%u\t%f\t%f\t%f\t%f\t%f\t%.6e\t%.6e\n", launchIndexTriangle.x, launchIndexTriangle.y, reflections,(acosf(cosA)*180.0f/3.1415),  argument.x, argument.y, root.x, root.y, R.x, R.y);
		

	}
	else {
		//Angle between tx and normal in 0 and 45 degrees
		//Approximation (no depolarization)
		//Hard reflection. Electric field in the plane of incidence Electric field is perpendicular to the wall, ie, parallel to the normal

		//float2 num = sca_complex_prod(cosA, make_float2(EMProperties.dielectricConstant.x, EMProperties.dielectricConstant.y));
		float2 num = sca_complex_prod(cosA, dielectricConstant);
		float2 div=num;
		num -=root;
		div +=root;
		R = complex_div(num, div);
//Rappaport version, depends on how you define the reflected ray geometrically
//	float2 num = sca_complex_prod(cosA, make_float2(-EMProperties.dielectricConstant.x, -EMProperties.dielectricConstant.y));
//	float2 div = sca_complex_prod(cosA, EMProperties.dielectricConstant);
//	num.x += root.x;
//	num.y += root.y;
//	div.x += root.x;
//	div.y += root.y;
//	R = complex_div(num, div);
//	////////////////////////////////////////

		//Reflection info log (to be used in external programs)
	//	float mycos = dot(-ray.direction, n);
		//rtPrintf("N\t%u\t%u\t%u\t%f\t%f\t%f\t%f\t%f\t%f\t%f\n", launchIndexTriangle.x, launchIndexTriangle.y, reflections,(acosf(cosA)*180.0f/3.141592), argument.x, argument.y, root.x, root.y, R.x, R.y);
	//	rtPrintf("NN dot=%f angle=%f hp=(%f,%f,%f)per=(%f,%f)rayR=(%f,%f)\n",mycos,  acosf(mycos), rayPayload.hitPoint.x, rayPayload.hitPoint.y, rayPayload.hitPoint.z, EMProperties.dielectricConstant.x, EMProperties.dielectricConstant.y, rayPayload.prodReflectionCoefficient.x, rayPayload.prodReflectionCoefficient.y);

	}




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
			HVWavePayload penPayload=rayPayload;
			penPayload.ndtd  = optix::make_float4(0, 0, 0, rayPayload.ndtd.w);
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
			penPayload.prodReflectionCoefficient = complex_prod(rayPayload.prodReflectionCoefficient,make_float2(1.0f+R.x, R.y)); 
			//Attenuation
			penPayload.hitPointAtt.w = tAtt;
			//rtPrintf("AT cosA=%f\tatt=%f\ttAtt=%f\nr=(%f,%f,%f)\tn=(%f,%f,%f)\n",cosA,dbAtt,tAtt,ray.direction.x,ray.direction.y,ray.direction.z,n.x,n.y,n.z);
			traceReflection<HVWavePayload>(penPayload, ray.ray_type, ch_triangle_data.hp, ray.direction, launchIndexTriangle.x,launchIndexTriangle.y,false);
		}
	}
	//Update here reflection coefficient, otherwise we multiply reflection and transmission in the transmission above
	rayPayload.prodReflectionCoefficient = complex_prod(rayPayload.prodReflectionCoefficient,R);
	//++rayPayload.reflections;
	++reflections;
	rayPayload.rhfr=make_uint4(reflections,hits,rayPayload.rhfr.z,hash);

}




