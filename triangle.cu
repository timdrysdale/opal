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

#include "Common.h"
#include <optix.h>
#include <optixu/optixu_math_namespace.h>
#include <optixu/optixu_aabb_namespace.h>
//#include <cmath>
using namespace optix;


//Complex sqrt. Adapted from thrust: https://github.com/thrust/thrust/blob/master/thrust/detail/complex/csqrtf.h 

RT_CALLABLE_PROGRAM float2 complex_sqrt(float2 z) {
	float a = z.x, b = z.y;
	float t;
	int scale;
	float2 result;

	/* We risk spurious overflow for components >= FLT_MAX / (1 + sqrt(2)). */
	const float THRESH = 1.40949553037932e+38f;

	/* Handle special cases. */
	if (b == 0.0f && a==0.0f)
		return (make_float2(0.0f, 0.0f));
	if (isinf(b))
		return (make_float2(__int_as_float(0x7f800000), b));//This should be infinity
	if (isnan(a)) {
		t = (b - b) / (b - b);	/* raise invalid if b is not a NaN */
		return (make_float2(a, t));	/* return NaN + NaN i */
	}
	if (isinf(a)) {
		/*
		* csqrtf(inf + NaN i)  = inf +  NaN i
		* csqrtf(inf + y i)    = inf +  0 i
		* csqrtf(-inf + NaN i) = NaN +- inf i
		* csqrtf(-inf + y i)   = 0   +  inf i
		*/
		if (signbit(a))
			return (make_float2(fabsf(b - b), copysignf(a, b)));
		else
			return (make_float2(a, copysignf(b - b, b)));
	}
	/*
	* The remaining special case (b is NaN) is handled just fine by
	* the normal code path below.
	*/

	/*
	* Unlike in the FreeBSD code we'll avoid using double precision as
	* not all hardware supports it.
	*/

	// FLT_MIN*2
	const float low_thresh = 2.35098870164458e-38f;
	scale = 0;

	if (fabsf(a) >= THRESH || fabsf(b) >= THRESH) {
		/* Scale to avoid overflow. */
		a *= 0.25f;
		b *= 0.25f;
		scale = 1;
	}
	else if (fabsf(a) <= low_thresh && fabsf(b) <= low_thresh) {
		/* Scale to avoid underflow. */
		a *= 4.f;
		b *= 4.f;
		scale = 2;
	}

	/* Algorithm 312, CACM vol 10, Oct 1967. */
	if (a >= 0.0f) {
		t = sqrtf((a + hypotf(a, b)) * 0.5f);
		result = make_float2(t, b / (2.0f * t));
	}
	else {
		t = sqrtf((-a + hypotf(a, b)) * 0.5f);
		result = make_float2(fabsf(b) / (2.0f * t), copysignf(t, b));
	}

	/* Rescale. */
	if (scale == 1)
		return make_float2(result.x* 2.0f, result.y*2.0f);
	else if (scale == 2)
		return make_float2(result.x * 0.5f, result.y*0.5f);
	else
		return result;
}
//Complex division. Adapted from thrust :https://github.com/thrust/thrust/blob/master/thrust/detail/complex/arithmetic.h
RT_CALLABLE_PROGRAM float2 complex_div(float2 lhs, float2 rhs ) {
	float s = abs(rhs.x) + abs(rhs.y);
	float oos = 1.0f / s;
	float ars = lhs.x * oos;
	float ais = lhs.y * oos;
	float brs = rhs.x * oos;
	float bis = rhs.y * oos;
	s = (brs * brs) + (bis * bis);
	oos = 1.0f / s;
	float2 quot=make_float2(((ars * brs) + (ais * bis)) * oos,
		((ais * brs) - (ars * bis)) * oos);
	return quot;
}
//Complex product. Adapted from thrust :https://github.com/thrust/thrust/blob/master/thrust/detail/complex/arithmetic.h

RT_CALLABLE_PROGRAM float2 complex_prod(float2 lhs, float2 rhs) {
	return make_float2(lhs.x*rhs.x - lhs.y*rhs.y,
		lhs.x*rhs.y + lhs.y*rhs.x);
}

//Complex scalar product. Adapted from thrust :https://github.com/thrust/thrust/blob/master/thrust/detail/complex/arithmetic.h

RT_CALLABLE_PROGRAM float2 sca_complex_prod(float f, float2 z) {
	return make_float2(f*z.x,f*z.y);
}

//Complex exponential JUST FOR  IMAGINARY EXPONENTS . Adapted from thrust :https://github.com/thrust/thrust/blob/master/thrust/detail/complex/arithmetic.h

RT_CALLABLE_PROGRAM float2 complex_exp_only_imaginary(float2 z) {
	return make_float2(cos(z.y), sin(z.y));
}



rtDeclareVariable(EMWavePayload, rayPayload, rtPayload, );
rtDeclareVariable(optix::Ray, ray, rtCurrentRay, );
rtDeclareVariable(TriangleHit, ch_triangle_data, attribute triangle_hit_data, );
rtDeclareVariable(MaterialEMProperties, EMProperties, , );
rtDeclareVariable(uint3, launchIndexTriangle, rtLaunchIndex, );

rtDeclareVariable(uint, meshId, , );

RT_PROGRAM void closestHitTriangle()
{

	//DECIDE IF WE KILL THE RAY


	rayPayload.hitPoint = ray.origin + ch_triangle_data.t * ray.direction;
	rayPayload.totalDistance += ch_triangle_data.t;
	rayPayload.totalDistanceTillLastReflection = rayPayload.totalDistance;
	rayPayload.t = ch_triangle_data.t;
	
	float3 n = normalize(rtTransformNormal(RT_OBJECT_TO_WORLD, ch_triangle_data.geom_normal));
	
	rayPayload.nextDirection = reflect(ray.direction, n);
	rayPayload.faceId = ch_triangle_data.faceId;

	//Compute reflection coefficient

	//Incidence angle (ai) is defined with respect to the surface, we use the complementary, which is 90-ai, and is the angle between ray and normal
	//WARNING: Assuming ray direction is normalized: dot(r,n)=cos(angle(r,n))
	//We use the fabs() because if a ray hits an internal face, the normal is reversed. The cos would be negative. For "closed" meshes this should not happen. However, in the borders, due to precision
	//it happens: a ray is not detected as hitting a face and gets inside the mesh, hitting an internal face later.
	float cosA = fabs(dot(-ray.direction, n));
	
	//if (cosA < 0) {
	//	rtPrintf("COS\t%u\t%u\t%d\t%f\t%d\t%f\t%f\t%f\n", launchIndexTriangle.x, launchIndexTriangle.y, rayPayload.reflections, cosA, rayPayload.faceId, n.x, n.y, n.z);
	//}
	

	rtPrintf("G\t%u\t%u\t%d\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\n", launchIndexTriangle.x, launchIndexTriangle.y, rayPayload.reflections, cosA, ray.direction.x, ray.direction.y, ray.direction.z, n.x, n.y, n.z, rayPayload.totalDistanceTillLastReflection);
	//rtPrintf("Gg\t%u\t%u\t%d\t%f\t%f\t%f\t%f\t%f\t%f\n", launchIndexTriangle.x, launchIndexTriangle.y, rayPayload.reflections, rayPayload.nextDirection.x, rayPayload.nextDirection.y, rayPayload.nextDirection.z, n.x, n.y, n.z);

	//Complex arithmetic: sum
	float2 argument = make_float2(EMProperties.dielectricConstant.x + (cosA*cosA) - 1.0f, EMProperties.dielectricConstant.y);
	float2 root = complex_sqrt(argument);
	float2 R;
	float polarization = dot(rayPayload.polarization, n); //Assuming polarization is normalized, we get the cos(angle(rayPayload.polarization, n))
	if (fabs(polarization) <= 0.7071f) {
		//Angle between tx polarization and normal in 45 and 90 degrees
		//Approximation (no depolarization)
		//Soft reflection. Electric field is parallel to the wall, ie, perpendicular to the triangle (wall) normal

		R = complex_div(make_float2(cosA-root.x,-root.y),make_float2(cosA+root.x,root.y));
	
			
		//Reflection info log (to be used in external programs)
	
		/*if (launchIndexTriangle.x == 987 && launchIndexTriangle.y == 3218) {
			float2 tR = complex_prod(rayPayload.prodReflectionCoefficient, R);
		//if (rayPayload.faceId >= 25) {
			rtPrintf("S\t%u\t%u\t%d\t%d\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\n", launchIndexTriangle.x, launchIndexTriangle.y, rayPayload.reflections, rayPayload.faceId, argument.x, argument.y, root.x, root.y, R.x, R.y, tR.x, tR.y);
		}
		*/
		
	}
	else {
		//Angle between tx and normal in 0 and 45 degrees
		//Approximation (no depolarization)
		//Hard reflection. Electric field is perpendicular to the wall, ie, parallel to the normal

		float2 num = sca_complex_prod(cosA, make_float2(-EMProperties.dielectricConstant.x, -EMProperties.dielectricConstant.y));
		
		float2 div = sca_complex_prod(cosA, EMProperties.dielectricConstant);
		
		/*if (launchIndexTriangle.x == 1118 && launchIndexTriangle.y == 900) {
			rtPrintf("N num=(%f,%f), div=(%f,%f) \n", num.x, num.y,  div.x, div.y);

		}
		*/
		num.x += root.x;
		num.y += root.y;
		div.x += root.x;
		div.y += root.y;
		/*if (launchIndexTriangle.x == 1118 && launchIndexTriangle.y == 900) {
			rtPrintf("N  num+=(%f,%f) div+=(%f,%f)\n", num.x, num.y, div.x, div.y);

		}*/
		R = complex_div(num, div);



		//Reflection info log (to be used in external programs)
		
		
			
		/*if (launchIndexTriangle.x == 987 && launchIndexTriangle.y == 3218) {
			float2 tR = complex_prod(rayPayload.prodReflectionCoefficient, R);
			//if (rayPayload.faceId>=25) {
			float mycos = dot(-ray.direction, n);
			rtPrintf("N\t%u\t%u\t%d\t%d\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\n", launchIndexTriangle.x, launchIndexTriangle.y, rayPayload.reflections, rayPayload.faceId, argument.x, argument.y, root.x, root.y, R.x, R.y, tR.x, tR.y);
			rtPrintf("NN dot=%f angle=%f hp=(%f,%f,%f)per=(%f,%f)rayR=(%f,%f)\n",mycos,  acosf(mycos), rayPayload.hitPoint.x, rayPayload.hitPoint.y, rayPayload.hitPoint.z, EMProperties.dielectricConstant.x, EMProperties.dielectricConstant.y, rayPayload.prodReflectionCoefficient.x, rayPayload.prodReflectionCoefficient.y);
		}
		*/
		
		
	}
	
	
	rayPayload.prodReflectionCoefficient = complex_prod(rayPayload.prodReflectionCoefficient,R);
	
	
	++rayPayload.reflections;
	
}




//Mesh buffers
rtBuffer<float3> vertex_buffer;
rtBuffer<int3>   index_buffer;
rtBuffer<uint> faceId_buffer;

rtDeclareVariable(TriangleHit, int_triangle_data, attribute triangle_hit_data, );

RT_PROGRAM void intersectTriangle(int primIdx)
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
			TriangleHit h;
			h.t = t;
			h.triId = primIdx;
			h.u = beta;
			h.v = gamma;
			
			h.geom_normal = normal;
			h.faceId = faceId_buffer[primIdx];


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
