/***************************************************************/
//
//Copyright (c) 2019 Esteban Egea-Lopez http://girtel.upct.es/~eegea
//
/**************************************************************/

#ifndef COMPLEX_H
#define COMPLEX_H

#include <optix.h>
#include <optixu/optixu_math_namespace.h>

using namespace optix;


//Complex sqrt. Adapted from thrust: https://github.com/thrust/thrust/blob/master/thrust/detail/complex/csqrtf.h 

__forceinline__ __device__ float2 complex_sqrt(float2 z) {
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
__forceinline__ __device__ float2 complex_div(float2 lhs, float2 rhs ) {
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

__forceinline__ __device__ float2 complex_prod(float2 lhs, float2 rhs) {
	return make_float2(lhs.x*rhs.x - lhs.y*rhs.y,
			lhs.x*rhs.y + lhs.y*rhs.x);
}

//Complex scalar product. Adapted from thrust :https://github.com/thrust/thrust/blob/master/thrust/detail/complex/arithmetic.h

__forceinline__ __device__ float2 sca_complex_prod(float f, float2 z) {
	return make_float2(f*z.x,f*z.y);
}

//Complex exponential JUST FOR  IMAGINARY EXPONENTS . Adapted from thrust :https://github.com/thrust/thrust/blob/master/thrust/detail/complex/arithmetic.h

__forceinline__ __device__ float2 complex_exp_only_imaginary(float2 z) {
	return make_float2(cosf(z.y), sinf(z.y));
}

#endif

