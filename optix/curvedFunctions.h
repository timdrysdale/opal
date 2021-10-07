/***************************************************************/
//
//Copyright (c) 2019 Esteban Egea-Lopez http://girtel.upct.es/~eegea
//
/**************************************************************/
/**********************************************************
 *
 * Functions for Curved surfaces polarization operations. . 
 * 
 * ********************************************************/
#ifndef CURVEDFUNCTIONS_H
#define CURVEDFUNCTIONS_H

#include "../Common.h"
#include "Complex.h"
#include "reflectionFunctions.h"
#include <optix_world.h>
using namespace optix;

template<class P>
__forceinline__ __device__ void updateCurvature(P& rayPayload, ReflectedRayBasis& rrb, float s_prime, uint reflections, float3& u1, float3& u2, float R1, float R2, bool flat) {
	float3 x1=rayPayload.p1;
	float3 x2=rayPayload.p2;	
	if (reflections==0) {
		//For the first hit, we should incorporate the first segment attenuation to the divergence, just 1/unfolded_path
		rayPayload.divergence=rayPayload.divergence*(1.0f/(s_prime)); //The total distance has already been updated.
		x1=rrb.apar_i;
		x2=rrb.anorm_i;
	} else {	

		//We have to incorporate the previous segment
		float rho1=rayPayload.radii.x;
		float rho2=rayPayload.radii.y;
		float2 divergence=computeDivergence(rho1,rho2,s_prime,rayPayload.divergence);
		//float2 divergence=computeDivergence(rho1,rho2,s_prime,rayPayload.divergence,launchIndexTriangle.x,launchIndexTriangle.y,reflections);
		if (isnan(divergence.x) || isnan(divergence.y)) {
			//rtPrintf("CDR \t%u\t%u\tref=%d ray=(%f,%f,%f)\n",launchIndexTriangle.x,launchIndexTriangle.y,reflections, ray.direction.x,ray.direction.y,ray.direction.z); 
			//Kill ray at the moment
			rayPayload.rhfr.z |= 1u<<FLAG_END_POSITION;
			//rayPayload.rhfr.z= FLAG_END;
			return;
		}	
		rayPayload.divergence=divergence;
	}

	float irho1=0.0f;
	float irho2=0.0f;
	bool valid=false;
	if (flat) {
		valid= computePrincipalDirectionsOnFlat<P>(rayPayload,u1,u2,s_prime,rrb.cosA, rrb.reflection_dir,rrb.normal,x1,x2,irho1,irho2);
		//valid= computePrincipalDirectionsOnFlat<RDNLPWavePayload>(rayPayload,u1,u2,s_prime,rrb.cosA, rrb.reflection_dir,rrb.normal,x1,x2,irho1,irho2,launchIndexTriangle,reflections);

	} else {
		valid= computePrincipalDirections<P>(rayPayload,u1,u2,R1,R2,s_prime,rrb.cosA, rrb.reflection_dir,rrb.normal,x1,x2,irho1,irho2);
		//valid= computePrincipalDirections<P>(rayPayload,u1,u2,R1,R2,s_prime,rrb.cosA, rrb.reflection_dir,rrb.normal,x1,x2,irho1,irho2,launchIndexTriangle,reflections);
	}
	if (!valid) {
		//At the moment I just ignore and kill this ray from now on
		//rtPrintf("Not valid ray=(%f,%f,%f) \n", ray.direction.x,ray.direction.y,ray.direction.z );
		//Flag to end ray
		rayPayload.rhfr.z |= 1u<<FLAG_END_POSITION;
		atomicAdd(&invalidRaysBuffer[0],1);
		return;
	}
	//rtPrintf("NORM n=(%f,%f,%f) |n|=%f u1=(%f,%f,%f) |u1|=%f u2=(%f,%f,%f) |u2|=%f \n",rrb.normal.x,rrb.normal.y,rrb.normal.z,length(rrb.normal),u1.x,u1.y,u1.z,length(u1),u2.x,u2.y,u2.z,length(u2));
	//rtPrintf("CHAF ray=(%f,%f,%f) |ray|=%f x1=(%f,%f,%f) |x1|=%f x2=(%f,%f,%f) |x2|=%f \n",ray.direction.x,ray.direction.y,ray.direction.z,length(ray.direction),x1.x,x1.y,x1.z,length(x1),x2.x,x2.y,x2.z,length(x2));

	rayPayload.radii=make_float2(1.0f/irho1,1.0f/irho2);

//	if (isnan(x1.x) || isnan(x1.y)  || isnan(x1.z)  || isnan(x2.x) ||isnan(x2.y)  || isnan(x2.z) ) { 
//		rtPrintf("RP2 x=%uy=%u r=%u x1=(%f,%f,%f) |x1|=%f x2=(%f,%f,%f) |x2|=%f \n", launchIndexTriangle.x,launchIndexTriangle.y,rayPayload.rhfr.x, x1.x,x1.y,x1.z, length(x1),x2.x,x2.y,x2.z,length(x2));
//	}
	rayPayload.p1=x1;
	rayPayload.p2=x2;	



}

__forceinline__ __device__ optix::float2 computeDivergence(float rho1,  float rho2,  float rayLength, optix::float2 prevDivergence /*, uint xi, uint yi, uint ref*/) {

	float sq1=rho1/(rho1+rayLength);
	float sq2=rho2/(rho2+rayLength);
	//What if caustics are infinity
	//If we apply the L'Hôpital rule, the fraction goes to 1...
	if (isinf(rho1)) {
		sq1=1;
	}
	if (isinf(rho2)) {
		sq2=1;
	}

	float2 sq1c=complex_sqrt(make_float2(sq1,0.0));	
	float2 sq2c=complex_sqrt(make_float2(sq2,0.0));
	float2 ad=complex_prod(sq1c,sq2c);


	//Test and assert that both ways are equal

	//float num = rho1*rho2;
	//float den=(rho1+rayLength)*(rho2+rayLength);
	//float div1=num/den;
	//float2 sss=complex_sqrt(make_float2(div1,0.0));
	//rtPrintf("DIV rho1=%.6e rho2=%.6e sq1=%.6e sq2=%.6e s=%.6e num=%6.e den=%.6e div1=%.6e sss=(%.6e,%.6e) ad=(%.6e,%.6e) \n",rho1,rho2,sq1,sq2,rayLength,num,den,div1,sss.x,sss.y,ad.x,ad.y);
	////rtPrintf("K\t%.6e\t%.6e\t%.6e\t%.6e\n",ad.x,ad.y,sss.x,sss.y);
	//float2 pdad=complex_prod(prevDivergence,ad);
	//rtPrintf("DIV prevDiv==(%.6e,%.6e) pdad=(%.6e,%.6e)\n",prevDivergence.x,prevDivergence.y, pdad.x,pdad.y);

	if (isnan(ad.x ) || isnan(ad.y)	){
		rtPrintf("CD  rho1=%f rho2=%f rayLength=%f sq1=%f sq2=%f ad=(%.6e,%.6e)  prev=(%.6e,%.6e) sq1c=(%f,%f)sq2c=(%f,%f)\n",rho1, rho2,rayLength, sq1, sq2, ad.x,ad.y,  prevDivergence.x,prevDivergence.y, sq1c.x,sq1c.y,sq2c.x,sq2c.y);

	}
	return complex_prod(prevDivergence,ad);
}

template<class T>
__forceinline__ __device__ bool computePrincipalDirections(T& rayPayload, optix::float3 u1, optix::float3 u2, float R1, float R2,  float rayLength, float cosA, optix::float3 reflection_dir, optix::float3 n,  optix::float3& x1, optix::float3& x2, float& irho1, float& irho2 /*Debug (to be removed), uint3 launchIndexTriangle, uint reflections*/  ) {

	if (isinf(R1) && isinf(R2)) {
		//This part of the curved surface is actually flat, e.g., part of the anvers tunnel
		return computePrincipalDirectionsOnFlat<T>(rayPayload,u1,u2,rayLength,cosA,reflection_dir,n,x1,x2,irho1,irho2);
	}

	// General formulation
	//Principal planes: x1 and x2 from the incident ray: for a spherical wave any pair of perpendicular vectors as well as perpendicular to the ray

	//rtPrintf("CC r=%u  x1=(%f,%f,%f) |x1|=%f x2=(%f,%f,%f) |x2|=%f n=(%f,%f,%f)\n", reflections, x1.x,x1.y,x1.z, length(x1),x2.x,x2.y,x2.z,length(x2),n.x,n.y,n.z);
	//rtPrintf("CC r=%u  u1=(%f,%f,%f) |u1|=%f u2=(%f,%f,%f) |u2|=%f n=(%f,%f,%f)\n", reflections, u1.x,u1.y,u1.z, length(u1),u2.x,u2.y,u2.z,length(u2),n.x,n.y,n.z);
	n=normalize(n);
	x1=normalize(x1);
	x2=normalize(x2);
	u1=normalize(u1);
	u2=normalize(u2);
	float t11=dot(x1,u1);
	float t12=dot(x1,u2);
	float t21=dot(x2,u1);
	float t22=dot(x2,u2);
	//Determinant
	float td=(t11*t22)-(t21*t12);
	float td2=td*td;
	//rtPrintf("CC r=%u t11=%f  t12=%f t21=%f t22=%f td=%.6e\n", reflections,t11,t12,t21,t22,td);

	float tpr1=0.0f;
	float tmr1=0.0f;
	float tr1=0.0f;
	float t12r1=0.0f;
	float tcr1=0.0f;
	if (!isinf(R1)) {
		tpr1=((t22*t22)+(t12*t12))/R1;
		tmr1=((t22*t22)-(t12*t12))/R1;
		tr1=(t22*t22)/R1;
		t12r1=(t12*t12)/R1;
		tcr1=t22*t12/R1;

		//rtPrintf("CCR1 r=%u tpr1=%f tmr1=%f tr1=%f t12r1=%f tcr1=%f\n", reflections,tpr1,tmr1,tr1,t12r1,tcr1);
	}
	float tpr2=0.0f;
	float tmr2=0.0f;
	float tr2=0.0f;
	float t21r2=0.0f;
	float tcr2=0.0f;
	if (!isinf(R2)) {
		tpr2=((t21*t21)+(t11*t11))/R2;
		tmr2=((t21*t21)-(t11*t11))/R2;
		tr2=(t11*t11)/R2;
		t21r2=(t21*t21)/R2;
		tcr2=t11*t21/R2;

		//rtPrintf("CCR2 r=%u tpr2=%f tmr2=%f tr2=%f t21r2=%f tcr2=%f\n", reflections,tpr2,tmr2,tr2,t21r2,tcr2);
	}
	float A = (cosA/td2)*(tpr1+tpr2);
	float r1r2=0.0f;	
	if (!isinf(R2) && !isinf(R1)) {
		r1r2=4.0f*td2/(R1*R2);
	}
	//TODO: Now, what is the principal radii of curvature of the reflected wave?? For a single-order reflection it is clear that it is the ray length, but for the higher order reflections must be the radii+length...

	float irhoi1=(1.0f/(rayPayload.radii.x+rayLength));
	float irhoi2=(1.0f/(rayPayload.radii.y+rayLength));

	//Avoid precision errors
	float aux2=(tpr1+tpr2);
	aux2 = aux2*aux2;
	float rhom=irhoi1-irhoi2;
	float B=0.5f*sqrt((rhom*rhom) + (rhom*4.0f*cosA*(tmr1+tmr2)/td2) + (4.0f*cosA*cosA/(td2*td2))*((aux2) -r1r2))  ;
	//float B=0.5*sqrt((rhom*rhom) + rhom*4*cosA*(tmr1+tmr2)/td2 + (4*cosA*cosA/(td2*td2))*(((tpr1+tpr2)*(tpr1+tpr2)) -r1r2))  ;
	float if1=A+B;
	float if2=A-B;
	//rtPrintf("if r=%u\t%u\t%u  A=%f B=%f tpr1=%f tpr2=%f if1=%f if2=%f rhom=%f\n", launchIndexTriangle.x, launchIndexTriangle.y,reflections,A,B,tpr1,tpr2,if1,if2,rhom);


	float C=(2.0f*cosA)/td2;
	//Not really used
	//float q11=(1/(rayPayload.radii.x+rayLength)) + C*(tr1+t21r2);
	float q12=-C*(tcr1+tcr2);
	float q22=irhoi2 + (C*(t12r1+tr2));

	float3 xu1=x1- 2.0f*dot(n,x1)*n;
	xu1=normalize(xu1);
	float3 xu2=x2- 2.0f*dot(n,x2)*n;
	xu2=normalize(xu2);
	//float3 xu1=rayPayload.p1- 2.0f*dot(n,rayPayload.p1)*n;
	//float3 xu2=rayPayload.p2- 2.0f*dot(n,rayPayload.p2)*n;
	//rtPrintf("XU r=%u xu1=(%f,%f,%f) |xu1|=%f xu2=(%f,%f,%f) |xu2|=%f dot1=%f dot2=%f\n", reflections,xu1.x,xu1.y,xu1.z,length(xu1),xu2.x,xu2.y,xu2.z,length(xu2), dot(xu1,reflection_dir),dot(xu2,reflection_dir));
	//rtPrintf("CC r=%u xu1=(%f,%f,%f) |xu1|=%f xu2=(%f,%f,%f) |xu2|=%f\n", reflections,xu1.x,xu1.y,xu1.z,length(xu1),xu2.x,xu2.y,xu2.z, length(xu2));

	//New principal curvatures (inverse of radii of curvature
	irho1=(0.5f*(irhoi1+irhoi2))+if1;	
	irho2=(0.5f*(irhoi1+irhoi2))+if2;	
	//rtPrintf("caustics r=%u\t%u\t%u  irhoi1=%f irhoi2=%f if1=%f if2=%f irho1=%f irho2=%f \n", launchIndexTriangle.x, launchIndexTriangle.y,reflections,irhoi1,irhoi2, if1, if2, irho1,irho2);
	//rtPrintf("caustics r=%u\t%u\t%u  irhoi1=%.6e irhoi2=%.6e if1=%.6e if2=%.6e irho1=%.6e irho2=%.6e rayL=%f \n", launchIndexTriangle.x, launchIndexTriangle.y,reflections,irhoi1,irhoi2, if1, if2, irho1,irho2,rayLength);
	//Precision error may render it negative, so first substraction and then square
	float aux1=(q22-irho1);
	aux1=aux1*aux1;
	float D= sqrt(aux1 + (q12*q12));

	//Depending on the surface and the election of the principal planes for the wave, this can be zero:
	//It is zero if q12=0 and aux1=0
	//TODO: use L'Hopital again, that should tend to 1...
	if (D<1e-8) {
		//return false;
		//Use L'Hopital
		float3 aux1=x1;
		float3 aux2=x2;
		x1=xu1 -xu2;
		x1=normalize(x1);
		//if (isnan(x1.x) || isnan(x1.y)  || isnan(x1.z)  || isnan(x2.x) ||isnan(x2.y)  || isnan(x2.z) ) { 
		//	rtPrintf("RP x=%uy=%u r=%u x1=(%f,%f,%f) |x1|=%f x2=(%f,%f,%f) |x2|=%f \n", launchIndexTriangle.x,launchIndexTriangle.y,rayPayload.rhfr.x, x1.x,x1.y,x1.z, length(x1),x2.x,x2.y,x2.z,length(x2));
		//	uint ref=rayPayload.rhfr.x;
		//	rtPrintf("CCR2 x=%uy=%u r=%u R1=%f R2=%f tpr2=%f tmr2=%f tr2=%f t21r2=%f tcr2=%f\n",launchIndexTriangle.x, launchIndexTriangle.y, ref, R1,R2, tpr2,tmr2,tr2,t21r2,tcr2);
		//	rtPrintf("CCR1 tpr1=%f tmr1=%f tr1=%f t12r1=%f tcr1=%f\n", tpr1,tmr1,tr1,t12r1,tcr1);
		//	rtPrintf("CC x=%uy=%u r=%u t11=%f  t12=%f t21=%f t22=%f td=%.6e\n",launchIndexTriangle.x,launchIndexTriangle.y, ref, t11,t12,t21,t22,td);
		//	rtPrintf("X x=%uy=%u r=%u aux1=(%f,%f,%f) |aux1|=%f aux2=(%f,%f,%f) |aux2|=%f \n", launchIndexTriangle.x,launchIndexTriangle.y, ref,  aux1.x,aux1.y,aux1.z, length(aux1),aux2.x,aux2.y,aux2.z,length(aux2));
		//	rtPrintf("U x=%uy=%u   u1=(%f,%f,%f) |u1|=%f u2=(%f,%f,%f) |u2|=%f n=(%f,%f,%f)\n", launchIndexTriangle.x,launchIndexTriangle.y,  u1.x,u1.y,u1.z, length(u1),u2.x,u2.y,u2.z,length(u2),n.x,n.y,n.z);
		//	rtPrintf("DDD x=%uy=%u r=%u cosA=%.6e D=%.6e aux1=%.6e q22=%.6e irho1=%.6e irho2=%.6e q12=%.6e\n",launchIndexTriangle.x,launchIndexTriangle.y, ref, cosA, D,  aux1, q22,irho1,irho2,q12);
		//}
	} else {
		x1=((q22-irho1)*xu1 -q12*xu2)/D;
		x1=normalize(x1);
	}
	//New principal directions
	//Should end up normalized	

	//x1=((q22-irho1)*xu1 -q12*xu2)/D;
	//x1=normalize(x1);
	x2=cross(-reflection_dir,x1);
	x2=normalize(x2);
	//rtPrintf("X r=%u x1=(%f,%f,%f) |x1|=%f x2=(%f,%f,%f) |x2|=%f\n", reflections,x1.x,x1.y,x1.z,length(x1),x2.x,x2.y,x2.z,length(x2));
	//	if (isnan(rayPayload.radii.x) || isnan(rayPayload.radii.y) || isnan(rayPayload.divergence.x ) || isnan(rayPayload.divergence.y)) {
	//		rtPrintf("CC r=%u u1=(%f,%f,%f) u2=(%f,%f,%f) R2=%f rayLength=%f \n", reflections,u1.x,u1.y,u1.z,u2.x,u2.y,u2.z,R2, rayLength );
	//		rtPrintf("CC r=%u x1=(%f,%f,%f) |x1|=%f x2=(%f,%f,%f) |x2|=%f\n", reflections,x1.x,x1.y,x1.z, length(x1),x2.x,x2.y,x2.z,length(x2));
	//		rtPrintf("CC2 r=%u\t%u\t%u  td=%.6e ref=(%f,%f,%f) \n", launchIndexTriangle.x, launchIndexTriangle.y,reflections,td, reflection_dir.x,reflection_dir.y,reflection_dir.z);
	//		rtPrintf("CC1 r=%u\t%u\t%u  tpr2=%f tmr2=%f tr2=%f t21r2=%f tcr2=%f\n",launchIndexTriangle.x, launchIndexTriangle.y, reflections,tpr2,tmr2,tr2,t21r2,tcr2);
	//		rtPrintf("DDD r=%u\t%u\t%u  cosA=%.6e D=%.6e aux1=%.6e q22=%.6e irho1=%.6e irho2=%.6e q12=%.6e\n",launchIndexTriangle.x, launchIndexTriangle.y,reflections, cosA, D,  aux1, q22,irho1,irho2,q12);
	//		rtPrintf("CC3 r=%u\t%u\t%u  xu1=(%f,%f,%f) |xu1|=%f xu2=(%f,%f,%f) |xu2|=%f\n",launchIndexTriangle.x, launchIndexTriangle.y, reflections,xu1.x,xu1.y,xu1.z,length(xu1),xu2.x,xu2.y,xu2.z, length(xu2));
	//		rtPrintf("CC4 r=%u\t%u\t%u  irho1=%f,irho2=%f\n",launchIndexTriangle.x, launchIndexTriangle.y, reflections,irho1,irho2);
	//		rtPrintf("CC5 r=%u\t%u\t%u  x1=(%f,%f,%f) x2=(%f,%f,%f)\n",launchIndexTriangle.x, launchIndexTriangle.y, reflections,x1.x,x1.y,x1.z,x2.x,x2.y,x2.z);
	//	}	

	return true;
}
template<class T>
__forceinline__ __device__ bool computePrincipalDirectionsOnFlat(T& rayPayload, optix::float3 u1, optix::float3 u2,   float rayLength, float cosA, optix::float3 reflection_dir, optix::float3 n,  optix::float3& x1, optix::float3& x2, float& irho1, float& irho2 /*Debug (to be removed), uint3 launchIndexTriangle, uint reflections */ ) {

	//	if (isinf(R1) && isinf(R2)) 
	//Already assume that both curvatures are infinity since this is a flat surface
	n=normalize(n);
	float irhoi1=(1.0f/(rayPayload.radii.x+rayLength));
	float irhoi2=(1.0f/(rayPayload.radii.y+rayLength));
	float if12=0.5*abs(irhoi1-irhoi2);
	float irhosum=0.5f*(irhoi1+irhoi2);
	irho1=irhosum+if12;	
	irho2=irhosum-if12;	
	//float q12=0.0f;
	//float q22=irhoi2;

	float3 xu1=x1- 2.0f*dot(n,x1)*n;
	xu1=normalize(xu1);
	float3 xu2=x2- 2.0f*dot(n,x2)*n;
	xu2=normalize(xu2);
	//float3 xu1=rayPayload.p1- 2.0f*dot(n,rayPayload.p1)*n;
	//float3 xu2=rayPayload.p2- 2.0f*dot(n,rayPayload.p2)*n;
	//rtPrintf("XU r=%u xu1=(%f,%f,%f) |xu1|=%f xu2=(%f,%f,%f) |xu2|=%f dot1=%f dot2=%f\n", reflections,xu1.x,xu1.y,xu1.z,length(xu1),xu2.x,xu2.y,xu2.z,length(xu2), dot(xu1,reflection_dir),dot(xu2,reflection_dir));
	//rtPrintf("CC r=%u xu1=(%f,%f,%f) |xu1|=%f xu2=(%f,%f,%f) |xu2|=%f\n", reflections,xu1.x,xu1.y,xu1.z,length(xu1),xu2.x,xu2.y,xu2.z, length(xu2));

	//rtPrintf("caustics r=%u\t%u\t%u  irhoi1=%f irhoi2=%f if1=%f if2=%f irho1=%f irho2=%f \n", launchIndexTriangle.x, launchIndexTriangle.y,reflections,irhoi1,irhoi2, if1, if2, irho1,irho2);
	//rtPrintf("caustics r=%u\t%u\t%u  irhoi1=%f irhoi2=%f if1=%f if2=%f irho1=%f irho2=%f rayL=%f \n", launchIndexTriangle.x, launchIndexTriangle.y,reflections,irhoi1,irhoi2, if1, if2, irho1,irho2,rayLength);

	/** This should not be necessary since it cancels itself in the formula***/
	//Precision error may render it negative, so 
	//float aux1=(q22-irho1);
	////aux1=aux1*aux1;
	////float D= sqrt(aux1 + (q12*q12));
	//float D= aux1 ;

	////Depending on the surface and the election of the principal planes for the wave, this can be zero
	//if (D<1e-8) {
	//	//rtPrintf("DDD cosA=%.6e D=%.6e aux1=%.6e q22=%.6e irho1=%.6e irho2=%.6e q12=%.6e\n", cosA, D,  aux1, q22,irho1,irho2,q12);
	//	return false;
	//}

	//rtPrintf("DDD cosA=%.6e D=%.6e aux1=%.6e q22=%.6e irho1=%.6e irho2=%.6e q12=%.6e\n", cosA, D,  aux1, q22,irho1,irho2,q12);
	//New principal directions
	//Should end up normalized	

	//We are basically reflecting the principal planes
	x1=xu1;
	x2=cross(-reflection_dir,x1);
	x2=normalize(x2);
	return true;
}
template<class T>
__forceinline__ __device__ optix::float2 computeRadiiSpherical(T& rayPayload, optix::float3 u1, optix::float3 u2, float R1, float R2,  float rayLength, float cosA, optix::float3 ray_dir,  /*Debug (to be removed)*/ uint3 launchIndexTriangle, uint reflections  ) {

	if (cosA==0) {
		return make_float2(0.0f,0.0f);
	}
	float icosA=1.0f/cosA;
	float cos1=dot(u1,ray_dir); //Assuming normalized vectors
	float cos2=dot(u2,ray_dir); //Assuming normalized vectors
	float sins1=1.0-(cos1*cos1);
	float sins2=1.0-(cos2*cos2);
	float s2r1=0.0;
	float s1r2=0.0;
	if (!isinf(R1)) {
		s2r1=sins2/R1;
	}
	if (!isinf(R2)) {
		s1r2=sins1/R2;
	}
	float r1r2=0.0;
	if (!isinf(R2) && !isinf(R1)) {
		r1r2=4.0f/(R1*R2);
	}
	float A= (s2r1+s1r2)*icosA;
	float As=A*A;
	float if1=A+sqrt(As-r1r2);
	float if2=A-sqrt(As-r1r2);
	float irhoi1=(1.0f/(rayPayload.radii.x+rayLength));
	float irhoi2=(1.0f/(rayPayload.radii.y+rayLength));
	float irho1=(0.5f*(irhoi1+irhoi2))+if1;	
	float irho2=(0.5f*(irhoi1+irhoi2))+if2;
	//rtPrintf("SPH if1=%f if2=%f irhoi1=%f irhoi2=%f sins1=%f sins2=%f acosA=%f acos1=%f acos2=%f\n",if1,if2,irhoi1,irhoi2,sins1,sins2,(acosf(cosA)*57.29577),(acosf(cos1)*57.29577),(acosf(cos2)*57.29577));

	return make_float2(1.0f/irho1,1.0f/irho2);	

}
#endif
