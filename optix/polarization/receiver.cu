/***************************************************************/
//
//Copyright (c) 2019 Esteban Egea-Lopez http://ait.upct.es/eegea
//
/**************************************************************/


#include "../../Common.h"
#include "../../Complex.h"
#include "linearPolarizationFunctions.h"
#include <optix_world.h>
#include <optixu/optixu_math_namespace.h>
#include <optixu/optixu_aabb_namespace.h>
using namespace optix;



//Receiver global buffers
rtBuffer<HitInfo, 1> globalHitInfoBuffer; //Buffer to store all the hits
rtBuffer<uint, 1> atomicIndex; //Buffer to store the current global buffer index 
rtDeclareVariable(uint, global_info_buffer_maxsize, ,);

//Transmitter buffer
rtBuffer<Transmitter, 1> txBuffer;


//Receiver local variables
rtDeclareVariable(uint, receiverBufferIndex, , ); //Buffer id
rtDeclareVariable(int, externalId, , ); //External id  used to identify receivers 
rtDeclareVariable(SphereHit, hit_attr, attribute hit_attr, );
rtDeclareVariable(float, k, , );
rtDeclareVariable(float4, sphere, , );
rtDeclareVariable(float3, receiverPolarization, , );
//Launch variables
rtDeclareVariable(uint3, receiverLaunchIndex, rtLaunchIndex, );
rtDeclareVariable(LPWavePayload, hitPayload, rtPayload, );
rtDeclareVariable(optix::Ray, ray_receiver, rtCurrentRay, );

//Global variables
rtDeclareVariable(float, asRadiusConstant, ,);

//Penetration configuration
rtDeclareVariable(uint, usePenetration, , );





//Closest hit program for receivers
RT_PROGRAM void closestHitReceiver()
{


	//Do not end the ray, it can pass through the reception sphere and reflect on a wall, inside or outside the receiver sphere

	//Update ray data
	const float rayLength = hit_attr.geom_normal_t.w;
	const float3 hitPoint = ray_receiver.origin + rayLength*ray_receiver.direction;
	//hitPayload.hitPoint=hitPoint;
	hitPayload.hitPointAtt.x=hitPoint.x;
	hitPayload.hitPointAtt.y=hitPoint.y;
	hitPayload.hitPointAtt.z=hitPoint.z;
	const float aux= hitPayload.ndtd.w; //Previous total distance
	hitPayload.ndtd = make_float4(ray_receiver.direction); //Next direction only
	hitPayload.ndtd.w = aux+ rayLength; //totalDistance 
	const uint txBufferIndex=receiverLaunchIndex.z;
	const Transmitter current_tx=txBuffer[txBufferIndex];
	float3 prx = make_float3(sphere.x, sphere.y, sphere.z);
#ifdef OPAL_AVOID_SI

	hitPayload.lastNormal=make_float3(hit_attr.geom_normal_t.x,hit_attr.geom_normal_t.y,hit_attr.geom_normal_t.z);
	//hitPayload.lastNormal=normalize(hitPoint-prx);
#endif
	//Check if ray is hitting his own tx (transmitter are also receivers usually) A transmitter cannot receive while it is transmitting, unless other channel is used.
	if (externalId == current_tx.externalId) {
		//My own outgoing ray
		//rtPrintf("External. txId=%d i.el=%u i.az=%u, ray=(%f,%f,%f) origin=(%f,%f,%f) t=%f rId[%u]=%d\n", current_tx.externalId, receiverLaunchIndex.x, receiverLaunchIndex.y, ray_receiver.direction.x, ray_receiver.direction.y, ray_receiver.direction.z, ray_receiver.origin.x, ray_receiver.origin.y, ray_receiver.origin.z, hit_attr.geom_normal_t, receiverBufferIndex,externalId);
		return;
	}


	//int reflections = hitPayload.reflections;
	uint reflections = hitPayload.rhfr.x;

	//Check if ray originated inside the reception radius of this receiver


	//HitInfo values
	float d;
	uint hash=0u;
	uint dtrx=0u;
	//Reflected or transmitted ray


	//Distance from ray line to receiver position. To keep only the closest hit later
	//Line is defined by ray
	//float3 pd = prx - hitPayload.hitPoint;
	float3 pd = prx - hitPoint;
	float u = dot(pd, ray_receiver.direction);
	float3 p3 = hitPoint + u*ray_receiver.direction;
	//float3 p3 = hitPayload.hitPoint + u*ray_receiver.direction;


	float3 rxtoh=prx-p3;
	float drxtohitsq=dot(rxtoh,rxtoh);
	//float drxtohit=length(prx - p3);

	//Unfolded path distance		
	const float3 lastReflectionHitPoint = make_float3(hitPayload.lrhpd.x,hitPayload.lrhpd.y,hitPayload.lrhpd.z);
	d=length(prx-lastReflectionHitPoint);

	d+=hitPayload.lrhpd.w; //totalDistanceTillLastReflection

	//Take into account radius and angular separation: ignore hit if distance from hit to receiver is > unfolded path distance * angular separation

	//		float vrsq=d*d*asRadiusConstant*asRadiusConstant/3; //sqr(d*as/sqrt(3));
	//		if (drxtohitsq>vrsq) {
	//			//	rtPrintf("DS\t%u\t%u\tRE\tp3=(%.6e,%.6e,%.6e) drxtohitsq=%.6e vrsq=%.6e hash=%u\n",receiverLaunchIndex.x,receiverLaunchIndex.y,p3.x,p3.y,p3.z,drxtohitsq,vrsq,hitPayload.refhash);
	//			return;
	//		}
	float dm = drxtohitsq*100000000000.0f;  //Multiply by 10000000000 to truncate later take 11 digits
	int dmt = __float2int_rz(dm);   //Truncate
	//HitInfo values
	dtrx=static_cast<uint>(dmt);
	//hash=hitPayload.refhash;
	hash=hitPayload.rhfr.w;


	//rtPrintf("DS\t%u\t%u\tp3=(%.6e,%.6e,%.6e) drxtohitsq=%.6e dm=%.6e dmt=%d dtrx=%u hash=%u\n",receiverLaunchIndex.x,receiverLaunchIndex.y,p3.x,p3.y,p3.z,drxtohitsq,dm,dmt,dtrx,hash);



	//Compute incident electric field.  


	float2 z = make_float2(0.0f, -k*d);
	float2 zexp = complex_exp_only_imaginary(z);
	//Apply distance loss to each component

	float2 Rzexph = complex_prod(hitPayload.hor_coeff, zexp);
	float2 Rzexpv = complex_prod(hitPayload.ver_coeff, zexp);


	float2 Eih = sca_complex_prod((hitPayload.electricFieldAmplitude / d), Rzexph);
	float2 Eiv = sca_complex_prod((hitPayload.electricFieldAmplitude / d), Rzexpv);

	//**************+ Tests and debug: Project on x and y **************
	float3 xu=make_float3(1.0f,0.0f,0.0f);
	float3 yu=make_float3(0.0f,1.0f,0.0f);
	float3 zu=make_float3(0.0f,0.0f,1.0f);
	const float2 Einorm=sca_complex_prod(dot(hitPayload.hor_v,xu),Eih) + sca_complex_prod(dot(hitPayload.ver_v,xu),Eiv);
	const float2 Eipar=sca_complex_prod(dot(hitPayload.hor_v,yu),Eih) + sca_complex_prod(dot(hitPayload.ver_v,yu),Eiv);
	const float2 Eirad=sca_complex_prod(dot(hitPayload.hor_v,zu),Eih) + sca_complex_prod(dot(hitPayload.ver_v,zu),Eiv);
	HitInfo aHit;
	aHit.thrd=make_uint4(txBufferIndex,hash,receiverBufferIndex,dtrx);
	aHit.Ex=Einorm;
	aHit.Ey=Eipar;
	aHit.Ez=Eirad;
#ifdef OPAL_LOG_TRACE
	aHit.rayDir=hitPayload.initialRayDir;
#endif 	
	//rtPrintf("TP\t%u\t%u\tRnorm(%.6e,%.6e)*hah + Rpar=(%.6e,%.6e)*vha=Einorm=(%.6e,%.6e)\n",receiverLaunchIndex.x,receiverLaunchIndex.y,hitPayload.hor_coeff.x,hitPayload.hor_coeff.y,hitPayload.ver_coeff.x,hitPayload.ver_coeff.y,Einorm.x,Einorm.y);
	//rtPrintf("TP\t%u\t%u\tRnorm(%.6e,%.6e)*hva + Rpar=(%.6e,%.6e)*vva=Eipar=(%.6e,%.6e)\n",receiverLaunchIndex.x,receiverLaunchIndex.y,hitPayload.hor_coeff.x,hitPayload.hor_coeff.y,hitPayload.ver_coeff.x,hitPayload.ver_coeff.y,Eipar.x,Eipar.y);
//	//aHit.h=hitPayload.hor_v;
	//aHit.v=hitPayload.ver_v;
	//aHit.Ex=Einorm;
	//aHit.Ey=Eipar;
	//aHit.Rp=hitPayload.hor_coeff;
	//aHit.Rn=hitPayload.ver_coeff;
	//float4 lh=make_float4(lastReflectionHitPoint,d);	
	//aHit.lh=lh;
	//aHit.lh=lastReflectionHitPoint;
	//aHit.in=receiverLaunchIndex;
	//	rtPrintf("TP\t%u\t%u\tRnorm(%.6e,%.6e)*hah + Rpar=(%.6e,%.6e)*vha=Einorm=(%.6e,%.6e)\n",receiverLaunchIndex.x,receiverLaunchIndex.y,hitPayload.hor_coeff.x,hitPayload.hor_coeff.y,hitPayload.ver_coeff.x,hitPayload.ver_coeff.y,Einorm.x,Einorm.y);
	//	rtPrintf("TP\t%u\t%u\tRnorm(%.6e,%.6e)*hva + Rpar=(%.6e,%.6e)*vva=Eipar=(%.6e,%.6e)\n",receiverLaunchIndex.x,receiverLaunchIndex.y,hitPayload.hor_coeff.x,hitPayload.hor_coeff.y,hitPayload.ver_coeff.x,hitPayload.ver_coeff.y,Eipar.x,Eipar.y);

	//****************************

	//Now we have the incident field separated in vertical and horizontal components. We should decide what we want to do with it. To get the induced voltage, we need to 
	//apply the dot product with the effective lenght of the received antenna. 
	//Transform the receiver polarization according to the incident ray direction (-ray.direction) to get the vertical and horizontal components of the receiver polarization
//
//	     float3 ver_o; //Receiver vertical field vector
//	     float3 hor_o; //Receiver horizontal field vector
//	
//	     //Reverse direction	
//	     const float3 ray=-ray_receiver.direction;	
//	
//	     //Get polarization for receiver for this reversed ray: 
//	     getLinearPolarizationInRayBasis(receiverPolarization, ray,  hor_o,ver_o);
//	
//	     //This would be equivalent to a dot product with the effective length (not the conjugated beacuse we already reversed and it is a linear polarization anyway)
//	     //Get the  components of received field for the normal and parallel field vectors (geometric projection on receiver polarization vectors times reflection coefficients)
//	     const float2 Einorm=sca_complex_prod(dot(hitPayload.hor_v,hor_o),Eih) + sca_complex_prod(dot(hitPayload.ver_v,hor_o),Eiv);
//	     const float2 Eipar=sca_complex_prod(dot(hitPayload.hor_v,ver_o),Eih) + sca_complex_prod(dot(hitPayload.ver_v,ver_o),Eiv);
//	
//	     
//	     //This is actually the induced voltage on the antenna. From it we can compute the received power
//	     float2 E=Einorm+Eipar;

	
	


	////TODO: move to separated programs or change to function
//	float attE=0.0f; //Defined here to be able to print below
//	if (usePenetration==1u) {
//		//Switch to linear
//		//attE=hitPayload.accumulatedAttenuation*0.05f; //Att(db)/20 From power att  to electric field amplitude
//		attE=hitPayload.hitPointAtt.w*0.05f;
//		//Check to avoid float overflows
//		if (attE>-15.f) {
//			attE=exp10f(attE); //10^(att/20) (att is negative)
//			//rtPrintf("Eatt=(%.10e,%.10e) attExp=%f hits=%u\n",E.x,E.y,attE,hitPayload.hits );
//		} else {
//			attE=1.e-15f; //Set this value directly to avoid overflows, it is neglectable anyway
//
//		}
//		E=sca_complex_prod(attE,E);
//
//	}
	//
	//
	//
	//Non-debug hit
//		HitInfo aHit;
//		aHit.thrd=make_uint4(txBufferIndex,hash,receiverBufferIndex,dtrx);
//		aHit.E=E;

	//Check if global buffer is full
	uint hitIndex=atomicAdd(&atomicIndex[0u],1u);
	if (hitIndex>=global_info_buffer_maxsize) {
		rtThrow(GLOBALHITBUFFEROVERFLOW);
		//if exceptions are disabled, let it crash...?
		hitIndex=global_info_buffer_maxsize-1;
	}
	//Store hit in global buffer
	globalHitInfoBuffer[hitIndex]=aHit;
	//		rtPrintf("DH\t%u\t%u\t%u\t%u\t%f\t%u\t%u\t%u\t%d\n", receiverLaunchIndex.x, receiverLaunchIndex.y,receiverBufferIndex, hitPayload.reflections,d,  aHit.thrd.y,aHit.thrd.z,aHit.thrd.w,externalId);

	//Log direct hit
	//	if (hitPayload.reflections==0 ) {
	//		rtPrintf("DH\t%u\t%u\t%u\t%u\t%f\t%f\t%f\t%f\t%u\t%u\t%u\t%d\n", receiverLaunchIndex.x, receiverLaunchIndex.y,receiverBufferIndex, hitPayload.reflections, attE,  E.x, E.y,d, aHit.thrd.x,aHit.thrd.y,aHit.thrd.w,externalId);
	//		rtPrintf("DH\t%u\t%u\tRnorm(%.6e,%.6e)Epolrx=(%.6e,%.6e)Rpar=(%.6e,%.6e)\n",receiverLaunchIndex.x,receiverLaunchIndex.y,hitPayload.hor_coeff.x,hitPayload.hor_coeff.y,Epolrx.x,Epolrx.y,hitPayload.ver_coeff.x,hitPayload.ver_coeff.y);
	//	}
}








rtDeclareVariable(LPWavePayload, missPayload, rtPayload, );
//Miss program. End ray
RT_PROGRAM void miss()
{
	//rtPrintf("miss i.x=%u. iy=%u \n", receiverLaunchIndex.x, receiverLaunchIndex.y);
	//missPayload.flags = FLAG_END;
	missPayload.rhfr.z= FLAG_END;
}

