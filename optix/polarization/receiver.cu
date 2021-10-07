/***************************************************************/
//
//Copyright (c) 2019 Esteban Egea-Lopez http://girtel.upct.es/~eegea
//
/**************************************************************/


#include "../../Common.h"
#include "../configuration.h"
#include "../Complex.h"
#include "../receiverFunctions.h"
#include "../penetrationFunctions.h"

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
//rtDeclareVariable(float, k, , );
rtDeclareVariable(float4, sphere, , );
rtDeclareVariable(float3, receiverPolarization, , );
//Launch variables
rtDeclareVariable(uint3, receiverLaunchIndex, rtLaunchIndex, );
rtDeclareVariable(LPWavePayload, hitPayload, rtPayload, );
rtDeclareVariable(optix::Ray, ray_receiver, rtCurrentRay, );

//Global variables
//rtDeclareVariable(float, asRadiusConstant, ,);
rtDeclareVariable(uint, computeMode, ,);

//Penetration configuration
//rtDeclareVariable(uint, usePenetration, , );



//Antenna gain
typedef rtBufferId<float,2> AGB;
rtDeclareVariable(AGB, gainBufferId, ,);
typedef optix::Matrix<4,4> TransMat; 
rtDeclareVariable(TransMat, transformToPolarization, ,);

//Closest hit program for receivers
RT_PROGRAM void closestHitReceiver()
{


	//Do not end the ray, it can pass through the reception sphere and reflect on a wall, inside or outside the receiver sphere

	//Update ray data
	const float rayLength = hit_attr.geom_normal_t.w; //distance from last ray origin to this hit point
	const float3 hitPoint = updateReceiverPayload<LPWavePayload>(hitPayload,rayLength,ray_receiver);
	

	const uint txBufferIndex=receiverLaunchIndex.z;
	const Transmitter current_tx=txBuffer[txBufferIndex];
	float3 prx = make_float3(sphere.x, sphere.y, sphere.z);

	//Check if ray is hitting his own tx (transmitter are also receivers usually) A transmitter cannot receive while it is transmitting, unless other channel is used.
	if (externalId == current_tx.externalId) {
		//My own outgoing ray
		//rtPrintf("External. txId=%d i.el=%u i.az=%u, ray=(%f,%f,%f) origin=(%f,%f,%f) t=%f rId[%u]=%d\n", current_tx.externalId, receiverLaunchIndex.x, receiverLaunchIndex.y, ray_receiver.direction.x, ray_receiver.direction.y, ray_receiver.direction.z, ray_receiver.origin.x, ray_receiver.origin.y, ray_receiver.origin.z, hit_attr.geom_normal_t, receiverBufferIndex,externalId);
		return;
	}



	//We compute the "reception point": since the ray does not hit the receiver but some point on the sphere, we
	//have to decide what is the reception point.
	const float2 ds=distancesToReceptionPoint<LPWavePayload>(hitPayload, hitPoint, prx, ray_receiver.direction);
	const float sqrDistanceReceiverPointToReceiver=ds.y;

	//Add previous distance to last reflection to get the total path length of the ray so far (unfolded path length)
	const float unfoldedPathLength = ds.x + hitPayload.lrhpd.w;
	





	//Compute incident electric field.  

	//Compute phase
	float k = hitPayload.polarization_k.w;	
	float2 z = make_float2(0.0f, -k*unfoldedPathLength);
	float2 zexp = complex_exp_only_imaginary(z);

	//Multiply by reflection coefficient
	float2 Rzexph = complex_prod(hitPayload.hor_coeff, zexp);
	float2 Rzexpv = complex_prod(hitPayload.ver_coeff, zexp);


	//Apply propagation loss
	float2 Eih = sca_complex_prod((hitPayload.electricFieldAmplitude / unfoldedPathLength), Rzexph);
	float2 Eiv = sca_complex_prod((hitPayload.electricFieldAmplitude / unfoldedPathLength), Rzexpv);


	//At this point we have the incident field separated in vertical and horizontal components. We decide what we want to do with it below 


	//Store hit information on buffer:
	
	HitInfo aHit;
	//Fill common part of hitinfo
 	fillHitInfo<LPWavePayload>(hitPayload,aHit,txBufferIndex, receiverBufferIndex, sqrDistanceReceiverPointToReceiver);


//Compute VOLTAGE
if (computeMode==0) {

	//****************************
	//To get the induced voltage, we need to 
	//apply the dot product with the effective lenght of the received antenna. 
	//Transform the receiver polarization according to the incident ray direction (-ray.direction) to get the vertical and horizontal components of the receiver polarization

	float3 ver_o; //Receiver vertical field vector
	float3 hor_o; //Receiver horizontal field vector

	//Reverse direction	
	const float3 ray=-ray_receiver.direction;	

	//Get polarization for receiver for this reversed ray: 
	getLinearPolarizationInRayBasis(receiverPolarization, ray,  hor_o,ver_o);

	//This would be equivalent to a dot product with the effective length (not the conjugated beacuse we already reversed and it is a linear polarization anyway)
	//Get the  components of received field for the normal and parallel field vectors (geometric projection on receiver polarization vectors times reflection coefficients)
	const float2 Einorm=sca_complex_prod(dot(hitPayload.hor_v,hor_o),Eih) + sca_complex_prod(dot(hitPayload.ver_v,hor_o),Eiv);
	const float2 Eipar=sca_complex_prod(dot(hitPayload.hor_v,ver_o),Eih) + sca_complex_prod(dot(hitPayload.ver_v,ver_o),Eiv);


	//This is actually the induced voltage on the antenna. From it we can compute the received power
	float2 E=Einorm+Eipar;


	if (usePenetration==1u) {
		E=applyPenetration<LPWavePayload>(hitPayload,E);

	}
	if (useAntennaGain) {
		
		float g=getAntennaGain(ray, gainBufferId,transformToPolarization);	
		E=sca_complex_prod(g,E);
	}
	//Non-debug hit
	//aHit.E=E;
	aHit.EEx=make_float4(E, make_float2(0.0f,0.0f));
	aHit.EyEz=make_float4(0.0f,0.f,0.0f,0.0f);
	aHit.doaD = make_float4(ray.x, ray.y,ray.z, unfoldedPathLength);	
	//printf("DH\t%u\t%u\t%u\t%u\t%f\t%f\t%f\t%u\t%u\t%u\t%d\n", receiverLaunchIndex.x, receiverLaunchIndex.y,receiverBufferIndex, hitPayload.rhfr.x,unfoldedPathLength, E.x,E.y, aHit.thrd.y,aHit.thrd.z,aHit.thrd.w,externalId);
}	
	//aHit.r=reflections;
	//rtPrintf("DH\t%u\t%u\t%u\t%u\t%f\t%f\t%f\t%u\t%u\t%u\t%d\n", receiverLaunchIndex.x, receiverLaunchIndex.y,receiverBufferIndex, reflections,d, E.x,E.y, aHit.thrd.y,aHit.thrd.z,aHit.thrd.w,externalId);
	//float2 ang= getAngles(ray_receiver.direction);
	//rtPrintf("H\t%u\t%u\t%u\t%u\t%f\t%f\t%f\t%f\t%f\n", receiverLaunchIndex.x, receiverLaunchIndex.y,receiverBufferIndex, reflections,d, E.x,E.y, (ang.x*180.0f/M_PIf), (ang.y*180.0f/M_PIf));


//Compute FIELD
if (computeMode==1) {

	//**************+ . Get the field components at the receiver point (ignoring receiver antenna): project on x, y and z axis **************
	float3 xu=make_float3(1.0f,0.0f,0.0f);
	float3 yu=make_float3(0.0f,1.0f,0.0f);
	float3 zu=make_float3(0.0f,0.0f,1.0f);
	const float2 Einorm=sca_complex_prod(dot(hitPayload.hor_v,xu),Eih) + sca_complex_prod(dot(hitPayload.ver_v,xu),Eiv);
	const float2 Eipar=sca_complex_prod(dot(hitPayload.hor_v,yu),Eih) + sca_complex_prod(dot(hitPayload.ver_v,yu),Eiv);
	const float2 Eirad=sca_complex_prod(dot(hitPayload.hor_v,zu),Eih) + sca_complex_prod(dot(hitPayload.ver_v,zu),Eiv);
	
	aHit.EEx=make_float4(make_float2(0.0f,0.0f),Einorm);
	aHit.EyEz=make_float4(Eipar,Eirad);	
	//aHit.Ex=Einorm;
	//aHit.Ey=Eipar;
	//aHit.Ez=Eirad;
//	aHit.r=reflections;


} 



	//Check if global buffer is full
	uint hitIndex=atomicAdd(&atomicIndex[0u],1u);
	if (hitIndex>=global_info_buffer_maxsize) {
		rtThrow(GLOBALHITBUFFEROVERFLOW);
		//if exceptions are disabled, let it crash...?
		hitIndex=global_info_buffer_maxsize-1;
	}
	//Store hit in global buffer
	globalHitInfoBuffer[hitIndex]=aHit;


}








rtDeclareVariable(LPWavePayload, missPayload, rtPayload, );
//Miss program. End ray
RT_PROGRAM void miss()
{
	//rtPrintf("miss i.x=%u. iy=%u \n", receiverLaunchIndex.x, receiverLaunchIndex.y);
	//missPayload.flags = FLAG_END;
	missPayload.rhfr.z |= 1u<<FLAG_END_POSITION;
	//missPayload.rhfr.z= FLAG_END;
}

