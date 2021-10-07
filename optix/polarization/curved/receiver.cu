/***************************************************************/
//
//Copyright (c) 2019 Esteban Egea-Lopez http://girtel.upct.es/~eegea
//
/**************************************************************/


#include "../../../Common.h"
#include "../../Complex.h"
#include "../../configuration.h"
#include "../../penetrationFunctions.h"
#include "../../receiverFunctions.h"

#include "../linearPolarizationFunctions.h"
#include "../../curvedFunctions.h"
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
rtDeclareVariable(CurvedMeshLPWavePayload, hitPayload, rtPayload, );
rtDeclareVariable(optix::Ray, ray_receiver, rtCurrentRay, );

//Global variables
rtDeclareVariable(uint, computeMode, ,);

//Penetration configuration
//rtDeclareVariable(uint, usePenetration, , );


//Antenna gain
typedef rtBufferId<float,2> AGB;
rtDeclareVariable(AGB, gainBufferId, ,);
typedef optix::Matrix<4,4> TransMat; 
rtDeclareVariable(TransMat, transformToPolarization, ,);



//Closest hit program for receivers in scenarios with curved and flat walls
RT_PROGRAM void closestHitReceiverCurved()
{


	//Do not end the ray, it can pass through the reception sphere and reflect on a wall, inside or outside the receiver sphere
	//Update ray data
	const float rayLength = hit_attr.geom_normal_t.w; //distance from last ray origin to this hit point
	const float3 hitPoint = updateReceiverPayload<CurvedMeshLPWavePayload>(hitPayload,rayLength,ray_receiver);

	const uint txBufferIndex=receiverLaunchIndex.z;
	const Transmitter current_tx=txBuffer[txBufferIndex];
	float3 prx = make_float3(sphere.x, sphere.y, sphere.z);

	//Check if ray is hitting his own tx (transmitter are also receivers usually) A transmitter cannot receive while it is transmitting, unless other channel is used.
	if (externalId == current_tx.externalId) {
		//My own outgoing ray
		//rtPrintf("External. txId=%d i.el=%u i.az=%u, ray=(%f,%f,%f) origin=(%f,%f,%f) t=%f rId[%u]=%d\n", current_tx.externalId, receiverLaunchIndex.x, receiverLaunchIndex.y, ray_receiver.direction.x, ray_receiver.direction.y, ray_receiver.direction.z, ray_receiver.origin.x, ray_receiver.origin.y, ray_receiver.origin.z, hit_attr.geom_normal_t, receiverBufferIndex,externalId);
		return;
	}


	//int reflections = hitPayload.reflections;
	uint reflections = hitPayload.rhfr.x;



	//We compute the "reception point": since the ray does not hit the receiver but some point on the sphere, we
	//have to decide what is the reception point.
	const float2 ds=distancesToReceptionPoint<CurvedMeshLPWavePayload>(hitPayload, hitPoint, prx, ray_receiver.direction);
	const float sqrDistanceReceiverPointToReceiver=ds.y;

	//Add previous distance to last reflection to get the total path length of the ray so far (unfolded path length)
	const float unfoldedPathLength = ds.x + hitPayload.lrhpd.w;
	//Get distance from last reflection to reception point to compute divergence below
	float s=ds.x;


	//Compute incident electric field.  

	//Compute phase
	float k = hitPayload.polarization_k.w;	
	float2 z = make_float2(0.0f, -k*unfoldedPathLength);
	float2 zexp = complex_exp_only_imaginary(z);


	float2 Rzexph = complex_prod(hitPayload.hor_coeff, zexp);
	float2 Rzexpv = complex_prod(hitPayload.ver_coeff, zexp);



	//For curved surfaces the propagation attenuation is already included in the divergence
	//So we only include the phase and reflections coeffients 
	float2 Eih =  Rzexph;
	float2 Eiv =  Rzexpv;


	float2 divergence=hitPayload.divergence;
	//Direct ray contribution
	if (reflections==0) {
		//For the direct ray we should incorporate the first segment attenuation to the divergence, just 1/unfolded_path
		divergence=divergence*(1.0f/unfoldedPathLength); //The total distance has already been updated.
		//rayDensity = rayDensity/(d*d);
		//rtPrintf("DH d=%f divergence=(%f,%f) \n",unfoldedPathLength,divergence.x,divergence.y);
	} else {
		//We just update the value with the distance to last physical interactions but do not change the payload, to be updated in a future reflection
		float rho1=hitPayload.radii.x;
		float rho2=hitPayload.radii.y;
		divergence=computeDivergence(rho1,rho2,s,hitPayload.divergence); 
		//divergence=computeDivergence(rho1,rho2,s,hitPayload.divergence, receiverLaunchIndex.x,receiverLaunchIndex.y,reflections); 
		//	if (isnan(divergence.x) || isnan(divergence.y)){
		rtPrintf("HCDIV\t%u\t%u\t%u\t%u rho1=%f,rho2=%f,divergence=%6e,%.6e  rayLength=%f s=%f\n",receiverBufferIndex,receiverLaunchIndex.x,receiverLaunchIndex.y,reflections,rho1,rho2,divergence.x,divergence.y,rayLength,s);
		//	}
		//	hitPayload.updateDivergence +=1u;
	}	

	Eih = complex_prod(divergence, Eih);
	Eiv = complex_prod(divergence, Eiv);



	if (isnan(divergence.x) || isnan(divergence.y)){
		//rtPrintf("HCNnan\t%u\t%u\t%u\t%u rho1=%f,rho2=%f,hp.divergence=(%6e,%.6e), divergence=(%6e,%.6e)s=%f Eih=(%.6e,%.6e)\n",receiverBufferIndex,receiverLaunchIndex.x,receiverLaunchIndex.y,reflections,hitPayload.radii.x,hitPayload.radii.y,hitPayload.divergence.x,hitPayload.divergence.y,divergence.x,divergence.y,s, Eih.x,Eiv.y);
		//Receiver is on a caustic. Ignore hit at the moment
		return;
	}		



	//	if (usePenetration==1u) {
	//		E=applyPenetration<CurvedMeshLPWavePayload>(hitPayload,E);
	//
	//	}



	HitInfo aHit;
	//Fill common part of hitinfo
	fillHitInfo<LPWavePayload>(hitPayload,aHit,txBufferIndex, receiverBufferIndex, sqrDistanceReceiverPointToReceiver);

	//Compute FIELD
	if (computeMode==1) {


		//**************+ Tests and debug. Get the field components at the receiver point (ignoring receiver antenna): project on x, y and z axis **************
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
		//aHit.r=reflections;
		//aHit.updateDivergence = hitPayload.updateDivergence;
		//rtPrintf("DH\t%u\t%u\t%u\t%u\t%f\t%f\t%f\t%f\t%f\t%f\t%f\n", receiverLaunchIndex.x, receiverLaunchIndex.y,receiverBufferIndex, reflections,d, aHit.Ey.x,aHit.Ey.y,aHit.divergence.x,aHit.divergence.y,aHit.rhos.x,aHit.rhos.y );



		//****************************
	} 
	//Compute VOLTAGE
	if (computeMode==0) {
		//To get the induced voltage, we need to 
		//apply the dot product with the effective lenght of the received antenna. 
		//Transform the receiver polarization according to the incident ray direction (-ray.direction) to get the vertical and horizontal components of the receiver polarization

		float3 ver_o; //Receiver vertical field vector
		float3 hor_o; //Receiver horizontal field vector

		//Reverse direction	
		const float3 ray=-ray_receiver.direction;	

		//Get polarization for receiver for this reversed ray: 
		getLinearPolarizationInRayBasis(receiverPolarization, ray,  hor_o,ver_o);
		//rtPrintf("VCN\t%u\t%u\t%u\t%u Eiv=(%.6e,%.6e) hor_o=(%f,%f,%f) ver_o=(%f,%f,%f)\n",receiverBufferIndex,receiverLaunchIndex.x,receiverLaunchIndex.y,reflections,Eiv.x,Eiv.y,hor_o.x,hor_o.y,hor_o.z,ver_o.x,ver_o.y,ver_o.z);

		//This would be equivalent to a dot product with the effective length (not the conjugated beacuse we already reversed and it is a linear polarization anyway)
		//Get the  components of received field for the normal and parallel field vectors (geometric projection on receiver polarization vectors times reflection coefficients)
		const float2 Einorm=sca_complex_prod(dot(hitPayload.hor_v,hor_o),Eih) + sca_complex_prod(dot(hitPayload.ver_v,hor_o),Eiv);
		const float2 Eipar=sca_complex_prod(dot(hitPayload.hor_v,ver_o),Eih) + sca_complex_prod(dot(hitPayload.ver_v,ver_o),Eiv);


		//This is actually the induced voltage on the antenna. From it we can compute the received power
		float2 E=Einorm+Eipar;

		if (useAntennaGain) {

			float g=getAntennaGain(ray, gainBufferId,transformToPolarization);	
			E=sca_complex_prod(g,E);
		}

		aHit.EEx=make_float4(E, make_float2(0.0f,0.0f));
		aHit.EyEz=make_float4(0.0f,0.f,0.0f,0.0f);	
		aHit.doaD = make_float4(ray.x, ray.y,ray.z, unfoldedPathLength);	
		//aHit.E=E;
		//aHit.r=reflections;
		//	rtPrintf("DH\t%u\t%u\t%u\t%u\t%f\t%f\t%f\t%u\t%u\t%u\t%d\n", receiverLaunchIndex.x, receiverLaunchIndex.y,receiverBufferIndex, reflections,d, E.x,E.y, aHit.thrd.y,aHit.thrd.z,aHit.thrd.w,externalId);

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








rtDeclareVariable(CurvedMeshLPWavePayload, missPayload, rtPayload, );
//Miss program. End ray
RT_PROGRAM void miss()
{
	//rtPrintf("miss i.x=%u. iy=%u \n", receiverLaunchIndex.x, receiverLaunchIndex.y);
	//missPayload.flags = FLAG_END;
	missPayload.rhfr.z |= 1u<<FLAG_END_POSITION;
	//missPayload.rhfr.z= FLAG_END;
}

