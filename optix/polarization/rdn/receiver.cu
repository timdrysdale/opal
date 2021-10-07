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
rtDeclareVariable(uint, computeMode, ,);

//Transmitter buffer
rtBuffer<Transmitter, 1> txBuffer;

//RDN config buffer
rtBuffer<RDNParameters, 1> rdnParametersBuffer;

//Debug invalid rays
rtBuffer<int, 1> invalidRaysBuffer; //Buffer to store the number of invalid rays 


//Receiver local variables
rtDeclareVariable(uint, receiverBufferIndex, , ); //Buffer id
rtDeclareVariable(int, externalId, , ); //External id  used to identify receivers 
rtDeclareVariable(SphereHit, hit_attr, attribute hit_attr, );
//rtDeclareVariable(float, k, , );
rtDeclareVariable(float4, sphere, , );
rtDeclareVariable(float3, receiverPolarization, , );

//Launch variables
rtDeclareVariable(uint3, receiverLaunchIndex, rtLaunchIndex, );
rtDeclareVariable(RDNLPWavePayload, hitPayload, rtPayload, );
rtDeclareVariable(optix::Ray, ray_receiver, rtCurrentRay, );

//Global variables
//rtDeclareVariable(float, asRadiusConstant, ,);




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
	//
	//	//Update ray data
	//	const float rayLength = hit_attr.geom_normal_t.w;
	//	const float3 hitPoint = ray_receiver.origin + rayLength*ray_receiver.direction;
	//	//hitPayload.hitPoint=hitPoint;
	//	hitPayload.hitPointAtt.x=hitPoint.x;
	//	hitPayload.hitPointAtt.y=hitPoint.y;
	//	hitPayload.hitPointAtt.z=hitPoint.z;
	//	const float aux= hitPayload.ndtd.w; //Previous total distance
	//	hitPayload.ndtd = make_float4(ray_receiver.direction); //Next direction only
	//	hitPayload.ndtd.w = aux+ rayLength; // update totalDistance 
	//
	//#ifdef OPAL_AVOID_SI
	//	hitPayload.lastNormal=make_float3(hit_attr.geom_normal_t.x,hit_attr.geom_normal_t.y,hit_attr.geom_normal_t.z);
	//	//hitPayload.lastNormal=normalize(hitPoint-prx);
	//#endif
	//

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




	const float2 ds=distancesToReceptionPoint<RDNLPWavePayload>(hitPayload, hitPoint, prx, ray_receiver.direction);
	const float sqrDistanceReceiverPointToReceiver=ds.y;
	//float radDistance;
	//if (reflections==0) { 
	//	radDistance=length(prx-current_tx.origin);
	//} else {
	//	radDistance=distancesProjectedToReceptionPoint<RDNLPWavePayload>(hitPayload,prx, ray_receiver.direction);
	//	const float3 lhp = make_float3(hitPayload.lrhpd.x,hitPayload.lrhpd.y,hitPayload.lrhpd.z);
	//	float drxhp=length(prx-lhp);
	//	rtPrintf("%u%u RD rD=%f ds=%f drxhp=%f  hp=(%f,%f,%f) dtor=%f\n",receiverLaunchIndex.x, receiverLaunchIndex.y, radDistance,ds.x,drxhp,lhp.x,lhp.y,lhp.z,sqrt(sqrDistanceReceiverPointToReceiver));
	//}
	//Add previous distance to last reflection to get the total path length of the ray so far (unfolded path length)

	const float unfoldedPathLength = ds.x + hitPayload.lrhpd.w;

	//const float unfoldedPathLength = radDistance + hitPayload.lrhpd.w;


	//const float unfoldedPathLength = length(prx-current_tx.origin);
	//Get distance from last reflection to reception point to compute divergence below
	float s=ds.x;



	//rtPrintf("DS\t%u\t%u\tp3=(%.6e,%.6e,%.6e) drxtohitsq=%.6e dm=%.6e dmt=%d dtrx=%u hash=%u\n",receiverLaunchIndex.x,receiverLaunchIndex.y,p3.x,p3.y,p3.z,drxtohitsq,dm,dmt,dtrx,hash);



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
	float rayDensity = hitPayload.rayDensity;
	//Direct ray contribution
	if (reflections==0) {
		//For the direct ray we should incorporate the first segment attenuation to the divergence, just 1/unfolded_path
		divergence=divergence*(1.0f/unfoldedPathLength); //The total distance has already been updated.
		rayDensity = rayDensity/(unfoldedPathLength*unfoldedPathLength);
		//rtPrintf("HCDIR\t%u\t%u\t%u\t%u divergence=%6e,%.6e density=%f d=%f rayLength=%f s=%f\n",receiverBufferIndex,receiverLaunchIndex.x,receiverLaunchIndex.y,reflections,divergence.x,divergence.y, rayDensity,unfoldedPathLength,rayLength,s);
	} else {
		float rho1=hitPayload.radii.x;
		float rho2=hitPayload.radii.y;
		divergence=computeDivergence(rho1,rho2,s,hitPayload.divergence); 
		float sq1=rho1/(rho1+s);
		float sq2=rho2/(rho2+s);
		//What if caustics are infinity
		//If we apply the L'Hôpital rule, the fraction goes to 1...
		if (isinf(rho1)) {
			sq1=1;
		}
		if (isinf(rho2)) {
			sq2=1;
		}

		rayDensity = rayDensity*abs(sq1*sq2);
		rtPrintf("HCDIV\t%u\t%u\t%u\t%u rho1=%f,rho2=%f,divergence=%6e,%.6e preDen=%.6e density=%.6e d=%f rayLength=%f s=%f\n",receiverBufferIndex,receiverLaunchIndex.x,receiverLaunchIndex.y,reflections,rho1,rho2,divergence.x,divergence.y, hitPayload.rayDensity, rayDensity,unfoldedPathLength,rayLength,s);
	}	
	float A=M_PIf*(sphere.w*sphere.w);
	//M is the expected number of rays with this density, used to normalize later. The additional 2 here is because we get two hits on the sphere per ray intersecting: one when enters and one when leaves. So we sum twice later
	// each ray.
	float M=A*rayDensity*2;
	//Filtering rays with low density
	const unsigned int filterDensity = rdnParametersBuffer[0].filter_rc_nrx.x;	
	//unsigned int filterDensity = rdnParametersBuffer[0].filterDensity;	
	if (filterDensity ==1) {
		if (M<1) {
			return;
		}
	} else if (filterDensity ==2) {
		if (M<1) {
			M=1;
		}

	}
	Eih = complex_prod(divergence, Eih);
	Eiv = complex_prod(divergence, Eiv);

	//Now we have the incident field separated in vertical and horizontal components. We should decide what we want to do with it.  


	if (isnan(divergence.x) || isnan(divergence.y) || isnan(rayDensity) || (rayDensity==0)){
		rtPrintf("HCNnan\t%u\t%u\t%u\t%u rho1=%f,rho2=%f,hp.divergence=(%6e,%.6e), divergence=(%6e,%.6e)s=%f Eih=(%.6e,%.6e)\n",receiverBufferIndex,receiverLaunchIndex.x,receiverLaunchIndex.y,reflections,hitPayload.radii.x,hitPayload.radii.y,hitPayload.divergence.x,hitPayload.divergence.y,divergence.x,divergence.y,s, Eih.x,Eiv.y);
		//At the moment just return and do not store the hit
		atomicAdd(&invalidRaysBuffer[0],1);
		return;
	}		





	//Fill common part of hitinfo

	HitInfo aHit;
	fillHitInfo<RDNLPWavePayload>(hitPayload,aHit,txBufferIndex, receiverBufferIndex,sqrDistanceReceiverPointToReceiver);

	//Compute FIELD
	if (computeMode==1) {

		//**************+  Get the field components at the receiver point (ignoring receiver antenna): project on x, y and z axis **************
		float3 xu=make_float3(1.0f,0.0f,0.0f);
		float3 yu=make_float3(0.0f,1.0f,0.0f);
		float3 zu=make_float3(0.0f,0.0f,1.0f);
		float2 Einorm=sca_complex_prod(dot(hitPayload.hor_v,xu),Eih) + sca_complex_prod(dot(hitPayload.ver_v,xu),Eiv);
		float2 Eipar=sca_complex_prod(dot(hitPayload.hor_v,yu),Eih) + sca_complex_prod(dot(hitPayload.ver_v,yu),Eiv);
		float2 Eirad=sca_complex_prod(dot(hitPayload.hor_v,zu),Eih) + sca_complex_prod(dot(hitPayload.ver_v,zu),Eiv);

		Einorm=sca_complex_prod(1.0f/M,Einorm); 
		Eipar=sca_complex_prod(1.0f/M,Eipar); 
		Eirad=sca_complex_prod(1.0f/M,Eirad); 
		aHit.EEx=make_float4(make_float2(0.0f,0.0f),Einorm);
		aHit.EyEz=make_float4(Eipar,Eirad);	
		//aHit.Ex=Einorm;
		//aHit.Ey=Eipar;
		//aHit.Ez=Eirad;
		//aHit.r=reflections;
		//aHit.m=A*rayDensity*2;
		//aHit.updateDivergence = hitPayload.updateDivergence;
		//rtPrintf("DH\t%u\t%u\t%u\t%u\t%f\t%f\t%f\t%f\t%f\t%f\t%f\n", receiverLaunchIndex.x, receiverLaunchIndex.y,receiverBufferIndex, reflections,d, aHit.Ey.x,aHit.Ey.y,aHit.divergence.x,aHit.divergence.y,aHit.rhos.x,aHit.rhos.y );


	}
	//****************************
	//#else
	//Compute VOLTAGE
	if (computeMode==0) {
		//To get the induced voltage, we need to 
		//apply the dot product with the effective lenght of the received antenna. 
		//Transform the receiver polarization according to the incident ray direction (-ray.direction) to get the vertical and horizontal components of the receiver polarization
		//rtPrintf("HCN\t%u\t%u\t%u\t%u rho1=%f,rho2=%f,hp.divergence=(%6e,%.6e), divergence=(%6e,%.6e)s=%f Eih=(%.6e,%.6e)\n",receiverBufferIndex,receiverLaunchIndex.x,receiverLaunchIndex.y,reflections,hitPayload.radii.x,hitPayload.radii.y,hitPayload.divergence.x,hitPayload.divergence.y,divergence.x,divergence.y,s, Eih.x,Eih.y);
		//rtPrintf("HCN\t%u\t%u\t%u\t%u rho1=%f,rho2=%f,hp.divergence=(%6e,%.6e), divergence=(%6e,%.6e)s=%f Eih=(%.6e,%.6e)\n",receiverBufferIndex,receiverLaunchIndex.x,receiverLaunchIndex.y,reflections,hitPayload.radii.x,hitPayload.radii.y,hitPayload.divergence.x,hitPayload.divergence.y,divergence.x,divergence.y,s, Eih.x,Eih.y);

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

		//float A=M_PIf*(sphere.w*sphere.w);
		//float solid_angle=2*M_PIf*(1-cosf(asinf(sphere.w/d)));
		E=sca_complex_prod(1.0f/M,E); 

		//E=sca_complex_prod(1.0f/(rayDensity*solid_angle*d*d*2),E); //Multiplied by 2 because we actually record two hits per intersection (entering and leaving the sphere)

		if (useAntennaGain) {
		
			float g=getAntennaGain(ray, gainBufferId,transformToPolarization);	
			E=sca_complex_prod(g,E);
		}


		//Non-debug hit
		//aHit.E=E;
		aHit.EEx=make_float4(E, make_float2(0.0f,0.0f));
		aHit.EyEz=make_float4(0.0f,0.f,0.0f,0.0f);	

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








rtDeclareVariable(RDNLPWavePayload, missPayload, rtPayload, );
//Miss program. End ray
RT_PROGRAM void miss()
{
	//rtPrintf("miss i.x=%u. iy=%u \n", receiverLaunchIndex.x, receiverLaunchIndex.y);
	//missPayload.flags = FLAG_END;
	missPayload.rhfr.z |= 1u<<FLAG_END_POSITION;
	//missPayload.rhfr.z= FLAG_END;
}

