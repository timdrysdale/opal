/***************************************************************/
//
//Copyright (c) 2019 Esteban Egea-Lopez http://girtel.upct.es/~eegea
//
/**************************************************************/


#include "../../../Common.h"
#include "../../Complex.h"
#include "../../penetrationFunctions.h"
#include "../../receiverFunctions.h"
#include "../../configuration.h"

#include "../linearPolarizationFunctions.h"
#include "../../curvedFunctions.h"
#include <optix_world.h>
#include <optixu/optixu_math_namespace.h>
#include <optixu/optixu_aabb_namespace.h>
using namespace optix;



//Receiver global buffers
rtBuffer<RDNHit, 3> frBuffer; //Buffer to store all the field for each ray and receiver

//Transmitter buffer
rtBuffer<Transmitter, 1> txBuffer;

//RDN config buffer
rtBuffer<RDNParameters, 1> rdnParametersBuffer;

//Debug invalid rays
//rtBuffer<int, 1> invalidRaysBuffer; //Buffer to store the number of invalid rays 


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
rtDeclareVariable(uint, computeMode, ,);


//Antenna gain
typedef rtBufferId<float,2> AGB;
rtDeclareVariable(AGB, gainBufferId, ,);
typedef optix::Matrix<4,4> TransMat; 
rtDeclareVariable(TransMat, transformToPolarization, ,);




//RDN with power trace for NOATOMIC. We do not use atomic functions and accumulate field per ray on the frBuffer
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


	const float2 ds=distancesToReceptionPoint<CurvedMeshLPWavePayload>(hitPayload, hitPoint, prx, ray_receiver.direction);
	const float sqrDistanceReceiverPointToReceiver=ds.y;

	//Add previous distance to last reflection to get the total path length of the ray so far (unfolded path length)
	const float unfoldedPathLength = ds.x + hitPayload.lrhpd.w;
	//const float unfoldedPathLength = length(prx-current_tx.origin);
	//Get distance from last reflection to reception point to compute divergence below
	float s=ds.x;



	//rtPrintf("DS\t%u\t%u\tp3=(%.6e,%.6e,%.6e) drxtohitsq=%.6e dm=%.6e dmt=%d dtrx=%u hash=%u\n",receiverLaunchIndex.x,receiverLaunchIndex.y,p3.x,p3.y,p3.z,drxtohitsq,dm,dmt,dtrx,hash);



	float2 divergence=hitPayload.divergence;
	float rayDensity = hitPayload.rayDensity;
	//Direct ray contribution
	int addJump=0;
	if (reflections==0) {
		//For the direct ray we should incorporate the first segment attenuation to the divergence, just 1/unfolded_path
		divergence=divergence*(1.0f/unfoldedPathLength); //The total distance has already been updated.
		rayDensity = rayDensity/(unfoldedPathLength*unfoldedPathLength);
		//rtPrintf("HCDIR\t%u\t%u\t%u\t%u divergence=%6e,%.6e density=%f d=%f rayLength=%f s=%f\n",receiverBufferIndex,receiverLaunchIndex.x,receiverLaunchIndex.y,reflections,divergence.x,divergence.y, rayDensity,unfoldedPathLength,rayLength,s);
	} else {
		float rho1=hitPayload.radii.x;
		float rho2=hitPayload.radii.y;
		//divergence=computeDivergence(rho1,rho2,s,hitPayload.divergence); 
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
		if ((sq1*sq2)<0) {
			addJump +=1;	
		}

		rayDensity = rayDensity*abs(sq1*sq2);
		//rtPrintf("HCDIV\t%u\t%u\t%u\t%u rho1=%f,rho2=%f,divergence=%6e,%.6e preDen=%.6e density=%.6e d=%f rayLength=%f s=%f\n",receiverBufferIndex,receiverLaunchIndex.x,receiverLaunchIndex.y,reflections,rho1,rho2,divergence.x,divergence.y, hitPayload.rayDensity, rayDensity,unfoldedPathLength,rayLength,s);
	}	
	//Compute incident electric field.  

	//Compute phase
	//float2 z = make_float2(0.0f, -k*unfoldedPathLength);
	//Include phase jumps
	float k = hitPayload.polarization_k.w;	
	float2 z = make_float2(0.0f, -((k*unfoldedPathLength) - ((hitPayload.phaseJumps+addJump)*M_PI_2f) ));
	float2 zexp = complex_exp_only_imaginary(z);


	float2 Rzexph = complex_prod(hitPayload.hor_coeff, zexp);
	float2 Rzexpv = complex_prod(hitPayload.ver_coeff, zexp);



	//For curved surfaces the propagation attenuation is already included in the divergence
	//So we only include the phase and reflections coeffients 
	float2 Eih =  Rzexph;
	float2 Eiv =  Rzexpv;

	//RDN part
	const unsigned int filterDensity = rdnParametersBuffer[0].filter_rc_nrx.x;	
	const unsigned int rayCount = rdnParametersBuffer[0].filter_rc_nrx.y;	
	const float initialDensity = rdnParametersBuffer[0].initialDensity;
	float A=M_PIf*(sphere.w*sphere.w);
	//M is the expected number of rays with this density, used to normalize later. The additional 2 here is because we get two hits on the sphere per ray intersecting: one when enters and one when leaves. So we sum twice later
	// each ray. It is easier (and probably more efficient) to just collect all hits than to try keep one hit. But this consumes twice memory on the hitInfoBuffer

	float M=A*rayDensity*2;
	//Filtering rays with low density
	if (filterDensity ==1) {
		if (M<1) {
			//atomicAdd(&invalidRaysBuffer[0],1);
			return;
		}
	} else if (filterDensity ==2) {
		if (M<1) {
			M=1;
		}

	}
	//The coherent coefficient (X_P^c in Didascalou notation) is modified below to take into account that our M is related to Didascalou M' as  M=M'*2. Then sqrt(initialDensity*rayDensity/M')=0.5*sqrt(2/(A*M*initialDensity)
	float fr=0.5*sqrt(2/(A*M*initialDensity));



	//Now we have the incident field separated in vertical and horizontal components. We should decide what we want to do with it.  


	if (isnan(divergence.x) || isnan(divergence.y) || isnan(rayDensity) || (rayDensity==0)){
		rtPrintf("HCNnan\t%u\t%u\t%u\t%u rho1=%f,rho2=%f,hp.divergence=(%6e,%.6e), divergence=(%6e,%.6e)s=%f Eih=(%.6e,%.6e)\n",receiverBufferIndex,receiverLaunchIndex.x,receiverLaunchIndex.y,reflections,hitPayload.radii.x,hitPayload.radii.y,hitPayload.divergence.x,hitPayload.divergence.y,divergence.x,divergence.y,s, Eih.x,Eiv.y);
		//At the moment just return and do not store the hit
		//atomicAdd(&invalidRaysBuffer[0],1);
		return;
	}		


	uint3 bindex=make_uint3(receiverBufferIndex,receiverLaunchIndex.x,receiverLaunchIndex.y);

	//Compute FIELD
	if (computeMode==1) {


		//**************+ Tests and debug. Get the field components at the receiver point (ignoring receiver antenna): project on x, y and z axis **************
		float3 xu=make_float3(1.0f,0.0f,0.0f);
		float3 yu=make_float3(0.0f,1.0f,0.0f);
		float3 zu=make_float3(0.0f,0.0f,1.0f);
		float2 Einorm=sca_complex_prod(dot(hitPayload.hor_v,xu),Eih) + sca_complex_prod(dot(hitPayload.ver_v,xu),Eiv);
		float2 Eipar=sca_complex_prod(dot(hitPayload.hor_v,yu),Eih) + sca_complex_prod(dot(hitPayload.ver_v,yu),Eiv);
		float2 Eirad=sca_complex_prod(dot(hitPayload.hor_v,zu),Eih) + sca_complex_prod(dot(hitPayload.ver_v,zu),Eiv);

		Einorm=sca_complex_prod(fr,Einorm); 
		Eipar=sca_complex_prod(fr,Eipar); 
		Eirad=sca_complex_prod(fr,Eirad); 

		RDNHit h=frBuffer[bindex];
		
		//Add field
		h.EEx.z +=Einorm.x;
		h.EEx.w +=Einorm.y;
		h.EyEz.x += Eipar.x;
		h.EyEz.y += Eipar.y;
		h.EyEz.z += Eirad.x;
		h.EyEz.w += Eirad.y;
		//Update buffer		
		frBuffer[bindex]=h;

	}
	//Compute VOLTAGE
	if (computeMode==0) {

		//****************************
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
		E=sca_complex_prod(fr,E); 
		
		if (useAntennaGain) {
		
			float g=getAntennaGain(ray, gainBufferId,transformToPolarization);	
			E=sca_complex_prod(g,E);
		}

		//frBuffer[bindex]+=E;
		//float3 hh=make_float3(receiverBufferIndex, receiverLaunchIndex.x, receiverLaunchIndex.y);
		//frBuffer[bindex] =hh;
		//	rtPrintf("DH\t%u\t%u\t%u\t%u\t%f\t%f\t%d\n", receiverLaunchIndex.x, receiverLaunchIndex.y,receiverBufferIndex, reflections, E.x,E.y,externalId);
		RDNHit h=frBuffer[bindex];
		//Add field
		h.EEx.x +=E.x;
		h.EEx.y +=E.y;
		//Update buffer		
		frBuffer[bindex]=h;


	}






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

