/***************************************************************/
//
//Copyright (c) 2019 Esteban Egea-Lopez http://girtel.upct.es/~eegea
//
/**************************************************************/
#include "../../../Common.h"
#include <optix_world.h>
#include "../linearPolarizationFunctions.h"
#include "../../configuration.h"
#include "../../reflectionFunctions.h"
#include "../../receiverFunctions.h"
using namespace optix;

//Constants
#define D_2_Pi 6.283185307179586f
#define D_SQRT_2_Pi 2.506628274631f
#define D_SQRT_2ByPi 0.797884560802865

//Scene root
//rtDeclareVariable(rtObject, root, , ); //Already defined in trace functions
//Static meshes root
//rtDeclareVariable(rtObject, staticMeshesRoot, , );

//Edge buffer
rtBuffer<Edge, 1> edgeBuffer;

//TODO:These  should be redundant if we used Receiver buffers
//Receiver position buffer 
rtBuffer<float4, 1> receiverPositionsBuffer;
rtBuffer<rtBufferId<float,2>, 1> antennaGainIdBuffer;
typedef optix::Matrix<4,4> TransMat; 
rtBuffer<TransMat, 1> transformToPolarizationBuffer;

//Hit buffer
rtBuffer<RDNHit, 3> difBuffer; //Buffer to store all the hits

//Local variables
//rtDeclareVariable(float, k, , ); //wavenumber 2pi/lambda
rtDeclareVariable(uint, computeMode, ,);
rtDeclareVariable(uint, traceDiffraction, ,);

//Transmitter buffer
rtBuffer<Transmitter, 1> txBuffer;

//Launch variables
rtDeclareVariable(uint3, launchIndex, rtLaunchIndex, );

//Visibility ray payload
rtDeclareVariable(VisibilityPayload, rayPayload, rtPayload, );
rtDeclareVariable(uint, rayTypeIndex, , );

//For debug only
rtDeclareVariable(TriangleHit, ch_triangle_data, attribute triangle_hit_data, );
rtDeclareVariable(CurvedTriangleHit, curved_triangle_data, attribute curved_triangle_hit_data, );
rtDeclareVariable(optix::Ray, ray_hit, rtCurrentRay, );


rtBuffer<LogTraceHitInfo, 1> traceBufferDiffraction;
rtBuffer<uint, 1> traceAtomicIndexDiffraction; //Buffer to store the current log trace buffer index 
//Sign function
template <typename T> 
__forceinline__ __device__ int sgn(T val) {
	return (T(0) < val) - (val < T(0));
}; 

__forceinline__ __device__ float signedAngle(float3 from, float3 to, float3 axis) {
	//WARNING: assuming vectors passed are normalized
	const float a=acosf(dot(from,to));
	const float s=sgn(dot(axis,cross(from, to)));
	return (s*a);

}
__forceinline__ __device__ float3 closestPointOnEdge(const float3& origin, const float3& v, const float3& end, const float3& s, float& t) {
	float3 diff=s-end; //Vector from endpoint to tx or rx
	t = dot(v,diff);
	float3 point;
	if (t>=0) {
		point=end;
		t=1.0f;
	} else {
		diff=s-origin;
		t=dot(v,diff);
		if (t<=0) {
			point=origin;
			t=0.0f;
		} else {
			point=origin +t*v;
		}
	}	
	return point;

}
__forceinline__ __device__ bool computeDiffractionPoint(const float3& origin, const float3& destination, const Edge& e, float3& dp) {

	//The diffraction point is the point that makes the angle between source ray (beta_prime_o) and edge and destination ray (beta_o) and edge equal
	const float edgeLength=e.v.w;
	const float3 uv=make_float3(e.v.x,e.v.y,e.v.z); //Extract edge unit vector
	const float3 oP=make_float3(e.pn.x,e.pn.y,e.pn.z); //Extract edge origin point
	const float3 eP=oP+(uv*edgeLength); //End point of edge

	//Project source on edge
	float ts;
	float3 sp=closestPointOnEdge(oP,uv,eP, origin,ts);	
	//Project destination on edge
	float tt;
	float3 fp=closestPointOnEdge(oP,uv,eP, destination,tt);

	if (ts==0.0f || ts==1.0) {
		const float3 stoe=origin-oP; //Vector from oP to source	
		sp=oP+dot(stoe,uv)*uv; //Point sp
	}
	if (tt==0.0f || tt==1.0) {
		const float3 dtoe=destination-oP; //Vector from oP to destination
		fp=oP+dot(dtoe,uv)*uv; //Point fp
	}



	const float deltas=length(origin-sp);
	const float deltaf=length(destination-fp);
	const float q=deltas/(deltas+deltaf);

	dp=sp+q*(fp-sp);
	//rtPrintf("%u\t%u\t%u e=%u o=(%f,%f,%f) uv=(%f,%f,%f) p=(%f,%f,%f)\n",launchIndex.x,launchIndex.y,launchIndex.z,e.id,origin.x,origin.y,origin.z,uv.x,uv.y,uv.z,e.p.x,e.p.y,e.p.z);
	//rtPrintf("%u\t%u\t%u e=%u to=%f destination=(%f,%f,%f) \n",launchIndex.x,launchIndex.y,launchIndex.z,e.id,to, destination.x,destination.y,destination.z);


	if (ts>0.0f && ts <1.0f && tt>0.0f && tt<1.0f) {
		//Projections are on the edge
		//ASSUMPTION: diffraction on edges made of infinite planes, we do not consider tips of edges this implementation
		//If the diffraction point is at the start o beginning of the edge, do not consider point
		const float EPSILON=1.0e-5f;
		float3 c = dp-oP;//Bottom tip
		//rtPrintf("bottom e.id=%d dp=(%f,%f,%f) c=(%f,%f,%f) e.p=(%f,%f,%f)\n",e.id,dp.x,dp.y,dp.z,c.x,c.y,c.z,e.p.x,e.p.y,e.p.z);
		if (fabs(c.x)<EPSILON && fabs(c.y)<EPSILON && fabs(c.z)<EPSILON) {
			//rtPrintf("discarded  hit on bottom e.id=%d dp=(%f,%f,%f) c=(%6e,%6e,%6e) e.p=(%f,%f,%f)\n",e.id,dp.x,dp.y,dp.z,c.x,c.y,c.z,e.pn.x,e.pn.y,e.pn.z);
			return false;
		}
		c=dp -eP; //Up tip
		//rtPrintf("up e.id=%d dp=(%f,%f,%f) c=(%f,%f,%f) e.p=(%f,%f,%f)\n",e.id,dp.x,dp.y,dp.z,c.x,c.y,c.z,e.p.x,e.p.y,e.p.z);
		if (fabs(c.x)<EPSILON && fabs(c.y)<EPSILON && fabs(c.z)<EPSILON) {
			//rtPrintf("discarded hit on up e.id=%d dp=(%f,%f,%f) c=(%6e,%6e,%6e) e.p=(%f,%f,%f)\n",e.id,dp.x,dp.y,dp.z,c.x,c.y,c.z,e.pn.x,e.pn.y,e.pn.z);
			return false;
		}
		return true;
	} else {
		const float3 diff=dp-oP;
		const float d=length(diff);
		const float3 diffu=diff/d;
		float t=dot(uv,diffu);
		if (t<0) {
			return false; //dp is outside the edge

		} else {
			if (d<edgeLength) {
				return true;
			} else {
				return false;
			}
		}
	}


}

//__forceinline__ __device__ bool computeDiffractionPoint(const float3& origin, const float3 destination, const Edge& e, float3& dp) {
//
//	//The diffraction point is the point that minimizes the distance between transmitter and receiver passing through the edge
//	//A simple way to compute it is to get the closest point from tx and rx to edge and then apply the formula below 
//	// (can be checked by looking at a figure and noticing that the angles from tx and rx to closest point on edge are square or  
//	// simply by minimizing the distance function L(q) = sqrt(d1^2+q^2) +sqrt(d2^2+(l-q)^2 where q is the fraction of the segment between closest points 
//	//at DP and l the length of the segment, d1 and d2 distance to tx and rx from edge
//
//
//
//	//Distance from point to edge (point to line,  constrained by end points)
//
//	const float edgeLength=e.v.w;
//	const float3 uv=make_float3(e.v.x,e.v.y,e.v.z); //Extract edge unit vector
//	const float3 eP=make_float3(e.pn.x,e.pn.y,e.pn.z); //Extract edge origin point
//	//Origin to edge
//	float to=dot(uv,origin-eP);
//	//rtPrintf("%u\t%u\t%u e=%u o=(%f,%f,%f) uv=(%f,%f,%f) p=(%f,%f,%f)\n",launchIndex.x,launchIndex.y,launchIndex.z,e.id,origin.x,origin.y,origin.z,uv.x,uv.y,uv.z,e.p.x,e.p.y,e.p.z);
//	//rtPrintf("%u\t%u\t%u e=%u to=%f destination=(%f,%f,%f) \n",launchIndex.x,launchIndex.y,launchIndex.z,e.id,to, destination.x,destination.y,destination.z);
//
//	if (to<0) {
//		to=0;
//	} else  if (to>edgeLength) {
//		to=edgeLength;
//	}	
//	//rtPrintf("%u\t%u\t%u e=%u to=%f \n",launchIndex.x,launchIndex.y,launchIndex.z,e.id,to);
//	const float3 oe=eP+(to*uv);	
//	//receiver to edge
//	to=dot(uv,destination-eP);
//	//rtPrintf("%u\t%u\t%u e=%u to=%f dot=%f \n",launchIndex.x,launchIndex.y,launchIndex.z,e.id,to,dot(uv,destination-e.p));
//	if (to<0) {
//		to=0;
//	} else  if (to>edgeLength) {
//		to=edgeLength;
//	}	
//	const float3 de=eP+(to*uv);	
//	//const float3 dif=de-oe;
//	//rtPrintf("%u\t%u\t%u e=%u oe=(%f,%f,%f) de=(%f,%f,%f) dif=(%f,%f,%f)\n",launchIndex.x,launchIndex.y,launchIndex.z,e.id,oe.x,oe.y,oe.z,de.x,de.y,de.z,dif.x,dif.y,dif.z);
//	const float d_oe=length(oe-origin);
//	const float d_de=length(de-destination);
//	//rtPrintf("%u\t%u\t%u e=%u dif=(%f,%f,%f) d_oe=%f d_de=%f\n",launchIndex.x,launchIndex.y,launchIndex.z,e.id,dif.x,dif.y,dif.z,d_oe,d_de);
//
//	//ASSUMPTION: diffraction on edges made of infinite planes, we do not consider tips of edges this implementation
//	//If the diffraction point is at the start o beginning of the edge, do not consider point
//	dp=oe +(de-oe)*(d_oe/(d_oe+d_de));
//	const float EPSILON=1.0e-5f;
//	float3 c = dp-eP;//Bottom tip
//	//rtPrintf("bottom e.id=%d dp=(%f,%f,%f) c=(%f,%f,%f) e.p=(%f,%f,%f)\n",e.id,dp.x,dp.y,dp.z,c.x,c.y,c.z,e.p.x,e.p.y,e.p.z);
//	if (fabs(c.x)<EPSILON && fabs(c.y)<EPSILON && fabs(c.z)<EPSILON) {
//		//rtPrintf("discarded  hit on bottom e.id=%d dp=(%f,%f,%f) c=(%6e,%6e,%6e) e.p=(%f,%f,%f)\n",e.id,dp.x,dp.y,dp.z,c.x,c.y,c.z,e.pn.x,e.pn.y,e.pn.z);
//		return false;
//	}
//	c=dp -(eP+uv*edgeLength); //Up tip
//	//rtPrintf("up e.id=%d dp=(%f,%f,%f) c=(%f,%f,%f) e.p=(%f,%f,%f)\n",e.id,dp.x,dp.y,dp.z,c.x,c.y,c.z,e.p.x,e.p.y,e.p.z);
//	if (fabs(c.x)<EPSILON && fabs(c.y)<EPSILON && fabs(c.z)<EPSILON) {
//		//rtPrintf("discarded hit on up e.id=%d dp=(%f,%f,%f) c=(%6e,%6e,%6e) e.p=(%f,%f,%f)\n",e.id,dp.x,dp.y,dp.z,c.x,c.y,c.z,e.pn.x,e.pn.y,e.pn.z);
//		return false;
//	}
//	return true;
//
//
//}

//Decide if the edge can be a diffracting edge according to some orientation. It is diffracting if the angle of the edge (from face to face) is > 180 as seen from the side  of origin
__forceinline__ __device__ bool isDiffractingEdge(const float3& origin,  const Edge& e) {
	//Use the normal of face a (arbitray) to decide on which side of the edge we are
	const float EPSILON=1.0e-8f;

	if (abs(e.pn.w)<EPSILON || abs(e.pn.w-1.0f)<EPSILON) {
		//n=1, Full plane: assume no diffraction at this edge
		//n=2 we have a valid half plane (both faces are together). However, for n=0, no diffraction
		return false;
	}
	const float3 eP=make_float3(e.pn.x,e.pn.y,e.pn.z); //Extract edge origin point
	const float proja=dot(origin-eP, e.n_a);
	const float projb=dot(origin-eP, e.n_b);
	if (proja==0 || projb==0) {
		//The origin is on the plane of one of the faces...trouble
		return false;

	} 
	//We just check the internal angle as stored in the edge
	//If both projections are negative, we are 'inside' the edge
	if (proja<0 && projb<0) {
		if ((2.0-e.pn.w)>1.0) { //(2.0-n)*PI>PI
			//Internal angle is greater than 180  
			return true;
		} else {
			return false;
		}
	} else {
		//The origin is on the side of the face planes that the normal is pointing at
		if ((2.0-e.pn.w)<1.0) { //(2.0-n)*PI<PI
			//Internal angle is less than 180 so, from our point of view (external) it is larger than 180
			return true;
		} else {
			return false;
		}
	} 

}
__forceinline__ __device__ float2 computeViB(float2 zexp, float2 Ds, float2 Dh, float s)
{
	const float2 Di=0.5f*(Ds+Dh);
	const float at=1.0f/sqrtf(s);
	float2 ViB=sca_complex_prod(at,complex_prod(zexp,Di));
	//RDNHit aHit;
	//aHit.EEx=make_float4(ViB.x,ViB.y, 0.0f,0.0f);
	//aHit.EyEz=make_float4(0.0f,0.0f,0.0f,0.0f);
	//difBuffer[index]=aHit;

	return ViB;
}
__forceinline__ __device__ float2 computeVrB(float2 zexp, float2 Ds, float2 Dh, float s)
{
	const float2 Dr=-0.5f*(Ds-Dh);
	const float at=1.0f/sqrtf(s);
	float2 VrB=sca_complex_prod(at,complex_prod(zexp,Dr));
	//RDNHit aHit;
	//aHit.EEx=make_float4(VrB.x,VrB.y, 0.0f,0.0f);
	//aHit.EyEz=make_float4(0.0f,0.0f,0.0f,0.0f);
	//difBuffer[index]=aHit;

	return VrB;
}


//__forceinline__ __device__ optix::float2 getDiffractionAngles(const float3 v, const float3& face_0, const float3& face_n, const float3& n_face_0, const float3& n_dplane, const float3& n_iplane) {
//	//float3 rot=cross(face_0,face_n);
//	float3 rot=face_0;
//	float phi=acosf(dot(n_face_0,n_dplane));
//	float phi_p=acosf(dot(n_face_0,n_iplane));
//	float3 cphi=cross(n_face_0,n_dplane);
//	float3 cphi_p=cross(n_face_0,n_iplane);
//	//if (dot(rot, cphi)<0) {
//	//	phi=D_2_Pi-phi;
//	//}
//	//if (dot(rot, cphi_p)<0) {
//	//	phi_p=D_2_Pi-phi_p;
//	//}
//	rtPrintf("%u\t%u\t rot=(%f,%f,%f) cphi=(%f,%f,%f) cphi_p=(%f,%f,%f) phi=%f phi_p=%f   \n",launchIndex.y,launchIndex.x, rot.x,rot.y,rot.z,cphi.x,cphi.y,cphi.z, cphi_p.x,cphi_p.y,cphi_p.z,(phi*180/M_PIf), (phi_p*180/M_PIf));
//	rtPrintf("%u\t%u\t rot=(%f,%f,%f) face_0=(%f,%f,%f)  n_face_0=(%f,%f,%f)   \n",launchIndex.y,launchIndex.x, rot.x,rot.y,rot.z,face_0.x ,face_0.y, face_0.z, n_face_0.x,n_face_0.y,n_face_0.z);
//	
//	return make_float2(phi, phi_p);
//
//
//}
//__forceinline__ __device__ optix::float2 getDiffractionAngles(const float3 v, const float3& face_0, const float3& face_n, const float3& n_face_0, const float3& s_u, const float3& s_p_u) {
//	//float3 rot=cross(face_0,face_n);
//	//Project on face_0/face_n plane
//	//float3 rot=v;
//	float3 plane=normalize(cross(face_0, face_n));
//	float3 rot=plane;
//	float3 sp;
//	float3 spp;
//	if (dot(plane,s_u)==0) {
//		sp=s_u;
//	} else {	
//		sp=normalize(s_u-dot(plane,s_u)*plane);
//	}
//	if (dot(plane,s_p_u)==0) {
//		spp=s_p_u;
//	} else {
//		spp=normalize(-s_p_u -dot(plane,-s_p_u)*plane);
//	}
//	float phi=acosf(dot(face_0,sp));
//	float phi_p=acosf(dot(face_0,spp));
//	float3 cphi=cross(face_0,sp);
//	float3 cphi_p=cross(face_0,spp);
//	if (dot(rot, cphi)<0) {
//		phi=D_2_Pi-phi;
//	}
//	if (dot(rot, cphi_p)<0) {
//		phi_p=D_2_Pi-phi_p;
//	}
//	rtPrintf("%u\t%u\t  plane=(%f,%f,%f)  s_u=(%f,%f,%f) s_p_u=(%f,%f,%f)  \n",launchIndex.y,launchIndex.x,plane.x ,plane.y, plane.z, s_u.x,s_u.y,s_u.z, s_p_u.x,s_p_u.y,s_p_u.z);
//	rtPrintf("%u\t%u\t rot=(%f,%f,%f) cphi=(%f,%f,%f) cphi_p=(%f,%f,%f) phi=%f phi_p=%f   \n",launchIndex.y,launchIndex.x, rot.x,rot.y,rot.z,cphi.x,cphi.y,cphi.z, cphi_p.x,cphi_p.y,cphi_p.z,(phi*180/M_PIf), (phi_p*180/M_PIf));
//	rtPrintf("%u\t%u\t  face_0=(%f,%f,%f)  sp=(%f,%f,%f) spp=(%f,%f,%f)  \n",launchIndex.y,launchIndex.x,face_0.x ,face_0.y, face_0.z, sp.x,sp.y,sp.z, spp.x,spp.y,spp.z);
//	
//	return make_float2(phi, phi_p);
//
//
//}

//__forceinline__ __device__ optix::float2 getDiffractionAngles(const float3 v, const float3& face_0, const float3& face_n, const float3& n_face_0, const float3& s_u, const float3& s_p_u) {
//	//float3 rot=cross(face_0,face_n);
//	//Project on face_0/face_n plane
//	//float3 rot=v;
//
//	float3 plane=normalize(cross(face_0, face_n));
//
//	float3 rot=plane;
//
//
//	float3 sp;
//	float3 spp;
//	if (dot(plane,s_u)==0) {
//		sp=s_u;
//	} else {	
//		sp=normalize(s_u-dot(plane,s_u)*plane);
//	}
//	if (dot(plane,s_p_u)==0) {
//		spp=-s_p_u;
//	} else {
//		spp=normalize(-s_p_u -dot(plane,-s_p_u)*plane);
//	}
//	float phi=atan2(dot(cross(sp,face_0),plane),dot(face_0,sp));
//	float phi_p=atan2(dot(cross( spp, face_0),plane),dot(face_0,spp));
//	//If this is negative, add 360 degrees
//	rtPrintf("%u\t%u\t phi=%f phi_p=%f efaces=%6e  \n",launchIndex.y,launchIndex.x,(phi*180/M_PIf), (phi_p*180/M_PIf), (efaces*180/M_PIf));
//	if (phi<0) {
//			phi=D_2_Pi+phi;
//	}
//	//If this is negative, add 360 degrees
//	if (phi_p<0) {
//			phi_p=D_2_Pi+phi_p;
//	}
//	rtPrintf("%u\t%u\t  plane=(%f,%f,%f)  s_u=(%f,%f,%f) s_p_u=(%f,%f,%f)  \n",launchIndex.y,launchIndex.x,plane.x ,plane.y, plane.z, s_u.x,s_u.y,s_u.z, s_p_u.x,s_p_u.y,s_p_u.z);
//	rtPrintf("%u\t%u\t  face_0=(%f,%f,%f)  sp=(%f,%f,%f) spp=(%f,%f,%f)  \n",launchIndex.y,launchIndex.x,face_0.x ,face_0.y, face_0.z, sp.x,sp.y,sp.z, spp.x,spp.y,spp.z);
//	
//	return make_float2(phi, phi_p);
//
//
//}
__forceinline__ __device__ optix::float2 getDiffractionAngles(const float3 v, const float3& face_0, const float3& face_n, const float3& n_face_0, const float3& s_u, const float3& s_p_u) {


	//Get signed angle form face_0

	//Plane of faces, to be used in projection, This should work even if the edge is not perpendicular to the plane of the faces
	float3 plane=normalize(cross(n_face_0, face_0));
	//Rotation axis is given by plane normal. With this convention, rotation on the internal angle should be positive
	//We invert the vector to make rotations on the external angle positive 
	float3 rot=-plane;
	//Project s and s_prime vectors on the face plane
	float3 sp;
	float3 spp;
	if (dot(plane,s_u)==0) { //Already on plane
		sp=s_u;
	} else {	
		sp=normalize(s_u-dot(plane,s_u)*plane);
	}
	if (dot(plane,s_p_u)==0) { //Already on plane
		spp=-s_p_u;
	} else {
		spp=normalize(-s_p_u -dot(plane,-s_p_u)*plane);
	}
	float phi=signedAngle(face_0, sp,rot);
	float phi_p=signedAngle(face_0, spp,rot);
	//rtPrintf("%u\t%u\t phi=%f phi_p=%f rot=(%f,%f,%f) \n",launchIndex.y,launchIndex.x,(phi*180/M_PIf),(phi_p*180/M_PIf), rot.x,rot.y,rot.z);
	//If this is negative, add 360 degrees
	if (phi<0) {
		phi=D_2_Pi+phi;
	}
	//If this is negative, add 360 degrees
	if (phi_p<0) {
		phi_p=D_2_Pi+phi_p;
	}
	//rtPrintf("%u\t%u\t  plane=(%f,%f,%f)  s_u=(%f,%f,%f) s_p_u=(%f,%f,%f)  \n",launchIndex.y,launchIndex.x,plane.x ,plane.y, plane.z, s_u.x,s_u.y,s_u.z, s_p_u.x,s_p_u.y,s_p_u.z);
	//rtPrintf("%u\t%u\t  face_0=(%f,%f,%f)  sp=(%f,%f,%f) spp=(%f,%f,%f)  \n",launchIndex.y,launchIndex.x,face_0.x ,face_0.y, face_0.z, sp.x,sp.y,sp.z, spp.x,spp.y,spp.z);

	return make_float2(phi, phi_p);


}

//__forceinline__ __device__ optix::float2 getDiffractionAngles(const float3 v, const float3& face_0, const float3& n_dplane, const float3& n_iplane) {
////v is edge, 
//	//Project s_prime on the plane to make sure the normal is "pointing" to the tx
//	//float sign_vtx=dot(vv,-s_prime_u);
//	//if (sign_vtx<0) {
//	//	vv=-vv;
//	//}
//	//if (launchIndex.x==6) {
//	rtPrintf("%u\t%u\t d=(%f,%f,%f) face_0=(%f,%f,%f) i=(%f,%f,%f)    \n",launchIndex.y,launchIndex.x, n_dplane.x,n_dplane.y,n_dplane.z,face_0.x,face_0.y,face_0.z, n_iplane.x,n_iplane.y,n_iplane.z);
//	//}
//	//if (abs(e.pn.w-2.0)<EPSILON ) {
//	//	//Half plane, full plane ..use the normal to the face
//	//	//Both faces are colinear.
//	//	//Use the edge as normal vector here...
//	//	//TODO:But, what if the edge is straight but not perpendicular to the faces (like an arrow edge..). Just compute, for now the perpendicular vector to get the plane
//	//	//Then compute the normal to the plane spanned by faces and edge: nf
//	//	//And use as vector the normal to the plane normal and the face (that is the normal vector of the plane spanned by nf and a): vv=cross(nf,a)
//	//	const float3 nf=normalize(cross(a,v));
//	//	vv=normalize(cross(nf,a));
//	//	//vv=v;
//	//} else {
//	//	//vv=normalize(cross(a,b));
//	//	vv=normalize(cross(v,face_0));
//	//}
//
//		//float3 psp = -s_prime_u-dot(-s_prime_u,vv)*vv; //Vector  projection on the plane defined by the edge unit vector (edge unit vector is the normal vector of that plane)
//		//float3 ps = s_u-dot(s_u,vv)*vv; //Vector projection on the plane defined by the edge unit vector (edge unit vector is the normal vector of that plane)
//
//		////rtPrintf("%u\t%u\t%u e=%u vv=(%f,%f,%f) ps=(%f,%f,%f) psp=(%f,%f,%f) n=%f   \n",launchIndex.x,launchIndex.y,launchIndex.z,e.id, vv.x,vv.y,vv.z,ps.x,ps.y,ps.z, psp.x,psp.y,psp.z,e.pn.w);
//		////rtPrintf("vv=(%f,%f,%f) ps=(%f,%f,%f) psp=(%f,%f,%f) n=%f   \n", vv.x,vv.y,vv.z,ps.x,ps.y,ps.z, psp.x,psp.y,psp.z,e.pn.w);
//		////Get the signed angle from the face to the vector projected. Get the absolute values, what is important is that they are measured from the same face in the same direction
//		////angles.z=abs(atan2(dot(cross(a,ps),-vv),dot(a,ps)));//Signed angle between the face and the diffraction plane (phi)
//		////angles.w=abs(atan2(dot(cross(a,-psp),-vv),dot(a,-psp))); //Signed angle between the face and the incident plane (phi') 
//
//		////const float3 a=e.n_a;
//		////angles.z=acosf(dot(dplane,e.n_a)); 
//		////angles.w=acosf(dot(-n_iplane,e.n_a)); 
//		//float phi=acosf(dot(face_0,ps));
//		////float phi=atan2();
//		//float3 nc=cross(face_0,ps);
//		////rtPrintf("nc=(%f,%f,%f) psp=(%f,%f,%f) phi=%f   \n", nc.x,nc.y,nc.z,psp.x,psp.y,psp.z,(phi*180/M_PIf));
//		//if (dot(nc,vv)<0) {
//		//	phi=D_2_Pi-phi;
//		//}
//		//float phi_p=acosf(dot(face_0,psp));
//		//nc=cross(face_0,psp);
//		////rtPrintf("nc=(%f,%f,%f) psp=(%f,%f,%f) phi_p=%f   \n", nc.x,nc.y,nc.z,psp.x,psp.y,psp.z,phi_p);
//		//if (dot(nc,vv)<0) {
//		//	phi_p=D_2_Pi-phi_p;
//		//	//rtPrintf("nci2=(%f,%f,%f) psp=(%f,%f,%f) phi_p2=%f  D_2=%f \n", nc.x,nc.y,nc.z,psp.x,psp.y,psp.z,phi_p,D_2_Pi);
//		//}
//	//float phi=acosf(dot(vv,-n_dplane));
//	
//	//Compute the angles as the angle between the incident/diffraction plane and the plane spanned by face 0 and edge
//	
//	//Normal to the plane spanned by face_0 and edge
//	//float3 vv=normalize(cross(v,face_0));
//	//float3 vv=n_face_0;
//	
//	//Vector normal to both vectors to get the signed angle	
//	//float3 nn_p=normalize(cross(vv,n_dplane));
//	//float3 nn_p=v;
//	//Ideally we could use the edge as the normal vector to face and angle. But the edge need not be normal to the faces....
//	//Let us use the edge as reference, this should typically be 0 when the edge is actually normal
////	if (dot(nn_p, v) <0) {
////		nn_p=-nn_p;
////	}
//	float phi_a=atan2(dot(cross(vv,n_dplane),nn_p),dot(vv,n_dplane));
//	float phi=atan2(dot(cross(vv,n_dplane),nn_p),dot(vv,n_dplane));
//	//If this is negative, add 360 degrees
//	//if (phi_a<0) {
//	//		phi_a=D_2_Pi+phi_a;
//	//}
//	//if (launchIndex.x==6) {
//	rtPrintf("%u\t nn_p=(%f,%f,%f) n_face_0=(%f,%f,%f)  phi=%f  phi_a=%f \n", launchIndex.x,nn_p.x,nn_p.y,nn_p.z,n_face_0.x,n_face_0.y,n_face_0.z,(phi*180/M_PIf),(phi_a*180/M_PIf));
//	//}
////		if (dot(v,nn_p)<0) {
////			phi=D_2_Pi-phi;
////		//if (launchIndex.x==6) {
//	//			rtPrintf("nci2=(%f,%f,%f)  phi=%f  phi_a=%f \n", nn_p.x,nn_p.y,nn_p.z,(phi*180/M_PIf),(phi_a*180/M_PIf));
////		//	}
////		}
//	//float phi_p=acosf(dot(vv,n_iplane));
//	//nn_p=normalize(cross(vv,-n_iplane));
//
////	if (dot(nn_p, v) <0) {
////		nn_p=-nn_p;
////	}
//	float phi_a_p=atan2(dot(cross(vv,-n_iplane),nn_p),dot(vv,-n_iplane));
//	float phi_p=atan2(dot(cross(vv,-n_iplane),nn_p),dot(vv,-n_iplane));
//	//if (phi_a_p<0) {
//	//		phi_a_p=D_2_Pi+phi_a_p;
//	//}
//	//if (launchIndex.x==6) {
//		rtPrintf("%u\t nn_p2=(%f,%f,%f)  phi_p=%f  phi_a_p=%f \n",launchIndex.x, nn_p.x,nn_p.y,nn_p.z,(phi_p*180/M_PIf),(phi_a_p*180/M_PIf));
//	//}
////		if (dot(v,nn_p)<0) {
////			phi_p=D_2_Pi-phi_p;
////			//if (launchIndex.x==6) {
////			//	rtPrintf("nn_p_phi_p=(%f,%f,%f)  phi=%f  phi_a_p=%f \n", nn_p.x,nn_p.y,nn_p.z,(phi_p*180/M_PIf),(phi_a_p*180/M_PIf));
////			//}
////		}
//	//if (launchIndex.x==6) {
//	//	rtPrintf("%u\t%u\t vv=(%f,%f,%f) d=(%f,%f,%f) i=(%f,%f,%f) phi=%f   \n",launchIndex.y,launchIndex.x, vv.x,vv.y,vv.z,n_dplane.x,n_dplane.y,n_dplane.z, n_iplane.x,n_iplane.y,n_iplane.z,(phi*180/M_PIf));
//	//}
//	if ((phi_a<0) && (phi_a_p<0)) {
//		phi_a=-phi_a;
//		phi_a_p=-phi_a_p;
//	} else {
//		if ((phi_a>0) && (phi_a_p<0)) {
//			phi_a_p=D_2_Pi+phi_a_p;
//		} else if ((phi_a<0) && (phi_a_p>0)) {
//			phi_a=D_2_Pi+phi_a;
//		}
//	}
//	return make_float2(phi_a, phi_a_p);
//}
// use the algorithm for distance to triangle in 10.3.3 from Schneider and Eberly, "Geometric tools for computer graphics", 2003

__forceinline__ __device__ float squareDistanceToRectangle(const float3& origin, const float4& face_a, const float4& face_b, const float3& point) {
	//Define rectangle 
	const float3 e0=make_float3(face_a)*face_a.w;
	const float3 e1=make_float3(face_b)*face_b.w;
	float3 d = point-origin;
	const float s = dot(e0,d);
	if (s>0) {
		float dot0=face_a.w*face_a.w;
		if (s<dot0) {
			d=d-(s/dot0)*e0;
		} else {
			d=d-e0;
		}	
	}
	const float t = dot(e1,d);
	if (t>0) {
		float dot1=face_b.w*face_b.w;
		if (t<dot1) {
			d = d - (t/dot1)*e1;
		} else {
			d = d-e1;
		}
	}
	return dot(d,d);

} 
//Returns the parameters  required to compute diffraction as defined,see e.g. Balanis, Advanced Engineering Electromagnetics. Returns angles in float4 [beta,beta_prime,phi, phi_prime],  distances in float2 [s, s_prime],  
//normals to the incidence and diffraction planes and face_0 and face_n vectors
template<class P>
__forceinline__ __device__ optix::float4 getDiffractionParameters(P& rayPayload, const float3& tx, const float3& rx,  const Edge& e, const float3& dp, float2& distances, float3& n_iplane, float3& n_dplane,  float4& R_0, float4& R_n) {

	const float EPSILON=1.0e-8f;
	float4 angles;
	const float3 eP=make_float3(e.pn.x,e.pn.y,e.pn.z); //Extract edge origin point
	const float3 v=make_float3(e.v.x,e.v.y,e.v.z);	//Extract edge vector
	const float3 s_prime=dp-tx; //Vector from tx to dp
	const float3 s=rx-dp; //vector from  dp to rx
	distances.x=length(s);
	distances.y=length(s_prime);
	const float3 s_prime_u=normalize(s_prime);
	const float3 s_u=normalize(s);
	//if (launchIndex.x==6) {
	//}
	angles.x=acosf(dot(s_u,v)); //beta
	angles.y=acosf(dot(-s_prime_u,-v)); //beta_ prime TODO: actually, later we only need sinf(beta_prime), not the angle, change

	n_iplane = normalize(cross(s_prime_u,v)); //Incidence plane normal vector
	n_dplane = normalize(cross(s_u,v)); //Diffraction plane 
	//For better readability
	float3 face_0;
	float3 face_n;
	float3 n_face_0;
	float3 n_face_n;
	const float3 a=make_float3(e.a.x,e.a.y,e.a.z);
	const float3 b=make_float3(e.b.x,e.b.y,e.b.z); //Extract face vector
	//Find the closest face to the tx
	float d_toa=squareDistanceToRectangle(eP,e.v,e.a,tx);
	float d_tob=squareDistanceToRectangle(eP,e.v,e.b,tx);
	//rtPrintf("%u\t%u\t s_prime_u=(%f,%f,%f) |s_prime|=%f s_u=(%f,%f,%f) dp=(%f,%f,%f)   \n",launchIndex.y,launchIndex.x, s_prime_u.x,s_prime_u.y,s_prime_u.z,distances.y,s_u.x,s_u.y,s_u.z, dp.x,dp.y,dp.z);
	//rtPrintf("%u\t%u\t s_prime_u=(%f,%f,%f) |s_prime|=%f s_u=(%f,%f,%f)  d_toa=%f, d_tob=%f  \n",launchIndex.y,launchIndex.x, s_prime_u.x,s_prime_u.y,s_prime_u.z,distances.y,s_u.x,s_u.y,s_u.z, d_toa,d_tob);
	if (d_toa<=d_tob) {
		face_0=a;
		n_face_0=e.n_a;
		face_n=b;
		n_face_n = e.n_b;
	} else {
		face_0=b;
		n_face_0=e.n_b;
		face_n=a;
		n_face_n = e.n_a;
	}

	//TODO: This is obviously not correct. We are using the distance to the planes. As an alternative, use the algorithm for distance to triangle in 10.3.3 from Schneider and Eberly, "Geometric tools for computer graphics", 2003
	//float d_tx_a=abs(dot(s_prime,e.n_a)); //Distance from tx to plane formed by face a
	//float d_tx_b=abs(dot(s_prime,e.n_b));
	////rtPrintf("%u\t%u\t%u e=%u v=(%f,%f,%f) dif=(%f,%f,%f) d_tx_a=%f d_tx_b=%f deltadis=%f   \n",launchIndex.x,launchIndex.y,launchIndex.z,e.id, v.x,v.y,v.z,n_dplane.x,n_dplane.y,n_dplane.z, d_tx_a, d_tx_b,fabs(d_tx_a-d_tx_b) );
	////rtPrintf("%u\t%u\t%u e=%u v=(%f,%f,%f) dif=(%f,%f,%f) inc=(%f,%f,%f) \n",launchIndex.x,launchIndex.y,launchIndex.z,e.id, v.x,v.y,v.z,n_dplane.x,n_dplane.y,n_dplane.z, n_iplane.x,n_iplane.y,n_iplane.z );
	////const float3 a=make_float3(e.a.x,e.a.y,e.a.z);
	////const float3 b=make_float3(e.b.x,e.b.y,e.b.z); //Extract face vector
	//if (fabs(d_tx_a-d_tx_b)<EPSILON) {
	//	//Tx is essentially at the same distance from both planes
	//	//I am not sure about the generality of this, but let us do it anyway
	//	float3 txtoa=(eP+a)-tx; //Vector from tx to end of  face vector a
	//	float3 txtob=(eP+b)-tx; //Vector from tx to end of  face vector b
	//	float dtoa = dot(txtoa, txtoa);
	//	float dtob = dot(txtob, txtob);
	//	//Now, both distances can also be equal.... In that case I guess it does not matter which face we choose
	//	if (dtoa<=dtob) {
	//		//rtPrintf("%u\t%u\t%u e=%u same use A dtoa=%6e dtob=%6e \n",launchIndex.x,launchIndex.y,launchIndex.z,e.id, dtoa, dtob );

	//		face_0=a;
	//		n_face_0=e.n_a;
	//		face_n=b;
	//		n_face_n = e.n_b;
	//	} else {
	//		//rtPrintf("%u\t%u\t%u e=%u same use B dtoa=%6e dtob=%6e \n",launchIndex.x,launchIndex.y,launchIndex.z,e.id, dtoa, dtob );
	//		face_0=b;
	//		n_face_0=e.n_b;
	//		face_n=a;
	//		n_face_n = e.n_a;
	//	}
	//	//rtPrintf("%u\t%u\t%u e=%u dtoa=%f dtob=%f   \n",launchIndex.x,launchIndex.y,launchIndex.z,e.id, dtoa, dtob);
	//}  else {
	//	if (d_tx_a<=d_tx_b) {
	//		//rtPrintf("%u\t%u\t%u e=%u Not same, use A dtoa=%6e dtob=%6e \n",launchIndex.x,launchIndex.y,launchIndex.z,e.id, d_tx_a, d_tx_b );
	//		//Use face a to compute the angles
	//		face_0=a;
	//		n_face_0=e.n_a;
	//		face_n=b;
	//		n_face_n = e.n_b;
	//	} else {
	//		//rtPrintf("%u\t%u\t%u e=%u Not same, use B dtoa=%6e dtob=%6e \n",launchIndex.x,launchIndex.y,launchIndex.z,e.id, d_tx_a, d_tx_b );
	//		face_0=b;
	//		n_face_0=e.n_b;
	//		face_n=a;
	//		n_face_n = e.n_a;
	//	}
	//}
	//rtPrintf("%u\t%u\t%u d_toa=%f d_tob=%f dtoa=%f dtob=%f\n",launchIndex.x,launchIndex.y,launchIndex.z,d_toa,d_tob,d_tx_a,d_tx_b );
	////angles.z=phi_a; 
	////angles.w=phi_a_p; 
	////float2 dangles = getDiffractionAngles(v, face_0, face_n, n_face_0, n_dplane, n_iplane);
	float2 dangles = getDiffractionAngles(v, face_0, face_n, n_face_0, s_u, s_prime_u);
	//Separate for readability
	const float phi=dangles.x;
	const float phi_p=dangles.y;
	angles.z=phi; 
	angles.w=phi_p; 
	float cosA_zero;	
	float cosA_n;
	//Now we can compute the reflection coefficients for the faces
	if (phi_p<=phi) {
		cosA_zero = sinf(phi_p); //In the reflection coefficient function we use the complementary angle (90-incidence) and pass directly the cosine of that complementary angle;	
		cosA_n = sinf(e.pn.w*M_PIf-phi); //In the reflection coefficient function we use the complementary angle (90-incidence) and pass directly the cosine of that complementary angle;	
	} else {
		cosA_zero = sinf(phi); //In the reflection coefficient function we use the complementary angle (90-incidence) and pass directly the cosine of that complementary angle;	
		cosA_n = sinf(e.pn.w*M_PIf-phi_p); //In the reflection coefficient function we use the complementary angle (90-incidence) and pass directly the cosine of that complementary angle;	
	}
	R_0 = getReflectionCoefficient<VisibilityPayload>(rayPayload,cosA_zero, e.mat);
	R_n = getReflectionCoefficient<VisibilityPayload>(rayPayload,cosA_n, e.mat);
	return angles;

	//if (d_tx_a<=d_tx_b) {
	//	//Use face a to compute the angles
	//	const float3 a=make_float3(e.a.x,e.a.y,e.a.z);
	//	const float3 b=make_float3(e.b.x,e.b.y,e.b.z); //Extract face vector
	//	float3 vv;
	//	const float EPSILON=1.0e-8f;
	//	if (abs(e.pn.w-2.0)<EPSILON ) {
	//		//Half plane, full plane ..use the normal to the face
	//		//Both faces are colinear.
	//		//Use the edge as normal vector here...
	//		//TODO:But, what if the edge is straight but not perpendicular to the faces (like an arrow edge..). Just compute, for now the perpendicular vector to get the plane
	//		//Then compute the normal to the plane spanned by faces and edge: nf
	//		//And use as vector the normal to the plane normal and the face (that is the normal vector of the plane spanned by nf and a): vv=cross(nf,a)
	//		 const float3 nf=normalize(cross(a,v));
	//		 vv=normalize(cross(nf,a));
	//		//vv=v;
	//	} else {
	//		vv=normalize(cross(a,b));
	//	}

	//	float3 psp = -s_prime_u-dot(-s_prime_u,vv)*vv; //Vector  projection on the plane defined by the edge unit vector (edge unit vector is the normal vector of that plane)
	//	float3 ps = s_u-dot(s_u,vv)*vv; //Vector projection on the plane defined by the edge unit vector (edge unit vector is the normal vector of that plane)
	//	
	//	//rtPrintf("%u\t%u\t%u e=%u vv=(%f,%f,%f) ps=(%f,%f,%f) psp=(%f,%f,%f) n=%f   \n",launchIndex.x,launchIndex.y,launchIndex.z,e.id, vv.x,vv.y,vv.z,ps.x,ps.y,ps.z, psp.x,psp.y,psp.z,e.pn.w);
	//	rtPrintf("vv=(%f,%f,%f) ps=(%f,%f,%f) psp=(%f,%f,%f) n=%f   \n", vv.x,vv.y,vv.z,ps.x,ps.y,ps.z, psp.x,psp.y,psp.z,e.pn.w);
	//	//Get the signed angle from the face to the vector projected. Get the absolute values, what is important is that they are measured from the same face in the same direction
	//	//angles.z=abs(atan2(dot(cross(a,ps),-vv),dot(a,ps)));//Signed angle between the face and the diffraction plane (phi)
	//	//angles.w=abs(atan2(dot(cross(a,-psp),-vv),dot(a,-psp))); //Signed angle between the face and the incident plane (phi') 
	//	
	//	//const float3 a=e.n_a;
	//	//angles.z=acosf(dot(dplane,e.n_a)); 
	//	//angles.w=acosf(dot(-n_iplane,e.n_a)); 
	//	float phi=acosf(dot(a,ps));
	//	float3 nc=cross(a,ps);
	//	if (dot(nc,vv)<0) {
	//		phi=D_2_Pi-phi;
	//	}
	//	float phi_p=acosf(dot(a,psp));
	//	nc=cross(a,psp);
	//	if (dot(nc,vv)<0) {
	//		phi_p=D_2_Pi-phi;
	//	}
	//	angles.z=phi; 
	//	angles.w=phi_p; 
	//	
	//	
	//	//angles.z=acosf(dot(s_u,a)); //Angle between the face and the receiver (phi)
	//	//angles.w=acosf(dot(-s_prime_u,a)); //Angle between the face and the transmitter (phi') 
	//	rtPrintf("%u\t%u\t%u e=%u a=(%f,%f,%f) v=(%f,%f,%f) phi=%f phi_p=%f   \n",launchIndex.x,launchIndex.y,launchIndex.z,e.id, a.x,a.y,a.z,v.x,v.y,v.z, (phi*180/M_PIf), (phi_p*180/M_PIf));

	//} else {
	//	//Use face b to compute the angles
	//	const float3 a=make_float3(e.a.x,e.a.y,e.a.z);
	//	const float3 b=make_float3(e.b.x,e.b.y,e.b.z); //Extract face vector
	//	float3 vv=normalize(cross(a,b));
	//	
	//	float3 psp = s_prime_u-dot(s_prime_u,vv)*vv; //Vector  projection on the plane defined by the edge unit vector (edge unit vector is the normal vector of that plane)
	//	float3 ps = s_u-dot(s_u,vv)*vv; //Vector projection on the plane defined by the edge unit vector (edge unit vector is the normal vector of that plane)
	//	
	//	//Get the siged angle from the face to the vector projected
	//	angles.z=abs(atan2(dot(cross(b,ps),vv),dot(b,ps)));
	//	angles.w=abs(atan2(dot(cross(b,-psp),vv),dot(b,-psp)));
	//	//const float3 b=e.n_b;
	//	//angles.z=acosf(dot(dplane,e.n_b)); //Angle between the face and the receiver
	//	//angles.w=acosf(dot(-n_iplane,e.n_b)); //Angle between the face and the transmitter 
	//	//angles.z=acosf(dot(s_u,b)); //Angle between the face and the receiver
	//	//angles.w=acosf(dot(-s_prime_u,b)); //Angle between the face and the transmitter 
	//	rtPrintf("%u\t%u\t%u e=%u b=(%f,%f,%f) v=(%f,%f,%f) \n",launchIndex.x,launchIndex.y,launchIndex.z,e.id, b.x,b.y,b.z,v.x,v.y ,v.z);
	//}
	//	return angles;

}

//Compute the g+ and g- functions for each pair or sums of angles [g+(phi+phi'),g+(phi-phi'),g-(phi+phi'),g-(phi-phi')]. Eq. 13.66c,d from Balanis
__forceinline__ __device__ float4 computeG(float  phi_plus, float phi_minus, float n) {
	float N_plus_plus=rintf((phi_plus+M_PIf)/(D_2_Pi*n));
	float N_plus_minus=rintf((phi_minus+M_PIf)/(D_2_Pi*n));
	float N_minus_plus=rintf((phi_plus-M_PIf)/(D_2_Pi*n));
	float N_minus_minus=rintf((phi_minus-M_PIf)/(D_2_Pi*n));
	float g_p_p=1+cosf(phi_plus-D_2_Pi*n*N_plus_plus);//g+(phi+phi')
	float g_p_m=1+cosf(phi_minus-D_2_Pi*n*N_plus_minus);//g+(phi-phi')
	float g_m_p=1+cosf(phi_plus-D_2_Pi*n*N_minus_plus); //g-(phi+phi')
	float g_m_m=1+cosf(phi_minus-D_2_Pi*n*N_minus_minus);//g-(phi-phi')
	//rtPrintf("%u\t n=%f phi_plus=%f phi_minus=%f \n",launchIndex.x,n, (phi_plus*180/M_PIf),(phi_minus*180/M_PIf));
	//rtPrintf("%u\t  N=(%f,%f,%f,%f) g=(%f,%f,%f,%f)\n",launchIndex.x,N_plus_plus,N_plus_minus,N_minus_plus,N_minus_minus,g_p_p,g_p_m,g_m_p,g_m_m);
	return make_float4(g_p_p,g_p_m,g_m_p,g_m_m);


}


//Compute Fresnel sin and cos integrals [fs,fc]
__forceinline__ __device__ float2 fresnel(float  X ) {
	if (X==0) {
		return make_float2(0.0f,0.0f);
	}
	float fs;
	float fc;
	float x=abs(X);
	if (x<=1.8) { //Cut point for switching between power series and rational approximation
		float s = M_PI_2f*(x*x);
		float ss = s*s;
		fs=x*s*(1.0f/3.0f+(-.23809523809523809524e-1+(.75757575757575757576e-3+(-.13227513227513227513e-4+(.14503852223150468764e-6+(-.10892221037148573380e-8+(.59477940136376350368e-11+(-.24668270102644569277e-13+(.80327350124157736091e-16+(-.21078551914421358249e-18+(.45518467589282002862e-21+(-.82301492992142213568e-24+(.12641078988989163522e-26+(-.16697617934173720270e-29+.19169428621097825308e-32*ss)*ss)*ss)*ss)*ss)*ss)*ss)*ss)*ss)*ss)*ss)*ss)*ss)*ss);

		fc=x*(1+(-0.1f+(.46296296296296296296e-2+(-.10683760683760683761e-3+(.14589169000933706816e-5+(-.13122532963802805073e-7+(.83507027951472395917e-10+(-.39554295164585257634e-12+(.14483264643598137265e-14+(-.42214072888070882330e-17+(.10025164934907719167e-19+(-.19770647538779051748e-22+(.32892603491757517328e-25+(-.46784835155184857737e-28+.57541916439821717722e-31*ss)*ss)*ss)*ss)*ss)*ss)*ss)*ss)*ss)*ss)*ss)*ss)*ss)*ss);


	} else {
		float is = D_SQRT_2ByPi/x;
		float ff=D_SQRT_2ByPi*is*(.36634901025764670680+(.84695666222194589182+(-3.7301273487349839902+(10.520237918558456228+(-10.472617402497801126+4.4169634834911107719*is)*is)*is)*is)*is)/(.73269802661202980832+(1.6939102288852613619+(-7.4599994789665215344+(21.032436583848862358+(-20.269535575486282590+8.9995024877628789836*is)*is)*is)*is)*is);

		float fg=D_SQRT_2ByPi*(is*is*is)*(.10935692320079194659+(.72025533055541994934+(-2.5630322339412334317+(7.2404268469720856874+(-7.0473933910697823445+2.9315207450903789503*is)*is)*is)*is)*is)/(.43742772185339366619+(2.8810049959848098312+(-10.250672312139277005+(28.912791022417600679+(-25.740131167525284201+15.145134363709932380*is)*is)*is)*is)*is);
		float aux=M_PI_2f*(x*x);
		float cx=cosf(aux);
		float sx=sinf(aux);
		fs = 0.5f-ff*cx -fg*sx;
		fc = 0.5f+ff*sx -fg*cx;

	}
	if (X<0) {
		fs=-1.0f*fs;
		fc=-1.0f*fc;
	}	
	return make_float2(fs,fc);


}
//TODO: try the Balanis approximations (13-74a), p. 785
//Compute the Fresnel transition function
__forceinline__ __device__ float2 fresnelTransition(float  X ) {
	//First constant jsqrt(2*pi*X)exp(jX) 
	float sqx=sqrtf(X);
	float sq=sqx*D_SQRT_2_Pi;
	//float sq=sqx*sqrtf(2*M_PIf);
	float2 K=make_float2(-1.0f*sq*sinf(X),sq*cosf(X)); //Complex number
	float2 fr=fresnel(sqx*D_SQRT_2ByPi);
	float2 aux=make_float2(0.5f-fr.y,-1.0f*(0.5f-fr.x));
	float2 faux=complex_prod(K,aux);
	//rtPrintf("%u\t K=(%6e,%6e) fr=(%6e,%6e) aux=(%6e,%6e) F=(%6e,%6e)\n",launchIndex.x,K.x,K.y,fr.x,fr.y,aux.x,aux.y,faux.x,faux.y);
	return complex_prod(K,aux);

}
//Returns the Luebbers difraction coefficients (2 complex numbers) [D_s,D_h]
__forceinline__ __device__ float4 computeLuebbersDiffractionCoefficient(float k, float n, float phi, float phi_prime, float beta_o_prime, float L , float4& R_0, float4& R_n) {

	//TODO: check and apply Keller simplification

	float2 D_i=make_float2(0.0f,0.0f);
	float2 D_ip=make_float2(0.0f,0.0f);
	float2 D_im=make_float2(0.0f,0.0f);
	float2 D_r=make_float2(0.0f,0.0f);
	float2 D_rp=make_float2(0.0f,0.0f);
	float2 D_rm=make_float2(0.0f,0.0f);

	const float phi_plus=phi+phi_prime;
	const float phi_minus=phi-phi_prime;
	float4 gf=computeG(phi_plus,phi_minus,n);
	//Separate for readability
	const float g_p_p=gf.x;
	const float g_p_m=gf.y;
	const float g_m_p=gf.z;
	const float g_m_m=gf.w;
	float phi_n=(M_PIf+phi_minus)/(2*n);
	float2 FT;
	float cot;
	FT=fresnelTransition(k*L*g_p_m);
	//float2 FT=fresnelTransition(k*L*gf.y);
	//rtPrintf("%u\t  k=%f L=%f g=%f arg=%f phi_n=%f FT=(%6e,%6e) \n",launchIndex.x,k,L,gf.y,k*L*gf.y,(phi_n*180/M_PIf),FT.x,FT.y);
	//float cot=1.0f/tanf(phi_n);
	cot=1.0f/tanf(phi_n);
	D_ip=sca_complex_prod(cot,FT);
	//rtPrintf("%u\t pi+phi+  cot=%f phi_n=%f FT=(%6e,%6e) D_ip(%f,%f) \n",launchIndex.x,cot,(phi_n*180/M_PIf),FT.x,FT.y, D_ip.x,D_ip.y);
	phi_n=(M_PIf-phi_minus)/(2*n);
	FT=fresnelTransition(k*L*g_m_m);
	//float2 FT=fresnelTransition(k*L*gf.w);
	//rtPrintf("%u\t k=%f L=%f g=%f arg=%f phi_n=%f FT=(%6e,%6e) \n",launchIndex.x,k,L,gf.w,k*L*gf.w,(phi_n*180/M_PIf),FT.x,FT.y);
	cot=1.0f/tanf(phi_n);
	//float cot=1.0f/tanf(phi_n);
	D_im=sca_complex_prod(cot,FT);
	//rtPrintf("%u\t pi+phi+  cot=%f phi_n=%f FT=(%6e,%6e) D_im(%f,%f) \n",launchIndex.x,cot,(phi_n*180/M_PIf),FT.x,FT.y, D_im.x,D_im.y);


	D_i=D_ip+D_im;
	//rtPrintf("%u\t D_ip=(%6e,%6e) D_im=(%6e,%6e) D_i=(%6e,%6e)  n=%f \n",launchIndex.x,D_ip.x,D_ip.y, D_im.x,D_im.y,D_i.x,D_i.y,n);

	phi_n=(M_PIf+phi_plus)/(2*n);
	FT=fresnelTransition(k*L*g_p_p);
	//float2 FT=fresnelTransition(k*L*gf.x);
	//rtPrintf("%u\t k=%f L=%f g=%f arg=%f phi_n=%f FT=(%6e,%6e) \n",launchIndex.x,k,L,gf.x,k*L*gf.x,phi_n,FT.x,FT.y);
	cot=1.0f/tanf(phi_n);
	//float cot=1.0f/tanf(phi_n);
	D_rp=sca_complex_prod(cot,FT);
	//rtPrintf("%u\t pi+phi+  cot=%f phi_n=%f FT=(%6e,%6e) D_rp(%f,%f) \n",launchIndex.x,cot,(phi_n*180/M_PIf),FT.x,FT.y, D_rp.x,D_rp.y);

	phi_n=(M_PIf-phi_plus)/(2*n);
	FT=fresnelTransition(k*L*g_m_p);
	//float2 FT=fresnelTransition(k*L*gf.z);
	//rtPrintf("%u\t k=%f L=%f g=%f arg=%f phi_n=%f FT=(%6e,%6e) \n",launchIndex.x,k,L,gf.z,k*L*gf.z,phi_n,FT.x,FT.y);
	//float cot=1.0f/tanf(phi_n);
	cot=1.0f/tanf(phi_n);
	//rtPrintf("%u\t pi-phi+  cot=%f phi_n=%f FT=(%6e,%6e) \n",launchIndex.x,cot,(phi_n*180/M_PIf),FT.x,FT.y);
	D_rm=sca_complex_prod(cot,FT);
	//rtPrintf("%u\t pi+phi+  cot=%f phi_n=%f FT=(%6e,%6e) D_rm(%f,%f) \n",launchIndex.x,cot,(phi_n*180/M_PIf),FT.x,FT.y, D_rm.x,D_rm.y);

	D_r=D_rp+D_rm;

	//rtPrintf("%u\t D_rp=(%6e,%6e) D_rm=(%6e,%6e) D_r=(%6e,%6e)  n=%f \n",launchIndex.x,D_rp.x,D_rp.y, D_rm.x,D_rm.y,D_r.x,D_r.y,n);
	//Complex constant that multiplies both terms
	//First, declare the -exp(-j*pi/4) (complex)
	const float2 Exp_pi=make_float2(-0.7071067811865476,0.7071067811865475); //Already multiplied by -1
	const float den=1.0f/(2.0f*n*D_SQRT_2_Pi*sqrtf(k)*sinf(beta_o_prime));
	const float2 EL=sca_complex_prod(den,Exp_pi); //Complex number

	//Separate coefficients
	const float2  Rs0=make_float2(R_0.x,R_0.y); //Rsoft=Rnorm
	const float2  Rh0=make_float2(R_0.z,R_0.w); //Rhard=Rpar
	const float2  Rsn=make_float2(R_n.x,R_n.y);
	const float2  Rhn=make_float2(R_n.z,R_n.w);
	float2 DRs0=complex_prod(D_rm,Rs0);
	float2 DRsn=complex_prod(D_rp,Rsn);
	float2 DRh0=complex_prod(D_rm,Rh0);
	float2 DRhn=complex_prod(D_rp,Rhn);
	float2 D_s=complex_prod(EL,D_i+DRs0+DRsn);	;//Coefficient for the parallel (soft) component of the incident field
	float2 D_h=complex_prod(EL,D_i+DRh0+DRhn);	;//Coefficient for the perpendicular (hard) component of the incident field

	//For perfect conductor below
	//float2 D_s=complex_prod(EL,D_i-D_r);	;//Coefficient for the parallel (soft) component of the incident field
	//float2 D_h=complex_prod(EL,D_i+D_r);	;//Coefficient for the perpendicular (hard) component of the incident field



	//rtPrintf("%u \t EL=(%6e,%6e) D_s=(%6e,%6e) Dh=(%6e,%6e)\n",launchIndex.x,EL.x,EL.y,D_s.x,D_s.y,D_h.x,D_h.y);
	//Grazing incidence
	if ((phi_prime==0)  || abs((n*M_PIf)-phi_prime)<1e-8) {
		D_s=make_float2(0.0f,0.0f);
		D_h=0.5f*D_h;
		//rtPrintf("Grazing incidence");
	}
	//TODO: around ISB and RSB coefficients are discontinuous, we may apply corrections there...
	return make_float4(D_s,D_h);

}
__forceinline__ __device__ void computeElectricFieldAtReceiver(const float4& angles, const float2& spar, const float3& txRay, const float4& polarization_k, const float3& n_iplane, const float3& n_dplane, float n, float3& rxRay, float4& R_0, float4& R_n , const float gain ) {
	uint3 index=launchIndex;
	const float k = polarization_k.w;
	const float3 pol=make_float3(polarization_k);
	const float s=spar.x;
	const float s_prime=spar.y;
	//Parameter A for spherical waves, see Balanis TODO: add support for other waves in the future
	const float A=sqrtf(s_prime/(s*(s_prime+s))); 
	const float sinbetap=sinf(angles.y);
	//L parameter for spherical and conical incidence, see Balanis. TODO: add support for other waves in the future 
	const float L=(s*s_prime*sinbetap*sinbetap)/(s_prime+s);
	//Get the electrical field vector for this ray
	const float3 Ev=getLinearPolarizationForRaySimple(pol,txRay); //It is normalized

	//rtPrintf("%u\tray=(%f,%f,%f) pol=(%f,%f,%f) Ev=(%f,%f,%f) A=%f L=%f)\n",launchIndex.y,txRay.x,txRay.y,txRay.z,pol.x,pol.y,pol.z,Ev.x,Ev.y,Ev.z,A,L);
	//rtPrintf("%u\tray=(%f,%f,%f)  Ev=(%f,%f,%f) A=%f L=%f s=%f,s'=%f)\n",launchIndex.x,txRay.x,txRay.y,txRay.z,Ev.x,Ev.y,Ev.z,A,L,s,s_prime);
	//rtPrintf("rt=(%f,%f,%f) pol=(%f,%f,%f) Ev2=(%f,%f,%f) Evsimple2=(%f,%f,%f)\n",rt.x,rt.y,rt.z,pol.x,pol.y,pol.z,Ev3.x,Ev3.y,Ev3.z,Ev4.x,Ev4.y,Ev4.z);

	//Get the unit vector parallel to the plane of incidence
	const float3 phi_p = n_iplane;
	const float3 beta_p=normalize(cross(txRay,phi_p)); 
	const float3 ss=normalize(cross(phi_p,beta_p)); 
	//rtPrintf("%u\t beta_p=(%f,%f,%f) phi_p=(%f,%f,%f) s_p=(%f,%f,%f) \n",launchIndex.x,beta_p.x,beta_p.y,beta_p.z,phi_p.x,phi_p.y,phi_p.z,ss.x,ss.y,ss.z);

	//Compute incident electric field at the point of diffraction (complex)	
	float2 z = make_float2(0.0f, -k*s_prime);
	float2 zexp = complex_exp_only_imaginary(z);
	const float2 Ei = sca_complex_prod((gain/s_prime), zexp); //TODO: Assuming the initial amplitude is 1.0. To Change if antenna radiation patter or similar is used 

	//Geometric parts
	const float Ei_beta=dot(beta_p,Ev); //component of the incident E field parallel to the plane of incidence at the point of diffraction
	const float Ei_phi=dot(phi_p,Ev); //component of the incident E field perpendicular to the plane of incidence at the point of diffraction



	//Incident complex amplitude components
	float2 Ei_beta_q=sca_complex_prod(Ei_beta,Ei);
	float2 Ei_phi_q=sca_complex_prod(Ei_phi,Ei);
	//rtPrintf("%u\t Ev=(%f,%f,%f) Ei_beta=%f Ei_phi=%f Ei_beta_q=(%f,%f) Ei_phi_q=(%f,%f) an=%f\n",launchIndex.y,Ev.x,Ev.y,Ev.z,Ei_beta,Ei_phi, Ei_beta_q.x,Ei_beta_q.y, Ei_phi_q.x,Ei_phi_q.y,(atan2(Ei_phi_q.y,Ei_phi_q.x)*180/M_PIf));


	//Diffraction coefficients
	float4 D=computeLuebbersDiffractionCoefficient(k,n, angles.z,angles.w,angles.y,L, R_0, R_n);


	//Test: store diffraction coefficient
	//		RDNHit aHit;
	//		aHit.EEx=make_float4(0.0f,0.0f,0.0f,0.0f);
	//		aHit.EyEz=D;
	//		difBuffer[index]=aHit;
	//		return;
	//Split for better readability
	const float2 Ds=make_float2(D.x,D.y);
	const float2 Dh=make_float2(D.z,D.w);


	//Attenuation term at receiver due to propagation 
	z = make_float2(0.0f, -k*s);
	zexp = complex_exp_only_imaginary(z);
	float2 E_at_r = sca_complex_prod(-1.0f*A, zexp);

	//Some tests	
	//float2 Vbi=computeViB(zexp,Ds,Dh,s); 
	//float2 Vbr=computeVrB(zexp,Ds,Dh,s); 
	//float2 Vbir=Vbi+Vbr;
	//float2 Dhb=sca_complex_prod(-1.0*sqrtf(s),complex_prod(zexp,Vbir));
	//
	//	RDNHit aHit;
	//	aHit.EEx=make_float4(Vbi.x,Vbi.y, 0.0f,0.0f);
	//	aHit.EyEz=make_float4(0.0f,0.0f,0.0f,0.0f);
	//	difBuffer[index]=aHit;
	//return;
	//Complex amplitude of the diffracted E field at the receiver. Eq. [13-88] Balanis
	float2 Er_beta=complex_prod(E_at_r,complex_prod(Ds,Ei_beta_q)); //component of the diffracted E field parallel to the plane of diffraction at the receiver
	float2 Er_phi=complex_prod(E_at_r,complex_prod(Dh,Ei_phi_q)); //component of the diffracted E field perpendicular to the plane of diffraction at the  receiver
	//rtPrintf("%u \t Er_beta=(%6e,%6e) |Er_beta|=%6e  Er_phi=(%6e,%6e) |Er_phi|=%6e Ds=(%6e,%6e) Dh=(%6e,%6e))\n",launchIndex.y,Er_beta.x,Er_beta.y,length(Er_beta),Er_phi.x,Er_phi.y,length(Er_phi),D.x,D.y,D.z,D.w);


	//float2 Ee=Er_beta+Er_phi;

	//rtPrintf("%u \t Dhb=(%6e,%6e) Dh=(%6e,%6e)) angle(Vi+Vr)=%f angle(Er_phi)=%f a(E)=%f\n",launchIndex.y,Dhb.x,Dhb.y,Dh.x,Dh.y, (atan2(Vbir.y,Vbir.x)*180/M_PIf),(atan2(-Er_phi.y,-Er_phi.x)*180/M_PIf),(atan2(-Ee.y,-Ee.x)*180/M_PIf));

	//Get the unit vectors for the plane of diffraction. The above complex amplitude multiply the corresponding (beta and phi) unit vectors in the diffraction plane
	const float3 phi_u = n_dplane; 
	//s_unit vector is defined from DP to receiver, so we have to reverse ray here
	const float3 beta_o_u=normalize(cross(-rxRay,phi_u)); 
	//const float3 sss = normalize(cross(phi_u,beta_o_u));
	//rtPrintf("%u\t beta_o_u=(%f,%f,%f) phi_u=(%f,%f,%f) s=(%f,%f,%f) \n",launchIndex.x,beta_o_u.x,beta_o_u.y,beta_o_u.z,phi_u.x,phi_u.y,phi_u.z,sss.x,sss.y,sss.z);

	//Compute FIELD
	if (computeMode==1) {
		float3 xu=make_float3(1.0f,0.0f,0.0f);
		float3 yu=make_float3(0.0f,1.0f,0.0f);
		float3 zu=make_float3(0.0f,0.0f,1.0f);
		const float2 Ex=sca_complex_prod(dot(beta_o_u,xu),Er_beta) + sca_complex_prod(dot(phi_u,xu),Er_phi);
		const float2 Ey=sca_complex_prod(dot(beta_o_u,yu),Er_beta) + sca_complex_prod(dot(phi_u,yu),Er_phi);
		const float2 Ez=sca_complex_prod(dot(beta_o_u,zu),Er_beta) + sca_complex_prod(dot(phi_u,zu),Er_phi);
		//rtPrintf("%u\t Ex=(%f,%f) |Ex|=%f Ey=(%f,%f) |Ey|=%f Ez=(%f,%f) |Ez|=%f \n",launchIndex.x,Ex.x, Ex.y,length(Ex),Ey.x,Ey.y,length(Ey),Ez.x,Ez.y,length(Ez));
		float4 sangles=angles*57.2968f;
		//rtPrintf("%u\t%u\t%u  sangles(beta, beta',phi, phi')=(%f,%f,%f,%f) dif=%6e \n",launchIndex.x,launchIndex.y,launchIndex.z, sangles.x,sangles.y,sangles.z,sangles.w, (sangles.x-sangles.y));
		RDNHit aHit;

		//aHit.EEx=make_float4(0.0f,0.0f,Ex.x,Ex.y);
		aHit.EEx=make_float4(1.0f,0.0f,Ex.x,Ex.y); //Use 1 on EEx.x as flag for real hit
		aHit.EyEz=make_float4(Ey.x,Ey.y,Ez.x,Ez.y);
		difBuffer[index]=aHit;
	}
	//Compute VOLTAGE
	if (computeMode==0) {

		//****************************
		//To get the induced voltage, we need to 
		//apply the dot product with the effective lenght of the received antenna. 

		//float3 ver_o; //Receiver vertical field vector
		//float3 hor_o; //Receiver horizontal field vector

		//Get polarization for receiver for this ray rxRay is already in the direction receiver to DP 
		//getLinearPolarizationInRayBasis(pol, rxRay,  hor_o,ver_o);

		//Get the  components of received field for the normal and parallel field vectors (geometric projection on receiver polarization vectors times reflection coefficients)
		//This would be equivalent to a dot product with the effective length (not the conjugated beacuse we already reversed and it is a linear polarization anyway)
		//const float2 Einorm=sca_complex_prod(dot(beta_o_u,hor_o),Er_beta) + sca_complex_prod(dot(phi_u,hor_o),Er_phi);
		//const float2 Eipar=sca_complex_prod(dot(beta_o_u,ver_o),Er_beta) + sca_complex_prod(dot(phi_u,ver_o),Er_phi);
		//const float2 Einorm=sca_complex_prod(dot(beta_o_u,beta_p),Er_beta) + sca_complex_prod(dot(phi_u,beta_p),Er_phi);
		//const float2 Eipar=sca_complex_prod(dot(beta_o_u,phi_p),Er_beta) + sca_complex_prod(dot(phi_u,phi_p),Er_phi);
		//float2 E=Einorm+Eipar;

		//The above formulation is equivalent to this below 
		//Geometric part due to polarization at the receiver
		//Get polarization for receiver for this ray rxRay is already in the direction receiver to DP 
		const float3 Er_pol=getLinearPolarizationForRaySimple(pol,rxRay); //It is normalized

		//rtPrintf("%u\trxRay=(%f,%f,%f) Er_pol=(%f,%f,%f) Ev=(%f,%f,%f) A=%f L=%f)\n",launchIndex.y,rxRay.x,rxRay.y,rxRay.z,Er_pol.x,Er_pol.y,Er_pol.z,Ev.x,Ev.y,Ev.z,A,L);

		const float Er_beta_v=dot(beta_o_u,Er_pol); 
		const float Er_phi_v=dot(phi_u,Er_pol); 
		Er_beta=sca_complex_prod(Er_beta_v,Er_beta);
		Er_phi=sca_complex_prod(Er_phi_v,Er_phi);
		//This is actually the induced voltage on the antenna. From it we can compute the received power
		float2 E=Er_beta+Er_phi;
		if (useAntennaGain) {

			float g=getAntennaGain(rxRay, antennaGainIdBuffer[index.y],transformToPolarizationBuffer[index.y]);	
			E=sca_complex_prod(g,E);
		}

		//rtPrintf("G\t |E|=%6e index=(%u,%u,%u) %f \n",length(E),  index.x,index.y,index.z,angles.z*57.2968f);

		//float4 sangles=angles*57.2968f;
		//rtPrintf("%u\t%u\t%u  sangles(beta, beta',phi, phi')=(%f,%f,%f,%f) L=%f dif=%6e  \n",launchIndex.x,launchIndex.y,launchIndex.z, sangles.x,sangles.y,sangles.z,sangles.w, L, (sangles.x-sangles.y));
		RDNHit aHit;
		//aHit.EEx=make_float4(E.x,E.y, 0.0f,0.0f);
		aHit.EEx=make_float4(E.x,E.y, 1.0f,0.0f); //Use 1 on EEx.z as flag that this has been an actual hit
		aHit.EyEz=make_float4(0.0f,0.0f,0.0f,0.0f);
		//Additional output
		float unfoldedPathLength = s+s_prime;
		aHit.doaD = make_float4(rxRay.x, rxRay.y,rxRay.z, unfoldedPathLength);
		aHit.doDu = make_float4(txRay.x, txRay.y,txRay.z, -1.0);
	
		difBuffer[index]=aHit;
		
	}
}


//Diffraction Launch program
RT_PROGRAM void computeSingleDiffraction() {

	//3D launch [edges,receivers,transmitters]
	//rtPrintf("%u\t%u\t%u Launch \n",launchIndex.x,launchIndex.y,launchIndex.z);
	uint3 difBufferIndex=launchIndex;
	//Initialize buffer to make sure it does not carry values from previous launches
	RDNHit aHit;
	aHit.EEx=make_float4(0.0f,0.0f, 0.0f,0.0f);
	aHit.EyEz=make_float4(0.0f,0.0f,0.0f,0.0f);
	difBuffer[difBufferIndex]=aHit;

	Transmitter tx = txBuffer[launchIndex.z];

	const float3 origin = make_float3(tx.origin_p);
	const float4 sphere = receiverPositionsBuffer[launchIndex.y];
	//Check if ray is hitting his own tx (transmitter are also receivers usually) A transmitter cannot receive while it is transmitting, unless other channel is used.
	if (static_cast<int>(sphere.w)==tx.externalId) {
		return;	
	}	
	const float3 destination =make_float3(sphere.x,sphere.y,sphere.z);
	Edge e = edgeBuffer[launchIndex.x];
	if (!isDiffractingEdge(origin,e)) {
		//rtPrintf("%u\t%u\t%u Not diffracting edge %d\n",launchIndex.x,launchIndex.y,launchIndex.z,e.id);
		return;
	}

	//Compute diffraction point (DP) between transmitter, receiver and edge
	float3 dp; 
	if (computeDiffractionPoint(origin,destination,e,dp)){
		//rtPrintf("%u\t%u\t%u e=%u dp=(%f,%f,%f) \n",launchIndex.x,launchIndex.y,launchIndex.z,e.id,dp.x,dp.y,dp.z);
		VisibilityPayload visibilityRayPayload;
		visibilityRayPayload.polarization_k = tx.polarization_k; 
		visibilityRayPayload.result.x=OPAL_DIFFRACTION_LOS;
		visibilityRayPayload.faces=e.faces; //Faces of the edge, where the ray can hit to compute the diffraction. If it hits any other face, there is no LoS
		visibilityRayPayload.result.y=0;
		////trace visibility from transmitter to DP
		float3 originToDP=dp-origin;
		float dist_originToDp=length(originToDP);
		float3 txRayDirection = originToDP/dist_originToDp;
		//optix::Ray visibilityRay(origin,txRayDirection , rayTypeIndex, 0.0f,dist_originToDp); //Visibility ray type = 1
		optix::Ray visibilityRay(origin,txRayDirection , rayTypeIndex, min_t_epsilon,dist_originToDp-min_t_epsilon); //Visibility ray type = 1
		//rtPrintf("%u\t%u\t%u e=%u dp=(%f,%f,%f) tx ray=(%f,%f,%f) \n",launchIndex.x,launchIndex.y,launchIndex.z,e.id, dp.x,dp.y,dp.z,visibilityRay.direction.x,visibilityRay.direction.y,visibilityRay.direction.z);
		//TODO: Only check visibility with static meshes so far. Change if we want to consider  moving meshes (such as vehicles)
		//WARNING: ONLY THIS METHODS WORKS. ANY OF THE ONE BELOW GIVES WRONG RESULTS, IT MAY BE A OPTIX BU
		rtTrace(root, visibilityRay, visibilityRayPayload,OPAL_STATIC_MESH_MASK,RT_RAY_FLAG_DISABLE_ANYHIT);

		//rtTrace(root, visibilityRay, visibilityRayPayload,RT_VISIBILITY_ALL);
		//rtTrace(root, visibilityRay, visibilityRayPayload,RT_VISIBILITY_ALL,RT_RAY_FLAG_DISABLE_ANYHIT);
		//rtTrace(root, visibilityRay, visibilityRayPayload);
		if (visibilityRayPayload.result.x!=OPAL_DIFFRACTION_BLOCKED) {
			//rtPrintf("%u\t%u\t%u e=%u dp=(%f,%f,%f) tx not blocked \n",launchIndex.x,launchIndex.y,launchIndex.z,e.id, dp.x,dp.y,dp.z);
			//trace visibility from receiver to DP
			float3 destinationToDP=dp-destination;
			//rtPrintf("%u\t%u\t%u e=%u destinationToDP=(%f,%f,%f) tx not blocked \n",launchIndex.x,launchIndex.y,launchIndex.z,e.id, destinationToDP.x,destinationToDP.y,destinationToDP.z);
			float dist_destinationToDp=length(destinationToDP);
			visibilityRay.origin=destination;
			visibilityRay.direction=destinationToDP/dist_destinationToDp;
			visibilityRay.tmin=0.0f;
			visibilityRay.tmax=dist_destinationToDp; 
			float3 rxRayDir=destinationToDP/dist_destinationToDp;
			//optix::Ray visibilityRayRx(destination, rxRayDir , rayTypeIndex, 0.0f,dist_destinationToDp); //Visibility ray type = 1
			optix::Ray visibilityRayRx(destination, rxRayDir , rayTypeIndex, min_t_epsilon,dist_destinationToDp-min_t_epsilon); //Visibility ray type = 1

			VisibilityPayload visibilityRayPayloadRx;
			visibilityRayPayloadRx.polarization_k = tx.polarization_k; 
			visibilityRayPayloadRx.result.x=OPAL_DIFFRACTION_LOS;
			visibilityRayPayloadRx.faces=e.faces; //Faces of the edge, where the ray can hit to compute the diffraction. If it hits any other face, there is no LoS
			visibilityRayPayloadRx.result.y=1;
			//visibilityRayPayload.result.x=OPAL_DIFFRACTION_LOS;
			//visibilityRayPayload.result.y=1;
			//rtPrintf("%u\t%u\t%u e=%u dp=(%f,%f,%f) rx ray=(%f,%f,%f) d=%f \n",launchIndex.x,launchIndex.y,launchIndex.z,e.id, dp.x,dp.y,dp.z,visibilityRay.direction.x,visibilityRay.direction.y,visibilityRay.direction.z, dist_destinationToDp	);
			float3 rxRay=visibilityRay.direction;
			//rtPrintf("%u\t%u\t%u e=%u rxRay=(%f,%f,%f) rx not blocked \n",launchIndex.x,launchIndex.y,launchIndex.z,e.id, rxRay.x,rxRay.y,rxRay.z);
			//rtPrintf("%u\t%u\t%u e=%u diff sangles(beta, beta',phi, phi')=(%f,%f,%f,%f) spar(s,s')=(%f,%f) face_0=(%f,%f,%f)  \n",launchIndex.x,launchIndex.y,launchIndex.z,e.id, sangles.x,sangles.y,sangles.z,sangles.w,spar.x,spar.y, face_0.x,face_0.y,face_0.z);

			rtTrace(root, visibilityRayRx, visibilityRayPayloadRx,OPAL_STATIC_MESH_MASK,RT_RAY_FLAG_DISABLE_ANYHIT);
			//rtTrace(root, visibilityRayRx, visibilityRayPayload,RT_VISIBILITY_ALL,RT_RAY_FLAG_DISABLE_ANYHIT);
			//rtTrace(root, visibilityRay, visibilityRayPayload,RT_VISIBILITY_ALL);
			//rtTrace(root, visibilityRayRx, visibilityRayPayloadRx);
			if (visibilityRayPayloadRx.result.x!=OPAL_DIFFRACTION_BLOCKED) {
				//LoS, so compute diffraction
				float2 spar; //Distance parameters [s,s']
				float3 n_iplane; //Normal vector of the incidence plane
				float3 n_dplane; //Normal vector of the diffraction plane
				float4 R_0; 
				float4 R_n;
				float4 angles=getDiffractionParameters<VisibilityPayload>(visibilityRayPayload,origin, destination,e,dp,spar, n_iplane, n_dplane, R_0, R_n);
				float4 sangles = angles*180.0f/M_PIf;
				//rtPrintf("%u\t%u\t%u e=%u  sangles(beta, beta',phi, phi')=(%f,%f,%f,%f) spar(s,s')=(%f,%f)   \n",launchIndex.x,(launchIndex.y),launchIndex.z,e.id, sangles.x,sangles.y,sangles.z,sangles.w,spar.x, spar.y);
				if (angles.z>=(M_PIf*e.pn.w)) {
					//Receiver is between face_0 and face_n (inside the edge). It cannot receive even if there is no blocking (actually blocking may not be detected by visibility)
					//rtPrintf("%u\t%u\t%u e=%u receiver is inside the wedge  sangles(beta, beta',phi, phi')=(%f,%f,%f,%f) spar(s,s')=(%f,%f)   \n",launchIndex.x,launchIndex.y,launchIndex.z,e.id, sangles.x,sangles.y,sangles.z,sangles.w,spar.x);
					return;
				}
				//rtPrintf("%u\t%u\t%u e=%u dp=(%f,%f,%f) rx not blocked \n",launchIndex.x,launchIndex.y,launchIndex.z,e.id, dp.x,dp.y,dp.z);
				float gain;
				if (useAntennaGain) {
					//rtPrintf("tx useAntennaGain\n");
					rtBufferId<float,2> bid=tx.gainId;
					const Matrix<4,4> tp=tx.transformToPolarization;
					gain=getAntennaGain(txRayDirection, bid, tp);	
				} else {
					gain=1.0f;
				}
				computeElectricFieldAtReceiver(angles, spar, txRayDirection, tx.polarization_k,n_iplane,n_dplane,e.pn.w, visibilityRay.direction, R_0, R_n, gain);
				if (traceDiffraction==1) {
					LogTraceHitInfo trace;
					trace.hitp=make_float4(dp);
					trace.cdata = make_uint4(launchIndex.y,launchIndex.z, launchIndex.x,0);
					uint traceIndex=atomicAdd(&traceAtomicIndexDiffraction[0u],1u);
					traceBufferDiffraction[traceIndex]=trace;
					trace.hitp=make_float4(destination);
					trace.cdata = make_uint4(launchIndex.y,launchIndex.z, launchIndex.x,1);
					traceIndex=atomicAdd(&traceAtomicIndexDiffraction[0u],1u);
					traceBufferDiffraction[traceIndex]=trace;
				}
			} 
		}
	} else {
		//rtPrintf("%u\t%u\t%u Not diffracting point on edge %d\n",launchIndex.x,launchIndex.y,launchIndex.z,e.id);
	}
}
//Closest hit program for triangles and visibility rays
//It is recommended to use CH instead of AH since AH forces to use the SM, see https://developer.download.nvidia.com/video/gputechconf/gtc/2019/presentation/s9768-new-features-in-optix-6.pdf
RT_PROGRAM void closestHitTriangleDiffraction() {
	//float3 hp=ray_hit.origin + ray_hit.direction*ch_triangle_data.geom_normal_t.w;
	//float3 hp=ray_hit.origin + ray_hit.direction*rtIntersectionDistance();
	//	rtPrintf("%u\t%u\t%u hit on face=%u hp=(%f,%f,%f) \n",launchIndex.x,launchIndex.y,launchIndex.z,ch_triangle_data.faceId,hp.x,hp.y,hp.z);
	//rtPrintf("%u\t%u\t%u\t%d   hit on face=%u hp=(%f,%f,%f) \n",launchIndex.x,launchIndex.y,launchIndex.z,rayPayload.result.y,ch_triangle_data.faceId,hp.x,hp.y,hp.z);
	if (ch_triangle_data.faceId!=rayPayload.faces.x && ch_triangle_data.faceId!=rayPayload.faces.y) { 
		//rtPrintf("%u\t%u\t%u\t%d blocked on face=%u ray.faces=%u,%u hp=(%f,%f,%f) \n",launchIndex.x,launchIndex.y,launchIndex.z,rayPayload.result.y,ch_triangle_data.faceId,rayPayload.faces.x, rayPayload.faces.y,hp.x,hp.y,hp.z);
		//LoS is blocked, set flag 
		rayPayload.result.x=OPAL_DIFFRACTION_BLOCKED;
	} else {
		//rtPrintf("%u\t%u\t%u\t%d LoS on face=%u ray.faces=%u,%u hp=(%f,%f,%f) \n",launchIndex.x,launchIndex.y,launchIndex.z,rayPayload.result.y,ch_triangle_data.faceId,rayPayload.faces.x, rayPayload.faces.y,hp.x,hp.y,hp.z);
	}

}
RT_PROGRAM void closestHitCurvedTriangleDiffraction() {
	//float3 hp=ray_hit.origin + ray_hit.direction*curved_triangle_data.geom_normal_t.w;
	if (curved_triangle_data.faceId!=rayPayload.faces.x && curved_triangle_data.faceId!=rayPayload.faces.y) { 
		//rtPrintf("%u\t%u\t%u\t%d blocked on curved face=%u ray.faces=%u,%u hp=(%f,%f,%f) \n",launchIndex.x,launchIndex.y,launchIndex.z,rayPayload.result.y,curved_triangle_data.faceId,rayPayload.faces.x, rayPayload.faces.y,hp.x,hp.y,hp.z);
		//LoS is blocked, set flag 
		rayPayload.result.x=OPAL_DIFFRACTION_BLOCKED;
	} else {
		//rtPrintf("%u\t%u\t%u\t%d LoS on curved face=%u ray.faces=%u,%u hp=(%f,%f,%f) \n",launchIndex.x,launchIndex.y,launchIndex.z,rayPayload.result.y,curved_triangle_data.faceId,rayPayload.faces.x, rayPayload.faces.y,hp.x,hp.y,hp.z);
	}

}
//Miss program for visibility rays
RT_PROGRAM void missDiffraction() {
	//rtPrintf("%u\t%u\t%u miss \n", launchIndex.x,launchIndex.y,launchIndex.z); 
	//rtPrintf("%u\t%u\t%u\t%d miss \n", launchIndex.x,launchIndex.y,launchIndex.z,rayPayload.result.y); 
	rayPayload.result.x=OPAL_DIFFRACTION_MISS;
}

RT_PROGRAM void exception()
{
	const unsigned int code = rtGetExceptionCode();
	if (RT_EXCEPTION_USER <= code)
	{
		printf("User exception %d at (%d, %d)\n", code - RT_EXCEPTION_USER, launchIndex.x, launchIndex.y);
	}
	else
	{
		printf("Exception code 0x%X at (%d, %d)\n", code, launchIndex.x, launchIndex.y);
	}

}

