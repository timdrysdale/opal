/***************************************************************/
//
//Copyright (c) 2019 Esteban Egea-Lopez http://girtel.upct.es/~eegea
//
/**************************************************************/
//
// Common definitions shared by host and device code
//

#ifndef COMMON_H
#define COMMON_H

#define GLOBALHITBUFFEROVERFLOW RT_EXCEPTION_USER + 0

//ONLY Comment out if using Optix 5.1
#define OPAL_USE_TRI 





/***********************/
//Use it to avoid self intersections
//See https://www.realtimerendering.com/raytracinggems/ 6.1
//But this is going to introduce some small precision errors in the computed electric field, since the ray lengths are modified due to the displacement
//If you do not need very high accuracy (which is the usual) you can  uncomment it
//Otherwise keep it, but then you have to tune minEpsilon, which is not scale-invariant. If your simulation gets stuck in a launch, try uncommenting it.
//If it is no longer stuck but you need high precision, comment it again an tune minEpsilon until it is removed.

//#define OPAL_AVOID_SI
/*******************/






//Flags and variables
#define FLAG_NONE 0u

//Use a uint with  32 bit positions for flags, at the moment. Positions here show the position of the flag to be checked
#define FLAG_END_POSITION 0u
#define FLAG_CURVED_MESH_POSITION 1u



//Visibility masks
typedef unsigned int OpalVisibilityMask;
enum {
	OPAL_STATIC_MESH_MASK = 0x01u,
	OPAL_RECEIVER_MASK = 0x02u
};

#define OPAL_DIFFRACTION_LOS 0
#define OPAL_DIFFRACTION_BLOCKED 1
#define OPAL_DIFFRACTION_MISS 2


//DO NOT MESS WITH THE ORDER OF INCLUDES OR THE INCLUDED HEADERS HERE
//TODO: I have not found yet why changing the order of some includes break either the compilation or the nvrtc compilation..
//#include <optix_world.h>
//#include <optixu/optixu_math_namespace.h>
#include <optixu/optixu_matrix_namespace.h> 
//#include <optixu/optixpp_namespace.h>
#include <optixu/optixu_math_namespace.h>



#define OPAL_INFINITY_F __int_as_float(0x7f800000);//This should be infinity

struct MaterialEMProperties {
	optix::float4 ITUParameters;
	//Keep this to make backward compatible
	optix::float2 dielectricConstant; // Complex. Set imaginary part to infinity for perfect conductors
	optix::float2 tattenuation; //[Thickness, attenuation/m (dB/m)  ] //attenuation should be negative, like -15 dB/m so [0.1,-15]
};


//Representation of an edge for simple diffraction. 
//We assume wedge diffraction. A diffracting edge is the edge separating two faces
//We assume the faces are rectangular, so we store the diffracting edge (v), and the starting point of the edge (P) and faces represented as vectors (a,b), plus their id and normals
//                       v|
//            faces.x     |
//            	      WA ^| faces.y
//                 a ___/_|____ b 
//                      | P  |
//               normal_a    normal_b
//
//
// The WA angle is given by the angle between a and b and is defined as WA=(2-n)*pi see e.g. Balanis, Advanced Engineering Electromagnetics. We store the n instead of WA
// WA is measured from a to b (clockwise)
// WA here is the internal angle in relation to the normals, that is, if a normal points to one side of the supporting plane, the angle is measured from the opposite side
// A face id has to correspond with the face id of the triangles it is made of, obviously
// Of course, some information is redundant, can be derived (such as normals), but as long as we have enough memory I prefer this implementation
struct Edge {
	optix::float4 v; //The diffracting edge [Unit vector, length] 
	optix::float4 a; //Vector of a face [Unit vector, length] 
	optix::float4 b; //Vector of the other face [Unit vector, length] 
	optix::float4 pn; //Point of origin, n (n corresponding to the internal angle of the wedge WA=(2-n)*pi [point,n]
	optix::uint2 faces; //Faces this edge belongs to
	optix::float3 n_a; //Normal of  face a (unit vector)
	optix::float3 n_b; //Normal of  face b (unit vector)
	MaterialEMProperties mat;
	unsigned int id; //For debugging, can be removed
};



//Packed to fullfill alignment rules, see for instance https://devtalk.nvidia.com/default/topic/1037798/optix/optix-time-for-launch/
struct HitInfo {
	optix::float4 rdud; //Packed  [rayDir (float3), sqrDistToReceiver (float)], rayDir is only used with angle discrimination and log traces, but is is probably better to use a float4 hee
	optix::uint4 thrd; // [txBufferIndex,refhash,rxBufferIndex,hitOnCurved] //Last integer can be used as different flags, used at the moment to flag hits on curved surfaces
	//Pack incident field or induced voltage on the antenna. We waste some memory, but we can avoid the use of the EXTENDED_HIT_INFO macro below and avoid changing a lot of code. 
	//Probably the performance is also better but I have not tested it
	optix::float4 EEx; //Packed [E (float, float), Ex (float, float)] Use E for induced voltage and Ex for X components of incident field at receiver
	optix::float4 EyEz; //Packed [Ey (float, float), Ez (float, float)] Ey and Ez are the X an Y components of the incident field at the receiver

	
	//Additional output. TODO: to make this properly, we should extend from this struct. But then, there is a lot of code to change in tutils.cu... change to templates..
	optix::float4 doaD; //Packed [DoA (float3), unfolded distance float]
	



 
	//Equality operator: used by thrust::unique. Hits are equal if  the transmitter  and the receiver is the same and  the combined hash is equal, that is, they have hit the same sequence of faces. We get
	//the closest one because we have previously sorted the sequence
	__forceinline__ __host__  __device__ bool operator==(const HitInfo &h) const {
		return ((thrd.x==h.thrd.x)  && (thrd.y == h.thrd.y) && (thrd.z == h.thrd.z)  );
	};
	//Sorting. Here we first order by txId (tx buffer index), then check receiver id (receiver buffer index), then hash and finally distance. Hits are ordered by txId, rxid, combined_hash and distance to receiver
	__forceinline__  __device__ bool operator<(const HitInfo &h) const {
		if (thrd.x==h.thrd.x) { //tx are equal
			if (thrd.z == h.thrd.z) { //rx are equal
				if (thrd.y == h.thrd.y) { //hash are equai
						return (rdud.w<h.rdud.w); //ray distance to receiver
					//return (unfoldedDistance<h.unfoldedDistance);
					//return (thrd.w < h.thrd.w); //distance
				} else {
					return (thrd.y < h.thrd.y); 
				}
			} else {
				return (thrd.z < h.thrd.z);
			}
		} else {
			return (thrd.x<h.thrd.x);
		}
	};
};



//Struct used to store log traces info
struct	LogTraceHitInfo {
	optix::float4 hitp;
	optix::uint4 cdata; //Context data. For reflections [rayIndex,reflections, unused,unused]. For diffraction [receiver, transmitter, edge,order]
	//Sorting. Here we first order by rayIndex  then reflections
	__forceinline__  __device__ bool operator<(const LogTraceHitInfo &h) const {
		if (cdata.x==h.cdata.x) { //rayIndex is equal
			if (cdata.y==h.cdata.y) {
				if (cdata.z==h.cdata.z) {
					return (cdata.w<h.cdata.w);
				} else {
					return (cdata.z<h.cdata.z);
				}
			} else {
				return (cdata.y<h.cdata.y);
			}
		} else {
			return (cdata.x<h.cdata.x);
		}	
	};
};




//Ray payloads

struct BaseReflectionPayload {
	optix::float4 ndtd; //Packed next direction and total distance [nextDirection.x,nextDirection.y,nextDirection.z,totalDistance]
	optix::float4 lrhpd; //Packed lastReflectionHitPoint and totalDistanceTillLastReflection [lastReflectionHitPoint.x,lastReflectionHitPoint.y,lastReflectionHitPoint.z, totalDistanceTillLastReflection]. lastReflectionHitPoint is used to compute wave attenuation mainly. It is different from hitPoint because the latter is updated when we hit a sphere receiver which is not a real physical entity
	optix::float4 hitPointAtt; //Packed hitPoint and attenuation [hitPoint.x,hitPoint.y,hitPoint.z,att]. hitPoint is used as origin in the generation of the next ray mainly
	optix::uint4 rhfr; //Packed [reflections,hits,flags,refhash]
	optix::float4 polarization_k;
#ifdef OPAL_AVOID_SI
	optix::float3 lastNormal;
#endif

//Use for trace logs: this introduce a performance penaly but it is easier to integrate visualization globally...
	optix::float4 initialRayDir;
};


//Used for pure Horizontally or Vertically polarized waves. polarization should only be horizontal or vertical with respect to the environment
//Order matters: using CUDA vector alignment rules: https://docs.nvidia.com/cuda/cuda-c-programming-guide/index.html#vector-types__alignment-requirements-in-device-code
struct HVWavePayload :  BaseReflectionPayload {
	//optix::float4 rpa; //Packed [prodReflectionCoefficient (x,y), polarization type (z), electricFieldAmplitude]

	optix::float2 prodReflectionCoefficient; // Accumulated product of reflection coefficients. Complex
	//TODO: pack the fields below
	float electricFieldAmplitude; //Can be removed if antenna gain not used
};

//Used for arbitrary linear polarizations (LP) (given by a float3)
struct LPWavePayload : BaseReflectionPayload {
	optix::float2 hor_coeff; //Complex 
	optix::float2 ver_coeff; //Complex 
	
	optix::float3 hor_v; //'Vertical' vector of the electric field
	optix::float3 ver_v; //'Horizontal' vector of the electric field 
	float electricFieldAmplitude; //Can be removed if antenna gain not used
};
//TODO: payloads  for curved meshes should use their own payload with packed and padded fields to be more efficient
//Used for simulations with curved meshes and linear polarizations (LP) 
struct CurvedMeshLPWavePayload : LPWavePayload {
	optix::float2 radii;
	optix::float2 divergence; //Complex to take into account phase shifts
	optix::float3 p1; //Principal planes
	optix::float3 p2; //Principal planes 
};
//Used for Ray Density Normalization 
struct RDNLPWavePayload :CurvedMeshLPWavePayload {
	float rayDensity;
	int phaseJumps;
};
struct Transmitter {
	// Since this is used in device, packed for a more efficient representation
	//Pack polarization and wavenumber [polarization.x,polarization.y,polarization.z,k]
	optix::float4 polarization_k; 	
	// Pack origin and transmit power [origin.x,origin.y,origin.z,txPower]
	optix::float4 origin_p;
	//float txPower;
	int externalId;
	//gain
	rtBufferId<float,2> gainId;
	optix::Matrix<4,4> transformToPolarization;
      
};

struct TriangleHit {
	optix::float4 geom_normal_t; //Packed normal and t [geom_normal.x,geom_normal.y,geom_normal.z,t] 
	optix::float3 hp;
	int triId;
	unsigned int faceId;
};
struct CurvedTriangleHit {
	optix::float4 geom_normal_t; //Packed normal and t [geom_normal.x,geom_normal.y,geom_normal.z,t] 
	optix::float4 principalDirection1; //Packed [unit principal direction, curvature]
	optix::float4 principalDirection2;
	optix::float3 hp;
	int triId;
	unsigned int faceId;
};

struct SphereHit {
	optix::float4 geom_normal_t; //Packed normal and t [geom_normal.x,geom_normal.y,geom_normal.z,t] 
};

struct RDNParameters {
	optix::uint4 filter_rc_nrx; //[Filtering mode, rayCount, number of receivers, not_used]
	float initialDensity;
	//unsigned int filterDensity;
};

//Used for simple diffraction visibility rays
struct VisibilityPayload {
	optix::float4 polarization_k; //Need to include this for multichannel to get the reflection coefficient in diffraction
	optix::uint2 faces;
	optix::int2 result;
	//int dir;
};

//Directly copied from boost
template <typename SizeT>
inline void hash_combine_impl(SizeT &seed, SizeT value) {
	seed ^= value + 0x9e3779b9 + (seed << 6) + (seed >> 2);
}

//Typedefs for RDN particular hits

typedef struct {float4 EEx; float4 EyEz; /*Compute additional output. Remove if performance requires it*/ float4 doaD; float4 doDu;} RDNHit;

//#ifdef OPAL_EXTENDED_HITINFO
//	typedef struct { float2 Ex; float2 Ey; float2 Ez;} RDNHit;
//#else
//	typedef float2 RDNHit;
//#endif

#ifdef OPAL_AVOID_SI

	//To avoid self-intersection, shift the ray along the geometric normal of the last intersection
	//This should be enough to get rid of minEpsilon, which is not scale-invariant
	//See https://www.realtimerendering.com/raytracinggems/ 6.1
	
	// constexpr __device__  float origin()      { return 1.0f / 32.0f; } 
	// constexpr __device__  float float_scale() { return 1.0f / 65536.0f; } 
	// constexpr __device__  float int_scale()   { return 256.0f; }    
	
	#define ORIGIN 1.0f/32.0f
	#define FLOAT_SCALE 1.0f/65536.0f
	#define INT_SCALE 256.0f
	// Normal points outward for rays exiting the surface, else is flipped. 
	
	__forceinline__ __device__ float3 offset_ray(const float3 p, const float3 n)  {   
		int3 of_i=make_int3(INT_SCALE * n.x, INT_SCALE * n.y, INT_SCALE * n.z); 
		float3 p_i=make_float3(optix::int_as_float(optix::float_as_int(p.x)+((p.x < 0) ? -of_i.x : of_i.x)), 
				optix::int_as_float(optix::float_as_int(p.y)+((p.y < 0) ? -of_i.y : of_i.y)), 
				optix::int_as_float(optix::float_as_int(p.z)+((p.z < 0) ? -of_i.z : of_i.z))); 
		return make_float3(fabsf(p.x) < ORIGIN ? p.x+FLOAT_SCALE*n.x : p_i.x, fabsf(p.y) < ORIGIN ? p.y+FLOAT_SCALE*n.y : p_i.y, fabsf(p.z) < ORIGIN ? p.z+FLOAT_SCALE*n.z : p_i.z); 
	}
	
#endif

#endif //COMMON_H

