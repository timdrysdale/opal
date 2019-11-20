/***************************************************************/
//
//Copyright (c) 2019 Esteban Egea-Lopez http://ait.upct.es/eegea
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

//Use to generate traces of the rays that make the incident electric field (rays that have hit, after filtering)
//Need an external script to process the traces
//#define OPAL_LOG_TRACE


//Use it to avoid self intersections
//See https://www.realtimerendering.com/raytracinggems/ 6.1
//But this is going to introduce some small precision errors in the computed electric field, since the ray lengths are modified due to the displacement
//If you do not need very high accuracy (which is the usual) you can  uncomment it
//Otherwise keep it, but then you have to tune minEpsilon, which is not scale-invariant. If your simulation gets stuck in a launch, try uncommenting it.
//If it is no longer stuck but you need high precision, comment it again an tune minEpsilon until it is removed.

//#define OPAL_AVOID_SI




//Flags and variables
#define FLAG_NONE 0
#define FLAG_END 1

#define OPAL_RAY_REFLECTION 0u
#define OPAL_RAY_LOG_TRACE_RAY 1u 


#include <optixu/optixu_math_namespace.h>


struct MaterialEMProperties {
	optix::float2 dielectricConstant; // Complex
	optix::float2 tattenuation; //[Thickness, attenuation/m (dB/m)  ] //attenuation should be negative, like -15 dB/m so [0.1,-15]
};

struct HitInfo {
	//Packed to fullfill alignment rules, see for instance https://devtalk.nvidia.com/default/topic/1037798/optix/optix-time-for-launch/
	optix::uint4 thrd; // [txBufferIndex,refhash,rxBufferIndex,distance] 
	optix::float2 E;   // Incident electric field or induced voltage on the antenna. Complex 
	//For debug only
	//	optix::float3 h;
	//	optix::float3 v;
	//	optix::uint3 in; //launchIndex 
	//optix::float4 lh; //LastHitPoint,d
	//optix::uint r; //reflections
	optix::float2 Ex;   // Complex
	optix::float2 Ey;   // Complex
	optix::float2 Ez;   // Complex
	//	optix::float2 Rn;   // Complex
	//	optix::float2 Rp;   // Complex

#ifdef OPAL_LOG_TRACE
	optix::float3 rayDir; //Used for visualization
#endif 	
	//Equality operator: used by thrust::unique. Hits are equal if  the transmitter  and the receiver is the same and  the combined hash is equal, that is, they have hit the same sequence of faces. We get
	//the closest one because we have previously sorted the sequence
	__forceinline__  __device__ bool operator==(const HitInfo &h) const {
		return ((thrd.x==h.thrd.x)  && (thrd.y == h.thrd.y) && (thrd.z == h.thrd.z)  );
	};
	//Sorting. Here we first order by txId (tx buffer index), then check receiver id (receiver buffer index), then hash and finally distance. Hits are ordered by txId, rxid, combined_hash and distance to receiver
	__forceinline__  __device__ bool operator<(const HitInfo &h) const {
		if (thrd.x==h.thrd.x) {
			if (thrd.z == h.thrd.z) {
				if (thrd.y == h.thrd.y) {
					return (thrd.w < h.thrd.w);
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








//Ray payloads

struct BaseReflectionPayload {
	optix::float4 ndtd; //Packed next direction and total distance [nextDirection.x,nextDirection.y,nextDirection.z,totalDistance]
	optix::float4 lrhpd; //Packed lastReflectionHitPoint and totalDistanceTillLastReflection [lastReflectionHitPoint.x,lastReflectionHitPoint.y,lastReflectionHitPoint.z, totalDistanceTillLastReflection]
	optix::float4 hitPointAtt; //Packed hitPoint and attenuation [hitPoint.x,hitPoint.y,hitPoint.z,att]
	optix::uint4 rhfr; //Packed [reflections,hits,flags,refhash]
#ifdef OPAL_AVOID_SI
	optix::float3 lastNormal;
#endif
#ifdef OPAL_LOG_TRACE
	optix::float3 initialRayDir;
#endif
};


//Used for pure Horizontally or Vertically polarized waves. polarization should only be horizontal or vertical with respect to the environment
//Order matters: using CUDA vector alignment rules: https://docs.nvidia.com/cuda/cuda-c-programming-guide/index.html#vector-types__alignment-requirements-in-device-code
struct HVWavePayload :  BaseReflectionPayload {
	optix::float2 prodReflectionCoefficient; // Accumulated product of reflection coefficients. Complex
	//TODO: pack the fields below
	optix::float3 polarization;
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

struct Transmitter {
	//TODO: Pack polarization and transmit power [polarization.x,polarization.y,polarization.z,txPower]
	optix::float3 polarization; 	
	optix::float3 origin;
	float txPower;
	int externalId;
};

struct TriangleHit {
	optix::float4 geom_normal_t; //Packed normal and t [geom_normal.x,geom_normal.y,geom_normal.z,t] 
	optix::float3 hp;
	int triId;
	unsigned int faceId;
};

struct SphereHit {
	optix::float4 geom_normal_t; //Packed normal and t [geom_normal.x,geom_normal.y,geom_normal.z,t] 
};

//Directly copied from boost
template <typename SizeT>
inline void hash_combine_impl(SizeT &seed, SizeT value) {
	seed ^= value + 0x9e3779b9 + (seed << 6) + (seed >> 2);
}


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

