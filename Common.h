/***************************************************************/
//
//Copyright (c) 2019 Esteban Egea-Lopez http://ait.upct.es/eegea
//
/**************************************************************/

#ifndef COMMON_H
#define COMMON_H

#define GLOBALHITBUFFEROVERFLOW RT_EXCEPTION_USER + 0
//Comment out if using Optix 5.1
#define OPAL_USE_TRI 

#include <optixu/optixu_math_namespace.h>
//
// Common definitions shared by host and device code
//

// Use our own complex arithmetic

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
//	optix::float2 Ex;   // Complex
//	optix::float2 Ey;   // Complex
//	optix::float2 Rn;   // Complex
//	optix::float2 Rp;   // Complex
	
	
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



//TODO:Pack these structures as suggested in documentation


//Ray payloads




#define FLAG_NONE 0
#define FLAG_END 1

//Used for pure Horizontally or Vertically polarized waves. polarization should only be horizontal or vertical with respect to the environment
//Order matters: using CUDA vector alignment rules: https://docs.nvidia.com/cuda/cuda-c-programming-guide/index.html#vector-types__alignment-requirements-in-device-code
struct HVWavePayload {
	optix::float4 ndtd; //Packed next direction and total distance [nextDirection.x,nextDirection.y,nextDirection.z,totalDistance]
	//Unpacked version
	//optix::float3 nextDirection;
	//float totalDistance;
	optix::float4 lrhpd; //Packed lastReflectionHitPoint and totalDistanceTillLastReflection [lastReflectionHitPoint.x,lastReflectionHitPoint.y,lastReflectionHitPoint.z, totalDistanceTillLastReflection]
	//Unpacked version
	//optix::float3 lastReflectionHitPoint;
	//float totalDistanceTillLastReflection;
	optix::float2 prodReflectionCoefficient; // Accumulated product of reflection coefficients. Complex
	optix::float3 hitPoint;
	optix::float3 polarization;
	float accumulatedAttenuation; //For penenetration, in dB (Power)
	float electricFieldAmplitude; //Can be removed if antenna gain not used
	int reflections;
	int hits;
	int flags; //Better to use int with flags (bool is system dependent), 
	unsigned int refhash; //Combined hash to filter duplicates
};

//Used for arbitrary linear polarizations (LP) (given by a float3)
//Order matters: using CUDA vector alignment rules: https://docs.nvidia.com/cuda/cuda-c-programming-guide/index.html#vector-types__alignment-requirements-in-device-code
struct LPWavePayload {
	//Copy common fields. I do not see the need for inheritance here yet
	optix::float4 ndtd; //Packed next direction and total distance [nextDirection.x,nextDirection.y,nextDirection.z,totalDistance]
	optix::float4 lrhpd; //Packed lastReflectionHitPoint and totalDistanceTillLastReflection [lastReflectionHitPoint.x,lastReflectionHitPoint.y,lastReflectionHitPoint.z, totalDistanceTillLastReflection]
	optix::float2 hor_coeff; //Complex 
	optix::float2 ver_coeff; //Complex 
	optix::float3 hor_v;
	optix::float3 ver_v; 
	optix::float3 hitPoint;
	float electricFieldAmplitude; //Can be removed if antenna gain not used
	float accumulatedAttenuation; //For penentration, in dB (Power)
	int hits;
	int reflections;
	int flags;
	unsigned int refhash; //Combined hash to filter duplicates
};

struct Transmitter {
	//TODO: Pack polarization and transmit power [polarization.x,polarization.y,polarization.z,txPower]
	optix::float3 polarization; 	
	optix::float3 origin;
	float txPower;
	int externalId;
};

struct TriangleHit {
	optix::float4 geom_normal_t; //Pack normal and t [geom_normal.x,geom_normal.y,geom_normal.z,t] 
	int triId;
	unsigned int faceId;
};

struct SphereHit {
	optix::float4 geom_normal_t; //Pack normal and t [geom_normal.x,geom_normal.y,geom_normal.z,t] 
};

//Directly copied from boost
template <typename SizeT>
inline void hash_combine_impl(SizeT &seed, SizeT value) {
	seed ^= value + 0x9e3779b9 + (seed << 6) + (seed >> 2);
}


#endif

