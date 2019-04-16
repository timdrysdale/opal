/***************************************************************/
//
//Copyright (c) 2019 Esteban Egea-Lopez http://ait.upct.es/eegea
//
/**************************************************************/


#pragma once
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
	optix::float2 E;   // Complex

	//Equality operator: hits are equal if the combined has is equal and the transmitter is the same, that is, the have hit the same sequence of faces
	__forceinline__  __device__ bool operator==(const HitInfo &h) const {
		return ((thrd.x==h.thrd.x) && (thrd.y == h.thrd.y)  );
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


//Ray payload
struct EMWavePayload {
	optix::float2 prodReflectionCoefficient; // Accumulated product of reflection coefficients. Complex
	optix::float3 geomNormal;
	optix::float3 nextDirection;
	optix::float3 hitPoint;
	optix::float3 polarization;
	optix::float3 lastReflectionHitPoint;
	float electricFieldAmplitude; //Can be removed if antenna gain not used
	//float t;
	int reflections;
	int internalRayInitialReflections;
	int hits;
	float totalDistance;
	float totalDistanceTillLastReflection;
	float accumulatedAttenuation; //For penentration, in dB (Power)
	bool end;
	//unsigned int faceId; //Can be removed
	int rxBufferIndex; //Last receiver hit 
	unsigned int refhash; //Combined hash to filter duplicates
};

struct Transmitter {
	optix::float3 origin;
	optix::float3 polarization;
	int externalId;
};

struct TriangleHit {
	optix::float4 geom_normal_t; //Pack normal and t 
	int triId;
	unsigned int faceId;
};

struct SphereHit {
	optix::float4 geom_normal_t; //Pack normal and t
};

//Directly copied from boost
template <typename SizeT>
inline void hash_combine_impl(SizeT &seed, SizeT value) {
	seed ^= value + 0x9e3779b9 + (seed << 6) + (seed >> 2);
}
