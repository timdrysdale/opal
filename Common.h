#pragma once
#define GLOBALHITBUFFEROVERFLOW RT_EXCEPTION_USER + 0

#include <optixu/optixu_math_namespace.h>

//
// Common definitions shared by host and device code
//

// Use our own complex arithmetic

struct MaterialEMProperties {
  optix::float2 dielectricConstant; // Complex
};

struct HitInfo {
  optix::uint4 whrd; // [written,refhash,rxBufferIndex,dist] 
  optix::float2 E;   // Complex

  //Equality operator: hits are equal if the combined has is equal, that is, the have hit the same sequence of faces
  __host__ __device__ bool operator==(const HitInfo &h) const {
    return (whrd.y == h.whrd.y);
  };
  //Sorting. First check id (buffer index), then hash and finally distance. Hits are ordered by id, combined_hash and distance to receiver
  __host__ __device__ bool operator<(const HitInfo &h) const {
    if (whrd.z == h.whrd.z) {
      if (whrd.y == h.whrd.y) {
        return (whrd.w < h.whrd.w);
      } else {
        return (whrd.y < h.whrd.y);
      }
    } else {
      return (whrd.z < h.whrd.z);
    }
  };
};


//Ray payload
struct EMWavePayload {
  optix::float2 prodReflectionCoefficient; // Accumulated product of reflection coefficients. Complex
  optix::float3 geomNormal;
  optix::float3 nextDirection;
  optix::float3 hitPoint;
  optix::float3 polarization;
  optix::float3 lastReflectionHitPoint;
  float electricFieldAmplitude; //Can be removed if antenna gain not used
  float t;
  int reflections;
  int internalRayInitialReflections;
  int hits;
  float totalDistance;
  float totalDistanceTillLastReflection;
  bool end;
  unsigned int faceId; //Can be removed
  int rxBufferIndex; //Last receiver hit 
  unsigned int refhash; //Combined hash to filter duplicates
};

struct Transmitter {
  optix::float3 origin;
  optix::float3 polarization;
  int externalId;
};

struct TriangleHit {
  optix::float3 geom_normal;
  float t;
  int triId;
  float u;
  float v;
  unsigned int faceId;
};

struct SphereHit {
  optix::float3 geom_normal;
  float t;
};

//Directly copied from boost
template <typename SizeT>
inline void hash_combine_impl(SizeT &seed, SizeT value) {
  seed ^= value + 0x9e3779b9 + (seed << 6) + (seed >> 2);
}
