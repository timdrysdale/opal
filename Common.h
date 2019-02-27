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

  __host__ __device__ bool operator==(const HitInfo &h) const {
    return (whrd.y == h.whrd.y);
  };
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

struct EMWavePayload {
  optix::float2 prodReflectionCoefficient; // Complex
  optix::float3 geomNormal;
  optix::float3 nextDirection;
  optix::float3 hitPoint;
  optix::float3 polarization;
  optix::float3 lastReflectionHitPoint;
  float electricFieldAmplitude;
  float t;
  int reflections;
  int internalRayInitialReflections;
  int hits;
  float totalDistance;
  float totalDistanceTillLastReflection;
  bool end;
  unsigned int faceId;
  int rxBufferIndex;
  unsigned int refhash;
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
template <typename SizeT>
inline void hash_combine_impl(SizeT &seed, SizeT value) {
  seed ^= value + 0x9e3779b9 + (seed << 6) + (seed >> 2);
}
