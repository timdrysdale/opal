#pragma once


#include <optixu/optixu_math_namespace.h>

//
// Common definitions shared by host and device code
//

//Use our own complex arithmetic

struct MaterialEMProperties {
	optix::float2 dielectricConstant;  //Complex
	
	//MaterialEMProperties(float relativePermitivity, float conductivity, float wavelength, int pol) {
		//WARNING: this is an appromixation of the dielectric constant
		//dielectricConstant = thrust::complex<float>(relativePermitivity, 60.0f*wavelength*conductivity);
		//polarization = pol;

	//}
};

struct DuplicateReflection {
	optix::float2 E; //Complex
	int r;
};

struct ReceptionInfo {
	optix::float2 sumRxElectricField; //Complex
	//bool hasDirectContribution;
	
	int directHits;
	int reflections;
};

struct EMWavePayload {
	optix::float3 geomNormal;
	optix::float3 nextDirection;
	optix::float3 hitPoint;
	optix::float3 polarization;
	float electricFieldAmplitude;
	float t;
	int reflections;
	int hits;
	float totalDistance;
	bool end;
	optix::float2 prodReflectionCoefficient; //Complex
	unsigned int faceId;



};

struct Transmitter {
	optix::float3 origin;
	optix::float3 polarization;
	int externalId;
};

struct TriangleHit
{
	optix::float3 geom_normal;
	float t;
	int   triId;
	float u;
	float v;
	unsigned int faceId;
	

};

struct SphereHit {
	optix::float3 geom_normal;
	float t;
	

};

/*struct ReflectionHit
{
	float3 hitPoint;
	float3 refDir;
};
*/
