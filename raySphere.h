/***************************************************************/
//
//Copyright (c) 2020 Esteban Egea-Lopez http://girtel.upct.es/~eegea
//
/**************************************************************/

#ifndef RAYSPHERE_H
#define RAYSPHERE_H
#include "Opal.h"
#include <vector>
#include <curand.h>
#include <curand_kernel.h>
namespace opal {
	class OpalRaySphereGenerator {
		protected:
			opalthrustutils::DVector<float3>* raysDev;
			curandGenerator_t gen;	
			curandGenerator_t gen2;	
		public:
			OpalRaySphereGenerator();
			virtual ~OpalRaySphereGenerator();
			std::vector<optix::float3> generateTesselatedSphere(unsigned int depth);
			std::vector<optix::float3> generateRandomUniform(float eli,float elf,float azi, float aze, long r);
			//void saveToBinaryFile(optix::float3* data, long size, std::string fileName) ;
			//std::vector<optix::float3>  readFromBinaryFile(std::string fileName);
			void saveToFile(optix::float3*,long size,  std::string  fileName); 
			std::vector<optix::float3> readFromFile(std::string fileName);
			void generateRandomUniformOnDevice(float eli,float elf,float azi, float aze, long r); 
			void generateRandomUniformSphereOnDevice(long r);
			//TODO: implement and try the Hammersley sequence on GPU
			float3* getDevicePointer();
		protected:
			//Sphere tesselation
			void subdivide(const optix::float3 &v1, const optix::float3 &v2, const optix::float3 &v3, std::vector<optix::float3> &sphere_points,  unsigned int depth);
	};
}
#endif
