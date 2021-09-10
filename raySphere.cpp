/***************************************************************/
//
//Copyright (c) 2020 Esteban Egea-Lopez http://girtel.upct.es/~eegea
//
/**************************************************************/
#include "raySphere.h"
#include <random>
#include "timer.h"
#include <fstream>
#include "util.h"
using namespace optix;
namespace opal {
	OpalRaySphereGenerator::OpalRaySphereGenerator() {
		this->raysDev=nullptr;
		//unsigned long long seed1=1234ULL;
		//unsigned long long seed2=2357ULL;
		unsigned long long seed1=74211ULL;
		unsigned long long seed2=43535ULL;
		std::cout<<"Setting seeds to "<<seed1<<","<<seed2<<std::endl;

		opalthrustutils::initializeGenerators(&gen,seed1,&gen2,seed2);
		//opalthrustutils::initializeGenerators(&gen,5631ULL,&gen2,1379ULL);
	};
	OpalRaySphereGenerator::~OpalRaySphereGenerator() {
		if (raysDev) {
			delete raysDev;
		}
		if (gen) {
			curandDestroyGenerator(gen);
		}	
		if (gen2) {
			curandDestroyGenerator(gen2);
		}	
	}

	std::vector<optix::float3> OpalRaySphereGenerator::generateTesselatedSphere(unsigned int depth) {
		std::cout<<"Generating tesselated rays"<< std::endl;
		Timer timer;
		timer.start();
		std::vector<optix::float3> v;
		const double X = 0.525731112119133606;
		const double Z = 0.850650808352039932;
		const float3 vdata[12] = {
			{-X, 0.0, Z}, { X, 0.0, Z }, { -X, 0.0, -Z }, { X, 0.0, -Z },
			{ 0.0, Z, X }, { 0.0, Z, -X }, { 0.0, -Z, X }, { 0.0, -Z, -X },
			{ Z, X, 0.0 }, { -Z, X, 0.0 }, { Z, -X, 0.0 }, { -Z, -X, 0.0 }
		};
		int tindices[20][3] = {
			{0, 4, 1}, { 0, 9, 4 }, { 9, 5, 4 }, { 4, 5, 8 }, { 4, 8, 1 },
			{ 8, 10, 1 }, { 8, 3, 10 }, { 5, 3, 8 }, { 5, 2, 3 }, { 2, 7, 3 },
			{ 7, 10, 3 }, { 7, 6, 10 }, { 7, 11, 6 }, { 11, 0, 6 }, { 0, 1, 6 },
			{ 6, 1, 10 }, { 9, 0, 11 }, { 9, 11, 2 }, { 9, 2, 5 }, { 7, 2, 11 }
		};
		for(int i = 0; i < 20; i++) {
			subdivide(vdata[tindices[i][0]], vdata[tindices[i][1]], vdata[tindices[i][2]], v, depth);
		}
		timer.stop();
		std::cout<<"Generation time="<<timer.getTime()<<std::endl;
		return v;
	}


	void OpalRaySphereGenerator::subdivide(const float3 &v1, const float3 &v2, const float3 &v3, std::vector<float3> &sphere_points,  unsigned int depth) {
		if(depth == 0) {
			sphere_points.push_back(v1);
			sphere_points.push_back(v2);
			sphere_points.push_back(v3);
			return;
		}
		const float3 v12 = normalize((v1 + v2));
		const float3 v23 = normalize((v2 + v3));
		const float3 v31 = normalize((v3 + v1));
		subdivide(v1, v12, v31, sphere_points, depth - 1);
		subdivide(v2, v23, v12, sphere_points, depth - 1);
		subdivide(v3, v31, v23, sphere_points, depth - 1);
		subdivide(v12, v23, v31, sphere_points, depth - 1);
	}

	std::vector<optix::float3> OpalRaySphereGenerator::generateRandomUniform(float eli,float elf,float azi, float aze, long r) {
		
		
		//Pass to rad
		float cc=M_PIf/180.0f;
		eli=eli*cc;
		elf=elf*cc;
		azi=azi*cc;
		aze=aze*cc;

		Timer timer;
		std::vector<float3> v;
		std::cout<<"Generating "<<r<<" random rays"<< std::endl;
		timer.start();
		std::random_device rd;  
		std::mt19937 gen(rd()); //Standard mersenne_twister_engine seeded with rd()
		//std::mt19937 gen1(2327); //Fixed seed
		//std::mt19937 gen2(5213); //Fixed seed
		std::mt19937 gen1(6437); //Fixed seed
		std::mt19937 gen2(8323); //Fixed seed
		std::uniform_real_distribution<> dis1(0.0, 1.0);
		std::uniform_real_distribution<> dis2(0.0, 1.0);
		long p=0; //Current points
		//Extremely slow. Unusable
		//while (p<r) {
		//	float elr=acosf(1-2*dis1(gen1));
		//	if ((elr>=eli) && (elr<=elf)) {
		//		float azr=2*M_PI*dis2(gen2);
		//		if ((azr>=azi) && (azr<=aze)) {
		//			v.push_back(make_float3(sinf(elr)*sinf(azr), cosf(elr), sinf(elr)*cosf(azr) ));
		//			++p;
		//		}

		//	} 
		//	

		//}
		while (p<r) {
			float cosEl=cosf(eli)+dis1(gen1)*(cosf(elf)-cosf(eli));
			float sinEl = sqrt(1.0f - (cosEl*cosEl));
			float azr=(aze-azi)*dis2(gen2) + azi;
			v.push_back(make_float3(sinEl*sinf(azr), cosEl, sinEl*cosf(azr) ));
			++p;
		}
		timer.stop();
		std::cout<<"Time="<<timer.getTime()<<std::endl;
		return v;
	}
	void OpalRaySphereGenerator::generateRandomUniformSphereOnDevice(long r) {
		//Generate random rays uniformly on the sphere surface
		if (raysDev) {
			delete raysDev;
			raysDev=nullptr;
		}

		Timer timer;

		std::cout<<"Generating "<<r<<" random rays on sphere on GPU"<< std::endl;
		timer.start();
		raysDev = new opalthrustutils::DVector<optix::float3>();
		opalthrustutils::generateRandomUniformRaysOnSphere(r,raysDev,gen,gen2);
		timer.stop();
		std::cout<<"Time="<<timer.getTime()<<"generated r="<<raysDev->getDeviceVectorSize()<<std::endl;
	}
	void OpalRaySphereGenerator::generateRandomUniformOnDevice(float eli,float elf,float azi, float aze, long r) {

		//TODO: probably better to pass the optix Buffer to thrust and use getDevicePointer...Otherwise we have to be sure the device ordinal we are using
		//Pass to rad
		float cc=M_PIf/180.0f;
		eli=eli*cc;
		elf=elf*cc;
		azi=azi*cc;
		aze=aze*cc;
		if (raysDev) {
			delete raysDev;
			raysDev=nullptr;
		}

		Timer timer;

		//std::cout<<"Generating "<<r<<" random rays on GPU"<< std::endl;
		//timer.start();
		//raysDev = new thrust::device_vector<float3>(r);
		raysDev = new opalthrustutils::DVector<optix::float3>();
		opalthrustutils::generateRandomUniformRays(eli, elf, azi, aze,r,raysDev,gen,gen2);
		//timer.stop();
		//std::cout<<"Time="<<timer.getTime()<<"generated r="<<raysDev->getDeviceVectorSize()<<std::endl;
	}
	void OpalRaySphereGenerator::saveToFile(optix::float3* f, long size, std::string fileName) {
		std::ofstream file(fileName.c_str(),std::ofstream::out);

		for (int i=0; i<size; ++i ) {
			file<<f[i].x<<"\t"<<f[i].y<<"\t"<<f[i].z<<std::endl;
		}
		file.close();
	}
	std::vector<optix::float3> OpalRaySphereGenerator::readFromFile(std::string fileName) {
		std::ifstream infile(fileName);
		float x, y, z;
		//char c;
		std::vector<float3> vertices;
		std::string line;


		while (std::getline(infile, line)) {

			//std::cout << line << std::endl;
			std::string delimiters = "\t";
			size_t current;
			size_t next = -1;
			int p = 0;
			do
			{
				current = next + 1;
				next = line.find_first_of(delimiters, current);
				if (p == 0) {
					x = std::stof(line.substr(current, next - current));
				}
				if (p == 1) {
					y = std::stof(line.substr(current, next - current));
				}
				if (p == 2) {
					z = std::stof(line.substr(current, next - current));
				}

				//std::cout << line.substr(current, next - current) <<"\t"<< std::endl;
				p++;
			} while (next != std::string::npos);

			vertices.push_back(make_float3(x, y, z));
		}
		//	std::cout << "Loaded " << vertices.size() << " rays from " << file << std::endl;
		infile.close();
		if (vertices.size()==0) {
			std::cout<<"WARNING: loaded zero rays!!!"<<std::endl;
		}
		return vertices;
	}	
	float3* OpalRaySphereGenerator::getDevicePointer() {
		return raysDev->getPointer();
	} 
}
