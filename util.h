/***************************************************************/
//
//Copyright (c) 2019 Esteban Egea-Lopez http://girtel.upct.es/~eegea
//
/**************************************************************/


#ifndef UTIL_H
#define UTIL_H
#include <vector>
//#include <optix_world.h>
#include "json.hpp"
#include "Opal.h"

using json=nlohmann::json;

using namespace optix;
namespace opal {
//Polarizations 
const float3 V=make_float3(0.0f, 1.0f, 0.0f); //Perpendicular to the floor. Assuming as in Unity that forward is z-axis and up is y-axis
const float3 H=make_float3(1.0f, 0.0f, 0.0f); //Parallel to the floor. Assuming as in Unity that forward is z-axis and up is y-axis
const float3 VH=make_float3(1.0f, 1.0f, 0.0f); 
const float3 HZ=make_float3(0.0f, 0.0f, 1.0f); 

	class ScenarioLoader {
		protected:
			OpalSceneManager* sceneManager;
		public:
			ScenarioLoader(OpalSceneManager* m);
			void loadMeshesFromFiles(std::string path); 
    			void loadEdgesFromFiles(std::string path);
			void loadEdgeFromFile(const char* file);
			std::vector<std::pair<optix::int3,unsigned int> > loadFaceIdsFromFile(std::ifstream& infile);
			optix::float3 readFloat3(std::string line);
			optix::float4 readFloat4(std::string line);
			optix::int4 readInt4(std::string line);	
			optix::Matrix4x4 loadTransformFromFile(const char* file);
		        MaterialEMProperties loadEMFromFile(const char* file);
			optix::float3 readJsonFloat3(json o); 
			optix::float4 readJsonFloat4(json o); 
			optix::float4 readJsonCurvatureData(json o); 
			void loadJSONScenario(std::string path);
			std::vector<float3> readJsonVertices(json o); 
			optix::Matrix4x4 readJsonMatrix(json o); 
			MaterialEMProperties readJsonEm(json o); 
			std::vector<std::pair<optix::int3,uint> > readJsonFaces(json o); 
   			void loadJsonEdge(json o, MaterialEMProperties prop); 
			std::vector<optix::float4>  readJsonCurvature(json o);
};
/*	void printPower(float power, int txId ); 
	std::vector<float3>  loadVerticesFromFile(const char* file); 
	std::vector<int>  loadTrianglesFromFile(const char* file) ;
	std::vector<float4>  loadPDFromFile(const char* file); 
	std::vector<float3>  loadRaysFromFile(const char* file);
*/	
	class Statistics {
		protected:
		double mean;
		double m2;
		long count;
		public:
		Statistics();
		void update (double v);
		double getVariance() const;
		double getMean() const;
		double getCount() const;
		double getStd() const;
		void reset();

	};
	//Very simple and inefficient class for histograms. Used only for debug. If more serious things are needed resort to a proper library, like boost.histogram 
	class Histogram {
		protected:
			std::vector<unsigned long> bins;
			float max;
			float min;
			float width;
		public:
			Histogram(unsigned int bins, float min, float max);
			void update(float value);
			std::string print();
			void reset();
	};
	

}
#endif

