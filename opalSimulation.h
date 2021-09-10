/***************************************************************/
//
//Copyright (c) 2019 Esteban Egea-Lopez http://girtel.upct.es/~eegea
//
/**************************************************************/

#ifndef OPALSIMULATION_H
#define OPALSIMULATION_H
#include "ptxUtil.h"
#include "Opal.h"

namespace opal {
	//Forward declarations
	class PtxUtil;
	class OpalSceneManager;
	struct OpalMesh;
	
	enum OpalSimulationTypes {
		NONE,
		BASIC,
		FLATWALLS,
		CURVEDWALLS,
		CURVEDFLATWALLS,
		RDN,
                SINGLEDIFFRACTION

	};
	enum ComputeMode {
		VOLTAGE, // (0) Induced voltage at the receiver antenna
		FIELD //(1) Field at the receiver position 
	};
	class OpalSimulation {
		protected:
			OpalSceneManager* myManager;
			OpalSimulationTypes simType;	
			ComputeMode mode;
			bool printHits;
			bool enableSimulation;
			bool generateTraceLog;
			//Programs
			PtxUtil*  ptxHandler;
			unsigned int reflectionEntryIndex;
			unsigned int reflectionRayIndex;
			unsigned int traceEntryIndex;
			unsigned int traceRayIndex;
	
			//Configuration
			std::string optixProgramsDir;
			std::string cudaProgramsDir;
			std::string currentDir;
			bool acceptCurved;
			bool acceptEdge;
			bool withPolarization;
			bool withRaySphere;
			float fractionMemGlobalBufferSize;
			uint maxGlobalBufferSize;
			bool initialized;	
			//Internal buffers
			optix::Buffer globalHitInfoBuffer;
			optix::Buffer atomicIndexBuffer;
			
			
			//Launches
			
			virtual void setIntersectDirectory();
			virtual void setOtherDirectory();
			
			
			virtual optix::Material createDefaultMeshMaterial(unsigned int ray_type_index, optix::Program closestHitProgram);

			virtual optix::Program createClosestHitMesh();

			virtual optix::Program createClosestHitReceiver();


			virtual optix::Program createBoundingBoxTriangle();
			virtual optix::Program createTriangleAttributesProgram();
			virtual void createIntersectionPrograms();
			virtual void createClosestHitPrograms();
			virtual optix::Program createIntersectionTriangle();
			virtual optix::Program createBoundingBoxSphere();
			virtual optix::Program createIntersectionSphere();

			virtual optix::Program createMissProgram();

			virtual optix::Program createRayGenerationProgram();
			
			virtual optix::Buffer setGlobalHitInfoBuffer();
			virtual optix::Buffer setAtomicIndexBuffer();
			optix::Buffer hitRays; // Buffer to store the rays that have hit in the launch to relaunch for log trace
			optix::Buffer traceBuffer; // Buffer to store the trace info
			optix::Buffer traceAtomicIndexBuffer; //Atomic index to store the hit index for traces	
			std::vector<float3> dirs; //Vector to store the ray directions to be log traced
			
			void createLogTracePrograms();
			//This launch generates traces for the rays that hit after being filtered to remove duplicates
			void executeLogRayTrace(optix::float3* rayDirs, uint hits, uint numTransmitters);
			virtual void saveTraceToFile(thrust::host_vector<LogTraceHitInfo> trace, std::string fileName);
		//Hits processing. HitInfo is passed here since at the moment most simulations use it. 
			
		//If different structs are used, processing can be done in the derived classes
			virtual void  processHits(HitInfo* host_hits, uint hits);
			virtual void  printHitInfo(HitInfo* host_hits, uint hits);
			virtual void processLaunch(HitInfo* host_hits, uint hits, uint numTransmitters); 
			
		//Compute power or any other quantity you need. Add your own
		//	virtual void computeReceivedPower(optix::float2 E, unsigned int index, int txId, float txPower, optix::float3 origin, uint raysHit);
		//	virtual void computeReceivedPower(optix::float2 Ex, optix::float2 Ey, optix::float2 Ez, unsigned int index, int txId, float txPower, optix::float3 origin, uint raysHit);
			virtual void computeReceivedPower(optix::float2 E, unsigned int index, unsigned int txIndex,  uint raysHit);
			virtual void computeReceivedPower(optix::float2 Ex, optix::float2 Ey, optix::float2 Ez, unsigned int index,unsigned int txIndex, uint raysHit);

			virtual void checkAndSetComputeMode(unsigned int mode);
			virtual void checkLaunchSize(unsigned int w, unsigned int h, unsigned int d);
		public:
			OpalSimulation(OpalSceneManager*  m);
			virtual ~OpalSimulation();
			virtual void init();
			virtual std::string printConfigInfo() const; 
			//virtual void setDefaultPrograms(std::map<std::string,optix::Program>& defaultPrograms, optix::Material& defaultMeshMaterial);
			virtual void setDefaultPrograms();
			virtual void setInternalBuffers();
			virtual void finishSceneContext();
			virtual void clearInternalBuffers();
			virtual std::string printInternalBuffersState();
			//std::map<std::string,optix::Program> getDefaultPrograms();
			//optix::Material getDefaultMeshMaterial() ;
			std::string getCudaProgramsDirectory() const;
			//To inform the simulation about new receivers
			virtual void addReceiver(int id, float3  position, float3 polarization, float radius, std::function<void(float, int)>  callback, std::vector<optix::Material>& materials);
			virtual void addStaticMesh(opal::OpalMesh& mesh, std::vector<optix::Material>& materials) ;
			virtual void addStaticCurvedMesh(opal::OpalMesh& mesh, std::vector<optix::Material>& materials) ;
			virtual void removeReceiver(int id);
			virtual void updateReceiver(int id, float3 position, float3 polarization, float radius);
			virtual ComputeMode getComputeMode() const { return mode;};	
			virtual void setComputeMode(ComputeMode m);	
			virtual OpalSimulationTypes getSimulationType() const;	
			virtual void registerReceiverGain(int rxId, int gainId); 
			virtual void transformEdge(Edge* e, optix::Matrix4x4 t);	
			virtual void setEnableSimulation(bool e);
			virtual void setEnableTraceLog(bool e);
			virtual bool acceptEdges() const;
			virtual bool acceptCurvedMesh() const;
			virtual bool usesPolarization() const;
			virtual bool usesRaySphere() const;
			virtual bool isEnabled() const;
			//Results
			void setPrintHits(bool p) ;		

			//Launch functions
			//Make abstract 		
			virtual void executeTransmitLaunch(uint numTransmitters, bool partial) = 0;
			virtual void endPartialLaunch(uint numTransmitters) = 0;
	
			

	};


}


#endif
