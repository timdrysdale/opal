/***************************************************************/
//
//Copyright (c) 2020 Esteban Egea-Lopez http://girtel.upct.es/~eegea
//
/**************************************************************/

#ifndef RAYDENSITYNORMALIZATION_H
#define RAYDENSITYNORMALIZATION_H
#include "curvedMeshSimulation.h"
#include <map>
namespace opal {
	//Simulation for scenarios with  curved surfaces and  flat elements (walls) and arbitrary linear polarization with Ray Density Normalization
	
	enum RDNExecutionMode{
		HITINFO, //Use HitInfo buffer
		NOATOMIC, //Fixed size ray buffer with no call to atomic functions
		NOMEM //No buffers with call to atomic functions
	};	
	class  RayDensityNormalizationSimulation: public LPCurvedMeshReflectionSimulation {
		protected:
			//Histogram* angles;
			//Histogram* modulus;
			//Histogram* reflections;
			
			optix::Buffer parametersBuffer;
			optix::Buffer receivedFieldBuffer;
			optix::Buffer fixedRayBuffer;



			bool sorted;
			bool useFieldTrace;
			RDNExecutionMode execMode;
			unsigned int filtering; 
			float initialDensity;
			unsigned int rayCount;
			unsigned int lastReceivedFieldBufferSize;
			unsigned int fixedRaySize; //Number of rays is fixedRaySize*fixedRaySize;
			//std::vector<HitInfo> rayDensityNormalization(HitInfo* host_hits, uint hits); 
			virtual void setOtherDirectory() override;
			virtual void setReceivedFieldBuffer();
			virtual void initReceivedFieldBuffer();
			//Hits processing
//#ifdef OPAL_EXTENDED_HITINFO
//			virtual void  processHitsExtendedUnordered(HitInfo* h, uint hits);
			//virtual void  processHitsExtended(HitInfo* host_hits, uint hits);
//#else	
//			virtual void  processHitsUnordered(HitInfo* h, uint hits);
			virtual void processReceivedFieldBuffer();
			virtual void processFixedRayBuffer();
			virtual void initFixedRayBuffer();
			virtual void  processHits(HitInfo* host_hits, uint hits) override;
			virtual void processLaunch(HitInfo* host_hits, uint hits, uint numTransmitters) override;
			virtual void  printHitInfo(HitInfo* host_hits, uint hits) override;
//#endif
			virtual optix::Program createClosestHitReceiver() override;
			void executeFixedRayLaunch(unsigned int numTransmitters);	
			void executeReceivedFieldLaunch(unsigned int numTransmitters);
			void executeHitInfoLaunch(unsigned int numTransmitters);	
		public:
			RayDensityNormalizationSimulation(OpalSceneManager*  m);
			RayDensityNormalizationSimulation(OpalSceneManager*  m, RDNExecutionMode execMode);
			virtual ~RayDensityNormalizationSimulation();
			virtual std::string printConfigInfo() const override; 
			virtual void executeTransmitLaunch(uint numTransmitters, bool partial) override;
			virtual void endPartialLaunch(uint numTransmitters) override;
			virtual float setInitialDensity(long rayCount,float initAzimuth, float endAzimuth, float initElevation, float endElevation);
			virtual void setInitialDensity(float f);
			virtual void setRayCount(unsigned int c);
			virtual void setFiltering(unsigned int f);
			virtual void setUseFieldTrace(bool f);
			virtual unsigned int getFiltering() const { return filtering;};	
			virtual void setInternalBuffers() override;
			virtual std::string printInternalBuffersState() override;
			float sphereRadiusForExpectedDirectHits(uint expectedHits, float initialDensity, float distance);
			virtual void setFixedRayBuffer(unsigned int rx, unsigned int raySize);
			virtual void setExecutionMethod(RDNExecutionMode mode);
	};


}


#endif

