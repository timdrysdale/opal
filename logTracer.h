/***************************************************************/
//
//Copyright (c) 2019 Esteban Egea-Lopez http://girtel.upct.es/~eegea
//
/**************************************************************/

#ifndef LOGTRACER_H
#define LOGTRACER_H
#include "ptxUtil.h"
#include "Opal.h"

namespace opal {
	//Forward declarations
	class OpalSceneManager;
	class LogTracer {
		protected:
			OpalSceneManager* myManager;

			optix::Buffer hitRays; // Buffer to store the rays that have hit in the launch to relaunch for log trace
			optix::Buffer traceBuffer; // Buffer to store the trace info
			optix::Buffer traceAtomicIndexBuffer; //Atomic index to store the hit index for traces	
			std::vector<float3> dirs; //Vector to store the ray directions to be log traced
			
		public:
			LogTracer(OpalSceneManager*  m);
			void setInternalBuffers();
			void createLogTracePrograms(std::map<std::string,optix::Program>& defaultPrograms,std::string cudaProgramsDir, PtxUtil* ptxHandler );
			void executeLogRayTrace(optix::float3* rayDirs, uint hits, uint numTransmitters);
			//This launch generates traces for the rays that hit after being filtered to remove duplicates
			void saveTraceToFile(thrust::host_vector<LogTraceHitInfo> trace, std::string fileName);
	};
}
#endif

