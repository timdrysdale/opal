/***************************************************************/
//
//Copyright (c) 2019 Esteban Egea-Lopez http://girtel.upct.es/~eegea
//
/**************************************************************/

#include "logTracer.h"
namespace opal {
	LogTracer::LogTracer(OpalSceneManager* m) {
		this->myManager=m;
	}
	void LogTracer::setInternalBuffers() {
		//Create ray direction buffer. Set it to one at the moment because we do not know the number of hits: resized later
		hitRays = myManager->getContext()->createBuffer(RT_BUFFER_INPUT, RT_FORMAT_FLOAT3, 1u);
		myManager->getContext()["hitRays"]->set(hitRays);
		traceBuffer = myManager->getContext()->createBuffer(RT_BUFFER_INPUT_OUTPUT | RT_BUFFER_GPU_LOCAL, RT_FORMAT_USER, 1u);
		traceBuffer->setElementSize(sizeof(LogTraceHitInfo));
		myManager->getContext()["traceBuffer"]->set(traceBuffer);
		traceAtomicIndexBuffer = myManager->getContext()->createBuffer(RT_BUFFER_INPUT_OUTPUT | RT_BUFFER_GPU_LOCAL, RT_FORMAT_UNSIGNED_INT, 1u);
		myManager->getContext()["traceAtomicIndex"]->set(traceAtomicIndexBuffer);
	}
	void LogTracer::createLogTracePrograms(std::map<std::string,optix::Program>& defaultPrograms,std::string cudaProgramsDir, PtxUtil*  ptxHandler) {
		std::string logDir=(cudaProgramsDir+ "/log");
		std::cout<<"Creating log trace programs from " <<logDir <<std::endl;
		optix::Context context=myManager->getContext();
		optix::Program prog = context->createProgramFromPTXString(ptxHandler->getPtxString(logDir.c_str(), "triangle.cu"), "closestHitTriangleLogTrace");
		if (prog->get()==nullptr) {
			std::cout<<"null program at  " <<logDir <<std::endl;

		} else {
			prog->validate();
		}
		defaultPrograms.insert(std::pair<std::string, optix::Program>("closestHitTriangleLogTrace",prog) );
		prog = context->createProgramFromPTXString(ptxHandler->getPtxString(logDir.c_str(), "curved.cu"), "closestHitCurvedLogTrace");
		if (prog->get()==nullptr) {
			std::cout<<"null program at  " <<logDir <<std::endl;

		} else {
			prog->validate();
		}
		defaultPrograms.insert(std::pair<std::string, optix::Program>("closestHitCurvedLogTrace",prog) );
		prog = context->createProgramFromPTXString(ptxHandler->getPtxString(logDir.c_str(), "receiver.cu"), "closestHitReceiverLogTrace");
		if (prog->get()==nullptr) {
			std::cout<<"null program at  " <<logDir <<std::endl;

		} else {
			prog->validate();
		}
		defaultPrograms.insert(std::pair<std::string, optix::Program>("closestHitReceiverLogTrace", prog));
		prog = context->createProgramFromPTXString(ptxHandler->getPtxString(logDir.c_str(), "receiver.cu"), "missLogTrace");
		if (prog->get()==nullptr) {
			std::cout<<"null program at  " <<logDir <<std::endl;

		} else {
			prog->validate();
		}
		defaultPrograms.insert(std::pair<std::string, optix::Program>("missLogTrace", prog));
		prog = context->createProgramFromPTXString(ptxHandler->getPtxString(logDir.c_str(), "generation.cu"), "genRayTracesFromHits");
		if (prog->get()==nullptr) {
			std::cout<<"null program at  " <<logDir <<std::endl;

		} else {
			prog->validate();
		}
		defaultPrograms.insert(std::pair<std::string, optix::Program>("rayGenerationLogTrace", prog));

	}
	void LogTracer::executeLogRayTrace(optix::float3* rayDirs, uint hits, uint numTransmitters) {
		//This launch generates traces for the rays that hit after being filtered to remove duplicates
		if (hits==0) {
			std::cout<<" No hits. Not executing Log Ray Trace for "<<hits<<" hits"<<std::endl;
			return;

		}
		std::cout<<"Executing Log Ray Trace for "<<hits<<" hits"<<std::endl;

		std::cout<<"PL****"<<std::endl;
		//Fill ray direction buffer
		hitRays->setSize(hits);
		optix::float3* rays_host = reinterpret_cast<optix::float3*>  (hitRays->map());

		for (size_t i=0; i<hits; ++i) 
		{
			rays_host[i]=rayDirs[i];
			//std::cout<<i<<"\t"<<rayDirs[i]<<std::endl;

		}
		hitRays->unmap();

		//Set trace buffer
		uint maxTraceBufferSize=(2*hits)*(myManager->getMaxReflections()+1);
		traceBuffer->setSize(maxTraceBufferSize);
		std::cout<<"traceBufferSize="<<maxTraceBufferSize<<std::endl;	
		uint* ai= reinterpret_cast<uint*>(traceAtomicIndexBuffer->map());
		ai[0]=0u;
		traceAtomicIndexBuffer->unmap();	
		//Launch
		myManager->getContext()->launch(OPAL_RAY_LOG_TRACE_RAY,hits,numTransmitters); //Launch 2D (hits, transmitters);
		std::vector<int> enabledDevices= myManager->getEnabledDevices();
		thrust::host_vector<LogTraceHitInfo> trace=opalthrustutils::getLogTraceOrderer(traceBuffer,traceAtomicIndexBuffer,enabledDevices, maxTraceBufferSize);
		if (trace.size()>0) {
			saveTraceToFile(trace,"trace.txt");
		}	
		std::cout<<"PL****"<<std::endl;
		//Remove buffer
		//hitRays->destroy();
		traceBuffer->setSize(1u);
	}

	void LogTracer::saveTraceToFile(thrust::host_vector<LogTraceHitInfo> trace, std::string fileName) {
		std::cout<<"Saving log trace to "<<fileName<<std::endl;
		std::ofstream file(fileName.c_str(),std::ofstream::out);
		uint currentIndex=trace[0].ray_ref.x;
		file<<currentIndex<<":"<<trace[0].hitp.x<<"\t"<<trace[0].hitp.y<<"\t"<<trace[0].hitp.z;
		for (int i=1; i<trace.size(); i++) {
			LogTraceHitInfo l=trace[i];
			if (l.ray_ref.x != currentIndex) {
				currentIndex=l.ray_ref.x;
				file<<std::endl;
				file<<currentIndex<<":"<<l.hitp.x<<"\t"<<l.hitp.y<<"\t"<<l.hitp.z;
			} else {
				file<<"|"<<l.hitp.x<<"\t"<<l.hitp.y<<"\t"<<l.hitp.z;	
			}
		}
		file.close();

	}
}
