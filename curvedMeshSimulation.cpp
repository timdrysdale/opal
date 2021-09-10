/***************************************************************/
//
//Copyright (c) 2020 Esteban Egea-Lopez http://girtel.upct.es/~eegea
//
/**************************************************************/

#include "curvedMeshSimulation.h"
#include "timer.h"
#include <algorithm>
namespace opal {
	LPCurvedMeshReflectionSimulation::LPCurvedMeshReflectionSimulation(OpalSceneManager* m) : OpalSimulation(m) {
		this->partialLaunchState = new opalthrustutils::PartialLaunchState<HitInfo>();
		this->optixProgramsDir ="optix";
		this->cudaProgramsDir=(m->getBaseDirectory()+"/"+optixProgramsDir);
		this->acceptCurved = true;
		this->useAngleDiscrimination = true;	
		this->printHits=false;
		this->withPolarization=true;
		this->cosAngleDuplicateRays;
		this->invalidRaysBuffer = nullptr;
		this->fractionMemGlobalBufferSize=0.4f;
		this->simType=OpalSimulationTypes::CURVEDWALLS;
	}
	LPCurvedMeshReflectionSimulation::~LPCurvedMeshReflectionSimulation() {
		if (partialLaunchState) {	
			delete this->partialLaunchState;
		}
		this->partialLaunchState=nullptr;
		this->invalidRaysBuffer=nullptr;
	}	
	void LPCurvedMeshReflectionSimulation::setOtherDirectory()  {
		//Curved meshes can only be used with general linear depolarization
		std::cout<<"Setting optix program directory to "<<(cudaProgramsDir+"/polarization/curved")<<std::endl;
		this->currentDir=(cudaProgramsDir+"/polarization/curved");
	}
	void LPCurvedMeshReflectionSimulation::disableAngleDiscrimination() {
		this->useAngleDiscrimination = false;
		std::cout<<"Angle discrimination disabled"<<std::endl;
	}
	void LPCurvedMeshReflectionSimulation::enableAngleDiscrimination() {
		this->useAngleDiscrimination = true;
		std::cout<<"Angle discrimination enabled"<<std::endl;
	}
	void LPCurvedMeshReflectionSimulation::setInternalBuffers() {
		OpalSimulation::setInternalBuffers();
		invalidRaysBuffer = myManager->getContext()->createBuffer(RT_BUFFER_INPUT_OUTPUT, RT_FORMAT_INT, 1u);
		myManager->getContext()["invalidRaysBuffer"]->set(invalidRaysBuffer);
	}
	//void LPCurvedMeshReflectionSimulation::setDefaultPrograms(std::map<std::string,optix::Program>& defaultPrograms, optix::Material& defaultMeshMaterial) 
//	void LPCurvedMeshReflectionSimulation::setDefaultPrograms()
//	{
//
//
//		//Create compiler options here
//		std::vector<const char *> nvccOptions;
//
//		nvccOptions.push_back("-arch");
//		nvccOptions.push_back("compute_30");
//		ConfigurationOptions options=myManager->getConfigurationOptions(); 
//		if (options.useFastMath) {
//			nvccOptions.push_back("-use_fast_math");
//		}
//		nvccOptions.push_back("-lineinfo");
//		nvccOptions.push_back("-default-device");
//		nvccOptions.push_back("-rdc");
//		nvccOptions.push_back("true");
//		nvccOptions.push_back("-D__x86_64");
//
//		ptxHandler = new PtxUtil(nvccOptions);
//
//
//		//TODO: we could add an intersection program for planes (ideal infinite planes), typically used to represent flat grounds, should be more efficient than a mesh?
//
//		//Intersection programs
//		setIntersectDirectory();
//#ifdef OPAL_USE_TRI
//		defaultPrograms.insert(std::pair<std::string, optix::Program>("triangleAttributes", createTriangleAttributesProgram()));
//		defaultPrograms.insert(std::pair<std::string, optix::Program>("triangleAttributesCurved", createCurvedTriangleAttributesProgram()));
//#else
//		defaultPrograms.insert(std::pair<std::string, optix::Program>("meshIntersection", createIntersectionTriangle()));
//		defaultPrograms.insert(std::pair<std::string, optix::Program>("meshBounds", createBoundingBoxTriangle()));
//		defaultPrograms.insert(std::pair<std::string, optix::Program>("meshIntersectionCurved", createIntersectionTriangleCurved()));
//#endif
//
//		defaultPrograms.insert(std::pair<std::string, optix::Program>("receiverIntersection", createIntersectionSphere()));
//		defaultPrograms.insert(std::pair<std::string, optix::Program>("receiverBounds", createBoundingBoxSphere()));
//
//
//		//Closest Hit Programs
//		setOtherDirectory();
//
//		defaultPrograms.insert(std::pair<std::string, optix::Program>("meshClosestHit", createClosestHitMesh()));
//		defaultPrograms.insert(std::pair<std::string, optix::Program>("meshClosestHitCurved", createClosestHitCurvedMesh()));
//		defaultPrograms.insert(std::pair<std::string, optix::Program>("receiverClosestHit", createClosestHitReceiver()));
//
//
//		defaultPrograms.insert(std::pair<std::string, optix::Program>("miss", createMissProgram()));
//		defaultPrograms.insert(std::pair<std::string, optix::Program>("rayGeneration", createRayGenerationProgram()));
//
//
//
//
//		//defaultMeshMaterial = createDefaultMeshMaterial(OPAL_RAY_REFLECTION,defaultPrograms.at("meshClosestHit")); //For normal rays
//		defaultMeshMaterial->setClosestHitProgram(OPAL_RAY_REFLECTION,defaultPrograms.at("meshClosestHit")); //For normal rays
//		//For visualization
//#ifdef OPAL_LOG_TRACE
//		createLogTracePrograms();
//		defaultMeshMaterial->setClosestHitProgram(OPAL_RAY_LOG_TRACE_RAY,defaultPrograms.at("closestHitTriangleLogTrace"));
//#endif
//
//
//
//
//	}
	void LPCurvedMeshReflectionSimulation::createIntersectionPrograms() {
		OpalSimulation::createIntersectionPrograms();
		std::map<std::string, optix::Program>& defaultPrograms=myManager->getDefaultPrograms();
	#ifdef OPAL_USE_TRI
		defaultPrograms["triangleAttributesCurved"]= createCurvedTriangleAttributesProgram();
	#else
		defaultPrograms["meshIntersectionCurved"]=createIntersectionTriangleCurved();
	#endif

	}
	void LPCurvedMeshReflectionSimulation::createClosestHitPrograms() {
		OpalSimulation::createClosestHitPrograms();
		std::map<std::string, optix::Program>& defaultPrograms=myManager->getDefaultPrograms();
		defaultPrograms.insert(std::pair<std::string, optix::Program>("meshClosestHitCurved", createClosestHitCurvedMesh()));
		
	}
	void LPCurvedMeshReflectionSimulation::addStaticCurvedMesh(OpalMesh& mesh, std::vector<optix::Material>& materials) {
		//materials.push_back(myManager->getDefaultCurvedMeshMaterial());
		//std::cout<<"LPCurvedMeshReflectionSimulation::addStaticCurvedMesh(): Setting closest hit for curved mesh"<<std::endl;
		std::map<std::string, optix::Program>& defaultPrograms=myManager->getDefaultPrograms();
		//The vector comes always with default mesh material at index 0
		materials[0]->setClosestHitProgram(reflectionRayIndex, defaultPrograms.at("meshClosestHitCurved"));
		if (generateTraceLog) {
			materials[0]->setClosestHitProgram(traceRayIndex,defaultPrograms.at("closestHitCurvedLogTrace"));
		}
	}
	std::string LPCurvedMeshReflectionSimulation::printConfigInfo() const  {
		std::ostringstream stream;
		stream<<"--- Simulation Type ---"<<std::endl;
		stream<<"\tUsing LPCurvedMeshReflectionSimulation with depolarization"<<std::endl;
		if (useAngleDiscrimination) {
			stream<<"\tUsing angle discrimination" <<std::endl;
		}
		stream << "-----" << std::endl;
		return stream.str();
	}
	std::string LPCurvedMeshReflectionSimulation::printInternalBuffersState() {
		std::string op=OpalSimulation::printInternalBuffersState();
		std::ostringstream stream;
		stream<<op;
		RTsize w;
		if (invalidRaysBuffer) {
			invalidRaysBuffer->getSize(w);
			stream <<"\t LPCurvedMeshReflectionSimulation invalidRaysBuffer=("<<w<<"). size="<<(w*sizeof(uint))<<" bytes"<<std::endl;
		}
		return stream.str();
	}
	optix::Program LPCurvedMeshReflectionSimulation::createClosestHitReceiver()
	{

		optix::Program chrx;
		std::cout<<"LPCurvedMeshReflectionSimulation::createClosestHitReceiver() called"<<std::endl;
		chrx = myManager->getContext()->createProgramFromPTXString(ptxHandler->getPtxString(currentDir.c_str(), "receiver.cu"), "closestHitReceiverCurved");


		//Program variables: common value for all receiver instances, since they all share the program. 
		//chrx["k"]->setFloat(myManager->getChannelParameters().k); //If multichannel is used, this should be set per transmission

		return chrx;
	}



	optix::Program LPCurvedMeshReflectionSimulation::createCurvedTriangleAttributesProgram()
	{
		return  myManager->getContext()->createProgramFromPTXString(ptxHandler->getPtxString(currentDir.c_str(),"optixGeometryTrianglesCurved.cu"), "triangle_attributes_curved" ) ;

	}
	optix::Program LPCurvedMeshReflectionSimulation::createIntersectionTriangleCurved()
	{
		return myManager->getContext()->createProgramFromPTXString(ptxHandler->getPtxString(currentDir.c_str(), "optixTriangleCurved.cu"), "intersectTriangleCurved");

	}



	//Default programs

	optix::Material LPCurvedMeshReflectionSimulation::createDefaultMeshMaterial(unsigned int ray_type_index, optix::Program closestHitProgram) {
		optix::Material mat= myManager->getContext()->createMaterial();
		mat->setClosestHitProgram(ray_type_index, closestHitProgram);

		return mat;
	}
	optix::Program LPCurvedMeshReflectionSimulation::createClosestHitCurvedMesh() {
		optix::Program chmesh = myManager->getContext()->createProgramFromPTXString(ptxHandler->getPtxString(currentDir.c_str(), "curved.cu"), "closestHitCurvedMesh");
		return chmesh;

	}
	optix::Program LPCurvedMeshReflectionSimulation::createClosestHitMesh() {
		//optix::Program chmesh = myManager->getContext()->createProgramFromPTXString(ptxHandler->getPtxString(currentDir.c_str(), "triangle.cu"), "closestHitTriangle");
		optix::Program chmesh = myManager->getContext()->createProgramFromPTXString(ptxHandler->getPtxString(currentDir.c_str(), "triangle.cu"), "closestHitFlatMesh");
		return chmesh;

	}

	void LPCurvedMeshReflectionSimulation::executeTransmitLaunch(uint numTransmitters,bool partial) {
		

		//Transmission launch
		Timer timer;
		timer.start();
		if (invalidRaysBuffer) {
			int* ir=reinterpret_cast<int*>(invalidRaysBuffer->map());
			ir[0]=0;
			invalidRaysBuffer->unmap();
		}
		RaySphere raySphere= myManager->getRaySphere();
		checkLaunchSize(raySphere.elevationSteps, raySphere.azimuthSteps, numTransmitters);
		myManager->getContext()->launch(reflectionEntryIndex, raySphere.elevationSteps, raySphere.azimuthSteps,numTransmitters); //Launch 3D (elevation, azimuth, transmitters);
		timer.stop();
		const double launchTime=timer.getTime();
		std::vector<int> enabledDevices= myManager->getEnabledDevices();
		if (partial) {
			timer.restart();

			if (useAngleDiscrimination) {	
				//We do not do anything, have to filter all the hits globally
			} else {
				//Filter with thrust multiple hits coming from the same face. Do not transfer the filtered vector
				uint hits=opalthrustutils::filterHitsMultiGPU(globalHitInfoBuffer,  atomicIndexBuffer, enabledDevices, partialLaunchState,maxGlobalBufferSize);
				partialLaunchState->setIndex(hits);
				uint vhits=partialLaunchState->getDeviceHitsBufferSize();
				std::cout<<"hits="<<hits<<"vector size="<<vhits<<std::endl;
				//Log times for performance tests
				timer.stop();
				uint numReceivers = myManager->getNumberOfReceivers();
				const double filterTime=timer.getTime();
				std::cout<<"#"<<numReceivers<<"\t"<<hits<<"\t"<<launchTime<<"\t"<<filterTime<<std::endl;
			}

		} else {
			timer.restart();

			if (useAngleDiscrimination) {	
				thrust::host_vector<HitInfo> host_hits=opalthrustutils::getAllHitsOrderedMultiGPU(globalHitInfoBuffer,  atomicIndexBuffer, enabledDevices,maxGlobalBufferSize );
				//std::vector<HitInfo> filtered=filterByAngle(host_hits.data(),host_hits.size());

				//processLaunch(filtered.data(), filtered.size(),numTransmitters);
				std::cout<<"Hits="<<host_hits.size()<<std::endl;
				processLaunch(host_hits.data(), host_hits.size(),numTransmitters);
			} else {

				//Filter with thrust multiple hits coming from the same face. Directly returns the filtered vector
				thrust::host_vector<HitInfo> host_hits=opalthrustutils::filterHitsAndTransfer(globalHitInfoBuffer,  atomicIndexBuffer, enabledDevices, maxGlobalBufferSize );

				//Log times for performance tests
				timer.stop();
				const double filterTime=timer.getTime();
				uint numReceivers = myManager->getNumberOfReceivers();
				uint hits=host_hits.size();
				std::cout<<"#"<<numReceivers<<"\t"<<host_hits.size()<<"\t"<<launchTime<<"\t"<<filterTime<<std::endl;
				processLaunch(host_hits.data(), hits,numTransmitters);

			}

		}
	}
	void LPCurvedMeshReflectionSimulation::endPartialLaunch(uint numTransmitters) {
		std::vector<int> enabledDevices= myManager->getEnabledDevices();
		if (useAngleDiscrimination) {
			uint hits=opalthrustutils::getAllHitsOrderedMultiGPU(globalHitInfoBuffer,  atomicIndexBuffer, enabledDevices, partialLaunchState,maxGlobalBufferSize);
			partialLaunchState->setIndex(hits);
			std::vector<HitInfo> hi=partialLaunchState->getHits();
			std::vector<HitInfo> filtered=filterByAngle(hi.data(),hi.size());
			//for (int i=0; i<filtered.size(); ++i) {
			//	for (int j=(i+1); j<filtered.size(); ++j) {
			//		std::cout<<i<<"j="<<j<<"a="<<(acosf(optix::dot(filtered[i].rayDir,filtered[j].rayDir))*180/M_PI)<<std::endl;

			//	}
			//}
			processLaunch(filtered.data(), filtered.size(), numTransmitters);
		} else {
			std::vector<HitInfo> hi=partialLaunchState->getHits();
			processLaunch(hi.data(), hi.size(), numTransmitters);
		}
		//partialLaunchState->reset();
		delete partialLaunchState;
		this->partialLaunchState = new opalthrustutils::PartialLaunchState<HitInfo>();


	}
	void LPCurvedMeshReflectionSimulation::setMaxAngleForDuplicateRays(float angle) { //angle in radians
		this->cosAngleDuplicateRays=cosf(angle); 
		std::cout<<"Changing angle discrimination to "<<(angle*180.0f/M_PI)<<std::endl;
	}
	std::vector<HitInfo> LPCurvedMeshReflectionSimulation::filterByAngle(HitInfo* host_hits, uint hits) {
		Timer timer;
	//	std::cout <<"Filtering by angle. Current hits="<<hits<<std::endl;
		std::vector<HitInfo> filtered;
		std::vector<HitInfo> v;
		//Statistics stats;	
		if (hits>0) {	
			timer.start();
			HitInfo current=host_hits[0];

			//std::cout <<"Filtering by angle. Current hits="<<hits<<std::endl;
			for (size_t i=0; i<hits; ++i) 
			{
				HitInfo h =host_hits[i];
				//std::cout<<i<<"\t"<<host_hits[i].E<<"\t"<<host_hits[i].thrd<<std::endl;

				//if (h.divergence.x==0) {
				//	stats.update(h.divergence.y);
				//} else {
				//	stats.update(h.divergence.x);
				//}
				if ((current.thrd.x==h.thrd.x)  && (current.thrd.y == h.thrd.y) && (current.thrd.z == h.thrd.z)  ) {
					v.push_back(h);
				} else {
					getUniqueByAngle(filtered,v);
					//New group
					current=h;
					v.clear();
					v.push_back(current);
					//stats.reset();
				}
			}
			//Last call
			getUniqueByAngle(filtered,v);
			timer.stop();
			std::cout <<" Filtered end size="<<filtered.size()<<". time="<<timer.getTime()<<std::endl;
			//for (int j=0; j<filtered.size();++j) {		
			//	//std::cout<<"\t"<<filtered[j].rayDir<<"\t"<<filtered[j].r<<"\t"<<filtered[j].thrd.y<<"\t"<<filtered[j].cosAngleDiscrimination<<"\t"<<filtered[j].thrd<<std::endl;
			//	if (j>0) {
			//		float dd=(acosf(optix::dot(filtered[j].rayDir,filtered[j-1].rayDir))/deg2rad);
			//		if (dd<=2.5f && filtered[j].thrd.y==filtered[j-1].thrd.y) {
			//			std::cout<<j<<"distance="<<dd<<std::endl;
			//		}
			//	}
			//}
		}
		return filtered;
	}
	void LPCurvedMeshReflectionSimulation::getUniqueByAngle(std::vector<HitInfo>& filtered, std::vector<HitInfo>& bucket) {
		//Rays already come ordered by distance so we start with the first
		HitInfo current=bucket[0];
	//	std::cout<<"Filtering for thrd="<<current.thrd<<std::endl;
		//std::cout <<"\t getUniqueByAngle bucket size="<<bucket.size()<<std::endl;
	//		for (HitInfo hi : bucket) {
	//				//std::cout<<std::scientific<<"\t"<<hi.rdud<<"\t"<<hi.thrd.y<<"\t"<<"\t"<<hi.thrd<<std::endl;
	//				std::cout<<std::scientific<<"\t"<<hi.divergence<<std::endl;
	//		}
		CheckAngle c;
		c.cosAngleDiscrimination=this->cosAngleDuplicateRays;
		//std::cout<<"Checking for angle="<<(acosf(c.cosAngleDiscrimination)/deg2rad)<<std::endl;
		//std::cout<<"Stats mean="<<stats.getMean()<<" std="<<stats.getStd()<<std::endl;
		auto pend=bucket.end();
		int i=0;
		auto b=bucket.begin();
		auto dev=bucket.begin();
		//std::cout<<"Remaining "<<(pend-b)<<" elements"<<std::endl;
		while (b<pend) {
		//	double currentDiv=0.0;
		//	if ((*b).divergence.x==0) {
		//		currentDiv=(*b).divergence.y;
		//	} else {
		//		currentDiv=(*b).divergence.x;
		//	}
		//	std::cout<<"Remaining "<<(pend-b)<<" elements"<<std::endl;
		//	double dif=abs(currentDiv-stats.getMean());
		//	double limit=1.5f*stats.getStd();
		//	std::cout<<"dif="<<dif<<"  limit="<<limit<<std::endl;
		//	while (dif>limit) {
		//		++b;
		//		if (b==bucket.end()) {
		//			//No one found
		//			b=dev;
		//			break;
		//		}

		//		if ((*b).divergence.x==0) {
		//			currentDiv=(*b).divergence.y;
		//		} else {
		//			currentDiv=(*b).divergence.x;
		//		}
		//		dif=abs(currentDiv-stats.getMean());
		//		std::cout<<"dif="<<dif<<std::endl;
		//	}
		//	std::cout<<"Removing "<<(b-dev)<<" outliers"<<std::endl;
		//	bucket.erase(dev,b);

			c.current=(*b);
			auto f = std::bind(&CheckAngle::IsInAngle, &c,std::placeholders::_1);
			auto aux=std::remove_if(++b, pend,f);
			pend=aux;
			dev=b;
			//std::cout<<"\t"<<i<<" left "<<(pend-b)<<std::endl;
			filtered.push_back((c.current));
			//std::cout<<c.current.Ex<<"\t"<<c.current.rdud<<std::endl;
			++i;

		}
		//std::cout <<"\t getUniqueByAngle. Filtered size="<<filtered.size()<<std::endl;

	}
//#ifdef OPAL_EXTENDED_HITINFO
//void LPCurvedMeshReflectionSimulation::processHitsExtended(HitInfo* host_hits, uint hits) {
//
//	std::cout<<"processHitsExtended hits="<<hits<<std::endl;
//	//** Debug
//	float2 Ex=make_float2(0.0f,0.0f);
//	float2 Ey=make_float2(0.0f,0.0f);
//	float2 Ez=make_float2(0.0f,0.0f);
//	//****
//	
//	uint index=0u;
//	uint raysHit=0u;
//	int j=0;
//	uint currentTx=0u;
//	std::vector<Transmitter*> activeTransmitters = myManager->getActiveTransmitters();
//	for (uint i=0; i<hits; i++) {
//		if (i==0) {
//			//Get first transmitter 			
//			currentTx=host_hits->thrd.x;
//			//Get first receiver
//			index=host_hits->thrd.z;
//
//		} else {
//			if (host_hits->thrd.x!=currentTx) {
//				if (raysHit!=0) {
//					computeReceivedPower(Ex,Ey,Ez,index,activeTransmitters[currentTx]->externalId,activeTransmitters[currentTx]->txPower,activeTransmitters[currentTx]->origin, raysHit); 			
//				} 				
//				//New transmitter 				
//				currentTx=host_hits->thrd.x; 				
//				//New receiver,  start new accumulation 				
//				index=host_hits->thrd.z; 				
//				Ex=make_float2(0.0f,0.0f);
//				Ey=make_float2(0.0f,0.0f);
//				Ez=make_float2(0.0f,0.0f);
//				raysHit=0u; 				
//				//std::cout<<"New transmitter tx="<<currentTx<<";rx="<<index<<std::endl; 			
//			} else {
//				if (host_hits->thrd.z!=index) {
//					if (raysHit!=0u) {
//						//At least one hit, callback
//						computeReceivedPower(Ex,Ey,Ez,index,activeTransmitters[currentTx]->externalId,activeTransmitters[currentTx]->txPower,activeTransmitters[currentTx]->origin, raysHit);
//					}
//					//New receiver, start new accumulation
//					index=host_hits->thrd.z;
//					//** Debug
//					Ex=make_float2(0.0f,0.0f);
//					Ey=make_float2(0.0f,0.0f);
//					Ez=make_float2(0.0f,0.0f);
//					//****
//					raysHit=0u;
//					j=0;
//
//				}
//			}
//		}
//
//		++j;
//		++raysHit;
//		//** Debug
//		Ex += host_hits->Ex;
//		Ey += host_hits->Ey;
//		Ez += host_hits->Ez;
//		//****
//
//
//	// Log hits received
//		if (printHits) {
//			//		std::cout<<"E["<<i<<"]="<<(host_hits)->E<<std::endl;
//
//			std::cout<<i<<"\t"<<j<<"\tEx="<<(host_hits)->Ex<<std::endl;
//			std::cout<<i<<"\t"<<j<<"\tEy="<<(host_hits)->Ey<<std::endl;
//			std::cout<<i<<"\t"<<j<<"\tEz="<<(host_hits)->Ez<<std::endl;
//			std::cout<<i<<"\t"<<j<<"\t [rayDir d]="<<(host_hits)->rdud<<std::endl;
//			std::cout<<i<<"\t"<<j<<"\t [tx hash rx flag]="<<(host_hits)->thrd<<std::endl;
//			//		//std::cout<<"\t rxBufferIndex="<<(host_hits)->thrd.z<<std::endl;
//			//		//std::cout<<"\t txBufferIndex="<<(host_hits)->thrd.x<<std::endl;
//			//std::cout<<i<<"\t"<<j<<"\t refhash="<<(host_hits)->thrd.y<<std::endl;
//			//std::cout<<i<<"\t"<<j<<"\t dist="<<(host_hits)->thrd.w<<std::endl;
//			//////		//Used only for debug. Uncomment in Common.h
//			////		std::cout<<"\t lastHP="<<(host_hits)->lh<<std::endl;
//			std::cout<<i<<"\t"<<j<<"\t reflections="<<(host_hits)->r<<std::endl;
//			std::cout<<i<<"\t"<<j<<"\t divergence="<<(host_hits)->divergence<<std::endl;
//			//std::cout<<i<<"\t"<<j<<"\t updateDivergence="<<(host_hits)->updateDivergence<<std::endl;
//			////		std::cout<<"\t index="<<(host_hits)->in<<std::endl;
//			////		std::cout<<"\t Rnorm="<<(host_hits)->Rn<<std::endl;
//			////		std::cout<<"\t Rpar="<<(host_hits)->Rp<<std::endl;
//			////		std::cout<<"\t h="<<(host_hits)->h<<std::endl;
//			////		std::cout<<"\t v="<<(host_hits)->v<<std::endl;
//		}
//		++host_hits;
//
//	}
//	//Last one
//	if (raysHit!=0u) {
//		computeReceivedPower(Ex,Ey,Ez,index,activeTransmitters[currentTx]->externalId,activeTransmitters[currentTx]->txPower,activeTransmitters[currentTx]->origin, raysHit); 
//	}	
//	//timer.stop();
//}
//#else
//void LPCurvedMeshReflectionSimulation::processHits(HitInfo* host_hits,uint hits) {
//	float2 E=make_float2(0.0f,0.0f);
//	uint index=0u;
//	uint raysHit=0u;
//
//	uint currentTx=0u;
//	std::vector<Transmitter*> activeTransmitters = myManager->getActiveTransmitters();
//	for (uint i=0; i<hits; i++) {
//		if (i==0) {
//			//Get first transmitter 			
//			currentTx=host_hits->thrd.x;
//
//			//Get first receiver
//			index=host_hits->thrd.z;
//
//		} else {
//
//			if (host_hits->thrd.x!=currentTx) {
//				if (raysHit!=0) {
//					computeReceivedPower(E,index,activeTransmitters[currentTx]->externalId,activeTransmitters[currentTx]->txPower,activeTransmitters[currentTx]->origin, raysHit); 			
//				} 				
//				//New transmitter 				
//				currentTx=host_hits->thrd.x; 				
//				//New receiver,  start new accumulation 				
//				index=host_hits->thrd.z; 				
//				E=make_float2(0.0f,0.0f); 				
//				raysHit=0u; 				
//				//std::cout<<"New transmitter tx="<<currentTx<<";rx="<<index<<std::endl; 			
//			} else { 				
//				if (host_hits->thrd.z!=index) { 					
//					if (raysHit!=0u) { 						
//						//At least one hit, callback 						
//						computeReceivedPower(E,index,activeTransmitters[currentTx]->externalId,activeTransmitters[currentTx]->txPower,activeTransmitters[currentTx]->origin, raysHit);
//					} 			
//					//New receiver, same transmitter, start new accumulation
//					//std::cout<<"New receiver tx="<<currentTx<<";rx="<<index<<std::endl; 					
//					index=host_hits->thrd.z;
//					E=make_float2(0.0f,0.0f);
//					raysHit=0u;
//				} 			
//			}
//
//
//		}
//
//		++raysHit;
//		E += host_hits->E;
//
//	// Log hits received
//		if (printHits){
//			std::cout<<"E["<<i<<"]="<<(host_hits)->E<<std::endl;
//			std::cout<<"\t [txi,hash,rxi,flag]="<<(host_hits)->thrd<<std::endl;
//			std::cout<<"\t divergence="<<(host_hits)->divergence<<std::endl;
//			std::cout<<"\t rhos="<<(host_hits)->rhos<<std::endl;
//			//	std::cout<<"\t rxBufferIndex="<<(host_hits)->thrd.z<<std::endl;
//			//	std::cout<<"\t written="<<(host_hits)->thrd.x<<std::endl;
//			//	std::cout<<"\t refhash="<<(host_hits)->thrd.y<<std::endl;
//			//	std::cout<<"\t dist="<<(host_hits)->thrd.w<<std::endl;
//			//std::cout<<"\t R="<<(host_hits)->R<<std::endl;
//			std::cout<<"\t ud="<<(host_hits)->rdud.w<<std::endl; //Unfolded distance
//			std::cout<<"\t rayDir="<<(host_hits)->rdud<<std::endl; //Unfolded distance
//
//		}
//
//		++host_hits;
//
//	}
//	//Last one
//	if (raysHit!=0u) {
//		computeReceivedPower(E,index,activeTransmitters[currentTx]->externalId,activeTransmitters[currentTx]->txPower,activeTransmitters[currentTx]->origin, raysHit); 
//	}	
//}
//#endif

}
