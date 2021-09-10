/***************************************************************/
//
//Copyright (c) 2020 Esteban Egea-Lopez http://girtel.upct.es/~eegea
//
/**************************************************************/

#include "rayDensityNormalizationSimulation.h"
#include "timer.h"
#include <iomanip>
#include <limits>
namespace opal {

	RayDensityNormalizationSimulation::RayDensityNormalizationSimulation(OpalSceneManager* m) : LPCurvedMeshReflectionSimulation(m) {
		this->sorted=false;
		this->filtering=2;
		this->useFieldTrace =false;
		this->fixedRaySize=1000u;
		this->simType=OpalSimulationTypes::RDN;
		this->execMode=RDNExecutionMode::NOMEM;
		//this->execMode=RDNExecutionMode::NOATOMIC;
		//this->execMode=RDNExecutionMode::HITINFO;
		this->initialDensity=0;
		this->rayCount =0;
		this->parametersBuffer=nullptr;
		this->fractionMemGlobalBufferSize=0.7f;
		this->receivedFieldBuffer=nullptr;	
		this->fixedRayBuffer=nullptr;	
		this->lastReceivedFieldBufferSize=0u;
		//this->angles = new Histogram(100, -M_PIf,M_PIf);
		//this->modulus = new Histogram(100,1e-8,1e-5);
		//this->reflections= new Histogram(40,0,40);

	}
	RayDensityNormalizationSimulation::RayDensityNormalizationSimulation(OpalSceneManager* m, RDNExecutionMode execMode) : LPCurvedMeshReflectionSimulation(m) {
		this->sorted=false;
		this->filtering=2;
		this->useFieldTrace =false;
		this->fixedRaySize=1000u;
		this->simType=OpalSimulationTypes::RDN;
		this->execMode=execMode;
		//this->execMode=RDNExecutionMode::NOATOMIC;
		//this->execMode=RDNExecutionMode::HITINFO;
		this->initialDensity=0;
		this->rayCount =0;
		this->parametersBuffer=nullptr;
		this->fractionMemGlobalBufferSize=0.7f;
		this->receivedFieldBuffer=nullptr;	
		this->fixedRayBuffer=nullptr;	
		this->lastReceivedFieldBufferSize=0u;
		//this->angles = new Histogram(100, -M_PIf,M_PIf);
		//this->modulus = new Histogram(100,1e-8,1e-5);
		//this->reflections= new Histogram(40,0,40);

	}
	RayDensityNormalizationSimulation::~RayDensityNormalizationSimulation() {
		if (partialLaunchState) {	
			delete this->partialLaunchState;
		}
		partialLaunchState=nullptr;
		parametersBuffer=nullptr;
		receivedFieldBuffer=nullptr;
		fixedRayBuffer=nullptr;	
	}	
	void RayDensityNormalizationSimulation::setInitialDensity(float f) {
		this->initialDensity=f;
	}
	void RayDensityNormalizationSimulation::setRayCount(unsigned int c) {
		this->rayCount=c;
	}
	void RayDensityNormalizationSimulation::setFiltering(unsigned int f) {
		this->filtering=f;
	}	
	void RayDensityNormalizationSimulation::setUseFieldTrace(bool f) {
		this->useFieldTrace=f;
	}	
	void RayDensityNormalizationSimulation::setExecutionMethod(RDNExecutionMode mode) {
		this->execMode=mode;
	}
	void RayDensityNormalizationSimulation::setInternalBuffers() 
	{
		switch(execMode) {
			case HITINFO:
				LPCurvedMeshReflectionSimulation::setInternalBuffers();
				break;
			case NOATOMIC:
				//Set it here with 1 rx to avoid errors in validate in initContext
				if (fixedRayBuffer) {
				} else {
					setFixedRayBuffer(1u, fixedRaySize);
				}
				break;
			case NOMEM:
				setReceivedFieldBuffer();
				break;
			default:
				throw  opal::Exception("RDN::setInternalBuffers(): unkown execution mode");
		}
		parametersBuffer = myManager->getContext()->createBuffer(RT_BUFFER_INPUT, RT_FORMAT_USER, 1u);
		parametersBuffer->setElementSize(sizeof(RDNParameters));
		myManager->getContext()["rdnParametersBuffer"]->set(parametersBuffer);

	}
	void RayDensityNormalizationSimulation::setReceivedFieldBuffer() {
		if (execMode!=NOMEM) {
			throw  opal::Exception("RDN::setReceiveFieldBuffer() can only be used with NOMEM");
		}	
		if (receivedFieldBuffer) {
			receivedFieldBuffer->destroy();
			receivedFieldBuffer=nullptr;
		}
		unsigned int nrx=myManager->getNumberOfReceivers();
		std::cout<<"Setting receivedFieldBuffer for "<<nrx<<std::endl;
		receivedFieldBuffer = myManager->getContext()->createBuffer(RT_BUFFER_INPUT_OUTPUT | RT_BUFFER_GPU_LOCAL, RT_FORMAT_USER, nrx);
		receivedFieldBuffer->setElementSize(sizeof(RDNHit));
		myManager->getContext()["eBuffer"]->set(receivedFieldBuffer);
	}
	void RayDensityNormalizationSimulation::setFixedRayBuffer(unsigned int nrx, unsigned int raySize) {
		if (execMode!=NOATOMIC) {
			throw  opal::Exception("RDN::setFixedRayBuffer() can only be used with NOATOMIC");
		}	
		if (fixedRayBuffer) {
			fixedRayBuffer->destroy();
			fixedRayBuffer=nullptr;
		}
		fixedRayBuffer = myManager->getContext()->createBuffer(RT_BUFFER_OUTPUT, RT_FORMAT_USER, nrx,raySize,raySize);
		fixedRayBuffer->setElementSize(sizeof(RDNHit));
		std::cout<<"size of fixed Ray buffer (GB)"<<((float)(sizeof(RDNHit)*nrx*raySize*raySize)/(1024*1024*1024))<<std::endl;
		myManager->getContext()["frBuffer"]->set(fixedRayBuffer);
	}
	void RayDensityNormalizationSimulation::initReceivedFieldBuffer() {
		if (receivedFieldBuffer) {
			RTsize w;
			receivedFieldBuffer->getSize(w);
			float2 E=make_float2(0.0f,0.0f);
			float2* pb=reinterpret_cast<float2*>(receivedFieldBuffer->map());
			for (unsigned int i=0; i<w; ++i) {
				pb[i]=E;
			}
			receivedFieldBuffer->unmap();
		} else {
			throw  opal::Exception("RDN::initReceivedFieldBuffer(): trying to initialize receivedFieldBuffer but it is null");
		}
	} 
	void RayDensityNormalizationSimulation::setOtherDirectory()  {
		//Curved meshes can only be used with general linear depolarization
		std::cout<<"Setting optix program directory to "<<(cudaProgramsDir+"/polarization/rdn")<<std::endl;
		this->currentDir=(cudaProgramsDir+"/polarization/rdn");
	}
	std::string RayDensityNormalizationSimulation::printConfigInfo() const  {
		std::ostringstream stream;
		stream<<"--- Simulation Type ---"<<std::endl;
		stream<<"\tUsing RayDensityNormalizationSimulation with depolarization"<<std::endl;
		switch(execMode) {
			case HITINFO:
				stream<<"\t Using HITINFO"<<std::endl;
				break;
			case NOATOMIC:
				stream<<"\t Using NOATOMIC"<<std::endl;
				break;
			case NOMEM:
				stream<<"\t Using NOMEM"<<std::endl;
				break;
			default:
				stream<<"\t Unknown execution method"<<std::endl;
		}

		if (useFieldTrace) { 
			stream<<"\tUsing field trace"<<std::endl;
		} else {
			stream<<"\tUsing  power trace"<<std::endl;
		}
		stream << "-----" << std::endl;
		return stream.str();
	}
	optix::Program RayDensityNormalizationSimulation::createClosestHitReceiver()
	{

		optix::Program chrx;
		if (useFieldTrace) {
			if (execMode == RDNExecutionMode::HITINFO) {
				chrx = myManager->getContext()->createProgramFromPTXString(ptxHandler->getPtxString(currentDir.c_str(), "receiver.cu"), "closestHitReceiverCurved");
			} else {
				throw  opal::Exception("RDN::createClosestHitReceiver(): fieldTrace can only be used with HITINFO execution mode");
			}
		} else {
			switch(execMode) {
				case HITINFO:
					std::cout<<"Using power.cu"<<std::endl;
					chrx = myManager->getContext()->createProgramFromPTXString(ptxHandler->getPtxString(currentDir.c_str(), "power.cu"), "closestHitReceiverCurved");
					break;
				case NOATOMIC:
					std::cout<<"Using power_mem_noa.cu"<<std::endl;
					chrx = myManager->getContext()->createProgramFromPTXString(ptxHandler->getPtxString(currentDir.c_str(), "power_mem_noa.cu"), "closestHitReceiverCurved");
					break;
				case NOMEM:
					std::cout<<"Using power_mem.cu"<<std::endl;
					chrx = myManager->getContext()->createProgramFromPTXString(ptxHandler->getPtxString(currentDir.c_str(), "power_mem.cu"), "closestHitReceiverCurved");
					break;
				default:
					throw  opal::Exception("RDN::createClosestHitReceiver(): unkown execution mode");
			}
		}

		//Program variables: common value for all receiver instances, since they all share the program. 
		//chrx["k"]->setFloat(myManager->getChannelParameters().k); //If multichannel is used, this should be set per transmission

		return chrx;
	}
	void RayDensityNormalizationSimulation::executeTransmitLaunch(uint numTransmitters,bool partial) {

		//Transmission launch
		Timer timer;
		RaySphere raySphere= myManager->getRaySphere();
		checkLaunchSize(raySphere.elevationSteps, raySphere.azimuthSteps, numTransmitters);
		// pass as buffer to increase performance
		RDNParameters par;
		par.initialDensity=this->initialDensity;
		par.filter_rc_nrx =make_uint4(static_cast<unsigned int>(this->filtering),this->rayCount, myManager->getNumberOfReceivers(),0u); 
		RDNParameters* pb=reinterpret_cast<RDNParameters*>(parametersBuffer->map());
		pb[0]=par;
		parametersBuffer->unmap();
		std::cout<<"Setting initial density to "<<this->initialDensity<<std::endl;
		std::cout<<"Setting filtering density to ";
		if (filtering ==0) {
			std::cout<<"no filtering"<<std::endl;
		} else if (filtering==1) {
			std::cout<<"filtering low density"<<std::endl;

		} else if (filtering==2) {
			std::cout<<"not dividing low density"<<std::endl;
		} else {
			throw  opal::Exception("RDN::launch(): unknown filtering density value");

		}
		switch(execMode) {
			case HITINFO:
				executeHitInfoLaunch(numTransmitters);
				break;
			case NOATOMIC:
				if (numTransmitters!=1) {
					throw  opal::Exception("RDN::executeTransmitLaunch() NOATOMIC can only be used with one transmitter");
				}
				executeFixedRayLaunch(numTransmitters);
				break;
			case NOMEM:
				if (numTransmitters != 1) {
					throw  opal::Exception("RayDensityNormalizationSimulation::executeReceivedFieldLaunch(): NOMEM type not implemented for more than 1 transmitter yet");
				}	
				executeReceivedFieldLaunch(numTransmitters);
				break;
			default:
				throw  opal::Exception("RDN::setInternalBuffers(): unkown execution mode");
		}
		if (!partial) {
			//myManager->computeTotalReceivedPower();	
			//std::cout<<"HA"<<angles->print()<<std::endl;	
			//std::cout<<"HM"<<modulus->print()<<std::endl;	
			//std::cout<<"HR"<<reflections->print()<<std::endl;	
			//angles->reset();
			//modulus->reset();
			//reflections->reset();
			//info->reset();
			//#ifdef OPAL_LOG_TRACE
			//			executeLogRayTrace(dirs.data(), dirs.size(),numTransmitters);
			//#endif //OPAL_LOG_TRACE
			//
		}
	}

	void RayDensityNormalizationSimulation::executeHitInfoLaunch(unsigned int numTransmitters) {
		RaySphere raySphere= myManager->getRaySphere();
		if (invalidRaysBuffer) {
			int* ir=reinterpret_cast<int*>(invalidRaysBuffer->map());
			ir[0]=0;
			invalidRaysBuffer->unmap();
		}
		std::cout<<"Transmitting RDN HitInfo with el="<<raySphere.elevationSteps<<"az="<<raySphere.azimuthSteps<<std::endl;
		myManager->getContext()->launch(reflectionEntryIndex, raySphere.elevationSteps, raySphere.azimuthSteps,numTransmitters); //Launch 3D (elevation, azimuth, transmitters);


		if (invalidRaysBuffer) {
			int* ir=reinterpret_cast<int*>(invalidRaysBuffer->map());
			int invalid=ir[0];
			double  fraction=((double)invalid)/(raySphere.rayCount);
			std::cout<<"Invalid rays="<<invalid<<"fraction="<<fraction<<std::endl;

			invalidRaysBuffer->unmap();
		}
		std::vector<int> enabledDevices= myManager->getEnabledDevices();
		thrust::host_vector<HitInfo> host_hits=opalthrustutils::copyHitsToHostMultiGPU(globalHitInfoBuffer,  atomicIndexBuffer, enabledDevices,maxGlobalBufferSize);
		//std::cout<<"Hits="<<host_hits.size()<<std::endl;
		processLaunch(host_hits.data(), host_hits.size(),numTransmitters);
	}
	void RayDensityNormalizationSimulation::executeReceivedFieldLaunch(unsigned int numTransmitters) {
		RaySphere raySphere= myManager->getRaySphere();
		unsigned int nrx=myManager->getNumberOfReceivers();
		if (nrx!=lastReceivedFieldBufferSize) {
			//Create a new buffer
			setReceivedFieldBuffer();
			lastReceivedFieldBufferSize=nrx;
		}
		//initReceivedFieldBuffer();
		std::cout<<"Transmitting RDN with NOMEM (receivedFieldBuffer) with el="<<raySphere.elevationSteps<<"az="<<raySphere.azimuthSteps<<std::endl;
		myManager->getContext()->launch(reflectionEntryIndex, raySphere.elevationSteps, raySphere.azimuthSteps,numTransmitters); //Launch 3D (elevation, azimuth, transmitters);

		processReceivedFieldBuffer();
	}
	void RayDensityNormalizationSimulation::executeFixedRayLaunch(unsigned int numTransmitters) {
		RaySphere raySphere= myManager->getRaySphere();
		//Check 
		RTsize w,h,d;
		fixedRayBuffer->getSize(w,h,d);	
		unsigned int nrx=myManager->getNumberOfReceivers();
		if ((nrx!=w) || (h!=raySphere.elevationSteps) || (d!=raySphere.azimuthSteps)) {
			throw  opal::Exception("RayDensityNormalizationSimulation::executeFixedRayLaunch(): fixedRayBuffer dimensions do not match launch receivers and ray count. Call setFixedRayBuffer() first");

		}			
		std::cout<<"Transmitting RDN with NOATIMIC (fixedRaySize) with el="<<raySphere.elevationSteps<<"az="<<raySphere.azimuthSteps<<std::endl;
		myManager->getContext()->launch(reflectionEntryIndex, raySphere.elevationSteps, raySphere.azimuthSteps,numTransmitters); //Launch 3D (elevation, azimuth, transmitters);

		processFixedRayBuffer();		
	}
	void RayDensityNormalizationSimulation::processReceivedFieldBuffer() {
		std::vector<int> enabledDevices= myManager->getEnabledDevices();
		std::vector<SphereReceiver*>	receivers=myManager->getReceivers(); 
		thrust::host_vector<RDNHit> h=opalthrustutils::getReceivedFieldMultiGPU(receivedFieldBuffer,enabledDevices,receivers.size());
		std::vector<Transmitter*> activeTransmitters = myManager->getActiveTransmitters();
		//With NOMEN we cannot know the number of real hits unless we store them in the buffer...
		for (unsigned int i=0; i<h.size(); ++i) {
			if (mode==ComputeMode::FIELD) {
				float2	Ex = make_float2(h[i].EEx.z,h[i].EEx.w);
				float2	Ey = make_float2(h[i].EyEz.x,h[i].EyEz.y);
				float2	Ez = make_float2(h[i].EyEz.z,h[i].EyEz.w);
				myManager->getFieldInfo()->updateField(Ex,Ey,Ez,receivers[i]->externalId,activeTransmitters[0]->externalId,i,1u); 			
				//info->updateField(h[i].Ex,h[i].Ey,h[i].Ez,receivers[i]->externalId,activeTransmitters[0]->externalId,i,1u); 			
			} else {
				float2 E = make_float2(h[i].EEx.x,h[i].EEx.y);
				myManager->getFieldInfo()->updateField(E,receivers[i]->externalId,activeTransmitters[0]->externalId,i,1u); 		
				//std::cout<<"RDN NOMEM field E="<<E<<std::endl;	
			}
		}
	}
	void RayDensityNormalizationSimulation::processFixedRayBuffer() {
		std::vector<Transmitter*> activeTransmitters = myManager->getActiveTransmitters();
		std::vector<SphereReceiver*>	receivers=myManager->getReceivers();
		RDNHit* re=reinterpret_cast<RDNHit*>(fixedRayBuffer->map());
		float ex=0.0f;
		float ey=0.0f;
		RTsize w,h,d;
		fixedRayBuffer->getSize(w,h,d);	
		for (unsigned int x = 0; x < w; ++x)
		{
			float2 Ex=make_float2(0.0f,0.0f);
			float2 Ey=make_float2(0.0f,0.0f);
			float2 Ez=make_float2(0.0f,0.0f);
			float2 E=make_float2(0.0f,0.0f);

			unsigned int raysHit=0u;
			for (unsigned int y = 0; y < h; ++y)
			{
				for (unsigned int z = 0; z < d; ++z)
				{
					unsigned int index=(z*w*h)+(y*w)+x;
					//			#ifdef OPAL_EXTENDED_HITINFO
					if (mode==ComputeMode::FIELD) {
						float4 EEx = re[index].EEx; 
						float4 EyEz = re[index].EyEz; 
						Ex +=make_float2(EEx.z,EEx.w);
						Ey +=make_float2(EyEz.x,EyEz.y);
						Ez +=make_float2(EyEz.z,EyEz.w);
					} else {
						float4 EEx=re[index].EEx;
						float2 h=make_float2(EEx.x,EEx.y);
						if ((h.x!=ex) && (h.y!=ey)){
							//std::cout<<x<<","<<y<<","<<z<<":E_>="<<re[index]<<std::endl;
							++raysHit;	
						}
						E += h;
					}

				}
			}
			if (mode==ComputeMode::FIELD) {
				myManager->getFieldInfo()->updateField(Ex,Ey,Ez,receivers[x]->externalId,activeTransmitters[0]->externalId,x,1u); 			
			} else {

				myManager->getFieldInfo()->updateField(E,receivers[x]->externalId,activeTransmitters[0]->externalId,x,raysHit); 			
			}
		}
		fixedRayBuffer->unmap();
	}
	void RayDensityNormalizationSimulation::initFixedRayBuffer() {
		std::vector<SphereReceiver*>	receivers=myManager->getReceivers();
		float2* re=reinterpret_cast<float2*>(fixedRayBuffer->map());

		for (unsigned int x = 0; x < receivers.size(); ++x)
		{
			for (unsigned int y = 0; y < 1000; ++y)
			{
				for (unsigned int z = 0; z < 1000; ++z)
				{
					//std::cout<<"E_>="<<(*re)<<std::endl;
					(*re)=make_float2(0.0f,0.0f);
					++re;

				}
			}
		}
		fixedRayBuffer->unmap();
	}
	void RayDensityNormalizationSimulation::endPartialLaunch(uint numTransmitters) {
		if (generateTraceLog) {
			executeLogRayTrace(dirs.data(), dirs.size(),numTransmitters);
		}


	}
	float RayDensityNormalizationSimulation::setInitialDensity(long rayCount,float initAzimuth, float endAzimuth, float initElevation, float endElevation) {
		float deg2rad=M_PIf/180.f;
		//Solid angle= area of surface of sphere between the limits-> Integral of  dS = sin(t) dt df. t=theta=elevatiom f=phi= azimuth.  
		float dens=(rayCount/((deg2rad*endAzimuth-deg2rad*initAzimuth)*(cosf(deg2rad*initElevation)-cosf(deg2rad*endElevation))));
		this->initialDensity=dens;
		this->rayCount=rayCount;
		return dens;


	}
	void RayDensityNormalizationSimulation::processLaunch(HitInfo* host_hits, uint hits, uint numTransmitters) {

		processHits(host_hits,hits);
		if (printHits) {
			printHitInfo(host_hits,hits);
		}
	}
	void RayDensityNormalizationSimulation::processHits(HitInfo* h, uint hits) {

		std::cout<<"processHits Unordered hits="<<hits<<std::endl;

		uint index=0u;
		uint raysHit=0u;
		uint currentTx=0u;
		std::vector<Transmitter*> activeTransmitters = myManager->getActiveTransmitters();
		std::vector<SphereReceiver*>	receivers=myManager->getReceivers(); 
		for (uint i=0; i<hits; i++) {

			HitInfo* host_hits=&h[i];
			currentTx=host_hits->thrd.x;
			index=host_hits->thrd.z;
			if (mode==ComputeMode::FIELD) {
				float2	Ex = make_float2(host_hits->EEx.z,host_hits->EEx.w);
				float2	Ey = make_float2(host_hits->EyEz.x,host_hits->EyEz.y);
				float2	Ez = make_float2(host_hits->EyEz.z,host_hits->EyEz.w);

				myManager->getFieldInfo()->updateField(Ex,Ey,Ez,receivers[index]->externalId,activeTransmitters[currentTx]->externalId,index,1u); 			
			} else {
				float2	E = make_float2(host_hits->EEx.x,host_hits->EEx.y);
				myManager->getFieldInfo()->updateField(E,receivers[index]->externalId,activeTransmitters[currentTx]->externalId,index,1u); 			
			}
			if (generateTraceLog) {
				//Store the ray directions for trace
				const float4 rd=(host_hits)->rdud;
				dirs.push_back(make_float3(rd.x,rd.y,rd.z));
			}

		}
	}
	void  RayDensityNormalizationSimulation::printHitInfo(HitInfo* host_hits, uint hits) {
		uint dh=0;
		for (uint i=0; i<hits; i++) {
			if ((host_hits)->thrd.y==0) {
				//std::cout<<"DH\t"<<i<<"\t"<<(host_hits)->E.x<<"\t"<<(host_hits)->E.y<<std::endl;
				dh++;

			}

			++host_hits;
		}
		std::cout<<"DH\t"<<dh<<std::endl;
	}
	std::string RayDensityNormalizationSimulation::printInternalBuffersState() {
		std::ostringstream stream;
		RTsize w,h,d;
		switch(execMode) {
			case HITINFO:
				stream<<LPCurvedMeshReflectionSimulation::printInternalBuffersState();
				break;
			case NOATOMIC:
				fixedRayBuffer->getSize(w,h,d);	
				stream <<"\t RDN fixedRayBuffer=("<<w<<","<<h<<","<<d<<"). size="<<((float)w*h*d*sizeof(float2)/(1024*1024*1024))<<" GB"<<std::endl;
				break;
			case NOMEM:
				receivedFieldBuffer->getSize(w);
				stream <<"\t RDN receivedFieldBuffer=("<<w<<"). size="<<(w*sizeof(RDNHit))<<" bytes"<<std::endl;
				break;
		}
		if (parametersBuffer) {
			parametersBuffer->getSize(w);
			stream <<"\t RDN parametersBuffer=("<<w<<"). size="<<(w*sizeof(RDNParameters))<<" bytes"<<std::endl;
		}
		return stream.str();
	}


	float RayDensityNormalizationSimulation::sphereRadiusForExpectedDirectHits(uint expectedHits, float initialDensity, float distance) {
		//Returns the radius of the sphere receiver to have M expected direct hits
		return (distance*sqrt(expectedHits/(M_PIf*initialDensity)));


	}

} //namespace opal
