/***************************************************************/
//
//Copyright (c) 2019 Esteban Egea-Lopez http://girtel.upct.es/~eegea
//
/**************************************************************/

#include "opalSimulation.h"
#include <iomanip>
using namespace optix;
namespace opal {
	OpalSimulation::OpalSimulation(OpalSceneManager* m) {
		this->ptxHandler=nullptr;
		this->myManager=m;
		this->optixProgramsDir ="optix";
		this->cudaProgramsDir=(m->getBaseDirectory()+"/"+optixProgramsDir);
		this->acceptCurved=false;
		this->acceptEdge=false;
		this->withPolarization=false;
		this->withRaySphere= true;	
		this->maxGlobalBufferSize=0u;
		this->fractionMemGlobalBufferSize=0.3f;
		this->printHits=false;
		this->simType=OpalSimulationTypes::NONE;
		this->mode=ComputeMode::VOLTAGE;
		this->initialized=false;
		this->reflectionEntryIndex=0;
		this->reflectionRayIndex=0;
		this->enableSimulation=true;
		this->traceEntryIndex=1;
		this->traceRayIndex = 1;
		this->generateTraceLog=false;
	}
	OpalSimulation::~OpalSimulation() {
		if (ptxHandler) {
			delete ptxHandler;
		}
	}
	void OpalSimulation::setComputeMode(ComputeMode m) {
		if (initialized) {
			throw opal::Exception("OpalSimulation::setComputeMode(): changind compute mode when simulation already initialized...Mode can only be changed before setting simulation in context");

		}
		this->mode=m;
	}
	std::string OpalSimulation::printConfigInfo() const  {
		std::ostringstream stream;
		stream<<"--- Simulation Type---"<<std::endl;
		stream<<"Simulation type not defined"<<std::endl;
		stream << "-----" << std::endl;
		return stream.str();
	}
	bool OpalSimulation::acceptCurvedMesh() const {
		return acceptCurved;
	}
	bool OpalSimulation::acceptEdges() const {
		return acceptEdge;
	}
	bool OpalSimulation::usesPolarization() const {
		return withPolarization;
	}
	bool OpalSimulation::usesRaySphere() const {
		return withRaySphere;
	}
	void OpalSimulation::setPrintHits(bool p)  {
		this->printHits=p;
	}
	void OpalSimulation::setEnableTraceLog(bool e) {
		this->generateTraceLog=e;
	}
	void OpalSimulation::setEnableSimulation(bool e) {
		this->enableSimulation=e;
	}
	bool OpalSimulation::isEnabled() const {
		return enableSimulation;
	}
	void OpalSimulation::setIntersectDirectory() {
		//Common dir for all of them
		this->currentDir=(cudaProgramsDir+"/intersect");
	}
	void OpalSimulation::setOtherDirectory() {
		ConfigurationOptions options=myManager->getConfigurationOptions(); 
		//Closest Hit Programs
		if (options.useDepolarization) {
			this->currentDir=(cudaProgramsDir+"/polarization");
		} else {
			this->currentDir=(cudaProgramsDir+"/basic");
		}
	}
	void OpalSimulation::init() {
		unsigned int entryPointCount= myManager->getContext()->getEntryPointCount();
		//Add this ray and entry type
		reflectionEntryIndex = entryPointCount; 
		myManager->getContext()->setEntryPointCount(reflectionEntryIndex+1);
		reflectionRayIndex= myManager->getContext()->getRayTypeCount();
		myManager->getContext()->setRayTypeCount(reflectionRayIndex+1u);
		std::cout<<"OpalSimulation::init() reflectionEntryIndex="<<reflectionEntryIndex<<"reflectionRayIndex="<<reflectionRayIndex<<std::endl;
		if (generateTraceLog) {
			traceEntryIndex = reflectionEntryIndex +1;
			traceRayIndex = reflectionRayIndex +1u;
			myManager->getContext()->setEntryPointCount(traceEntryIndex+1);
			myManager->getContext()->setRayTypeCount(traceRayIndex +1u);
		}
		if (mode==ComputeMode::VOLTAGE) {
			//myManager->getContext()["computeMode"]->setUint(0u);
			checkAndSetComputeMode(0u);
		} else {
			//myManager->getContext()["computeMode"]->setUint(1u);
			checkAndSetComputeMode(1u);
		}
		//setDefaultPrograms();

	}
	void OpalSimulation::checkLaunchSize(unsigned int w, unsigned int h, unsigned int d) {

		if ((w*h*d) >= 4294967296) {
			throw opal::Exception("opalSimulation::checkLaunchSize(): launch dimensions cannot exceed 2^32");
		} 
	}
	void OpalSimulation::checkAndSetComputeMode(unsigned int mode) {
		std::string c("computeMode");
		bool found=false;
		for (unsigned int i=0; i<myManager->getContext()->getVariableCount(); ++i) {
			Variable v=myManager->getContext()->getVariable(i);
			//std::cout<<"context variable set: "<<i<<v->getName()<<std::endl;
			if (v->getName()==c) {	
				found=true;
				if (v->getUint()!=mode) {
					throw opal::Exception("opalSimulation::checkAndSetComputeMode(): another simulation has previously set a computeMode not compatible");

				}
			}		
		}
		if (!found) {
			myManager->getContext()["computeMode"]->setUint(mode);
		}
	}
	void OpalSimulation::finishSceneContext() {
		myManager->getContext()->setRayGenerationProgram(reflectionEntryIndex, myManager->getDefaultProgram("rayGeneration")); //Generation
		myManager->getContext()->setMissProgram(reflectionEntryIndex, myManager->getDefaultProgram("miss"));
		myManager->getContext()->setExceptionProgram(reflectionEntryIndex, myManager->getDefaultProgram("exceptionReflection"));
		if (generateTraceLog) {
			myManager->getContext()->setRayGenerationProgram(traceEntryIndex, myManager->getDefaultProgram("rayGenerationLogTrace")); //Generation
			myManager->getContext()->setMissProgram(traceEntryIndex, myManager->getDefaultProgram("missLogTrace"));
			myManager->getContext()->setExceptionProgram(traceEntryIndex, myManager->getDefaultProgram("exceptionReflection"));
		}

	}
	//void OpalSimulation::setDefaultPrograms(std::map<std::string,optix::Program>& defaultPrograms, optix::Material& defaultMeshMaterial)
	void OpalSimulation::setDefaultPrograms()
	{
		std::cout<<"OpalSimulation::setDefaultPrograms() "<<std::endl;
		//Create compiler options here
		std::vector<const char *> nvccOptions;

		nvccOptions.push_back("-arch");
		nvccOptions.push_back("compute_30");
		ConfigurationOptions options=myManager->getConfigurationOptions(); 
		if (options.useFastMath) {
			nvccOptions.push_back("-use_fast_math");
		}
		nvccOptions.push_back("-lineinfo");
		nvccOptions.push_back("-default-device");
		nvccOptions.push_back("-rdc");
		nvccOptions.push_back("true");
		nvccOptions.push_back("-D__x86_64");

		ptxHandler = new PtxUtil(nvccOptions);

		setIntersectDirectory();
		createIntersectionPrograms();

		setOtherDirectory();
		createClosestHitPrograms();

		//defaultMeshMaterial = createDefaultMeshMaterial(OPAL_RAY_REFLECTION,defaultPrograms.at("meshClosestHit")); //For normal rays
		//		defaultMeshMaterial->setClosestHitProgram(reflectionEntryIndex,defaultPrograms.at("meshClosestHit")); //For normal rays
		//For visualization
		if (generateTraceLog) {
			createLogTracePrograms();
		}



	}
	void OpalSimulation::createClosestHitPrograms() {
		std::map<std::string, optix::Program>& defaultPrograms=myManager->getDefaultPrograms();
		//defaultPrograms.insert(std::pair<std::string, optix::Program>("meshClosestHit", createClosestHitMesh()));
		//defaultPrograms.insert(std::pair<std::string, optix::Program>("receiverClosestHit", createClosestHitReceiver()));


		//defaultPrograms.insert(std::pair<std::string, optix::Program>("miss", createMissProgram()));
		//defaultPrograms.insert(std::pair<std::string, optix::Program>("rayGeneration", createRayGenerationProgram()));
		std::cout<<"OpalSimulation::createClosestHitPrograms() creating programs"<<std::endl;
		//std::map<std::string, optix::Program>::iterator it=defaultPrograms.begin();
		//while (it!=defaultPrograms.end()) {
		//	std::cout<<it->first<<std::endl;
		//	++it;
		//}

		defaultPrograms["meshClosestHit"]= createClosestHitMesh();
		//	defaultPrograms.insert(std::pair<std::string, optix::Program>("meshClosestHit", createClosestHitMesh()));
		defaultPrograms["receiverClosestHit"]= createClosestHitReceiver();
		defaultPrograms["miss"]= createMissProgram();
		defaultPrograms["rayGeneration"]= createRayGenerationProgram();

		//	it=defaultPrograms.begin();
		//	while (it!=defaultPrograms.end()) {
		//		std::cout<<it->first<<std::endl;
		//		++it;
		//	}

	}
	void OpalSimulation::createIntersectionPrograms() {
		std::map<std::string, optix::Program>& defaultPrograms=myManager->getDefaultPrograms();
		//TODO: we could add an intersection program for planes (ideal infinite planes), typically used to represent flat grounds, should be more efficient than a mesh?

		//Intersection programs

		//#ifdef OPAL_USE_TRI
		//		defaultPrograms.insert(std::pair<std::string, optix::Program>("triangleAttributes", createTriangleAttributesProgram()));
		//#else
		//		defaultPrograms.insert(std::pair<std::string, optix::Program>("meshIntersection", createIntersectionTriangle()));
		//		defaultPrograms.insert(std::pair<std::string, optix::Program>("meshBounds", createBoundingBoxTriangle()));
		//#endif
		//		defaultPrograms.insert(std::pair<std::string, optix::Program>("receiverIntersection", createIntersectionSphere()));
		//		defaultPrograms.insert(std::pair<std::string, optix::Program>("receiverBounds", createBoundingBoxSphere()));
#ifdef OPAL_USE_TRI
		defaultPrograms["triangleAttributes"]= createTriangleAttributesProgram();
#else
		defaultPrograms["meshIntersection"]= createIntersectionTriangle();
		defaultPrograms["meshBounds"]= createBoundingBoxTriangle();
#endif
		defaultPrograms["receiverIntersection"]= createIntersectionSphere();
		defaultPrograms["receiverBounds"]= createBoundingBoxSphere();

	}
	OpalSimulationTypes OpalSimulation::getSimulationType() const {
		return simType;
	}
	//std::map<std::string, optix::Program> OpalSimulation::getDefaultPrograms() {
	//	return defaultPrograms;
	//}
	//optix::Material OpalSimulation::getDefaultMeshMaterial()  {
	//	return defaultMeshMaterial;
	//}
	optix::Program OpalSimulation::createClosestHitReceiver()
	{

		optix::Program chrx;
		chrx = myManager->getContext()->createProgramFromPTXString(ptxHandler->getPtxString(currentDir.c_str(), "receiver.cu"), "closestHitReceiver");


		//Program variables: common value for all receiver instances, since they all share the program. 
		//chrx["k"]->setFloat(myManager->getChannelParameters().k); //If multichannel is used, this should be set per transmission

		return chrx;
	}


	optix::Program OpalSimulation::createBoundingBoxTriangle()
	{
		return myManager->getContext()->createProgramFromPTXString(ptxHandler->getPtxString(currentDir.c_str(), "optixTriangle.cu"), "boundsTriangle");
	}

	optix::Program OpalSimulation::createTriangleAttributesProgram()
	{
		return  myManager->getContext()->createProgramFromPTXString(ptxHandler->getPtxString(currentDir.c_str(),"optixGeometryTriangles.cu"), "triangle_attributes" ) ;

	}
	optix::Program OpalSimulation::createIntersectionTriangle()
	{
		return myManager->getContext()->createProgramFromPTXString(ptxHandler->getPtxString(currentDir.c_str(), "optixTriangle.cu"), "intersectTriangle");

	}

	optix::Program OpalSimulation::createBoundingBoxSphere()
	{


		return myManager->getContext()->createProgramFromPTXString(ptxHandler->getPtxString(currentDir.c_str(), "sphere.cu"), "boundsSphere");

	}


	optix::Program OpalSimulation::createIntersectionSphere()
	{


		return myManager->getContext()->createProgramFromPTXString(ptxHandler->getPtxString(currentDir.c_str(), "sphere.cu"), "rtgem_intersectSphere");
		//return myManager->getContext()->createProgramFromPTXString(ptxHandler->getPtxString(currentDir.c_str(), "sphere.cu"), "robust_intersectSphere");

	}

	optix::Program OpalSimulation::createMissProgram() 
	{
		return myManager->getContext()->createProgramFromPTXString(ptxHandler->getPtxString(currentDir.c_str(), "receiver.cu"), "miss");

	}



	optix::Program  OpalSimulation::createRayGenerationProgram()
	{
		ConfigurationOptions options=myManager->getConfigurationOptions();
		optix::Program prog; 
		if (options.generateRaysOnLaunch==true) {
			prog= myManager->getContext()->createProgramFromPTXString(ptxHandler->getPtxString(currentDir.c_str(), "generationGPU.cu"), "genRaysOnLaunch");
		} else {

			prog= myManager->getContext()->createProgramFromPTXString(ptxHandler->getPtxString(currentDir.c_str(), "generation.cu"), "genRayAndReflectionsFromSphereIndex");
		}
		prog["rayTypeIndex"]->setUint(reflectionRayIndex);
		return prog;
	}

	//Default programs

	optix::Material OpalSimulation::createDefaultMeshMaterial(unsigned int ray_type_index, optix::Program closestHitProgram) {
		optix::Material mat= myManager->getContext()->createMaterial();
		mat->setClosestHitProgram(ray_type_index, closestHitProgram);

		return mat;
	}
	optix::Program OpalSimulation::createClosestHitMesh() {
		//optix::Program chmesh = myManager->getContext()->createProgramFromPTXString(ptxHandler->getPtxString(currentDir.c_str(), "triangle.cu"), "closestHitTriangle");
		optix::Program chmesh = myManager->getContext()->createProgramFromPTXString(ptxHandler->getPtxString(currentDir.c_str(), "triangle.cu"), "closestHitTriangle");
		return chmesh;

	}
	std::string OpalSimulation::getCudaProgramsDirectory() const  {
		return cudaProgramsDir;
	}
	void OpalSimulation::clearInternalBuffers() {
		if (globalHitInfoBuffer) {
			globalHitInfoBuffer->destroy();
		}
		if (atomicIndexBuffer) {
			atomicIndexBuffer->destroy();
		}




	}
	//Following the advice in https://devtalk.nvidia.com/default/topic/1064715/optix/optix-6-behavior-of-buffer-resizing/
	//we now try keep resizes at a minimum

	void OpalSimulation::setInternalBuffers() {

		globalHitInfoBuffer = setGlobalHitInfoBuffer();
		atomicIndexBuffer=setAtomicIndexBuffer();
		//Always create these buffers, set to 1 even if log trace is not used, because otherwise the traceFunctions.h raise error due to unset buffer
		//Create ray direction buffer. Set it to one at the moment because we do not know the number of hits: resized later
		hitRays = myManager->getContext()->createBuffer(RT_BUFFER_INPUT, RT_FORMAT_FLOAT3, 1u);
		myManager->getContext()["hitRays"]->set(hitRays);
		traceBuffer = myManager->getContext()->createBuffer(RT_BUFFER_INPUT_OUTPUT | RT_BUFFER_GPU_LOCAL, RT_FORMAT_USER, 1u);
		traceBuffer->setElementSize(sizeof(LogTraceHitInfo));
		myManager->getContext()["traceBuffer"]->set(traceBuffer);
		traceAtomicIndexBuffer = myManager->getContext()->createBuffer(RT_BUFFER_INPUT_OUTPUT | RT_BUFFER_GPU_LOCAL, RT_FORMAT_UNSIGNED_INT, 1u);
		myManager->getContext()["traceAtomicIndex"]->set(traceAtomicIndexBuffer);
	}

	//TODO: change constants to parameters
	optix::Buffer OpalSimulation::setGlobalHitInfoBuffer() {

		RTsize bytes=0;
		std::vector<int> enabledDevices=myManager->getEnabledDevices();
		for (size_t i = 0; i < enabledDevices.size(); ++i) 
		{
			bytes+=myManager->getContext()->getAvailableDeviceMemory(enabledDevices[i]);
		}
		//Ignore current state and use a maximum size buffer, to be reused between launches
		//We use a fraction of the available memory. Assuming here that all the devices have the same memory...
		long bsize=floor(fractionMemGlobalBufferSize*bytes/(sizeof(HitInfo)*enabledDevices.size()));
		std::cout<<"-- Creating global buffer: available "<<enabledDevices.size()<<" devices with total device memory   "<<(bytes/(1024*1024))<<" MiB. globalHitBuffer  size (MiB)="<<((bsize*sizeof(HitInfo))/(1024*1024))<<std::endl;
		std::cout<<"-- Global buffer: size of HitInfo= "<<sizeof(HitInfo)<<". Maximum number of hits per buffer=   "<<bsize<< std::endl;
		if (bsize>4294967296) {
			//TODO: What to do in this case?, we can have potentially an index greater that an int size, can we actually address that memory with []... check documentation
			throw opal::Exception("setGlobalHitInfoBuffer(): maximum number of hits is greater than the maximum number allowed for the atomic index...");
		}

		optix::Buffer b;
		b = myManager->getContext()->createBuffer(RT_BUFFER_INPUT_OUTPUT | RT_BUFFER_GPU_LOCAL, RT_FORMAT_USER, static_cast<uint>(bsize));
		b->setElementSize(sizeof(HitInfo));
		myManager->getContext()["globalHitInfoBuffer"]->set(b);
		myManager->getContext()["global_info_buffer_maxsize"]->setUint(bsize);
		maxGlobalBufferSize=bsize;
		return b;
	}
	optix::Buffer OpalSimulation::setAtomicIndexBuffer() {
		optix::Buffer b;
		b = myManager->getContext()->createBuffer(RT_BUFFER_INPUT_OUTPUT | RT_BUFFER_GPU_LOCAL, RT_FORMAT_UNSIGNED_INT, 1u);
		myManager->getContext()["atomicIndex"]->set(b);
		return b;
	}
	std::string opal::OpalSimulation::printInternalBuffersState()
	{

		std::ostringstream stream;
		unsigned long long totalBytes = 0;
		unsigned long long sb;
		stream << "--Simulation Internal buffers--" << std::endl;
		RTsize w;
		RTsize h;
		RTsize d;
		//std::cout<<"Available device memory: "<<context->getAvailableDeviceMemory(0)<<std::endl;
		//std::cout<<"sizeof(HitInfo)="<<sizeof(HitInfo)<<std::endl;
		//std::cout<<"Receivers="<<currentInternalBuffersState.rx<<std::endl;
		if (globalHitInfoBuffer) {
			globalHitInfoBuffer->getSize(w);
			sb = sizeof(HitInfo)*w;
			totalBytes += sb;
			stream << "\t globalHitInfoBuffers=(" << w <<"). size=" << (sb / (1024.f*1024.f)) << " MiB" << std::endl;
		}
		if (atomicIndexBuffer) {
			atomicIndexBuffer->getSize(w);
			sb = sizeof(uint)*w;
			totalBytes += sb;
			stream << "\t atomicIndexBuffer=(" << w <<"). size=" << (sb / (1024.f*1024.f)) << " MiB" << std::endl;
		}
		//Check memory usage
		stream << "Total memory in OpalSimulation internal buffers:  " << (totalBytes / (1024.f*1024.f)) << " MiB" << std::endl;
		return stream.str();

	}
	void OpalSimulation::addStaticMesh(OpalMesh& mesh, std::vector<optix::Material>& materials) {

		//Add closest hit for receiver on the default receiver Material
		std::map<std::string, optix::Program>& defaultPrograms=myManager->getDefaultPrograms();
		//std::cout<<"OpalSimulation::addStaticMesh(): Setting closest hit for mesh with reflectionRayIndex="<<reflectionRayIndex<<std::endl;
	
		//The vector comes always with default mesh material at index 0
		materials[0]->setClosestHitProgram(reflectionRayIndex,defaultPrograms.at("meshClosestHit")); //For normal rays
		if (generateTraceLog) {
			//std::cout<<"OpalSimulation::addStaticMesh(): Setting closest hit for trace with traceRayIndex="<<traceRayIndex<<std::endl;
			materials[0]->setClosestHitProgram(traceRayIndex,defaultPrograms.at("closestHitTriangleLogTrace"));
		}	
	}
	void OpalSimulation::addStaticCurvedMesh(OpalMesh& mesh, std::vector<optix::Material>& materials) {
	}
	void OpalSimulation::updateReceiver(int id, float3 position, float3 polarization, float radius) {
	}
	void OpalSimulation::removeReceiver(int id) {
	}
	void OpalSimulation::transformEdge(Edge* e, optix::Matrix4x4 t) {
	}	
	void OpalSimulation::addReceiver(int id, float3  position, float3 polarization, float radius, std::function<void(float, int)>  callback, std::vector<optix::Material>& materials) {
		//Add closest hit for receiver on the default receiver Material
		std::map<std::string, optix::Program>& defaultPrograms=myManager->getDefaultPrograms();
		materials.push_back(myManager->getDefaultReceiverMaterial());
		materials[0]->setClosestHitProgram(reflectionEntryIndex,defaultPrograms.at("receiverClosestHit")); //For normal rays
		if (generateTraceLog) {
			materials[0]->setClosestHitProgram(traceEntryIndex,defaultPrograms.at("closestHitReceiverLogTrace"));
		}

	}
	void OpalSimulation::processLaunch(HitInfo* host_hits, uint hits, uint numTransmitters) {
		processHits(host_hits,hits);
		// Log hits received
		if (printHits) {
			//		std::cout<<"E["<<i<<"]="<<(host_hits)->E<<std::endl;
			printHitInfo(host_hits,hits);		
		}
		if (generateTraceLog) {
			for (size_t i=0; i<hits; ++i) {
				float3 dir=make_float3(host_hits[i].rdud.x,host_hits[i].rdud.y,host_hits[i].rdud.z);
				//std::cout<<std::setprecision(15)<<i<<"\t"<<dir<<"\t"<<host_hits[i].r<<"\t"<<host_hits[i].thrd<<std::endl;

				dirs.push_back(dir);
				//std::cout<<"Log index="<<host_hits[i].index<<std::endl;
			}
			executeLogRayTrace(dirs.data(), hits,numTransmitters);
			dirs.clear();
		}
	}

	void OpalSimulation::computeReceivedPower(optix::float2 E, unsigned int index, unsigned int txIndex,  uint raysHit) {
		std::vector<Transmitter*> activeTransmitters = myManager->getActiveTransmitters();
		float3 origin = make_float3(activeTransmitters[txIndex]->origin_p);
		float txPower=activeTransmitters[txIndex]->origin_p.w;
		uint txId=activeTransmitters[txIndex]->externalId; 
		float k= activeTransmitters[txIndex]->polarization_k.w;
		float eA= 1.0f / (4*k*k); //Effective Area of the antenna (lambda/4*pi)^2
		float power = eA*((E.x*E.x) + (E.y*E.y))*txPower;
		std::vector<SphereReceiver*> receivers	=myManager->getReceivers(); 	
		std::cout << "rx["<<receivers[index]->externalId<<"]=" << receivers[index]->position << ".r=" << receivers[index]->radius << "(sphere="<<receivers[index]->geomInstance["sphere"]->getFloat4()<<"); tx["<<txId<<"]=" << origin << " eA=" << eA << " txPower=" << txPower << " E=(" << E.x << "," << E.y << ")" << " p=" << power <<  " d=" << length(origin - receivers[index]->position) << std::endl;
		float radius=receivers[index]->radius;
		//receivers[index]->lastPower=power;
		//receivers[index]->lastField=E;
		//Call callbacks...add your own
		//receivers[index]->callback(power, txId);
		//Power
		std::cout<<std::setprecision(10)<<"PR\t"<<power<<"\t"<<receivers[index]->externalId<<"\t"<<receivers[index]->position.x<<"\t"<<receivers[index]->position.y <<"\t"<<receivers[index]->position.z<<"\t"<<raysHit<<"\t"<<radius<<"\t"<<length(origin - receivers[index]->position)<< std::endl;

		//E
		std::cout<<std::setprecision(10)<<"ER\t"<<E.x<<"\t"<<E.y<<"\t"<<receivers[index]->externalId<<"\t"<<receivers[index]->position.x<<"\t"<<receivers[index]->position.y<<"\t"<<receivers[index]->position.z<<"\t"<<raysHit<< "\t"<<radius<<"\t"<<length(origin - receivers[index]->position)<< std::endl;

	}


	void OpalSimulation::computeReceivedPower(optix::float2 Ex, optix::float2 Ey, optix::float2 Ez, unsigned int index,unsigned int txIndex, uint raysHit) {
		//E
		std::vector<SphereReceiver*>	receivers=myManager->getReceivers(); 	
		float radius=(receivers[index]->radius);
		//float area=4*M_PI*radius*radius;
		//std::cout << "rx["<<receivers[index]->externalId<<"]=" << receivers[index]->position << ".r=" << receivers[index]->radius << "(sphere="<<receivers[index]->geomInstance["sphere"]->getFloat4()<<"); tx["<<txId<<"]=" << origin << " eA=" << defaultChannel.eA << " txPower=" << txPower << " E=(" << E.x << "," << E.y << ")" << " p=" << power <<  " d=" << length(origin - receivers[index]->position) << std::endl;


		std::cout<<"ERX\t"<<Ex.x<<"\t"<<Ex.y<<"\t"<<receivers[index]->externalId<<"\t"<<receivers[index]->position.x<<"\t"<<receivers[index]->position.y<<"\t"<<receivers[index]->position.z<<"\t"<<raysHit<<"\t"<<radius<< std::endl;
		std::cout<<"ERY\t"<<Ey.x<<"\t"<<Ey.y<<"\t"<<receivers[index]->externalId<<"\t"<<receivers[index]->position.x<<"\t"<<receivers[index]->position.y<<"\t"<<receivers[index]->position.z<<"\t"<<raysHit<<"\t"<<radius<< std::endl;
		std::cout<<"PRY\t"<<(10*log10(dot(Ey,Ey)))<<"\t"<<receivers[index]->externalId<<"\t"<<receivers[index]->position.x<<"\t"<<receivers[index]->position.y<<"\t"<<receivers[index]->position.z<<"\t"<<raysHit<<"\t"<<radius<< std::endl;
		std::cout<<"ERZ\t"<<Ez.x<<"\t"<<Ez.y<<"\t"<<receivers[index]->externalId<<"\t"<<receivers[index]->position.x<<"\t"<<receivers[index]->position.y<<"\t"<<receivers[index]->position.z<<"\t"<<raysHit<<"\t"<<radius<< std::endl;

	}


	void OpalSimulation::processHits(HitInfo* host_hits, uint hits) {
		//This implementation assumes that hits come ordered by transmitter and receiver previously
		if (hits==0) {
			return;
		}
		//#ifdef OPAL_EXTENDED_HITINFO	
		if (mode==ComputeMode::FIELD) {
			std::cout<<"processHits Field hits="<<hits<<std::endl;	
		}  else {
			std::cout<<"processHits Voltage hits="<<hits<<std::endl;	
		}	
		float2 Ex=make_float2(0.0f,0.0f);
		float2 Ey=make_float2(0.0f,0.0f);
		float2 Ez=make_float2(0.0f,0.0f);
		//#else
		float2 E=make_float2(0.0f,0.0f);
		//#endif
		//std::vector<Transmitter*> activeTransmitters = myManager->getActiveTransmitters();
		uint raysHit=0u;
		//Get first transmitter 			
		uint currentTx=host_hits->thrd.x;
		//Get first receiver
		uint index=host_hits->thrd.z;
		std::vector<Transmitter*> activeTransmitters = myManager->getActiveTransmitters();
		std::vector<SphereReceiver*>	receivers=myManager->getReceivers(); 
		FieldInfo* info=myManager->getFieldInfo();
		for (uint i=0; i<hits; i++) {

			if (host_hits->thrd.x!=currentTx) {
				if (raysHit!=0) {
					//#ifdef OPAL_EXTENDED_HITINFO	
					if (mode==ComputeMode::FIELD) {
						computeReceivedPower(Ex,Ey,Ez,index,currentTx, raysHit); 
						if (info) {	
							info->updateField(Ex,Ey,Ez,receivers[index]->externalId,activeTransmitters[currentTx]->externalId,index,raysHit); 			
						}
					} else {
						//#else
						computeReceivedPower(E,index,currentTx, raysHit); 
						if (info) {
							info->updateField(E,receivers[index]->externalId,activeTransmitters[currentTx]->externalId,index,raysHit); 			
						}
					}
					//#endif			
				} 				
				//New transmitter 				
				currentTx=host_hits->thrd.x; 				
				//New receiver,  start new accumulation 				
				index=host_hits->thrd.z; 				
				raysHit=0u; 				
				//#ifdef OPAL_EXTENDED_HITINFO	
				if (mode==ComputeMode::FIELD) {
					Ex=make_float2(0.0f,0.0f);
					Ey=make_float2(0.0f,0.0f);
					Ez=make_float2(0.0f,0.0f);
					//#else
				} else {
					E=make_float2(0.0f,0.0f); 				
				}
				//#endif
				//std::cout<<"New transmitter tx="<<currentTx<<";rx="<<index<<std::endl; 			
			} else {
				if (host_hits->thrd.z!=index) {
					if (raysHit!=0u) {
						//At least one hit, callback
						//#ifdef OPAL_EXTENDED_HITINFO	
						if (mode==ComputeMode::FIELD) {
							computeReceivedPower(Ex,Ey,Ez,index,currentTx, raysHit); 
							if (info) {	
								info->updateField(Ex,Ey,Ez,receivers[index]->externalId,activeTransmitters[currentTx]->externalId,index,raysHit); 			
							}
							//#else
						} else {
							computeReceivedPower(E,index,currentTx, raysHit); 
							if (info) {
								info->updateField(E,receivers[index]->externalId,activeTransmitters[currentTx]->externalId,index,raysHit); 			
							}
						}
						//#endif			
					}
					//New receiver, start new accumulation
					index=host_hits->thrd.z;
					//#ifdef OPAL_EXTENDED_HITINFO	
					//** Debug
					if (mode==ComputeMode::FIELD) {
						Ex=make_float2(0.0f,0.0f);
						Ey=make_float2(0.0f,0.0f);
						Ez=make_float2(0.0f,0.0f);
						//****
						//#else
					} else {
						E=make_float2(0.0f,0.0f); 				
					}
					//#endif
					raysHit=0u;

				}
			}


			//#ifdef OPAL_EXTENDED_HITINFO	
			if (mode==ComputeMode::FIELD) {
				//** Debug
				Ex += make_float2(host_hits->EEx.z,host_hits->EEx.w);
				Ey += make_float2(host_hits->EyEz.x,host_hits->EyEz.y);
				Ez += make_float2(host_hits->EyEz.z,host_hits->EyEz.w);
				//****
				//#else
			} else {
				E += make_float2(host_hits->EEx.x,host_hits->EEx.y);
			}
			//#endif
			++raysHit;
			++host_hits;

		}
		//Last one
		if (raysHit!=0u) {
			//#ifdef OPAL_EXTENDED_HITINFO	
			if (mode==ComputeMode::FIELD) {
				computeReceivedPower(Ex,Ey,Ez,index,currentTx, raysHit); 
				if (info) {	
					info->updateField(Ex,Ey,Ez,receivers[index]->externalId,activeTransmitters[currentTx]->externalId,index,raysHit); 			
				}
				//#else
			} else {
				computeReceivedPower(E,index,currentTx, raysHit); 
				if (info) {
					info->updateField(E,receivers[index]->externalId,activeTransmitters[currentTx]->externalId,index,raysHit); 			
				}
			}
			//#endif			
		}

	}
	void  OpalSimulation::printHitInfo(HitInfo* host_hits, uint hits) {
		//std::map<int,int> refMap;
		for (uint i=0; i<hits; i++) {

			//#ifdef OPAL_EXTENDED_HITINFO	
			if (mode==ComputeMode::FIELD) {
				std::cout<<i<<"\tEx="<<make_float2(host_hits->EEx.z,host_hits->EEx.w) <<std::endl;
				std::cout<<i<<"\tEy="<<make_float2(host_hits->EyEz.x,host_hits->EyEz.y) <<std::endl;
				std::cout<<i<<"\tEz="<<make_float2(host_hits->EyEz.z,host_hits->EyEz.w) <<std::endl;
				//#else
			} else {
				//std::cout<<i<<"\tE="<<(host_hits)->E<<std::endl;
				std::cout<<"TT\t"<<i<<"\t"<<make_float2(host_hits->EEx.x,host_hits->EEx.y)<<std::endl;
			}
			//#endif
			std::cout<<i<<"\t refhash="<<(host_hits)->thrd.y<<std::endl;
			std::cout<<i<<"\t dist="<<(host_hits)->rdud.w<<std::endl;
			std::cout<<std::setprecision(15)<<i<<"\t dir="<<(host_hits)->rdud<<std::endl;
			//////	    d only for debug. Uncomment in Common.h
			//#ifdef OPAL_EXTENDED_HITINFO	
			//std::cout<<i<<"\t reflections="<<(host_hits)->r<<std::endl;
			//refMap[(host_hits)->r]++;
			//#endif
			////std::cout<<i<<"\t"<<j<<"\t divergence="<<(host_hits)->divergence<<std::endl;
			////		std::cout<<"\t index="<<(host_hits)->in<<std::endl;
			////		std::cout<<"\t Rnorm="<<(host_hits)->Rn<<std::endl;
			////		std::cout<<"\t Rpar="<<(host_hits)->Rp<<std::endl;
			////		std::cout<<"\t h="<<(host_hits)->h<<std::endl;
			////		std::cout<<"\t v="<<(host_hits)->v<<std::endl;
			++host_hits;
		}
		//	for (auto ref : refMap) {
		//		std::cout<<ref.first<<" ref = "<<ref.second<<std::endl;
		//	}	
	}
	void OpalSimulation::createLogTracePrograms() {
		std::map<std::string, optix::Program>& defaultPrograms=myManager->getDefaultPrograms();
		std::string logDir=(cudaProgramsDir+ "/log");
		//std::cout<<"Creating log trace programs from " <<logDir <<std::endl;

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
		prog["rayTypeIndex"]->setUint(traceRayIndex);

		defaultPrograms.insert(std::pair<std::string, optix::Program>("rayGenerationLogTrace", prog));

	}
	void OpalSimulation::executeLogRayTrace(optix::float3* rayDirs, uint hits, uint numTransmitters) {
		//This launch generates traces for the rays that hit after being filtered to remove duplicates
		if (hits==0) {
			std::cout<<" No hits. Not executing Log Ray Trace for "<<hits<<" hits"<<std::endl;
			return;

		}
		if (myManager->getNumberOfReceivers()!=1) {
			throw opal::Exception("OpalSimulation::executeLogRayTrace(): Log trace can only be executed for one receiver at the moment");

		}
		std::cout<<"Executing Log Ray Trace for "<<hits<<" hits"<<std::endl;

		//std::cout<<"PL****"<<std::endl;
		//Fill ray direction buffer
		hitRays->setSize(hits);
		optix::float3* rays_host = reinterpret_cast<optix::float3*>  (hitRays->map());

		for (size_t i=0; i<hits; ++i) 
		{
			rays_host[i]=rayDirs[i];
			std::cout<<i<<"\t"<<rayDirs[i]<<std::endl;

		}
		hitRays->unmap();

		//Set trace buffer
		//We have to take into account the number of transmitter and receivers
		uint maxTraceBufferSize=(2*hits)*(myManager->getMaxReflections()+1);
		traceBuffer->setSize(maxTraceBufferSize);
		//std::cout<<"traceBufferSize="<<maxTraceBufferSize<<std::endl;	

		//This raise an error for multiple devices. It is anyway initialized after every launch in getLogTraceOrderer
		//uint* ai= reinterpret_cast<uint*>(traceAtomicIndexBuffer->map());
		//ai[0]=0u;
		//traceAtomicIndexBuffer->unmap();	
		//Launch
		myManager->getContext()->launch(traceEntryIndex,hits,numTransmitters); //Launch 2D (hits, transmitters);
		std::vector<int> enabledDevices= myManager->getEnabledDevices();
		thrust::host_vector<LogTraceHitInfo> trace=opalthrustutils::getLogTraceOrderer(traceBuffer,traceAtomicIndexBuffer,enabledDevices, maxTraceBufferSize);
		if (trace.size()>0) {
			saveTraceToFile(trace,"ref-trace.txt");
		}	
		//std::cout<<"PL****"<<std::endl;
		//Remove buffer
		//hitRays->destroy();
		traceBuffer->setSize(1u);
	}

	void OpalSimulation::saveTraceToFile(thrust::host_vector<LogTraceHitInfo> trace, std::string fileName) {
		std::cout<<"Saving reflection log trace to "<<fileName<<std::endl;
		std::ofstream file(fileName.c_str(),std::ofstream::out);
		uint currentIndex=trace[0].cdata.x;
		//std::cout<<"cdata="<<trace[0].cdata<<"hp="<<trace[0].hitp<<std::endl;
		file<<currentIndex<<":"<<trace[0].hitp.x<<"\t"<<trace[0].hitp.y<<"\t"<<trace[0].hitp.z;
		if (trace.size()>1) {
			for (int i=1; i<trace.size(); i++) {
				LogTraceHitInfo l=trace[i];
				//std::cout<<"cdata="<<l.cdata<<"hp="<<l.hitp<<std::endl;
				if (l.cdata.x != currentIndex) {
					currentIndex=l.cdata.x;
					file<<":"<<trace[i-1].cdata.y<<std::endl;
					file<<currentIndex<<":"<<l.hitp.x<<"\t"<<l.hitp.y<<"\t"<<l.hitp.z;
				} else {
					file<<"|"<<l.hitp.x<<"\t"<<l.hitp.y<<"\t"<<l.hitp.z;	
				}
			}
			file<<":"<<trace[trace.size()-1].cdata.y<<std::endl;
		} else {
			file<<":"<<trace[0].cdata.y<<std::endl;
		}
		file.close();

	}
	void OpalSimulation::registerReceiverGain(int rxId, int gainId) {

	}
}
