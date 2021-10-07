/***************************************************************/
//
//Copyright (c) 2019 Esteban Egea-Lopez http://girtel.upct.es/~eegea
//
/**************************************************************/

#include "Config.h"
#include "timer.h"
#include "Opal.h"
#include <optixu/optixu_math_namespace.h>
#include <optixu/optixu_matrix_namespace.h> 
#include <optixu/optixu_quaternion_namespace.h> 
#include <optixu/optixpp_namespace.h>
#include "sutil.h"

#include <algorithm>
#include <functional>
#include <iostream>
#include <fstream>
#include <cstring>
#ifdef _WIN32
#include <sstream>
#endif
#include <iomanip>
#include <thrust/host_vector.h>
#include <memory>


//Simulation types implemented
#include "basicSimulation.h"
#include "flatSimulation.h"
#include "curvedMeshSimulation.h"
#include "curvedFlatMeshSimulation.h"
#include "rayDensityNormalizationSimulation.h"
using namespace opal;
using namespace optix;
//Used to accumulate rays hit on receiver from a particular transmitter. Needed to compute the RDN for instance
//Ids are external ids
//This could probably be done on GPU
void FieldInfo::updateTransmitters(std::vector<Transmitter*>& activeTransmitters) {
	//if already inserted, nothing is done
	for (auto t : activeTransmitters) {
		//std::map<int,optix::float2> f;
		//E.insert({t->externalId,f});
		//E.insert({t->externalId,make_float2(0.0f,0.0f)});
		std::map<int,EComponents>* c=new std::map<int,EComponents>();
		Emap.insert({t->externalId,c});
	}

}
void FieldInfo::updateField(optix::float2 Ex, optix::float2 Ey, optix::float2 Ez,  int rxId, int txId, unsigned int index, unsigned int raysHit) {
	try {
		//std::cout<<"Updating field"<<Emap.size()<<std::endl;
		auto f = Emap.at(txId);
		//std::cout<<"Tx field"<<f->size()<<std::endl;

		//Find the receiver
		auto r=f->find(rxId);
		if (r!=f->end()) {
			//Add field
			//std::cout<<"Adding field "<<rxId<<"Ex="<<Ex<<std::endl;
			r->second.Ex+=Ex;
			r->second.Ey+=Ey;
			r->second.Ez+=Ez;
			r->second.hits +=raysHit;

		} else {
			EComponents c;
			c.E=make_float2(0.0f,0.0f);
			c.Ex=Ex;
			c.Ey=Ey;
			c.Ez=Ez;
			c.index=index;		
			c.hits=raysHit;	
			//std::cout<<"Inserting field "<<rxId<<"Ex="<<Ex<<std::endl;
			f->insert({rxId,c});
		}	
	}
	catch (std::out_of_range e) {
		std::cout << "FieldInfo::updateField(): Transmitter not found " << txId << std::endl;
		throw  opal::Exception("FieldInfo::updateField(): transmitter not found");
		return;

	}

}
void FieldInfo::updateField(optix::float2 Ee,  int rxId, int txId, unsigned int index, unsigned int raysHit) {
	try {

		auto f = Emap.at(txId);

		//Find the receiver
		auto r=f->find(rxId);
		if (r!=f->end()) {
			//Add field
			r->second.E+=Ee;
			r->second.hits+=raysHit;
		} else {
			EComponents c;
			c.Ex=make_float2(0.0f,0.0f);
			c.Ey=c.Ex;
			c.Ez=c.Ex;
			c.E=Ee;
			c.index=index;		
			c.hits=raysHit;	
			f->insert({rxId,c});

		}	

	}
	catch (std::out_of_range e) {
		std::cout << "FieldInfo::updateField(): Transmitter not found " << txId << std::endl;
		throw  opal::Exception("FieldInfo::updateField(): transmitter not found");
		return;

	}
}
void FieldInfo::reset() {	
	//Clear maps
	auto it =Emap.begin();
	while (it!=Emap.end()) {
		it->second->clear();
		delete it->second;
		++it;
	}
	Emap.clear();
}
std::map<int, std::map<int,EComponents>*> FieldInfo::getE() {	
	return Emap;
}
float SphereReceiver::getLastReceivedPower(int txId) {
	for (auto t: lastReceivedPower) {
		if (std::get<0>(t)==txId) {
			return std::get<1>(t).x;
		}
	}
	return -1;
}
EComponents SphereReceiver::getLastReceivedE(int txId) {
	for (auto t: lastReceivedE) {
		if (std::get<0>(t)==txId) {
			return std::get<1>(t);
		}
	}
	EComponents e;
	return e;
}
void SphereReceiver::setLastReceivedResult(int txId, float3 r) {
	lastReceivedPower.push_back(std::make_tuple(txId,r));
}
void SphereReceiver::setLastReceivedE(int txId, EComponents e) {
	lastReceivedE.push_back(std::make_tuple(txId,e));
}
void SphereReceiver::clearLastResult() {
	lastReceivedPower.clear();
	lastReceivedE.clear();
}

// -------------------------------------------------------------------------------------
// Init Optix Functions
// -------------------------------------------------------------------------------------

OpalSceneManager::OpalSceneManager() {
	std::cout<<"OpalSceneManager() called"<<std::endl;
	initMembers();

}
OpalSceneManager::OpalSceneManager(float f, bool useExactSpeedOfLight)
{
	initMembers();
	if (!useExactSpeedOfLight) {
		useApproximateSpeedLight();
	}
	initContext(f); 

}

opal::OpalSceneManager::~OpalSceneManager()
{
	try {
		if (context) {
			context->destroy();
		}
		context=nullptr;
		if (transmitterManager) {
			delete this->transmitterManager;
		}
		transmitterManager=nullptr;
		if (rayGenerator) {
			delete this->rayGenerator;
		}
		rayGenerator = nullptr;
		if (simulations.size()>0) {
			for (auto simulation : simulations ) {
				delete simulation;
			}
		}
		//	simulation = nullptr;
		simulations.clear();
		if (info) {
			delete info;
		}		
		info=nullptr;
		for (const auto& kv: dynamicMeshes) {
			delete kv.second;
		}
		for (auto e : edges) {
			delete e;
		}

#ifdef OPALDEBUG
		outputFile.close();
#endif // OPALDEBUG
	}
	catch (optix::Exception e) {
		std::cout << "exception at ~OpalSceneManager():" << e.getErrorString() << "with code:" << e.getErrorCode() << std::endl;
	}
}
void OpalSceneManager::initMembers() {

	sysInfo.getSystemInformation();
	std::cout<<sysInfo.printSystemInformation();
	if (sysInfo.numberOfDevices ==0) {
		throw  opal::Exception("initMembers(): No supported GPU found");
		return;
	} else if (sysInfo.numberOfDevices ==1) {
		options.useMultiGPU=false;
	} else {
		options.useMultiGPU=true;
	}
	this->context=nullptr;
	this->deg2rad=M_PIf/180.f;
	this->baseDir=OPAL_SOURCE_DIR;
	//this->baseDir="tunnels";
	//***
	this->optixProgramsDir= "optix";
	this->cudaProgramsDir=(getBaseDirectory()+"/"+optixProgramsDir);
	this->maxReflections = 10u;
	this->minEpsilon = 1.e-3f;

	//Increased every time a new face is added to the scene.  
	this->numberOfFaces = 0u; 
	this->sceneGraphCreated = false;
	this->sceneFinished = false;
	this->contextInitialized = false;
	this->usingCurvedMeshes = false;

	this->txOriginBuffer = nullptr;
	this->rayRangeBuffer = nullptr;
	this->raySphereParametersBuffer=nullptr;



	this->options.useExactSpeedOfLight=true;
	this->options.usePenetration=false;
	this->options.useDepolarization=false;
	this->options.useFastMath = true;
	this->options.generateRaysOnLaunch = false;
	this->options.useMultiChannel=false;
	this->options.executeCallbacks=false;
	this->options.useAntennaGain=false;
	this->attenuationLimit = -80.0f;

	this->raySphere.raySphereBuffer = nullptr;
	this->raySphere.elevationSteps=0u;
	this->raySphere.azimuthSteps=0u;
	this->raySphere.rayCount=0u;

	this->rayGenerator= nullptr;
	//	this->ptxHandler=nullptr;
	this->transmitterManager=nullptr;
	//this->simulation=nullptr;
	this->info=new FieldInfo();
	this->currentTransmitter.polarization_k=make_float4(0.0f,0.0f,0.0f,0.0f);
	this->currentTransmitter.origin_p=make_float4(0.0f,0.0f,0.0f,0.0f);
	this->currentTransmitter.externalId=-1;
	this->currentTransmitter.gainId=RT_BUFFER_ID_NULL;

}
void OpalSceneManager::setUseAntennaGain(bool use) {
	if (contextInitialized) {
		throw opal::Exception("setUseAntennaGain(): set use antenna gain before intializing context");
	}
	this->options.useAntennaGain=use;

}
bool OpalSceneManager::isUsingCurvedMeshes() const {
	return this->usingCurvedMeshes;
}
void OpalSceneManager::setExecuteCallback(bool execute) {
	this->options.executeCallbacks=execute;
}
void OpalSceneManager::useApproximateSpeedLight() {
	if (contextInitialized) {
		throw opal::Exception("useApproximateSpeedLight(): set approximated speedlight  before intializing context");
	}
	this->options.useExactSpeedOfLight=false;

}
void OpalSceneManager::setSimulation(OpalSimulation* sim) {
	this->simulations.push_back(sim);
	//this->simulation=sim;
}
std::map<std::string, optix::Program>& OpalSceneManager::getDefaultPrograms() {
	return defaultPrograms;
}
optix::Material OpalSceneManager::getDefaultMeshMaterial()  {
	return defaultMeshMaterial;
}
optix::Material OpalSceneManager::getDefaultCurvedMeshMaterial()  {
	return defaultCurvedMeshMaterial;
}
optix::Material OpalSceneManager::getDefaultReceiverMaterial()  {
	return defaultReceiverMaterial;
}
void OpalSceneManager::initContext(float f) {


	createSceneContext();
	setFrequency(f);
	if (simulations.size()==0) {
		throw opal::Exception("initContext(): No simulation set");
	} else {
	}	
	//std::cout<<configInfo.str()<<simulation->printConfigInfo();
	defaultMeshMaterial= context->createMaterial();
	defaultCurvedMeshMaterial= context->createMaterial();
	defaultReceiverMaterial= context->createMaterial();
	createDefaultPrograms();
	rayGenerator = new OpalRaySphereGenerator();
	for (auto simulation : simulations) {
		simulation->init();
		simulation->setDefaultPrograms();
	}


	contextInitialized=true;	
}	
void OpalSceneManager::createDefaultPrograms() {
	//Create compiler options here
	std::vector<const char *> nvccOptions;

	nvccOptions.push_back("-arch");
	nvccOptions.push_back("compute_30");
	ConfigurationOptions options=getConfigurationOptions(); 
	if (options.useFastMath) {
		nvccOptions.push_back("-use_fast_math");
	}
	nvccOptions.push_back("-lineinfo");
	nvccOptions.push_back("-default-device");
	nvccOptions.push_back("-rdc");
	nvccOptions.push_back("true");
	nvccOptions.push_back("-D__x86_64");

	ptxHandler = new PtxUtil(nvccOptions);
	//TODO: we could add an intersection program for planes (ideal infinite planes), typically used to represent flat grounds, should be more efficient than a mesh?

	//Intersection programs
	//Create default programs, can be overriden by simulations later
	//Common dir for all of them
	this->currentDir=(cudaProgramsDir+"/intersect");
#ifdef OPAL_USE_TRI
	defaultPrograms.insert(std::pair<std::string, optix::Program>("triangleAttributes", createTriangleAttributesProgram()));
	defaultPrograms.insert(std::pair<std::string, optix::Program>("triangleAttributesCurved", createCurvedTriangleAttributesProgram()));
#else
	defaultPrograms.insert(std::pair<std::string, optix::Program>("meshIntersection", createIntersectionTriangle()));
	defaultPrograms.insert(std::pair<std::string, optix::Program>("meshBounds", createBoundingBoxTriangle()));
#endif
	defaultPrograms.insert(std::pair<std::string, optix::Program>("receiverIntersection", createIntersectionSphere()));
	defaultPrograms.insert(std::pair<std::string, optix::Program>("receiverBounds", createBoundingBoxSphere()));
	
	defaultPrograms.insert(std::pair<std::string, optix::Program>("exceptionReflection", createExceptionReflectionProgram()));


}
void OpalSceneManager::setBaseDir(std::string b) {
	if (contextInitialized) {
		throw  opal::Exception("setBaseDir(): Context already initialized with another base directory. Call it before initializing context");
	} else {
		this->baseDir=b;
	}
}

void OpalSceneManager::setFrequency(float f)
{
	if (options.useMultiChannel) {
		configInfo<<"\t - Using multichannel. Default channel parameters set below"<<std::endl;
		context["useMultichannel"]->setUint(1u);

	} else {
		context["useMultichannel"]->setUint(0u);
	}
	this->defaultChannel.frequency = f;
	configInfo<<"\t - Frequency (Hz) ="<<f<<std::endl;
	if (options.useExactSpeedOfLight) {
		this->defaultChannel.waveLength = 299792458.0f / f; // c/f
		configInfo<<"\t - Speed of light, c= 299792458.0 m/s"<<std::endl;
		context["speedOfLight"]->setFloat(299792458.0f);
	} else {
		this->defaultChannel.waveLength = 3.0e8f / f;
		configInfo<<"\t - Speed of light, c= 3e8 m/s"<<std::endl;
		context["speedOfLight"]->setFloat(3.0e8f);
	}
	configInfo<<"\t - Wavelength (m) ="<<this->defaultChannel.waveLength<<std::endl;
	this->defaultChannel.k = 2 * 3.14159265358979323846f / this->defaultChannel.waveLength; //wavenumber (2*pi/lambda)
	this->defaultChannel.eA = 1.0f / ((2 * this->defaultChannel.k)*(2 * this->defaultChannel.k)); //Effective Area of the antenna (lambda/4*pi)^2

}
void OpalSceneManager::setMaxReflections(unsigned int m)
{
	this->maxReflections = m;
	if (sceneFinished) {
		context["max_interactions"]->setUint(maxReflections);
	}
	configInfo << "\t - maxReflections=" << maxReflections << std::endl;

}
void OpalSceneManager::setEnabledDevices() {
	if (!options.useMultiGPU) {
		//TODO: Use the device with the highest compute capability
		std::vector<int> dm;
		dm.push_back(0);
		context->setDevices(dm.begin(), dm.end());
	}
	enabledDevices=context->getEnabledDevices();
	for (size_t i = 0; i < enabledDevices.size(); ++i) 
	{
		configInfo << " \t - Context is using local device " << enabledDevices[i] << ": " << context->getDeviceName(enabledDevices[i]) << std::endl;
	}
}
optix::Context OpalSceneManager::getContext() const {
	return context;
}
RaySphere OpalSceneManager::getRaySphere() const {
	return raySphere;
}
std::vector<int> OpalSceneManager::getEnabledDevices() const {
	return enabledDevices;
}
SystemInformation OpalSceneManager::getSystemInformation() const {
	return sysInfo;
}
void OpalSceneManager::createSceneContext()
{

	// Set up context
	//Enable RTX...Needed to really use the RT cores. Does not seem necessary to check if the device actually supports it.
#ifdef OPAL_USE_TRI 
	const int RTX=1;
	if (rtGlobalSetAttribute(RT_GLOBAL_ATTRIBUTE_ENABLE_RTX, sizeof(RTX), &RTX)!=RT_SUCCESS)  {
		std::cout<<"WARNING: Enabling RTX mode failed!"<<std::endl;
	}
#endif
	context = optix::Context::create();
	setEnabledDevices();
	//#ifdef OPAL_LOG_TRACE
	//	context->setRayTypeCount(2u); //Normal, log  ray
	//	context->setEntryPointCount(2u); //2 program: ray,log  generation 
	//#else 
	//	context->setRayTypeCount(1u); //Normal  ray
	//	context->setEntryPointCount(1u); //1 program: ray generation 
	//#endif
}

Edge* OpalSceneManager::addEdgeToGroup(optix::float3 p, optix::float3 v, optix::uint2 faces, optix::float3 face_a, optix::float3 face_b, optix::float3 normal_a, optix::float3 normal_b, MaterialEMProperties emProp, int id, int groupId) {
	OpalDynamicMeshGroup* dmg;
	try {
		dmg = dynamicMeshes.at(groupId);
	}
	catch (std::out_of_range e) {
		std::cout << "addEdgeToGroup(): Not found OpalDynamicMeshGroup with id= " << groupId << std::endl;
		throw  opal::Exception("addEdgeToGroup(): Not found OpalDynamicMeshGroup with this id");
		return nullptr;
	}
	std::cout << "addEdgeToGroup(): with groupId= " << groupId << "and id="<<id<<std::endl;
	Edge* e=addEdge(p,v,faces,face_a,face_b,normal_a,normal_b,emProp,id);
	Edge original=(*e);
	dmg->edges.push_back(std::pair<Edge,Edge*>(original,e));
	return e;
}

//Adds a new edge to the scene. For the Luebbers diffraction coefficient we need a material for the edge
Edge* OpalSceneManager::addEdge(optix::float3 p, optix::float3 v, optix::uint2 faces, optix::float3 face_a, optix::float3 face_b, optix::float3 normal_a, optix::float3 normal_b, MaterialEMProperties emProp, int id) {
	bool accept=false;
	for (auto simulation: simulations) {
		if (simulation->acceptEdges()) {
			accept=true;
			break;
		}
	}
	if (!accept) {
		throw  opal::Exception("addEdge(): No simulation accept edges");
	}
	//Edges are not added to GPU scene since they are not intersected by rays with simple diffraction. They are only used to compute the diffraction points and coefficients
	Edge* e = new Edge();
	e->n_a=normalize(normal_a); //Make sure they are unit vectors 
	e->n_b=normalize(normal_b);
	//The vector defining the edge is a unit vector and length
	const float l=length(v);
	const float3 uv=v/l;
	e->v=make_float4(uv,l);
	e->faces=faces;
	const float la=length(face_a);
	const float3 ua=face_a/la;
	e->a=make_float4(ua,la);
	const float lb=length(face_b);
	const float3 ub=face_b/lb;
	e->b=make_float4(ub,lb);
	//To get the signed angle (internal) between a and b

	//This should work even if the edge is not perpendicular to the plane of the faces
	float angle=signedAngle(ua,ub,uv);
	//Checks of conventions
	if (angle<0) {
		//Interchange origin and edge direction to make signed angle from a to b positive
		std::cout<<"ua="<<ua<<"normal_a="<<normal_a<<"angle="<<(angle/deg2rad)<<std::endl;
		throw  opal::Exception("addEdge(): Angle from face A to B negative. Interchange edge origin and edge direction");
	}
	//Make sure internal angle is less than 180
	if (angle>M_PIf) {
		std::cout<<"ua="<<ua<<"normal_a="<<normal_a<<"angle="<<(angle/deg2rad)<<std::endl;
		throw  opal::Exception("addEdge(): Internal angle   from face A to B > 180 degrees. ");
	}	
	float anb=signedAngle(ua,ub,cross(e->n_a, ua)); //Have to be positive to get normals pointing out of the internal angle
	if (anb<0) {
		std::cout<<"ua="<<ua<<"normal_a="<<normal_a<<"anb="<<(anb/deg2rad)<<std::endl;
		throw  opal::Exception("addEdge(): Angle from face A to B negative. Is normal A pointing out of the internal angle?");
	}
	float bna=signedAngle(ub,ua,cross(e->n_b, ub)); //Have to be positive to get normals pointing out of the internal angle
	if (bna<0) {
		std::cout<<"ub="<<ub<<"normal_b="<<normal_b<<"bna="<<(bna/deg2rad)<<std::endl;
		throw  opal::Exception("addEdge(): Angle from face B to A negative. Is normal B pointing out of the internal angle?");
	}
/***
	//We do not assume that the edge is perpendicular to the plane of the faces, so compute here the normal
	//const float3 nf=cross(normal_a,ua); // This defines the side of the angle measured to be the opposite of the one the normal points at
	//float angle=atan2(dot(cross(ua,ub),nf), dot(ua,ub));
	//if (angle<0) {
	//	angle = 2*M_PI-angle;
	//}
**/
	if (isnan(angle) || isinf(angle)) {	
		std::cout<<"uv="<<uv<<"ua="<<ua<<"normal_a="<<normal_a<<"ub="<<ub<<"normal_b"<<normal_b<<"angle="<<(angle/deg2rad)<<std::endl;
		throw  opal::Exception("addEdge(): Angle from face B to A is nan or inf. Correct or remove the edge");
	}
	e->pn=make_float4(p,2.0-(angle/M_PI));
	e->mat=emProp;
	if (id<0) {
		e->id=edges.size(); 
	} else {
		e->id=id;
	}
	edges.push_back(e);
	std::cout<<"\t Added edge["<<(edges.size()-1)<<"]; id="<<e->id<<";p="<<p<<"; v="<<v<<"e->v="<<e->v<<"e->faces="<<e->faces<<"a="<<e->a<<"n_a="<<e->n_a<<"b="<<e->b<<"n_b="<<e->n_b<<"angle="<<(angle/deg2rad)<<"n="<<e->pn.w<<std::endl;
	return e;
}
float OpalSceneManager::signedAngle(optix::float3 from, optix::float3 to, optix::float3 axis) {
	const float3 fn=normalize(from);
	const float3 tn=normalize(to);
	const float3 an=normalize(axis);
	float a=acosf(dot(fn,tn));
	//Even after normalization, numeric errors may render it out of domain. Try to correct
	if (isnan(a)) {
		float d=dot(fn,tn);
		if (d>1.0000 && d<1.00001) {
			a=0;
		} else if (d<-1.0000 && d>-1.00001) {
			a=M_PIf;
		}
		//std::cout.precision(20);
		//std::cout<<std::scientific<<"fn="<<fn<<"tn="<<tn<<"an="<<an<<"dot="<<dot(fn,tn)<<std::endl;
	}
	float s=sgn(dot(an,cross(fn,tn)));
	//std::cout<<"a="<<a<<"s="<<s<<"cross="<<cross(fn,tn)<<std::endl;
	return (a*s);
}
void OpalSceneManager::addMeshWithFacesToGroup(int groupId, std::vector<optix::float3> &meshVertices,std::vector<std::pair<optix::int3, unsigned int>> &triangleIndexFaceBuffer,  MaterialEMProperties emProp) {
	OpalDynamicMeshGroup* dmg;

	try {
		dmg	= dynamicMeshes.at(groupId);
	}
	catch (std::out_of_range e) {
		std::cout << "addMeshToGroup(): Not found OpalDynamicMeshGroup with id= " << groupId << std::endl;
		throw  opal::Exception("addMeshToGroup(): Not found OpalDynamicMeshGroup with this id");
		return;
	}
	//Now check the face ids are unique
	int uniqueFaces=0;
	if (!checkFaceIds(triangleIndexFaceBuffer, uniqueFaces)) {
		throw opal::Exception("OpalSceneManager::addMeshWithFacesToGroup(): Duplicate face ids");
	} 
	std::cout << "-- Add  mesh to group:"<<groupId<<" with " << meshVertices.size() << " vertices,  " << triangleIndexFaceBuffer.size() << " indices and " <<uniqueFaces<<" faces. totalNumberOfFaces="<<numberOfFaces<< std::endl;

#ifdef OPAL_USE_TRI
	OpalMesh mesh=setMeshOnDevice(meshVertices.size(), meshVertices.data(), triangleIndexFaceBuffer,  defaultPrograms.at("triangleAttributes"), defaultPrograms.at("triangleAttributes"));
#else
	OpalMesh mesh=setMeshOnDevice(meshVertices.size(), meshVertices.data(), triangleIndexFaceBuffer, defaultPrograms.at("meshIntersection"), defaultPrograms.at("meshBounds"));

#endif
	std::vector<optix::Material> optix_materials;
	optix_materials.push_back(getDefaultMeshMaterial());
	//Assign closest hit programs to material from simulations	
	for (auto simulation : simulations) {
		simulation->addStaticMesh(mesh, optix_materials);
	}


#ifdef OPAL_USE_TRI 
	if (optix_materials.size()>1) {
		throw opal::Exception("OpalSceneManager::addStaticMesh(): RT geometryTrianglesInstance can only have one material at the moment");

	}
	mesh.geom_instance->addMaterial(optix_materials[0]); //With RT triangles cannot directly have multiple materials. A different approach is describe in documentation 3.6.4 
#else
	for (auto mat : optix_materials) {
		mesh.geom_instance->addMaterial(mat);
	}

#endif



	setMeshEMProperties(mesh.geom_instance, emProp);

	dmg->geom_group->addChild(mesh.geom_instance);
	dmg->geom_group->getAcceleration()->markDirty();
	if (sceneGraphCreated) {
		rootGroup->getAcceleration()->markDirty();
	}

}
void OpalSceneManager::addMeshToGroup(int groupId, int meshVertexCount, optix::float3* meshVertices, int meshTriangleCount, int* meshTriangles, MaterialEMProperties emProp) {
	OpalDynamicMeshGroup* dmg;

	try {
		dmg	= dynamicMeshes.at(groupId);
	}
	catch (std::out_of_range e) {
		std::cout << "addMeshToGroup(): Not found OpalDynamicMeshGroup with id= " << groupId << std::endl;
		throw  opal::Exception("addMeshToGroup(): Not found OpalDynamicMeshGroup with this id");
		return;
	}
#ifdef OPAL_USE_TRI
	OpalMesh mesh=createMesh(meshVertexCount, meshVertices, meshTriangleCount, meshTriangles,  defaultPrograms.at("triangleAttributes"), defaultPrograms.at("triangleAttributes"));
#else
	OpalMesh mesh = createMesh(meshVertexCount, meshVertices, meshTriangleCount, meshTriangles,  defaultPrograms.at("meshIntersection"), defaultPrograms.at("meshBounds"));

#endif
	std::vector<optix::Material> optix_materials;
	optix_materials.push_back(getDefaultMeshMaterial());
	//Assign closest hit programs to material from simulations	
	for (auto simulation : simulations) {
		simulation->addStaticMesh(mesh, optix_materials);
	}


#ifdef OPAL_USE_TRI 
	if (optix_materials.size()>1) {
		throw opal::Exception("OpalSceneManager::addStaticMesh(): RT geometryTrianglesInstance can only have one material at the moment");

	}
	mesh.geom_instance->addMaterial(optix_materials[0]); //With RT triangles cannot directly have multiple materials. A different approach is describe in documentation 3.6.4 
#else
	for (auto mat : optix_materials) {
		mesh.geom_instance->addMaterial(mat);
	}

#endif



	setMeshEMProperties(mesh.geom_instance, emProp);

	dmg->geom_group->addChild(mesh.geom_instance);
	dmg->geom_group->getAcceleration()->markDirty();
	if (sceneGraphCreated) {
		rootGroup->getAcceleration()->markDirty();
	}

}
void  OpalSceneManager::updateTransformInGroup(int groupId, optix::Matrix4x4 transformationMatrix) {
	OpalDynamicMeshGroup* dmg;
	try {
		dmg = dynamicMeshes.at(groupId);
	}
	catch (std::out_of_range e) {
		std::cout << "updateTransformInGroup(): Not found OpalDynamicMeshGroup with id= " << groupId << std::endl;
		throw  opal::Exception("updateTransformInGroup(): Not found OpalDynamicMeshGroup with this id");
		return;
	}
	for (auto e: dmg->edges) {
		transformEdge(e.first,e.second,transformationMatrix);
	}
	dmg->transform->setMatrix(0, transformationMatrix.getData(), nullptr);
	if (sceneGraphCreated) {
		rootGroup->getAcceleration()->markDirty();
	}
}
void OpalSceneManager::transformEdge(const Edge& original, Edge* e, optix::Matrix4x4 t) {
	//TODO: Assuming we are just translating and/or rotating, not scaling or searing, otherwise the normal transform is not correct and the angles change
	//The transform is supposed to translate or rotate from the original position, otherwise we keep on translating even if the transform does not really change
	//Translate origin
	float4 origin=original.pn;
	float aux=original.pn.w;
	origin.w=1.0f;
	e->pn=t*origin;
	e->pn.w=aux;

	//Transform vectors
	float4 vt=original.v;
	vt.w=0;
	aux=original.v.w;
	e->v=t*vt;
	e->v.w=aux;
	float4 at=original.a;
	at.w=0;
	aux=original.a.w;
	e->a=t*at;
	e->a.w=aux;
	float4 bt=original.b;
	bt.w=0;
	aux=original.b.w;
	e->b=t*bt;
	e->b.w=aux;
	float4 nat=make_float4(original.n_a);
	nat.w=0;
	e->n_a=make_float3(t*nat);
	float4 nbt=make_float4(original.n_b);
	nbt.w=0;
	e->n_b=make_float3(t*nbt);
	//Check
	//std::cout<<"transform Edge: origin="<<e->pn<<"v="<<e->v<<"|v|="<<length(make_float3(e->v))<<"a="<<e->a<<"|a|="<<length(make_float3(e->a))<<"b="<<e->b<<"|b|="<<length(make_float3(e->b))<<"na="<<e->n_a<<"|na|="<<length(e->n_a)<<"nb="<<e->n_b<<"|nb|="<<length(e->n_b)<<std::endl;
	for (auto sim : simulations) {
		sim->transformEdge(e,t);	
	}


}
void opal::OpalSceneManager::finishDynamicMeshGroup(int groupId)
{
	OpalDynamicMeshGroup* dmg;
	try {
		dmg = dynamicMeshes.at(groupId);
	}
	catch (std::out_of_range e) {
		std::cout << "finishDynamicMeshGroup(): Not found OpalDynamicMeshGroup with id= " << groupId << std::endl;
		throw  opal::Exception("finishDynamicMeshGroup(): Not found OpalDynamicMeshGroup with this id");
		return;
	}
	if (sceneGraphCreated) {
		rootGroup->addChild(dmg->transform);
		rootGroup->getAcceleration()->markDirty();
	}
	//	std::cout<<printSceneReport()<<std::endl;

}
//Create material EM properties from ITU parameters (ITU-R P.2040-1)
MaterialEMProperties OpalSceneManager::ITUparametersToMaterial(float a, float b, float c, float d) {
	MaterialEMProperties prop;
	float relativePermitivity;
	if (b==0) {
		relativePermitivity = a;
	}
	else {
		relativePermitivity=a*powf((defaultChannel.frequency / 1.0e9f), b); //Frequency in GHz
	}
	float conductivity;
	if (d == 0) {
		conductivity = c;
	}
	else {
		conductivity = c*powf((defaultChannel.frequency / 1.0e9f), d); //Frequency in GHz
	}
	prop.ITUParameters = make_float4(a,b,c,d);
	prop.dielectricConstant = make_float2(relativePermitivity,-60.0f*defaultChannel.waveLength*conductivity);
	return prop;

}

OpalMesh OpalSceneManager::addStaticMesh(std::vector<optix::float3>& meshVertices, std::vector<int>& meshTriangles, optix::Matrix4x4 transformationMatrix, MaterialEMProperties emProp, bool makeSingleFace) {
	return addStaticMesh(meshVertices.size(),meshVertices.data(),meshTriangles.size(),meshTriangles.data(), transformationMatrix,emProp,makeSingleFace);
}

bool OpalSceneManager::checkFaceIds(std::vector<std::pair<optix::int3, unsigned int>> &triangleIndexFaceBuffer, int& uniqueFaces ) {
	std::set<unsigned int> faceIds;
	for (auto tf: triangleIndexFaceBuffer) {
	//	std::cout<<"OpalSceneManager::addStaticMeshWithFaces(): adding face id "<<tf.second<<std::endl;
		faceIds.insert(tf.second);
	}
	//for (auto i: faceIds) {
	//	std::cout<<"OpalSceneManager::addStaticMeshWithFaces(): faceIds  "<<i<<std::endl;
	//}
	std::vector<unsigned int> v(faceIds.size());
	auto ls = std::set_intersection(faceIds.begin(), faceIds.end(),globalFaces.begin(), globalFaces.end(), v.begin());
	if (ls-v.begin()>0) {
		auto it=v.begin();
		while (it!=ls) {
			std::cout<<"checkFaceIds(): duplicate face id="<<(*it)<<std::endl;
			++it;		
		}
		uniqueFaces = faceIds.size();
		return false;
	} else {
		globalFaces.insert(faceIds.begin(), faceIds.end());
		//for (auto i: globalFaces) {
		//	std::cout<<"checkFaceIds(): global faces  "<<i<<"numberOfFaces="<<numberOfFaces<<std::endl;
		//}
		numberOfFaces += faceIds.size();
		std::cout<<"\t Added new "<<faceIds.size()<<"faces. numberOfFaces="<<numberOfFaces<<std::endl;
		uniqueFaces = faceIds.size();
		return true;
	}
}

//Creates and add a static mesh with face ids provided by user. Materials are set in the simulation classes
OpalMesh OpalSceneManager::addStaticMeshWithFaces(std::vector<optix::float3> &meshVertices,std::vector<std::pair<optix::int3, unsigned int>> &triangleIndexFaceBuffer,  optix::Matrix4x4 transformationMatrix, MaterialEMProperties emProp) {
	//A new copy in case the mesh is being reused with different transforms
	std::vector<optix::float3> transformedVertices(meshVertices.size());
	//Apply first transformation matrix
	for (size_t i = 0; i < meshVertices.size(); i++)
	{
		//std::cout <<"vertex=("<< meshVertices[i].x <<","<< meshVertices[i].y <<","<< meshVertices[i].z <<")"<< std::endl;
		const float3 v = optix::make_float3(transformationMatrix*optix::make_float4(meshVertices[i], 1.0f));
		//std::cout<<"v["<<i<<"]="<<v<<std::endl;
		transformedVertices[i]=v;
	}

	//TODO: Warning, one has to be careful with the following fact: a normal vector does not transform like a
	// a position vector.
	// The proper way of doing it is to multiply by the inverse of the transposed transformationMatrix and multiply it by the determinant of the transformationMatrix.
	// Matrix4x4 it=transformationMatrix.transpose();
	// it=it.inverse();
	// float det=transformationMatrix.det();
	// Then multiply as below
	//See  Dorst, L. "Geometric algebra for computing science", sect. 4.3.6, 
	//See also: https://www.scratchapixel.com/lessons/mathematics-physics-for-computer-graphics/geometry/transforming-normals

	//In our case, the normals are computed from the transformed vertices at the triangle intersections, so there is no problem, but if the normals are precomputed and stored in a buffer, this must be applied

	//Now check the face ids are unique
	int uniqueFaces=0;
	if (!checkFaceIds(triangleIndexFaceBuffer, uniqueFaces)) {
		throw opal::Exception("OpalSceneManager::addStaticMeshWithFaces(): Duplicate face ids");
	}
	std::cout << "-- Create mesh: with " << meshVertices.size() << " vertices,  " << triangleIndexFaceBuffer.size() << " indices and " <<uniqueFaces<<" faces. totalNumberOfFaces="<<numberOfFaces<< std::endl;

#ifdef OPAL_USE_TRI
	OpalMesh mesh=setMeshOnDevice(transformedVertices.size(), transformedVertices.data(), triangleIndexFaceBuffer,  defaultPrograms.at("triangleAttributes"), defaultPrograms.at("triangleAttributes"));
#else
	OpalMesh mesh=setMeshOnDevice(transformedVertices.size(), transformedVertices.data(), triangleIndexFaceBuffer, defaultPrograms.at("meshIntersection"), defaultPrograms.at("meshBounds"));

#endif
	std::vector<optix::Material> optix_materials;
	//Insert default mesh material at index 0 always
	optix_materials.push_back(getDefaultMeshMaterial());
	//Assign closest hit programs to material from simulations	
	for (auto simulation : simulations) {
		simulation->addStaticMesh(mesh, optix_materials);
	}


#ifdef OPAL_USE_TRI 
	if (optix_materials.size()>1) {
		throw opal::Exception("OpalSceneManager::addStaticMesh(): RT geometryTrianglesInstance can only have one material at the moment");

	}
	mesh.geom_instance->addMaterial(optix_materials[0]); //With RT triangles cannot directly have multiple materials. A different approach is describe in documentation 3.6.4 
#else
	for (auto mat : optix_materials) {
		mesh.geom_instance->addMaterial(mat);
	}

#endif


	setMeshEMProperties(mesh.geom_instance, emProp);
	addStaticMesh(mesh);
	std::cout<<"\t \t Mesh added" <<std::endl;
	return mesh;
}

//Creates and add a static mesh. Materials are set in the simulation classes
OpalMesh OpalSceneManager::addStaticMesh(int meshVertexCount, optix::float3* meshVertices, int meshTriangleCount, int* meshTriangles, optix::Matrix4x4 transformationMatrix, MaterialEMProperties emProp, bool makeSingleFace) {
	//TODO: Warning, one has to be careful with the following fact: a normal vector does not transform like a
	// a position vector.
	// The proper way of doing it is to multiply by the inverse of the transposed transformationMatrix and multiply it by the determinant of the transformationMatrix.
	// Matrix4x4 it=transformationMatrix.transpose();
	// it=it.inverse();
	// float det=transformationMatrix.det();
	// Then multiply as below
	//See  Dorst, L. "Geometric algebra for computing science", sect. 4.3.6, 
	//See also: https://www.scratchapixel.com/lessons/mathematics-physics-for-computer-graphics/geometry/transforming-normals
	//In our case, the normals are computed from the transformed vertices at the triangle intersections, so there is no problem, but if the normals are precomputed and stored in a buffer, this must be applied
	
	//We are only transforming points here (vertices)
	//A new copy in case the mesh is being reused with different transforms
	std::vector<optix::float3> transformedVertices(meshVertexCount);
	//Apply first transformation matrix
	for (size_t i = 0; i < meshVertexCount; i++)
	{

		//std::cout <<"vertex=("<< meshVertices[i].x <<","<< meshVertices[i].y <<","<< meshVertices[i].z <<")"<< std::endl;
		const float3 v = optix::make_float3(transformationMatrix*optix::make_float4(meshVertices[i], 1.0f));
		//std::cout<<"v="<<v<<std::endl;
		transformedVertices[i]=v;
	}
#ifdef OPAL_USE_TRI
	OpalMesh mesh=createMesh(meshVertexCount, transformedVertices.data(), meshTriangleCount, meshTriangles,  defaultPrograms.at("triangleAttributes"), defaultPrograms.at("triangleAttributes"), makeSingleFace);
#else
	OpalMesh mesh=createMesh(meshVertexCount, transformedVertices.data(), meshTriangleCount, meshTriangles, defaultPrograms.at("meshIntersection"), defaultPrograms.at("meshBounds"),  makeSingleFace);

#endif
	std::vector<optix::Material> optix_materials;
	//Insert default mesh material at index 0 always
	optix_materials.push_back(getDefaultMeshMaterial());
	//Assign closest hit programs to material from simulations	
	for (auto simulation : simulations) {
		simulation->addStaticMesh(mesh, optix_materials);
	}


#ifdef OPAL_USE_TRI 
	if (optix_materials.size()>1) {
		throw opal::Exception("OpalSceneManager::addStaticMesh(): RT geometryTrianglesInstance can only have one material at the moment");

	}
	mesh.geom_instance->addMaterial(optix_materials[0]); //With RT triangles cannot directly have multiple materials. A different approach is describe in documentation 3.6.4 
#else
	for (auto mat : optix_materials) {
		mesh.geom_instance->addMaterial(mat);
	}

#endif


	setMeshEMProperties(mesh.geom_instance, emProp);
	addStaticMesh(mesh);
	std::cout<<"\t \t Mesh added" <<std::endl;
	return mesh;
}
OpalMesh OpalSceneManager::addStaticCurvedMesh(std::vector<optix::float3>& meshVertices, std::vector<int>& meshTriangles, std::vector<optix::float4>& pd1, std::vector<optix::float4>& pd2, optix::Matrix4x4 transformationMatrix, MaterialEMProperties emProp, bool makeSingleFace, int faceId) {
	bool accepted=false;
	for (auto simulation : simulations) {
		if (simulation->acceptCurvedMesh()) {
			accepted=true;
			break;
		}
	}	
	if (!accepted) {
		throw opal::Exception("Current simulation types do not accept curved meshes");
	}
	//IMPORTANT: BE CAREFUL IF PRINCIPAL DIRECTIONS ARE COMPUTED AS CROSS PRODUCTS OF VECTORS
	// Warning, one has to be careful with the following fact: a normal vector does not transform like a
	// a position vector.
	// The proper way of doing it is to multiply by the inverse of the transposed transformationMatrix and multiply it by the determinant of the transformationMatrix.
	// Matrix4x4 it=transformationMatrix.transpose();
	// it=it.inverse();
	// float det=transformationMatrix.det();
	// Then multiply as below
	//See  Dorst, L. "Geometric algebra for computing science", sect. 4.3.6, 
	//See also: https://www.scratchapixel.com/lessons/mathematics-physics-for-computer-graphics/geometry/transforming-normals


	//std::cout<<"pd1="<<pd1.size()<<"pd2="<<pd2.size()<<std::endl;

	//A new copy in case the mesh is being reused with different transforms
	std::vector<optix::float3> transformedVertices(meshVertices.size());
	//Apply first transformation matrix
	for (size_t i = 0; i < meshVertices.size(); i++)
	{

		//std::cout <<"vertex=("<< meshVertices[i].x <<","<< meshVertices[i].y <<","<< meshVertices[i].z <<")"<< std::endl;
		const float3 v = optix::make_float3(transformationMatrix*optix::make_float4(meshVertices[i], 1.0f));
		//	std::cout<<"v="<<v<<std::endl;
		transformedVertices[i]=v;
	}


	//Now, transform curvature information
	//Device buffers
	optix::Buffer tpd1_b = context->createBuffer(RT_BUFFER_INPUT, RT_FORMAT_FLOAT4, pd1.size());
	optix::Buffer tpd2_b = context->createBuffer(RT_BUFFER_INPUT, RT_FORMAT_FLOAT4, pd2.size());

	//Map to host buffers to write
	float4* h_tpd1 = reinterpret_cast<float4*>  (tpd1_b->map());
	float4* h_tpd2 = reinterpret_cast<float4*>  (tpd2_b->map());

	//Apply proper transformation..	
	Matrix4x4 it=transformationMatrix.transpose();
	it=it.inverse();
	float det=transformationMatrix.det();

	int j=0;
	for (size_t i = 0; i < pd1.size(); i++)
	{
		float4 cv1=pd1[i];	
		float4 cv2=pd2[i];	
		cv1.w=1.0f;
		cv2.w=1.0f;
		float3 u1=optix::make_float3(det*it*cv1);
		float3 u2=optix::make_float3(det*it*cv2);
		//float3 u1=optix::make_float3(transformationMatrix*cv1);
		//float3 u2=optix::make_float3(transformationMatrix*cv2);
		u1=normalize(u1);
		u2=normalize(u2);
		h_tpd1[i]=make_float4(u1.x,u1.y,u1.z,pd1[i].w);
		h_tpd2[i]=make_float4(u2.x,u2.y,u2.z,pd2[i].w);
		//std::cout<<i<<"pd1="<<pd1[i]<<"tpd1="<<h_tpd1[i]<<std::endl;
		//std::cout<<i<<"pd2="<<pd2[i]<<"tpd2="<<h_tpd2[i]<<std::endl;

		//Check correctness: a principal direction has to be orthogonal to the normal of the triangle
		//	int ix=meshTriangles[j];	
		//	int iy=meshTriangles[j+1];	
		//	int iz=meshTriangles[j+2];	
		//	const float3 p0    = transformedVertices[ix];
		//	const float3 p1    = transformedVertices[iy];
		//	const float3 p2    = transformedVertices[iz];
		//	const float3 e0 = p1 - p0;
		//	const float3 e1 = p0 - p2;
		//	const float3 n  = normalize(cross( e1, e0 ));
		//	if (abs((acosf(dot(u1,n))*180/M_PI)-90)>1e-3) {
		//		std::cout<<i<<"n="<<n<<"|n|="<<optix::length(n)<<"u1="<<u1<<"dot(u1,n)="<<dot(u1,n)<<"p0="<<p0<<"p1="<<p1<<"p2="<<p2<<"ix="<<ix<<"iy="<<iy<<"iz"<<iz<<std::endl;
		//		std::cout<<i<<"angle="<<(acosf(dot(u1,n))*180/M_PI)<<std::endl;
		//	}
		//	if (abs((acosf(dot(u2,n))*180/M_PI)-90)>1e-3) {
		//		std::cout<<i<<"n="<<n<<"|n|="<<optix::length(n)<<"u2="<<u2<<"dot(u2,n)="<<dot(u2,n)<<"p0="<<p0<<"p1="<<p1<<"p2="<<p2<<std::endl;
		//		std::cout<<i<<"angle="<<(acosf(dot(u2,n))*180/M_PI)<<std::endl;
		//	}
		j+=3;
	}




	tpd1_b->unmap();
	tpd2_b->unmap();

	//	//Now create our closest Hit
	//	optix::Material mat= context->createMaterial();
	//	mat->setClosestHitProgram(OPAL_RAY_REFLECTION, defaultPrograms.at("meshClosestHitCurved"));
	//#ifdef OPAL_LOG_TRACE
	//	mat->setClosestHitProgram(OPAL_RAY_LOG_TRACE_RAY,defaultPrograms.at("closestHitCurvedLogTrace"));
	//#endif
	//
#ifdef OPAL_USE_TRI
	OpalMesh mesh;
	if (makeSingleFace && faceId>=0) {
		//Check faces 
		auto fit=globalFaces.find(faceId);
		if (fit!=globalFaces.end()) {
			throw opal::Exception("OpalSceneManager::addStaticCurvedMesh(): Duplicate face id");
		}
		//Create the triangleIndexFaceBuffer
		std::vector<std::pair<optix::int3, unsigned int>> triangleIndexFaceBuffer;
		for (int k=0; k< meshTriangles.size(); k=k+3) {
			int3 tri=make_int3(meshTriangles[k],meshTriangles[k+1], meshTriangles[k+2]);
			triangleIndexFaceBuffer.push_back(std::pair<optix::int3,unsigned int>(tri, static_cast<unsigned int>(faceId)));

		}
		globalFaces.insert(faceId);
		numberOfFaces += 1;
		mesh=setMeshOnDevice(transformedVertices.size(), transformedVertices.data(), triangleIndexFaceBuffer,  defaultPrograms.at("triangleAttributes"), defaultPrograms.at("triangleAttributes"));

	} else {
		mesh=createMesh(transformedVertices.size(), transformedVertices.data(), meshTriangles.size(), meshTriangles.data(),  defaultPrograms.at("triangleAttributesCurved"), defaultPrograms.at("triangleAttributesCurved"), makeSingleFace);
	}
#else
	if (makeSingleFace && faceId>=0) {
		//Create the triangleIndexFaceBuffer
		auto fit=globalFaces.find(faceId);
		if (fit==globalFaces.end()) {
			throw opal::Exception("OpalSceneManager::addStaticCurvedMesh(): Duplicate face id");
		}
		std::vector<std::pair<optix::int3, unsigned int>> triangleIndexFaceBuffer;
		for (int k=0; k< meshTriangles.size(); k=k+3) {
			int3 tri=make_int3(meshTriangles[k],meshTriangles[k+1], meshTriangles[k+2]);
			triangleIndexFaceBuffer.push_back(std::pair<optix::int3,unsigned int>(tri, static_cast<unsigned int>(faceId)));

		}
		globalFaces.insert(faceId);
		numberOfFaces += 1;
		mesh=setMeshOnDevice(transformedVertices.size(), transformedVertices.data(), triangleIndexFaceBuffer, defaultPrograms.at("meshIntersection"), defaultPrograms.at("meshBounds"));
	} else {
		mesh=createMesh(transformedVertices.size(), transformedVertices.data(), meshTriangles.size(), meshTriangles.data(), defaultPrograms.at("meshIntersectionCurved"), defaultPrograms.at("meshBounds"),  makeSingleFace);
	}
#endif

	std::vector<optix::Material> optix_materials;
	optix_materials.push_back(getDefaultCurvedMeshMaterial());
	//Assign closest hit programs to material from simulations	
	for (auto simulation : simulations) {
		simulation->addStaticCurvedMesh(mesh, optix_materials);
	}


#ifdef OPAL_USE_TRI 
	if (optix_materials.size()>1) {
		throw opal::Exception("OpalSceneManager::addStaticCurvedMesh(): RT geometryTrianglesInstance can only have one material at the moment");

	}
	mesh.geom_instance->addMaterial(optix_materials[0]);
	optix::GeometryTriangles gt=mesh.geom_instance->getGeometryTriangles();
	//Add additional buffers

	gt["principalDirection1_buffer"]->setBuffer(tpd1_b);	
	gt["principalDirection2_buffer"]->setBuffer(tpd2_b);	
#else
	for (auto mat : optix_materials) {
		mesh.geom_instance->addMaterial(mat);
	}
	optix::Geometry gt=mesh.geom_instance->getGeometry();
	//Add additional buffers

	gt["principalDirection1_buffer"]->setBuffer(tpd1_b);	
	gt["principalDirection2_buffer"]->setBuffer(tpd2_b);	

	mesh.geom_instance = context->createGeometryInstance(geometry, optix_materials.begin(), optix_materials.end());
#endif





	//Set curved mesh
	mesh.geom_instance["curvedMesh"]->setUint(1u);

	setMeshEMProperties(mesh.geom_instance, emProp);
	addStaticMesh(mesh);
	std::cout<<"\t \t Curved Mesh added" <<std::endl;
	this->usingCurvedMeshes = true;
	return mesh;
}

void OpalSceneManager::addStaticMesh(OpalMesh mesh) {
	mesh.geom_instance["meshId"]->setUint(static_cast<uint>(staticMeshes.size())); //Used by the closest hit program only for DEBUG. Could be eliminated
	staticMeshes.push_back(mesh);
	if (sceneGraphCreated) {
		staticMeshesGroup->addChild(mesh.geom_instance);
		staticMeshesGroup->getAcceleration()->markDirty();
		rootGroup->getAcceleration()->markDirty();
	}

}


void OpalSceneManager::extractFaces(optix::float3* meshVertices, std::vector<std::pair<optix::int3, unsigned int>> &triangleIndexBuffer) {
	FaceMap normals;
	//unsigned int faceId=numberOfFaces;
	unsigned int lastId=numberOfFaces;
	if (globalFaces.size()>0) {
		lastId=(*(--globalFaces.end())) +1;
	}
	unsigned int faceId=lastId;
	std::cout<<"extractFaces():: faceId="<<faceId<<"globalFaces.size()="<<globalFaces.size()<<std::endl;
	for (size_t i = 0; i < triangleIndexBuffer.size(); i++)
	{
		//Get the normal of the triangle plane
		const float3 p0 = meshVertices[triangleIndexBuffer[i].first.x];
		const float3 p1 = meshVertices[triangleIndexBuffer[i].first.y];
		const float3 p2 = meshVertices[triangleIndexBuffer[i].first.z];
		const float3 e0 = p1 - p0;
		const float3 e1 = p0 - p2;

		optix::float3 normal = normalize(cross(e1, e0)); //We usually have to normalize if we want to properly compare faces in the map below. The normal is defined by the direction, no the magnitude
		//std::cout <<"i="<<i<<"t="<<triangleIndexBuffer[i].first<< "p0=" << p0 << "p1=" << p1 << "p2=" << p2 << "normal=" << normal << std::endl;




		std::pair<FaceMap::iterator, bool> ret = normals.insert(std::pair<optix::float3, unsigned int>(normal, faceId));
		if (ret.second==true) {
			//New element
			triangleIndexBuffer[i].second = faceId;
			//Next faceId
			++faceId;

		}
		else {
			//Get the faceId
			triangleIndexBuffer[i].second = (*(ret.first)).second;
		}
	}
	std::cout  <<"\t"<< normals.size() <<" faces added."<< std::endl;
	//			for (auto v: normals)
	//		{
	//
	//		std::cout << std::scientific<<"face normal=" << v.first << "id=" << v.second << std::endl;
	//
	//		}






}


OpalDynamicMeshGroup* OpalSceneManager::addDynamicMeshGroup(int groupId) {
	std::cout<<"Dynamic mesh group called with "<<groupId<<std::endl;
	OpalDynamicMeshGroup* dmg = new OpalDynamicMeshGroup();
	dmg->geom_group = context->createGeometryGroup();
	dmg->geom_group->setAcceleration(context->createAcceleration("Trbvh"));
	dmg->transform = context->createTransform();
	dmg->transform->setChild(dmg->geom_group);
	std::pair<std::map<int, OpalDynamicMeshGroup*>::iterator,bool> r= dynamicMeshes.insert(std::pair<int, OpalDynamicMeshGroup*>(groupId, dmg));
	if (r.second == false) {
		std::cout << "A dynamich mesh exists with id= " << groupId << std::endl;
		throw  opal::Exception("A dynamich mesh with this id already exists");
		return nullptr;
	}
	else {

		return dmg;
	}
}
void OpalSceneManager::removeDynamicMeshGroup(int groupId) {
	//Remove this mesh from the scene graph
	OpalDynamicMeshGroup* dmg;
	try {
		dmg = dynamicMeshes.at(groupId);
	}
	catch (std::out_of_range e) {
		std::cout << "removeDynamicMeshGroup(): Not found OpalDynamicMeshGroup with id= " << groupId << std::endl;
		throw  opal::Exception("removeDynamicMeshGroup(): Not found OpalDynamicMeshGroup with this id");
		return;
	}
	if (sceneGraphCreated) {
		rootGroup->removeChild(dmg->transform);
		rootGroup->getAcceleration()->markDirty();
	}
	for (unsigned int i = 0; i < dmg->geom_group->getChildCount(); i++)
	{
		GeometryInstance gi = dmg->geom_group->getChild(i);
#ifdef OPAL_USE_TRI 
		gi->getGeometryTriangles()->removeReference();

#else
		gi->getGeometry()->removeReference();
#endif
		gi->removeReference();
	}
	dmg->geom_group->removeReference();
	dmg->transform->removeReference();
	for (auto e : dmg->edges) {
		removeEdge(e.second);
	}
	dynamicMeshes.erase(groupId);
	delete dmg;
}
void OpalSceneManager::removeEdge(Edge* e) {
	auto it=std::find(edges.begin(), edges.end(),e);
	if (it!=edges.end()) {
		edges.erase(it);
	} else {
		throw  opal::Exception("removeEdge: edge to be removed not found");
	}
}
OpalMesh OpalSceneManager::createMesh(int meshVertexCount, optix::float3* meshVertices, int meshTriangleCount, int* meshTriangles, optix::Program intersectionProgram, optix::Program boundingBoxProgram,  bool makeSingleFace) {

	// Create a float3 formatted buffer (each triplet of floats in the array is now a vector3 in the order x,y,z)

	std::cout << "-- Create mesh: with " << meshVertexCount << " vertices and " << meshTriangleCount << " indices" << std::endl;
	//Make int3s
	if (meshTriangleCount % 3 != 0) {
		throw  opal::Exception("Error: Number of triangle indices is not a multiple of 3");

	}
	std::vector<std::pair<optix::int3, unsigned int>> triangleIndexBuffer;

	for (size_t i = 0; i < meshTriangleCount; i += 3)
	{

		if (makeSingleFace) {
			//Use the current faceId for all the triangles
			triangleIndexBuffer.push_back(std::pair<optix::int3, unsigned int>(make_int3(meshTriangles[i], meshTriangles[i + 1], meshTriangles[i + 2]), numberOfFaces));
		} else {	
			triangleIndexBuffer.push_back(std::pair<optix::int3, unsigned int>(make_int3(meshTriangles[i], meshTriangles[i + 1], meshTriangles[i + 2]), 0u));
		}

	}
	if (makeSingleFace) {
		//Increment current number of faces, that is, faceId
		//++numberOfFaces;
	} else {

		extractFaces(meshVertices, triangleIndexBuffer);
	}
	//for (size_t i = 0; i < triangleIndexBuffer.size(); i++)
	//  {
	//  std::cout<<triangleIndexBuffer[i].second << std::endl;

	//  }

	//Check face id and increase number of faces
	int uniqueFaces=0;
	if (!checkFaceIds(triangleIndexBuffer, uniqueFaces)) {
		throw opal::Exception("OpalSceneManager::createMesh(): Duplicate face ids");
	}

	return setMeshOnDevice(meshVertexCount,meshVertices,triangleIndexBuffer, intersectionProgram, boundingBoxProgram);

}


OpalMesh OpalSceneManager::setMeshOnDevice(int meshVertexCount, optix::float3* meshVertices,std::vector<std::pair<optix::int3, unsigned int>> &triangleIndexBuffer, optix::Program intersectionProgram, optix::Program boundingBoxProgram) {

	OpalMesh mesh;

	//Device buffers
	optix::Buffer tri_indices;
	optix::Buffer positions;
	optix::Buffer faces;

	int numTriangles = static_cast<int>(triangleIndexBuffer.size());
	tri_indices = context->createBuffer(RT_BUFFER_INPUT, RT_FORMAT_INT3, numTriangles);
	positions = context->createBuffer(RT_BUFFER_INPUT, RT_FORMAT_FLOAT3, meshVertexCount);
	faces = context->createBuffer(RT_BUFFER_INPUT, RT_FORMAT_UNSIGNED_INT, numTriangles);


	//Map to host buffers to write
	int3* host_tri_indices = reinterpret_cast<int3*>(tri_indices->map());
	float3* host_positions = reinterpret_cast<float3*>  (positions->map());
	uint* host_faces = reinterpret_cast<uint*>  (faces->map());

	for (size_t i = 0; i < numTriangles; i++)
	{
		host_tri_indices[i] = triangleIndexBuffer[i].first;
		host_faces[i] = triangleIndexBuffer[i].second;
	}



	//Copy to mapped buffer
	memmove(host_positions, meshVertices,sizeof(optix::float3)*meshVertexCount);

	//Release buffers
	positions->unmap();
	tri_indices->unmap();
	faces->unmap();
#ifdef OPAL_USE_TRI 
	// Create a Optix 6.0 GeometryTriangles object.
	optix::GeometryTriangles geom_tri = context->createGeometryTriangles();

	geom_tri["vertex_buffer"]->setBuffer(positions); //Vertices buffer
	geom_tri["index_buffer"]->setBuffer(tri_indices); //Indices buffer
	geom_tri["faceId_buffer"]->setBuffer(faces); //Ids for the faces
	geom_tri->setPrimitiveCount( numTriangles );
	geom_tri->setTriangleIndices( tri_indices, RT_FORMAT_UNSIGNED_INT3 );
	geom_tri->setVertices( meshVertexCount, positions, RT_FORMAT_FLOAT3 );
	geom_tri->setBuildFlags( RTgeometrybuildflags( 0 ) );
	// Set an attribute program for the GeometryTriangles, which will compute
	geom_tri->setAttributeProgram(intersectionProgram );
	GeometryInstance result = context->createGeometryInstance();
	result->setGeometryTriangles( geom_tri );
	mesh.geom_instance = result;

#else

	optix::Geometry geometry = context->createGeometry();
	geometry["vertex_buffer"]->setBuffer(positions);
	geometry["index_buffer"]->setBuffer(tri_indices);
	geometry["faceId_buffer"]->setBuffer(faces);

	geometry->setPrimitiveCount(numTriangles);
	geometry->setBoundingBoxProgram(boundingBoxProgram);
	geometry->setIntersectionProgram(intersectionProgram);
	GeometryInstance result = context->createGeometryInstance();
	result->setGeometry( geometry );
	mesh.geom_instance = result;


#endif
	mesh.num_triangles = numTriangles;



	std::cout<<"\t \t Mesh set on device "<<std::endl;
	return mesh;


}


void OpalSceneManager::setMeshFaceId(OpalMesh mesh, uint id) {
	//WARNING: This changes the mesh face id. No attempt to check that the face ids are still unique is made
	//WARNING: All triangles are set the same id
#ifdef OPAL_USE_TRI
	GeometryTriangles gt=mesh.geom_instance->getGeometryTriangles();
	optix::Buffer faces = gt->queryVariable("faceId_buffer")->getBuffer();
	uint* host_faces = reinterpret_cast<uint*>  (faces->map());
	for (size_t i = 0; i < gt->getPrimitiveCount() ; i++)
	{
		host_faces[i] = id; 
	}
	faces->unmap();
#else
	Geometry g= mesh.geom_instance->getGeometry();
	optix::Buffer faces = g->queryVariable("faceId_buffer")->getBuffer();

	uint* host_faces = reinterpret_cast<uint*>  (faces->map());
	for (size_t i = 0; i < g->getPrimitiveCount() ; i++)
	{
		host_faces[i] = id; 
	}
	faces->unmap();
#endif	
}

void OpalSceneManager::setMeshEMProperties(optix::GeometryInstance geom_instance, MaterialEMProperties emProp) {
	//Conductivity is already multiplied by wavelength
#ifdef OPALDEBUG
	outputFile << "mesh EM="<< emProp.dielectricConstant << std::endl;
#endif

	geom_instance["EMProperties"]->setUserData(sizeof(MaterialEMProperties), &emProp);
}


OpalRaySphereGenerator* OpalSceneManager::getRaySphereGenerator() const {
	return rayGenerator;
}

void OpalSceneManager::createRaySphereFromExternalBuffer(int elevationSteps, int azimuthSteps, optix::float3*  bpointer) {
	if (options.generateRaysOnLaunch==true) {
		throw  opal::Exception("createRaySphereFromExternalBuffer(): generateRaysOnLaunch enabled. This sphere is not going to be traced");

	}
	//First remove the previous buffer if it exists
	if (raySphere.raySphereBuffer) {
		raySphere.raySphereBuffer->destroy();
		raySphere.raySphereBuffer=nullptr;

	}
	//Not standard sphere, trace all
	//	context["standardSphere"]->setUint(0u);
	raySphere.elevationSteps = static_cast<optix::uint>(elevationSteps);
	raySphere.azimuthSteps = static_cast<optix::uint>(azimuthSteps);
	raySphere.rayCount = elevationSteps*azimuthSteps;
	//std::cout << "RaySphere2D rays=" << raySphere.rayCount <<"elevationSteps="<< elevationSteps <<"azimtuhSteps="<< azimuthSteps <<std::endl;
	//std::cout << "RaySphere2D rays=" << raySphere.rayCount <<"elevationSteps="<< raySphere.elevationSteps <<"azimtuhSteps="<< raySphere.azimuthSteps <<std::endl;

	optix::Buffer rays = context->createBuffer(RT_BUFFER_INPUT, RT_FORMAT_FLOAT3, elevationSteps, azimuthSteps);
	rays->setDevicePointer(0,bpointer);
	context["raySphere2D"]->set(rays);
	//	context["raySphereSize"]->setUint(make_uint2(elevationSteps, azimuthSteps));

	//Not standard sphere, trace all
	setRaySphereParameters(elevationSteps,azimuthSteps,0u);
	//	float as;
	//	if (elevationSteps<azimuthSteps) {
	//		as=360.0f/azimuthSteps;
	//	} else {
	//		as=180.0f/elevationSteps;
	//	}
	//	context["asRadiusConstant"]->setFloat(as*deg2rad);
	raySphere.raySphereBuffer = rays;
#ifdef OPALDEBUG
	outputFile << "RaySphere2D rays=" << raySphere.rayCount <<"elevationSteps="<< raySphere.elevationSteps <<"azimtuhSteps="<< raySphere.azimuthSteps <<std::endl;
#endif
}

//Create and arbitrary 2D ray sphere, with ray directions provided by the user in a buffer
void OpalSceneManager::createRaySphere2D(int elevationSteps, int azimuthSteps, optix::float3*  rayDirections)
{
	if (options.generateRaysOnLaunch==true) {
		throw  opal::Exception("createRaySphere2D(): generateRaysOnLaunch enabled. This sphere is not going to be traced");

	}


	//First remove the previous buffer if it exists
	if (raySphere.raySphereBuffer) {
		raySphere.raySphereBuffer->destroy();
		raySphere.raySphereBuffer=nullptr;

	}
	//Not standard sphere, trace all
	//context["standardSphere"]->setUint(0u);
	raySphere.elevationSteps = static_cast<optix::uint>(elevationSteps);
	raySphere.azimuthSteps = static_cast<optix::uint>(azimuthSteps);
	raySphere.rayCount = elevationSteps*azimuthSteps;
	//std::cout << "RaySphere2D rays=" << raySphere.rayCount <<"elevationSteps="<< elevationSteps <<"azimtuhSteps="<< azimuthSteps <<std::endl;
	//std::cout << "RaySphere2D rays=" << raySphere.rayCount <<"elevationSteps="<< raySphere.elevationSteps <<"azimtuhSteps="<< raySphere.azimuthSteps <<std::endl;

	optix::Buffer rays = context->createBuffer(RT_BUFFER_INPUT, RT_FORMAT_FLOAT3, elevationSteps, azimuthSteps);
	optix::float3* rays_host = reinterpret_cast<optix::float3*>  (rays->map());
	for (size_t i = 0; i <  raySphere.rayCount; i++)
	{
		rays_host[i].x = rayDirections[i].x;
		rays_host[i].y = rayDirections[i].y;
		rays_host[i].z = rayDirections[i].z;


	}
	context["raySphere2D"]->set(rays);
	//context["raySphereSize"]->setUint(make_uint2(elevationSteps, azimuthSteps));

	//Not standard sphere, trace all
	setRaySphereParameters(elevationSteps,azimuthSteps,0u);
	//float as;
	//if (elevationSteps<azimuthSteps) {
	//	as=360.0f/azimuthSteps;
	//} else {
	//	as=180.0f/elevationSteps;
	//}
	//context["asRadiusConstant"]->setFloat(as*deg2rad);
	rays->unmap();
	raySphere.raySphereBuffer = rays;
#ifdef OPALDEBUG
	outputFile << "RaySphere2D rays=" << raySphere.rayCount <<"elevationSteps="<< raySphere.elevationSteps <<"azimtuhSteps="<< raySphere.azimuthSteps <<std::endl;
#endif

}

//Create a 2D ray sphere in discrete steps of elevation and azimuth
void OpalSceneManager::createRaySphere2D(int elevationDelta, int azimuthDelta) 
{

	if (360 % azimuthDelta != 0) {
		throw  opal::Exception("360 must be an integer multiple of azimuthDelta");

	}
	if (180 % elevationDelta != 0) {
		throw  opal::Exception("180 must be an integer multiple of elevatioNDelta");

	}
	//First remove the previous buffer if it exists
	if (raySphere.raySphereBuffer) {
		raySphere.raySphereBuffer->destroy();
		raySphere.raySphereBuffer=nullptr;

	}
	//Standard sphere, only trace one up and one down ray
	//context["standardSphere"]->setUint(1u);
	optix::uint elevationSteps = (180u / elevationDelta);

	optix::uint azimuthSteps = (360u / azimuthDelta);
	//std::cout << "createRaySphere2D: elevationSteps=" << elevationSteps << "azimuthSteps=" << azimuthSteps << std::endl;

	++elevationSteps; //elevation includes both 0 and 180. elevation in [0,180], but 0 and 180 maps to the same ray, so we remove them from the multiplication and add later


	raySphere.rayCount = elevationSteps*azimuthSteps; //elevation includes [0,180] whereas azimuth includes [0,360[
	raySphere.elevationSteps = elevationSteps;
	raySphere.azimuthSteps = azimuthSteps;
	//We put it here to make test programs backwards compatible...
	if (options.generateRaysOnLaunch==true) {
		setRayRange(0.0f,1.0f,0.0f,1.0f,elevationSteps,azimuthSteps);
		return;
	}

	optix::Buffer rays = context->createBuffer(RT_BUFFER_INPUT, RT_FORMAT_FLOAT3, raySphere.elevationSteps, raySphere.azimuthSteps);
	optix::float3* rays_host = reinterpret_cast<optix::float3*>  (rays->map());

	optix::float4 forward4 = make_float4(0, 0, 1,1);
	optix::float3 xaxis = make_float3(1, 0, 0);
	optix::float3 yaxis = make_float3(0, 1, 0);
	int x = 0;
	int y = 0;
	int es = 0;
	int as = 0;
	int rc = 0;
	const float EPS=1e-15;
	for (size_t i = 0; i <= 180; i += elevationDelta)
	{
		for (size_t j = 0; j < 360; j+= azimuthDelta)
		{
			float ir = deg2rad*i; //To radians
			float jr = deg2rad*j;//To radians

			//Spherical to cartesian coordinates with r=1, ISO (physics) convention: elevation (inclination) from z-axis, azimuth on the XY plane from x counterclockwise to Y
			//All rays with elevation 0 or 180 and any azimuth  are repeated, because they map to (0,1,0) or (0,-1,0). They will not be traced, but we generate them to keep the symmetry  in the 
			//2D and 3D buffers

			//We use the Unity convention and set UP as the Y axis, being FORWARD the z-axis. That is the Y and Z axis are interchanged with respect to the previous convention
			//float3 ray = make_float3(sinf(ir)*cosf(jr), cosf(ir), sinf(ir)*sinf(jr) );
			//Unity left-handed coordinate system: Y=up, Z=forward, X=right; elevation from Y to Z (around X) clockwise, azimuth from Z to X (around Y) clockwise 
			float3 ray = make_float3(sinf(ir)*sinf(jr), cosf(ir), sinf(ir)*cosf(jr) );

			rays_host[x + elevationSteps*y] = ray;
			y++;
			// std::cout <<"x="<<x<< "i=" << i << "y="<<y <<"j=" << j <<"="<< ray << std::endl;
			as++;
			rc++;
		}
		x++;
		y = 0;
		es++;

	}
	context["raySphere2D"]->set(rays);
	//context["raySphereSize"]->setUint(make_uint2(raySphere.elevationSteps, raySphere.azimuthSteps));

	//Standard sphere, only trace one up and one down ray
	setRaySphereParameters(elevationSteps,azimuthSteps,1u);
	//	float asRadiusConstant;
	//	if (elevationDelta<azimuthDelta) {
	//		asRadiusConstant=elevationDelta*deg2rad;
	//	} else {
	//		asRadiusConstant=azimuthDelta*deg2rad;
	//	}
	//	context["asRadiusConstant"]->setFloat(asRadiusConstant);
	//std::cout <<"-- Create RaySphere 2D: asRadiusConstant="<<asRadiusConstant<<". raySphere.azimuthSteps=" << raySphere.azimuthSteps << ". as=" << as << ". raySphere.elevationSteps =" << raySphere.elevationSteps << ". es=" << es << "raySphere.rayCount=" << raySphere.rayCount << ". rc=" << rc << std::endl;
	rays->unmap();
	raySphere.raySphereBuffer = rays;
#ifdef OPALDEBUG
	outputFile << "RaySphere2D rays=" << raySphere.rayCount << ". elevationSteps=" << raySphere.elevationSteps << ". azimtuhSteps=" << raySphere.azimuthSteps << std::endl;
#endif
}


//Create a ray sphere with fractions of degree
//Now elevation delta and azimuthDelta are a decimal fraction of degree, that is, every unit is 0.1 degree, so elevationDelta=1 means 0.1 degree, elevationDelta=2 means 0.2 degree
void OpalSceneManager::createRaySphere2DSubstep(int elevationDelta, int azimuthDelta)
{

	//Create a ray sphere with fractions of degree
	if (elevationDelta > 10 || azimuthDelta > 10) {
		throw  opal::Exception("Angle substep is greater than 10");
	}

	//First remove the previous buffer if it exists
	if (raySphere.raySphereBuffer) {
		raySphere.raySphereBuffer->destroy();
		raySphere.raySphereBuffer=nullptr;

	}
	//Standard sphere, only trace one up and one down ray
	//context["standardSphere"]->setUint(1u);

	optix::uint elevationSteps = 0;
	optix::uint azimuthSteps = 0;
	for (size_t i = 0; i <= 1800u; i += elevationDelta)
	{
		//Just precompute
		++elevationSteps;
	}
	for (size_t j = 0; j < 3600u; j += azimuthDelta)
	{
		++azimuthSteps;
	}


	raySphere.rayCount = elevationSteps*azimuthSteps;
	raySphere.elevationSteps = elevationSteps;
	raySphere.azimuthSteps = azimuthSteps;
	//We put it here to make test programs backwards compatible...
	if (options.generateRaysOnLaunch==true) {
		setRayRange(0.0f,0.1f,0.0f,0.1f,elevationSteps,azimuthSteps);
		return;
	}

	int x = 0;
	int y = 0;

	int rc = 0;
	optix::Buffer rays = context->createBuffer(RT_BUFFER_INPUT, RT_FORMAT_FLOAT3, elevationSteps, azimuthSteps);
	optix::float3* rays_host = reinterpret_cast<optix::float3*>  (rays->map());
	for (size_t i = 0; i <= 1800u; i+=elevationDelta)
	{
		for (size_t j = 0; j < 3600u; j+=azimuthDelta)
		{
			float ir = deg2rad*(i/10.0f); //To radians
			float jr = deg2rad*(j/10.0f);//To radians

			//Spherical to cartesian coordinates with r=1, ISO (physics) convention: elevation (inclination) from z-axis, azimuth on the XY plane from x counterclockwise to Y
			//All rays with elevation 0 or 180 and any azimuth  are repeated, because they map to (0,1,0) or (0,-1,0). They will not be traced, but we generate them to keep the symmetry  and use 
			//2D and 3D buffers
			//If we use the Unity convention and set UP as the Y axis, being FORWARD the z-axis. That is the Y and Z axis are interchanged with respect to the previous convention
			//float3 ray = make_float3(sinf(ir)*cosf(jr), cosf(ir), sinf(ir)*sinf(jr));

			//Unity left-handed coordinate system: Y=up, Z=forward, X=right; elevation from Y to Z (around X) clockwise, azimuth from Z to X (around Y) clockwise 
			float3 ray = make_float3(sinf(ir)*sinf(jr), cosf(ir), sinf(ir)*cosf(jr) );

			rays_host[x + elevationSteps*y] = ray;

			y++;
			// std::cout << "i=" << i << "j=" << j << "(" << ray.x << "," << ray.y << "," << ray.z << ")" << std::endl;
			rc++;
		}
		x++;

		y = 0;


	}
	std::cout << "-- Create Ray Sphere 2D with decimal degree steps: rays=" << raySphere.rayCount << ". elevationSteps=" << raySphere.elevationSteps << ". azimtuhSteps=" << raySphere.azimuthSteps << "rc=" << rc << std::endl;






	context["raySphere2D"]->set(rays);
	//context["raySphereSize"]->setUint(make_uint2(elevationSteps, azimuthSteps));
	//Standard sphere, only trace one up and one down ray
	setRaySphereParameters(elevationSteps,azimuthSteps,1u);

	//float asRadiusConstant;
	//if (elevationDelta<azimuthDelta) {
	//	asRadiusConstant=elevationDelta*deg2rad*0.1f;
	//} else {
	//	std::cout <<"\t Substep Sphere: azimuthDelta="<<azimuthDelta<<"deg2rad="<<deg2rad<<std::endl; 	
	//	asRadiusConstant=azimuthDelta*deg2rad*0.1f;
	//	std::cout <<"\t Substep Sphere: asRadiusConstant="<<asRadiusConstant<<std::endl;
	//}
	//context["asRadiusConstant"]->setFloat(asRadiusConstant);
	//std::cout <<"Substep Sphere: asRadiusConstant="<<asRadiusConstant<<". raySphere.azimuthSteps=" << raySphere.azimuthSteps << ". raySphere.elevationSteps =" << raySphere.elevationSteps << "raySphere.rayCount=" << raySphere.rayCount <<  std::endl;

	rays->unmap();
	raySphere.raySphereBuffer = rays;
#ifdef OPALDEBUG
	outputFile << "RaySphere2D rays=" << raySphere.rayCount << ". elevationSteps=" << raySphere.elevationSteps << ". azimtuhSteps=" << raySphere.azimuthSteps << std::endl;
#endif
}


//Create a ray sphere with arbitrary steps for a given solid angle
void OpalSceneManager::createRaySphere2D(float initElevation, float elevationDelta, float endElevation, float initAzimuth, float azimuthDelta, float endAzimuth)
{


	optix::uint elevationSteps = 0;
	optix::uint azimuthSteps = 0;
	for (float i=initElevation; i <= endElevation; i += elevationDelta)
	{
		//Just precompute
		++elevationSteps;
	}
	for (float j = initAzimuth; j < endAzimuth; j += azimuthDelta)
	{
		++azimuthSteps;
	}


	raySphere.rayCount = elevationSteps*azimuthSteps;
	raySphere.elevationSteps = elevationSteps;
	raySphere.azimuthSteps = azimuthSteps;

	//We put it here to make test programs backwards compatible...
	if (options.generateRaysOnLaunch==true) {
		setRayRange(initElevation,elevationDelta,initAzimuth,azimuthDelta,elevationSteps,azimuthSteps);
		return;
	}

	//First remove the previous buffer if it exists
	if (raySphere.raySphereBuffer) {
		//std::cout<<"Removing previous ray sphere"<<std::endl;
		raySphere.raySphereBuffer->destroy();
		raySphere.raySphereBuffer=nullptr;

	}

	//Not standard sphere, trace all
	//context["standardSphere"]->setUint(0u);

	int x = 0;
	int y = 0;

	int rc = 0;
	optix::Buffer rays = context->createBuffer(RT_BUFFER_INPUT, RT_FORMAT_FLOAT3, elevationSteps, azimuthSteps);
	optix::float3* rays_host = reinterpret_cast<optix::float3*>  (rays->map());
	for (float i=initElevation; i <= endElevation; i += elevationDelta)
	{
		for (float j = initAzimuth; j < endAzimuth; j += azimuthDelta)
		{
			float ir = deg2rad*(i); //To radians
			float jr = deg2rad*(j);//To radians

			//Spherical to cartesian coordinates with r=1, ISO (physics) convention: elevation (inclination) from z-axis, azimuth on the XY plane from x counterclockwise to Y
			//All rays with elevation 0 or 180 and any azimuth  are repeated, because they map to (0,1,0) or (0,-1,0). They will not be traced, but we generate them to keep the symmetry  and use 
			//Unity left-handed coordinate system: Y=up, Z=forward, X=right; elevation from Y to Z (around X) clockwise, azimuth from Z to X (around Y) clockwise 
			float3 ray = make_float3(sinf(ir)*sinf(jr), cosf(ir), sinf(ir)*cosf(jr) );

			rays_host[x + elevationSteps*y] = ray;

			y++;
			// std::cout << "i=" << i << "j=" << j << "(" << ray.x << "," << ray.y << "," << ray.z << ")" << std::endl;
			rc++;
		}
		x++;

		y = 0;


	}
	//std::cout << "-- Create arbitrary Sphere 2D with total rays=" << (raySphere.rayCount/1e6) <<" (Mrays) Elevation=["<<initElevation<<":"<< elevationDelta<<":" << endElevation<<"]. steps="<<elevationSteps << "; Azimuth=[" << initAzimuth<<":"<<azimuthDelta<<":"<<endAzimuth<<"]. steps="<<azimuthSteps <<"; size of ray sphere buffer="<<(raySphere.rayCount*sizeof(float3)/(1024.f*1024.f))<<"MBi"<< std::endl;






	context["raySphere2D"]->set(rays);
	//context["raySphereSize"]->setUint(make_uint2(elevationSteps, azimuthSteps));
	//Not standard sphere, trace all
	setRaySphereParameters(elevationSteps,azimuthSteps,0u);
	//	float asRadiusConstant;
	//	if (elevationDelta<azimuthDelta) {
	//		asRadiusConstant=elevationDelta*deg2rad;
	//	} else {
	//		//std::cout <<"\t Substep Sphere: azimuthDelta="<<azimuthDelta<<"deg2rad="<<deg2rad<<std::endl; 	
	//		asRadiusConstant=azimuthDelta*deg2rad;
	//		//std::cout <<"\t Substep Sphere: asRadiusConstant="<<asRadiusConstant<<std::endl;
	//	}
	//context["asRadiusConstant"]->setFloat(asRadiusConstant);
	//std::cout <<"Substep Sphere: asRadiusConstant="<<asRadiusConstant<<". raySphere.azimuthSteps=" << raySphere.azimuthSteps << ". raySphere.elevationSteps =" << raySphere.elevationSteps << "raySphere.rayCount=" << raySphere.rayCount <<  std::endl;

	rays->unmap();
	raySphere.raySphereBuffer = rays;
#ifdef OPALDEBUG
	outputFile << "RaySphere2D rays=" << raySphere.rayCount << ". elevationSteps=" << raySphere.elevationSteps << ". azimtuhSteps=" << raySphere.azimuthSteps << std::endl;
#endif
}

void OpalSceneManager::setRayRange(float initElevation, float elevationDelta,  float initAzimuth, float azimuthDelta,  uint elevationSteps, uint azimuthSteps) {
	if (options.generateRaysOnLaunch==false) {
		throw  opal::Exception("setRayRange called but generateRaysOnLaunch not enabled");
	}
	raySphere.rayCount = elevationSteps*azimuthSteps;
	raySphere.elevationSteps = elevationSteps;
	raySphere.azimuthSteps = azimuthSteps;

	//TODO:WARNING: the ray range means two different things, depending on the type of simulation. For RDN elevationDelta is actually the endElevation, because rays are uniformly generated from initEl to endElevation,
	// whereas for the other simulations, the elevationDelta is the angular separation between the rays generated on the GPU
	//Have to change this, but I left it for backward compatibility
	for (auto simulation : simulations) {
		if (simulation->getSimulationType()==OpalSimulationTypes::RDN) {
			std::cout<<"Setting RDN ray range for launch generation to init elevation="<<initElevation<<"end elevation="<<elevationDelta<<"init Azimuth="<<initAzimuth<<"end azimuth="<<azimuthDelta<<std::endl; //In degrees
			dynamic_cast<RayDensityNormalizationSimulation*>(simulation)->setInitialDensity(raySphere.rayCount, initAzimuth,azimuthDelta,initElevation,elevationDelta);
		} else {
			std::cout<<"Setting ray range for launch generation to init elevation="<<initElevation<<"elevationDelta="<<elevationDelta<<"initAzimuth="<<initAzimuth<<"azimuthDelta="<<azimuthDelta<<std::endl; //In degrees
		}
	}
	std::cout << "rays=" << raySphere.rayCount << ". elevationSteps=" << raySphere.elevationSteps << ". azimtuhSteps=" << raySphere.azimuthSteps << std::endl;
	if (rayRangeBuffer) {
		optix::float4* range = reinterpret_cast<optix::float4*>  (rayRangeBuffer->map());
		range[0]=make_float4(initElevation,elevationDelta,initAzimuth,azimuthDelta)*deg2rad; //In radians
		std::cout<<"Setting the value of the rayRangeBuffer to "<<range[0]<<std::endl; 
		rayRangeBuffer->unmap();
	} else {
		std::cout<<"setRayRange(): rayRangeBuffer not created yet, call finishSceneContext before setting the ray range"<< std::endl;
		throw  opal::Exception("setRayRange(): rayRangeBuffer not created yet, call finishSceneContext before setting the ray range");
	}



}

void OpalSceneManager::setRaySphereParameters(unsigned int elevationSteps, unsigned int azimuthSteps, unsigned int standardSphere) {
	optix::uint4 p=make_uint4(elevationSteps,azimuthSteps,standardSphere,0);
	if (raySphereParametersBuffer) {
		optix::uint4* pars=reinterpret_cast<optix::uint4*>(raySphereParametersBuffer->map());
		pars[0]=p;
		raySphereParametersBuffer->unmap();

	}
}

int OpalSceneManager::registerAntennaGain(AntennaGain& gain) {
	unsigned int azimuthGains=gain.size(); //Number of rows
	if (azimuthGains==0) {
		throw  opal::Exception("Antenna gain has to be provided as a matrix of azimuth/elevation (rows/columns) values. Number of rows is 0");
	}
	unsigned int elevationGains=gain[0].size(); //Number of columns
	
	std::cout<<"Creating antennaGain buffer with "<<azimuthGains<<" azimuthGains values and "<<elevationGains<<"elevation values"<<std::endl;
	optix::Buffer b = context->createBuffer(RT_BUFFER_INPUT, RT_FORMAT_FLOAT, azimuthGains, elevationGains);
	float* gain_device = reinterpret_cast<float*>  (b->map());
	for (int x=0; x< azimuthGains; ++x) {
		for (int y=0; y< elevationGains; ++y) {
			unsigned int index=y*azimuthGains + x;

			gain_device[index]=gain[x][y];

		}

	}
	b->unmap();
	int id=antennaGainBuffers.size();
	antennaGainBuffers.insert({id,b});
	return id;

}
optix::Buffer OpalSceneManager::getAntennaGainBuffer(int gainId) {
	optix::Buffer b;
	try {

		b= antennaGainBuffers.at(gainId);

	}
	catch (std::out_of_range e) {
		std::cout << "Opal::getAntennaGainBuffer(): Not found antenna gain with id " << gainId<< std::endl;
		throw  opal::Exception("getAntennaGainBuffer(): antenna gain not found, has it been previously registered?");
		return nullptr;

	}
	return b;
}
void OpalSceneManager::registerReceiverGain(int rxId, int gainId) {
	if (!options.useAntennaGain) {
		throw  opal::Exception("registerReceiverGain(): useAntennaGain is not enabled");
	}
	SphereReceiver* rx=getReceiver(rxId);
	optix::Buffer b=getAntennaGainBuffer(gainId);
	rx->antennaGainId=gainId;
	rtBufferId<float,2> id=b->getId();
	rx->geomInstance["gainBufferId"]->setUserData(sizeof(rtBufferId<float,2>), &id);
	for (auto simulation : simulations) {
		simulation->registerReceiverGain(rxId,gainId);
	}
	
}
void OpalSceneManager::registerTransmitterGain(int txId, int gainId) {
	if (!options.useAntennaGain) {
		throw  opal::Exception("registerTransmitterGain(): useAntennaGain is not enabled");
	}
	if (transmitterManager) {
		transmitterManager->registerTransmitterGain(txId, gainId);
	} else {
		optix::Buffer b=getAntennaGainBuffer(gainId);
		currentTransmitter.gainId=b->getId();
	}
}
void OpalSceneManager::addReceiver(int id, float3  position, float3 polarization, float radius, std::function<void(float, int)>  callback)
{
	std::cout << "--Add receiver: receiver " << id << " with radius " <<radius<<" at "<<position<<"with polarization = "<<polarization<< std::endl;
	optix::Geometry geometry = context->createGeometry();
	float4 sphere = make_float4(position.x, position.y, position.z, radius);

	geometry->setPrimitiveCount(1);
	geometry->setBoundingBoxProgram(defaultPrograms.at("receiverBounds"));
	geometry->setIntersectionProgram(defaultPrograms.at("receiverIntersection"));

	std::vector<optix::Material> optix_materials;

	//Assign closest hit programs to material from simulations	
	for (auto simulation : simulations) {
		simulation->addReceiver(id, position,polarization, radius, callback, optix_materials);
	}
//	if (defaultReceiverMaterial->getClosestHitProgram(0)) {
//		std::cout<<"OpalSceneManager::addReceiver() defaultReceiverMaterial count="<<std::endl;
//	}
	//optix::Program chrx = defaultPrograms.at("receiverClosestHit");
	//optix::Material mat = context->createMaterial();
	//	mat->setClosestHitProgram(OPAL_RAY_REFLECTION, chrx);
	//
	//#ifdef OPAL_LOG_TRACE
	//	mat->setClosestHitProgram(OPAL_RAY_LOG_TRACE_RAY, defaultPrograms.at("closestHitReceiverLogTrace"));
	//#endif
	//	optix_materials.push_back(mat);
	if (optix_materials.size()==0) {
		optix_materials.push_back(defaultReceiverMaterial);
		std::cout<<"WARNING: OpalSceneManager::addReceiver() no material for receiver added in simulations. Adding defaultReceiverMaterial"<<std::endl;

	}
	optix::GeometryInstance geom_instance = context->createGeometryInstance(geometry, optix_materials.begin(), optix_materials.end());

	geom_instance->validate();
	//We use a common program for all the receivers: individual variables must be set per geometry instance

	//Add id and position to instace
	geom_instance["sphere"]->setFloat(sphere);

	//The buffer id is consecutive, independent of the external Id

	uint nextId = static_cast<uint>(receivers.size());
	geom_instance["receiverBufferIndex"]->setUint(nextId);
	geom_instance["externalId"]->setInt(id);
	//No need to set the gainBufferId, it will return a RT_BUFFER_ID_NULL if not set. Anyway, what is below does not compile
	//if (options.useAntennaGain) {
	//	geom_instance["gainBufferId"]->setUserData(sizeof(rtBufferId<float,2>), );
	//}
	for (auto simulation : simulations) {
		if (simulation->usesPolarization()) {
			geom_instance["receiverPolarization"]->setFloat(polarization);
			Matrix<4,4> pol_t=computeMatrixFromWorldToPolarization(polarization);
			geom_instance["transformToPolarization"]->setMatrix4x4fv(false,pol_t.getData());
			std::cout<<"\t Using depolarization. Receiver polarization is "<<polarization << " and polarization transform matrix = "<<pol_t<<std::endl;
			break;
		}
	}
	std::cout<<"\t Receiver buffer index for "<<id << " is "<<nextId<< " and external Id is "<<id<<std::endl;
	SphereReceiver* rx = new SphereReceiver();
	rx->geomInstance = geom_instance;
	rx->position = position;
	rx->polarization=polarization;
	rx->radius = radius;
	rx->callback = callback;
	//rx->closestHitProgram = chrx;
	rx->externalId = id;
	rx->dirty=false;
	rx->antennaGainId=-1;
	//Add to map
	receiverExtToBufferId.insert(std::pair<int, unsigned int>(id, nextId));
	receivers.push_back(rx);

	uint newReceivers = static_cast<uint>(receivers.size());
	if (sceneFinished) {
		//Update sceneGraph
		receiversGroup->setChildCount(newReceivers);
		receiversGroup->setChild(newReceivers-1u, rx->geomInstance);
		receiversGroup->getAcceleration()->markDirty();

		rootGroup->getAcceleration()->markDirty();

	}




	//Do not really need transform since the geometry (sphere) has no structure apart from position and radius...?
	//optix::Transform transform = sceneManager->sceneContext->createTransform();
	//TODO: we could use transforms to place the receivers and update their positions

}


optix::Matrix<4,4> OpalSceneManager::computeMatrixFromWorldToPolarization(float3 pol) {
	const float3 polarization=normalize(pol); //In case it is not normalized


	const float EPSILON=1.0e-7f;
	const float3 up=make_float3(0.0f,1.0f,0.0f);
	const float3 forward=make_float3(0.0f,0.0f,1.0f);
	const float3 right=make_float3(1.0f,0.0f,0.0f);
	optix::Matrix<4,4> C_po = optix::Matrix<4,4>::identity(); //Change of basis matrix from polarization to world;	
	if (fabs(polarization.x-up.x)<EPSILON && fabs(polarization.y-up.y)<EPSILON && fabs(polarization.z-up.z)<EPSILON) {
		//Already aligned, no need to create additional reference frame
		std::cout<<"C="<<C_po<<std::endl;
		return C_po;
	} else {	
		//Set a reference frame aligned with linear polarization, so that Y' (up') is aligned with the polarization in that base
		float3 ypo;	
		float3 xpo;
		float3 zpo;	
		//We get the parameters to compute the rotation of the axis
		const float t = acosf(dot(up,polarization));
		//Rotation axis: normal to the vectors
		const float3 n = normalize(cross(up, polarization));
		//Directly compute the quaternion
		const float s=sinf(t/2.0f);
		const float c=cosf(t/2.0f);
		const Quaternion q(n.x*s, n.y*s, n.z*s, c);
		
		
		ypo=q*up; // y' axis in world coordinates=polarization vector
		zpo=normalize(q*forward);
		xpo=normalize(q*right); 
		//std::cout<<"ypo="<<ypo<<"xpo="<<xpo<<"zpo="<<zpo<<std::endl;

		//Create change of basis matrix from polarization basis to world;
		C_po.setCol(0u,make_float4(xpo));
		C_po.setCol(1u,make_float4(ypo));
		C_po.setCol(2u,make_float4(zpo));
		C_po.setCol(3u,make_float4(0.0f,0.0f,0.0f,1.0f));
		Matrix<4,4> Cinv_op= C_po.inverse(); //Create change of basis matrix from world to polarization;
		//std::cout<<"C="<<Cinv_op<<std::endl;
		return Cinv_op;
	}

}
void OpalSceneManager::removeReceiver(int id) {

	//Find receiver
	unsigned int index;
	try {
		index = receiverExtToBufferId.at(id); //Throw exception if not found
	}
	catch (std::out_of_range e) {
		std::cout << "Not found receiver " << id << std::endl;
		throw  opal::Exception("Receiver is not registered with Opal");
		return;

	}
	SphereReceiver* rx = receivers[index];
	GeometryInstance gi = rx->geomInstance;
	Geometry g = gi->getGeometry();
	receiversGroup->removeChild(gi);
	g->removeReference();
	gi->removeReference();
	receiversGroup->getAcceleration()->markDirty();

	rootGroup->getAcceleration()->markDirty();
	std::cout << "Opal:: Removing receiver " << id << "with buffer index="<<index<<std::endl;

	//context->validate();
	delete rx;
	//Swap contents in the buffers if not last
	if (index != (receivers.size() - 1)) {
		receivers[index] = receivers[receivers.size() - 1];
		receiverExtToBufferId.at(receivers[index]->externalId) = index;
		receivers[index]->geomInstance["receiverBufferIndex"]->setUint(index);
		std::cout << "Swapping with " << (receivers.size() -1) << " with externalId= "<<receivers[index]->externalId<<std::endl;
		std::cout << "New index for " << receivers[index]->externalId<<" is "<<receiverExtToBufferId.at(receivers[index]->externalId)<<std::endl;
		std::cout << "Receiver buffer index for  " << receivers[index]->externalId<<" is "<<receivers[index]->geomInstance["receiverBufferIndex"]->getUint()<<std::endl;
	}
	receivers.pop_back();
	receiverExtToBufferId.erase(id);

	//Notify simulations	
	for (auto simulation : simulations) {
		simulation->removeReceiver(id);
	}	

}

//Remove all the receivers currently in the scene
void OpalSceneManager::clearReceivers() {
	//Surely there are more efficient ways to do this
	for (auto it = receiverExtToBufferId.begin(); it != receiverExtToBufferId.end(); ++it) {
		removeReceiver(it->first);
	}

}
unsigned int OpalSceneManager::getReceiverIndex(int externalId) {
	unsigned int index;
	try {

		index = receiverExtToBufferId.at(externalId);

	}
	catch (std::out_of_range e) {
		std::cout << "Opal::getReceiverIndex(): Not found receiver " << externalId << std::endl;
		throw  opal::Exception("Opal::getReceiverIndex is not registered with Opal");
		return 0;

	}
	return index;
}
SphereReceiver* OpalSceneManager::getReceiver(int externalId) {
	unsigned int index;
	try {

		index = receiverExtToBufferId.at(externalId);

	}
	catch (std::out_of_range e) {
		std::cout << "Opal::getReceiver(): Not found receiver " << externalId<< std::endl;
		throw  opal::Exception("Opal::getReceiver is not registered with Opal");
		return nullptr;

	}
	return receivers[index];
}
void OpalSceneManager::updateReceiver(int id, float3 position) {
	unsigned int index;
	try {

		index = receiverExtToBufferId.at(id);

	}
	catch (std::out_of_range e) {
		std::cout << "updateReceiver(): Not found receiver " << id << std::endl;
		throw  opal::Exception("Receiver is not registered with Opal");
		return;

	}
	SphereReceiver* r = receivers[index];
	r->position = position;
	//std::cout << "sphere=" << position << "f4=" << make_float4(position.x, position.y, position.z, r->radius) << std::endl;
	r->geomInstance["sphere"]->setFloat(make_float4(position.x, position.y, position.z, r->radius));
	if (sceneFinished) {
		//Otherwise rootGroup is not created
		receiversGroup->getAcceleration()->markDirty();
		rootGroup->getAcceleration()->markDirty();
	}
	//Notify simulations	
	for (auto simulation : simulations) {
		simulation->updateReceiver(id,position, r->polarization, r->radius);
	}	
}
void OpalSceneManager::updateReceiver(int id, float3 position, float radius) {
	unsigned int index;
	try {

		index = receiverExtToBufferId.at(id);

	}
	catch (std::out_of_range e) {
		std::cout << "updateReceiver(): Not found receiver " << id << std::endl;
		throw  opal::Exception("Receiver is not registered with Opal");
		return;

	}
	SphereReceiver* r = receivers[index];
	r->position = position;
	r->radius = radius;
	r->geomInstance["sphere"]->setFloat(make_float4(position.x, position.y, position.z, radius));
	if (sceneFinished) {
		//Otherwise rootGroup is not created
		receiversGroup->getAcceleration()->markDirty();
		rootGroup->getAcceleration()->markDirty();
	}
	//Notify simulations	
	for (auto simulation : simulations) {
		simulation->updateReceiver(id,position, r->polarization, radius);
	}	
}

void OpalSceneManager::updateReceiver(int id, float3 position, float3 polarization, float radius) {
	unsigned int index;
	try {

		index = receiverExtToBufferId.at(id);

	}
	catch (std::out_of_range e) {
		std::cout << "updateReceiver(): Not found receiver " << id << std::endl;
		throw  opal::Exception("Receiver is not registered with Opal");
		return;

	}
	SphereReceiver* r = receivers[index];
	r->position = position;
	r->radius = radius;
	r->geomInstance["sphere"]->setFloat(make_float4(position.x, position.y, position.z, radius));
	r->polarization=polarization;
	for (auto simulation : simulations) {
		if (simulation->usesPolarization()) {
			r->geomInstance["receiverPolarization"]->setFloat(polarization);
			Matrix<4,4> pol_t=computeMatrixFromWorldToPolarization(polarization);
			r->geomInstance["transformToPolarization"]->setMatrix4x4fv(false,pol_t.getData());
			break;
		}
	}
	if (sceneFinished) {
		//Otherwise rootGroup is not created
		receiversGroup->getAcceleration()->markDirty();
		rootGroup->getAcceleration()->markDirty();
	}
	//Notify simulations	
	for (auto simulation : simulations) {
		simulation->updateReceiver(id,position, polarization, radius);
	}	
}


void OpalSceneManager::transmit(int txId, float txPower,  float3 origin, float3 polarization, bool partial) {

	//std::cout<<"Transmitting["<<txId<<"]["<<txPower<<"] at "<<origin<<std::endl;
	for (auto simulation : simulations) {	
		if (raySphere.rayCount <= 0 && simulation->usesRaySphere()) {
			throw  opal::Exception("OpalSceneManager::transmit(): Ray count=0. Create Ray sphere before transmitting");
		}
	}

	uint numReceivers = static_cast<uint>(receivers.size());
	if (numReceivers == 0) {
		//For debug or testing reflections or any other thing, at least one receiver should be added
		return;

	}

	if (numReceivers == 1) {
		if (receivers[0]->externalId == txId) {
			//No power will be received, since we are not assuming   dual transceiver
			std::cout<<"Only one transmitter and receiver with the same id. No power will be received since dual transceiver is not implemented"<<std::endl;
			return;
		}
	}


	currentTransmitter.origin_p=make_float4(origin, txPower);
	currentTransmitter.polarization_k=make_float4(polarization, defaultChannel.k);
	//	currentTransmitter.txPower=txPower;
	currentTransmitter.externalId=txId;

	//According to https://devtalk.nvidia.com/default/topic/1048952/optix/recompile-question/ it is better to use a small buffer rather than setting variables
	Transmitter* host_tx = reinterpret_cast<Transmitter*>  (txOriginBuffer->map());
	host_tx->origin_p = currentTransmitter.origin_p;
	host_tx->polarization_k = currentTransmitter.polarization_k;
	//host_tx->txPower=txPower;
	host_tx->externalId = txId;
	if (options.useAntennaGain) {
		host_tx->gainId=currentTransmitter.gainId;
		Matrix<4,4> t=computeMatrixFromWorldToPolarization(polarization);
		//std::cout<<"Transmitter transform pol="<<t<<std::endl;
		host_tx->transformToPolarization =t; 	
		
	} else {
		host_tx->gainId=RT_BUFFER_ID_NULL;
	}
	txOriginBuffer->unmap();

	//std::cout<<"Executing transmit launch" <<std::endl;

	//Init accumulations
	std::vector<Transmitter*> activeTransmitters=getActiveTransmitters();
	info->updateTransmitters(activeTransmitters);
	for (auto r: receivers) {
		r->clearLastResult();
	}

	for (auto simulation : simulations) {
		if (simulation->isEnabled()) {
			simulation->executeTransmitLaunch(1u,partial);
		}
	}

	if (!partial) {	
		//Process results and call callbacks
		computeTotalReceivedPower(options.executeCallbacks);
		info->reset();
	}	
}


void OpalSceneManager::groupTransmit(bool partial) {

	if (!transmitterManager) {
		throw opal::Exception("groupTransmit():: transmitterManager not created");
	}
	std::vector<Transmitter*> activeTransmitters=getActiveTransmitters();
	uint numReceivers = static_cast<uint>(receivers.size());
	if (numReceivers == 0) {
		//For debug or testing reflections or any other thing, at least one receiver should be added
		return;

	}

	uint numTransmitters=static_cast<uint>(activeTransmitters.size());
	if (numTransmitters==0) {
		return;
	}
	if (numReceivers == 1 && numTransmitters==1) {
		if (receivers[0]->externalId == activeTransmitters[0]->externalId) {
			//No power will be received, since we are not assuming   dual transceiver
			return;
		}
	}


	txOriginBuffer->setSize(numTransmitters);

	Transmitter* host_tx = reinterpret_cast<Transmitter*>  (txOriginBuffer->map());
	for (uint i=0; i<numTransmitters; ++i) {
		Transmitter trI;
		trI.origin_p = activeTransmitters[i]->origin_p;
		trI.polarization_k =activeTransmitters[i]-> polarization_k;
		trI.externalId =activeTransmitters[i]-> externalId;
		if (options.useAntennaGain) {
			trI.gainId = activeTransmitters[i]->gainId;
		} else {
		}
		//trI.txPower=activeTransmitters[i]->txPower;
		host_tx[i]=trI;	
	}

	txOriginBuffer->unmap();



	info->updateTransmitters(activeTransmitters);



	std::cout<<"Group transmitting. Number of transmitters= "<<numTransmitters<<std::endl;	
	for (auto simulation : simulations) {
		simulation->executeTransmitLaunch(numTransmitters, partial);
	}
	if (!partial) {
		//Process results and call callbacks
		computeTotalReceivedPower(options.executeCallbacks);
		transmitterManager->clearGroup();
		info->reset();
	}	

}

optix::float2 OpalSceneManager::getAngles(float3 const ray  ) {

	const float EPSILON=1.0e-6f;
	//Get angles from ray r=[sin(e)sin(a) cos(e) cos(a)sin(e)]
	//Since elevation is between 0 and 180 degrees we always get the right value
	float el=acosf(ray.y); //In radians
	float az;
	if ((fabs(1.0f-ray.y)<EPSILON) || (fabs(-1.0f-ray.y)<EPSILON)) {
		//Vertical ray (elevation=0 or 180). All the azimuth angles result in the same vectors, just use 0

		az=0.0f;
	} else {

		//We need to get the right quadrant
		az=atan2f(ray.x/sinf(el),ray.z/sinf(el));//In radians	
	}
	return make_float2(el,az);
}
void OpalSceneManager::endPartialLaunch(uint numTransmitters) {
	for (auto simulation : simulations) {

		simulation->endPartialLaunch(numTransmitters);
	}	
	computeTotalReceivedPower(options.executeCallbacks);
	info->reset();
}
unsigned int OpalSceneManager::getTransmitterIndex(int externalId) {
	if (transmitterManager) {
		return transmitterManager->getTransmitterIndex(externalId);
	} else {
		return 0;
	}
}
Transmitter* OpalSceneManager::getTransmitter(int externalId) {
	if (transmitterManager) {
		return transmitterManager->getTransmitter(externalId);
	} else {
		return &currentTransmitter;
	}
}
std::vector<Transmitter*> OpalSceneManager::getActiveTransmitters() {
	if (transmitterManager) {
		return transmitterManager->getActiveTransmitters();
	} else {
		std::vector<Transmitter*> activeTransmitters;
		activeTransmitters.push_back(&currentTransmitter);
		return activeTransmitters;
	}

}
std::vector<SphereReceiver*> OpalSceneManager::getReceivers() {
	return receivers;
}
OpalSimulation* OpalSceneManager::getSimulation() const {
	if (simulations.size()>0) {
		return simulations[0];
	} else {
		throw opal::Exception("OpalSceneManager::getSimulation(): Simulation not set yet. Call after call initContext");
		return nullptr;
	}
}
OpalSimulation* OpalSceneManager::getSimulation(uint index) const {
	if (simulations.size()>index) {
		return simulations[index];
	} else {
		throw opal::Exception("OpalSceneManager::getSimulation(): Simulation index not found");
		return nullptr;
	}
}
unsigned int OpalSceneManager::getNumberOfEdges() const {
	return edges.size();
}
std::vector<Edge*> OpalSceneManager::getEdges() {
	return edges;
}
unsigned int OpalSceneManager::getNumberOfActiveTransmitters() const {
	if (transmitterManager) {
		return transmitterManager->getActiveTransmitters().size();
	} else {
		return 1u;
	}
}
unsigned int OpalSceneManager::getNumberOfReceivers() const {
	return receivers.size();
}
void OpalSceneManager::setInternalBuffers() {

	if (transmitterManager) {
		txOriginBuffer=setTransmitterBuffer(getNumberOfActiveTransmitters());	

	} else {
		txOriginBuffer=setTransmitterBuffer(1u);	
	}
	if (options.generateRaysOnLaunch) {
		rayRangeBuffer=setRayRangeBuffer();
	} else {
		raySphereParametersBuffer = setRaySphereParametersBuffer();
	}
	for (auto simulation : simulations) {
		simulation->setInternalBuffers();
	}

}


optix::Buffer OpalSceneManager::setTransmitterBuffer(optix::uint tx) {
	uint tra=1u;
	if (tx>0) {
		tra=tx;
	}
	optix::Buffer b = context->createBuffer(RT_BUFFER_INPUT, RT_FORMAT_USER, tra);
	b->setElementSize(sizeof(Transmitter));
	context["txBuffer"]->set(b);
	return b;
}
optix::Buffer OpalSceneManager::setRayRangeBuffer() {
	std::cout<<"setRayRangeBuffer()"<<std::endl;
	optix::Buffer b = context->createBuffer(RT_BUFFER_INPUT, RT_FORMAT_FLOAT4, 1u);
	context["rayRangeBuffer"]->set(b);
	return b;
}
optix::Buffer OpalSceneManager::setRaySphereParametersBuffer() {

	optix::Buffer b = context->createBuffer(RT_BUFFER_INPUT, RT_FORMAT_UNSIGNED_INT4, 1u); //Only used the first 3 uints
	context["raySphereParametersBuffer"]->set(b);
	return b;
}
std::string opal::OpalSceneManager::printContextInformation()
{
	std::ostringstream stream;
	stream<<"Context: Num CPU Threads: "<<context->getCPUNumThreads()<<std::endl;
	int RTX;
	rtGlobalGetAttribute(RT_GLOBAL_ATTRIBUTE_ENABLE_RTX,sizeof(RTX),&RTX);
	stream<<"Context: RTX mode enabled: "<<RTX<<std::endl;

	return stream.str();
}



void OpalSceneManager::finishSceneContext() {

	//std::cout<<"finishSceneContext() called "<<std::endl;

	if (sceneFinished) {
		std::cout<<"finishSceneContext() called multiple times. Check your code"<<std::endl;
		return;
	}
	//Build scene graph
	buildSceneGraph();

	//Create internal buffers
	setInternalBuffers();

	//TODO: move to simulation code
	//Set some minimum extent for the rays. Otherwise we are going to run into numerical accuracy problems with the reflections, almost surely
	context["min_t_epsilon"]->setFloat(minEpsilon);
	context["max_interactions"]->setUint(maxReflections); //TODO: move to per-launch configuration


	//Initial hash value
	context["initialHash"]->setUint(0u);

	if (!options.useFastMath) {
		configInfo <<"\t - Fast math disabled" <<std::endl;

	}
	if (options.useAntennaGain) {
		context["useAntennaGain"]->setUint(1u);
		std::cout<<"\t Using antenna gains"<<std::endl;
	} else {
		context["useAntennaGain"]->setUint(0u);
	}
	if (options.usePenetration) {
		for (auto simulation : simulations) {

			if (simulation->acceptCurvedMesh() ) {
				//Not implemented yet, raise exception here
				throw opal::Exception("Penetration is not implemented yet for curved surfaces");
			}
		}
		context["usePenetration"]->setUint(1u);
		//Show warning about assumptions
		configInfo <<"\t - You are using penetration: check that the  receiver sphere(s) does not overlap any wall, otherwise you are getting wrong results almost surely" <<std::endl;
	} else {
		context["usePenetration"]->setUint(0u);
	}
	context["attenuationLimit"]->setFloat(attenuationLimit);


	if (std::get<0>(options.printEnabled)) {
		if (std::get<2>(options.printEnabled)) {
			setPrintEnabled(std::get<1>(options.printEnabled),std::get<3>(options.printEnabled));
		} else {
			setPrintEnabled(std::get<1>(options.printEnabled));
		}
	}
	for (auto simulation : simulations) {
		simulation->finishSceneContext();
	}
	//context->setRayGenerationProgram(OPAL_RAY_REFLECTION, defaultPrograms.at("rayGeneration")); //Generation
	//context->setMissProgram(OPAL_RAY_REFLECTION, defaultPrograms.at("miss"));
	//
	//#ifdef OPAL_LOG_TRACE
	//context->setRayGenerationProgram(OPAL_RAY_LOG_TRACE_RAY, defaultPrograms.at("rayGenerationLogTrace")); //Generation
	//context->setMissProgram(OPAL_RAY_LOG_TRACE_RAY, defaultPrograms.at("missLogTrace"));

	//#endif


	std::cout<<"Validating the context ..."<< std::endl;
	try {
		context->validate();
	}	catch (optix::Exception& e) {
			std::cout << "Validating context error occurred with error code "
				<< e.getErrorCode() << " and message "
				<< e.getErrorString() << std::endl;
		throw;

	}
	sceneFinished = true;
	std::cout<<"--- Check your configuration and assumptions --"<<std::endl;
	for (auto simulation : simulations) {
		configInfo<<simulation->printConfigInfo();
		configInfo<<"\t - CUDA programs directory: "<<simulation->getCudaProgramsDirectory()<<std::endl;
	}
	configInfo<<printSceneReport();
	std::cout<<configInfo.str()<<std::endl;
}

optix::Program OpalSceneManager::getDefaultProgram(std::string p) {
	return defaultPrograms.at(p);
}
void OpalSceneManager::enableExceptions() {
	context->setExceptionEnabled(RT_EXCEPTION_ALL, true);
}

std::string opal::OpalSceneManager::printSceneReport()  {
	std::ostringstream stream;
	stream << "--- Scene Summary ---" << std::endl;
	stream << "\t numberOfFaces=" << numberOfFaces <<". globalFaces = "<<globalFaces.size()<< ". minEpsilon= " << minEpsilon << ". maxReflections=" << maxReflections << ". useExactSpeedOfLight="<<this->options.useExactSpeedOfLight<<std::endl;
	stream << "\t Receivers=" << receivers.size() <<  std::endl;
	stream << "\t RayCount=" << raySphere.rayCount << ". azimuthSteps=" << raySphere.azimuthSteps << ". elevationSteps=" << raySphere.elevationSteps << std::endl;
	if (options.usePenetration) {
		stream << "\t **Options: Penetration enabled. attenuationLimit (dB)=" << attenuationLimit<< std::endl;
	}
	if (options.useDepolarization) {
		stream << "\t **Options: Depolarization enabled."<< std::endl;
	}
	if (options.generateRaysOnLaunch) {
		stream << "\t **Options: Generate rays on launch enabled."<< std::endl;
	}
	if (options.useFastMath) {
		stream << "\t **Options: Fast Math enabled."<< std::endl;
	}
	if (options.useMultiGPU) {
		stream << "\t **Options: Multi GPU enabled."<< std::endl;
	}
	if (options.useMultiChannel) {
		stream << "\t **Options: Multi channel enabled."<< std::endl;
	}
	stream << "-- Scene Graph" << std::endl;
	stream << "\t rootGroup. Children=" << rootGroup->getChildCount() << std::endl;
	stream << "\t max_interactions=" << context["max_interactions"]->getUint() << std::endl;
	optix::GeometryGroup gg  = rootGroup->getChild<optix::GeometryGroup>(0);
	//Static meshes
	 
	for (unsigned int i = 0; i <gg->getChildCount(); i++)
	{
		optix::GeometryInstance gi = gg->getChild(i);
		MaterialEMProperties em;
		gi["EMProperties"]->getUserData(sizeof(MaterialEMProperties), &em);

		stream << "\t StaticMesh[" << i << "].EMProperties=" << em.dielectricConstant<< std::endl;
	}
	stream << "\t Static meshes loaded=" << gg->getChildCount() << std::endl;
	
	//Face ids
	stream<<"\t Face ids used in scene: " <<globalFaces.size()<<std::endl;;
//	for (auto f : globalFaces) {
//		stream<<f<<",";
//	}	
//	stream<<std::endl;
	//Edges
	stream<<"\t Number of edges loaded: " <<edges.size()<<std::endl;

	//Receivers
	gg = rootGroup->getChild<optix::GeometryGroup>(1);
	for (unsigned int i = 0; i < gg->getChildCount(); i++)
	{
		optix::GeometryInstance gi = gg->getChild(i);
		stream << "\t Receiver[" << i << "].sphere=" << gi["sphere"]->getFloat4()<<".internalBufferId="<<gi["receiverBufferIndex"]->getUint()<<".externalId="<< gi["externalId"]->getInt() << std::endl;
	}
	for (unsigned int i = 2; i < rootGroup->getChildCount(); i++)
	{
		optix::Transform t = rootGroup->getChild<optix::Transform>(i);
		gg= t->getChild<optix::GeometryGroup>();
		stream << "\t DynamicMesh[" << (i - 2) << "].children=" << gg->getChildCount() << std::endl;
	}
	stream<<printInternalBuffersState()<<std::endl;
	for (auto simulation : simulations) {
		stream<<simulation->printInternalBuffersState()<<std::endl;
	}
	stream<<printContextInformation()<<std::endl;
	stream << "-----" << std::endl;
	return stream.str();

}
std::string OpalSceneManager::printInternalBuffersState()
{
	std::ostringstream stream;
	unsigned long long totalBytes = 0;
	unsigned long long sb;
	stream << "--OpalSceneManager Internal buffers--" << std::endl;
	RTsize w;
	RTsize h;
	RTsize d;
	if (txOriginBuffer) {
		txOriginBuffer->getSize(w);
		sb = sizeof(Transmitter)*w;
		totalBytes += sb;
		stream << "\t txOriginBuffer=(" << w <<"). size=" << (sb / 1024.f) << " KiB" << std::endl;
	}
	if (raySphere.raySphereBuffer) {
		raySphere.raySphereBuffer->getSize(w,d);
		sb = sizeof(float3)*w*d;
		totalBytes += sb;
		stream << "\t raySphereBuffer=(" << w <<","<<d<<"). size=" << (sb / 1024.f) << " KiB" << std::endl;
	}
	//Check memory usage
	stream << "\t Total memory in OpalSceneManager internal buffers:  " << (totalBytes / (1024.f*1024.f)) << " MiB" << std::endl;
	return stream.str();
}
void opal::OpalSceneManager::setPrintEnabled(int bufferSize)
{

	options.printEnabled= std::make_tuple(true, bufferSize,false, make_uint3(0u));
	if (context) {
		context->setPrintEnabled(true);
		context->setPrintBufferSize(bufferSize);
		std::cout << "printEnabled" << std::endl;
	}


}
void opal::OpalSceneManager::setPrintEnabled(int bufferSize, optix::uint3 index)
{

	options.printEnabled= std::make_tuple(true, bufferSize, true, index);
	if (context) {
		context->setPrintEnabled(true);
		context->setPrintBufferSize(bufferSize);
		std::cout << "printEnabled" << std::endl;
		//Set a particular print index. Comment out if necessary
		context->setPrintLaunchIndex(index.x,index.y,index.z);
		std::cout<<"Showing launch index "<<index<<std::endl;
	} 

}

uint OpalSceneManager::getNumberOfFaces() {
	return numberOfFaces;
}

void OpalSceneManager::setInitialHash(uint h) {
	context["initialHash"]->setUint(h);
}
void OpalSceneManager::setUsageReport()
{
	context->setUsageReportCallback(this->callbackUsageReport, 3, NULL);
}

void OpalSceneManager::buildSceneGraph()
{

	// Create geometry group for static meshes
	staticMeshesGroup = context ->createGeometryGroup();
	staticMeshesGroup->setChildCount(static_cast<uint>(staticMeshes.size()));

	for (unsigned int i = 0; i < staticMeshes.size(); i++)
	{
		staticMeshesGroup->setChild(i, staticMeshes[i].geom_instance);

	}
	staticMeshesGroup->setAcceleration(context->createAcceleration("Trbvh"));
	//context["staticMeshes"]->set(staticMeshesGroup);
#ifdef OPAL_USE_TRI
	staticMeshesGroup->setVisibilityMask(OPAL_STATIC_MESH_MASK);
	//staticMeshesGroup->setVisibilityMask(0u);
	//std::cout<<"setVisibilityMask="<<OPAL_STATIC_MESH_MASK<<"RT_VISIBILITY_ALL="<<RT_VISIBILITY_ALL<<std::endl;
#endif
	context["staticMeshesRoot"]->set(staticMeshesGroup);


	//Create Geometry group for receivers
	receiversGroup = context->createGeometryGroup();
	receiversGroup->setChildCount(static_cast<uint>(receivers.size()));
	unsigned int cindex = 0;
	for (const auto val : receivers)
	{
		receiversGroup->setChild(cindex, val->geomInstance);
		++cindex;

	}
	receiversGroup->setAcceleration(context->createAcceleration("Trbvh"));
#ifdef OPAL_USE_TRI
	receiversGroup->setVisibilityMask(OPAL_RECEIVER_MASK);
#endif
	context["receivers"]->set(receiversGroup);



	
	//Create root
	rootGroup = context->createGroup();
	//rootGroup->setChildCount(2+ statc);
	rootGroup->addChild(staticMeshesGroup);
	rootGroup->addChild(receiversGroup);
	//Group g = context->createGroup();
	//g->setAcceleration(context->createAcceleration("NoAccel"));
	//rootGroup->addChild(g);
	//Add dynamic meshes
	for (auto val : dynamicMeshes)
	{
		val.second->childIndex = rootGroup->addChild(val.second->transform);
		//val.second->transform->setChild(val.second->geom_group);
	}

	rootGroup->setAcceleration(context->createAcceleration("Trbvh")); //All Groups and GeometryGroups MUST have an acceleration
	//rootGroup->setVisibilityMask(RT_VISIBILITY_ALL);
	//std::cout<<"static meshes="<<staticMeshesGroup->getVisibilityMask()<<"receiver="<<receiversGroup->getVisibilityMask()<<"root Mask"<<rootGroup->getVisibilityMask()<<std::endl;
	//std::cout<<"static meshes*OPAL="<<(staticMeshesGroup->getVisibilityMask() & OPAL_STATIC_MESH_MASK)<<"RT_VISIBILITY_ALL*OPAL_STATIC_MESH_MASK"<<(staticMeshesGroup->getVisibilityMask() & RT_VISIBILITY_ALL)<<std::endl;
	context["root"]->set(rootGroup);

	sceneGraphCreated = true;


}
void OpalSceneManager::callbackUsageReport(int level, const char* tag, const char* msg, void* cbdata)
{
	std::cout << "[" << level << "][" << std::left << std::setw(12) << tag << "] " << msg;
}
void OpalSceneManager::disableGenerateRaysOnLaunch() {
	if (contextInitialized) {
		//We would need to rebuild the whole scene and context. It is possible but for now, we consider it an error
		throw  opal::Exception("Do not disable  generate rays on launch after creating the context");
		return;
	}
	//Not implemented yet. 
	options.generateRaysOnLaunch=false;
	//throw  opal::Exception(" Generate Rays On Launch not implemented yet");

}

void OpalSceneManager::enableGenerateRaysOnLaunch() {
	if (contextInitialized) {
		//We would need to rebuild the whole scene and context. It is possible but for now, we consider it an error
		throw  opal::Exception("Do not enable  generate rays on launch after creating the context");
		return;
	}
	options.generateRaysOnLaunch=true;
	//throw  opal::Exception(" Generate Rays On Launch not implemented yet");

}

void OpalSceneManager::disableFastMath() {
	if (contextInitialized) {
		//We would need to rebuild the whole scene and context. It is possible but for now, we consider it an error
		throw  opal::Exception("Do not enable  FAST MATH after creating the context");
		return;
	}
	options.useFastMath=false;

}
void OpalSceneManager::enableFastMath() {
	if (contextInitialized) {
		//We would need to rebuild the whole scene and context. It is possible but for now, we consider it an error
		throw  opal::Exception("Do not enable  FAST MATH after creating the context");
		return;
	}
	options.useFastMath=true;

}
void OpalSceneManager::enableMultitransmitter() {
	this->transmitterManager = new TransmitterManager(this);
}
ConfigurationOptions OpalSceneManager::getConfigurationOptions() const {
	return options;
}
TransmitterManager* OpalSceneManager::getTransmitterManager() {
	if (transmitterManager) {
		return transmitterManager;
	} else {
		throw  opal::Exception("getTransmitterManager(): multitransmitter has not been enabled previously");

	}
}
void OpalSceneManager::enableMultiGPU() {
	if (contextInitialized) {
		//It is actually perfectly safe to enable GPU devices after creating context, but for now we consider it an error
		throw  opal::Exception("Do not enable multi GPU support after creating the context");
		return;

	}
	options.useMultiGPU=true;
}
void OpalSceneManager::disableMultiGPU() {
	if (contextInitialized) {
		//It is actually perfectly safe to enable GPU devices after creating context, but for now we consider it an error
		throw  opal::Exception("Do not disable multi GPU support after creating the context");
		return;

	}
	options.useMultiGPU=false;
}
void OpalSceneManager::enablePenetration() {
	if (contextInitialized) {
		//We would need to rebuild the whole scene and context. It is possible but for now, we consider it an error
		throw  opal::Exception("Do not enable  penetration after creating the context");
		return;
	}
	options.usePenetration=true;	
}
void OpalSceneManager::disablePenetration() {
	if (contextInitialized) {
		//We would need to rebuild the whole scene and context. It is possible but for now, we consider it an error
		throw  opal::Exception("Do not disable  penetration after creating the context");
		return;
	}
	options.usePenetration=false;	
}
void OpalSceneManager::enableDepolarization() {
	if (contextInitialized) {
		//We would need to rebuild the whole scene and context. It is possible but for now, we consider it an error
		throw  opal::Exception("Do not enable  penetration after creating the context");
		return;
	}
	options.useDepolarization=true;	
}
void OpalSceneManager::disableDepolarization() {
	if (contextInitialized) {
		//We would need to rebuild the whole scene and context. It is possible but for now, we consider it an error
		throw  opal::Exception("Do not disable  depolarization after creating the context");
		return;
	}
	options.useDepolarization=false;	
}
void OpalSceneManager::enableMultiChannel() {
	if (contextInitialized) {
		//We would need to rebuild the whole scene and context. It is possible but for now, we consider it an error
		throw  opal::Exception("Do not enable  multichannel after creating the context");
		return;
	}
	options.useMultiChannel = true;
}
void OpalSceneManager::disableMultiChannel() {
	if (contextInitialized) {
		//We would need to rebuild the whole scene and context. It is possible but for now, we consider it an error
		throw  opal::Exception("Do not disable  multichannel after creating the context");
		return;
	}
	options.useMultiChannel = false;
}
void OpalSceneManager::setAttenuationLimit(float a) {
	attenuationLimit=a;
}
void OpalSceneManager::setMinEpsilon(float e) {
	if (e<0) {
		throw  opal::Exception("setMinEpsilon(): epsilon cannot be negative");

	}
	minEpsilon=e;
	if (sceneFinished) {
		context["min_t_epsilon"]->setFloat(minEpsilon);
	}

}
optix::Program OpalSceneManager::createBoundingBoxTriangle()
{
	return context->createProgramFromPTXString(ptxHandler->getPtxString(currentDir.c_str(), "optixTriangle.cu"), "boundsTriangle");
}

#ifdef OPAL_USE_TRI
optix::Program OpalSceneManager::createTriangleAttributesProgram()
{
	return  context->createProgramFromPTXString(ptxHandler->getPtxString(currentDir.c_str(),"optixGeometryTriangles.cu"), "triangle_attributes" ) ;

}
optix::Program OpalSceneManager::createCurvedTriangleAttributesProgram()
{
	return context->createProgramFromPTXString(ptxHandler->getPtxString(currentDir.c_str(),"optixGeometryTrianglesCurved.cu"), "triangle_attributes_curved" ) ;

}
#endif
optix::Program OpalSceneManager::createIntersectionTriangle()
{
	return context->createProgramFromPTXString(ptxHandler->getPtxString(currentDir.c_str(), "optixTriangle.cu"), "intersectTriangle");

}

optix::Program OpalSceneManager::createBoundingBoxSphere()
{


	return context->createProgramFromPTXString(ptxHandler->getPtxString(currentDir.c_str(), "sphere.cu"), "boundsSphere");

}
optix::Program OpalSceneManager::createExceptionReflectionProgram()
{


	return context->createProgramFromPTXString(ptxHandler->getPtxString(currentDir.c_str(), "exception.cu"), "exception");

}


optix::Program OpalSceneManager::createIntersectionSphere()
{


	return context->createProgramFromPTXString(ptxHandler->getPtxString(currentDir.c_str(), "sphere.cu"), "rtgem_intersectSphere");

}


const std::map<int, OpalDynamicMeshGroup*> &  OpalSceneManager::getDynamicMeshes()  const {
	return dynamicMeshes;
}

ChannelParameters OpalSceneManager::getChannelParameters() const {
	return defaultChannel;
}
std::string OpalSceneManager::getBaseDirectory() const {
	return baseDir;
}
void OpalSceneManager::computeTotalReceivedPower(bool callCallback) {
	//std::cout<<"computeTotalReceivedPower()"<<std::endl;
	//So far all simulations are assumed to use the same compute mode. Otherwise an exception is thrown when checking the mode
	ComputeMode mode=simulations[0]->getComputeMode();
	auto t=info->getE();
	auto itt=t.begin();
	std::vector<SphereReceiver*> receivers	=getReceivers(); 	

	while (itt!=t.end()) {	
		Transmitter* currentTx=getTransmitter(itt->first);
		//std::cout<<"currentTx="<<currentTx->externalId<<std::endl;
		unsigned int txIndex=getTransmitterIndex(itt->first);
		auto itrx=itt->second->begin();
		while (itrx!=itt->second->end()) {
			if (mode==ComputeMode::FIELD) {
				printFieldComponents(itrx->second.Ex,itrx->second.Ey, itrx->second.Ez,itrx->second.index,txIndex, itrx->second.hits);
			} else {
				unsigned int rxIndex=itrx->second.index;
				float p=computeReceivedPower(itrx->second.E,rxIndex,txIndex, itrx->second.hits);
				//computeReceivedPower(itrx->second.E,itrx->second.index,currentTx->externalId,currentTx->txPower,currentTx->origin, itrx->second.hits);
				if (callCallback) {
					//We do not call calback for our own transmitter
					if (receivers[rxIndex]->callback && receivers[rxIndex]->externalId != currentTx->externalId) {
						receivers[rxIndex]->callback(p,currentTx->externalId);
					}

				}
			}
			++itrx;
		}
		++itt;	 
	}
	//	computeReceivedPower(E,index,activeTransmitters[currentTx]->externalId,activeTransmitters[currentTx]->txPower,activeTransmitters[currentTx]->origin, raysHit);
}
float OpalSceneManager::computeReceivedPower(optix::float2 E, unsigned int index, unsigned int txIndex,  uint raysHit) {
	std::vector<Transmitter*> activeTransmitters = getActiveTransmitters();
	float3 origin = make_float3(activeTransmitters[txIndex]->origin_p);
	float txPower=activeTransmitters[txIndex]->origin_p.w;
	uint txId=activeTransmitters[txIndex]->externalId; 
	float k= activeTransmitters[txIndex]->polarization_k.w;
	float eA= 1.0f / (4*k*k); //Effective Area of the antenna (lambda/4*pi)^2
	float power = eA*((E.x*E.x) + (E.y*E.y))*txPower;
	std::vector<SphereReceiver*> receivers	=getReceivers(); 	
	std::cout << "rx["<<receivers[index]->externalId<<"]=" << receivers[index]->position << ".r=" << receivers[index]->radius << "(sphere="<<receivers[index]->geomInstance["sphere"]->getFloat4()<<"); tx["<<txId<<"]=" << origin << " eA=" << eA << " txPower=" << txPower << " E=(" << E.x << "," << E.y << ")" << " p=" << power <<  " d=" << length(origin - receivers[index]->position) << std::endl;
	float radius=receivers[index]->radius;
	//Update to let callbacks be called later
	//receivers[index]->lastPower=power;
	//receivers[index]->lastField=E;
	receivers[index]->setLastReceivedResult(txId,make_float3(power,E.x,E.y));
	//Call callbacks...add your own
	//	receivers[index]->callback(power, txId);
	//Power
	std::cout<<std::setprecision(10)<<"TPR\t"<<power<<"\t"<<receivers[index]->externalId<<"\t"<<receivers[index]->position.x<<"\t"<<receivers[index]->position.y <<"\t"<<receivers[index]->position.z<<"\t"<<raysHit<<"\t"<<radius<<"\t"<<length(origin - receivers[index]->position)<<"\t"<<txId<< std::endl;

	//E
	std::cout<<std::setprecision(10)<<"TER\t"<<E.x<<"\t"<<E.y<<"\t"<<receivers[index]->externalId<<"\t"<<receivers[index]->position.x<<"\t"<<receivers[index]->position.y<<"\t"<<receivers[index]->position.z<<"\t"<<raysHit<< "\t"<<radius<<"\t"<<length(origin - receivers[index]->position)<<"\t"<<txId<< std::endl;
	return power;
}
float OpalSceneManager::getReceivedPower(int rxId, int txId) {

	SphereReceiver* r = getReceiver(rxId);
	return r->getLastReceivedPower(txId);
}
EComponents OpalSceneManager::getReceivedE(int rxId, int txId) {

	SphereReceiver* r = getReceiver(rxId);
	return r->getLastReceivedE(txId);
}


//Mainly for debug
void OpalSceneManager::printFieldComponents(optix::float2 Ex, optix::float2 Ey, optix::float2 Ez, unsigned int index,unsigned int txIndex, uint raysHit) {
	//E
	std::vector<SphereReceiver*>	receivers=getReceivers(); 	
	float radius=(receivers[index]->radius);
	//float area=4*M_PI*radius*radius;
	EComponents e;
	e.Ex=Ex;
	e.Ey=Ey;
	e.Ez=Ez;
	std::vector<Transmitter*> activeTransmitters = getActiveTransmitters();
	uint txId=activeTransmitters[txIndex]->externalId; 
	receivers[index]->setLastReceivedE(txId,e);	

	std::cout<<"TERX\t"<<Ex.x<<"\t"<<Ex.y<<"\t"<<receivers[index]->externalId<<"\t"<<receivers[index]->position.x<<"\t"<<receivers[index]->position.y<<"\t"<<receivers[index]->position.z<<"\t"<<raysHit<<"\t"<<radius<< std::endl;
	std::cout<<"TPRX\t"<<(10*log10(dot(Ex,Ex)))<<"\t"<<receivers[index]->externalId<<"\t"<<receivers[index]->position.x<<"\t"<<receivers[index]->position.y<<"\t"<<receivers[index]->position.z<<"\t"<<raysHit<<"\t"<<radius<< std::endl;
	std::cout<<"TERY\t"<<Ey.x<<"\t"<<Ey.y<<"\t"<<receivers[index]->externalId<<"\t"<<receivers[index]->position.x<<"\t"<<receivers[index]->position.y<<"\t"<<receivers[index]->position.z<<"\t"<<raysHit<<"\t"<<radius<< std::endl;
	std::cout<<"TPRY\t"<<(10*log10(dot(Ey,Ey)))<<"\t"<<receivers[index]->externalId<<"\t"<<receivers[index]->position.x<<"\t"<<receivers[index]->position.y<<"\t"<<receivers[index]->position.z<<"\t"<<raysHit<<"\t"<<radius<< std::endl;
	std::cout<<"TERZ\t"<<Ez.x<<"\t"<<Ez.y<<"\t"<<receivers[index]->externalId<<"\t"<<receivers[index]->position.x<<"\t"<<receivers[index]->position.y<<"\t"<<receivers[index]->position.z<<"\t"<<raysHit<<"\t"<<radius<< std::endl;
	std::cout<<"TPRZ\t"<<(10*log10(dot(Ez,Ez)))<<"\t"<<receivers[index]->externalId<<"\t"<<receivers[index]->position.x<<"\t"<<receivers[index]->position.y<<"\t"<<receivers[index]->position.z<<"\t"<<raysHit<<"\t"<<radius<< std::endl;

}


// Copyright NVIDIA Corporation 2011
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//  * Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the
//    documentation and/or other materials provided with the distribution.
//  * Neither the name of NVIDIA CORPORATION nor the names of its
//    contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS ``AS IS'' AND ANY
// EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
// PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR
// CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
// EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
// PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
// PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY
// OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

//From optix advanced samples (see licence): https://github.com/nvpro-samples/optix_advanced_samples/blob/master/src/optixIntroduction/optixIntro_07/inc/Application.h
// For rtDevice*() function error checking. No OptiX context present at that time.
#define RT_CHECK_ERROR_NO_CONTEXT( func ) \
	do { \
		RTresult code = func; \
		if (code != RT_SUCCESS) \
		std::cerr << "ERROR: Function " << #func << std::endl; \
	} while (0)


//From NVIDIA (see license): https://github.com/nvpro-samples/optix_advanced_samples/blob/master/src/optixIntroduction/optixIntro_07/src/Application.cpp
void SystemInformation::getSystemInformation()
{
	RT_CHECK_ERROR_NO_CONTEXT(rtGetVersion(&optixVersion));


	major = optixVersion / 1000; // Check major with old formula.
	minor;
	micro;
	if (3 < major) // New encoding since OptiX 4.0.0 to get two digits micro numbers?
	{
		major =  optixVersion / 10000;
		minor = (optixVersion % 10000) / 100;
		micro =  optixVersion % 100;
	}
	else // Old encoding with only one digit for the micro number.
	{
		minor = (optixVersion % 1000) / 10;
		micro =  optixVersion % 10;
	}

	numberOfDevices = 0;
	RT_CHECK_ERROR_NO_CONTEXT(rtDeviceGetDeviceCount(&numberOfDevices));

	for (unsigned int i = 0; i < numberOfDevices; ++i)
	{ 

		DeviceInformation d;	
		RT_CHECK_ERROR_NO_CONTEXT(rtDeviceGetAttribute(i, RT_DEVICE_ATTRIBUTE_NAME, sizeof(d.name), d.name));

		RT_CHECK_ERROR_NO_CONTEXT(rtDeviceGetAttribute(i, RT_DEVICE_ATTRIBUTE_COMPUTE_CAPABILITY, sizeof(d.computeCapability), &d.computeCapability));

		RT_CHECK_ERROR_NO_CONTEXT(rtDeviceGetAttribute(i, RT_DEVICE_ATTRIBUTE_TOTAL_MEMORY, sizeof(d.totalMemory), &d.totalMemory));

		RT_CHECK_ERROR_NO_CONTEXT(rtDeviceGetAttribute(i, RT_DEVICE_ATTRIBUTE_CLOCK_RATE, sizeof(d.clockRate), &d.clockRate));

		RT_CHECK_ERROR_NO_CONTEXT(rtDeviceGetAttribute(i, RT_DEVICE_ATTRIBUTE_MAX_THREADS_PER_BLOCK, sizeof(d.maxThreadsPerBlock), &d.maxThreadsPerBlock));

		RT_CHECK_ERROR_NO_CONTEXT(rtDeviceGetAttribute(i, RT_DEVICE_ATTRIBUTE_MULTIPROCESSOR_COUNT, sizeof(d.smCount), &d.smCount));

		RT_CHECK_ERROR_NO_CONTEXT(rtDeviceGetAttribute(i, RT_DEVICE_ATTRIBUTE_EXECUTION_TIMEOUT_ENABLED, sizeof(d.executionTimeoutEnabled), &d.executionTimeoutEnabled));

		RT_CHECK_ERROR_NO_CONTEXT(rtDeviceGetAttribute(i, RT_DEVICE_ATTRIBUTE_MAX_HARDWARE_TEXTURE_COUNT, sizeof(d.maxHardwareTextureCount), &d.maxHardwareTextureCount));

		RT_CHECK_ERROR_NO_CONTEXT(rtDeviceGetAttribute(i, RT_DEVICE_ATTRIBUTE_TCC_DRIVER, sizeof(d.tccDriver), &d.tccDriver));

		RT_CHECK_ERROR_NO_CONTEXT(rtDeviceGetAttribute(i, RT_DEVICE_ATTRIBUTE_CUDA_DEVICE_ORDINAL, sizeof(d.cudaDeviceOrdinal), &d.cudaDeviceOrdinal));
		devices.push_back(d);
	}
}

std::string SystemInformation::printSystemInformation() {
	std::ostringstream stream;
	stream << "-- System information --  " << std::endl;
	stream << "OptiX " << major << "." << minor << "." << micro << std::endl;
	stream << "Number of Devices = " << numberOfDevices << std::endl << std::endl;
	for (unsigned int i = 0; i < numberOfDevices; ++i)
	{
		DeviceInformation d=devices[i]; 
		stream << "Device " << i << ": " << d.name << std::endl;
		stream << "  Compute Support: " << d.computeCapability[0] << "." << d.computeCapability[1] << std::endl;
		stream << "  Total Memory: " << (unsigned long long) d.totalMemory << std::endl;
		stream << "  Clock Rate: " << d.clockRate << " kHz" << std::endl;
		stream << "  Max. Threads per Block: " << d.maxThreadsPerBlock << std::endl;
		stream << "  Streaming Multiprocessor Count: " << d.smCount << std::endl;
		stream << "  Execution Timeout Enabled: " << d.executionTimeoutEnabled << std::endl;
		stream << "  Max. Hardware Texture Count: " << d.maxHardwareTextureCount << std::endl;
		stream << "  TCC Driver enabled: " << d.tccDriver << std::endl;
		stream << "  CUDA Device Ordinal: " << d.cudaDeviceOrdinal << std::endl ;
	}
	stream << "------  " << std::endl;
	return stream.str();
}

std::vector<float4>  OpalSceneManager::loadPDFromFile(const char* file) {
	std::ifstream infile(file);
	if (!infile.good()) {
		std::cout<<"Error opening "<<file<<std::endl;
		throw  opal::Exception("loadPDFromFile(): error opening file");
		
	} 
	float x, y, z,w;
	//char c;
	std::vector<float4> pd;
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
			if (p == 3) {
				w = std::stof(line.substr(current, next - current));
			}

			//std::cout << line.substr(current, next - current) <<"\t"<< std::endl;
			p++;
		} while (next != std::string::npos);

		pd.push_back(make_float4(x, y, z,w));
	}
	std::cout << "Loaded " << pd.size() << " principal directions from " << file << std::endl;
	infile.close();
	if (pd.size()==0) {
		std::cout<<"WARNING: loaded an  mesh with empty curvature information!!!"<<std::endl;
	}
	return pd;
}
//Antenna gains must come as a matrix of azimuth (rows)/elevation (columns) gains in dB power, with columns separated by tabs.Ranges of azimuth go from [0, 2pi[ and elevation from [0, pi]. 
//So a file with 360*181 values correspond to steps of 1 degree and a matrix of 3600*1800 values correspond to steps of 0.1 degrees
//We assume that elevations are measured from Y axis (UP) and azimuth from Z to X clockwise in our left-hand reference system X right, Y up, Z forward.
//Since the gain is applied to the EM field, the values are passed to linear and then the square root is taken 
AntennaGain OpalSceneManager::loadGainsFromFileIndBPower(const char* file) {
	std::ifstream infile(file);
	if (!infile.good()) {
		std::cout<<"Error opening "<<file<<std::endl;
		throw  opal::Exception("loadGainsFromFileIndBPower: error opening file");
		
	} 
	float x, y;
	//char c;
	AntennaGain gains;
	std::string line;
	std::string delimiters("\t");
	unsigned int azi=0;
	unsigned int eli=0;
	for (std::string line; std::getline(infile, line); ) {
		std::vector<float> az;
		std::istringstream iline(line);
		std::string val;
		//std::cout<<line<<std::endl;
		while(std::getline(iline,val,'\t')) {
			float db= std::stof(val);
		
			float g=pow(10, db/20.0f); 
			//std::cout<<"\t"<<val<<"db"<<db<<"g="<<g<<std::endl;
			if (isnan(g)) {
				std::cout<<"\t"<<val<<"db"<<db<<"g="<<g<<"azi="<<azi<<"eli"<<eli<<std::endl;
				throw  opal::Exception("loadGainsFromFileIndBPower(): value is nan");
			}
	       		az.push_back(g);
			++eli;
		}
		gains.push_back(az);
		++azi;
    	}	
	
	if (gains.size()==0) {
		std::cout<<"WARNING: loaded an empty gaing!!!"<<std::endl;
		throw  opal::Exception("loadGainsFromFileIndBPower(): empty gain file");
	} else {
		std::cout << "--- Antenna Gain **  " <<std::endl;
		std::cout << "\t Loaded " << gains.size() << " azimuth values and "<<gains[0].size() <<"elevation values  from " << file << std::endl;
	}
	infile.close();
	return gains;

}
std::vector<float3>  OpalSceneManager::loadRaysFromFile(const char* file) {
	std::ifstream infile(file);
	if (!infile.good()) {
		std::cout<<"Error opening "<<file<<std::endl;
		throw  opal::Exception("loadRaysFromFile(): error opening file");
		
	} 
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

std::vector<float3>  OpalSceneManager::loadVerticesFromFile(const char* file) {
	std::ifstream infile(file);
	if (!infile.good()) {
		std::cout<<"Error opening "<<file<<std::endl;
		throw  opal::Exception("loadVerticesFromFile(): error opening file");
		
	} 
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
	std::cout << "Loaded " << vertices.size() << " vertices from " << file << std::endl;
	infile.close();
	if (vertices.size()==0) {
		std::cout<<"WARNING: loaded an empty mesh!!!"<<std::endl;
	}
	return vertices;
}
std::vector<int>  OpalSceneManager::loadTrianglesFromFile(const char* file) {
	std::ifstream infile(file);
	if (!infile.good()) {
		std::cout<<"Error opening "<<file<<std::endl;
		throw  opal::Exception("loadTrianglesFromFile(): error opening file");
		
	} 
	int i;
	std::vector<int> triangles;

	while (infile>>i) {
		//std::cout << i << std::endl;
		triangles.push_back(i);
	}
	std::cout << "Loaded " << triangles.size() << "indices from " << file << std::endl;
	infile.close();
	return triangles;
}
void  OpalSceneManager::writeMeshToPLYFile(std::string fileName, std::vector<float3>& vertices, std::vector<int>& indices, Matrix4x4& transformationMatrix) {
	std::ofstream outFile(fileName.c_str(),std::ofstream::out);

	if ( !outFile )
	{
		throw  opal::Exception("writeMeshToPLYFile(): Error opening output file");
	}

	////
	// Header
	////

	std::vector<optix::float3> transformedVertices(vertices.size());
	//Apply first transformation matrix
	for (size_t i = 0; i < vertices.size(); i++)
	{

		//std::cout <<"vertex=("<< meshVertices[i].x <<","<< meshVertices[i].y <<","<< meshVertices[i].z <<")"<< std::endl;
		const float3 v = optix::make_float3(transformationMatrix*optix::make_float4(vertices[i], 1.0f));
		std::cout<<"v="<<v<<std::endl;
		transformedVertices[i]=v;
	}

	outFile << "ply" << std::endl;
	outFile << "format ascii 1.0" << std::endl;
	outFile << "element vertex " << vertices.size() << std::endl;
	outFile << "property float x" << std::endl;
	outFile << "property float y" << std::endl;
	outFile << "property float z" << std::endl;
	outFile << "element face " << (indices.size()/3) << std::endl;
	outFile << "property list uchar int vertex_indices" << std::endl;
	outFile << "end_header" << std::endl;
	for (auto p : transformedVertices) {
		outFile<< p.x <<" "<<p.y<<" "<<p.z<<std::endl;
	}
	for (int i=0; i<indices.size(); i=i+3) {
		outFile<<"3 "<<indices[i]<<" "<<indices[i+1]<<" "<<indices[i+2]<<std::endl;

	}
	outFile.close();


}

void OpalSceneManager::printPower(float power, int txId ) {
	std::cout << "PR\t" << power << std::endl;
}



