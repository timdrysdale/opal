/***************************************************************/
//
//Copyright (c) 2019 Esteban Egea-Lopez http://ait.upct.es/eegea
//
/**************************************************************/


#include "timer.h"
#include "tutils.h"
#include "Opal.h"
#include "Common.h"
#include <optixu/optixu_math_namespace.h>
#include <optixu/optixu_matrix_namespace.h> 
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


using namespace opal;
using namespace optix;
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
	initContext(f, useExactSpeedOfLight);

}

opal::OpalSceneManager::~OpalSceneManager()
{
	try {
		if (context) {
			context->destroy();
		}
		context=nullptr;

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
		useMultiGPU=false;
	} else {
		useMultiGPU=true;
	}
	this->context=nullptr;
	this->deg2rad=M_PI/180.f;
	//Change to your own directory
	this->cudaProgramsDir="opal";
	this->maxReflections = 10u;
	this->minEpsilon = 1.e-3f;
	this->useExactSpeedOfLight=true;

	//Increased every time a new face is added to the scene.  
	this->numberOfFaces = 0u; 
	this->sceneGraphCreated = false;
	this->sceneFinished = false;

	this->globalHitInfoBuffer = nullptr;
	this->resultHitInfoBuffer = nullptr;
	this->txOriginBuffer = nullptr;
	this->atomicIndexBuffer=nullptr;

	this->radioReductionFraction=1.0/sqrt(2); //To avoid transmitters inside reception sphere
	this->usePenetration=false;
	this->useDepolarization=false;
	this->attenuationLimit = -80.0f;
}
void OpalSceneManager::initContext(float f,  bool useExactSpeedOfLight) {
#ifdef OPALDEBUG
	outputFile.open("opal_log.txt");
#endif // OPALDEBUG
	if (useDepolarization) {
		this->cudaProgramsDir="opal/polarization";
	} else {
		//Show warning to remark the assumptions
		configInfo<<"\t - You are assuming that: (1)  both transmitters and receivers have the same polarization and (2) the polarization is either purely vertical (0, 1, 0) or horizontal (1,0,0) or (0,0,1). " <<std::endl; 
		configInfo<<"\t   If this is not the intended behaviour, check the use of depolarization, though it has a performance cost. " <<std::endl; 
	}
	configInfo<<"\t - CUDA programs directory: "<<cudaProgramsDir<<std::endl;
	this->useExactSpeedOfLight=useExactSpeedOfLight;
	setFrequency(f);
	createSceneContext();
	setDefaultPrograms();
}	

void OpalSceneManager::setFrequency(float f)
{
	this->defaultChannel.frequency = f;
	if (useExactSpeedOfLight) {
		this->defaultChannel.waveLength = 299792458.0f / f; // c/f
		configInfo<<"\t - Speed of light, c= 299792458.0 m/s"<<std::endl;
	} else {
		this->defaultChannel.waveLength = 3.0e8f / f;
		configInfo<<"\t - Speed of light, c= 3e8 m/s"<<std::endl;
	}
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
	if (!useMultiGPU) {
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
	context->setRayTypeCount(1); //Normal  ray
	context->setEntryPointCount(1); //1 program: ray generation 

}

void opal::OpalSceneManager::setDefaultPrograms()
{


	//TODO: we could add an intersection program for planes (ideal infinite planes), typically used to represent flat grounds, should be more efficient than a mesh?
	defaultPrograms.insert(std::pair<std::string, optix::Program>("meshClosestHit", createClosestHitMesh()));
#ifdef OPAL_USE_TRI
	defaultPrograms.insert(std::pair<std::string, optix::Program>("triangleAttributes", createTriangleAttributesProgram()));
#else
	defaultPrograms.insert(std::pair<std::string, optix::Program>("meshIntersection", createIntersectionTriangle()));
	defaultPrograms.insert(std::pair<std::string, optix::Program>("meshBounds", createBoundingBoxTriangle()));
#endif

	defaultPrograms.insert(std::pair<std::string, optix::Program>("receiverClosestHit", createClosestHitReceiver()));


	defaultPrograms.insert(std::pair<std::string, optix::Program>("receiverIntersection", createIntersectionSphere()));
	defaultPrograms.insert(std::pair<std::string, optix::Program>("receiverBounds", createBoundingBoxSphere()));
	defaultPrograms.insert(std::pair<std::string, optix::Program>("miss", createMissProgram()));
	defaultPrograms.insert(std::pair<std::string, optix::Program>("rayGeneration", createRayGenerationProgram()));



	defaultMeshMaterial = createMeshMaterial(0u,defaultPrograms.at("meshClosestHit")); //For normal rays

#ifdef OPALDEBUG
	outputFile << "defaultPrograms size=" << defaultPrograms.size() << std::endl;
#endif


}

void OpalSceneManager::addMeshToGroup(int id, int meshVertexCount, optix::float3* meshVertices, int meshTriangleCount, int* meshTriangles, MaterialEMProperties emProp) {
	OpalDynamicMeshGroup* dmg;
	try {
		dmg	= dynamicMeshes.at(id);
	}
	catch (std::out_of_range e) {
		std::cout << "addMeshToGroup(): Not found OpalDynamicMeshGroup with id= " << id << std::endl;
		throw  opal::Exception("addMeshToGroup(): Not found OpalDynamicMeshGroup with this id");
		return;
	}
	OpalMesh mesh = createMesh(meshVertexCount, meshVertices, meshTriangleCount, meshTriangles,  defaultPrograms.at("meshIntersection"), defaultPrograms.at("meshBounds"), defaultMeshMaterial);
	setMeshEMProperties(mesh.geom_instance, emProp);

	dmg->geom_group->addChild(mesh.geom_instance);
	dmg->geom_group->getAcceleration()->markDirty();
	if (sceneGraphCreated) {
		rootGroup->getAcceleration()->markDirty();
	}
}
void  OpalSceneManager::updateTransformInGroup(int id, optix::Matrix4x4 transformationMatrix) {
	OpalDynamicMeshGroup* dmg;
	try {
		dmg = dynamicMeshes.at(id);
	}
	catch (std::out_of_range e) {
		std::cout << "updateTransformInGroup(): Not found OpalDynamicMeshGroup with id= " << id << std::endl;
		throw  opal::Exception("updateTransformInGroup(): Not found OpalDynamicMeshGroup with this id");
		return;
	}
	dmg->transform->setMatrix(0, transformationMatrix.getData(), nullptr);
	if (sceneGraphCreated) {
		rootGroup->getAcceleration()->markDirty();
	}
}

void opal::OpalSceneManager::finishDynamicMeshGroup(int id)
{
	OpalDynamicMeshGroup* dmg;
	try {
		dmg = dynamicMeshes.at(id);
	}
	catch (std::out_of_range e) {
		std::cout << "finishDynamicMeshGroup(): Not found OpalDynamicMeshGroup with id= " << id << std::endl;
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
	prop.dielectricConstant = make_float2(relativePermitivity,-60.0f*defaultChannel.waveLength*conductivity);
	return prop;

}



//Creates and add a static mesh with default programs and material
OpalMesh OpalSceneManager::addStaticMesh(int meshVertexCount, optix::float3* meshVertices, int meshTriangleCount, int* meshTriangles, optix::Matrix4x4 transformationMatrix, MaterialEMProperties emProp) {

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
	OpalMesh mesh=createMesh(meshVertexCount, transformedVertices.data(), meshTriangleCount, meshTriangles,  defaultPrograms.at("triangleAttributes"), defaultPrograms.at("triangleAttributes"), defaultMeshMaterial);
#else
	OpalMesh mesh=createMesh(meshVertexCount, transformedVertices.data(), meshTriangleCount, meshTriangles, defaultPrograms.at("meshIntersection"), defaultPrograms.at("meshBounds"), defaultMeshMaterial);

#endif
	setMeshEMProperties(mesh.geom_instance, emProp);
	addStaticMesh(mesh);
	return mesh;
}
void OpalSceneManager::addStaticMesh(OpalMesh mesh) {
	mesh.geom_instance["meshId"]->setUint(static_cast<uint>(staticMeshes.size())); //Used by the closest hit program only for DEBUG. Could be eliminated
#ifdef OPALDEBUG
	outputFile << "adding mesh =" << staticMeshes.size() << std::endl;
#endif
	staticMeshes.push_back(mesh);
	if (sceneGraphCreated) {
		staticMeshesGroup->addChild(mesh.geom_instance);
		staticMeshesGroup->getAcceleration()->markDirty();
		rootGroup->getAcceleration()->markDirty();
	}

}


void OpalSceneManager::extractFaces(optix::float3* meshVertices, std::vector<std::pair<optix::int3, unsigned int>> &triangleIndexBuffer) {
	FaceMap normals;

	for (size_t i = 0; i < triangleIndexBuffer.size(); i++)
	{
		//Get the normal of the triangle plane
		const float3 p0 = meshVertices[triangleIndexBuffer[i].first.x];
		const float3 p1 = meshVertices[triangleIndexBuffer[i].first.y];
		const float3 p2 = meshVertices[triangleIndexBuffer[i].first.z];
		const float3 e0 = p1 - p0;
		const float3 e1 = p0 - p2;

		optix::float3 normal = cross(e1, e0);
		//std::cout <<"i="<<i<<"t="<<triangleIndexBuffer[i].first<< "p0=" << p0 << "p1=" << p1 << "p2=" << p2 << "normal=" << normal << std::endl;




		std::pair<FaceMap::iterator, bool> ret = normals.insert(std::pair<optix::float3, unsigned int>(normal, numberOfFaces));
		if (ret.second==true) {
			//New element
			triangleIndexBuffer[i].second = numberOfFaces;
			//Next faceId
			++numberOfFaces;

		}
		else {
			//Get the faceId
			triangleIndexBuffer[i].second = (*(ret.first)).second;
		}
	}
	std::cout  <<"\t"<< normals.size() <<" faces added=. numberOfFaces="<< numberOfFaces<< std::endl;
	/*	for (auto v: normals)
		{
		std::cout << std::scientific<<"face normal=" << v.first << "id=" << v.second << std::endl;

		}*/







}


OpalDynamicMeshGroup* OpalSceneManager::addDynamicMeshGroup(int id) {
	std::cout<<"Dynamic mesh group called with "<<id<<std::endl;
	OpalDynamicMeshGroup* dmg = new OpalDynamicMeshGroup();
	dmg->geom_group = context->createGeometryGroup();
	dmg->geom_group->setAcceleration(context->createAcceleration("Trbvh"));
	dmg->transform = context->createTransform();
	dmg->transform->setChild(dmg->geom_group);
	std::pair<std::map<int, OpalDynamicMeshGroup*>::iterator,bool> r= dynamicMeshes.insert(std::pair<int, OpalDynamicMeshGroup*>(id, dmg));
	if (r.second == false) {
		std::cout << "A dynamich mesh exists with id= " << id << std::endl;
		throw  opal::Exception("A dynamich mesh with this id already exists");
		return nullptr;
	}
	else {

		return dmg;
	}
}
void OpalSceneManager::removeDynamicMeshGroup(int id) {
	//Remove this mesh from the scene graph
	OpalDynamicMeshGroup* dmg;
	try {
		dmg = dynamicMeshes.at(id);
	}
	catch (std::out_of_range e) {
		std::cout << "removeDynamicMeshGroup(): Not found OpalDynamicMeshGroup with id= " << id << std::endl;
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

		gi->getGeometry()->removeReference();
		gi->removeReference();
	}
	dmg->geom_group->removeReference();
	dmg->transform->removeReference();
	dynamicMeshes.erase(id);
	delete dmg;
}

//Creates a  mesh that may be moved. A rtTransform is assigned when added to a group. Material includes closest hit program. Instance-specific material properties are not set
OpalMesh OpalSceneManager::createMesh(int meshVertexCount, optix::float3* meshVertices, int meshTriangleCount, int* meshTriangles, optix::Program intersectionProgram, optix::Program boundingBoxProgram, optix::Material material) {

	// Create a float3 formatted buffer (each triplet of floats in the array is now a vector3 in the order x,y,z)

	std::cout << "-- Create mesh: with " << meshVertexCount << " vertices and " << meshTriangleCount << " indices" << std::endl;
	//Make int3s
	if (meshTriangleCount % 3 != 0) {
		throw  opal::Exception("Error: Number of triangle indices is not a multiple of 3");

	}
	std::vector<std::pair<optix::int3, unsigned int>> triangleIndexBuffer;


	for (size_t i = 0; i < meshTriangleCount; i += 3)
	{

		triangleIndexBuffer.push_back(std::pair<optix::int3, unsigned int>(make_int3(meshTriangles[i], meshTriangles[i + 1], meshTriangles[i + 2]), 0u));

	}


	extractFaces(meshVertices, triangleIndexBuffer);
	/*for (size_t i = 0; i < triangleIndexBuffer.size(); i++)
	  {
	  std::cout<<triangleIndexBuffer[i].second << std::endl;

	  }*/

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
	std::cout<<"\t Adding triangle attributes"<<std::endl;
	geom_tri->setAttributeProgram(intersectionProgram );
#else

	optix::Geometry geometry = context->createGeometry();
	geometry["vertex_buffer"]->setBuffer(positions);
	geometry["index_buffer"]->setBuffer(tri_indices);
	geometry["faceId_buffer"]->setBuffer(faces);

	geometry->setPrimitiveCount(numTriangles);
	geometry->setBoundingBoxProgram(boundingBoxProgram);
	geometry->setIntersectionProgram(intersectionProgram);
#endif
	mesh.num_triangles = numTriangles;



	std::vector<optix::Material> optix_materials;
	optix_materials.push_back(material);
#ifdef OPAL_USE_TRI 
	mesh.geom_instance = context->createGeometryInstance(geom_tri,material); //If called with iterator does not compile, but it appears in the documentation 
#else

	mesh.geom_instance = context->createGeometryInstance(geometry, optix_materials.begin(), optix_materials.end());
#endif

	return mesh;

}



void OpalSceneManager::setMeshEMProperties(optix::GeometryInstance geom_instance, MaterialEMProperties emProp) {
	//Conductivity is already multiplied by wavelength
#ifdef OPALDEBUG
	outputFile << "mesh EM="<< emProp.dielectricConstant << std::endl;
#endif

	geom_instance["EMProperties"]->setUserData(sizeof(MaterialEMProperties), &emProp);
}

//Default programs

optix::Material OpalSceneManager::createMeshMaterial(unsigned int ray_type_index, optix::Program closestHitProgram) {
	optix::Material mat= context->createMaterial();
	mat->setClosestHitProgram(ray_type_index, closestHitProgram);

	return mat;
}
optix::Program OpalSceneManager::createClosestHitMesh() {
	optix::Program chmesh = context->createProgramFromPTXString(sutil::getPtxString(cudaProgramsDir.c_str(), "triangle.cu"), "closestHitTriangle");
	return chmesh;

}

optix::Program OpalSceneManager::createClosestHitReceiver()
{

	optix::Program chrx;
	chrx = context->createProgramFromPTXString(sutil::getPtxString(cudaProgramsDir.c_str(), "receiver.cu"), "closestHitReceiver");


	//Program variables: common value for all receiver instances, since they all share the program. 
	chrx["k"]->setFloat(defaultChannel.k); //If multichannel is used, this should be set per transmission

	return chrx;
}


optix::Program OpalSceneManager::createBoundingBoxTriangle()
{
	return context->createProgramFromPTXString(sutil::getPtxString(cudaProgramsDir.c_str(), "triangle.cu"), "boundsTriangle");
}

#ifdef OPAL_USE_TRI
optix::Program OpalSceneManager::createTriangleAttributesProgram()
{
	return  context->createProgramFromPTXString(sutil::getPtxString(cudaProgramsDir.c_str(),"optixGeometryTriangles.cu"), "triangle_attributes" ) ;

}
#endif

optix::Program OpalSceneManager::createIntersectionTriangle()
{
	return context->createProgramFromPTXString(sutil::getPtxString(cudaProgramsDir.c_str(), "triangle.cu"), "intersectTriangle");

}

optix::Program OpalSceneManager::createBoundingBoxSphere()
{


	return context->createProgramFromPTXString(sutil::getPtxString(cudaProgramsDir.c_str(), "sphere.cu"), "boundsSphere");

}


optix::Program OpalSceneManager::createIntersectionSphere()
{


	return context->createProgramFromPTXString(sutil::getPtxString(cudaProgramsDir.c_str(), "sphere.cu"), "robust_intersectSphere");

}

optix::Program OpalSceneManager::createMissProgram() 
{
	return context->createProgramFromPTXString(sutil::getPtxString(cudaProgramsDir.c_str(), "receiver.cu"), "miss");

}



optix::Program  OpalSceneManager::createRayGenerationProgram()
{


	return context->createProgramFromPTXString(sutil::getPtxString(cudaProgramsDir.c_str(), "generation.cu"), "genRayAndReflectionsFromSphereIndex");

}

//Create and arbitrary 2D ray sphere, with ray directions provided by the user
void OpalSceneManager::createRaySphere2D(int elevationSteps, int azimuthSteps, optix::float3*  rayDirections)
{




	raySphere.rayCount = elevationSteps*azimuthSteps;
	raySphere.elevationSteps = static_cast<optix::uint>(elevationSteps);
	raySphere.azimuthSteps = static_cast<optix::uint>(azimuthSteps);

	optix::Buffer rays = context->createBuffer(RT_BUFFER_INPUT, RT_FORMAT_FLOAT3, elevationSteps, azimuthSteps);
	optix::float3* rays_host = reinterpret_cast<optix::float3*>  (rays->map());
	for (size_t i = 0; i <  raySphere.rayCount; i++)
	{
		rays_host[i].x = rayDirections[i].x;
		rays_host[i].y = rayDirections[i].y;
		rays_host[i].z = rayDirections[i].z;


	}
	context["raySphere2D"]->set(rays);
	context["raySphereSize"]->setUint(make_uint2(elevationSteps, azimuthSteps));
	float as;
	if (elevationSteps<azimuthSteps) {
		as=360.0f/azimuthSteps;
	} else {
		as=180.0f/elevationSteps;
	}
	context["asRadiusConstant"]->setFloat(as*deg2rad);
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
	optix::uint elevationSteps = (180u / elevationDelta);

	optix::uint azimuthSteps = (360u / azimuthDelta);
	//std::cout << "createRaySphere2D: elevationSteps=" << elevationSteps << "azimuthSteps=" << azimuthSteps << std::endl;

	++elevationSteps; //elevation includes both 0 and 180. elevation in [0,180], but 0 and 180 maps to the same ray, so we remove them from the multiplication and add later


	raySphere.rayCount = elevationSteps*azimuthSteps; //elevation includes [0,180] whereas azimuth includes [0,360[
	raySphere.elevationSteps = elevationSteps;
	raySphere.azimuthSteps = azimuthSteps;

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
	context["raySphereSize"]->setUint(make_uint2(raySphere.elevationSteps, raySphere.azimuthSteps));
	float asRadiusConstant;
	if (elevationDelta<azimuthDelta) {
		asRadiusConstant=elevationDelta*deg2rad;
	} else {
		asRadiusConstant=azimuthDelta*deg2rad;
	}
	context["asRadiusConstant"]->setFloat(asRadiusConstant);
	std::cout <<"-- Create RaySphere 2D: asRadiusConstant="<<asRadiusConstant<<". raySphere.azimuthSteps=" << raySphere.azimuthSteps << ". as=" << as << ". raySphere.elevationSteps =" << raySphere.elevationSteps << ". es=" << es << "raySphere.rayCount=" << raySphere.rayCount << ". rc=" << rc << std::endl;
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
	std::cout << "-- Create Ray Sphere 2D with decimal dgree steps: rays=" << raySphere.rayCount << ". elevationSteps=" << raySphere.elevationSteps << ". azimtuhSteps=" << raySphere.azimuthSteps << "rc=" << rc << std::endl;






	context["raySphere2D"]->set(rays);
	context["raySphereSize"]->setUint(make_uint2(elevationSteps, azimuthSteps));

	float asRadiusConstant;
	if (elevationDelta<azimuthDelta) {
		asRadiusConstant=elevationDelta*deg2rad*0.1f;
	} else {
		std::cout <<"\t Substep Sphere: azimuthDelta="<<azimuthDelta<<"deg2rad="<<deg2rad<<std::endl; 	
		asRadiusConstant=azimuthDelta*deg2rad*0.1f;
		std::cout <<"\t Substep Sphere: asRadiusConstant="<<asRadiusConstant<<std::endl;
	}
	context["asRadiusConstant"]->setFloat(asRadiusConstant);
	//std::cout <<"Substep Sphere: asRadiusConstant="<<asRadiusConstant<<". raySphere.azimuthSteps=" << raySphere.azimuthSteps << ". raySphere.elevationSteps =" << raySphere.elevationSteps << "raySphere.rayCount=" << raySphere.rayCount <<  std::endl;

	rays->unmap();
	raySphere.raySphereBuffer = rays;
#ifdef OPALDEBUG
	outputFile << "RaySphere2D rays=" << raySphere.rayCount << ". elevationSteps=" << raySphere.elevationSteps << ". azimtuhSteps=" << raySphere.azimuthSteps << std::endl;
#endif
}





void OpalSceneManager::addReceiver(int id, float3  position, float3 polarization, float radius, std::function<void(float, int)>  callback)
{
	std::cout << "--Add receiver: receiver " << id << " with radius " <<radius<<" at "<<position<< std::endl;
	optix::Geometry geometry = context->createGeometry();
	float4 sphere = make_float4(position.x, position.y, position.z, radius);

	geometry->setPrimitiveCount(1);
	geometry->setBoundingBoxProgram(defaultPrograms.at("receiverBounds"));
	geometry->setIntersectionProgram(defaultPrograms.at("receiverIntersection"));
	std::vector<optix::Material> optix_materials;
	optix::Program chrx = defaultPrograms.at("receiverClosestHit");



	optix::Material mat = context->createMaterial();
	mat->setClosestHitProgram(0u, chrx);
	optix_materials.push_back(mat);


	optix::GeometryInstance geom_instance = context->createGeometryInstance(geometry, optix_materials.begin(), optix_materials.end());

	//We use a common program for all the receivers: individual variables must be set per geometry instance

	//Add id and position to instace
	geom_instance["sphere"]->setFloat(sphere);

	//The buffer id is consecutive, independent of the external Id

	uint nextId = static_cast<uint>(receivers.size());
	geom_instance["receiverBufferIndex"]->setUint(nextId);
	geom_instance["externalId"]->setInt(id);

	if (useDepolarization) {
		geom_instance["receiverPolarization"]->setFloat(polarization);
		std::cout<<"\t Using depolarization. Receiver polarization is "<<polarization << std::endl;

	}
	std::cout<<"\t Receiver buffer index for "<<id << " is "<<nextId<< " and external Id is "<<id<<std::endl;
	SphereReceiver* rx = new SphereReceiver();
	rx->geomInstance = geom_instance;
	rx->position = position;
	rx->polarization=polarization;
	rx->radius = radius;
	rx->callback = callback;
	rx->closestHitProgram = chrx;
	rx->externalId = id;
	rx->dirty=false;
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

	//myfile.close();




	//Do not really need transform since the geometry (sphere) has no structure apart from position and radius...?
	//optix::Transform transform = sceneManager->sceneContext->createTransform();

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
	receiversGroup->getAcceleration()->markDirty();
	rootGroup->getAcceleration()->markDirty();
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
	r->geomInstance["sphere"]->setFloat(make_float4(position.x, position.y, position.z, radius));
	receiversGroup->getAcceleration()->markDirty();
	rootGroup->getAcceleration()->markDirty();
}


void OpalSceneManager::transmit(int txId, float txPower,  float3 origin, float3 polarization) {

	uint numReceivers = static_cast<uint>(receivers.size());
	if (numReceivers == 0) {
		//For debug or testing reflections or any other thing, at least one receiver should be added
		return;

	}

	if (numReceivers == 1) {
		if (receivers[0]->externalId == txId) {
			//No power will be received, since we are not assuming   dual transceiver
			return;
		}
	}

	checkInternalBuffers();	

	//Change to buffer, according to https://devtalk.nvidia.com/default/topic/1048952/optix/recompile-question/ it is better to use a small buffer rather than setting variables
	Transmitter* host_tx = reinterpret_cast<Transmitter*>  (txOriginBuffer->map());
	host_tx->origin = origin;
	host_tx->polarization = polarization;
	host_tx->externalId = txId;
	txOriginBuffer->unmap();

	//Adaptive radio just to avoid a transmitter being inside a receiver sphere
	bool applyDirty=false;
	for (int i=0; i<numReceivers; i++) {
		if (receivers[i]->externalId!=txId) {
			float distance=length(receivers[i]->position-origin);
			if (distance<=receivers[i]->radius+0.001f) {
				receivers[i]->geomInstance["sphere"]->setFloat(make_float4(receivers[i]->position.x, receivers[i]->position.y, receivers[i]->position.z,distance*radioReductionFraction ));
				receivers[i]->dirty=true;
				applyDirty=true;
			} else if (receivers[i]->dirty) {
				receivers[i]->geomInstance["sphere"]->setFloat(make_float4(receivers[i]->position.x, receivers[i]->position.y, receivers[i]->position.z,receivers[i]->radius ));
				receivers[i]->dirty=false;
				applyDirty=true;
			}
		}
	}
	if (applyDirty)  {
		receiversGroup->getAcceleration()->markDirty();
		rootGroup->getAcceleration()->markDirty();
	}
	if (useMultiGPU) {
		executeTransmitLaunchMultiGPU(txId,txPower,origin);
	} else {
		executeTransmitLaunch(txId,txPower,origin);
	}

}

void OpalSceneManager::executeTransmitLaunch(int txId, float txPower,  float3 origin) {
	//Initialize index for global buffer	
	uint* aib=reinterpret_cast<uint*>(atomicIndexBuffer->map());
	(*aib)=0u;
	atomicIndexBuffer->unmap();


	//Transmission launch
	//std::cout<<"Transmitting["<<txId<<"]["<<txPower<<"]"<<origin<<std::endl;	
	Timer timer;
	timer.start();
	context->launch(0, raySphere.elevationSteps, raySphere.azimuthSteps,1u); //Launch 3D (elevation, azimuth, transmitters);
	timer.stop();
	const double launchTime=timer.getTime();
	transmissionLaunches++;

	//Get total number of hits (last global buffer index used)

	aib=reinterpret_cast<uint*>(atomicIndexBuffer->map());
	uint lastHitIndex= (*aib);
	atomicIndexBuffer->unmap();


	//std::cout<<"lastHitIndex="<<lastHitIndex<<std::endl;

	timer.restart();
	//Filter with thrust multiple hits coming from the same face
	uint hits=opalthrustutils::filterHitsWithCopyResize(globalHitInfoBuffer, resultHitInfoBuffer, lastHitIndex);
	
	//Performance test
	timer.stop();
	const double filterTime=timer.getTime();
	timer.restart();

	//Transfer the filtered hits to the host
	HitInfo* host_hits=reinterpret_cast<HitInfo*>  (resultHitInfoBuffer->map());
	
	//Just for logging performance
	timer.stop();
	const double transferTime=timer.getTime();
	float2 E=make_float2(0.0f,0.0f);
	RTsize gsize;
	globalHitInfoBuffer->getSize(gsize);
	//Log times for performance tests
	uint numReceivers = static_cast<uint>(receivers.size());
	std::cout<<"#"<<numReceivers<<"\t"<<gsize<<"\t"<<lastHitIndex<<"\t"<<hits<<"\t"<<launchTime<<"\t"<<filterTime<<"\t"<<transferTime<<std::endl;

	//Compute received power by adding EM waves of all hits. Global computation. Not done with thrust because thrust::reduce does not seem to allow a different type as output of the sum

	uint index=0u;
	uint raysHit=0u;


	for (uint i=0; i<hits; i++) {
		if (i==0) {
			//Get first receiver
			index=host_hits->thrd.z;

		} else {
			if (host_hits->thrd.z!=index) {
				if (raysHit!=0u) {
					//At least one hit, callback
					computeReceivedPower(E,index,txId,txPower,origin);
				}
				//New receiver, start new accumulation
				index=host_hits->thrd.z;
				E=make_float2(0.0f,0.0f);
				raysHit=0u;
			}
		}

		++raysHit;
		E += host_hits->E;

	// Log hits received
	//	std::cout<<"E["<<i<<"]="<<(host_hits)->E<<std::endl;
	//	std::cout<<"\t rxBufferIndex="<<(host_hits)->thrd.z<<std::endl;
	//	std::cout<<"\t written="<<(host_hits)->thrd.x<<std::endl;
	//	std::cout<<"\t refhash="<<(host_hits)->thrd.y<<std::endl;
	//	std::cout<<"\t dist="<<(host_hits)->thrd.w<<std::endl;


		++host_hits;

	}
	//Last one
	if (raysHit!=0u) {
		computeReceivedPower(E,index,txId,txPower,origin);
	}	
	//timer.stop();
	resultHitInfoBuffer->unmap();
}
void OpalSceneManager::executeTransmitLaunchMultiGPU(int txId, float txPower,  float3 origin) {

	//Transmission launch
	//std::cout<<"Transmitting["<<txId<<"]["<<txPower<<"]"<<origin<<std::endl;	
	Timer timer;
	timer.start();
	context->launch(0, raySphere.elevationSteps, raySphere.azimuthSteps,1u); //Launch 3D (elevation, azimuth, transmitters);
	timer.stop();
	const double launchTime=timer.getTime();
	transmissionLaunches++;


	timer.restart();
	//Filter with thrust multiple hits coming from the same face. Directly returns the filtered vector
	thrust::host_vector<HitInfo> host_hits=opalthrustutils::filterHitsMultiGPU(globalHitInfoBuffer,  atomicIndexBuffer, enabledDevices );
	
	//Log times for performance tests
	timer.stop();
	const double filterTime=timer.getTime();
	uint numReceivers = static_cast<uint>(receivers.size());
	std::cout<<"#"<<numReceivers<<"\t"<<host_hits.size()<<"\t"<<launchTime<<"\t"<<filterTime<<std::endl;

	//Compute received power by adding EM waves of all hits. Global computation. Not done with thrust because thrust::reduce does not seem to allow a different type as output of the sum

	float2 E=make_float2(0.0f,0.0f);
	uint index=0u;
	uint raysHit=0u;


	for (uint i=0; i<host_hits.size(); i++) {

		if (i==0) {
			//Get first receiver
			index=host_hits[i].thrd.z;

		} else {
			if (host_hits[i].thrd.z!=index) {
				if (raysHit!=0u) {
					//At least one hit, callback
					computeReceivedPower(E,index,txId,txPower,origin);
				}
				//New receiver, start new accumulation
				index=host_hits[i].thrd.z;
				E=make_float2(0.0f,0.0f);
				raysHit=0u;
			}
		}

		++raysHit;
		E += host_hits[i].E;

	//	std::cout<<"E["<<i<<"]="<<host_hits[i].E<<std::endl;
	//	std::cout<<"\t rxBufferIndex="<<host_hits[i].thrd.z<<std::endl;
	//	std::cout<<"\t written="<<host_hits[i].thrd.x<<std::endl;
	//	std::cout<<"\t refhash="<<host_hits[i].thrd.y<<std::endl;
	//	std::cout<<"\t dist="<<host_hits[i].thrd.w<<std::endl;



	}
	//Last one
	if (raysHit!=0u) {
		computeReceivedPower(E,index,txId,txPower,origin);
	}	


}

void OpalSceneManager::computeReceivedPower(optix::float2 E, unsigned int index, int txId, float txPower, optix::float3 origin) {
	float power = defaultChannel.eA*((E.x*E.x) + (E.y*E.y))*txPower;
	std::cout << "rx["<<receivers[index]->externalId<<"]=" << receivers[index]->position << ".r=" << receivers[index]->radius << "(sphere="<<receivers[index]->geomInstance["sphere"]->getFloat4()<<"); tx["<<txId<<"]=" << origin << " eA=" << defaultChannel.eA << " txPower=" << txPower << " E=(" << E.x << "," << E.y << ")" << " p=" << power <<  " d=" << length(origin - receivers[index]->position) << std::endl;
	//std::cout<<"PR\t"<<power<<std::endl;
	receivers[index]->callback(power, txId);

}

void OpalSceneManager::clearInternalBuffers() {
	//Should we do this? It is not clear in the Optix documentation whether a resizing a buffer is preferrable to recreating buffers, or whether resizing deletes the previously allocated one
	//std::cout << "Clear internal buffers: " << std::endl;




	if (globalHitInfoBuffer) {
		globalHitInfoBuffer->destroy();
	}
	if (resultHitInfoBuffer) {
		resultHitInfoBuffer->destroy();
	}


	currentInternalBuffersState.tx=0u;
	currentInternalBuffersState.rx=0u;
	currentInternalBuffersState.reflections=0u;


}
void OpalSceneManager::setInternalBuffers() {
	optix::uint rxSize=static_cast<uint>(receivers.size());
	optix::uint reflectionsSize=maxReflections;
	optix::uint elevationSize=raySphere.elevationSteps;
	optix::uint azimuthSize=raySphere.azimuthSteps;

	//Store current state
	currentInternalBuffersState.tx=1u;
	currentInternalBuffersState.rx=rxSize;
	currentInternalBuffersState.reflections=reflectionsSize;
	currentInternalBuffersState.elevation=elevationSize;
	currentInternalBuffersState.azimuth=azimuthSize;


	if (elevationSize==0 || azimuthSize==0) {
		throw opal::Exception("setInternalBuffers(): Cannot set internal buffers with zero rays in raySphere");
		return;	
	}

	globalHitInfoBuffer = setGlobalHitInfoBuffer(elevationSize, azimuthSize, rxSize, reflectionsSize);
	if (!useMultiGPU) {
		resultHitInfoBuffer = setResultHitInfoBuffer(rxSize, reflectionsSize);
	} else {
		resultHitInfoBuffer=nullptr;
	}
	atomicIndexBuffer=setAtomicIndexBuffer();
	txOriginBuffer=setTransmitterBuffer(1u);	
}


void OpalSceneManager::checkInternalBuffers() {
	//Check for changes in internal buffers size
	optix::uint rxSize=static_cast<uint>(receivers.size());
	optix::uint reflectionsSize=maxReflections;
	optix::uint elevationSize=raySphere.elevationSteps;
	optix::uint azimuthSize=raySphere.azimuthSteps;
	if (	currentInternalBuffersState.rx==rxSize && currentInternalBuffersState.reflections==reflectionsSize &&  currentInternalBuffersState.elevation==elevationSize &&	currentInternalBuffersState.azimuth==azimuthSize)
	{
		return;
	} else {
		//Re create internal buffers
		//std::cout<<"Reconfiguring internal buffers: "<<std::endl;
		resizeGlobalHitInfoBuffer(elevationSize, azimuthSize, rxSize, reflectionsSize);
	}
	//Store current state
	currentInternalBuffersState.rx=rxSize;
	currentInternalBuffersState.reflections=reflectionsSize;
	currentInternalBuffersState.elevation=elevationSize;
	currentInternalBuffersState.azimuth=azimuthSize;
	//std::cout<<printInternalBuffersState()<<std::endl;
}



void OpalSceneManager::resizeGlobalHitInfoBuffer(optix::uint ele, optix::uint azi, optix::uint rx, optix::uint reflections) {
	RTsize bytes=0;
	for (size_t i = 0; i < enabledDevices.size(); ++i) 
	{
		bytes+=context->getAvailableDeviceMemory(enabledDevices[i]);
	}
	uint rec=1u;
	if (rx>0) {
		rec=rx;
	}
	uint bsize=ele*azi*rec*reflections;
	if ((bsize*sizeof(HitInfo))>(0.7f*bytes)) {
		bsize=floor(0.7*bytes/sizeof(HitInfo));
		std::cout<<"WARNING: globalHitInfoBuffer size exceeds 70% of available memory. Available device memory: "<<bytes<<"; buffer size set to "<<bsize<<std::endl;
	}
	//std::cout<<"Available device memory: "<<bytes<<". globalHitBuffer  size="<<bsize<<std::endl;
	//optix::Buffer b = context->createBuffer(RT_BUFFER_INPUT_OUTPUT | RT_BUFFER_GPU_LOCAL, RT_FORMAT_USER, bsize );

	globalHitInfoBuffer->setSize(bsize);	
	context["global_info_buffer_maxsize"]->setUint(bsize);
}
//TODO: change constants to parameters
optix::Buffer OpalSceneManager::setGlobalHitInfoBuffer(optix::uint ele, optix::uint azi, optix::uint rx, optix::uint reflections) {

	RTsize bytes=0;
	for (size_t i = 0; i < enabledDevices.size(); ++i) 
	{
		bytes+=context->getAvailableDeviceMemory(enabledDevices[i]);
	}
	uint rec=1u;
	if (rx>0) {
		rec=rx;
	}
	uint bsize=ele*azi*rec*reflections;
	if ((bsize*sizeof(HitInfo))>(0.7f*bytes)) {
		bsize=floor(0.7*bytes/sizeof(HitInfo));
		std::cout<<"WARNING: globalHitInfoBuffer size exceeds 70% of available memory. Available device memory: "<<bytes<<"; buffer size set to "<<bsize<<std::endl;
	}
	std::cout<<"-- Creating global buffer: available "<<enabledDevices.size()<<" devices with total device memory   "<<(bytes/(1024*1024))<<" MiB. globalHitBuffer  size (MiB)="<<(bsize/(1024*1024))<<std::endl;
	optix::Buffer b;
	if (useMultiGPU) {
		b = context->createBuffer(RT_BUFFER_INPUT_OUTPUT | RT_BUFFER_GPU_LOCAL, RT_FORMAT_USER, bsize );
	} else {
		b = context->createBuffer(RT_BUFFER_OUTPUT, RT_FORMAT_USER, bsize );
	}
	b->setElementSize(sizeof(HitInfo));
	context["globalHitInfoBuffer"]->set(b);
	context["global_info_buffer_maxsize"]->setUint(bsize);
	return b;
}
optix::Buffer OpalSceneManager::setResultHitInfoBuffer(optix::uint rx, optix::uint reflections) {
	uint rec=1u;
	if (rx>0) {
		rec=rx;
	}
	optix::Buffer b = context->createBuffer(RT_BUFFER_OUTPUT, RT_FORMAT_USER, 10u*rec*(reflections+1u) );
	b->setElementSize(sizeof(HitInfo));
	context["resultHitInfoBuffer"]->set(b);
	return b;
}
optix::Buffer OpalSceneManager::setAtomicIndexBuffer() {
	optix::Buffer b;
	if (useMultiGPU) {
		b = context->createBuffer(RT_BUFFER_INPUT_OUTPUT | RT_BUFFER_GPU_LOCAL, RT_FORMAT_UNSIGNED_INT, 1u);
	} else {
		b = context->createBuffer(RT_BUFFER_INPUT_OUTPUT, RT_FORMAT_UNSIGNED_INT, 1u);

	}
	context["atomicIndex"]->set(b);
	return b;
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
std::string opal::OpalSceneManager::printContextInformation()
{
	std::ostringstream stream;
	stream<<"Context: Num CPU Threads: "<<context->getCPUNumThreads()<<std::endl;
	int RTX;
	rtGlobalGetAttribute(RT_GLOBAL_ATTRIBUTE_ENABLE_RTX,sizeof(RTX),&RTX);
	stream<<"Context: RTX mode enabled: "<<RTX<<std::endl;

	return stream.str();
}

std::string opal::OpalSceneManager::printInternalBuffersState()
{

	std::ostringstream stream;
	unsigned long long totalBytes = 0;
	unsigned long long sb;
	stream << "--Internal buffers--" << std::endl;
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
		stream << "\t globalHitInfoBuffers=(" << w <<"). size=" << (sb / 1024.f) << " KiB" << std::endl;
	}
	if (resultHitInfoBuffer) {
		resultHitInfoBuffer->getSize(w);
		sb = sizeof(HitInfo)*w;
		totalBytes += sb;
		stream << "\t resulHitInfoBuffers=(" << w <<"). size=" << (sb / 1024.f) << " KiB" << std::endl;
	}
	if (txOriginBuffer) {
		txOriginBuffer->getSize(w);
		sb = sizeof(Transmitter)*w;
		totalBytes += sb;
		stream << "\t txOriginBuffer=(" << w <<"). size=" << (sb / 1024.f) << " KiB" << std::endl;
	}
	//Check memory usage
	stream << "Total memory in internal buffers:  " << (totalBytes / (1024.f*1024.f)) << " MiB" << std::endl;
	return stream.str();

}


void OpalSceneManager::finishSceneContext() {

	if (raySphere.rayCount <= 0) {
		throw  opal::Exception("Scene not created. Ray count=0. Create Ray sphere before finishing scene context");
	}

	//Build scene graph
	buildSceneGraph();

	//Create internal buffers
	setInternalBuffers();

	//Set some minimum extent for the rays. Otherwise we are going to run into numerical accuracy problems with the reflections, almost surely
	context["min_t_epsilon"]->setFloat(minEpsilon);
	context["max_interactions"]->setUint(maxReflections);
	if (usePenetration) {
		context["usePenetration"]->setUint(1u);
		//Show warning about assumptions
		configInfo <<"\t - You are using penetration: check that the  receiver sphere(s) does not overlap any wall, otherwise you are getting wrong results almost surely" <<std::endl;
	} else {
		context["usePenetration"]->setUint(0u);
	}
	context["attenuationLimit"]->setFloat(attenuationLimit);

	context->setRayGenerationProgram(0, defaultPrograms.at("rayGeneration")); //Generation
	context->setMissProgram(0, defaultPrograms.at("miss"));
	context->validate();
	//context->setExceptionEnabled(RT_EXCEPTION_ALL, true);
	sceneFinished = true;
	std::cout<<"--- Check your configuration and assumptions --"<<std::endl;
	configInfo<<printSceneReport();
	std::cout<<configInfo.str()<<std::endl;
}


std::string opal::OpalSceneManager::printSceneReport()  {
	std::ostringstream stream;
	stream << "--- Scene Summary ---" << std::endl;
	stream << "\t numberOfFaces=" << numberOfFaces << ". minEpsilon= " << minEpsilon << ". maxReflections=" << maxReflections << ". useExactSpeedOfLight="<<this->useExactSpeedOfLight<<std::endl;
	stream << "\t Receivers=" << receivers.size() <<  std::endl;
	stream << "\t RayCount=" << raySphere.rayCount << ". azimuthSteps=" << raySphere.azimuthSteps << ". elevationSteps=" << raySphere.elevationSteps << std::endl;
	if (usePenetration) {
		stream << "\t **Feature: Penetration enabled. attenuationLimit (dB)=" << attenuationLimit<< std::endl;
	}
	if (useDepolarization) {
		stream << "\t **Feature: Depolarization enabled."<< std::endl;
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

		stream << "\t StaticMesh[" << i << "].EMProperties=" << em.dielectricConstant << std::endl;
	}
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
	stream<<printContextInformation()<<std::endl;
	stream << "-----" << std::endl;
	return stream.str();

}
void opal::OpalSceneManager::setPrintEnabled(int bufferSize)
{

	context->setPrintEnabled(true);
	context->setPrintBufferSize(bufferSize);
	std::cout << "printEnabled" << std::endl;
	//Set a particular print index. Comment out if necessary
	context->setPrintLaunchIndex(3,0,0);
	std::cout<<"Showing launch index [3,0,0]"<<std::endl;

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


	//Create Geometry group for receivers
	receiversGroup = context->createGeometryGroup();
	receiversGroup->setChildCount(static_cast<uint>(receivers.size()));
	unsigned int cindex = 0;
	for (const auto val : receivers)
	{
		receiversGroup->setChild(cindex, val->geomInstance);
#ifdef OPALDEBUG
		outputFile << "Receiver[ " << cindex << "].position=" << val->position <<". radius="<<val->radius <<std::endl;
#endif // OPALDEBUG
		++cindex;

	}
	receiversGroup->setAcceleration(context->createAcceleration("Trbvh"));
	context["receivers"]->set(receiversGroup);





	//Create root
	rootGroup = context->createGroup();
	//rootGroup->setChildCount(2+ statc);
	rootGroup->addChild(staticMeshesGroup);
	rootGroup->addChild(receiversGroup);
	//Add dynamic meshes
	for (auto val : dynamicMeshes)
	{
		val.second->childIndex = rootGroup->addChild(val.second->transform);
		//val.second->transform->setChild(val.second->geom_group);
	}

	rootGroup->setAcceleration(context->createAcceleration("Trbvh")); //All Groups and GeometryGroups MUST have an acceleration
	context["root"]->set(rootGroup);

	sceneGraphCreated = true;
#ifdef OPALDEBUG
	outputFile << "Building scene graph. Static  meshes: " << staticMeshes.size() << ". Receivers: " << receivers.size() << ". Dynamic meshes:"<< dynamicMeshes.size() <<std::endl;
#endif // OPALDEBUG


}
void OpalSceneManager::callbackUsageReport(int level, const char* tag, const char* msg, void* cbdata)
{
	std::cout << "[" << level << "][" << std::left << std::setw(12) << tag << "] " << msg;
}

void OpalSceneManager::enableMultiGPU() {
	if (context) {
		//It is actually perfectly safe to enable GPU devices after creating context, but for now we consider it an error
		throw  opal::Exception("Do not enable multi GPU support after creating the context");
		return;

	}
	useMultiGPU=true;
}
void OpalSceneManager::disableMultiGPU() {
	if (context) {
		//It is actually perfectly safe to enable GPU devices after creating context, but for now we consider it an error
		throw  opal::Exception("Do not disable multi GPU support after creating the context");
		return;

	}
	useMultiGPU=false;
}
void OpalSceneManager::enablePenetration() {
	usePenetration=true;	
}
void OpalSceneManager::disablePenetration() {
	usePenetration=false;	
}
void OpalSceneManager::enableDepolarization() {
	useDepolarization=true;	
}
void OpalSceneManager::disableDepolarization() {
	useDepolarization=false;	
}
void OpalSceneManager::setAttenuationLimit(float a) {
	attenuationLimit=a;
}
const std::map<int, OpalDynamicMeshGroup*> &  OpalSceneManager::getDynamicMeshes()  const {
	return dynamicMeshes;
}

ChannelParameters OpalSceneManager::getChannelParameters() const {
	return defaultChannel;
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




