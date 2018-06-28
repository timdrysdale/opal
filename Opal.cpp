/*
 *
 */


#include "Opal.h"
#include "Common.h"
#include <optixu/optixu_math_namespace.h>
#include <optixu/optixu_matrix_namespace.h> 
#include <optixu/optixpp_namespace.h>
#include <optixu\optixu_quaternion_namespace.h>
#include "sutil.h"

#include <algorithm>
#include <functional>
#include <iostream>
#include <fstream>

#include <iomanip>



using namespace opal;
using namespace optix;
  // -------------------------------------------------------------------------------------
 // Init Optix Functions
 // -------------------------------------------------------------------------------------

OpalSceneManager::OpalSceneManager(float f, bool holdReflections)
{
#ifdef OPALDEBUG
	this->holdReflections = holdReflections;
	outputFile.open("opal_log.txt");
#endif // OPALDEBUG
	setFrequency(f);
	createSceneContext();
	setDefaultPrograms();
	this->maxReflections = 10u;
	this->minEpsilon = 1.e-4f;

	this->numberOfFaces = 0u; //Increased every time a new face is added to the scene. TODO: a MAX_FACE_ID should be added to limit the buffers size
	this->sceneGraphCreated = false;
	this->sceneFinished = false;


	this->receptionInfoBuffer = nullptr;
	this->txPowerBuffer = nullptr;
	this->txOriginBuffer = nullptr;

	this->internalRaysBuffer = nullptr;
	this->facesBuffer = nullptr;
	this->facesMinDBuffer = nullptr;;
	this->facesMinEBuffer = nullptr;;


}

opal::OpalSceneManager::~OpalSceneManager()
{
	context->destroy();
#ifdef OPALDEBUG
	outputFile.close();
#endif // OPALDEBUG

}



void OpalSceneManager::setFrequency(float f)
{
	this->defaultChannel.frequency = f;
	//this->defaultChannel.waveLength = 299792458.0f / f; // c/f
	this->defaultChannel.waveLength = 3.0e8f / f;
	this->defaultChannel.k = 2 * 3.14159265358979323846f / this->defaultChannel.waveLength; //wavenumber (2*pi/lambda)
	this->defaultChannel.eA = 1.0f / ((2 * this->defaultChannel.k)*(2 * this->defaultChannel.k)); //Effective Area of the antenna (lambda/4*pi)^2
	

}
void OpalSceneManager::setMaxReflections(unsigned int m)
{
	this->maxReflections = m;
	if (sceneFinished) {
		context["max_interactions"]->setUint(maxReflections);
		std::cout << "maxReflections=" << maxReflections << std::endl;
		updateFacesBuffers();
	}
	
}
void OpalSceneManager::createSceneContext()
{

	// Set up context
	context = optix::Context::create();
	context->setRayTypeCount(1);  //Just one type of ray at the moment
	context->setEntryPointCount(2); //2 programs: initialization and generation programs

}

void opal::OpalSceneManager::setDefaultPrograms()
{

	//Complex functions and arithmetic. CREATE THEM FIRST, since closest hit programs use them
	defaultPrograms.insert(std::pair<std::string, optix::Program>("complex_sqrt", createComplexSqrt()));
	defaultPrograms.insert(std::pair<std::string, optix::Program>("sca_complex_prod", createComplexScaProd()));
	defaultPrograms.insert(std::pair<std::string, optix::Program>("complex_prod", createComplexProd()));
	defaultPrograms.insert(std::pair<std::string, optix::Program>("complex_div", createComplexDiv()));
	defaultPrograms.insert(std::pair<std::string, optix::Program>("complex_exp_only_imaginary", createComplexExpImaginary()));


	defaultPrograms.insert(std::pair<std::string, optix::Program>("meshClosestHit", createClosestHitMesh()));
	defaultPrograms.insert(std::pair<std::string, optix::Program>("meshIntersection", createIntersectionTriangle()));
	defaultPrograms.insert(std::pair<std::string, optix::Program>("meshBounds", createBoundingBoxTriangle()));

#ifdef OPALDEBUG
	if (holdReflections) {
		defaultPrograms.insert(std::pair<std::string, optix::Program>("receiverClosestHit", createClosestHitReceiverHoldReflections()));
	}
	else {

		defaultPrograms.insert(std::pair<std::string, optix::Program>("receiverClosestHit", createClosestHitReceiver()));
	}
#else

	defaultPrograms.insert(std::pair<std::string, optix::Program>("receiverClosestHit", createClosestHitReceiver()));
#endif // OPALDEBUG


	defaultPrograms.insert(std::pair<std::string, optix::Program>("receiverIntersection", createIntersectionSphere()));
	defaultPrograms.insert(std::pair<std::string, optix::Program>("receiverBounds", createBoundingBoxSphere()));
	defaultPrograms.insert(std::pair<std::string, optix::Program>("miss", createMissProgram()));
	defaultPrograms.insert(std::pair<std::string, optix::Program>("rayGeneration", createRayGenerationProgram()));
	defaultPrograms.insert(std::pair<std::string, optix::Program>("initialize", createInitializationProgram()));
	
	

	defaultMeshMaterial = createMeshMaterial(0u,defaultPrograms.at("meshClosestHit"));
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
	updateFacesBuffers();

}

//Creates and add a static mesh with default programs and material
OpalMesh OpalSceneManager::addStaticMesh(int meshVertexCount, optix::float3* meshVertices, int meshTriangleCount, int* meshTriangles, optix::Matrix4x4 transformationMatrix, MaterialEMProperties emProp) {

	OpalMesh mesh=createStaticMesh(meshVertexCount, meshVertices, meshTriangleCount, meshTriangles, transformationMatrix, defaultPrograms.at("meshIntersection"), defaultPrograms.at("meshBounds"), defaultMeshMaterial);
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
		//std::cout << "p0=" << p0 << "p1=" << p1 << "p2=" << p2 << "normal=" << normal << std::endl;
		
		

		
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
	//std::cout << "size=" << normals.size() <<"numberOfFaces="<< numberOfFaces<< std::endl;
	for (auto v: normals)
	{
		std::cout << "face normal=" << v.first << "id=" << v.second << std::endl;

	}

	
	
	

}

//Creates a static mesh. Points are directly translated with the transformation matrix and no rtTransform is assigned. Material includes closest hit program. Instance-specific material properties are not set
OpalMesh OpalSceneManager::createStaticMesh (int meshVertexCount, optix::float3* meshVertices, int meshTriangleCount, int* meshTriangles, optix::Matrix4x4 transformationMatrix, optix::Program intersectionProgram, optix::Program boundingBoxProgram, optix::Material material) {
	
	// Create a float3 formatted buffer (each triplet of floats in the array is now a vector3 in the order x,y,z)


		//Make int3s
		if (meshTriangleCount % 3 != 0) {
			throw  opal::Exception("Error: Number of triangle indices is not a multiple of 3");
			
		}
		std::vector<std::pair<optix::int3,unsigned int>> triangleIndexBuffer;


		for (size_t i = 0; i < meshTriangleCount; i += 3)
		{

			triangleIndexBuffer.push_back(std::pair<optix::int3, unsigned int>(make_int3(meshTriangles[i], meshTriangles[i + 1], meshTriangles[i + 2]),0u));

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
		uint* host_faces= reinterpret_cast<uint*>  (faces->map());

		for (size_t i = 0; i < numTriangles; i++)
		{
			host_tri_indices[i] = triangleIndexBuffer[i].first;
			host_faces[i] = triangleIndexBuffer[i].second;
		}


		mesh.bbox_min = make_float3(1.0e26f, 1.0e26f, 1.0e26f);
		mesh.bbox_max = make_float3(-1.0e26f, -1.0e26f, -1.0e26f);

		//Apply transformation matrix
		for (size_t i = 0; i < meshVertexCount; i++)
		{

			//Should we use Transform?
			//std::cout <<"vertex=("<< meshVertices[i].x <<","<< meshVertices[i].y <<","<< meshVertices[i].z <<")"<< std::endl;
			const float3 v = optix::make_float3(transformationMatrix*optix::make_float4(meshVertices[i], 1.0f));
//#ifdef OPALDEBUG
//			outputFile << "v=(" << v.x << "," << v.y << "," << v.z << ")" << std::endl;
//#endif
			host_positions[i] = v;
			mesh.bbox_min.x = std::min<float>(mesh.bbox_min.x, v.x);
			mesh.bbox_min.y = std::min<float>(mesh.bbox_min.y, v.y);
			mesh.bbox_max.x = std::max<float>(mesh.bbox_max.x, v.x);
			mesh.bbox_max.y = std::max<float>(mesh.bbox_max.y, v.y);
			mesh.bbox_max.z = std::max<float>(mesh.bbox_max.z, v.z);

		}
		//Release buffers
		positions->unmap();
		tri_indices->unmap();
		faces->unmap();
		
		optix::Geometry geometry = context->createGeometry();
		geometry["vertex_buffer"]->setBuffer(positions);
		geometry["index_buffer"]->setBuffer(tri_indices);
		geometry["faceId_buffer"]->setBuffer(faces);

		geometry->setPrimitiveCount(numTriangles);
		geometry->setBoundingBoxProgram(boundingBoxProgram);
		geometry->setIntersectionProgram(intersectionProgram);
		mesh.num_triangles = numTriangles;
		


		std::vector<optix::Material> optix_materials;
		optix_materials.push_back(material);

		mesh.geom_instance = context->createGeometryInstance(geometry, optix_materials.begin(), optix_materials.end());

		
		return mesh;
	
}

OpalDynamicMeshGroup* OpalSceneManager::addDynamicMeshGroup(int id) {
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
	//TODO: the faces buffers are not modified, since we would have to remap all the faces to the internal buffers. Basically rebuild the graph. 
	//It just increases in size as dynamic meshes are added but never decreases
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


	mesh.bbox_min = make_float3(1.0e26f, 1.0e26f, 1.0e26f);
	mesh.bbox_max = make_float3(-1.0e26f, -1.0e26f, -1.0e26f);

	//Do not apply transformation to mesh vertices

	for (size_t i = 0; i < meshVertexCount; i++)
	{

		//Should we use Transform?
		//std::cout <<"vertex=("<< meshVertices[i].x <<","<< meshVertices[i].y <<","<< meshVertices[i].z <<")"<< std::endl;
		const float3 v = meshVertices[i];
		//#ifdef OPALDEBUG
		//			outputFile << "v=(" << v.x << "," << v.y << "," << v.z << ")" << std::endl;
		//#endif
		host_positions[i] = v;
		mesh.bbox_min.x = std::min<float>(mesh.bbox_min.x, v.x);
		mesh.bbox_min.y = std::min<float>(mesh.bbox_min.y, v.y);
		mesh.bbox_max.x = std::max<float>(mesh.bbox_max.x, v.x);
		mesh.bbox_max.y = std::max<float>(mesh.bbox_max.y, v.y);
		mesh.bbox_max.z = std::max<float>(mesh.bbox_max.z, v.z);

	}
	//Release buffers
	positions->unmap();
	tri_indices->unmap();
	faces->unmap();

	optix::Geometry geometry = context->createGeometry();
	geometry["vertex_buffer"]->setBuffer(positions);
	geometry["index_buffer"]->setBuffer(tri_indices);
	geometry["faceId_buffer"]->setBuffer(faces);

	geometry->setPrimitiveCount(numTriangles);
	geometry->setBoundingBoxProgram(boundingBoxProgram);
	geometry->setIntersectionProgram(intersectionProgram);
	mesh.num_triangles = numTriangles;



	std::vector<optix::Material> optix_materials;
	optix_materials.push_back(material);

	mesh.geom_instance = context->createGeometryInstance(geometry, optix_materials.begin(), optix_materials.end());


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
	optix::Program chmesh = context->createProgramFromPTXString(sutil::getPtxString("opal", "triangle.cu"), "closestHitTriangle");
	//Add programs for complex arithmetic
	chmesh["complex_sqrt"]->setProgramId(defaultPrograms.at("complex_sqrt"));
	chmesh["sca_complex_prod"]->setProgramId(defaultPrograms.at("sca_complex_prod"));
	chmesh["complex_prod"]->setProgramId(defaultPrograms.at("complex_prod"));
	chmesh["complex_div"]->setProgramId(defaultPrograms.at("complex_div"));
	return chmesh;

}

optix::Program OpalSceneManager::createClosestHitReceiver()
{

	

	optix::Program chrx = context->createProgramFromPTXString(sutil::getPtxString("opal", "propagationKernels.cu"), "closestHitReceiverFaceMin");

	//Add programs for complex arithmetic
	chrx["complex_exp_only_imaginary"]->setProgramId(defaultPrograms.at("complex_exp_only_imaginary"));
	chrx["sca_complex_prod"]->setProgramId(defaultPrograms.at("sca_complex_prod"));
	chrx["complex_prod"]->setProgramId(defaultPrograms.at("complex_prod"));

	//Program variable: common value for all receiver instances, since they all share the program. If multichannel is used, this should be set per transmission
	chrx["k"]->setFloat(defaultChannel.k);
	return chrx;
}

//Use with hold reflections for debug
optix::Program OpalSceneManager::createClosestHitReceiverHoldReflections()
{

	//Use with hold reflections for debug
	optix::Program chrx= context->createProgramFromPTXString(sutil::getPtxString("opal", "propagationKernels.cu"), "closestHitReceiverFaceMinHoldReflections");


	

	//Add programs for complex arithmetic
	chrx["complex_exp_only_imaginary"]->setProgramId(defaultPrograms.at("complex_exp_only_imaginary"));
	chrx["sca_complex_prod"]->setProgramId(defaultPrograms.at("sca_complex_prod"));
	chrx["complex_prod"]->setProgramId(defaultPrograms.at("complex_prod"));

	//Program variable: common value for all receiver instances, since they all share the program. If multichannel is used, this should be set per transmission
	chrx["k"]->setFloat(defaultChannel.k);
	return chrx;
}

optix::Program OpalSceneManager::createBoundingBoxTriangle()
{
	return context->createProgramFromPTXString(sutil::getPtxString("opal", "triangle.cu"), "boundsTriangle");
}


optix::Program OpalSceneManager::createIntersectionTriangle()
{
	return context->createProgramFromPTXString(sutil::getPtxString("opal", "triangle.cu"), "intersectTriangle");

}

optix::Program OpalSceneManager::createBoundingBoxSphere()
{


	return context->createProgramFromPTXString(sutil::getPtxString("opal", "sphere.cu"), "boundsSphere");

}


optix::Program OpalSceneManager::createIntersectionSphere()
{


	return context->createProgramFromPTXString(sutil::getPtxString("opal", "sphere.cu"), "robust_intersectSphere");

}

optix::Program OpalSceneManager::createMissProgram() 
{
	return context->createProgramFromPTXString(sutil::getPtxString("opal", "propagationKernels.cu"), "miss");

}



optix::Program OpalSceneManager::createComplexProd()
{

	
	return context->createProgramFromPTXString(sutil::getPtxString("opal", "triangle.cu"), "complex_prod");
	

}
optix::Program OpalSceneManager::createComplexDiv()
{

	
		return context->createProgramFromPTXString(sutil::getPtxString("opal", "triangle.cu"), "complex_div");

}
optix::Program OpalSceneManager::createComplexScaProd()
{


	return context->createProgramFromPTXString(sutil::getPtxString("opal", "triangle.cu"), "sca_complex_prod");

}
optix::Program OpalSceneManager::createComplexExpImaginary()
{

	
	return context->createProgramFromPTXString(sutil::getPtxString("opal", "triangle.cu"), "complex_exp_only_imaginary");
	
	
}
optix::Program OpalSceneManager::createComplexSqrt()
{


	return context->createProgramFromPTXString(sutil::getPtxString("opal", "triangle.cu"), "complex_sqrt");

}
optix::Program  OpalSceneManager::createRayGenerationProgram()
{

	
		return context->createProgramFromPTXString(sutil::getPtxString("opal", "propagationKernels.cu"), "genRayAndReflectionsFromSphereIndex");

	
}
optix::Program  OpalSceneManager::createInitializationProgram()
{


	return context->createProgramFromPTXString(sutil::getPtxString("opal", "propagationKernels.cu"), "initializeBuffersFaceBased");


}

 void OpalSceneManager::createRaySphere2D(int elevationSteps, int azimuthSteps, optix::float3*  rayDirections)
{


	 //Check duplicates size. Must be divisible 
	/* if (elevationSteps%duplicates.duplicateBlockSize.x != 0) {
		 throw  opal::Exception("Elevation steps / duplicateBlockSize.x is not an integer");
	 }
	 if ((azimuthSteps/2)%duplicates.duplicateBlockSize.y != 0) {
		 throw  opal::Exception("Azimuth steps / duplicateBlockSize.y is not an integer");
	 }
	 */

	 
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

	rays->unmap();
	raySphere.raySphereBuffer = rays;
#ifdef OPALDEBUG
	outputFile << "RaySphere2D rays=" << raySphere.rayCount <<"elevationSteps="<< raySphere.elevationSteps <<"azimtuhSteps="<< raySphere.azimuthSteps <<std::endl;
#endif

}
 void OpalSceneManager::createRaySphere2D(int elevationDelta, int azimuthDelta) 
 {
	
	// std::cout << "db=" << duplicates.duplicateBlockSize.x << "," << duplicates.duplicateBlockSize.y << std::endl;
	 if (360 % azimuthDelta != 0) {
		throw  opal::Exception("360 must be an integer multiple of azimuthDelta");
	
	 }
	 if (180 % elevationDelta != 0) {
		 throw  opal::Exception("180 must be an integer multiple of elevatioNDelta");
		
	 }
	 optix::uint elevationSteps = (180u / elevationDelta);
	
	 optix::uint azimuthSteps = (360u / azimuthDelta);
	// std::cout << "elevationSteps=" << elevationSteps << "azimuthSteps=" << azimuthSteps << std::endl;

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
	 for (size_t i = 0; i <= 180; i += elevationDelta)
	 {
		 for (size_t j = 0; j < 360; j+= azimuthDelta)
		 {
			
			 //Spherical to cartesian coordinates with r=1, ISO (physics) convention: elevation (inclination) from z-axis, azimuth on the XY plane from x counterclockwise to Y
			 //We use the Unity convention and set UP as the Y axis, being FORWARD the z-axis. That is the Y and Z axis are interchanged with respect to the previous convention
			 //All rays with elevation 0 or 180 and any azimuth  are repeated, because they map to (0,1,0) or (0,-1,0). They will not be traced, but we generate them to keep the symmetry  in the 
			 //2D and 3D buffers
			 float ir = deg2rad*i; //To radians
			 float jr = deg2rad*j;//To radians
			
			 float3 ray = make_float3(sinf(ir)*cosf(jr), cosf(ir), sinf(ir)*sinf(jr) );

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
	 //std::cout << " raySphere.azimuthSteps=" << raySphere.azimuthSteps << "as=" << as << "raySphere.elevationSteps =" << raySphere.elevationSteps << "es=" << es << "raySphere.rayCount=" << raySphere.rayCount << "rc=" << rc << std::endl;
	 rays->unmap();
	 raySphere.raySphereBuffer = rays;
#ifdef OPALDEBUG
	 outputFile << "RaySphere2D rays=" << raySphere.rayCount << ". elevationSteps=" << raySphere.elevationSteps << ". azimtuhSteps=" << raySphere.azimuthSteps << std::endl;
#endif
 }
 void OpalSceneManager::createRaySphere2DSubstep(int elevationDelta, int azimuthDelta)
 {

	 //Create a ray sphere with fractions of degree
	 if (elevationDelta > 10 || azimuthDelta > 10) {
		 throw  opal::Exception("Angle substep is greater than 10");
	 }
	 
	 //optix::uint elevationSteps = (1800u / elevationDelta);
	 
	 //optix::uint azimuthSteps = (3600u / azimuthDelta);
	 

	

	

	

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
	 std::cout << "RaySphere2D rays=" << raySphere.rayCount << ". elevationSteps=" << raySphere.elevationSteps << ". azimtuhSteps=" << raySphere.azimuthSteps  << std::endl;
	 int x = 0;
	 int y = 0;
	
	 int rc = 0;
	 optix::Buffer rays = context->createBuffer(RT_BUFFER_INPUT, RT_FORMAT_FLOAT3, elevationSteps, azimuthSteps);
	 optix::float3* rays_host = reinterpret_cast<optix::float3*>  (rays->map());
	 for (size_t i = 0; i <= 1800u; i+=elevationDelta)
	 {
		 for (size_t j = 0; j < 3600u; j+=azimuthDelta)
		 {

			 //Spherical to cartesian coordinates with r=1, ISO (physics) convention: elevation (inclination) from z-axis, azimuth on the XY plane from x counterclockwise to Y
			 //We use the Unity convention and set UP as the Y axis, being FORWARD the z-axis. That is the Y and Z axis are interchanged with respect to the previous convention
			 //All rays with elevation 0 or 180 and any azimuth  are repeated, because they map to (0,1,0) or (0,-1,0). They will not be traced, but we generate them to keep the symmetry  and use 
			 //2D and 3D buffers
			 float ir = deg2rad*(i/10.0f); //To radians
			 float jr = deg2rad*(j/10.0f);//To radians
			 float3 ray = make_float3(sinf(ir)*cosf(jr), cosf(ir), sinf(ir)*sinf(jr));

			 rays_host[x + elevationSteps*y] = ray;
			
			 y++;
			 // std::cout << "i=" << i << "j=" << j << "(" << ray.x << "," << ray.y << "," << ray.z << ")" << std::endl;
			 rc++;
		 }
		 x++;
		 
		 y = 0;
		

	 }
	 std::cout << "RaySphere2D rays=" << raySphere.rayCount << ". elevationSteps=" << raySphere.elevationSteps << ". azimtuhSteps=" << raySphere.azimuthSteps << "rc=" << rc << std::endl;






	 context["raySphere2D"]->set(rays);
	 context["raySphereSize"]->setUint(make_uint2(elevationSteps, azimuthSteps));

	
	 rays->unmap();
	 raySphere.raySphereBuffer = rays;
#ifdef OPALDEBUG
	 outputFile << "RaySphere2D rays=" << raySphere.rayCount << ". elevationSteps=" << raySphere.elevationSteps << ". azimtuhSteps=" << raySphere.azimuthSteps << std::endl;
#endif
 }

 /*void opal::OpalSceneManager::setDuplicateBlockSize(unsigned int elevationBlockSize ,unsigned int azimuthBlockSize)
 {
	 duplicates.duplicateBlockSize = make_uint2(elevationBlockSize, azimuthBlockSize);
 }

 */



optix::Buffer OpalSceneManager::setReceptionInfoBuffer(optix::uint receivers, optix::uint transmitters) {
	optix::Buffer b = context->createBuffer(RT_BUFFER_OUTPUT, RT_FORMAT_USER, receivers, transmitters);
	b->setElementSize(sizeof(ReceptionInfo));
	context["receptionInfoBuffer"]->set(b);
	return b;
}
optix::Buffer OpalSceneManager::setTransmittersOriginBuffer(optix::uint transmitters) {
	optix::Buffer b = context->createBuffer(RT_BUFFER_INPUT, RT_FORMAT_USER, transmitters);
	b->setElementSize(sizeof(Transmitter));
	context["tx_origin"]->set(b);
	return b;
}
/*optix::Buffer OpalSceneManager::setDuplicatesBuffer(optix::uint receivers, optix::uint transmitters, uint2 duplicateBlockSize) {
	
	optix::Buffer  duplicatesBuffer = context->createBuffer(RT_BUFFER_INPUT, RT_FORMAT_BUFFER_ID, receivers);
	RTsize elevationBlocks =raySphere.elevationSteps / duplicateBlockSize.x;
	//Only half of the azimuth steps are used for duplicates
	RTsize azimuthBlocks = (raySphere.azimuthSteps/2) / duplicateBlockSize.y;
	//std::cout << "elBlocks=" << elevationBlocks << "azBlocks=" << azimuthBlocks << std::endl;;
	int* buffers = static_cast<int*>(duplicatesBuffer->map());
	for (unsigned int i = 0; i < receivers; i++)
	{
		optix::Buffer aux = context->createBuffer(RT_BUFFER_INPUT_OUTPUT | RT_BUFFER_GPU_LOCAL, RT_FORMAT_UNSIGNED_INT, elevationBlocks, azimuthBlocks, transmitters);
		buffers[i] = aux->getId();

	}
	duplicatesBuffer->unmap();
	context["duplicatesBuffer"]->set(duplicatesBuffer);
	context["duplicateBlockSize"]->setUint(duplicateBlockSize);
#ifdef OPALDEBUG
	outputFile << "elevationBlocks=" << elevationBlocks << "azimuthBlocks=" << azimuthBlocks << std::endl;
#endif
	return duplicatesBuffer;
}
*/

optix::Buffer OpalSceneManager::setFacesBuffer(optix::uint receivers) {

	optix::Buffer b = context->createBuffer(RT_BUFFER_OUTPUT, RT_FORMAT_UNSIGNED_INT, numberOfFaces, receivers);
	
	context["facesBuffer"]->set(b);
	return b;


}
optix::Buffer OpalSceneManager::setFacesMinDBuffer(optix::uint receivers) {

	optix::Buffer b = context->createBuffer(RT_BUFFER_OUTPUT, RT_FORMAT_INT, maxReflections, numberOfFaces, receivers);

	context["bufferMinD"]->set(b);
	return b;



}
optix::Buffer OpalSceneManager::setFacesMinEBuffer(optix::uint receivers) {
	optix::Buffer c = context->createBuffer(RT_BUFFER_OUTPUT, RT_FORMAT_USER, maxReflections, numberOfFaces, receivers);
	c->setElementSize(sizeof(DuplicateReflection));
	context["bufferMinE"]->set(c);
	return c;
}
optix::Buffer OpalSceneManager::setFacesMinEBuffersHoldReflections(optix::uint receivers) {

	
	optix::Buffer c = context->createBuffer(RT_BUFFER_INPUT_OUTPUT, RT_FORMAT_USER, maxReflections, numberOfFaces, receivers);
	c->setElementSize(sizeof(DuplicateReflection));
	context["bufferMinE"]->set(c);
	return c;


}
optix::Buffer OpalSceneManager::setInternalRaysBuffer(optix::uint receivers, optix::uint transmitters) {

	optix::Buffer  internalRaysBuffer = context->createBuffer(RT_BUFFER_INPUT, RT_FORMAT_BUFFER_ID, receivers);
	int* buffers = static_cast<int*>(internalRaysBuffer->map());
	for (size_t i = 0; i < receivers; i++)
	{
		optix::Buffer aux = context->createBuffer(RT_BUFFER_INPUT_OUTPUT | RT_BUFFER_GPU_LOCAL, RT_FORMAT_INT, raySphere.elevationSteps, raySphere.azimuthSteps, transmitters);
		internalRays.push_back(aux);
		buffers[i] = aux->getId();

	}
	internalRaysBuffer->unmap();
	context["internalRaysBuffer"]->set(internalRaysBuffer);
	
	return internalRaysBuffer;
}

void OpalSceneManager::addReceiver(int id, float3  position, float radius, std::function<void(float, int)>  callback)
{
	//std::ofstream myfile;
	//myfile.open("D:\\log.txt", std::ifstream::app);
	//myfile << "Adding receiver " << id << std::endl;
		std::cout << "Adding receiver " << id << std::endl;
		optix::Geometry geometry = context->createGeometry();
		float4 sphere = make_float4(position.x, position.y, position.z, radius);
		
		geometry->setPrimitiveCount(1);
		geometry->setBoundingBoxProgram(defaultPrograms.at("receiverBounds"));
		geometry->setIntersectionProgram(defaultPrograms.at("receiverIntersection"));
		std::vector<optix::Material> optix_materials;
		optix::Program chrx = defaultPrograms.at("receiverClosestHit");
	
		
		//chrx["sphere"]->setFloat(sphere);
		optix::Material mat = context->createMaterial();
		mat->setClosestHitProgram(0u, chrx);
		optix_materials.push_back(mat);


		optix::GeometryInstance geom_instance = context->createGeometryInstance(geometry, optix_materials.begin(), optix_materials.end());

		//We use a common program for all the receivers: individual variables must be set per geometry instance
		
		//Add id and position to instace
		geom_instance["sphere"]->setFloat(sphere);

		//The buffer id is consecutive, independent of the external Id
		
		uint nextId = static_cast<uint>(receivers.size());
		geom_instance["receiverId"]->setUint(nextId);
		geom_instance["externalId"]->setUint(id);
		SphereReceiver* rx = new SphereReceiver();
		rx->geomInstance = geom_instance;
		rx->position = position;
		rx->radius = radius;
		rx->callback = callback;
		rx->closestHitProgram = chrx;
		rx->externalId = id;

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

			//Update buffers
			updateReceiverBuffers(nextId, newReceivers);
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
	std::cout << "Opal:: Removing receiver " << id << std::endl;
	SphereReceiver* rx = receivers[index];
	GeometryInstance gi = rx->geomInstance;
	Geometry g = gi->getGeometry();
	receiversGroup->removeChild(gi);
	g->removeReference();
	gi->removeReference();
	receiversGroup->getAcceleration()->markDirty();

	rootGroup->getAcceleration()->markDirty();

	//context->validate();
	delete rx;
	//Swap contents in the buffers
	receivers[index] = receivers[receivers.size() - 1];
	receivers.pop_back();
	receiverExtToBufferId.erase(id);
	//Update buffers
	if (sceneFinished) {
		uint s = static_cast<uint>(receivers.size());
		updateReceiverBuffers(s + 1u, s);
	}
/*	std::map<int, SphereReceiver*>::iterator it;
	int last = static_cast<int>(receivers.size()) - 1; //The last element always has this key
	
	for ( it= receivers.begin(); it != receivers.end(); ++it) {
		
		if (it->second->externalId == id) {
			
			std::cout << "Opal:: Removing receiver " << id << std::endl;
			SphereReceiver* rx = it->second;
			GeometryInstance gi = rx->geomInstance;
			
			receiversGroup->removeChild(gi);
			
			receiversGroup->getAcceleration()->markDirty();
			
			rootGroup->getAcceleration()->markDirty();
			//context->validate();
			
			delete rx;
			break;
		}
	}

	if (it == receivers.end()) {
		//Not found
		std::cout << "Not found receiver " << id << std::endl;
		throw  opal::Exception("Receiver is not registered with Opal");
		return;
	}

	
		if (it->first == last) {
			//This is the last element, just remove

			receivers.erase(it);
		}
		else {
			//Swap the contents with the last element
			it->second = receivers.at(last);
			it->second->geomInstance["receiverId"]->setUint(it->first);

			//remove the last element
			receivers.erase(last);
		}

		//Update buffers
		if (sceneFinished) {
			uint s = static_cast<uint>(receivers.size());
			updateReceiverBuffers(s + 1u, s);
		}
	
	*/
	

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
	std::cout << "sphere=" << position << "f4=" << make_float4(position.x, position.y, position.z, r->radius) << std::endl;
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
	//std::ofstream myfile;
	//myfile.open("D:\\log.txt", std::ifstream::app);
	//myfile << "Transmitting from (" <<origin.x<<","<<origin.y<<","<<origin.z<<") "<< txId << std::endl;
		//sceneManager->sceneContext["ray_origin"]->setFloat(origin); //Create buffer for tx

	
		//printInternalBuffersState();
		Transmitter* host_tx = reinterpret_cast<Transmitter*>  (txOriginBuffer->map());
		host_tx[txId].origin = origin;
		host_tx[txId].polarization = polarization;
		txOriginBuffer->unmap();
		
		context["tx_rx"]->setUint(make_uint2(1u, static_cast<uint>(receivers.size()))); //Move to add/remove transmitters and receiver
		//sceneManager->outputFile << sceneManager->transmissions<< "Tx from (" << origin.x << ","<<origin.y <<","<< origin.z << ")"<<std::endl;
		transmissions++;	
	

		//Update receiver radius
		/*for (int i = 0; i < receivers.size(); i++)
		{
			float distance = length(origin - receivers[i]->position);
			float radius = 10.0f*deg2rad*distance/sqrt(3.0f);

			std::cout << "d="<<distance<<"radius=" << radius << std::endl;
			receivers[i]->geomInstance["sphere"]->setFloat(make_float4(receivers[i]->position,radius));

		}*/
		
		//First initialize
		context->launch(0, raySphere.elevationSteps, raySphere.azimuthSteps, 1);//Launch 3D (elevation, azimut,transmitters)
		//context["initialize"]->setUint(0u);

		
		context->launch(1, raySphere.elevationSteps, raySphere.azimuthSteps, 1);
		//One ray
		//sceneManager->sceneContext->launch(0, 1, 1, 1);

#ifdef OPALDEBUG
		DuplicateReflection* ref_host;
		if (holdReflections) {
			ref_host = reinterpret_cast<DuplicateReflection*>(facesMinEBuffer->map(0, RT_BUFFER_MAP_READ));
		}
#endif // OPALDEBUG

		//Use hold reflections for debug
		//DuplicateReflection* ref_host = reinterpret_cast<DuplicateReflection*>(facesMinEBuffer->map(0, RT_BUFFER_MAP_READ)); 


		ReceptionInfo* host_hits = reinterpret_cast<ReceptionInfo*>  (receptionInfoBuffer->map(0, RT_BUFFER_MAP_READ));
		int hitCount = 0;
		unsigned int width = static_cast<unsigned int>(receivers.size());
		//myfile << "receivers =" << width << std::endl;
		for (unsigned int  x = 0; x < width; ++x)
		{
			for (unsigned int y = 0; y < 1; ++y)
			{
				unsigned int index = x + y*width;
				if (host_hits[index].directHits > 0 || host_hits[index].reflections > 0) {
					//Compute power and callback
					float2 E = host_hits[index].sumRxElectricField;
					
#ifdef OPALDEBUG
					//used with hold reflections for debug
					if (holdReflections) {
						std::cout <<"DH E=" << E << std::endl;
						E += sumReflections(ref_host, x);
					}
#endif // OPALDEBUG

				

					float power = defaultChannel.eA*((E.x*E.x) + (E.y*E.y))*txPower;
					std::cout << "rx=" << receivers[x]->position <<".r="<<receivers[x]->radius <<". tx=" << origin << ".eA=" << defaultChannel.eA << ". txPower=" << txPower << "E=(" << E.x << "," << E.y << ")" << "p=" << power << "dh=" << host_hits[index].directHits << "rf=" << host_hits[index].reflections << "d=" << length(origin - receivers[x]->position) << std::endl;
#ifdef OPALDEBUG
					outputFile << "rx=" << receivers[x]->position << ".r=" << receivers[x]->radius << ". tx=" << origin << ".eA=" << defaultChannel.eA << ". txPower=" << txPower << "E=(" << E.x << "," << E.y << ")" << "p=" << power <<"dh="<< host_hits[index].directHits <<"rf="<< host_hits[index].reflections <<"d="<< length(origin - receivers[x]->position) << std::endl;
#endif // OPALDEBUG
					receivers[x]->callback(power,y);
				}
			}

		}

		receptionInfoBuffer->unmap();
#ifdef OPALDEBUG
		//used with hold reflections for debug
		if (holdReflections) {
			facesMinEBuffer->unmap();
		}
#endif // OPALDEBUG

		//used with hold reflections for debug
		//facesMinEBuffer->unmap();
	
		//myfile.close();
	


}

optix::float2 OpalSceneManager::sumReflections(DuplicateReflection* ref_host, unsigned int receiver) {

	
	
	float2 sum = make_float2(0.0f, 0.0f);
	for (unsigned int x = 0; x < maxReflections; x++)
	{
		for (unsigned int  y = 0; y < numberOfFaces; y++)
		{
			unsigned int index = x + y*maxReflections + numberOfFaces*maxReflections*receiver;
			std::cout << "ref ref="<<(x+1) << "faceId="<<y << "E=" << ref_host[index].E<< std::endl;
			
			sum += ref_host[index].E;
			
		}

	}
	return sum;
}


void OpalSceneManager::updateReceiverBuffers(uint oldReceivers, uint newReceivers) {
	if (sceneFinished == true) {
	
		receptionInfoBuffer->setSize(newReceivers, 1u);
		
		facesBuffer->setSize(numberOfFaces, newReceivers);
		
	
		facesMinDBuffer->setSize(maxReflections, numberOfFaces, newReceivers);
		
		
		facesMinEBuffer->setSize(maxReflections, numberOfFaces, newReceivers);
	
		if (newReceivers - oldReceivers >= 0) {
			
			//Create new internal buffers
			internalRaysBuffer->setSize(newReceivers);
			int* buffers = static_cast<int*>(internalRaysBuffer->map());
			for (unsigned int i = 0; i < newReceivers; i++)
			{
				
				if (i < oldReceivers) {
					buffers[i] = internalRays[i]->getId();
				}
				else {
					optix::Buffer aux = context->createBuffer(RT_BUFFER_INPUT_OUTPUT | RT_BUFFER_GPU_LOCAL, RT_FORMAT_INT, raySphere.elevationSteps, raySphere.azimuthSteps, 1u);
					internalRays.push_back(aux);
					buffers[i] = aux->getId();
				}
			}
			internalRaysBuffer->unmap();
		}
		else {
			//Remove internal buffers
			for (unsigned int i = newReceivers; i < oldReceivers; i++)
			{
				internalRays[i]->destroy();
				internalRays.pop_back();
			}
			internalRaysBuffer->setSize(newReceivers);
			int* buffers = static_cast<int*>(internalRaysBuffer->map());
			for (unsigned int i = 0; i < newReceivers; i++)
			{
				
			
					buffers[i] = internalRays[i]->getId();
				
			}
			internalRaysBuffer->unmap();
		}
		//context["internalRaysBuffer"]->set(internalRaysBuffer);
		context->validate();
		
		
		
		
	}
	else {
		throw  opal::Exception("Buffers not created yet. Call finishSceneContext() before updating any buffer");
	}
}

void OpalSceneManager::recreateReceiverBuffers() {
	
	receptionInfoBuffer->destroy();
	facesBuffer->destroy();
	facesMinDBuffer->destroy();
	facesMinEBuffer->destroy();

	for (unsigned int i = 0; i < internalRays.size(); i++)
	{
	internalRays[i]->destroy();
	}
	internalRays.clear();

	

	receptionInfoBuffer = setReceptionInfoBuffer(static_cast<uint>(receivers.size()), 1);
	//txOriginBuffer = setTransmittersOriginBuffer(1);
	//setDuplicatesBuffer(static_cast<uint>(receivers.size()), 1u, duplicates.duplicateBlockSize);
	facesBuffer = setFacesBuffer(static_cast<uint>(receivers.size()));
	facesMinDBuffer = setFacesMinDBuffer(static_cast<uint>(receivers.size()));
	facesMinEBuffer = setFacesMinEBuffer(static_cast<uint>(receivers.size()));
	internalRaysBuffer = setInternalRaysBuffer(static_cast<uint>(receivers.size()), 1u);
	

}

void opal::OpalSceneManager::updateFacesBuffers()
{
	if (sceneFinished) {
		facesBuffer->setSize(numberOfFaces, static_cast<unsigned int>(receivers.size()));


		facesMinDBuffer->setSize(maxReflections, numberOfFaces, static_cast<unsigned int>(receivers.size()));


		facesMinEBuffer->setSize(maxReflections, numberOfFaces, static_cast<unsigned int>(receivers.size()));

		context["number_of_faces"]->setUint(numberOfFaces);
	}
}

void opal::OpalSceneManager::printInternalBuffersState()
{
	RTsize w;
	RTsize h;
	RTsize d;
	receptionInfoBuffer->getSize(w,h);
	std::cout << "receptionInfoBuffer=(" << w<<","<< h<<")"<< std::endl;
	facesBuffer->getSize(w, h);
	std::cout << "facesBuffer=(" << w << "," << h << ")" << std::endl;
	facesMinDBuffer->getSize(w, h,d);
	std::cout << "facesMinDBuffer=(" << w << "," << h<< ","<<d<<")" << std::endl;
	facesMinEBuffer->getSize(w, h, d);
	std::cout << "facesMinEBuffer=(" << w << "," << h << "," << d << ")" << std::endl;
	internalRaysBuffer->getSize(w);
	std::cout << "internalRaysBuffer=(" << w << ")" << std::endl;
	for (size_t i = 0; i < internalRays.size(); i++)
	{
		internalRays[i]->getSize(w, h, d);
		std::cout << "internalRays["<<i<<"]=(" << w << "," << h << "," << d << ")" << std::endl;
	}
}

void OpalSceneManager::finishSceneContext() {
	
	if (raySphere.rayCount <= 0) {
		throw  opal::Exception("Scene not created. Ray count=0. Create Ray sphere before finishing scene context");
	}
	
	//Build scene graph
	buildSceneGraph();


	receptionInfoBuffer = setReceptionInfoBuffer(static_cast<uint>(receivers.size()), 1);
	txOriginBuffer = setTransmittersOriginBuffer(1);
	//setDuplicatesBuffer(static_cast<uint>(receivers.size()), 1u, duplicates.duplicateBlockSize);
	facesBuffer=setFacesBuffer(static_cast<uint>(receivers.size()));
	facesMinDBuffer=setFacesMinDBuffer(static_cast<uint>(receivers.size()));

#ifdef OPALDEBUG
	// Use with hold reflections for debug
	if (holdReflections) {
		facesMinEBuffer = setFacesMinEBuffersHoldReflections(static_cast<uint>(receivers.size()));
	}
	else {
		facesMinEBuffer = setFacesMinEBuffer(static_cast<uint>(receivers.size()));
	}
#else
	facesMinEBuffer = setFacesMinEBuffer(static_cast<uint>(receivers.size()));
#endif // OPALDEBUG

	
	internalRaysBuffer= setInternalRaysBuffer(static_cast<uint>(receivers.size()), 1u);
	
	//Set some minimum extent for the rays. Otherwise we are going to run into numerical accuracy problems with the reflections, almost surely
	context["number_of_faces"]->setUint(numberOfFaces);
	context["min_t_epsilon"]->setFloat(minEpsilon);
	context["max_interactions"]->setUint(maxReflections);

	context->setRayGenerationProgram(0, defaultPrograms.at("initialize"));//Initialization
	context->setRayGenerationProgram(1, defaultPrograms.at("rayGeneration")); //Generation
	context->setMissProgram(0, defaultPrograms.at("miss"));
	context->validate();
	//context->setExceptionEnabled(RT_EXCEPTION_BUFFER_INDEX_OUT_OF_BOUNDS, true);
	sceneFinished = true;
	std::cout << "finishSceneContext()-- Summary---" << std::endl;
	std::cout << "numberOfFaces=" << numberOfFaces << ". minEpsilon= " << minEpsilon << ". maxReflections=" << maxReflections << std::endl;
	std::cout << "Receivers=" << receivers.size() << ". Trasnmitters=" << transmitters.size() << std::endl;
	std::cout << "RayCount=" << raySphere.rayCount << "azimuthSteps=" << raySphere.azimuthSteps << "elevationSteps" << raySphere.elevationSteps << std::endl;
	printInternalBuffersState();
	std::cout << "-----" << std::endl;
}

void opal::OpalSceneManager::setPrintEnabled(int bufferSize)
{

	context->setPrintEnabled(true);
	context->setPrintBufferSize(bufferSize);
	std::cout << "printEnabled" << std::endl;
	
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
	context["staticMeshes"]->set(staticMeshesGroup);

	
	//Create Geometry group for receivers
    receiversGroup = context->createGeometryGroup();
	receiversGroup->setChildCount(static_cast<uint>(receivers.size()));
	unsigned int cindex = 0;
	for (const auto val : receivers)
	{
		receiversGroup->setChild(cindex, val->geomInstance);
#ifdef OPALDEBUG
		outputFile << "Receiver[ " << cindex << "].position=" << val->position << std::endl;
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
	outputFile << "Building scene graph. Static  meshes: " << staticMeshes.size() << ". Receivers: " << receivers.size() << std::endl;
#endif // OPALDEBUG

	
}
void OpalSceneManager::callbackUsageReport(int level, const char* tag, const char* msg, void* cbdata)
{
	std::cout << "[" << level << "][" << std::left << std::setw(12) << tag << "] " << msg;
}












void printPower(float power, int txId) {
	std::cout << "PR\t" << power << std::endl;
}


//Tests. Compile as exe
int main(int argc, char** argv)
{
	try {
		//Timer m_timer;
		
		//std::cout << "Initializing " << std::endl;
		float freq = 5.9e9f;
		std::unique_ptr<OpalSceneManager> sceneManager(new OpalSceneManager(freq));
		
		
		
	
		//m_timer.restart();
		//const double timeInit = m_timer.getTime();

		
		//sceneManager = addRemoveDynamicMeshes(std::move(sceneManager));
		//sceneManager = addRemoveReceivers(std::move(sceneManager));
		
		//sceneManager = planeTest(std::move(sceneManager));
		//sceneManager = moveReceivers(std::move(sceneManager));
		sceneManager = crossingTest(std::move(sceneManager));
		//const double timeLaunch = m_timer.getTime();
		//std::cout << "Rx=" << rx.x << "+j" << rx.y << std::endl;
		/*std::cout << "initScene(): " << timeLaunch - timeInit << " seconds overall" << std::endl;
		std::cout << "{" << std::endl;
		std::cout << "  createScene() = " << timeScene - timeInit << " seconds" << std::endl;
		std::cout << "  context()    = " << timeContext - timeScene << " seconds" << std::endl;
		std::cout << "  launch()      = " << timeLaunch - timeContext << " seconds" << std::endl;
		std::cout << "}" << std::endl;

		*/
		return 0;
	}
	catch (optix::Exception& e) {
		std::cout << "main error occurred with error code "
			<< e.getErrorCode() << " and message "
			<< e.getErrorString() << std::endl;

		return 0;
	}
	catch (opal::Exception& e) {
		std::cout << "main error occurred with  message "
			<< e.getErrorString()
			<< std::endl;

		return 0;
	}
	
}


//Adding, moving and removing dynamic meshes
std::unique_ptr<OpalSceneManager> addRemoveDynamicMeshes(std::unique_ptr<OpalSceneManager> sceneManager) {
	try {

	//Quad
	int quadind[6] = { 0,1,2,1,0,3 };
	optix::float3 quadv[4] = { make_float3(-0.5f,-0.5f,0.f),make_float3(0.5f,0.5f,0.f) ,make_float3(0.5f,-0.5f,0.f) ,make_float3(-0.5f,0.5f,0.f) };



	Matrix4x4 tm;
	tm.setRow(0, make_float4(1, 0, 0, 0.f));
	tm.setRow(1, make_float4(0, 1, 0, 2.f));
	tm.setRow(2, make_float4(0, 0, 1, 75.f));
	tm.setRow(3, make_float4(0, 0, 0.f, 1));
	MaterialEMProperties emProp1;
	emProp1.dielectricConstant = make_float2(3.75f, -60.0f*sceneManager->defaultChannel.waveLength*0.038f);
	
	//Creation of dynamic meshes  requires calling these 4 functions
	sceneManager->addDynamicMeshGroup(0);
	sceneManager->addMeshToGroup(0,4, quadv, 6, quadind,  emProp1);  //Call for each new mesh in the group
	sceneManager->updateTransformInGroup(0, tm); 
	sceneManager->finishDynamicMeshGroup(0);

	sceneManager->createRaySphere2D(1, 1);
	//sceneManager->createRaySphere2DSubstep(1, 1);
	//receivers
	optix::float3 posrx = make_float3(0.0f, 2.0f, 50.0f);


	optix::float3 postx = make_float3(0.0f, 2.0f, 0.0f);

	sceneManager->addReceiver(1, posrx, 5.0f, printPower);


	sceneManager->finishSceneContext();
	sceneManager->setPrintEnabled(1024 * 1024 * 1024);
	optix::float3 polarization = make_float3(0.0f, 1.0f, 0.0f); //Perpendicular to the floor. Assuming as in Unity that forward is z-axis and up is y-axis

	sceneManager->transmit(0, 1.0f, postx, polarization);

	Matrix4x4 tm1;
	tm1.setRow(0, make_float4(1.f, 0, 0, 0.0f));
	tm1.setRow(1, make_float4(0, 1, 0, 2.f));
	tm1.setRow(2, make_float4(0, 0, 1.0f, -75.f));
	tm1.setRow(3, make_float4(0, 0, 0.f, 1));
	sceneManager->updateTransformInGroup(0, tm1);
	
	posrx = make_float3(0.0f, 2.0f, -50.0f);
	sceneManager->updateReceiver(1, posrx);

	std::cout << "transmit again" << std::endl;
	sceneManager->transmit(0, 1.0f, postx, polarization);

	//Add a new quad 
	sceneManager->addDynamicMeshGroup(1);
	sceneManager->addMeshToGroup(1, 4, quadv, 6, quadind, emProp1);
	sceneManager->updateTransformInGroup(1, tm);
	sceneManager->finishDynamicMeshGroup(1);

	std::cout << "transmit with new quad. Num quads= "<< sceneManager->dynamicMeshes.size() << std::endl;
	sceneManager->transmit(0, 1.0f, postx, polarization);

	//Remove first quad
	sceneManager->removeDynamicMeshGroup(0);
	
	posrx = make_float3(0.0f, 2.0f, 50.0f);
	sceneManager->updateReceiver(1, posrx);
	std::cout << "transmit again  Num quads= " << sceneManager->dynamicMeshes.size() << std::endl;
	Matrix4x4 mym;
	Matrix4x4 mymi;
	sceneManager->dynamicMeshes.at(1)->transform->getMatrix(0, mym.getData(), mymi.getData());
	std::cout << "Tm of quad 1: " <<  mym<< std::endl;
	sceneManager->transmit(0, 1.0f, postx, polarization);

	//Remove second quad
	sceneManager->removeDynamicMeshGroup(1);

	return sceneManager;
}
catch (optix::Exception& e) {
	std::cout << "addRemoveDynamicMeshes occurred with error code "
		<< e.getErrorCode() << " and message "
		<< e.getErrorString() << std::endl;

	return 0;
}
catch (opal::Exception& e) {
	std::cout << "addRemoveDynamicMeshes occurred with  message "
		<< e.getErrorString()
		<< std::endl;

	return 0;
}


}


//Adding and removing dynamic meshes
std::unique_ptr<OpalSceneManager> addRemoveReceivers(std::unique_ptr<OpalSceneManager> sceneManager) {
	try {
		//Horizontal plane
		std::vector<int> planeind = loadTrianglesFromFile("D:\\tri.txt");
		std::vector<float3> planever = loadVerticesFromFile("D:\\vert.txt");
		//std::cout << "indices=" << planeind.size() << "vertices=" << planever.size() << std::endl;
		Matrix4x4 tm;
		tm.setRow(0, make_float4(10.0f, 0, 0, 0.0f));
		tm.setRow(1, make_float4(0, 1, 0, 0.0f));
		tm.setRow(2, make_float4(0, 0, 10.0f, 50.0f));
		tm.setRow(3, make_float4(0, 0, 0, 1));
		MaterialEMProperties emProp1;
		emProp1.dielectricConstant = make_float2(3.75f, -60.0f*sceneManager->defaultChannel.waveLength*0.038f);
		sceneManager->addStaticMesh(static_cast<int>(planever.size()), planever.data(), static_cast<int>(planeind.size()), planeind.data(), tm, emProp1);


		sceneManager->createRaySphere2D(1, 1);
		//sceneManager->createRaySphere2DSubstep(1, 1);
		//receivers
		optix::float3 posrx = make_float3(0.0f, 2.0f, 100.0f);


		optix::float3 postx = make_float3(0.0f, 2.0f, 50.0f);

		sceneManager->addReceiver(1, posrx, 5.0f, printPower);


		sceneManager->finishSceneContext();
		sceneManager->setPrintEnabled(1024 * 1024 * 1024);
		optix::float3 polarization = make_float3(0.0f, 1.0f, 0.0f); //Perpendicular to the floor. Assuming as in Unity that forward is z-axis and up is y-axis
		sceneManager->transmit(0, 1.0f, postx, polarization);

		//Add new receiver
		posrx = make_float3(0.0f, 2.0f, 70.0f);
		sceneManager->addReceiver(2, posrx, 5.0f, printPower);

		std::cout << "transmit again" << std::endl;
		sceneManager->transmit(0, 1.0f, postx, polarization);


		//Remove receiver
		sceneManager->removeReceiver(2);
		std::cout << "transmit again" << std::endl;
		sceneManager->transmit(0, 1.0f, postx, polarization);
		//std::cout << "Launching" << std::endl;

		return sceneManager;
	}
	catch (optix::Exception& e) {
		std::cout << "addRemoveReceivers occurred with error code "
			<< e.getErrorCode() << " and message "
			<< e.getErrorString() << std::endl;

		return 0;
	}
	catch (opal::Exception& e) {
		std::cout << "addRemoveReceivers occurred with  message "
			<< e.getErrorString()
			<< std::endl;

		return 0;
	}


}



//moving receivers
std::unique_ptr<OpalSceneManager> moveReceivers(std::unique_ptr<OpalSceneManager> sceneManager) {
	try {
	//Horizontal plane
	std::vector<int> planeind = loadTrianglesFromFile("D:\\tri.txt");
	std::vector<float3> planever = loadVerticesFromFile("D:\\vert.txt");
	//std::cout << "indices=" << planeind.size() << "vertices=" << planever.size() << std::endl;
	Matrix4x4 tm;
	tm.setRow(0, make_float4(10.0f, 0, 0, 0.0f));
	tm.setRow(1, make_float4(0, 1, 0, 0.0f));
	tm.setRow(2, make_float4(0, 0, 10.0f, 50.0f));
	tm.setRow(3, make_float4(0, 0, 0, 1));
	MaterialEMProperties emProp1;
	emProp1.dielectricConstant = make_float2(3.75f, -60.0f*sceneManager->defaultChannel.waveLength*0.038f);
	sceneManager->addStaticMesh(static_cast<int>(planever.size()), planever.data(), static_cast<int>(planeind.size()), planeind.data(), tm, emProp1);


	//sceneManager->createRaySphere2D(1, 1);
	sceneManager->createRaySphere2DSubstep(1, 1);
	//receivers
	optix::float3 postx = make_float3(0.0f, 2.0f, 100.0f);


	optix::float3 posrx = make_float3(0.0f, 2.0f, 0.0f);
	optix::float3 polarization = make_float3(0.0f, 1.0f, 0.0f); //Perpendicular to the floor. Assuming as in Unity that forward is z-axis and up is y-axis

	sceneManager->addReceiver(1, posrx, 5.0f, printPower);


	sceneManager->finishSceneContext();
	sceneManager->setPrintEnabled(1024 * 1024 * 1024);
	
	for (size_t i = 0; i < 100; ++i)
	{
		posrx = make_float3(0.0f, 2.0f, 99.0f - i);
		sceneManager->updateReceiver(1, posrx);
		//postx = make_float3(0.0f, 2.0f, 0.f);
		sceneManager->transmit(0, 1.0f, postx, polarization);
		//	postx = make_float3(0.0f, 2.0f, 1.f);
		//sceneManager->transmit(0, 1.0f, postx, polarization);

	}
		

	

	return sceneManager;
	}
	catch (optix::Exception& e) {
		std::cout << "moveReceivers occurred with error code "
			<< e.getErrorCode() << " and message "
			<< e.getErrorString() << std::endl;

		return 0;
	}
	catch (opal::Exception& e) {
		std::cout << "moveReceivers occurred with  message "
			<< e.getErrorString()
			<< std::endl;

		return 0;
	}

	
}



//Street crossing test. Cubes are intended to be buildings and a plane is the floor
std::unique_ptr<OpalSceneManager> crossingTest(std::unique_ptr<OpalSceneManager> sceneManager) {
	//Cubes
	std::vector<int> cubeind = loadTrianglesFromFile("D:\\tricube.txt");
	std::vector<float3> cubevert = loadVerticesFromFile("D:\\vertcube.txt");
	//std::cout << "indices=" << cubeind.size() << "vertices=" << cubevert.size() << std::endl;
	//Cube(4) NW
	Matrix4x4 tm;
	tm.setRow(0, make_float4(40.0f, 0, 0, -30.0f));
	tm.setRow(1, make_float4(0, 40.0f, 0, 20.0f));
	tm.setRow(2, make_float4(0, 0, 40.0f, 80.0f));
	tm.setRow(3, make_float4(0, 0, 0, 1));
	MaterialEMProperties emProp1;
	emProp1.dielectricConstant = make_float2(3.75f, -60.0f*sceneManager->defaultChannel.waveLength*0.038f);
	//emProp1.dielectricConstant = make_float2(3.75f, -0.4576f);
	std::cout << "Adding NW. Em="<< emProp1.dielectricConstant << std::endl;
	sceneManager->addStaticMesh(static_cast<int>(cubevert.size()), cubevert.data(), static_cast<int>(cubeind.size()), cubeind.data(), tm, emProp1);

	//Cube SW
	tm.setRow(0, make_float4(40.0f, 0, 0, -30.0f));
	tm.setRow(1, make_float4(0, 40.0f, 0, 20.0f));
	tm.setRow(2, make_float4(0, 0, 40.0f, 20.0f));
	tm.setRow(3, make_float4(0, 0, 0, 1));
	std::cout << "Adding SW. Em = "<< emProp1.dielectricConstant << std::endl;
	sceneManager->addStaticMesh(static_cast<int>(cubevert.size()), cubevert.data(), static_cast<int>(cubeind.size()), cubeind.data(), tm, emProp1);
	//Cube(2) NE

	tm.setRow(0, make_float4(40.0f, 0, 0, 30.0f));
	tm.setRow(1, make_float4(0, 40.0f, 0, 20.0f));
	tm.setRow(2, make_float4(0, 0, 40.0f, 80.0f));
	tm.setRow(3, make_float4(0, 0, 0, 1));
	std::cout << "Adding NE. Em = "<< emProp1.dielectricConstant << std::endl;
	sceneManager->addStaticMesh(static_cast<int>(cubevert.size()), cubevert.data(), static_cast<int>(cubeind.size()), cubeind.data(), tm, emProp1);

	//Cube(1) SE

	tm.setRow(0, make_float4(40.0f, 0, 0, 30.0f));
	tm.setRow(1, make_float4(0, 40.0f, 0, 20.0f));
	tm.setRow(2, make_float4(0, 0, 40.0f, 20.0f));
	tm.setRow(3, make_float4(0, 0, 0, 1));
	std::cout << "Adding SE. Em = "<< emProp1.dielectricConstant << std::endl;
	sceneManager->addStaticMesh(static_cast<int>(cubevert.size()), cubevert.data(), static_cast<int>(cubeind.size()), cubeind.data(), tm, emProp1);

	//Horizontal plane
	std::vector<int> planeind = loadTrianglesFromFile("D:\\tri.txt");
	std::vector<float3> planever = loadVerticesFromFile("D:\\vert.txt");
	//std::cout << "indices=" << planeind.size() << "vertices=" << planever.size() << std::endl;

	tm.setRow(0, make_float4(10.0f, 0, 0, 0.0f));
	tm.setRow(1, make_float4(0, 1, 0, 0.0f));
	tm.setRow(2, make_float4(0, 0, 10.0f, 50.0f));
	tm.setRow(3, make_float4(0, 0, 0, 1));
	
	//emProp1.dielectricConstant = make_float2(3.75f, -60.0f*sceneManager->defaultChannel.waveLength*0.15f);
	std::cout << "Adding Plane. Em=" << emProp1.dielectricConstant << std::endl;
	sceneManager->addStaticMesh(static_cast<int>(planever.size()), planever.data(), static_cast<int>(planeind.size()), planeind.data(), tm, emProp1);

	sceneManager->createRaySphere2D(1, 1);
	//sceneManager->createRaySphere2DSubstep(1, 1);
	
	//receivers

	optix::float3 posrx = make_float3(0.0f, 10.0f, 100.0f);
	sceneManager->addReceiver(1, posrx, 5.f, printPower);


	sceneManager->setMaxReflections(4u);

	sceneManager->finishSceneContext();


	sceneManager->setPrintEnabled(1024 * 1024 * 1024);
	

	optix::float3 postx;
	optix::float3 polarization = make_float3(0.0f, 1.0f, 0.0f); //Perpendicular to the floor. Assuming as in Unity that forward is z-axis and up is y-axis
																
	for (int i = -50; i <= 50; ++i)
	{
		postx = make_float3(i, 10.0f, 50.0f);

		sceneManager->transmit(0, 1.0f, postx, polarization);


	}
	//postx = make_float3(-18.0f, 10.0f, 50.0f);
	//sceneManager->transmit(0, 1.0f, postx, polarization);
	
	return sceneManager;
	
}


//Horizontal plane test. To validate against a two-ray model
std::unique_ptr<OpalSceneManager> planeTest(std::unique_ptr<OpalSceneManager> sceneManager) {
	//Horizontal plane
	std::vector<int> planeind = loadTrianglesFromFile("D:\\tri.txt");
	std::vector<float3> planever = loadVerticesFromFile("D:\\vert.txt");
	//std::cout << "indices=" << planeind.size() << "vertices=" << planever.size() << std::endl;
	Matrix4x4 tm;
	tm.setRow(0, make_float4(10.0f, 0, 0, 0.0f));
	tm.setRow(1, make_float4(0, 1, 0, 0.0f));
	tm.setRow(2, make_float4(0, 0, 10.0f, 50.0f));
	tm.setRow(3, make_float4(0, 0, 0, 1));
	MaterialEMProperties emProp1;
	emProp1.dielectricConstant = make_float2(3.75f, -60.0f*sceneManager->defaultChannel.waveLength*0.038f);
	sceneManager->addStaticMesh(static_cast<int>(planever.size()), planever.data(), static_cast<int>(planeind.size()), planeind.data(), tm, emProp1);


	//sceneManager->createRaySphere2D(1, 1);
	sceneManager->createRaySphere2DSubstep(1, 1);
	//receivers
	optix::float3 posrx = make_float3(0.0f, 2.0f, 100.0f);

	
	optix::float3 postx = make_float3(0.0f, 2.0f, 50.0f);

	sceneManager->addReceiver(1, posrx, 5.0f, printPower);


	sceneManager->finishSceneContext();
	

	sceneManager->setPrintEnabled(1024 * 1024 * 1024);


	//std::cout << "Launching" << std::endl;

	
	optix::float3 polarization = make_float3(0.0f, 1.0f, 0.0f); //Perpendicular to the floor. Assuming as in Unity that forward is z-axis and up is y-axis
													
	for (size_t i = 0; i < 100; ++i)
	{
		postx = make_float3(0.0f, 2.0f, 99.0f - i);
		//postx = make_float3(0.0f, 2.0f, 0.f);
		sceneManager->transmit(0, 1.0f, postx, polarization);
		//	postx = make_float3(0.0f, 2.0f, 1.f);
		//sceneManager->transmit(0, 1.0f, postx, polarization);

	}
	return sceneManager;
}


//Two quads as walls and two overlapping receivers
std::unique_ptr<OpalSceneManager> quadTest(std::unique_ptr<OpalSceneManager> sceneManager) {
	//First quad
	int quadind[6] = { 0,1,2,1,0,3 };
	optix::float3 quadv[4] = { make_float3(-0.5f,-0.5f,-3.06162e-17f),make_float3(0.5f,0.5f,3.06162e-17f) ,make_float3(0.5f,-0.5f,-3.06162e-17f) ,make_float3(-0.5f,0.5f,3.06162e-17f) };


	Matrix4x4 tm;
	tm.setRow(0, make_float4(1, 0, 0, 0.302f));
	tm.setRow(1, make_float4(0, 1, 0, 0.931f));
	tm.setRow(2, make_float4(0, 0, 1, 0.684f));
	tm.setRow(3, make_float4(0, 0, 0,  1));
	MaterialEMProperties emProp1;
	emProp1.dielectricConstant = make_float2(3.75f, -60.0f*sceneManager->defaultChannel.waveLength*0.038f);

	sceneManager->addStaticMesh(4, quadv, 6, quadind, tm, emProp1 );

	//Second quad
	int quadind2[6] = { 0,1,2,1,0,3 };
	optix::float3 quadv2[4] = { make_float3(-0.5f,-0.5f,-3.06162e-17f),make_float3(0.5f,0.5f,3.06162e-17f) ,make_float3(0.5f,-0.5f,-3.06162e-17f) ,make_float3(-0.5f,0.5f,3.06162e-17f) };


	Matrix4x4 tm2;
	tm2.setRow(0, make_float4(1, 0, 0, 0.14f));
	tm2.setRow(1, make_float4(0, 1, 0, 0.83f));
	tm2.setRow(2, make_float4(0, 0, 1, -12.43f));
	tm2.setRow(3, make_float4(0, 0, 0, 1));
	MaterialEMProperties emProp2;
	emProp2.dielectricConstant = make_float2(3.75f, -60.0f*sceneManager->defaultChannel.waveLength*0.038f);


	sceneManager->addStaticMesh(4, quadv2, 6, quadind2, tm2, emProp2);
	optix::float3 posrx = make_float3(0.53f, -2.04f, -4.73f);
	sceneManager->addReceiver(1, make_float3(0.53f, -2.04f, -2.45f), 5.0f, printPower);
	optix::float3 postx = make_float3(0.0f, 1.0f, -10.68f);
	
	optix::float3 polarization = make_float3(0.0f, 1.0f, 0.0f); //Perpendicular to the floor. Assuming as in Unity that forward is z-axis and up is y-axis
	sceneManager->transmit(0, 0.158f,postx, polarization);
	return sceneManager;
}


std::vector<float3>  loadVerticesFromFile(const char* file) {
	std::ifstream infile(file);
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
	infile.close();
	
	return vertices;
}
std::vector<int>  loadTrianglesFromFile(const char* file) {
	std::ifstream infile(file);
	int i;
	std::vector<int> triangles;

	while (infile>>i) {
		//std::cout << i << std::endl;
		triangles.push_back(i);
	}
	infile.close();
	return triangles;
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

// This code is part of the NVIDIA nvpro-pipeline https://github.com/nvpro-pipeline/pipeline



#if defined(_WIN32)
#  define GETTIME(x) QueryPerformanceCounter(x)
#else     
#  define GETTIME(x) gettimeofday( x, 0 )
#endif 

Timer::Timer()
	: m_running(false)
	, m_seconds(0)
{
#if defined(_WIN32)
	QueryPerformanceFrequency(&m_freq);
#endif
}

Timer::~Timer()
{}

void Timer::start()
{
	if (!m_running)
	{
		m_running = true;
		// starting a timer: store starting time last
		GETTIME(&m_begin);
	}
}

void Timer::stop()
{
	// stopping a timer: store stopping time first
	Time tmp;
	GETTIME(&tmp);
	if (m_running)
	{
		m_seconds += calcDuration(m_begin, tmp);
		m_running = false;
	}
}

void Timer::reset()
{
	m_running = false;
	m_seconds = 0;
}

void Timer::restart()
{
	reset();
	start();
}

double Timer::getTime() const
{
	Time tmp;
	GETTIME(&tmp);
	if (m_running)
	{
		return m_seconds + calcDuration(m_begin, tmp);
	}
	else
	{
		return m_seconds;
	}
}

double Timer::calcDuration(Time begin, Time end) const
{
	double seconds;
#if defined(_WIN32)
	LARGE_INTEGER diff;
	diff.QuadPart = (end.QuadPart - begin.QuadPart);
	seconds = (double)diff.QuadPart / (double)m_freq.QuadPart;
#else
	timeval diff;
	if (begin.tv_usec <= end.tv_usec)
	{
		diff.tv_sec = end.tv_sec - begin.tv_sec;
		diff.tv_usec = end.tv_usec - begin.tv_usec;
	}
	else
	{
		diff.tv_sec = end.tv_sec - begin.tv_sec - 1;
		diff.tv_usec = end.tv_usec - begin.tv_usec + (int)1e6;
	}
	seconds = diff.tv_sec + diff.tv_usec / 1e6;
#endif
	return seconds;
}

