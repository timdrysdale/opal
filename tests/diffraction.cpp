/***************************************************************/
//
//Copyright (c) 2021 Esteban Egea-Lopez http://girtel.upct.es/~eegea
//
/**************************************************************/

#include "diffraction.h"
#include <limits>
#include "../singleDiffraction.h"
#include "../flatSimulation.h"
#include "../basicSimulation.h"
#include "../rayDensityNormalizationSimulation.h"
#include "../util.h"
#include "../timer.h"

using namespace optix;
using namespace opal;
DiffractionTests::DiffractionTests(OpalSceneManager*   sceneManager, float sphereRadius, bool useDepolarization) {
	this->sceneManager=sceneManager;
	this->sphereRadius=sphereRadius;
	this->useDepolarization;	
}
void DiffractionTests::loadSWCorner(float xsize, float ysize, float zsize, MaterialEMProperties emProp1) {
	//Plane XZ as quad with left border at X=0 
	int quadind[6] = { 0,1,2,1,3,2 };
	optix::float3 quadh[4] = { make_float3(0.0f,0.0f,-0.5f),make_float3(0.0f,0.f,0.5f) ,make_float3(1.0f,0.f,-0.5f) ,make_float3(1.0f,0.0f,0.5f) };

	//Scale 
	Matrix4x4 tm;
	tm.setRow(0, make_float4(xsize, 0, 0, 0.f));
	tm.setRow(1, make_float4(0, 1, 0, 0.f));
	tm.setRow(2, make_float4(0, 0, zsize, 0.f));
	tm.setRow(3, make_float4(0, 0, 0,  1));

	std::vector<float3> v(std::begin(quadh), std::end(quadh));
	std::vector<int> ind(std::begin(quadind), std::end(quadind));
	std::cout<<"Writing quad" <<std::endl;
	sceneManager->writeMeshToPLYFile("quad-dif.ply", v,ind, tm);

	sceneManager->addStaticMesh(4, quadh, 6, quadind, tm, emProp1 );
	
	//Plane YZ as quad with up border at Y=0
	int quadind2[6] = { 0,1,2,1,3,2 };
	optix::float3 quadh2[4] = { make_float3(0.0f,0.0f,-0.5f),make_float3(0.0f,0.f,0.5f) ,make_float3(0.0f,-1.0f,-0.5f) ,make_float3(0.0f,-1.0f,0.5f) };

	//Scale 
	tm.setRow(0, make_float4(0, 0, 0, 0.f));
	tm.setRow(1, make_float4(0, ysize, 0, 0.f));
	tm.setRow(2, make_float4(0, 0, zsize, 0.f));
	tm.setRow(3, make_float4(0, 0, 0,  1));
	
	std::vector<float3> v2(std::begin(quadh2), std::end(quadh2));
	std::vector<int> ind2(std::begin(quadind2), std::end(quadind2));
	std::cout<<"Writing quad" <<std::endl;
	sceneManager->writeMeshToPLYFile("quad-dif2.ply", v2,ind2, tm);


	sceneManager->addStaticMesh(4, quadh2, 6, quadind2, tm, emProp1 );
	float3 x=make_float3(1.0f,0.0f,0.0f);//East (x axis)
	float3 y=make_float3(0.0f,1.0f,0.0f); //Up (Y axis)
	float3 z=make_float3(0.0f,0,1.0f);//North (Z axis)

	//Edge is defined along the Z axis. Face a is X axis, face b is -Y
	sceneManager->addEdge(make_float3(0.0f,0.0f,-zsize*0.5f),zsize*z,make_uint2(0u,1u),-y,x,-x,y, emProp1);
}
void DiffractionTests::loadSWCornerAsGroup(float xsize, float ysize, float zsize, MaterialEMProperties emProp1) {
	//Plane XZ as quad with left border at X=0 
	int quadind[6] = { 0,1,2,1,3,2 };
	optix::float3 quadh[4] = { make_float3(0.0f,0.0f,-0.5f),make_float3(0.0f,0.f,0.5f) ,make_float3(1.0f,0.f,-0.5f) ,make_float3(1.0f,0.0f,0.5f) };

	//Scale 
	Matrix4x4 tm;
	tm.setRow(0, make_float4(xsize, 0, 0, 0.f));
	tm.setRow(1, make_float4(0, 1, 0, 0.f));
	tm.setRow(2, make_float4(0, 0, zsize, 0.f));
	tm.setRow(3, make_float4(0, 0, 0,  1));

	std::vector<float3> v(std::begin(quadh), std::end(quadh));
	//Transform to scale before adding to group, we do not want to scale the group
	for (int i=0; i< v.size(); ++i) {
		float4 p=make_float4(v[i]);
		p.w=1.0f;
		quadh[i]=make_float3(tm*p);
		
		std::cout<<"quadh["<<i<<"]="<<quadh[i]<<std::endl;
	}
	std::vector<int> ind(std::begin(quadind), std::end(quadind));
	std::cout<<"Writing quad" <<std::endl;
	sceneManager->writeMeshToPLYFile("quad-dif.ply", v,ind, tm);
		
//Creation of dynamic meshes  requires calling these 4 functions
	sceneManager->addDynamicMeshGroup(0);
	sceneManager->addMeshToGroup(0, 4, quadh, 6, quadind, emProp1);  //Call for each new mesh in the group

	
	//Plane YZ as quad with up border at Y=0
	int quadind2[6] = { 0,1,2,1,3,2 };
	optix::float3 quadh2[4] = { make_float3(0.0f,0.0f,-0.5f),make_float3(0.0f,0.f,0.5f) ,make_float3(0.0f,-1.0f,-0.5f) ,make_float3(0.0f,-1.0f,0.5f) };

	//Scale 
	tm.setRow(0, make_float4(0, 0, 0, 0.f));
	tm.setRow(1, make_float4(0, ysize, 0, 0.f));
	tm.setRow(2, make_float4(0, 0, zsize, 0.f));
	tm.setRow(3, make_float4(0, 0, 0,  1));
	
	std::vector<float3> v2(std::begin(quadh2), std::end(quadh2));
	for (int i=0; i< v2.size(); ++i) {
		float4 p=make_float4(v2[i]);
		p.w=1.0f;
		quadh2[i]=make_float3(tm*p);
		std::cout<<"quadh2["<<i<<"]="<<quadh2[i]<<std::endl;
		
	}
	std::vector<int> ind2(std::begin(quadind2), std::end(quadind2));
	std::cout<<"Writing quad" <<std::endl;
	sceneManager->writeMeshToPLYFile("quad-dif2.ply", v2,ind2, tm);


	//sceneManager->addStaticMesh(4, quadh2, 6, quadind2, tm, emProp1 );
	sceneManager->addMeshToGroup(0, 4, quadh2, 6, quadind2, emProp1);  //Call for each new mesh in the group
	float3 x=make_float3(1.0f,0.0f,0.0f);//East (x axis)
	float3 y=make_float3(0.0f,1.0f,0.0f); //Up (Y axis)
	float3 z=make_float3(0.0f,0,1.0f);//North (Z axis)

	//Edge is defined along the Z axis. Face a is -Y axis (z-y plane), face b is X (x-z plane)
	sceneManager->addEdgeToGroup(make_float3(0.0f,0.0f,-zsize*0.5f),zsize*z,make_uint2(0u,1u),-y,x,-x,y, emProp1,0,0);
	tm=Matrix4x4::identity();
	sceneManager->updateTransformInGroup(0, tm);
	sceneManager->finishDynamicMeshGroup(0);
}
void DiffractionTests::loadSWAcuteCorner(float xsize, float ysize, float zsize, float aperture, MaterialEMProperties emProp1) {
	//Plane XZ as quad with left border at X=0 
	int quadind[6] = { 0,1,2,1,3,2 };
	optix::float3 quadh[4] = { make_float3(0.0f,0.0f,-0.5f),make_float3(0.0f,0.f,0.5f) ,make_float3(1.0f,0.f,-0.5f) ,make_float3(1.0f,0.0f,0.5f) };

	//Scale 
	Matrix4x4 tm;
	tm.setRow(0, make_float4(xsize, 0, 0, 0.f));
	tm.setRow(1, make_float4(0, 1, 0, 0.f));
	tm.setRow(2, make_float4(0, 0, zsize, 0.f));
	tm.setRow(3, make_float4(0, 0, 0,  1));

	//std::vector<float3> v(std::begin(quadh), std::end(quadh));
	//std::vector<int> ind(std::begin(quadind), std::end(quadind));
	//std::cout<<"Writing quad" <<std::endl;
	//sceneManager->writeMeshToPLYFile("quad-dif.ply", v,ind, tm);

	sceneManager->addStaticMesh(4, quadh, 6, quadind, tm, emProp1 );
	
	//Plane YZ as quad with up border at Y=0
	//int quadind2[6] = { 0,1,2,1,3,2 };
	//optix::float3 quadh2[4] = { make_float3(0.0f,0.0f,-0.5f),make_float3(0.0f,0.f,0.5f) ,make_float3(0.0f,-1.0f,-0.5f) ,make_float3(0.0f,-1.0f,0.5f) };

	////Rotate and Scale the previous quad
	//Matrix4x4 sm; 
	//sm.setRow(0, make_float4(0, 0, 0, 0.f));
	//sm.setRow(1, make_float4(0, ysize, 0, 0.f));
	//sm.setRow(2, make_float4(0, 0, zsize, 0.f));
	//sm.setRow(3, make_float4(0, 0, 0,  1));
	Matrix4x4 rm;
	float ar=(360-aperture)*M_PIf/180.0f; //Rotate counterclockwise around Z axis, so since the corner is SW we get the complementary angle here 
	rm.setRow(0, make_float4(cosf(ar), -sinf(ar), 0, 0.f));
	rm.setRow(1, make_float4(sinf(ar), cosf(ar), 0, 0.f));
	rm.setRow(2, make_float4(0, 0, 1, 0.f));
	rm.setRow(3, make_float4(0, 0, 0,  1));
	Matrix4x4 sr=rm*tm;//First rotate and then scale (matrix order is reversed)
	//std::cout<<"rm="<<rm<<"sr="<<sr<<std::endl;
	//std::vector<float3> v2(std::begin(quadh), std::end(quadh));
	//std::vector<int> ind2(std::begin(quadind), std::end(quadind));
	//std::cout<<"Writing quad acute" <<std::endl;
	//sceneManager->writeMeshToPLYFile("quad-difa.ply", v2,ind2, sr); 


	//sceneManager->addStaticMesh(4, quadh2, 6, quadind2, sr, emProp1 );
	sceneManager->addStaticMesh(4, quadh, 6, quadind, sr, emProp1 );
	float3 x=make_float3(1.0f,0.0f,0.0f);//East (x axis)
	float3 y=make_float3(0.0f,1.0f,0.0f); //Up (Y axis)
	float3 z=make_float3(0.0f,0,1.0f);//North (Z axis)

	//Edge is defined along the Z axis. Face a is X axis, 
	float angle=(180.0-aperture)*M_PIf/180.0f;
	float3 face_b=normalize(make_float3(-cosf(angle),-sinf(angle), 0.0f));
	float3 face_b_n=normalize(cross(face_b, z));
	std::cout<< "Adding SW acute face_b="<<face_b<<"n_b="<<face_b_n<<std::endl;
	//Define origin and edge direction this way to follow edge conventions
	sceneManager->addEdge(make_float3(0.0f,0.0f,-zsize*0.5f)+zsize*z,-zsize*z,make_uint2(0u,1u),x,face_b,y,face_b_n, emProp1);
}
void DiffractionTests::loadRaggedCorners(float xsize, float ysize, float zsize, float aperture, MaterialEMProperties emProp1) {
	//Plane XZ as quad with left border at X=0 
	int quadind[6] = { 0,1,2,1,3,2 };
	optix::float3 quadh[4] = { make_float3(0.0f,0.0f,-0.5f),make_float3(0.0f,0.f,0.5f) ,make_float3(1.0f,0.f,-0.5f) ,make_float3(1.0f,0.0f,0.5f) };

	//Scale 
	Matrix4x4 tm;
	tm.setRow(0, make_float4(xsize, 0, 0, 0.f));
	tm.setRow(1, make_float4(0, 1, 0, 0.f));
	tm.setRow(2, make_float4(0, 0, zsize, 0.f));
	tm.setRow(3, make_float4(0, 0, 0,  1));

	//std::vector<float3> v(std::begin(quadh), std::end(quadh));
	//std::vector<int> ind(std::begin(quadind), std::end(quadind));
	//std::cout<<"Writing quad" <<std::endl;
	//sceneManager->writeMeshToPLYFile("quad-dif.ply", v,ind, tm);

	sceneManager->addStaticMesh(4, quadh, 6, quadind, tm, emProp1 );
	
	//Plane YZ as quad with up border at Y=0
	//int quadind2[6] = { 0,1,2,1,3,2 };
	//optix::float3 quadh2[4] = { make_float3(0.0f,0.0f,-0.5f),make_float3(0.0f,0.f,0.5f) ,make_float3(0.0f,-1.0f,-0.5f) ,make_float3(0.0f,-1.0f,0.5f) };

	////Rotate and Scale the previous quad
	//Matrix4x4 sm; 
	//sm.setRow(0, make_float4(0, 0, 0, 0.f));
	//sm.setRow(1, make_float4(0, ysize, 0, 0.f));
	//sm.setRow(2, make_float4(0, 0, zsize, 0.f));
	//sm.setRow(3, make_float4(0, 0, 0,  1));
	Matrix4x4 rm;
	float ar=(360-aperture)*M_PIf/180.0f; //Rotate counterclockwise around Z axis, so since the corner is SW we get the complementary angle here 
	rm.setRow(0, make_float4(cosf(ar), -sinf(ar), 0, 0.f));
	rm.setRow(1, make_float4(sinf(ar), cosf(ar), 0, 0.f));
	rm.setRow(2, make_float4(0, 0, 1, 0.f));
	rm.setRow(3, make_float4(0, 0, 0,  1));
	Matrix4x4 sr=rm*tm;//First rotate and then scale (matrix order is reversed)
	//std::cout<<"rm="<<rm<<"sr="<<sr<<std::endl;
	std::vector<float3> v2(std::begin(quadh), std::end(quadh));
	std::vector<int> ind2(std::begin(quadind), std::end(quadind));
	std::cout<<"Writing quad acute" <<std::endl;
	sceneManager->writeMeshToPLYFile("quad-difa.ply", v2,ind2, sr); 


	//sceneManager->addStaticMesh(4, quadh2, 6, quadind2, sr, emProp1 );
	sceneManager->addStaticMesh(4, quadh, 6, quadind, sr, emProp1 );
	float3 x=make_float3(1.0f,0.0f,0.0f);//East (x axis)
	float3 y=make_float3(0.0f,1.0f,0.0f); //Up (Y axis)
	float3 z=make_float3(0.0f,0,1.0f);//North (Z axis)

	//Edge is defined along the Z axis. Face a is X axis, 
	float angle=(180.0-aperture)*M_PIf/180.0f;
	float3 face_b=normalize(make_float3(-cosf(angle),-sinf(angle), 0.0f));
	float3 face_b_n=normalize(cross(face_b, z));
	std::cout<< "Adding SW acute face_b="<<face_b<<"n_b="<<face_b_n<<std::endl;
	//Define origin and edge direction this way to follow edge conventions
	sceneManager->addEdge(make_float3(0.0f,0.0f,-zsize*0.5f)+zsize*z,-zsize*z,make_uint2(0u,1u),x,face_b,y,face_b_n, emProp1);

//Second edge
//Displace first quad
	Matrix4x4 dm;
	dm.setRow(0, make_float4(1, 0, 0, 0));
	dm.setRow(1, make_float4(0, 1, 0, -xsize*sinf(aperture*M_PIf/180.f)));
	dm.setRow(2, make_float4(0, 0, 1, 0.f));
	dm.setRow(3, make_float4(0, 0, 0,  1));
	sr=tm*dm;
	std::vector<float3> v3(std::begin(quadh), std::end(quadh));
	std::vector<int> ind3(std::begin(quadind), std::end(quadind));
	std::cout<<"Writing quad acute" <<std::endl;
	sceneManager->writeMeshToPLYFile("quad-difa2.ply", v3,ind3, sr); 
	sceneManager->addStaticMesh(4, quadh, 6, quadind, sr, emProp1 );
	sceneManager->addEdge(make_float3(xsize*cosf(aperture*M_PIf/180.0f),-xsize*sinf(aperture*M_PIf/180.f),zsize*0.5f)-zsize*z,zsize*z,make_uint2(1u,2u),-face_b,-x,-face_b_n,-y, emProp1);
}

void DiffractionTests::loadCrossing(MaterialEMProperties emProp1) {
	//Cubes
	std::vector<int> cubeind = sceneManager->loadTrianglesFromFile("meshes/tricube.txt");
	std::vector<float3> cubevert = sceneManager->loadVerticesFromFile("meshes/vertcube.txt");
	std::vector<std::pair<optix::int3, unsigned int>> triangleIndexFaceBuffer;
	uint cubeFaces=6u;
	uint lastIndex=0u;
	//TODO: load from file
	//Handcrafted face Id and triangles
	triangleIndexFaceBuffer.push_back(std::pair<optix::int3, unsigned int>(make_int3(0,2,3), 0u));
	triangleIndexFaceBuffer.push_back(std::pair<optix::int3, unsigned int>(make_int3(0,3,1), 0u));
	triangleIndexFaceBuffer.push_back(std::pair<optix::int3, unsigned int>(make_int3(8,4,5), 1u));
	triangleIndexFaceBuffer.push_back(std::pair<optix::int3, unsigned int>(make_int3(8,5,9), 1u));
	triangleIndexFaceBuffer.push_back(std::pair<optix::int3, unsigned int>(make_int3(10,6,7), 2u));
	triangleIndexFaceBuffer.push_back(std::pair<optix::int3, unsigned int>(make_int3(10,7,11), 2u));
	triangleIndexFaceBuffer.push_back(std::pair<optix::int3, unsigned int>(make_int3(12,13,14), 3u));
	triangleIndexFaceBuffer.push_back(std::pair<optix::int3, unsigned int>(make_int3(12,14,15), 3u));
	triangleIndexFaceBuffer.push_back(std::pair<optix::int3, unsigned int>(make_int3(16,17,18), 4u));
	triangleIndexFaceBuffer.push_back(std::pair<optix::int3, unsigned int>(make_int3(16,18,19), 4u));
	triangleIndexFaceBuffer.push_back(std::pair<optix::int3, unsigned int>(make_int3(20,21,22), 5u));
	triangleIndexFaceBuffer.push_back(std::pair<optix::int3, unsigned int>(make_int3(20,22,23), 5u));


	//std::cout << "indices=" << cubeind.size() << "vertices=" << cubevert.size() << std::endl;
	//Cube(4) NW
	Matrix4x4 tm;
	tm.setRow(0, make_float4(40.0f, 0, 0, -30.0f));
	tm.setRow(1, make_float4(0, 40.0f, 0, 20.0f));
	tm.setRow(2, make_float4(0, 0, 40.0f, 80.0f));
	tm.setRow(3, make_float4(0, 0, 0, 1));
	std::cout << "Adding NW. Em = "<< emProp1.dielectricConstant << std::endl;
	sceneManager->addStaticMeshWithFaces(cubevert,triangleIndexFaceBuffer,  tm, emProp1);
	//sceneManager->addStaticMesh(cubevert,cubeind, tm, emProp1);
	addCubeEdges(make_float3(-50.0f,0.0f,60.0f),40.0f,lastIndex, emProp1);
	lastIndex =6;
	//Cube SW
	tm.setRow(0, make_float4(40.0f, 0, 0, -30.0f));
	tm.setRow(1, make_float4(0, 40.0f, 0, 20.0f));
	tm.setRow(2, make_float4(0, 0, 40.0f, 20.0f));
	tm.setRow(3, make_float4(0, 0, 0, 1));
	std::cout << "Adding SW. Em = "<< emProp1.dielectricConstant << std::endl;
	//Update the face ids
	int j=0;
	for (int i=0; i<12;++i) {
		triangleIndexFaceBuffer[i].second=lastIndex;
		++j;
		//triangleIndexFaceBuffer[i+1].second=lastIndex;
		if (j==2) {
			++lastIndex;
			j=0;
		}
		//std::cout<<"tri="<<triangleIndexFaceBuffer[i].first<<"="<<triangleIndexFaceBuffer[i].second<<std::endl;
		//std::cout<<"tri="<<triangleIndexFaceBuffer[i+1].first<<"="<<triangleIndexFaceBuffer[i+1].second<<std::endl;
	}
	sceneManager->addStaticMeshWithFaces(cubevert,triangleIndexFaceBuffer, tm, emProp1);
	//sceneManager->addStaticMesh(cubevert,cubeind, tm, emProp1);
	addCubeEdges(make_float3(-50.0f,0.0f,0.0f),40.0f, 6,emProp1);
	//lastIndex += cubeFaces;
	lastIndex = 18;
	//Cube(2) NE
	
	tm.setRow(0, make_float4(40.0f, 0, 0, 30.0f));
	tm.setRow(1, make_float4(0, 40.0f, 0, 20.0f));
	tm.setRow(2, make_float4(0, 0, 40.0f, 80.0f));
	tm.setRow(3, make_float4(0, 0, 0, 1));
	std::cout << "Adding NE. Em = "<< emProp1.dielectricConstant << std::endl;
	//Update the face ids
	j=0;
	for (int i=0; i<12;++i) {
		triangleIndexFaceBuffer[i].second=lastIndex;
		++j;
		//triangleIndexFaceBuffer[i+1].second=lastIndex;
		if (j==2) {
			++lastIndex;
			j=0;
		}
		std::cout<<"tri="<<triangleIndexFaceBuffer[i].first<<"="<<triangleIndexFaceBuffer[i].second<<std::endl;
		//std::cout<<"tri="<<triangleIndexFaceBuffer[i+1].first<<"="<<triangleIndexFaceBuffer[i+1].second<<std::endl;
	}
	sceneManager->addStaticMeshWithFaces(cubevert,triangleIndexFaceBuffer,  tm, emProp1);
	//sceneManager->addStaticMesh(cubevert,cubeind, tm, emProp1);

	addCubeEdges(make_float3(10.0f,0.0f,60.0f),40.0f, 18, emProp1);
	//lastIndex += cubeFaces;
	lastIndex = 12;
	//Cube(1) SE

	tm.setRow(0, make_float4(40.0f, 0, 0, 30.0f));
	tm.setRow(1, make_float4(0, 40.0f, 0, 20.0f));
	tm.setRow(2, make_float4(0, 0, 40.0f, 20.0f));
	tm.setRow(3, make_float4(0, 0, 0, 1));
	std::cout << "Adding SE. Em = "<< emProp1.dielectricConstant << std::endl;
	//Update the face ids
	j=0;
	for (int i=0; i<12;++i) {
		triangleIndexFaceBuffer[i].second=lastIndex;
		++j;
		//triangleIndexFaceBuffer[i+1].second=lastIndex;
		if (j==2) {
			++lastIndex;
			j=0;
		}
		//std::cout<<"tri="<<triangleIndexFaceBuffer[i].first<<"="<<triangleIndexFaceBuffer[i].second<<std::endl;
		//std::cout<<"tri="<<triangleIndexFaceBuffer[i+1].first<<"="<<triangleIndexFaceBuffer[i+1].second<<std::endl;
	}
	//for (int i=0; i<cubeFaces;++i) {
	//	triangleIndexFaceBuffer[i].second=lastIndex+i;
	//	triangleIndexFaceBuffer[i+1].second=lastIndex+i;
	//}
	sceneManager->addStaticMeshWithFaces(cubevert,triangleIndexFaceBuffer, tm, emProp1);
	//sceneManager->addStaticMesh(cubevert,cubeind, tm, emProp1);

	addCubeEdges(make_float3(10.0f,0.0f,0.0f),40.0f,12, emProp1);
	lastIndex += cubeFaces;

	
	//Finally, add horizontal plane
	//Horizontal plane as quad at origin 
	int quadind[6] = { 0,1,2,1,0,3 };
	optix::float3 quadh[4] = { make_float3(-0.5f,0.0f,-0.5f),make_float3(0.5f,0.f,0.5f) ,make_float3(0.5f,0.f,-0.5f) ,make_float3(-0.5f,0.0f,0.5f) };

	//Scale 200x200
	tm.setRow(0, make_float4(200, 0, 0, 0.f));
	tm.setRow(1, make_float4(0, 1, 0, 0.f));
	tm.setRow(2, make_float4(0, 0, 200, 0.f));
	tm.setRow(3, make_float4(0, 0, 0,  1));
	
	sceneManager->addStaticMesh(4, quadh, 6, quadind, tm, emProp1 );

}
//Pass south-west point of cube and side length
void DiffractionTests::addCubeEdges(float3 sw,float length, uint index, MaterialEMProperties emProp1) {
	//Only 8 edges, the edges on the floor are not considered
	//Vertical edges (0,1,0) with  length
	float3 x=make_float3(length,0.0f,0.0f);//East (x axis)
	float3 up=make_float3(0.0f,length,0.0f); //Up (Y axis)
	float3 z=make_float3(0.0f,0,length);//North (z axis)

	float3 xu={1.0f,0.0f,0.0f};
	float3 yu={0.0f,1.0f,0.0f};
	float3 zu={0.0f,0.0f,1.0f};
		

	//Points
	float3 nw=sw+z;
	float3 ne=nw+x;
	float3 se=ne-z;

	//MaterialEMProperties emProp1= sceneManager->ITUparametersToMaterial(3.75f,0.0f,0.038f,0.0f);
	//MaterialEMProperties emProp1;
	//Set perfect conductor
	//emProp1.dielectricConstant=make_float2(0.0f,std::numeric_limits<float>::infinity());
	//MaterialEMProperties emProp1;
	//emProp1.dielectricConstant = make_float2(3.75f, -60.0f*sceneManager->getChannelParameters().waveLength*0.038f);
		//There is a dependency on the frequency again, we use -15 dB per 203 mm at 5 GHz => -75 dB/m
	emProp1.tattenuation = make_float2(0.1f,-75.f );
	
	//Add verticals 
	sceneManager->addEdge(sw,up,make_uint2(index+4u,index+2u),z,x,-xu,-zu, emProp1);
	//sceneManager->addEdge(nw,up,make_uint2(index+4u,index),-z,x,-xu,zu,emProp1);
	sceneManager->addEdge(nw+up,-up,make_uint2(index+4u,index),-z,x,-xu,zu,emProp1); //To follow conventions
	//sceneManager->addEdge(ne,up,make_uint2(index,index+5),-x,-z, zu,xu, emProp1);
	sceneManager->addEdge(ne+up,-up,make_uint2(index,index+5),-x,-z, zu,xu, emProp1);
	//sceneManager->addEdge(se,up,make_uint2(index+5,index+2),z,-x,xu,-zu, emProp1);
	sceneManager->addEdge(se+up,-up,make_uint2(index+5,index+2),z,-x,xu,-zu, emProp1);

	//Add horizontals at elevation
	sceneManager->addEdge(sw+up,z,make_uint2(index+4,index+1),-up,x,-xu,yu, emProp1);
	sceneManager->addEdge(nw+up,x,make_uint2(index,index+1),-up,-z,zu,yu, emProp1);
	sceneManager->addEdge(ne+up,-z,make_uint2(index+5,index+1),-up,-x,xu,yu,emProp1);
	sceneManager->addEdge(se+up,-x,make_uint2(index+2,index+1),-up,z,-zu,yu, emProp1);

}
void DiffractionTests::semiplaneDiffraction() {
	Timer timer;
	float frequency=900e6;
	SingleDiffraction* sim= new SingleDiffraction(sceneManager);
	sim->setComputeMode(ComputeMode::VOLTAGE);
	sceneManager->setSimulation(sim);
	sceneManager->initContext(frequency);
	std::cout << "Simulating edge diffraction" << std::endl;
	//Half plane on the XZ plane
	float3 x=make_float3(1.0f,0.0f,0.0f);//East (x axis)
	float3 y=make_float3(0.0f,1.0f,0.0f); //Up (Y axis)
	float3 z=make_float3(0.0f,0,1.0f);//North (Z axis)

	//MaterialEMProperties emProp1= sceneManager->ITUparametersToMaterial(3.75f,0.0f,0.038f,0.0f);
	MaterialEMProperties emProp1;
	//Set perfect conductor
	emProp1.dielectricConstant=make_float2(0.0f,std::numeric_limits<float>::infinity());
	//MaterialEMProperties emProp1;
	//emProp1.dielectricConstant = make_float2(3.75f, -60.0f*sceneManager->getChannelParameters().waveLength*0.038f);
		//There is a dependency on the frequency again, we use -15 dB per 203 mm at 5 GHz => -75 dB/m
	emProp1.tattenuation = make_float2(0.1f,-75.f );

	//Perfect conductor half plane. Plane is on the XZ plane (normals on Y and -Y)	
	sceneManager->addEdge(make_float3(0.0f,0.0f,0.0f),100.f*z,make_uint2(0u,1u),x,x,y,-y, emProp1);
	
	sceneManager->setMinEpsilon(1e-4f);

	//receivers
	//optix::float3 polarization = make_float3(0.0f, 1.0f, 0.0f); //Hard
	optix::float3 polarization = make_float3(0.0f, 0.0f, 1.0f); //Soft
	//optix::float3 posrx = make_float3(0.0f, 10.0f, 95.0f);
	float dist=sceneManager->getChannelParameters().waveLength;
	for (int i=0;i<360; ++i) {
		float phi=i*M_PIf/180.0f;
		//float phi=210*M_PIf/180.0f;
		
		//optix::float3 posrx = make_float3(0.0f,dist, 10.0f);
		optix::float3 posrx = make_float3(cosf(phi)*dist,sinf(phi)*dist, 10.0f);
		sceneManager->addReceiver(i, posrx,polarization, sphereRadius, sceneManager->printPower);
	}

	//sceneManager->setMaxReflections(3u);
	//sceneManager->enableExceptions();
	sceneManager->finishSceneContext();

	//sceneManager->setPrintEnabled(1024 * 1024 * 1024);	
	//sceneManager->setUsageReport();

	float phi_prime=30;
	float disX=20.0f;
	optix::float3 postx;
	timer.start();
	postx = make_float3(disX, tanf(phi_prime*M_PIf/180.0f)*disX, 10.0f);
	std::cout<<"Transmitting from "<<postx<<" with polarization "<<polarization<<std::endl;
	sceneManager->transmit(361, 1.0f, postx, polarization);
	std::cout<<"Time="<<timer.getTime()<<std::endl;


}
void DiffractionTests::semiplaneTotal() {
	Timer timer;
	float frequency=900e6;

	//Add reflection simulation
	LPFlatMeshReflectionSimulation* sim= new LPFlatMeshReflectionSimulation(sceneManager);
	//RayDensityNormalizationSimulation* sim= new RayDensityNormalizationSimulation(sceneManager);
	ComputeMode mode=ComputeMode::VOLTAGE;
	//ComputeMode mode=ComputeMode::FIELD;
	sim->setComputeMode(mode);
	//Disable or enable if necessary to check the different contributions
	//sim->setEnableSimulation(false);
	sceneManager->enableGenerateRaysOnLaunch();
	sceneManager->setSimulation(sim);

	//Add diffraction simulation
	SingleDiffraction* simd= new SingleDiffraction(sceneManager);
	sceneManager->setSimulation(simd);
	simd->setComputeMode(mode);
	//Disable or enable if necessary to check the different contributions
	simd->setEnableSimulation(false);

	sceneManager->initContext(frequency);
	std::cout << "Simulating total semiplane" << std::endl;
	//Half plane on the XZ plane
	float3 x=make_float3(1.0f,0.0f,0.0f);//East (x axis)
	float3 y=make_float3(0.0f,1.0f,0.0f); //Up (Y axis)
	float3 z=make_float3(0.0f,0,1.0f);//North (Z axis)

	//MaterialEMProperties emProp1= sceneManager->ITUparametersToMaterial(3.75f,0.0f,0.038f,0.0f);
	MaterialEMProperties emProp1;
	//Set perfect conductor
	emProp1.dielectricConstant=make_float2(0.0f,std::numeric_limits<float>::infinity());
	//MaterialEMProperties emProp1;
	//emProp1.dielectricConstant = make_float2(3.75f, -60.0f*sceneManager->getChannelParameters().waveLength*0.038f);
		//There is a dependency on the frequency again, we use -15 dB per 203 mm at 5 GHz => -75 dB/m
	emProp1.tattenuation = make_float2(0.1f,-75.f );

	//Perfect conductor half plane. Plane is on the XZ plane (normals on Y and -Y)	
	sceneManager->addEdge(make_float3(0.0f,0.0f,0.0f),100.f*z,make_uint2(0u,1u),x,x,y,-y, emProp1);
	
	sceneManager->setMinEpsilon(1e-4f);

	//Plane XZ as quad with left border at X=0 
	int quadind[6] = { 0,1,2,1,3,2 };
	optix::float3 quadh[4] = { make_float3(0.0f,0.0f,-0.5f),make_float3(0.0f,0.f,0.5f) ,make_float3(1.0f,0.f,-0.5f) ,make_float3(1.0f,0.0f,0.5f) };

	//Scale 100x100
	Matrix4x4 tm;
	tm.setRow(0, make_float4(100, 0, 0, 0.f));
	tm.setRow(1, make_float4(0, 1, 0, 0.f));
	tm.setRow(2, make_float4(0, 0, 100, 0.f));
	tm.setRow(3, make_float4(0, 0, 0,  1));


	sceneManager->addStaticMesh(4, quadh, 6, quadind, tm, emProp1 );


	//receivers
	optix::float3 polarization = make_float3(0.0f, 1.0f, 0.0f); //Hard
	//optix::float3 polarization = make_float3(0.0f, 0.0f, 1.0f); //Soft
	//optix::float3 posrx = make_float3(0.0f, 10.0f, 95.0f);
	float dist=sceneManager->getChannelParameters().waveLength;
	for (int i=0;i<360; ++i) {
		float phi=i*M_PIf/180.0f;
		//float phi=60*M_PIf/180.0f;
		
		//optix::float3 posrx = make_float3(0.0f,dist, 10.0f);
		optix::float3 posrx = make_float3(cosf(phi)*dist,sinf(phi)*dist, 10.0f);
		sceneManager->addReceiver(i, posrx,polarization, sphereRadius, sceneManager->printPower);
	}

	//sceneManager->setMaxReflections(3u);
	//sceneManager->enableExceptions();
	sceneManager->finishSceneContext();

//Uncomment for flat
	sceneManager->createRaySphere2D(0.0f,0.1,180.0f,0.0f,0.1,360.0f);
	//Uncomment for RDN	
//		unsigned int rayD=10000;
//
//		sceneManager->setRayRange(0.0,180.0,0.0,360.0,rayD,rayD);
//		sim->setInitialDensity(((float)sceneManager->getRaySphere().rayCount)/(4*M_PIf));
//		sim->setFiltering(2);//Not divided by m	
	//sceneManager->setPrintEnabled(1024 * 1024 * 1024);	
	//sceneManager->setUsageReport();

	float phi_prime=30*M_PIf/180.0f;
	float disX=sceneManager->getChannelParameters().waveLength*10;
	//float disX=20.0f;
	optix::float3 postx;
	timer.start();
	//postx = make_float3(disX, tanf(phi_prime)*disX, 10.0f);
	postx = make_float3(cosf(phi_prime)*disX, sinf(phi_prime)*disX, 10.0f);
	std::cout<<"Transmitting from "<<postx<<" with polarization "<<polarization<<std::endl;
	sceneManager->transmit(361, 1.0f, postx, polarization);
	std::cout<<"Time="<<timer.getTime()<<std::endl;


}
void DiffractionTests::runSWCornerDynamicMesh() {
	Timer timer;
	float frequency=1.8e9;

	//Add reflection simulation
	LPFlatMeshReflectionSimulation* sim= new LPFlatMeshReflectionSimulation(sceneManager);
	//RayDensityNormalizationSimulation* sim= new RayDensityNormalizationSimulation(sceneManager);
	ComputeMode mode=ComputeMode::VOLTAGE;
	//ComputeMode mode=ComputeMode::FIELD;
	sim->setComputeMode(mode);
	//sim->setComputeMode(ComputeMode::VOLTAGE);
	//Disable or enable if necessary to check the different contributions
	//sim->setEnableSimulation(false);
	sceneManager->enableGenerateRaysOnLaunch();
	sceneManager->setSimulation(sim);

	//Add diffraction simulation
	SingleDiffraction* simd= new SingleDiffraction(sceneManager);
	sceneManager->setSimulation(simd);
	simd->setComputeMode(mode);
	//Disable or enable if necessary to check the different contributions
	//simd->setEnableSimulation(false);

	sceneManager->initContext(frequency);
	std::cout << "Simulating SW corner" << std::endl;

	//MaterialEMProperties emProp1= sceneManager->ITUparametersToMaterial(3.75f,0.0f,0.038f,0.0f);
	MaterialEMProperties emProp1;
	//Set perfect conductor
	emProp1.dielectricConstant=make_float2(0.0f,std::numeric_limits<float>::infinity());
	//MaterialEMProperties emProp1;
	//emProp1.dielectricConstant = make_float2(3.75f, -60.0f*sceneManager->getChannelParameters().waveLength*0.038f);
		//There is a dependency on the frequency again, we use -15 dB per 203 mm at 5 GHz => -75 dB/m
	emProp1.tattenuation = make_float2(0.1f,-75.f );
	
	//loadSWCorner(100,100,100,emProp1);	
	loadSWCornerAsGroup(100,100,100,emProp1);	

	//translate, rotate, some tests...
	//Matrix4x4 tl=Matrix4x4::translate(make_float3(0.0f, 0.0f, 50.0f));
	//Matrix4x4 tr=Matrix4x4::rotate(M_PIf/2,make_float3(0.0f, 1.0f, 0.0f));
	//Remember it is in reverse order: first, operations on the right: rotate then translate
	//Matrix4x4 t=tl*tr;
	//translate and then rotate
	//Matrix4x4 t=tr*tl;
	Matrix4x4 tr=Matrix4x4::rotate(-M_PIf/4,make_float3(0.0f, 0.0f, 1.0f));
	sceneManager->updateTransformInGroup(0,tr);

	
	//Matrix4x4 tlb=Matrix4x4::translate(make_float3(0.0f, 0.0f, -50.0f));
	//sceneManager->updateTransformInGroup(0,tlb);
	//receivers
	//optix::float3 polarization = make_float3(0.0f, 1.0f, 0.0f); //Hard
	optix::float3 polarization = make_float3(0.0f, 0.0f, 1.0f); //Soft
	//optix::float3 posrx = make_float3(0.0f, 10.0f, 95.0f);
	float dist=20.0f;
	for (int i=0;i<1; ++i) {
		//float phi=i*M_PIf/180.0f;
		float phi=60*M_PIf/180.0f;
		
		//optix::float3 posrx = make_float3(0.0f,dist, 10.0f);
		optix::float3 posrx = make_float3(cosf(phi)*dist,sinf(phi)*dist, 10.0f);
		sceneManager->addReceiver(i, posrx,polarization, sphereRadius, sceneManager->printPower);
	}

	//sceneManager->setMaxReflections(3u);
	//sceneManager->enableExceptions();
	sceneManager->finishSceneContext();

//Uncomment for flat

	sceneManager->createRaySphere2D(0.0f,0.1,180.0f,0.0f,0.1,360.0f);
	//Uncomment for RDN	
	//	unsigned int rayD=10000;

	//	sceneManager->setRayRange(0.0,180.0,0.0,360.0,rayD,rayD);
	//	sim->setInitialDensity(((float)sceneManager->getRaySphere().rayCount)/(4*M_PIf));
	//	sim->setFiltering(2);//Not divided by m	
	
	//sceneManager->setPrintEnabled(1024 * 1024 * 1024);	
	//sceneManager->setUsageReport();

	float phi_prime=45*M_PIf/180.0f;
	float disX=dist*sqrt(2);
	//float disX=20.0f;
	optix::float3 postx;
	timer.start();
	//postx = make_float3(disX, tanf(phi_prime)*disX, 10.0f);
	postx = make_float3(cosf(phi_prime)*disX, sinf(phi_prime)*disX, 10.0f);
	std::cout<<"Transmitting from "<<postx<<" with polarization "<<polarization<<std::endl;
	sceneManager->transmit(361, 1.0f, postx, polarization);
	std::cout<<"Time="<<timer.getTime()<<std::endl;


}
void DiffractionTests::runSWCorner(bool multitransmitter) {
	Timer timer;
	float frequency=1.8e9;

	//Add reflection simulation
	LPFlatMeshReflectionSimulation* sim= new LPFlatMeshReflectionSimulation(sceneManager);
	//RayDensityNormalizationSimulation* sim= new RayDensityNormalizationSimulation(sceneManager);
	ComputeMode mode=ComputeMode::VOLTAGE;
	//ComputeMode mode=ComputeMode::FIELD;
	sim->setComputeMode(mode);
	//sim->setComputeMode(ComputeMode::VOLTAGE);
	//Disable or enable if necessary to check the different contributions
	//sim->setEnableSimulation(false);
	sceneManager->enableGenerateRaysOnLaunch();
	sceneManager->setSimulation(sim);

	//Add diffraction simulation
	SingleDiffraction* simd= new SingleDiffraction(sceneManager);
	sceneManager->setSimulation(simd);
	simd->setComputeMode(mode);
	//Disable or enable if necessary to check the different contributions
	//simd->setEnableSimulation(false);

	sceneManager->initContext(frequency);
	if (multitransmitter) {
		sceneManager->enableMultitransmitter();
		std::cout << "Simulating SW corner with multitransmitter" << std::endl;
	} else {
		std::cout << "Simulating SW corner" << std::endl;
	}
	//MaterialEMProperties emProp1= sceneManager->ITUparametersToMaterial(3.75f,0.0f,0.038f,0.0f);
	MaterialEMProperties emProp1;
	//Set perfect conductor
	emProp1.dielectricConstant=make_float2(0.0f,std::numeric_limits<float>::infinity());
	//MaterialEMProperties emProp1;
	//emProp1.dielectricConstant = make_float2(3.75f, -60.0f*sceneManager->getChannelParameters().waveLength*0.038f);
		//There is a dependency on the frequency again, we use -15 dB per 203 mm at 5 GHz => -75 dB/m
	emProp1.tattenuation = make_float2(0.1f,-75.f );
	
	loadSWCorner(100,100,100,emProp1);	

	
	//receivers
	optix::float3 polarization = make_float3(0.0f, 1.0f, 0.0f); //Hard
	//optix::float3 polarization = make_float3(0.0f, 0.0f, 1.0f); //Soft
	//optix::float3 posrx = make_float3(0.0f, 10.0f, 95.0f);
	optix::float3 posrx;
	float dist=20.0f;
	for (int i=0;i<360; ++i) {
		float phi=i*M_PIf/180.0f;
		//float phi=60*M_PIf/180.0f;
		
		//optix::float3 posrx = make_float3(0.0f,dist, 10.0f);
		//posrx = make_float3(cosf(phi)*dist,sinf(phi)*dist, -60.0f);
		posrx = make_float3(cosf(phi)*dist,sinf(phi)*dist, 10.0f);
		sceneManager->addReceiver(i, posrx,polarization, sphereRadius, sceneManager->printPower);
	}

	sceneManager->finishSceneContext();

//Uncomment for flat

	sceneManager->createRaySphere2D(0.0f,0.1,180.0f,0.0f,0.1,360.0f);
	//Uncomment for RDN	
	//	unsigned int rayD=10000;

	//	sceneManager->setRayRange(0.0,180.0,0.0,360.0,rayD,rayD);
	//	sim->setInitialDensity(((float)sceneManager->getRaySphere().rayCount)/(4*M_PIf));
	//	sim->setFiltering(2);//Not divided by m	
	
	//sceneManager->setPrintEnabled(1024 * 1024 * 1024);	
	//sceneManager->setUsageReport();

	float phi_prime=45*M_PIf/180.0f;
	float disX=dist*sqrt(2);
	//float disX=20.0f;
	optix::float3 postx;
	timer.start();
	//postx = make_float3(disX, tanf(phi_prime)*disX, 10.0f);
	//postx = make_float3(0.5f, 20, -60.0f);
	//postx = make_float3(0.5f, 20, 10.0f);
	postx = make_float3(cosf(phi_prime)*disX, sinf(phi_prime)*disX, 10.0f);
	if (multitransmitter)  {
		TransmitterManager* transmitterManager=sceneManager->getTransmitterManager();
		transmitterManager->registerTransmitter(361,postx,polarization,1.0f);
		transmitterManager->addTransmitterToGroup(361,1.0f,postx,polarization);
		transmitterManager->registerTransmitter(362,postx,polarization,1.0f);
		transmitterManager->addTransmitterToGroup(362,1.0f,postx,polarization);
		std::cout<<"Transmitting with 2 transmitters from "<<postx<<" with polarization "<<polarization<<std::endl;
		sceneManager->groupTransmit();
	
	} else {
	
		std::cout<<"Transmitting from "<<postx<<" with polarization "<<polarization<<std::endl;
		sceneManager->transmit(361, 1.0f, postx, polarization);
	}
//Interchange test
	//sceneManager->updateReceiver(0, postx);
	//postx=posrx;
	//std::cout<<"Transmitting from "<<postx<<" with polarization "<<polarization<<std::endl;
	//sceneManager->transmit(361, 1.0f, postx, polarization);
	std::cout<<"Time="<<timer.getTime()<<std::endl;


}

void DiffractionTests::runSWAcuteCorner() {
	//An edge with 150 degrees between faces
	Timer timer;
	float frequency=2.5e9;

	//Add reflection simulation
	LPFlatMeshReflectionSimulation* sim= new LPFlatMeshReflectionSimulation(sceneManager);
	//RayDensityNormalizationSimulation* sim= new RayDensityNormalizationSimulation(sceneManager);
	//ComputeMode mode=ComputeMode::VOLTAGE;
	ComputeMode mode=ComputeMode::FIELD;
	sim->setComputeMode(mode);
	//sim->setComputeMode(ComputeMode::VOLTAGE);
	//Disable or enable if necessary to check the different contributions
	sim->setEnableSimulation(false);
	sceneManager->enableGenerateRaysOnLaunch();
	sceneManager->setSimulation(sim);

	//Add diffraction simulation
	SingleDiffraction* simd= new SingleDiffraction(sceneManager);
	sceneManager->setSimulation(simd);
	simd->setComputeMode(mode);
	//Disable or enable if necessary to check the different contributions
	//simd->setEnableSimulation(false);

	sceneManager->initContext(frequency);
	std::cout << "Simulating SW corner" << std::endl;

	//MaterialEMProperties emProp1= sceneManager->ITUparametersToMaterial(3.75f,0.0f,0.038f,0.0f);
	MaterialEMProperties emProp1;
	//MaterialEMProperties emProp1;
	emProp1.dielectricConstant = make_float2(8.0f, -60.0f*sceneManager->getChannelParameters().waveLength*0.001f);
		//There is a dependency on the frequency again, we use -15 dB per 203 mm at 5 GHz => -75 dB/m
	emProp1.tattenuation = make_float2(0.1f,-75.f );
	
	//loadSWAcuteCorner(100,100,100, 50,emProp1);	
	loadRaggedCorners(100,100,100, 50,emProp1);	


	//receivers
	//optix::float3 polarization = make_float3(0.0f, 1.0f, 0.0f); //Hard
	optix::float3 polarization = make_float3(0.0f, 0.0f, 1.0f); //Soft
	float dist=2.0f;
	optix::float3 posrx = make_float3(dist*cosf(135*M_PIf/180.f), dist*sinf(135*M_PIf/180.0f), 10.0f);
	sceneManager->addReceiver(0, posrx,polarization, sphereRadius, sceneManager->printPower);
	//for (int i=0;i<360; i=i+4) {
	//	float phi=i*M_PIf/180.0f;
	//	//float phi=256*M_PIf/180.0f;
	//	
	//	//optix::float3 posrx = make_float3(0.0f,dist, 10.0f);
	//	optix::float3 posrx = make_float3(cosf(phi)*dist,sinf(phi)*dist, 10.0f);
	//	sceneManager->addReceiver(i, posrx,polarization, sphereRadius, sceneManager->printPower);
	//}

	//sceneManager->setMaxReflections(3u);
	//sceneManager->enableExceptions();
	sceneManager->finishSceneContext();

//Uncomment for flat

	sceneManager->createRaySphere2D(0.0f,0.1,180.0f,0.0f,0.1,360.0f);
	//Uncomment for RDN	
	//	unsigned int rayD=10000;

	//	sceneManager->setRayRange(0.0,180.0,0.0,360.0,rayD,rayD);
	//	sim->setInitialDensity(((float)sceneManager->getRaySphere().rayCount)/(4*M_PIf));
	//	sim->setFiltering(2);//Not divided by m	
	
	//sceneManager->setPrintEnabled(1024 * 1024 * 1024);	
	//sceneManager->setUsageReport();

	float phi_prime=5*M_PIf/180.0f;
	float disX=3.0f;
	//float disX=20.0f;
	optix::float3 postx;
	timer.start();
	//postx = make_float3(disX, tanf(phi_prime)*disX, 10.0f);
	postx = make_float3(cosf(phi_prime)*disX, sinf(phi_prime)*disX, 10.0f);
	std::cout<<"Transmitting from "<<postx<<" with polarization "<<polarization<<std::endl;
	sceneManager->transmit(361, 1.0f, postx, polarization);
	std::cout<<"Time="<<timer.getTime()<<std::endl;


}
void DiffractionTests::runCrossing() {
	//Four cubes, like a cross road. Tx is fixed
	Timer timer;
	float frequency=5.9e9;

	//Add reflection simulation
	LPFlatMeshReflectionSimulation* sim= new LPFlatMeshReflectionSimulation(sceneManager);
	//BasicFlatMeshReflectionSimulation* sim= new BasicFlatMeshReflectionSimulation(sceneManager);
	//RayDensityNormalizationSimulation* sim= new RayDensityNormalizationSimulation(sceneManager);
	ComputeMode mode=ComputeMode::VOLTAGE;
	//ComputeMode mode=ComputeMode::FIELD;
	sim->setComputeMode(mode);
	
//Use antenna gains
	//sceneManager->setUseAntennaGain(true);

	//sim->setComputeMode(ComputeMode::VOLTAGE);
	//Disable or enable if necessary to check the different contributions
	//sim->setEnableSimulation(false);
	//sim->setEnableTraceLog(true);
	sceneManager->enableGenerateRaysOnLaunch();
	sceneManager->setSimulation(sim);

	//Add diffraction simulation
	SingleDiffraction* simd= new SingleDiffraction(sceneManager);
	sceneManager->setSimulation(simd);
	simd->setComputeMode(mode);
	//simd->setEnableTraceLog(true);
	//Disable or enable if necessary to check the different contributions
	//simd->setEnableSimulation(false);

	sceneManager->initContext(frequency);
	std::cout << "Simulating Four corners" << std::endl;

	//MaterialEMProperties emProp1= sceneManager->ITUparametersToMaterial(3.75f,0.0f,0.038f,0.0f);
	MaterialEMProperties emProp1;
	//MaterialEMProperties emProp1;
	emProp1.dielectricConstant = make_float2(3.75f, -60.0f*sceneManager->getChannelParameters().waveLength*0.038f);
		//There is a dependency on the frequency again, we use -15 dB per 203 mm at 5 GHz => -75 dB/m
	emProp1.tattenuation = make_float2(0.1f,-75.f );
	
	loadCrossing(emProp1);

	//receivers
	optix::float3 polarization = make_float3(0.0f, 1.0f, 0.0f); //Hard
	//optix::float3 polarization = make_float3(0.0f, 0.0f, 1.0f); //Soft

//Fixed receiver, transmitter moving, multiple launches
	optix::float3 posrx = make_float3(0.0f, 10.0f, 100.0f);
	sceneManager->addReceiver(0, posrx,polarization, sphereRadius, sceneManager->printPower);
//Use antenna gains
//	AntennaGain gains=sceneManager->loadGainsFromFileIndBPower("gain.txt");
//	int gainId=sceneManager->registerAntennaGain(gains);
//	sceneManager->registerReceiverGain(0,gainId);

//All receivers, transmitter fixed, one launch
	//for (int i=-50;i<=50; ++i) {
	//	
	//	optix::float3 posrx = make_float3(i, 10.0, 50.0f);
	//	sceneManager->addReceiver(i, posrx,polarization, sphereRadius, sceneManager->printPower);
	//	//optix::float3 posrx = make_float3(38, 10.0f, 50.0f);
	//}

	sceneManager->finishSceneContext();

//Uncomment for flat

	sceneManager->createRaySphere2D(0.0f,0.1,180.0f,0.0f,0.1,360.0f);
//Uncomment for RDN	
	//	unsigned int rayD=10000;

	//	sceneManager->setRayRange(0.0,180.0,0.0,360.0,rayD,rayD);
	//	sim->setInitialDensity(((float)sceneManager->getRaySphere().rayCount)/(4*M_PIf));
	//	sim->setFiltering(2);//Not divided by m	
	

	optix::float3 postx;
	timer.start();
//Antenna gain
	//sceneManager->registerTransmitterGain(361,gainId);
//All receivers, transmitter fixed, one launch
	//postx = make_float3(0.0f, 10.0f, 100.0f);
	//postx = make_float3(-38.0f, 10.0f, 50.0f);


	//std::cout<<"Transmitting from "<<postx<<" with polarization "<<polarization<<std::endl;
	//sceneManager->transmit(361, 1.0f, postx, polarization);
	
////Fixed receiver, transmitter moving, multiple launches
	for (int i=-50;i<=50; ++i) {
		postx = make_float3(i, 10.0f, 50.0f);
		std::cout<<"Transmitting from "<<postx<<" with polarization "<<polarization<<std::endl;
		sceneManager->transmit(361, 1.0f, postx, polarization);
	}
	std::cout<<"Time="<<timer.getTime()<<std::endl;
}
//Adding compund dynamic meshes
void  DiffractionTests::addCompoundDynamicMeshes() {
	Timer timer;
	float freq = 5.9e9f;
	std::cout<<"Running free space test"<<std::endl;
	timer.start();	
	//Init context before doing anything else
	if (useDepolarization) {
		LPFlatMeshReflectionSimulation* sim = new LPFlatMeshReflectionSimulation(sceneManager);
		sceneManager->setSimulation(sim);
	} else {
		BasicFlatMeshReflectionSimulation* sim = new BasicFlatMeshReflectionSimulation(sceneManager);
		sceneManager->setSimulation(sim);
	}
	

	
	sceneManager->initContext(freq);
		//Quad
		int quadind[6] = { 0,1,2,1,0,3 };
		optix::float3 quadv[4] = { make_float3(-0.5f,-0.5f,0.f),make_float3(0.5f,0.5f,0.f) ,make_float3(0.5f,-0.5f,0.f) ,make_float3(-0.5f,0.5f,0.f) };

		//45-degrees  x-titled down -0.7 quad with respect to parent

		optix::float3 quadt[4] = { make_float3(-0.5f, -1.1f, -0.9f),make_float3(0.5f, -0.3f, -0.1f) ,make_float3(0.5f, -0.3f, -0.1f) ,make_float3(0.5f, -0.3f, -0.1f) };

		Matrix4x4 tm;
		tm.setRow(0, make_float4(1, 0, 0, 0.f));
		tm.setRow(1, make_float4(0, 1, 0, 2.f));
		tm.setRow(2, make_float4(0, 0, 1, 75.f));
		tm.setRow(3, make_float4(0, 0, 0.f, 1));
		MaterialEMProperties emProp1;
		emProp1.dielectricConstant = make_float2(3.75f, -60.0f*sceneManager->getChannelParameters().waveLength*0.038f);
		//There is a dependency on the frequency again, we use -15 dB per 203 mm at 5 GHz => -75 dB/m
		emProp1.tattenuation = make_float2(0.1f,-75.f );

		//Creation of dynamic meshes  requires calling these 4 functions
		sceneManager->addDynamicMeshGroup(0);
		sceneManager->addMeshToGroup(0, 4, quadv, 6, quadind, emProp1);  //Call for each new mesh in the group
		sceneManager->addMeshToGroup(0, 4, quadt, 6, quadind, emProp1);  //Call for each new mesh in the group
		sceneManager->updateTransformInGroup(0, tm);
		sceneManager->finishDynamicMeshGroup(0);


		sceneManager->createRaySphere2D(1, 1);
		//sceneManager->createRaySphere2DSubstep(1, 1);
		//receivers
		optix::float3 posrx = make_float3(0.0f, 2.0f, 50.0f);


		optix::float3 postx = make_float3(0.0f, 2.0f, 0.0f);
		optix::float3 polarization = make_float3(0.0f, 1.0f, 0.0f); //Perpendicular to the floor. Assuming as in Unity that forward is z-axis and up is y-axis

		std::function<void(float,int)> cb=&sceneManager->printPower;
		sceneManager->addReceiver(1, posrx, polarization, 5.0f, cb);


		sceneManager->finishSceneContext();
		sceneManager->setPrintEnabled(1024 * 1024 * 1024);

		sceneManager->transmit(0, 1.0f, postx, polarization);


		//Translated and rotated 180 degrees, ends up symmetric to the previous position
		Matrix4x4 tm1;
		tm1.setRow(0, make_float4(-1.f, 0, 0, 0.f));
		tm1.setRow(1, make_float4(0, 1, 0, 2.f));
		tm1.setRow(2, make_float4(0, 0, -1.0f, -75.f));
		tm1.setRow(3, make_float4(0, 0, 0.f, 1));
		sceneManager->updateTransformInGroup(0, tm1);

		std::cout << "Only quad moved. Transmit again" << std::endl;
		sceneManager->transmit(0, 1.0f, postx, polarization);

		posrx = make_float3(0.0f, 2.0f, -50.0f);
		sceneManager->updateReceiver(1, posrx);

		std::cout << "Symmetric situation if everything has transformed well. Expect the  same power as first transmission. Transmit again" << std::endl;
		sceneManager->transmit(0, 1.0f, postx, polarization);

	timer.stop();
	std::cout<<"Time="<<timer.getTime()<<std::endl;



}
////To test edge conventions
//void DiffractionTests::ragged() {
//	//Four cubes, like a cross road. Tx is fixed
//	Timer timer;
//	float frequency=5.9e9;
//
//	//Add reflection simulation
//	LPFlatMeshReflectionSimulation* sim= new LPFlatMeshReflectionSimulation(sceneManager);
//	//BasicFlatMeshReflectionSimulation* sim= new BasicFlatMeshReflectionSimulation(sceneManager);
//	//RayDensityNormalizationSimulation* sim= new RayDensityNormalizationSimulation(sceneManager);
//	ComputeMode mode=ComputeMode::VOLTAGE;
//	//ComputeMode mode=ComputeMode::FIELD;
//	sim->setComputeMode(mode);
//	
////Use antenna gains
//	//sceneManager->setUseAntennaGain(true);
//
//	//sim->setComputeMode(ComputeMode::VOLTAGE);
//	//Disable or enable if necessary to check the different contributions
//	//sim->setEnableSimulation(false);
//	//sim->setEnableTraceLog(true);
//	sceneManager->enableGenerateRaysOnLaunch();
//	sceneManager->setSimulation(sim);
//
//	//Add diffraction simulation
//	SingleDiffraction* simd= new SingleDiffraction(sceneManager);
//	sceneManager->setSimulation(simd);
//	simd->setComputeMode(mode);
//	//simd->setEnableTraceLog(true);
//	//Disable or enable if necessary to check the different contributions
//	//simd->setEnableSimulation(false);
//
//	sceneManager->initContext(frequency);
//	std::cout << "Simulating ragged corners" << std::endl;
//
//	//MaterialEMProperties emProp1= sceneManager->ITUparametersToMaterial(3.75f,0.0f,0.038f,0.0f);
//	MaterialEMProperties emProp1;
//	//MaterialEMProperties emProp1;
//	emProp1.dielectricConstant = make_float2(3.75f, -60.0f*sceneManager->getChannelParameters().waveLength*0.038f);
//		//There is a dependency on the frequency again, we use -15 dB per 203 mm at 5 GHz => -75 dB/m
//	emProp1.tattenuation = make_float2(0.1f,-75.f );
//	
//}
//
