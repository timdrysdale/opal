/***************************************************************/
//
//Copyright (c) 2019 Esteban Egea-Lopez http://girtel.upct.es/~eegea
//
/**************************************************************/
#include "tunnelsBase.h"
#include "../util.h"
#include "../timer.h"
#include <iostream>
using namespace optix;
using namespace opal;
TunnelsBase::TunnelsBase(OpalSceneManager* sceneManager, float sphereRadius, bool useDepolarization) {
	this->sceneManager=sceneManager;
	this->sphereRadius=sphereRadius;
	this->useDepolarization=useDepolarization;
}
void TunnelsBase::loadHalfCylinder( float radius,  float length, float height, MaterialEMProperties emProp1) {
	std::vector<int> hcind = sceneManager->loadTrianglesFromFile("meshes/HalfCylinder1000-i.txt");
	std::vector<float3> hcvert = sceneManager->loadVerticesFromFile("meshes/HalfCylinder1000-v.txt");
	std::vector<float4> pd1 = sceneManager->loadPDFromFile("meshes/HalfCylinder1000-pd1.txt");	
	std::vector<float4> pd2 = sceneManager->loadPDFromFile("meshes/HalfCylinder1000-pd2.txt");	
	
	//TODO: We have to properly set the curvature radius. Is there any way to do this in a general way?
	int j=0;
	for (uint i=0; i<pd1.size();++i) {
		if (!std::isinf(pd1[i].w)) {
			pd1[i].w=pd1[i].w*radius; 

		}
		//Check correctness
	//	uint3 v_idx=make_uint3(j,j+1,j+2);
	//	int ix=hcind[v_idx.x];	
	//	int iy=hcind[v_idx.y];	
	//	int iz=hcind[v_idx.z];	
	//	const float3 p0    = hcvert[ix];
	//	const float3 p1    = hcvert[iy];
	//	const float3 p2    = hcvert[iz];
	//	const float3 e0 = p1 - p0;
	//	const float3 e1 = p0 - p2;
	//	const float3 n  = normalize(cross( e1, e0 ));
	//	float3 u1=make_float3(pd1[i].x,pd1[i].y,pd1[i].z);
	//	if (abs((acosf(dot(u1,n))*180/M_PI)-90)>1e-3) {
	//		std::cout<<i<<"n="<<n<<"|n|="<<optix::length(n)<<"u1="<<u1<<"dot(u1,n)="<<dot(u1,n)<<"p0="<<p0<<"p1="<<p1<<"p2="<<p2<<"ix="<<ix<<"iy="<<iy<<"iz"<<iz<<std::endl;
	//		std::cout<<i<<"angle="<<(acosf(dot(u1,n))*180/M_PI)<<std::endl;
	//	}
	//	j+=3;
	}
	j=0;
	for (size_t i=0; i<pd2.size();++i) {
		if (!std::isinf(pd2[i].w)) {
			pd2[i].w=pd2[i].w*radius;

		}
		//Check correctness
		uint3 v_idx=make_uint3(j,j+1,j+2);
		const float3 p0    = hcvert[hcind[v_idx.x]];
		const float3 p1    = hcvert[hcind[v_idx.y]];
		const float3 p2    = hcvert[hcind[v_idx.z]];
		const float3 e0 = p1 - p0;
		const float3 e1 = p0 - p2;
		const float3 n  = normalize(cross( e1, e0 ));
		float3 u2=make_float3(pd2[i].x,pd2[i].y,pd2[i].z);
		if (i==8) {
		//if (abs((acosf(dot(u2,n))*180/M_PI)-90)>1e-3) {
			std::cout<<i<<"n="<<n<<"|n|="<<optix::length(n)<<"u2="<<u2<<"dot(u2,n)="<<dot(u2,n)<<"p0="<<p0<<"p1="<<p1<<"p2="<<p2<<std::endl;
			std::cout<<i<<"angle="<<(acosf(dot(u2,n))*180/M_PI)<<std::endl;
		}
		j+=3;
	}
	
	
	std::cout << "Loading HalfCylinder with indices=" << hcind.size() << ", triangles="<<(hcind.size()/3)<<", vertices=" << hcvert.size() <<" and curvatures pd1="<<pd1.size()<<"pd2="<<pd2.size() << std::endl;
	Matrix4x4 tm;
	tm.setRow(0, make_float4(radius, 0, 0, 0.0f));
	tm.setRow(1, make_float4(0, radius, 0, height));
	tm.setRow(2, make_float4(0, 0, length, 0.0f));
	tm.setRow(3, make_float4(0, 0, 0, 1));
	std::cout << "Adding HalfCylinder. tm="<<tm<<"; Em="<< emProp1.dielectricConstant << std::endl;
	//Here we add this mesh as a single face one, since  the curved mesh is used to approximate the normals of the half cylinder, but it is the same wall 
	sceneManager->addStaticCurvedMesh(hcvert,  hcind, pd1, pd2, tm, emProp1, true);

}

void TunnelsBase::loadCircularTunnel(float radius,  float length, MaterialEMProperties emProp1) {
	//A circular tunnel represented with a full cylinder
	//Origin is at center
	std::string file("FullCylinder10000");
	std::vector<int> hcind = sceneManager->loadTrianglesFromFile(("meshes/"+file+"-i.txt").c_str());
	std::vector<float3> hcvert = sceneManager->loadVerticesFromFile(("meshes/"+file+"-v.txt").c_str());
	std::vector<float4> pd1 = sceneManager->loadPDFromFile(("meshes/"+file+"-pd1.txt").c_str());	
	std::vector<float4> pd2 = sceneManager->loadPDFromFile(("meshes/"+file+"-pd2.txt").c_str());	
	//std::vector<int> hcind = sceneManager->loadTrianglesFromFile("meshes/HalfCylinder10c-i.txt");
	//std::vector<float3> hcvert = sceneManager->loadVerticesFromFile("meshes/HalfCylinder10c-v.txt");
	//std::vector<float4> pd1 = sceneManager->loadPDFromFile("meshes/HalfCylinder10c-pd1.txt");	
	//std::vector<float4> pd2 = sceneManager->loadPDFromFile("meshes/HalfCylinder10c-pd2.txt");	
	//std::vector<int> hcind = sceneManager->loadTrianglesFromFile("meshes/FullCylinder10-i.txt");
	//std::vector<float3> hcvert = sceneManager->loadVerticesFromFile("meshes/FullCylinder10-v.txt");
	//std::vector<float4> pd1 = sceneManager->loadPDFromFile("meshes/FullCylinder10-pd1.txt");	
	//std::vector<float4> pd2 = sceneManager->loadPDFromFile("meshes/FullCylinder10-pd2.txt");	
	
	//TODO: We have to properly set the curvature radius. Is there any way to do this in a general way?
	for (size_t i=0; i<pd1.size();++i) {
		if (!std::isinf(pd1[i].w)) {
			pd1[i].w=pd1[i].w*radius; 

		}
	}
	for (size_t i=0; i<pd2.size();++i) {
		if (!std::isinf(pd2[i].w)) {
			pd2[i].w=pd2[i].w*radius;

		}
	}
	
	std::cout << "Loading "<<file<<" with indices=" << hcind.size() << ", triangles="<<(hcind.size()/3)<<", vertices=" << hcvert.size() <<" and curvatures pd1="<<pd1.size()<<"pd2="<<pd2.size() << std::endl;
	Matrix4x4 tm;
	tm.setRow(0, make_float4(radius, 0, 0, 0.0f));
	tm.setRow(1, make_float4(0, radius, 0, 0.0f));
	tm.setRow(2, make_float4(0, 0, length, 0.0f)); //Origin is at center and at the beginning of the tunnel
	tm.setRow(3, make_float4(0, 0, 0, 1));
	//Get concrete
	//MaterialEMProperties emProp1 = sceneManager->ITUparametersToMaterial(5.31,0,0.0326,0.8905);
	
	//Material from Dudley (2006). Fig. 8
	std::cout << "Adding "<<file<<". Em="<< emProp1.dielectricConstant << std::endl;
	
	//sceneManager->addStaticMesh(static_cast<int>(hcvert.size()), hcvert.data(), static_cast<int>(hcind.size()), hcind.data(), tm, emProp1, true);
//	sceneManager->addStaticMesh(static_cast<int>(hcvert.size()), hcvert.data(), static_cast<int>(hcind.size()), hcind.data(), tm, emProp1, false);
//	std::cout<<"Writing cylinder" <<std::endl;
//	sceneManager->writeMeshToPLYFile("cyl50.ply", hcvert,hcind, tm);
	//Here we add this mesh as a single face one, since  the curved mesh is used to approximate the normals of the  cylinder, but it is the same wall 
	sceneManager->addStaticCurvedMesh(hcvert,  hcind, pd1, pd2, tm, emProp1, true);
}
std::vector<int> TunnelsBase::parseTestString(std::string test) {
	//Parse tests
	std::vector<int> tokens; 
	if (test.empty()) {
		throw opal::Exception("tunnelsBase::parseTestString(): empty test string");
	}	
      
    // stringstream class check1 
	std::stringstream check1(test); 
      
    std::string intermediate; 
      
    while(getline(check1, intermediate, '-')) 
    { 
        tokens.push_back(std::stoi(intermediate)); 
    } 
	return tokens;
}
void TunnelsBase::loadSquareTunnel( float width, float height, float length, MaterialEMProperties emProp1) {
	//Tunnel is a cube with the XY faces removed (entrance and exit). So tunnel runs on Z axis (front, in Unity)
	std::cout << "Loading square tunnel with width=" <<width<<",  height="<<height<< " and length="<<length<<" from meshes/tunnel-i/v.txt"<< std::endl;
	std::vector<int> cubeind = sceneManager->loadTrianglesFromFile("meshes/tunnel-i.txt");
	std::vector<float3> cubevert = sceneManager->loadVerticesFromFile("meshes/tunnel-v.txt");
	//Cube(4) NW
	Matrix4x4 tm;
	tm.setRow(0, make_float4(width, 0, 0, 0.0f));
	tm.setRow(1, make_float4(0, height, 0, 0.0f));
	tm.setRow(2, make_float4(0, 0, length, length/2.0f));
	tm.setRow(3, make_float4(0, 0, 0, 1));
	std::cout << "Adding Square tunnel tm="<<tm<<" Em="<< emProp1.dielectricConstant << std::endl;
	sceneManager->addStaticMesh(static_cast<int>(cubevert.size()), cubevert.data(), static_cast<int>(cubeind.size()), cubeind.data(), tm, emProp1);

}
