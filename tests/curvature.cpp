/***************************************************************/
//
//Copyright (c) 2019 Esteban Egea-Lopez http://girtel.upct.es/~eegea
//
/**************************************************************/
#include "curvature.h"
#include "../util.h"
#include "../timer.h"
#include "../curvedMeshSimulation.h"

#include <fstream>
#include <iostream>
using namespace optix;
using namespace opal;
void CurvatureTests::loadCylinder (float radius,  float length, MaterialEMProperties emProp1, bool outside) {
	//WARNING: this cylinder mesh has a thickness of 0.1. The radius is the internal radius. If you hit from outside, take this into accoun to set properly the external radius of your cylinder, 
	//We add  bool outside to explicit this
	//Load  a full cylinder running along the z-axis, origin is at center (0,0,0)
	std::vector<int> hcind = sceneManager->loadTrianglesFromFile("meshes/FullCylinder2000-i.txt");
	std::vector<float3> hcvert = sceneManager->loadVerticesFromFile("meshes/FullCylinder2000-v.txt");
	std::vector<float4> pd1 = sceneManager->loadPDFromFile("meshes/FullCylinder2000-pd1.txt");	
	std::vector<float4> pd2 = sceneManager->loadPDFromFile("meshes/FullCylinder2000-pd2.txt");	

	//TODO: We have to properly set the curvature radius. Is there any way to do this in a general way?
	for (size_t i=0; i<pd1.size();++i) {
		if (!std::isinf(pd1[i].w)) {
			if (outside) {
				pd1[i].w=pd1[i].w*(radius+0.1); 
			} else {
				pd1[i].w=pd1[i].w*radius; 
			}

		}
	}
	for (size_t i=0; i<pd2.size();++i) {
		if (!std::isinf(pd2[i].w)) {
			if (outside) {
				pd2[i].w=pd2[i].w*(radius+0.1);
			} else {
				pd2[i].w=pd2[i].w*radius;
			}

		}
	}

	std::cout << "Loading FullCylinder with radius="<<radius<<" length="<<length<<" with indices=" << hcind.size() << ", triangles="<<(hcind.size()/3)<<", vertices=" << hcvert.size() <<" and curvatures pd1="<<pd1.size()<<"pd2="<<pd2.size() << std::endl;
	Matrix4x4 tm;
	tm.setRow(0, make_float4(radius, 0, 0, 0.0f));
	tm.setRow(1, make_float4(0, radius, 0, 0.0f));
	tm.setRow(2, make_float4(0, 0, length, 0.0f)); //Origin is at center and at the beginning of cylinder
	tm.setRow(3, make_float4(0, 0, 0, 1));
	std::cout << "Adding FullCylinder. Em="<< emProp1.dielectricConstant << std::endl;

	//Here we add this mesh as a single face one, since  the curved mesh is used to approximate the normals of the  cylinder, but it is the same wall 
	sceneManager->addStaticCurvedMesh(hcvert,  hcind, pd1, pd2, tm, emProp1, true);
}

CurvatureTests::CurvatureTests(OpalSceneManager*   sceneManager, float sphereRadius) {
	this->sceneManager=sceneManager;
	this->sphereRadius=sphereRadius;
}
void CurvatureTests::cylinderTest() {
	float freq = 25e9f;
	LPCurvedMeshReflectionSimulation* sim=new LPCurvedMeshReflectionSimulation(sceneManager);
	sceneManager->setSimulation(sim);
	sceneManager->initContext(freq);
	sceneManager->getSimulation()->setPrintHits(true);
	Timer timer;
	//Receiver polarization
	//optix::float3 polarization = V; 	
	optix::float3 polarization = make_float3(0,0,1); 	

	//********Single receiver
	int j=1;	
	////Position
	//optix::float3 posrx = make_float3(1.5f, 0.0f, 1.0f);	
	optix::float3 posrx = make_float3(-2.0f,-3.0f, 2.0f);	
	//optix::float3 posrx = make_float3(-3.0f, 0.0f, 2.0f);	
	//////Add to scene
	sceneManager->addReceiver(j, posrx,polarization, sphereRadius, sceneManager->printPower);
	//*********************

	//Transmitter
	//optix::float3 postx = make_float3(-1.0f, 0.0f, 2.0f);
	optix::float3 postx = make_float3(-2.0f,3.0f, 2.0f);
	float3 polarizationTx = polarization; 

	std::cout <<"**** Single ray cylinder ***"<<std::endl;	
	//MaterialEMProperties emProp1;
	//emProp1.dielectricConstant = make_float2(5.0f, -60.0f*sceneManager->getChannelParameters().waveLength*0.01f);
	//There is a dependency on the frequency again, we use -15 dB per 203 mm at 5 GHz => -75 dB/m
	MaterialEMProperties emProp1 = sceneManager->ITUparametersToMaterial(2.58,0,0.0217,0.78);
	emProp1.tattenuation = make_float2(0.1f,-75.f );
	//WARNING: the cylinder mesh has a thickness of 0.01
	loadCylinder(0.4,1200.0f, emProp1, true);
	std::cout<<"Transmitting at "<<postx<<" with polarization="<<polarizationTx<<std::endl;
	std::cout<<"Receiving with radius="<<sphereRadius<<" with polarization="<<polarization<<std::endl;
	sceneManager->setMinEpsilon(1e-4f);

	//***Single ray transmit****
	float3 mRay=normalize(make_float3(1.5,-3.0f,0.0f));

	sceneManager->createRaySphere2D(1,1,&mRay);
	//sceneManager->createRaySphere2D(45,0.1,46,0,0.1,1);

	//Angle for separating duplicate rays
	float discriminateAngle=5.0f; //In degrees
	//LPCurvedMeshReflectionSimulation* sim=dynamic_cast<LPCurvedMeshReflectionSimulation*>(sceneManager->getSimulation());
	sim->setMaxAngleForDuplicateRays(discriminateAngle*M_PIf/180.f);

	sceneManager->finishSceneContext();

	//First launch
	sceneManager->transmit(0, 1.0f, postx, polarizationTx, false);
	//Divergence computed at the hit point (0.5,0,2) should be 0.1250 and at the reciver 0.125*1/rayLength=0.125/3.354n
}
void CurvatureTests::symmetricDivergenceTest() {

	float3 rays[]={make_float3(1.5519056e+00,   2.7781636e+00,  -3.3497798e-01), 
		make_float3(1.5473953e+00,   2.4875172e+00,  -3.2612345e-01),
		make_float3(1.5423610e+00,   2.1985887e+00,  -3.1662103e-01),
		make_float3(1.5367751e+00,   1.9117908e+00,  -3.0652287e-01),
		make_float3(1.5306531e+00,   1.6276241e+00,  -2.9596244e-01),
		make_float3(1.5240931e+00,   1.3466618e+00,  -2.8519869e-01),
		make_float3(1.5173340e+00,   1.0694874e+00,  -2.7466722e-01),
		make_float3(1.5108196e+00,   7.9654675e-01,  -2.6501906e-01),
		make_float3(1.5052267e+00,   5.2789316e-01,  -2.5710479e-01),
		make_float3(1.5013798e+00,   2.6288008e-01,  -2.5184907e-01),
		make_float3(1.5000000e+00,   2.7002161e-55,  -2.5000000e-01),
		make_float3(1.5013798e+00,  -2.6288008e-01,  -2.5184907e-01),
		make_float3(1.5052267e+00,  -5.2789316e-01,  -2.5710479e-01),
		make_float3(1.5108196e+00,  -7.9654675e-01,  -2.6501906e-01),
		make_float3(1.5173340e+00,  -1.0694874e+00,  -2.7466722e-01),
		make_float3(1.5240931e+00,  -1.3466618e+00,  -2.8519869e-01),
		make_float3(1.5306531e+00,  -1.6276241e+00,  -2.9596244e-01),
		make_float3(1.5367751e+00,  -1.9117908e+00,  -3.0652287e-01),
		make_float3(1.5423610e+00,  -2.1985887e+00,  -3.1662103e-01),
		make_float3(1.5473953e+00,  -2.4875172e+00,  -3.2612345e-01),
		make_float3(1.5519056e+00,  -2.7781636e+00,  -3.3497798e-01)};

	float freq = 25e9f;
	LPCurvedMeshReflectionSimulation* sim=new LPCurvedMeshReflectionSimulation(sceneManager);
	sceneManager->setSimulation(sim);
	sceneManager->initContext(freq);
	sceneManager->getSimulation()->setPrintHits(true);
	Timer timer;
	//Receiver polarization
	//optix::float3 polarization = V; 	
	optix::float3 polarization = make_float3(0,0,1); 	


	std::cout <<"**** Single ray cylinder ***"<<std::endl;	
	//MaterialEMProperties emProp1;
	//emProp1.dielectricConstant = make_float2(5.0f, -60.0f*sceneManager->getChannelParameters().waveLength*0.01f);
	//There is a dependency on the frequency again, we use -15 dB per 203 mm at 5 GHz => -75 dB/m
	MaterialEMProperties emProp1 = sceneManager->ITUparametersToMaterial(2.58,0,0.0217,0.78);
	emProp1.tattenuation = make_float2(0.1f,-75.f );
	//WARNING: the cylinder mesh has a thickness of 0.01
	loadCylinder(0.4,1200.0f, emProp1, true);
	sceneManager->setMinEpsilon(1e-4f);


	//********Single receiver
	int j=1;	
	////Position
	optix::float3 posrx = make_float3(-2.0f, 0.0f, 2.1f);	
	//////Add to scene
	sceneManager->addReceiver(j, posrx,polarization, sphereRadius, sceneManager->printPower);
	//*********************

	//Transmitter
	optix::float3 postx = make_float3(-2.0f,3.0f, 2.0f);
	float3 polarizationTx = polarization; 

	std::cout<<"Receiving with radius="<<sphereRadius<<" with polarization="<<polarization<<std::endl;
	//***Single ray transmit****
	float3 mRay=normalize(make_float3(1.5,-3.0f,0.0f));

	sceneManager->createRaySphere2D(1,1,&mRay);
	//sceneManager->createRaySphere2D(45,0.1,46,0,0.1,1);

	//Angle for separating duplicate rays
	float discriminateAngle=5.0f; //In degrees
	//LPCurvedMeshReflectionSimulation* sim=dynamic_cast<LPCurvedMeshReflectionSimulation*>(sceneManager->getSimulation());
	sim->setMaxAngleForDuplicateRays(discriminateAngle*M_PIf/180.f);

	sceneManager->finishSceneContext();
	//First launch
	for (int i=0;i<20;i++) {
		postx=make_float3(-2.0f,-3.0f +0.3f*i,2.6f);
		float3 mRay=normalize(rays[i]);
		sceneManager->createRaySphere2D(1,1,&mRay);
		std::cout<<"Transmitting at "<<postx<<" with polarization="<<polarizationTx<< "with ray="<<rays[0]<<",r="<<mRay<<std::endl;
		sceneManager->transmit(0, 1.0f, postx, polarizationTx, false);
	}
	//The shape of the curve of the divergence at the hit points on the outside surface of the cylinder must be symmetric

	//int i=7;
	//	postx=make_float3(-2.0f,-3.0f +0.3f*i,2.6f);
	//	mRay=normalize(rays[i]);
	//	sceneManager->createRaySphere2D(1,1,&mRay);
	//	std::cout<<"Transmitting at "<<postx<<" with polarization="<<polarizationTx<< "with ray="<<rays[0]<<",r="<<mRay<<std::endl;
	//	sceneManager->transmit(0, 1.0f, postx, polarizationTx, false);
}

