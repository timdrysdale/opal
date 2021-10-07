/***************************************************************/
//
//Copyright (c) 2019 Esteban Egea-Lopez http://girtel.upct.es/~eegea
//
/**************************************************************/
#include "roux.h"
#include "../timer.h"
#include <memory>
#include <fstream>
#include "../curvedMeshSimulation.h"
#include "../curvedFlatMeshSimulation.h"
#include "../rayDensityNormalizationSimulation.h"
#include "../util.h"
using namespace opal;
using namespace optix;
RouxTests::RouxTests(OpalSceneManager* sceneManager, float sphereRadius, bool useDepolarization) : TunnelsBase(sceneManager,sphereRadius,useDepolarization) {
}
void RouxTests::loadRouxTunnel(float radius,  float length, float height) {
	//Roux tunnel is a dome (half cylinder and 3  cube walls with the XY faces removed (entrance and exit). So tunnel runs on Z axis (front, in Unity)
	//Load a half cylinder and 3 quads as walls and floors
	
	//Get concrete
	//MaterialEMProperties emProp1 = sceneManager->ITUparametersToMaterial(5.31,0,0.0326,0.8905);
	//else
	MaterialEMProperties emProp1;
	emProp1.dielectricConstant = make_float2(5.0f, -60.0f*sceneManager->getChannelParameters().waveLength*0.01f);
	emProp1.tattenuation = make_float2(0.1f,-75.f );
	
	loadHalfCylinder(radius,length,height, emProp1);

	Matrix4x4 tm;
	//Quad triangles
	int quadind[6] = { 0,1,2,1,0,3 };
	//XY quad
	optix::float3 quadv[4] = { make_float3(-0.5f,-0.5f,0.f),make_float3(0.5f,0.5f,0.f) ,make_float3(0.5f,-0.5f,0.f) ,make_float3(-0.5f,0.5f,0.f) };
	//Add left wall (looking from origin to end of tunnel, that is, looking forward, Z axis)
	//Scale to length and radius and rotate 
	tm.setRow(0, make_float4(0, 0, -1, -radius));
	tm.setRow(1, make_float4(0, height, 0, 1.f));
	tm.setRow(2, make_float4(length, 0, 0, length/2.0f));
	tm.setRow(3, make_float4(0, 0, 0,  1));
	
	sceneManager->addStaticMesh(4, quadv, 6, quadind, tm, emProp1 );
	
	//Add right wall (looking from origin to end of tunnel, that is, looking forward, Z axis)
	//Scale to length and radius 
	tm.setRow(0, make_float4(0, 0, 1, radius));
	tm.setRow(1, make_float4(0, height, 0, 1.f));
	tm.setRow(2, make_float4(-length, 0, 0, length/2.0f));
	tm.setRow(3, make_float4(0, 0, 0,  1));
	
	sceneManager->addStaticMesh(4, quadv, 6, quadind, tm, emProp1 );
	//Add floor
	//Scale to length and radius 
	tm.setRow(0, make_float4(2.0f*radius, 0, 0, 0.f));
	tm.setRow(1, make_float4(0, 0, -1, 0.f));
	tm.setRow(2, make_float4(0, length, 0 ,  length/2.0f));
	tm.setRow(3, make_float4(0, 0, 0,  1));

	sceneManager->addStaticMesh(4, quadv, 6, quadind, tm, emProp1 );
}
void RouxTests::runTunnel() {
	float freq = 1e9f;
	LPCurvedFlatMeshReflectionSimulation* sim = new LPCurvedFlatMeshReflectionSimulation(sceneManager);
	sceneManager->setSimulation(sim);
	sceneManager->initContext(freq);

	sceneManager->getSimulation()->setPrintHits(false);
	loadRouxTunnel(4.0f,1200.0f,2.0f);
	Timer timer;
	//Receiver polarization
	optix::float3 polarization = V; 	
	
	//Transmitter
	optix::float3 postx = make_float3(0.0f,4.5f, 0.0f);
	float3 polarizationTx = V; 
	//Sphere scanning
	float initElevation=0.0f;
	float initAzimuth=-90.0f;
	float endElevation=180.0f;
	float endAzimuth=90.0f;
	float deltaEl=10.0f;
	float deltaAz=10.0f;
	float asEl=0.01f;
	float asAz=0.01f;
	float overlap=0.5f;
	//float overlap=0.0f;
	std::cout<<"****+ Simulating Rous tunnel ***** "<<std::endl;
	std::cout<<"Transmitting at "<<postx<<" with polarization="<<polarizationTx<<std::endl;
	std::cout<<"Receiving with radius="<<sphereRadius<<" with polarization="<<polarization<<std::endl;
	std::cout<<"Scanning the sphere with ASElevation="<<asEl<< " and ASAzimuth="<<asAz<<std::endl;
	sceneManager->setMinEpsilon(1e-4f);
	
	float currentElevation=initElevation;
	float currentAzimuth=initAzimuth;
	//std::cout<<"Tracing angle (el/az)="<<(currentElevation-overlap)<<","<<(currentElevation+deltaEl+overlap)<<"/"<<(currentAzimuth-overlap)<<","<<(currentAzimuth+deltaAz+overlap)<<std::endl;
	
	sceneManager->createRaySphere2D(currentElevation-overlap,asEl,currentElevation+deltaEl+overlap,currentAzimuth-overlap,asAz,currentAzimuth+deltaAz+overlap);
	
	//sceneManager->setPrintEnabled(1024*1024*1024, make_uint3(976,552,0));
	sceneManager->finishSceneContext();
	timer.start();
	
	//Angle for separating duplicate rays

	float discriminateAngle=2.5f; //In degrees
	//LPCurvedMeshReflectionSimulation* sim=dynamic_cast<LPCurvedMeshReflectionSimulation*>(sceneManager->getSimulation());
	sim->setMaxAngleForDuplicateRays(discriminateAngle*M_PIf/180.f);
	
	
	/******** With angle discrimination ********/
	/** Sequential test. For a really large number of reflections and receivers  (that is, potential hits) have to do a sequential test, to avoid buffer overflows */
	sceneManager->addReceiver(1,make_float3(1.0f,-1.0f, 10.f),polarization, sphereRadius, sceneManager->printPower);
	float zinit=1.0f;
	float deltar=sphereRadius;
	uint launches=0;
	//Longitudinal
	for (int i=0;i<=250;++i) {
		float3 posrx=make_float3(-2.0f,1.5f,zinit );
		sceneManager->updateReceiver(1, posrx,deltar);
		zinit+=2.0f;
		//deltar+=0.01;

	//Transversal
//	float xinit=-3.5f;
//	for (int i=-35;i<=35;++i) {
//		float3 posrx=make_float3(i/10.0f,1.5f,5.0f);
//		sceneManager->updateReceiver(1, posrx,deltar);
//		xinit+=0.07f;
//		//deltar+=0.01;
//		
//	//sceneManager->addReceiver(1,make_float3(-1.0f,1.5f, 5.f),polarization, sphereRadius, sceneManager->printPower);
//	//sceneManager->addReceiver(2,make_float3(1.0f,1.5f, 5.f),polarization, sphereRadius, sceneManager->printPower);
		
		
		//First launch
		sceneManager->transmit(0, 1.0f, postx, polarizationTx, true);

		//Now loop to fill the solid angle
		currentAzimuth += deltaAz;
		//Trace all elevations
		while (currentElevation<endElevation) {

			//Trace all azimuth	
			while(currentAzimuth<endAzimuth) {
				std::cout<<"Tracing angle (el/az)="<<(currentElevation-overlap)<<","<<(currentElevation+deltaEl+overlap)<<"/"<<(currentAzimuth-overlap)<<","<<(currentAzimuth+deltaAz+overlap)<<std::endl;
				sceneManager->createRaySphere2D(currentElevation-overlap,asEl,currentElevation+deltaEl+overlap,currentAzimuth-overlap,asAz,currentAzimuth+deltaAz+overlap);
				sceneManager->transmit(0, 1.0f, postx, polarizationTx, true);
				currentAzimuth += deltaAz;
			}
			currentAzimuth=initAzimuth;
			currentElevation += deltaEl;
		}
		sceneManager->endPartialLaunch(1u);

		currentElevation=initElevation;
		currentAzimuth=initAzimuth;
		++launches;

	} 
	
	timer.stop();
	std::cout<<"Time="<<timer.getTime()<<". Time/launch="<<(timer.getTime()/launches)<<std::endl;
	

}

