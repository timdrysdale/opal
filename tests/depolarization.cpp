/***************************************************************/
//
//Copyright (c) 2019 Esteban Egea-Lopez http://girtel.upct.es/~eegea
//
/**************************************************************/
#include "depolarization.h"
#include "../timer.h"
#include "../Opal.h"
#include <memory>
#include "../flatSimulation.h"
using namespace opal;
using namespace optix;
//Polarization test. Horizontal plane test but with arbitrary polarizations. To validate against a two-ray model
std::unique_ptr<OpalSceneManager> polarizationPlaneTest(std::unique_ptr<OpalSceneManager> sceneManager, bool print, bool subSteps) {
	float freq=5.9e9f;
	LPFlatMeshReflectionSimulation* sim=new LPFlatMeshReflectionSimulation(sceneManager.get());
	sceneManager->setSimulation(sim);	
	sceneManager->initContext(freq);
	
	
	std::cout << "Running polarizationPlaneTest" << std::endl;
	//Horizontal plane as quad at origin. Better to use this plane, since precision problems occur when rays are aligned with triangles and some intersections with the plane may be missed
	int quadind[6] = { 0,1,2,1,0,3 };
	optix::float3 quadh[4] = { make_float3(-0.5f,0.0f,-0.5f),make_float3(0.5f,0.f,0.5f) ,make_float3(0.5f,0.f,-0.5f) ,make_float3(-0.5f,0.0f,0.5f) };

	//Scale to 200x200
	Matrix4x4 tm;
	tm.setRow(0, make_float4(200, 0, 0, 0.f));
	tm.setRow(1, make_float4(0, 1, 0, 0.f));
	tm.setRow(2, make_float4(0, 0, 200, 0.f));
	tm.setRow(3, make_float4(0, 0, 0,  1));

	MaterialEMProperties emProp1;
	emProp1.dielectricConstant = make_float2(3.75f, -60.0f*sceneManager->getChannelParameters().waveLength*0.038f);
		//There is a dependency on the frequency again, we use -15 dB per 203 mm at 5 GHz => -75 dB/m
		emProp1.tattenuation = make_float2(0.1f,-75.f );
	sceneManager->addStaticMesh(4, quadh, 6, quadind, tm, emProp1 );


//	if (subSteps) {
//		sceneManager->createRaySphere2DSubstep(1, 1);
//	} else {
//		sceneManager->createRaySphere2D(60,120);
//	}
	
	sceneManager->createRaySphere2D(120.0f,1.0f,121.0f,0.0f,1.0f,1.0f);
//receivers
	//optix::float3 polarization = make_float3(1.0f, 0.0f, 0.0f); //Horizontal to the floor. Assuming as in Unity that forward is z-axis and up is y-axis
	optix::float3 polarization = make_float3(1.0f, 0.0f, 0.0f); //Perpendicular to the floor. Assuming as in Unity that forward is z-axis and up is y-axis
	//optix::float3 polarization = make_float3(1.0f, 1.0f, 0.0f); //Tilted to the floor. Assuming as in Unity that forward is z-axis and up is y-axis. Not normalized
	//optix::float3 polarization = make_float3(1, 1, 0.0f); //Tilted to the floor. Assuming as in Unity that forward is z-axis and up is y-axis. Not normalized
	//optix::float3 posrx = make_float3(0.0f, 2.0f, 0.0f);


	optix::float3 postx = make_float3(0.0f, 10.0f, 0.0f);


	//sceneManager->addReceiver(1, posrx, polarization, 1.0f, sceneManager->printPower);
	
	//With different polarization for the receiver
	optix::float3 posrx = make_float3(0.0f, 2.0f,10.0f*tanf(M_PI/3.0)+ 2.0/tanf(M_PI/6.0));
	sceneManager->addReceiver(1, posrx, make_float3(1.0f,0.0f,0.0f), 1.0f, sceneManager->printPower);


	sceneManager->finishSceneContext();

	if (print) {
		sceneManager->setPrintEnabled(1024 * 1024 * 1024);
	}
	//	sceneManager->setUsageReport();


	//std::cout << "Launching" << std::endl;




	sceneManager->transmit(0, 1.0f, postx, polarization);
//	Timer timer;
//	timer.start();
//	for (size_t i = 0; i < 100; ++i)
//	{
//		postx = make_float3(0.0f, 10.0f, 99.0f - i);
//		sceneManager->transmit(0, 1.0f, postx, polarization);
//
//	}
//	timer.stop();
//	std::cout<<"Time="<<timer.getTime()<<std::endl;


	return sceneManager;
}
//Street crossing test with arbitray polarizations. Cubes are intended to be buildings and a plane is the floor
std::unique_ptr<OpalSceneManager> crossingTestDepolarization(std::unique_ptr<OpalSceneManager> sceneManager, bool print, bool subSteps) {

	Timer timer;
	float freq=5.9e9f;
	LPFlatMeshReflectionSimulation* sim=new LPFlatMeshReflectionSimulation(sceneManager.get());
	sceneManager->setSimulation(sim);	
	sceneManager->initContext(freq);

	std::cout << "Simulating crossing streets test" << std::endl;
	//Cubes
	std::vector<int> cubeind = sceneManager->loadTrianglesFromFile("meshes/tricube.txt");
	std::vector<float3> cubevert = sceneManager->loadVerticesFromFile("meshes/vertcube.txt");
	//std::cout << "indices=" << cubeind.size() << "vertices=" << cubevert.size() << std::endl;
	//Cube(4) NW
	Matrix4x4 tm;
	tm.setRow(0, make_float4(40.0f, 0, 0, -30.0f));
	tm.setRow(1, make_float4(0, 40.0f, 0, 20.0f));
	tm.setRow(2, make_float4(0, 0, 40.0f, 80.0f));
	tm.setRow(3, make_float4(0, 0, 0, 1));
	MaterialEMProperties emProp1;
	emProp1.dielectricConstant = make_float2(3.75f, -60.0f*sceneManager->getChannelParameters().waveLength*0.038f);
		//There is a dependency on the frequency again, we use -15 dB per 203 mm at 5 GHz => -75 dB/m
		emProp1.tattenuation = make_float2(0.1f,-75.f );
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
//	std::vector<int> planeind = sceneManager->loadTrianglesFromFile("meshes/tri.txt");
//	std::vector<float3> planever = sceneManager->loadVerticesFromFile("meshes/vert.txt");
//	//std::cout << "indices=" << planeind.size() << "vertices=" << planever.size() << std::endl;
//
//	tm.setRow(0, make_float4(10.0f, 0, 0, 0.0f));
//	tm.setRow(1, make_float4(0, 1, 0, 0.0f));
//	tm.setRow(2, make_float4(0, 0, 10.0f, 50.0f));
//	tm.setRow(3, make_float4(0, 0, 0, 1));
	

	//Horizontal plane as quad at origin. Better to use this plane, since precision problems occur when rays are aligned with triangles and some intersections with the plane may be missed
	int quadind[6] = { 0,1,2,1,0,3 };
	optix::float3 quadh[4] = { make_float3(-0.5f,0.0f,-0.5f),make_float3(0.5f,0.f,0.5f) ,make_float3(0.5f,0.f,-0.5f) ,make_float3(-0.5f,0.0f,0.5f) };

	//Scale to 200x200
	tm.setRow(0, make_float4(200, 0, 0, 0.f));
	tm.setRow(1, make_float4(0, 1, 0, 0.f));
	tm.setRow(2, make_float4(0, 0, 200, 0.f));
	tm.setRow(3, make_float4(0, 0, 0,  1));

	//emProp1.dielectricConstant = make_float2(3.75f, -60.0f*sceneManager->defaultChannel.waveLength*0.15f);
	std::cout << "Adding Plane. Em=" << emProp1.dielectricConstant << std::endl;
	//sceneManager->addStaticMesh(static_cast<int>(planever.size()), planever.data(), static_cast<int>(planeind.size()), planeind.data(), tm, emProp1);
	sceneManager->addStaticMesh(4, quadh, 6, quadind, tm, emProp1 );
//
	if (subSteps) {
		sceneManager->createRaySphere2DSubstep(1, 1); //0.1 degree delta step
	} else {
		sceneManager->createRaySphere2D(1, 1); //1 degree delta step
	}

	//receivers

	optix::float3 posrx = make_float3(0.0f, 10.0f, 100.0f);
	optix::float3 polarization = make_float3(0.0f, 1.0f, 0.0f); //Perpendicular to the floor. Assuming as in Unity that forward is z-axis and up is y-axis
	sceneManager->addReceiver(1, posrx,polarization, 5.0f, sceneManager->printPower);


	//sceneManager->setMaxReflections(3u);

	sceneManager->finishSceneContext();

	if (print) {
		sceneManager->setPrintEnabled(1024 * 1024 * 1024);	
	}
	//sceneManager->setUsageReport();

	optix::float3 postx;
	timer.start();

	for (int i = -50; i <= 50; ++i) {

		float x=i;
		postx = make_float3(x, 10.f, 50.0f);

		sceneManager->transmit(0, 1.0f, postx, polarization);


	}
			//postx = make_float3(0.0f, 10.0f, 50.0f);
			//sceneManager->transmit(0, 1.0f, postx, polarization);
	timer.stop();
	std::cout<<"Time="<<timer.getTime()<<std::endl;

	return sceneManager;

}



