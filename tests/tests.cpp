/***************************************************************/
//
//Copyright (c) 2019 Esteban Egea-Lopez http://girtel.upct.es/~eegea
//
/**************************************************************/
#include "tests.h"
#include "../timer.h"
#include "../Opal.h"
#include <memory>
#include <fstream>
#include "../curvedMeshSimulation.h"
#include "../curvedFlatMeshSimulation.h"
#include "../basicSimulation.h"
#include "../flatSimulation.h"
#include "../singleDiffraction.h"
#include "../util.h"

using namespace opal;
using namespace optix;





BasicTests::BasicTests(OpalSceneManager*   sceneManager, float sphereRadius, bool useDepolarization) {
	this->sceneManager=sceneManager;
	this->sphereRadius=sphereRadius;
	this->useDepolarization=useDepolarization;
}


//Horizontal plane test. To validate against a two-ray model
void BasicTests::planeTest(int mode) {
	Timer timer;
	float freq = 5.9e9f;
	std::cout<<"Running plane test"<<std::endl;
	
	//Init context before doing anything else
	if (useDepolarization) {
		
		//LPFlatMeshReflectionSimulation* sim = new LPFlatMeshReflectionSimulation(sceneManager);
		LPCurvedFlatMeshReflectionSimulation* sim = new LPCurvedFlatMeshReflectionSimulation(sceneManager);
		sceneManager->setSimulation(sim);
	} else {
		BasicFlatMeshReflectionSimulation* sim = new BasicFlatMeshReflectionSimulation(sceneManager);
		sceneManager->setSimulation(sim);
	}
	sceneManager->initContext(freq);
	sceneManager->getSimulation()->setPrintHits(true);	
	
	
	//Horizontal plane
	//std::vector<int> planeind = sceneManager->loadTrianglesFromFile("meshes/tri.txt");
	//std::vector<float3> planever = sceneManager->loadVerticesFromFile("meshes/vert.txt");
	////std::cout << "indices=" << planeind.size() << "vertices=" << planever.size() << std::endl;
	//Matrix4x4 tm;
	//tm.setRow(0, make_float4(10.0f, 0, 0, 0.0f));
	//tm.setRow(1, make_float4(0, 1, 0, 0.0f));
	//tm.setRow(2, make_float4(0, 0, 10.0f, 50.0f));
	//tm.setRow(3, make_float4(0, 0, 0, 1));
	//sceneManager->addStaticMesh(static_cast<int>(planever.size()), planever.data(), static_cast<int>(planeind.size()), planeind.data(), tm, emProp1);

	//Horizontal plane as quad at origin 
	int quadind[6] = { 0,1,2,1,0,3 };
	optix::float3 quadh[4] = { make_float3(-0.5f,0.0f,-0.5f),make_float3(0.5f,0.f,0.5f) ,make_float3(0.5f,0.f,-0.5f) ,make_float3(-0.5f,0.0f,0.5f) };

	//Scale 200x200
	Matrix4x4 tm;
	tm.setRow(0, make_float4(200, 0, 0, 0.f));
	tm.setRow(1, make_float4(0, 1, 0, 0.f));
	tm.setRow(2, make_float4(0, 0, 200, 0.f));
	tm.setRow(3, make_float4(0, 0, 0,  1));


	MaterialEMProperties emProp1= sceneManager->ITUparametersToMaterial(3.75f,0.0f,0.038f,0.0f);
	//MaterialEMProperties emProp1;
	//emProp1.dielectricConstant = make_float2(3.75f, -60.0f*sceneManager->getChannelParameters().waveLength*0.038f);
		//There is a dependency on the frequency again, we use -15 dB per 203 mm at 5 GHz => -75 dB/m
	emProp1.tattenuation = make_float2(0.1f,-75.f );
	sceneManager->addStaticMesh(4, quadh, 6, quadind, tm, emProp1 );


	timer.start();
	if (mode==0) {
		sceneManager->createRaySphere2DSubstep(1, 1);
	} else {
		uint rayD=10000u;
		OpalRaySphereGenerator* gen=sceneManager->getRaySphereGenerator();
		gen->generateRandomUniformSphereOnDevice(rayD*rayD);
		sceneManager->createRaySphereFromExternalBuffer(rayD,rayD,gen->getDevicePointer());
	}
//	sceneManager->createRaySphere2D(1, 1);


	//receivers
	optix::float3 posrx = make_float3(0.0f, 2.0f, 100.0f);
	//optix::float3 posrx2 = make_float3(10.0f, 10.0f, 100.0f);


	optix::float3 postx = make_float3(0.0f, 10.0f, 0.0f);
	optix::float3 polarization = make_float3(0.0f, 1.0f, 0.0f); //Perpendicular to the floor. Assuming as in Unity that forward is z-axis and up is y-axis

	sceneManager->addReceiver(1, posrx,polarization, sphereRadius, sceneManager->printPower);
	int nrx=100;
	for (int i=1;i<=nrx;++i) {
		//sceneManager->addReceiver(i,make_float3(3.72f,2.7f-6.0f, 20.0f),polarization, sphereRadius, sceneManager->printPower);
		//	sceneManager->addReceiver(i,make_float3(0.0,2.0f, 2),polarization, sphereRadius, sceneManager->printPower);
			sceneManager->addReceiver(i,make_float3(0.0,2.0f, i),polarization, sphereRadius, sceneManager->printPower);
	}

	timer.start();
	sceneManager->finishSceneContext();

	//	sceneManager->setUsageReport();


	//std::cout << "Launching" << std::endl;




	//postx = make_float3(0.0f, 2.0f, 35.0f);
	//sceneManager->transmit(0, 1.0f, postx, polarization);
	//postx = make_float3(0.0f, 2.0f, 98.0f);
	//sceneManager->transmit(0, 1.0f, postx, polarization);
	/*postx = make_float3(0.0f, 2.0f, 97.0f);
	  sceneManager->transmit(0, 1.0f, postx, polarization);

	  postx = make_float3(0.0f, 2.0f, 96.0f);
	  sceneManager->transmit(0, 1.0f, postx, polarization);

	  size_t i=4;
	  postx = make_float3(0.0f, 2.0f, 99.0f -i);
	  sceneManager->transmit(0, 1.0f, postx, polarization);
	  */
//	for (size_t i = 0; i < 100; ++i)
//	{
//		postx.z = 99.0f - i;
		sceneManager->transmit(0, 1.0f, postx, polarization);

//	}
	timer.stop();
	std::cout<<"Time="<<timer.getTime()<<std::endl;


}


//Load scenario from files (simila to what is done in veneris-omnet)
void  BasicTests::loadScenario() {
	//A few receivers
	optix::float3 rx0 = make_float3(1547.14f, 40.69f, 620.8f);
	optix::float3 rx1 = make_float3(1501.6, 25.1f, 609.9f);
	optix::float3 rx9 = make_float3(1632.9f, 21.67f, 560.55f);
	optix::float3 rx_e = make_float3(1462.1f, 45.33f, 622.9f);
	optix::float3 tx_m = make_float3(1040.74f, 32.89, 799.6f);
	optix::float3 tx_e = make_float3(1633.4f, 42.3, 679.9f);
	optix::float3 tx_f = make_float3(1520.5f, 26.836, 589.6f);
	optix::float3 rx3 = make_float3(875.22, 4.19f, 386.65f);
	optix::float3 tx3 = make_float3(862.62, 7.04f, 386.09f);
	Timer timer;
	float freq = 868e6f;
	std::cout<<"Load from scenario"<<std::endl;
	timer.start();	
	//Init context before doing anything else
	//sceneManager->enableGenerateRaysOnLaunch();
	sceneManager->setMinEpsilon(1e-2);
	sceneManager->setUseAntennaGain(true);
	ComputeMode mode=ComputeMode::VOLTAGE;
	if (useDepolarization) {
		LPFlatMeshReflectionSimulation* sim = new LPFlatMeshReflectionSimulation(sceneManager);
		sceneManager->setSimulation(sim);
		sim->setComputeMode(mode);
		sim->setEnableTraceLog(true);
		//sim->setEnableSimulation(false);
	} else {
		BasicFlatMeshReflectionSimulation* sim = new BasicFlatMeshReflectionSimulation(sceneManager);
		sceneManager->setSimulation(sim);
		sim->setComputeMode(mode);
	}
//Add diffraction
	SingleDiffraction* simd= new SingleDiffraction(sceneManager);
	sceneManager->setSimulation(simd);
	simd->setComputeMode(mode);
	simd->setEnableTraceLog(true);
	simd->setEnableSimulation(false);
	
	sceneManager->initContext(freq);
//Exceptions
	sceneManager->enableExceptions();	

//Load files here
	ScenarioLoader* loader=new ScenarioLoader(sceneManager);
        //std::string path("lora/cartagena.json");
        //std::string path("cartagena2.json");
        //std::string path("eldi.json");
        std::string path("cartagena-catastro.json");
	loader->loadJSONScenario(path);
        //std::string path("meshes/cartagena");
	//loader->loadMeshesFromFiles(path);
	//loader->loadEdgesFromFiles(path);

	optix::float3 polarization = make_float3(0.0f, 1.0f, 0.0f); //Hard
//	optix::float3 posrx = make_float3(1547.14f, 40.69f, 620.8f);
//	float3 posrx = make_float3(1501.6, 25.1f, 609.9f);
	//float3 posrx=rx_e;
	float3 posrx=rx3;
	sceneManager->addReceiver(0, posrx,polarization, sphereRadius, sceneManager->printPower);
	
	//AntennaGain gains=sceneManager->loadGainsFromFileIndBPower("lora/gananciasAntena.txt");
	//int gainId=sceneManager->registerAntennaGain(gains);
	//sceneManager->registerReceiverGain(0,gainId);
	//***Single ray transmit****
	//float3 mRay=normalize(make_float3(-0.004128737841, -0.9902680516, 0.1391119212));
	//sceneManager->createRaySphere2D(1,1,&mRay);
	
	sceneManager->finishSceneContext();

////Uncomment for flat
//
	sceneManager->createRaySphere2D(0.0f,0.1,180.0f,0.0f,0.1,360.0f);
//	//float3 postx = make_float3(1501.6, 25.1f, 609.9f);
//	//optix::float3 postx = make_float3(1547.14f, 40.69f, 620.8f);
//	optix::float3 postx = tx_f;
	optix::float3 postx = tx3;
	sceneManager->transmit(1, 0.0251187f, postx, polarization);

//Interchange
//	posrx=postx;
//	sceneManager->updateReceiver(0,posrx);
//	postx=rx1;
//	sceneManager->transmit(1, 0.0251187f, postx, polarization);
//
	timer.stop();
	std::cout<<"Time="<<timer.getTime()<<std::endl;
}
//Adding compund dynamic meshes
void  BasicTests::addCompoundDynamicMeshes() {
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


void BasicTests::freeSpace() {
	//A free space to test that the filtering procedure works
	Timer timer;
	float freq = 5.9e9f;
	std::cout<<"Running free space test"<<std::endl;
	
	//Init context before doing anything else
	if (useDepolarization) {
		LPFlatMeshReflectionSimulation* sim = new LPFlatMeshReflectionSimulation(sceneManager);
		sceneManager->setSimulation(sim);
	} else {
		BasicFlatMeshReflectionSimulation* sim = new BasicFlatMeshReflectionSimulation(sceneManager);
		sceneManager->setSimulation(sim);
	}
	

	
	sceneManager->initContext(freq);
	sceneManager->getSimulation()->setPrintHits(true);	
	timer.start();
	optix::float3 postx = make_float3(0.0f, 10.0f, 0.0f);
	optix::float3 polarizationTx = make_float3(0.0f, 1.0f, 0.0f); //Perpendicular to the floor. Assuming as in Unity that forward is z-axis and up is y-axis
	optix::float3 polarization = make_float3(0.0f, 1.0f, 0.0f); //Perpendicular to the floor. Assuming as in Unity that forward is z-axis and up is y-axis
	optix::float3 posrx = make_float3(0.0f, 10.0f, 10.0f);
	sceneManager->addReceiver(1, posrx, polarization, sphereRadius, sceneManager->printPower);

	sceneManager->createRaySphere2DSubstep(1, 1); //0.1 degree delta step
	

	//***Single ray transmit****
	//float3 mRay=normalize(make_float3(0.0,0.0,1));
	//sceneManager->createRaySphere2D(1,1,&mRay);
	
	sceneManager->finishSceneContext();
	sceneManager->transmit(0, 1.0f, postx, polarizationTx, false);
	timer.stop();
	std::cout<<"Time="<<timer.getTime()<<std::endl;
	//		postx = make_float3(-18.0f, 10.0f, 50.0f);
	//		sceneManager->transmit(0, 1.0f, postx, polarization);

}
//Two quads as walls and two overlapping receivers
void BasicTests::quadTest( bool print, bool subSteps) {
	float freq=5.9e9;	
	LPCurvedFlatMeshReflectionSimulation* sim= new LPCurvedFlatMeshReflectionSimulation(sceneManager);
	sceneManager->setSimulation(sim);
	sceneManager->initContext(freq);
	//First quad
	int quadind[6] = { 0,1,2,1,0,3 };
	optix::float3 quadv[4] = { make_float3(-0.5f,-0.5f,0.f),make_float3(0.5f,0.5f,0.5f) ,make_float3(0.5f,-0.5f,0.0f) ,make_float3(-0.5f,0.5f,0.5f) };

	//One quad at (0,0,100)

	Matrix4x4 tm;
	tm.setRow(0, make_float4(1, 0, 0, 0.f));
	tm.setRow(1, make_float4(0, 1, 0, 0.f));
	tm.setRow(2, make_float4(0, 0, 1, 100.0f));
	tm.setRow(3, make_float4(0, 0, 0,  1));
	MaterialEMProperties emProp1;
		//There is a dependency on the frequency again, we use -15 dB per 203 mm at 5 GHz => -75 dB/m
		emProp1.tattenuation = make_float2(0.1f,-75.f );
	emProp1.dielectricConstant = make_float2(3.75f, -60.0f*sceneManager->getChannelParameters().waveLength*0.038f);

	sceneManager->addStaticMesh(4, quadv, 6, quadind, tm, emProp1 );

	//Second quad at (0,0,-10)
	int quadind2[6] = { 0,1,2,1,0,3 };
	optix::float3 quadv2[4] = { make_float3(-0.5f,-0.5f,0.f),make_float3(0.5f,0.5f,0.f) ,make_float3(0.5f,-0.5f,0.f) ,make_float3(-0.5f,0.5f,0.f) };


	Matrix4x4 tm2;
	tm2.setRow(0, make_float4(1, 0, 0, 0.f));
	tm2.setRow(1, make_float4(0, 1, 0, 0.f));
	tm2.setRow(2, make_float4(0, 0, 1, -10.0f));
	tm2.setRow(3, make_float4(0, 0, 0, 1));
	MaterialEMProperties emProp2;
	emProp2.dielectricConstant = make_float2(3.75f, -60.0f*sceneManager->getChannelParameters().waveLength*0.038f);


	//sceneManager->addStaticMesh(4, quadv2, 6, quadind2, tm2, emProp2);
	
	optix::float3 polarization = make_float3(0.0f, 1.0f, 0.0f); //Perpendicular to the floor. Assuming as in Unity that forward is z-axis and up is y-axis


	optix::float3 posrx = make_float3(0.f, 0.f, 97.0f);
	//sceneManager->addReceiver(1, posrx,polarization, 1.0f, sceneManager->printPower);
	sceneManager->addReceiver(1, posrx,polarization, 5.0f, sceneManager->printPower);
	posrx=make_float3(0.0f,0.0f,99.0f);
	sceneManager->addReceiver(2, posrx, polarization, 5.0f, sceneManager->printPower);
	optix::float3 postx = make_float3(0.0f, 0.f,0.f);

	if (subSteps) {
		sceneManager->createRaySphere2DSubstep(1, 1);
	} else {
		sceneManager->createRaySphere2D(1, 1);
	}
	//sceneManager->createRaySphere2D(30, 30); //1 degree delta step

	sceneManager->finishSceneContext();

	if (print) {
		sceneManager->setPrintEnabled(1024 * 1024 * 1024);
	}
	//sceneManager->setUsageReport();
	sceneManager->transmit(0, 1.0f,postx, polarization);
}


////Street crossing test. Cubes are intended to be buildings and a plane is the floor. A complex vehicle mesh is moved
//std::unique_ptr<OpalSceneManager> crossingTestAndVehicle(std::unique_ptr<OpalSceneManager> sceneManager) {
//	//Cubes
//	std::vector<int> cubeind = sceneManager->loadTrianglesFromFile("meshes/tricube.txt");
//	std::vector<float3> cubevert = sceneManager->loadVerticesFromFile("meshes/vertcube.txt");
//	//std::cout << "indices=" << cubeind.size() << "vertices=" << cubevert.size() << std::endl;
//	//Cube(4) NW
//	Matrix4x4 tm;
//	tm.setRow(0, make_float4(40.0f, 0, 0, -30.0f));
//	tm.setRow(1, make_float4(0, 40.0f, 0, 20.0f));
//	tm.setRow(2, make_float4(0, 0, 40.0f, 80.0f));
//	tm.setRow(3, make_float4(0, 0, 0, 1));
//	MaterialEMProperties emProp1;
//	emProp1.dielectricConstant = make_float2(3.75f, -60.0f*sceneManager->getChannelParameters().waveLength*0.038f);
//		//There is a dependency on the frequency again, we use -15 dB per 203 mm at 5 GHz => -75 dB/m
//		emProp1.tattenuation = make_float2(0.1f,-75.f );
//	//emProp1.dielectricConstant = make_float2(3.75f, -0.4576f);
//	std::cout << "Adding NW. Em=" << emProp1.dielectricConstant << std::endl;
//	sceneManager->addStaticMesh(static_cast<int>(cubevert.size()), cubevert.data(), static_cast<int>(cubeind.size()), cubeind.data(), tm, emProp1);
//
//	//Cube SW
//	tm.setRow(0, make_float4(40.0f, 0, 0, -30.0f));
//	tm.setRow(1, make_float4(0, 40.0f, 0, 20.0f));
//	tm.setRow(2, make_float4(0, 0, 40.0f, 20.0f));
//	tm.setRow(3, make_float4(0, 0, 0, 1));
//	std::cout << "Adding SW. Em = " << emProp1.dielectricConstant << std::endl;
//	sceneManager->addStaticMesh(static_cast<int>(cubevert.size()), cubevert.data(), static_cast<int>(cubeind.size()), cubeind.data(), tm, emProp1);
//	//Cube(2) NE
//
//	tm.setRow(0, make_float4(40.0f, 0, 0, 30.0f));
//	tm.setRow(1, make_float4(0, 40.0f, 0, 20.0f));
//	tm.setRow(2, make_float4(0, 0, 40.0f, 80.0f));
//	tm.setRow(3, make_float4(0, 0, 0, 1));
//	std::cout << "Adding NE. Em = " << emProp1.dielectricConstant << std::endl;
//	sceneManager->addStaticMesh(static_cast<int>(cubevert.size()), cubevert.data(), static_cast<int>(cubeind.size()), cubeind.data(), tm, emProp1);
//
//	//Cube(1) SE
//
//	tm.setRow(0, make_float4(40.0f, 0, 0, 30.0f));
//	tm.setRow(1, make_float4(0, 40.0f, 0, 20.0f));
//	tm.setRow(2, make_float4(0, 0, 40.0f, 20.0f));
//	tm.setRow(3, make_float4(0, 0, 0, 1));
//	std::cout << "Adding SE. Em = " << emProp1.dielectricConstant << std::endl;
//	sceneManager->addStaticMesh(static_cast<int>(cubevert.size()), cubevert.data(), static_cast<int>(cubeind.size()), cubeind.data(), tm, emProp1);
//
//	//Horizontal plane
//	std::vector<int> planeind = sceneManager->loadTrianglesFromFile("meshes/tri.txt");
//	std::vector<float3> planever = sceneManager->loadVerticesFromFile("meshes/vert.txt");
//	//std::cout << "indices=" << planeind.size() << "vertices=" << planever.size() << std::endl;
//
//	tm.setRow(0, make_float4(10.0f, 0, 0, 0.0f));
//	tm.setRow(1, make_float4(0, 1, 0, 0.0f));
//	tm.setRow(2, make_float4(0, 0, 10.0f, 50.0f));
//	tm.setRow(3, make_float4(0, 0, 0, 1));
//
//	//emProp1.dielectricConstant = make_float2(3.75f, -60.0f*sceneManager->defaultChannel.waveLength*0.15f);
//	std::cout << "Adding Plane. Em=" << emProp1.dielectricConstant << std::endl;
//	sceneManager->addStaticMesh(static_cast<int>(planever.size()), planever.data(), static_cast<int>(planeind.size()), planeind.data(), tm, emProp1);
//
//
//
//	//Create vehicle group
//	std::vector<int> bodyi = sceneManager->loadTrianglesFromFile("meshes/CC_ME_Body_R4-i.txt");
//	std::vector<float3> bodyv = sceneManager->loadVerticesFromFile("meshes/CC_ME_Body_R4-v.txt");
//	std::vector<int> wheeli = sceneManager->loadTrianglesFromFile("meshes/CC_ME_Wheel_FL-i.txt");
//	std::vector<float3> wheelv = sceneManager->loadVerticesFromFile("meshes/CC_ME_Wheel_FL-v.txt");
//
//
//	//Creation of dynamic meshes  requires calling these 4 functions
//	sceneManager->addDynamicMeshGroup(0);
//	sceneManager->addMeshToGroup(0, static_cast<int>(bodyv.size()), bodyv.data(), static_cast<int>(bodyi.size()),bodyi.data(), emProp1);  //Call for each new mesh in the group
//	sceneManager->addMeshToGroup(0, static_cast<int>(wheelv.size()), wheelv.data(), static_cast<int>(wheeli.size()), wheeli.data(),emProp1);  //Call for each new mesh in the group
//	wheeli = sceneManager->loadTrianglesFromFile("meshes/CC_ME_Wheel_FR-i.txt");
//	wheelv = sceneManager->loadVerticesFromFile("meshes/CC_ME_Wheel_FR-v.txt");
//	sceneManager->addMeshToGroup(0, static_cast<int>(wheelv.size()), wheelv.data(), static_cast<int>(wheeli.size()), wheeli.data(), emProp1);  //Call for each new mesh in the group
//
//	wheeli = sceneManager->loadTrianglesFromFile("meshes/CC_ME_Wheel_BL-i.txt");
//	wheelv = sceneManager->loadVerticesFromFile("meshes/CC_ME_Wheel_BL-v.txt");
//	sceneManager->addMeshToGroup(0, static_cast<int>(wheelv.size()), wheelv.data(), static_cast<int>(wheeli.size()), wheeli.data(), emProp1);  //Call for each new mesh in the group
//	wheeli = sceneManager->loadTrianglesFromFile("meshes/CC_ME_Wheel_BR-i.txt");
//	wheelv = sceneManager->loadVerticesFromFile("meshes/CC_ME_Wheel_BR-v.txt");
//	sceneManager->addMeshToGroup(0, static_cast<int>(wheelv.size()), wheelv.data(), static_cast<int>(wheeli.size()), wheeli.data(), emProp1);  //Call for each new mesh in the group
//
//
//	tm.setRow(0, make_float4(0.0f, 0.f, 1.0f, -50.0f));
//	tm.setRow(1, make_float4(0.f, 1.0f, 0.f, 0.6f));
//	tm.setRow(2, make_float4(-1.f, 0.f, 0.0f, 50.0f));
//	tm.setRow(3, make_float4(0, 0, 0, 1));
//
//	/*
//	   tm.setRow(0, make_float4(0.0f, 0.f, 0.f, -50.0f));
//	   tm.setRow(1, make_float4(0.f, 1.0f, 0.f, 0.6f));
//	   tm.setRow(2, make_float4(-1.f, 0.f, 0.0f, 50.0f));
//	   tm.setRow(3, make_float4(0, 0, 0, 1));
//
//*/
//	sceneManager->updateTransformInGroup(0, tm);
//	sceneManager->finishDynamicMeshGroup(0);
//
//
//
//	//sceneManager->createRaySphere2D(1, 1);
//	sceneManager->createRaySphere2DSubstep(1, 1);
//
//	//receivers
//
//	optix::float3 posrx = make_float3(0.0f, 2.0f, 100.0f);
//	optix::float3 polarization = make_float3(0.0f, 1.0f, 0.0f); //Perpendicular to the floor. Assuming as in Unity that forward is z-axis and up is y-axis
//	sceneManager->addReceiver(1, posrx,polarization, 1.f, sceneManager->printPower);
//
//
//	//sceneManager->setMaxReflections(3u);
//
//	sceneManager->finishSceneContext();
//
//
//	sceneManager->setPrintEnabled(1024 * 1024 * 1024);
//
//
//	optix::float3 postx;
//
//	for (int i = -50; i <= 50; ++i)
//	{
//		postx = make_float3(i-2, 1.1f, 50.0f);//On top of the vehicle
//		tm.setRow(0, make_float4(-1.19209e-07f, 0.f, 1.0f, i));
//		tm.setRow(2, make_float4(-1.f, 0.f, -1.19209e-07f, 50.0f));
//		//std::cout << "tm=" << tm << std::endl;
//		sceneManager->updateTransformInGroup(0, tm);
//		sceneManager->transmit(0, 1.0f, postx, polarization);
//
//
//	}
//	//postx = make_float3(-50.0f, 1.43f, 50.0f); //On top of the vehicle
//	//postx = make_float3(-50.0f, 3.f, 50.0f); //On top of the vehicle
//	//sceneManager->transmit(0, 1.0f, postx, polarization);
//
//	return sceneManager;
//
//}
//
//
////Penetration test. One cube, transmitter and receiver
//std::unique_ptr<OpalSceneManager> penetrationTest(std::unique_ptr<OpalSceneManager> sceneManager, bool print, bool subSteps) {
//
//	Timer timer;
//
//	std::cout << "Penetration test" << std::endl;
//	//Cubes
//	std::vector<int> cubeind = sceneManager->loadTrianglesFromFile("meshes/tricube.txt");
//	std::vector<float3> cubevert = sceneManager->loadVerticesFromFile("meshes/vertcube.txt");
//	//std::cout << "indices=" << cubeind.size() << "vertices=" << cubevert.size() << std::endl;
//	//Cube(4) NW
//	Matrix4x4 tm;
//	tm.setRow(0, make_float4(40.0f, 0, 0, -30.0f));
//	tm.setRow(1, make_float4(0, 40.0f, 0, 20.0f));
//	tm.setRow(2, make_float4(0, 0, 40.0f, 80.0f));
//	tm.setRow(3, make_float4(0, 0, 0, 1));
//	MaterialEMProperties emProp1;
//	emProp1.dielectricConstant = make_float2(3.75f, -60.0f*sceneManager->getChannelParameters().waveLength*0.038f);
//		//There is a dependency on the frequency again, we use -15 dB per 203 mm at 5 GHz => -75 dB/m
//		emProp1.tattenuation = make_float2(0.1f,-75.f );
//	//emProp1.dielectricConstant = make_float2(3.75f, -0.4576f);
//	std::cout << "Adding NW. Em="<< emProp1.dielectricConstant << std::endl;
//	sceneManager->addStaticMesh(static_cast<int>(cubevert.size()), cubevert.data(), static_cast<int>(cubeind.size()), cubeind.data(), tm, emProp1);
//
//
//	sceneManager->createRaySphere2D(30,60);	
////	if (subSteps) {
////		sceneManager->createRaySphere2DSubstep(1, 1); //0.1 degree delta step
////	} else {
////		sceneManager->createRaySphere2D(1, 1); //1 degree delta step
////	}
//
//	//receivers
//
//	optix::float3 posrx = make_float3(-8.48f,10.0f, 78.0856f); //Hit with 60 degrees ray reflected on cube
//	optix::float3 polarization = make_float3(0.0f, 1.0f, 0.0f); //Perpendicular to the floor. Assuming as in Unity that forward is z-axis and up is y-axis
//	sceneManager->addReceiver(1, posrx, polarization, 5.0f, sceneManager->printPower);
//
//
//	sceneManager->enablePenetration();
//	sceneManager->finishSceneContext();
//
//	if (print) {
//		sceneManager->setPrintEnabled(1024 * 1024 * 1024);	
//	}
//	//sceneManager->setUsageReport();
//
//	optix::float3 postx;
//	timer.start();
//			postx = make_float3(-50.0f, 10.0f, 50.0f);
//			sceneManager->transmit(0, 1.0f, postx, polarization);
//
//	timer.stop();
//	std::cout<<"Time="<<timer.getTime()<<std::endl;
//
//	return sceneManager;
//
//}
//
//std::unique_ptr<OpalSceneManager> penetrationPlane(std::unique_ptr<OpalSceneManager> sceneManager, bool print, bool subSteps) {
//
//	Timer timer;
//
//	std::cout << "Simulating penetration through plane" << std::endl;
//	//Cubes
//	std::vector<int> cubeind = sceneManager->loadTrianglesFromFile("meshes/tricube.txt");
//	std::vector<float3> cubevert = sceneManager->loadVerticesFromFile("meshes/vertcube.txt");
////	//std::cout << "indices=" << cubeind.size() << "vertices=" << cubevert.size() << std::endl;
////	//Cube(4) NW
//	Matrix4x4 tm;
//	tm.setRow(0, make_float4(40.0f, 0, 0, -30.0f));
//	tm.setRow(1, make_float4(0, 40.0f, 0, 20.0f));
//	tm.setRow(2, make_float4(0, 0, 40.0f, 80.0f));
//	tm.setRow(3, make_float4(0, 0, 0, 1));
//	MaterialEMProperties emProp1;
//	emProp1.dielectricConstant = make_float2(3.75f, -60.0f*sceneManager->getChannelParameters().waveLength*0.038f);
//		//There is a dependency on the frequency again, we use -15 dB per 203 mm at 5 GHz => -75 dB/m
//	emProp1.tattenuation = make_float2(0.1f,-15.f );
//	//emProp1.dielectricConstant = make_float2(3.75f, -0.4576f);
//	std::cout << "Adding NW. Em="<< emProp1.dielectricConstant << std::endl;
//	sceneManager->addStaticMesh(static_cast<int>(cubevert.size()), cubevert.data(), static_cast<int>(cubeind.size()), cubeind.data(), tm, emProp1);
////
////	//Cube SW
////	tm.setRow(0, make_float4(40.0f, 0, 0, -30.0f));
////	tm.setRow(1, make_float4(0, 40.0f, 0, 20.0f));
////	tm.setRow(2, make_float4(0, 0, 40.0f, 20.0f));
////	tm.setRow(3, make_float4(0, 0, 0, 1));
////	std::cout << "Adding SW. Em = "<< emProp1.dielectricConstant << std::endl;
////	sceneManager->addStaticMesh(static_cast<int>(cubevert.size()), cubevert.data(), static_cast<int>(cubeind.size()), cubeind.data(), tm, emProp1);
////	//Cube(2) NE
////
////	tm.setRow(0, make_float4(40.0f, 0, 0, 30.0f));
////	tm.setRow(1, make_float4(0, 40.0f, 0, 20.0f));
////	tm.setRow(2, make_float4(0, 0, 40.0f, 80.0f));
////	tm.setRow(3, make_float4(0, 0, 0, 1));
////	std::cout << "Adding NE. Em = "<< emProp1.dielectricConstant << std::endl;
////	sceneManager->addStaticMesh(static_cast<int>(cubevert.size()), cubevert.data(), static_cast<int>(cubeind.size()), cubeind.data(), tm, emProp1);
////
////	//Cube(1) SE
////
////	tm.setRow(0, make_float4(40.0f, 0, 0, 30.0f));
////	tm.setRow(1, make_float4(0, 40.0f, 0, 20.0f));
////	tm.setRow(2, make_float4(0, 0, 40.0f, 20.0f));
////	tm.setRow(3, make_float4(0, 0, 0, 1));
////	std::cout << "Adding SE. Em = "<< emProp1.dielectricConstant << std::endl;
////	sceneManager->addStaticMesh(static_cast<int>(cubevert.size()), cubevert.data(), static_cast<int>(cubeind.size()), cubeind.data(), tm, emProp1);
////
//	//Horizontal plane as quad at origin 
//	//int quadind[6] = { 0,1,2,1,0,3 };
//	//optix::float3 quadh[4] = { make_float3(-0.5f,0.0f,-0.5f),make_float3(0.5f,0.f,0.5f) ,make_float3(0.5f,0.f,-0.5f) ,make_float3(-0.5f,0.0f,0.5f) };
//
////	//Scale 100x100
////	Matrix4x4 tm;
////	tm.setRow(0, make_float4(100, 0, 0, 0.f));
////	tm.setRow(1, make_float4(0, 1, 0, 0.f));
////	tm.setRow(2, make_float4(0, 0, 100, 0.f));
////	tm.setRow(3, make_float4(0, 0, 0,  1));
//
///**********/	
////	std::vector<int> planeind = sceneManager->loadTrianglesFromFile("meshes/tri.txt");
////	std::vector<float3> planever = sceneManager->loadVerticesFromFile("meshes/vert.txt");
////	std::cout << "indices=" << planeind.size() << "vertices=" << planever.size() << std::endl;
////
////	tm.setRow(0, make_float4(10.0f, 0, 0, 0.0f));
////	tm.setRow(1, make_float4(0, 1, 0, 0.0f));
////	tm.setRow(2, make_float4(0, 0, 10.0f, 50.0f));
////	tm.setRow(3, make_float4(0, 0, 0, 1));
////
////	std::cout << "Adding Plane. Em.dielectric=" << emProp1.dielectricConstant << std::endl;
////	std::cout << "Adding Plane. Em.attenuation=" << emProp1.tattenuation << std::endl;
////	sceneManager->addStaticMesh(static_cast<int>(planever.size()), planever.data(), static_cast<int>(planeind.size()), planeind.data(), tm, emProp1);
////	sceneManager->addStaticMesh(4, quadh, 6, quadind, tm, emProp1 );
//
//	if (subSteps) {
//		sceneManager->createRaySphere2DSubstep(1, 1); //0.1 degree delta step
//	} else {
//		sceneManager->createRaySphere2D(1, 1); //1 degree delta step
//	}
//
//	//receivers
//
//	optix::float3 posrx = make_float3(0.0f, 10.0f, 100.0f);
//	optix::float3 polarization = make_float3(0.0f, 1.0f, 0.0f); //Parallel to the floor. Assuming as in Unity that forward is z-axis and up is y-axis
//	sceneManager->addReceiver(1, posrx,polarization, 5.0f, sceneManager->printPower);
//
//
//	//sceneManager->setMaxReflections(3u);
//
//	sceneManager->finishSceneContext();
//
//	if (print) {
//		sceneManager->setPrintEnabled(1024 * 1024 * 1024);	
//	}
//	//sceneManager->setUsageReport();
//
//	optix::float3 postx;
//	timer.start();
//
////	for (int i = -50; i <= 50; ++i) {
////
////		float x=i;
////		postx = make_float3(x, 10.f, 50.0f);
////
////		sceneManager->transmit(0, 1.0f, postx, polarization);
////
////
////	}
//	timer.stop();
//	std::cout<<"Time="<<timer.getTime()<<std::endl;
//			postx = make_float3(-40.0f, 10.0f, 50.0f);
//			sceneManager->transmit(0, 1.0f, postx, polarization);
//
//	return sceneManager;
//
//}
//
////Adding, moving and removing dynamic meshes
//std::unique_ptr<OpalSceneManager> addRemoveDynamicMeshes(std::unique_ptr<OpalSceneManager> sceneManager, bool print, bool subSteps) {
//	try {
//
//		BasicFlatMeshReflectionSimulation* sim= new BasicFlatMeshReflectionSimulation(sceneManager.get());
//		sceneManager->setSimulation(sim);
//		//Quad
//		int quadind[6] = { 0,1,2,1,0,3 };
//		optix::float3 quadv[4] = { make_float3(-0.5f,-0.5f,0.f),make_float3(0.5f,0.5f,0.f) ,make_float3(0.5f,-0.5f,0.f) ,make_float3(-0.5f,0.5f,0.f) };
//
//
//
//		Matrix4x4 tm;
//		tm.setRow(0, make_float4(1, 0, 0, 0.f));
//		tm.setRow(1, make_float4(0, 1, 0, 2.f));
//		tm.setRow(2, make_float4(0, 0, 1, 75.f));
//		tm.setRow(3, make_float4(0, 0, 0.f, 1));
//		MaterialEMProperties emProp1;
//		emProp1.dielectricConstant = make_float2(3.75f, -60.0f*sceneManager->getChannelParameters().waveLength*0.038f);
//		//There is a dependency on the frequency again, we use -15 dB per 203 mm at 5 GHz => -75 dB/m
//		emProp1.tattenuation = make_float2(0.1f,-75.f );
//
//		//Creation of dynamic meshes  requires calling these 4 functions
//		sceneManager->addDynamicMeshGroup(0);
//		sceneManager->addMeshToGroup(0,4, quadv, 6, quadind,  emProp1);  //Call for each new mesh in the group
//		sceneManager->updateTransformInGroup(0, tm); 
//		sceneManager->finishDynamicMeshGroup(0);
//
//		if (subSteps) {
//			sceneManager->createRaySphere2DSubstep(1, 1);
//		} else {
//			sceneManager->createRaySphere2D(1, 1);
//		}
//		//receivers
//		optix::float3 posrx = make_float3(0.0f, 2.0f, 50.0f);
//
//
//		optix::float3 postx = make_float3(0.0f, 2.0f, 0.0f);
//
//		optix::float3 polarization = make_float3(0.0f, 1.0f, 0.0f); //Perpendicular to the floor. Assuming as in Unity that forward is z-axis and up is y-axis
//		
//		sceneManager->addReceiver(0, posrx, polarization, 5.0f, sceneManager->printPower);
//
//
//		sceneManager->finishSceneContext();
//		if (print) {
//			sceneManager->setPrintEnabled(1024 * 1024 * 1024);
//		}
//
//		sceneManager->transmit(1, 1.0f, postx, polarization);
//
//		Matrix4x4 tm1;
//		tm1.setRow(0, make_float4(1.f, 0, 0, 0.0f));
//		tm1.setRow(1, make_float4(0, 1, 0, 2.f));
//		tm1.setRow(2, make_float4(0, 0, 1.0f, -75.f));
//		tm1.setRow(3, make_float4(0, 0, 0.f, 1));
//		sceneManager->updateTransformInGroup(0, tm1);
//
//		posrx = make_float3(0.0f, 2.0f, -50.0f);
//		sceneManager->updateReceiver(0, posrx);
//
//		std::cout << "Symmetric situation if everything has transformed well. Expect the same power as first transmission. Transmit again" << std::endl;
//		sceneManager->transmit(1, 1.0f, postx, polarization);
//
//		//Add a new quad 
//		sceneManager->addDynamicMeshGroup(1);
//		sceneManager->addMeshToGroup(1, 4, quadv, 6, quadind, emProp1);
//		sceneManager->updateTransformInGroup(1, tm);
//		sceneManager->finishDynamicMeshGroup(1);
//
//		std::cout << "Transmit with new quad. Num quads= "<< sceneManager->getDynamicMeshes().size() << std::endl;
//		sceneManager->transmit(1, 1.0f, postx, polarization);
//
//		//Remove first quad
//		sceneManager->removeDynamicMeshGroup(0);
//
//		posrx = make_float3(0.0f, 2.0f, 50.0f);
//		sceneManager->updateReceiver(0, posrx);
//		std::cout << "Removing first quad. Expect again the first power. Transmit again.  Num quads= " << sceneManager->getDynamicMeshes().size() << std::endl;
//		Matrix4x4 mym;
//		Matrix4x4 mymi;
//		sceneManager->getDynamicMeshes().at(1)->transform->getMatrix(0, mym.getData(), mymi.getData());
//		std::cout << "Tm of quad 1: " <<  mym<< std::endl;
//		sceneManager->transmit(1, 1.0f, postx, polarization);
//
//		//Remove second quad
//		sceneManager->removeDynamicMeshGroup(1);
//
//		return sceneManager;
//	}
//	catch (optix::Exception& e) {
//		std::cout << "addRemoveDynamicMeshes occurred with error code "
//			<< e.getErrorCode() << " and message "
//			<< e.getErrorString() << std::endl;
//
//		return 0;
//	}
//	catch (opal::Exception& e) {
//		std::cout << "addRemoveDynamicMeshes occurred with  message "
//			<< e.getErrorString()
//			<< std::endl;
//
//		return 0;
//	}
//
//
//}
//
//
////Adding and removing dynamic meshes
//std::unique_ptr<OpalSceneManager> addRemoveReceivers(std::unique_ptr<OpalSceneManager> sceneManager) {
//	try {
//		BasicFlatMeshReflectionSimulation* sim= new BasicFlatMeshReflectionSimulation(sceneManager.get());
//		sceneManager->setSimulation(sim);
//		//Horizontal plane
//		std::vector<int> planeind = sceneManager->loadTrianglesFromFile("meshes/tri.txt");
//		std::vector<float3> planever = sceneManager->loadVerticesFromFile("meshes/vert.txt");
//		//std::cout << "indices=" << planeind.size() << "vertices=" << planever.size() << std::endl;
//		Matrix4x4 tm;
//		tm.setRow(0, make_float4(10.0f, 0, 0, 0.0f));
//		tm.setRow(1, make_float4(0, 1, 0, 0.0f));
//		tm.setRow(2, make_float4(0, 0, 10.0f, 50.0f));
//		tm.setRow(3, make_float4(0, 0, 0, 1));
//		MaterialEMProperties emProp1;
//		emProp1.dielectricConstant = make_float2(3.75f, -60.0f*sceneManager->getChannelParameters().waveLength*0.038f);
//		//There is a dependency on the frequency again, we use -15 dB per 203 mm at 5 GHz => -75 dB/m
//		emProp1.tattenuation = make_float2(0.1f,-75.f );
//		sceneManager->addStaticMesh(static_cast<int>(planever.size()), planever.data(), static_cast<int>(planeind.size()), planeind.data(), tm, emProp1);
//
//
//		sceneManager->createRaySphere2D(1, 1);
//		//sceneManager->createRaySphere2DSubstep(1, 1);
//		//receivers
//		optix::float3 posrx = make_float3(0.0f, 2.0f, 100.0f);
//
//
//		optix::float3 postx = make_float3(0.0f, 2.0f, 50.0f);
//
//		optix::float3 polarization = make_float3(0.0f, 1.0f, 0.0f); //Perpendicular to the floor. Assuming as in Unity that forward is z-axis and up is y-axis
//		
//		sceneManager->addReceiver(1, posrx, polarization, 5.0f, sceneManager->printPower);
//
//
//		sceneManager->finishSceneContext();
//		sceneManager->setPrintEnabled(1024 * 1024 * 1024);
//		sceneManager->transmit(0, 1.0f, postx, polarization);
//
//		//Add new receiver
//		posrx = make_float3(0.0f, 2.0f, 70.0f);
//		sceneManager->addReceiver(2, posrx, polarization, 5.0f, sceneManager->printPower);
//
//		std::cout << "transmit again" << std::endl;
//		sceneManager->transmit(0, 1.0f, postx, polarization);
//
//
//		//Remove receiver
//		sceneManager->removeReceiver(2);
//		std::cout << "transmit again" << std::endl;
//		sceneManager->transmit(0, 1.0f, postx, polarization);
//		//std::cout << "Launching" << std::endl;
//
//		return sceneManager;
//	}
//	catch (optix::Exception& e) {
//		std::cout << "addRemoveReceivers occurred with error code "
//			<< e.getErrorCode() << " and message "
//			<< e.getErrorString() << std::endl;
//
//		return 0;
//	}
//	catch (opal::Exception& e) {
//		std::cout << "addRemoveReceivers occurred with  message "
//			<< e.getErrorString()
//			<< std::endl;
//
//		return 0;
//	}
//
//
//}
//
//
//
////moving receivers
//std::unique_ptr<OpalSceneManager> moveReceivers(std::unique_ptr<OpalSceneManager> sceneManager, bool useDepolarization) {
//	try {
//	float freq = 5.9e9f;
//
//	
//	//Init context before doing anything else
//	if (useDepolarization) {
//		LPFlatMeshReflectionSimulation* sim = new LPFlatMeshReflectionSimulation(sceneManager.get());
//		sceneManager->setSimulation(sim);
//	} else {
//		BasicFlatMeshReflectionSimulation* sim = new BasicFlatMeshReflectionSimulation(sceneManager.get());
//		sceneManager->setSimulation(sim);
//	}
//	
//	
//	sceneManager->initContext(freq);
//		//Horizontal plane
//		std::vector<int> planeind = sceneManager->loadTrianglesFromFile("D:\\tri.txt");
//		std::vector<float3> planever = sceneManager->loadVerticesFromFile("D:\\vert.txt");
//		//std::cout << "indices=" << planeind.size() << "vertices=" << planever.size() << std::endl;
//		Matrix4x4 tm;
//		tm.setRow(0, make_float4(10.0f, 0, 0, 0.0f));
//		tm.setRow(1, make_float4(0, 1, 0, 0.0f));
//		tm.setRow(2, make_float4(0, 0, 10.0f, 50.0f));
//		tm.setRow(3, make_float4(0, 0, 0, 1));
//		MaterialEMProperties emProp1;
//		emProp1.dielectricConstant = make_float2(3.75f, -60.0f*sceneManager->getChannelParameters().waveLength*0.038f);
//		//There is a dependency on the frequency again, we use -15 dB per 203 mm at 5 GHz => -75 dB/m
//		emProp1.tattenuation = make_float2(0.1f,-75.f );
//		sceneManager->addStaticMesh(static_cast<int>(planever.size()), planever.data(), static_cast<int>(planeind.size()), planeind.data(), tm, emProp1);
//
//
//		//sceneManager->createRaySphere2D(1, 1);
//		sceneManager->createRaySphere2DSubstep(1, 1);
//		//receivers
//		optix::float3 postx = make_float3(0.0f, 2.0f, 100.0f);
//
//
//		optix::float3 posrx = make_float3(0.0f, 2.0f, 0.0f);
//		optix::float3 polarization = make_float3(0.0f, 1.0f, 0.0f); //Perpendicular to the floor. Assuming as in Unity that forward is z-axis and up is y-axis
//
//		sceneManager->addReceiver(1, posrx, polarization, 5.0f, sceneManager->printPower);
//
//
//		sceneManager->finishSceneContext();
//		sceneManager->setPrintEnabled(1024 * 1024 * 1024);
//
//		for (size_t i = 0; i < 100; ++i)
//		{
//			posrx = make_float3(0.0f, 2.0f, 99.0f - i);
//			sceneManager->updateReceiver(1, posrx);
//			//postx = make_float3(0.0f, 2.0f, 0.f);
//			sceneManager->transmit(0, 1.0f, postx, polarization);
//			//	postx = make_float3(0.0f, 2.0f, 1.f);
//			//sceneManager->transmit(0, 1.0f, postx, polarization);
//
//		}
//
//
//
//
//		return sceneManager;
//	}
//	catch (optix::Exception& e) {
//		std::cout << "moveReceivers occurred with error code "
//			<< e.getErrorCode() << " and message "
//			<< e.getErrorString() << std::endl;
//
//		return 0;
//	}
//	catch (opal::Exception& e) {
//		std::cout << "moveReceivers occurred with  message "
//			<< e.getErrorString()
//			<< std::endl;
//
//		return 0;
//	}
//
//
//}
//std::unique_ptr<OpalSceneManager> freeSpaceRDN(std::unique_ptr<OpalSceneManager> sceneManager,  bool useDepolarization, float radius) {
//	//A free space to test that the filtering procedure works
//	Timer timer;
//	float freq = 5.9e9f;
//	std::cout<<"Running free space test"<<std::endl;
//	
//	LPCurvedMeshReflectionSimulation* sim= new LPCurvedMeshReflectionSimulation(sceneManager.get());
//	sceneManager->setSimulation(sim);
//	
//
//	
//	sceneManager->initContext(freq);
//	timer.start();
//	optix::float3 postx = make_float3(0.0f, 10.0f, 0.0f);
//	optix::float3 polarizationTx = make_float3(0.0f, 1.0f, 0.0f); //Perpendicular to the floor. Assuming as in Unity that forward is z-axis and up is y-axis
//	optix::float3 polarization = make_float3(0.0f, 1.0f, 0.0f); //Perpendicular to the floor. Assuming as in Unity that forward is z-axis and up is y-axis
//	optix::float3 posrx = make_float3(0.0f, 10.0f, 10.0f);
//	sceneManager->addReceiver(1, posrx, polarization, radius, sceneManager->printPower);
//	//sceneManager->createRaySphere2DSubstep(1, 1); //0.1 degree delta step
//	//sceneManager->createRaySphere2DSubstep(1, 1); //0.1 degree delta step
//	float deg2rad=M_PIf/180.f;	
//	//sceneManager->createRaySphere2D(70.0f,1.0f,110.0f,-20.0f,1.0f,20.0f);
//	//std::cout<<"dens="<<(sceneManager->getRaySphere().rayCount/((deg2rad*10.0f+deg2rad*10.0f)*(cosf(deg2rad*80.0f)-cosf(deg2rad*100.0f))))<<std::endl;
//	OpalRaySphereGenerator* gen=sceneManager->getRaySphereGenerator();
//	std::vector<float3> rays=gen->generateRandomUniform(80.0f,100.0f,-10.0f,10.0f,1000000);
//	
//	//std::string fileName=std::string("spheres/ru-")+std::to_string(0.0)+std::string("-")+std::to_string(180)+std::string("-")+std::to_string(0.0)+std::string("-")+std::to_string(360)+std::string("-r1e8.txt");
////std::ofstream file(fileName.c_str(),std::ofstream::out);
////
////for (auto f : rays) {
////	file<<f.x<<"\t"<<f.y<<"\t"<<f.z<<std::endl;
////}
////file.close();
//	//std::string fileName=std::string("spheres/ru-")+std::to_string(0.0)+std::string("-")+std::to_string(180)+std::string("-")+std::to_string(0.0)+std::string("-")+std::to_string(360)+std::string("-r1e8.bin");
//
//// auto myfile = std::fstream(fileName.c_str(), std::ios::out | std::ios::binary);
////    myfile.write((char*)rays.data(), rays.size()*sizeof(float3));
////    myfile.close();
//	//std::vector<float3> rays=sceneManager->loadRaysFromFile(fileName.c_str());
//	sceneManager->createRaySphere2D(1000,1000,rays.data());
//	//float dens=1801.0f*3600.0f/(4.0f*M_PIf);
//	//float dens=1000000.0f/(4*M_PI);
//	float dens=(sceneManager->getRaySphere().rayCount/((deg2rad*10.0f+deg2rad*10.0f)*(cosf(deg2rad*80.0f)-cosf(deg2rad*100.0f))));
//	std::cout<<"initialDensity="<<dens<<std::endl;
//	//sim->setInitialDensity(dens);
//	sceneManager->finishSceneContext();
//	sceneManager->transmit(0, 1.0f, postx, polarizationTx, false);
//	timer.stop();
//	std::cout<<"Time="<<timer.getTime()<<std::endl;
//	//		postx = make_float3(-18.0f, 10.0f, 50.0f);
//	//		sceneManager->transmit(0, 1.0f, postx, polarization);
//
//	return sceneManager;
//}
//
//
//std::unique_ptr<OpalSceneManager> freeSpace(std::unique_ptr<OpalSceneManager> sceneManager,  bool useDepolarization, float radius) {
//	//A free space to test that the filtering procedure works
//	Timer timer;
//	float freq = 5.9e9f;
//	std::cout<<"Running free space test"<<std::endl;
//	
//	//Init context before doing anything else
//	if (useDepolarization) {
//		LPFlatMeshReflectionSimulation* sim = new LPFlatMeshReflectionSimulation(sceneManager.get());
//		sceneManager->setSimulation(sim);
//	} else {
//		BasicFlatMeshReflectionSimulation* sim = new BasicFlatMeshReflectionSimulation(sceneManager.get());
//		sceneManager->setSimulation(sim);
//	}
//	
//
//	
//	sceneManager->initContext(freq);
//	timer.start();
//	optix::float3 postx = make_float3(0.0f, 10.0f, 0.0f);
//	optix::float3 polarizationTx = make_float3(0.0f, 1.0f, 0.0f); //Perpendicular to the floor. Assuming as in Unity that forward is z-axis and up is y-axis
//	optix::float3 polarization = make_float3(0.0f, 1.0f, 0.0f); //Perpendicular to the floor. Assuming as in Unity that forward is z-axis and up is y-axis
//	optix::float3 posrx = make_float3(0.0f, 10.0f, 10.0f);
//	sceneManager->addReceiver(1, posrx, polarization, radius, sceneManager->printPower);
//	//sceneManager->createRaySphere2DSubstep(1, 1); //0.1 degree delta step
//	//sceneManager->createRaySphere2DSubstep(1, 1); //0.1 degree delta step
//	float deg2rad=M_PIf/180.f;	
//	float eli=84.0f;
//	float ele=96.0f;
//	float del=3.0f;
//	float azi=-6.0f;
//	float aze=6.0f;
//	float daz=3.0f;
//	sceneManager->createRaySphere2D(eli,del,ele,azi,daz,aze);
//	std::cout<<"dens="<<(sceneManager->getRaySphere().rayCount/((deg2rad*aze-deg2rad*azi)*(cosf(deg2rad*eli)-cosf(deg2rad*ele))))<<std::endl;
//	sceneManager->finishSceneContext();
//	sceneManager->transmit(0, 1.0f, postx, polarizationTx, false);
//	timer.stop();
//	std::cout<<"Time="<<timer.getTime()<<std::endl;
//	//		postx = make_float3(-18.0f, 10.0f, 50.0f);
//	//		sceneManager->transmit(0, 1.0f, postx, polarization);
//
//	return sceneManager;
//}
//
//std::unique_ptr<OpalSceneManager> crossingTestMulti(std::unique_ptr<OpalSceneManager> sceneManager, bool print, bool subSteps, bool useDepolarization, float radius) {
//
//	Timer timer;
//	float freq = 5.9e9f;
//
//	
//	//Init context before doing anything else
//	if (useDepolarization) {
//		LPFlatMeshReflectionSimulation* sim = new LPFlatMeshReflectionSimulation(sceneManager.get());
//		sceneManager->setSimulation(sim);
//	} else {
//		BasicFlatMeshReflectionSimulation* sim = new BasicFlatMeshReflectionSimulation(sceneManager.get());
//		sceneManager->setSimulation(sim);
//	}
//	
//	
//	sceneManager->initContext(freq);
//
//	sceneManager->enableMultitransmitter();
//	std::cout << "Simulating crossing streets test with multitransmitter" << std::endl;
//	//Cubes
//	std::vector<int> cubeind = sceneManager->loadTrianglesFromFile("meshes/tricube.txt");
//	std::vector<float3> cubevert = sceneManager->loadVerticesFromFile("meshes/vertcube.txt");
//	//std::cout << "indices=" << cubeind.size() << "vertices=" << cubevert.size() << std::endl;
//	//Cube(4) NW
//	Matrix4x4 tm;
//	tm.setRow(0, make_float4(40.0f, 0, 0, -30.0f));
//	tm.setRow(1, make_float4(0, 40.0f, 0, 20.0f));
//	tm.setRow(2, make_float4(0, 0, 40.0f, 80.0f));
//	tm.setRow(3, make_float4(0, 0, 0, 1));
//	MaterialEMProperties emProp1;
//	emProp1.dielectricConstant = make_float2(3.75f, -60.0f*sceneManager->getChannelParameters().waveLength*0.038f);
//		//There is a dependency on the frequency again, we use -15 dB per 203 mm at 5 GHz => -75 dB/m
//		emProp1.tattenuation = make_float2(0.1f,-75.f );
//	//emProp1.dielectricConstant = make_float2(3.75f, -0.4576f);
//	std::cout << "Adding NW. Em="<< emProp1.dielectricConstant << std::endl;
//	sceneManager->addStaticMesh(static_cast<int>(cubevert.size()), cubevert.data(), static_cast<int>(cubeind.size()), cubeind.data(), tm, emProp1);
//
//	//Cube SW
//	tm.setRow(0, make_float4(40.0f, 0, 0, -30.0f));
//	tm.setRow(1, make_float4(0, 40.0f, 0, 20.0f));
//	tm.setRow(2, make_float4(0, 0, 40.0f, 20.0f));
//	tm.setRow(3, make_float4(0, 0, 0, 1));
//	std::cout << "Adding SW. Em = "<< emProp1.dielectricConstant << std::endl;
//	sceneManager->addStaticMesh(static_cast<int>(cubevert.size()), cubevert.data(), static_cast<int>(cubeind.size()), cubeind.data(), tm, emProp1);
//	//Cube(2) NE
//
//	tm.setRow(0, make_float4(40.0f, 0, 0, 30.0f));
//	tm.setRow(1, make_float4(0, 40.0f, 0, 20.0f));
//	tm.setRow(2, make_float4(0, 0, 40.0f, 80.0f));
//	tm.setRow(3, make_float4(0, 0, 0, 1));
//	std::cout << "Adding NE. Em = "<< emProp1.dielectricConstant << std::endl;
//	sceneManager->addStaticMesh(static_cast<int>(cubevert.size()), cubevert.data(), static_cast<int>(cubeind.size()), cubeind.data(), tm, emProp1);
//
//	//Cube(1) SE
//
//	tm.setRow(0, make_float4(40.0f, 0, 0, 30.0f));
//	tm.setRow(1, make_float4(0, 40.0f, 0, 20.0f));
//	tm.setRow(2, make_float4(0, 0, 40.0f, 20.0f));
//	tm.setRow(3, make_float4(0, 0, 0, 1));
//	std::cout << "Adding SE. Em = "<< emProp1.dielectricConstant << std::endl;
//	sceneManager->addStaticMesh(static_cast<int>(cubevert.size()), cubevert.data(), static_cast<int>(cubeind.size()), cubeind.data(), tm, emProp1);
//
//	//Horizontal plane
//	std::vector<int> planeind = sceneManager->loadTrianglesFromFile("meshes/tri.txt");
//	std::vector<float3> planever = sceneManager->loadVerticesFromFile("meshes/vert.txt");
//	//std::cout << "indices=" << planeind.size() << "vertices=" << planever.size() << std::endl;
//
//	tm.setRow(0, make_float4(10.0f, 0, 0, 0.0f));
//	tm.setRow(1, make_float4(0, 1, 0, 0.0f));
//	tm.setRow(2, make_float4(0, 0, 10.0f, 50.0f));
//	tm.setRow(3, make_float4(0, 0, 0, 1));
//
//	//emProp1.dielectricConstant = make_float2(3.75f, -60.0f*sceneManager->defaultChannel.waveLength*0.15f);
//	std::cout << "Adding Plane. Em=" << emProp1.dielectricConstant << std::endl;
//	sceneManager->addStaticMesh(static_cast<int>(planever.size()), planever.data(), static_cast<int>(planeind.size()), planeind.data(), tm, emProp1);
//
//	if (subSteps) {
//		sceneManager->createRaySphere2DSubstep(1, 1); //0.1 degree delta step
//	} else {
//		sceneManager->createRaySphere2D(1, 1); //1 degree delta step
//	}
//
//	
//	//**** Test 1: tow receivers at symmetrical configuration, power must be almost equal
//	//**** Two transmitters moving
//	//optix::float3 polarization = make_float3(0.0f, 1.0f, 0.0f); //Perpendicular to the floor. Assuming as in Unity that forward is z-axis and up is y-axis
//
//	//optix::float3 posrx = make_float3(0.0f, 10.0f, 100.0f);
//	//sceneManager->addReceiver(1, posrx, polarization, radius, sceneManager->printPower);
//	//posrx = make_float3(0.0f, 10.0f, 0.0f);
//	//sceneManager->addReceiver(3, posrx,polarization, radius, sceneManager->printPower);
//
//
//	////sceneManager->setMaxReflections(3u);
//	//sceneManager->finishSceneContext();
//
//	//if (print) {
//	//	sceneManager->setPrintEnabled(1024 * 1024 * 1024);	
//	//}
//	////sceneManager->setUsageReport();
//
//	//optix::float3 postx=make_float3(0.0f,0.0f,0.f);
//	//timer.start();
//	////Two transmitters moving
//	//TransmitterManager* transmitterManager=sceneManager->getTransmitterManager();
//	//	transmitterManager->registerTransmitter(0,postx,polarization,1.0f);
//	//	transmitterManager->registerTransmitter(2,postx,polarization,1.0f);
//
//
//	//	
//	//for (int i = -50; i <= 50; ++i) {
//
//	//	float x=i;
//	//	postx = make_float3(x, 10.f, 50.0f);
//	//	transmitterManager->addTransmitterToGroup(0,1.0f,postx,polarization);
//	//	transmitterManager->addTransmitterToGroup(2,1.0f,postx,polarization);
//	//	sceneManager->groupTransmit();
//	//	//sceneManager->transmit(0, 1.0f, postx, polarization);
//
//
//	//}
//	
//	//*******************
//	
//
//	//**** Test 2: One receiver, One transmitter per position
//	//**** Equal to crossingTest above but much faster
//
//	timer.start();
//	optix::float3 polarization = make_float3(0.0f, 1.0f, 0.0f); //Perpendicular to the floor. Assuming as in Unity that forward is z-axis and up is y-axis
//	optix::float3 posrx = make_float3(0.0f, 10.0f, 100.0f);
//	sceneManager->addReceiver(51, posrx, polarization, radius, sceneManager->printPower);
//	sceneManager->finishSceneContext();
//
//	if (print) {
//		sceneManager->setPrintEnabled(1024 * 1024 * 1024);	
//	}
//	optix::float3 postx=make_float3(0.0f,0.0f,0.f);
//	TransmitterManager* transmitterManager=sceneManager->getTransmitterManager();
//
//
//		
//	for (int i = -50; i <= 50; ++i) {
//
//		float x=i;
//		postx = make_float3(x, 10.f, 50.0f);
//		transmitterManager->registerTransmitter(i,postx,polarization,1.0f);
//		transmitterManager->addTransmitterToGroup(i,1.0f,postx,polarization);
//		//sceneManager->transmit(0, 1.0f, postx, polarization);
//
//
//	}
//	sceneManager->groupTransmit();
//	
//	
//	timer.stop();
//	std::cout<<"Time="<<timer.getTime()<<std::endl;
//	//		postx = make_float3(-18.0f, 10.0f, 50.0f);
//	//		sceneManager->transmit(0, 1.0f, postx, polarization);
//
//	return sceneManager;
//
//}
//
//
//
////Street crossing test. Cubes are intended to be buildings and a plane is the floor
//std::unique_ptr<OpalSceneManager> crossingTest(std::unique_ptr<OpalSceneManager> sceneManager, bool print, bool subSteps, bool useDepolarization, float radius) {
//
//	Timer timer;
//	float freq = 5.9e9f;
//
//	
//	//Init context before doing anything else
//	if (useDepolarization) {
//		LPFlatMeshReflectionSimulation* sim = new LPFlatMeshReflectionSimulation(sceneManager.get());
//		sceneManager->setSimulation(sim);
//	} else {
//		BasicFlatMeshReflectionSimulation* sim = new BasicFlatMeshReflectionSimulation(sceneManager.get());
//		sceneManager->setSimulation(sim);
//	}
//	
//	
//	sceneManager->initContext(freq);
//
//	std::cout << "Simulating crossing streets test" << std::endl;
//	//Cubes
//	std::vector<int> cubeind = sceneManager->loadTrianglesFromFile("meshes/tricube.txt");
//	std::vector<float3> cubevert = sceneManager->loadVerticesFromFile("meshes/vertcube.txt");
//	//std::cout << "indices=" << cubeind.size() << "vertices=" << cubevert.size() << std::endl;
//	//Cube(4) NW
//	Matrix4x4 tm;
//	tm.setRow(0, make_float4(40.0f, 0, 0, -30.0f));
//	tm.setRow(1, make_float4(0, 40.0f, 0, 20.0f));
//	tm.setRow(2, make_float4(0, 0, 40.0f, 80.0f));
//	tm.setRow(3, make_float4(0, 0, 0, 1));
//	MaterialEMProperties emProp1;
//	emProp1.dielectricConstant = make_float2(3.75f, -60.0f*sceneManager->getChannelParameters().waveLength*0.038f);
//		//There is a dependency on the frequency again, we use -15 dB per 203 mm at 5 GHz => -75 dB/m
//		emProp1.tattenuation = make_float2(0.1f,-75.f );
//	//emProp1.dielectricConstant = make_float2(3.75f, -0.4576f);
//	std::cout << "Adding NW. Em="<< emProp1.dielectricConstant << std::endl;
//	sceneManager->addStaticMesh(static_cast<int>(cubevert.size()), cubevert.data(), static_cast<int>(cubeind.size()), cubeind.data(), tm, emProp1);
//
//	//Cube SW
//	tm.setRow(0, make_float4(40.0f, 0, 0, -30.0f));
//	tm.setRow(1, make_float4(0, 40.0f, 0, 20.0f));
//	tm.setRow(2, make_float4(0, 0, 40.0f, 20.0f));
//	tm.setRow(3, make_float4(0, 0, 0, 1));
//	std::cout << "Adding SW. Em = "<< emProp1.dielectricConstant << std::endl;
//	sceneManager->addStaticMesh(static_cast<int>(cubevert.size()), cubevert.data(), static_cast<int>(cubeind.size()), cubeind.data(), tm, emProp1);
//	//Cube(2) NE
//
//	tm.setRow(0, make_float4(40.0f, 0, 0, 30.0f));
//	tm.setRow(1, make_float4(0, 40.0f, 0, 20.0f));
//	tm.setRow(2, make_float4(0, 0, 40.0f, 80.0f));
//	tm.setRow(3, make_float4(0, 0, 0, 1));
//	std::cout << "Adding NE. Em = "<< emProp1.dielectricConstant << std::endl;
//	sceneManager->addStaticMesh(static_cast<int>(cubevert.size()), cubevert.data(), static_cast<int>(cubeind.size()), cubeind.data(), tm, emProp1);
//
//	//Cube(1) SE
//
//	tm.setRow(0, make_float4(40.0f, 0, 0, 30.0f));
//	tm.setRow(1, make_float4(0, 40.0f, 0, 20.0f));
//	tm.setRow(2, make_float4(0, 0, 40.0f, 20.0f));
//	tm.setRow(3, make_float4(0, 0, 0, 1));
//	std::cout << "Adding SE. Em = "<< emProp1.dielectricConstant << std::endl;
//	sceneManager->addStaticMesh(static_cast<int>(cubevert.size()), cubevert.data(), static_cast<int>(cubeind.size()), cubeind.data(), tm, emProp1);
//
//	//Horizontal plane
//	std::vector<int> planeind = sceneManager->loadTrianglesFromFile("meshes/tri.txt");
//	std::vector<float3> planever = sceneManager->loadVerticesFromFile("meshes/vert.txt");
//	//std::cout << "indices=" << planeind.size() << "vertices=" << planever.size() << std::endl;
//
//	tm.setRow(0, make_float4(10.0f, 0, 0, 0.0f));
//	tm.setRow(1, make_float4(0, 1, 0, 0.0f));
//	tm.setRow(2, make_float4(0, 0, 10.0f, 50.0f));
//	tm.setRow(3, make_float4(0, 0, 0, 1));
//
//	//emProp1.dielectricConstant = make_float2(3.75f, -60.0f*sceneManager->defaultChannel.waveLength*0.15f);
//	std::cout << "Adding Plane. Em=" << emProp1.dielectricConstant << std::endl;
//	sceneManager->addStaticMesh(static_cast<int>(planever.size()), planever.data(), static_cast<int>(planeind.size()), planeind.data(), tm, emProp1);
//
//	if (subSteps) {
//		sceneManager->createRaySphere2DSubstep(1, 1); //0.1 degree delta step
//	} else {
//		sceneManager->createRaySphere2D(1, 1); //1 degree delta step
//	}
//
//	//receivers
//
//	optix::float3 posrx = make_float3(0.0f, 10.0f, 100.0f);
//	//optix::float3 polarization = make_float3(1.0f, 0.0f, 0.0f); //Perpendicular to the floor. Assuming as in Unity that forward is z-axis and up is y-axis
//	optix::float3 polarization = make_float3(0.0f, 1.0f, 0.0f); //Perpendicular to the floor. Assuming as in Unity that forward is z-axis and up is y-axis
//	sceneManager->addReceiver(1, posrx,polarization, radius, sceneManager->printPower);
//
//
//	//sceneManager->setMaxReflections(3u);
//
//	sceneManager->finishSceneContext();
//
//	if (print) {
//		sceneManager->setPrintEnabled(1024 * 1024 * 1024);	
//	}
//	//sceneManager->setUsageReport();
//
//	optix::float3 postx;
//	timer.start();
//
//	for (int i = -50; i <= 50; ++i) {
//
//		float x=i;
//		postx = make_float3(x, 10.f, 50.0f);
//
//		sceneManager->transmit(0, 1.0f, postx, polarization);
//
//
//	}
////	postx = make_float3(-7.0f, 10.0f, 50.0f);
////	sceneManager->transmit(0, 1.0f, postx, polarization);
//	timer.stop();
//	std::cout<<"Time="<<timer.getTime()<<std::endl;
//
//	return sceneManager;
//
//}
//
//





