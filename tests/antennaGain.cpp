/***************************************************************/
//
//Copyright (c) 2021 Esteban Egea-Lopez http://girtel.upct.es/~eegea
//
/**************************************************************/

#include "antennaGain.h"
#include <limits>
#include "../singleDiffraction.h"
#include "../flatSimulation.h"
#include "../curvedFlatMeshSimulation.h"
#include "../basicSimulation.h"
#include "../rayDensityNormalizationSimulation.h"
#include "../util.h"
#include "../timer.h"

using namespace optix;
using namespace opal;
AntennaGainTests::AntennaGainTests(OpalSceneManager*   sceneManager, float sphereRadius) {
	this->sceneManager=sceneManager;
	this->sphereRadius=sphereRadius;
}
void AntennaGainTests::freeSpace(bool useDepolarization) {
	Timer timer;
	float freq = 5.9e9f;
	std::cout<<"Running free space test"<<std::endl;
        //sceneManager->enableGenerateRaysOnLaunch();	
	
	//Init context before doing anything else
	if (useDepolarization) {
		//LPCurvedFlatMeshReflectionSimulation* sim = new LPCurvedFlatMeshReflectionSimulation(sceneManager);
		LPFlatMeshReflectionSimulation* sim = new LPFlatMeshReflectionSimulation(sceneManager);
		sceneManager->setSimulation(sim);
	} else {
		BasicFlatMeshReflectionSimulation* sim = new BasicFlatMeshReflectionSimulation(sceneManager);
		sceneManager->setSimulation(sim);
	}
	

	sceneManager->setUseAntennaGain(true);
	
	sceneManager->initContext(freq);
	timer.start();
	optix::float3 postx = make_float3(0.0f, 10.0f, 0.0f);
	//optix::float3 polarizationTx = make_float3(0.0f, 1.0f, 0.0f); //Perpendicular to the floor. Assuming as in Unity that forward is z-axis and up is y-axis
	optix::float3 polarizationTx = normalize(make_float3(1.0f, 1.0f, 0.0f)); //Perpendicular to the floor. Assuming as in Unity that forward is z-axis and up is y-axis
	optix::float3 polarization = normalize(make_float3(0.0f, 1.0f, 0.0f)); //Perpendicular to the floor. Assuming as in Unity that forward is z-axis and up is y-axis
	optix::float3 posrx = make_float3(0.0f, 10.0f, 10.0f);
	sceneManager->addReceiver(1, posrx, polarization, sphereRadius, sceneManager->printPower);
	//AntennaGain gains=sceneManager->loadGainsFromFileIndBPower("gain17514.txt");
	AntennaGain gains=sceneManager->loadGainsFromFileIndBPower("dipole.txt");
	int gainId=sceneManager->registerAntennaGain(gains);
	sceneManager->registerReceiverGain(1,gainId);
	sceneManager->registerTransmitterGain(0,gainId);
	//sceneManager->createRaySphere2DSubstep(1, 1); //0.1 degree delta step
	

	//***Single ray transmit****
	float3 mRay=normalize(make_float3(0.0,0,1));
	sceneManager->createRaySphere2D(1,1,&mRay);
	
	sceneManager->finishSceneContext();
	sceneManager->transmit(0, 1.0f, postx, polarizationTx, false);
	timer.stop();
	std::cout<<"Time="<<timer.getTime()<<std::endl;

}
void AntennaGainTests::freeSpaceRDN() {
	Timer timer;
	float freq = 5.9e9f;
	std::cout<<"Running free space test"<<std::endl;
	RayDensityNormalizationSimulation* sim= new RayDensityNormalizationSimulation(sceneManager);
	
	sim->setComputeMode(ComputeMode::VOLTAGE);
        sceneManager->enableGenerateRaysOnLaunch();	
	sceneManager->setSimulation(sim);
	sceneManager->setUseAntennaGain(true);
	
	sceneManager->initContext(freq);
	
	//sceneManager->getSimulation()->setPrintHits(true);	
	sceneManager->finishSceneContext();

	uint rayD=1000u;
	sceneManager->setRayRange(0.0,180.0,0.0,360.0,rayD,rayD);
	sim->setInitialDensity(((float)sceneManager->getRaySphere().rayCount)/(4*M_PIf));
	sim->setFiltering(2);

	timer.start();
	optix::float3 postx = make_float3(0.0f, 10.0f, 0.0f);
	optix::float3 polarizationTx = make_float3(0.0f, 1.0f, 0.0f); //Perpendicular to the floor. Assuming as in Unity that forward is z-axis and up is y-axis
	optix::float3 polarization = make_float3(0.0f, 1.0f, 0.0f); //Perpendicular to the floor. Assuming as in Unity that forward is z-axis and up is y-axis
	optix::float3 posrx = make_float3(0.0f, 10.0f, 10.0f);
	sceneManager->addReceiver(1, posrx, polarization, sphereRadius, sceneManager->printPower);
	AntennaGain gains=sceneManager->loadGainsFromFileIndBPower("gain17514.txt");
	int gainId=sceneManager->registerAntennaGain(gains);
	sceneManager->registerReceiverGain(1,gainId);
	sceneManager->registerTransmitterGain(0,gainId);
	
	sceneManager->transmit(0, 1.0f, postx, polarizationTx, false);
	timer.stop();
	std::cout<<"Time="<<timer.getTime()<<std::endl;
	//		postx = make_float3(-18.0f, 10.0f, 50.0f);
	//		sceneManager->transmit(0, 1.0f, postx, polarization);

}
