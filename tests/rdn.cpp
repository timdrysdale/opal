/***************************************************************/
//
//Copyright (c) 2019 Esteban Egea-Lopez http://girtel.upct.es/~eegea
//
/**************************************************************/
#include "rdn.h"
#include "../timer.h"
#include <memory>
#include <fstream>
#include "../curvedMeshSimulation.h"
#include "../curvedFlatMeshSimulation.h"
#include "../util.h"
using namespace opal;
using namespace optix;
RDNTests::RDNTests(OpalSceneManager* sceneManager, float sphereRadius, bool useDepolarization) : TunnelsBase(sceneManager,sphereRadius,useDepolarization) {
	this->sim= new RayDensityNormalizationSimulation(sceneManager);
}
void RDNTests::freeSpace() {
	//A free space to test that the filtering procedure works
	Timer timer;
	float freq = 5.9e9f;
	std::cout<<"Running free space test"<<std::endl;

	sceneManager->setSimulation(sim);
	sceneManager->initContext(freq);

	//sceneManager->getSimulation()->setPrintHits(true);	

	uint rayD=1000u;
	OpalRaySphereGenerator* gen=sceneManager->getRaySphereGenerator();
	//gen->generateRandomUniformOnDevice(eli,ele,azi,aze,rayD*rayD);
	gen->generateRandomUniformSphereOnDevice(rayD*rayD);
	sceneManager->createRaySphereFromExternalBuffer(rayD,rayD,gen->getDevicePointer());
	//RayDensityNormalizationSimulation* sim=dynamic_cast<RayDensityNormalizationSimulation*>(sceneManager->getSimulation());
	//sim->setInitialDensity(sceneManager->getRaySphere().rayCount,azi,aze,eli, ele);
	sim->setInitialDensity(((float)sceneManager->getRaySphere().rayCount)/(4*M_PIf));
	sim->setRayCount(sceneManager->getRaySphere().rayCount);


	timer.start();
	optix::float3 postx = make_float3(0.0f, 10.0f, 0.0f);
	optix::float3 polarizationTx = make_float3(0.0f, 1.0f, 0.0f); //Perpendicular to the floor. Assuming as in Unity that forward is z-axis and up is y-axis
	optix::float3 polarization = make_float3(0.0f, 1.0f, 0.0f); //Perpendicular to the floor. Assuming as in Unity that forward is z-axis and up is y-axis
	optix::float3 posrx = make_float3(0.0f, 10.0f, 10.0f);
	sceneManager->addReceiver(1, posrx, polarization, sphereRadius, sceneManager->printPower);

	sceneManager->finishSceneContext();
	sceneManager->transmit(0, 1.0f, postx, polarizationTx, false);
	timer.stop();
	std::cout<<"Time="<<timer.getTime()<<std::endl;
	//		postx = make_float3(-18.0f, 10.0f, 50.0f);
	//		sceneManager->transmit(0, 1.0f, postx, polarization);

}

void  RDNTests::rdnPlaneTestMultiReceiver() {
	Timer timer;
	float freq = 5.9e9f;
	std::cout<<"Running plane test"<<std::endl;

	sceneManager->setSimulation(sim);
	sceneManager->enableGenerateRaysOnLaunch();
	sceneManager->enableMultiChannel();	
	sceneManager->initContext(freq);
	//sceneManager->getSimulation()->setPrintHits(true);	


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
	//emProp1.dielectricConstant = make_float2(3.75f, -60.0f*sceneManager->getChannelParameters().waveLength*0.038f);
	//There is a dependency on the frequency again, we use -15 dB per 203 mm at 5 GHz => -75 dB/m
	emProp1.tattenuation = make_float2(0.1f,-75.f );
	sceneManager->addStaticMesh(4, quadh, 6, quadind, tm, emProp1 );


	float deg2rad=M_PIf/180.f;	
	float eli=0.0f;
	float ele=180.0f;
	float azi=-90.0f;
	float aze=90.0f;
	uint rayD=10000u;
	//OpalRaySphereGenerator* gen=sceneManager->getRaySphereGenerator();
	//gen->generateRandomUniformOnDevice(eli,ele,azi,aze,rayD*rayD);
	//gen->generateRandomUniformSphereOnDevice(rayD*rayD);
	//sceneManager->createRaySphereFromExternalBuffer(rayD,rayD,gen->getDevicePointer());

	//receivers

	optix::float3 polarization = make_float3(0.0f, 1.0f, 0.0f); //Perpendicular to the floor. Assuming as in Unity that forward is z-axis and up is y-axis

	int nrx=100;
	for (int i=1;i<=nrx;++i) {
		//sceneManager->addReceiver(i,make_float3(3.72f,2.7f-6.0f, 20.0f),polarization, sphereRadius, sceneManager->printPower);
		//	sceneManager->addReceiver(i,make_float3(0.0,2.0f, 2),polarization, sphereRadius, sceneManager->printPower);
		sceneManager->addReceiver(i,make_float3(0.0,2.0f, i),polarization, sphereRadius, sceneManager->printPower);
	}
	//sceneManager->addReceiver(1,make_float3(0.0,2.0f, 99),polarization, sphereRadius, sceneManager->printPower);
	sceneManager->finishSceneContext();
	sceneManager->setRayRange(0.0,180.0,0.0,360.0,rayD,rayD);
	sim->setInitialDensity(((float)sceneManager->getRaySphere().rayCount)/(4*M_PIf));
	sim->setFiltering(2);

	optix::float3 postx = make_float3(0.0f, 10.0f, 0.0f);

	timer.start();
	//	sceneManager->setUsageReport();



	sceneManager->transmit(0, 1.0f, postx, polarization);

	timer.stop();
	std::cout<<"Time="<<timer.getTime()<<std::endl;


}

void  RDNTests::rdnPlaneTest() {
	Timer timer;
	float freq = 5.9e9f;
	std::cout<<"Running plane test"<<std::endl;

	//sceneManager->setSimulationType(OpalSimulationTypes::RDN);
	sceneManager->setSimulation(sim);
	sceneManager->initContext(freq);
	//sceneManager->getSimulation()->setPrintHits(true);	


	//Horizontal plane as quad at origin 
	int quadind[6] = { 0,1,2,1,0,3 };
	optix::float3 quadh[4] = { make_float3(-0.5f,0.0f,-0.5f),make_float3(0.5f,0.f,0.5f) ,make_float3(0.5f,0.f,-0.5f) ,make_float3(-0.5f,0.0f,0.5f) };

	//Scale 200x200
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


	float deg2rad=M_PIf/180.f;	
	float eli=0.0f;
	float ele=180.0f;
	float azi=-90.0f;
	float aze=90.0f;
	uint rayD=10000u;
	OpalRaySphereGenerator* gen=sceneManager->getRaySphereGenerator();
	//gen->generateRandomUniformOnDevice(eli,ele,azi,aze,rayD*rayD);
	gen->generateRandomUniformSphereOnDevice(rayD*rayD);
	sceneManager->createRaySphereFromExternalBuffer(rayD,rayD,gen->getDevicePointer());
	//RayDensityNormalizationSimulation* sim=dynamic_cast<RayDensityNormalizationSimulation*>(sceneManager->getSimulation());
	//sim->setInitialDensity(sceneManager->getRaySphere().rayCount,azi,aze,eli, ele);
	sim->setInitialDensity(((float)sceneManager->getRaySphere().rayCount)/(4*M_PIf));
	sim->setRayCount(sceneManager->getRaySphere().rayCount);


	//receivers
	optix::float3 posrx = make_float3(0.0f, 2.0f, 100.0f);
	//optix::float3 posrx2 = make_float3(10.0f, 10.0f, 100.0f);

	optix::float3 polarization = make_float3(0.0f, 1.0f, 0.0f); //Perpendicular to the floor. Assuming as in Unity that forward is z-axis and up is y-axis
	sceneManager->addReceiver(1, posrx,polarization, sphereRadius, sceneManager->printPower);


	optix::float3 postx = make_float3(0.0f, 10.0f, 50.0f);

	timer.start();
	sceneManager->finishSceneContext();
	//	sceneManager->setUsageReport();



	for (size_t i = 0; i < 100; ++i)
	{
		postx.z = 99.0f - i;
		sceneManager->transmit(0, 1.0f, postx, polarization);

	}
	timer.stop();
	std::cout<<"Time="<<timer.getTime()<<std::endl;


}

void RDNTests::runDidascalouDielectricRDN(int mode) {
	Timer timer;
	float freq = 1e9f;
	//OpalSimulationTypes type=OpalSimulationTypes::RDN;
	sim->setComputeMode(ComputeMode::VOLTAGE);
	//sim->setComputeMode(ComputeMode::FIELD);
	sceneManager->setSimulation(sim);
	//TODO: pass this as a flag
	sceneManager->enableGenerateRaysOnLaunch();	
	sceneManager->initContext(freq);

	//sceneManager->getSimulation()->setPrintHits(true);	
	//Receiver polarization
	optix::float3 polarization = V; 	

	//Transmitter
	optix::float3 postx = make_float3(0.0f,1.0f, 0.0f);
	float3 polarizationTx = V; 
	std::cout <<"**** Didascalou RDN Dielectric Tunnel ***"<<std::endl;	
	MaterialEMProperties emProp1;

	//Didascalou
	//emProp1.dielectricConstant = make_float2(5.0f, 0.0f);
	emProp1.dielectricConstant = make_float2(5.0f, -60.0f*sceneManager->getChannelParameters().waveLength*0.01f);
	//There is a dependency on the frequency again, we use -15 dB per 203 mm at 5 GHz => -75 dB/m
	emProp1.tattenuation = make_float2(0.1f,-75.f );
	loadCircularTunnel(2.0f,1200.0f, emProp1);
	std::cout<<"Transmitting at "<<postx<<" with polarization="<<polarizationTx<<std::endl;
	std::cout<<"Receiving with radius="<<sphereRadius<<" with polarization="<<polarization<<std::endl;
	sceneManager->setMinEpsilon(1e-4f);


	if (mode==0) {
		didascalouDielectricIsotropic(false, polarization,postx, polarizationTx);
	} else if (mode==1) {
		didascalouDielectricIsotropic(true, polarization,postx, polarizationTx);
	} else {

		//Sphere scanning
		float initElevation=0.0f;
		float initAzimuth=-90.0f;
		float endElevation=180.0f;
		float endAzimuth=90.0f;
		float deltaEl=10.0f;
		float deltaAz=10.0f;
		float asEl=0.01f;
		float asAz=0.01f;
		//float overlap=0.5f;
		float overlap=0.0f;
		float currentElevation=initElevation;
		float currentAzimuth=initAzimuth;

		std::cout<<"\t Start sectorized Didascalou dielectric RDN. Scanning the sphere with ASElevation="<<asEl<< " and ASAzimuth="<<asAz<<std::endl;
		Timer timer;
		timer.start();


		//Sectorized transmit
		uint rayD=1000u;
		sceneManager->finishSceneContext();
		sceneManager->setRayRange(currentElevation,currentElevation+deltaEl,currentAzimuth,currentAzimuth+deltaAz,rayD,rayD);
		//RayDensityNormalizationSimulation* sim=dynamic_cast<RayDensityNormalizationSimulation*>(sceneManager->getSimulation());
		sim->setInitialDensity(sceneManager->getRaySphere().rayCount,currentAzimuth,currentAzimuth+deltaAz,currentElevation, currentElevation+deltaEl);
		sim->setFiltering(2);	

		//sceneManager->setPrintEnabled(1024*1024*1024, make_uint3(976,552,0));
		timer.start();
		int nrx=100;
		for (int i=1;i<=nrx;++i) {
			//sceneManager->addReceiver(i,make_float3(3.72f,2.7f-6.0f, 20.0f),polarization, sphereRadius, sceneManager->printPower);
			sceneManager->addReceiver(i,make_float3(-0.648f,-1.0f, 14.f),polarization, sphereRadius, sceneManager->printPower);
		}



		/** Sequential test. For a really large number of reflections and receivers  (that is, potential hits) have to do a sequential test, to avoid buffer overflows */
		//	sceneManager->addReceiver(1,make_float3(-0.648f,-1.0f, 14.f),polarization, sphereRadius, sceneManager->printPower);
		float zinit=10.0f;
		uint launches=0;
		float xinit=-1.8f;
		for (int i=0;i<=floor(100/nrx);++i) {
			for (int j=1;j<=nrx;++j) {
				float3 posrx=make_float3(1.0f,-1.0f,zinit );
				sceneManager->updateReceiver(j, posrx);
				zinit+=0.1f;
			}

			//First launch
			std::cout<<"Tracing angle (el/az)="<<(currentElevation-overlap)<<","<<(currentElevation+deltaEl+overlap)<<"/"<<(currentAzimuth-overlap)<<","<<(currentAzimuth+deltaAz+overlap)<<std::endl;
			sceneManager->transmit(0, 1.0f, postx, polarizationTx, true);

			//Now loop to fill the solid angle
			currentAzimuth += deltaAz;
			//Trace all elevations
			while (currentElevation<endElevation) {

				//Trace all azimuth	
				while(currentAzimuth<endAzimuth) {
					std::cout<<"Tracing angle (el/az)="<<(currentElevation-overlap)<<","<<(currentElevation+deltaEl+overlap)<<"/"<<(currentAzimuth-overlap)<<","<<(currentAzimuth+deltaAz+overlap)<<std::endl;


					sceneManager->setRayRange(currentElevation,currentElevation+deltaEl,currentAzimuth,currentAzimuth+deltaAz,rayD,rayD);
					sim->setInitialDensity(sceneManager->getRaySphere().rayCount,currentAzimuth,currentAzimuth+deltaAz,currentElevation, currentElevation+deltaEl);
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

}
void RDNTests::didascalouDielectricIsotropic(bool half, float3 polarization, float3 postx, float3 polarizationTx) {
	// Isotropic transmit
	//OpalRaySphereGenerator* gen=sceneManager->getRaySphereGenerator();
	//uint rayD=4472u; //approx 20e6 rays
	uint rayD=10000u; //approx 100e6 rays
	int nrx=100;
	sceneManager->finishSceneContext();
	if (half) { 
		//gen->generateRandomUniformOnDevice(0.0,180.0,-90.0,90.0,rayD*rayD); 

		//ray generation on launch
		sceneManager->setRayRange(0.0,180.0,-90.0,90.0,rayD,rayD);
	} else {
		//gen->generateRandomUniformSphereOnDevice(rayD*rayD);
		sceneManager->setRayRange(0.0,180.0,0.0,360.0,rayD,rayD);
	}

	//sceneManager->createRaySphereFromExternalBuffer(rayD,rayD,gen->getDevicePointer());

	if (half) { 
		sim->setInitialDensity(((float)sceneManager->getRaySphere().rayCount)/(2*M_PIf));
	} else {
		sim->setInitialDensity(((float)sceneManager->getRaySphere().rayCount)/(4*M_PIf));
	}
	sim->setFiltering(2);
	Timer timer;
	std::cout <<"\t Start Isotropic Didascalou RDN Dielectric Tunnel ***"<<std::endl;	
	timer.start();



	/******** With angle discrimination ********/
	/** Sequential test. For a really large number of reflections and receivers  (that is, potential hits) have to do a sequential test, to avoid buffer overflows */
	//sceneManager->addReceiver(1,make_float3(-0.648f,-1.0f, 14.f),polarization, sphereRadius, sceneManager->printPower);
	for (int i=1;i<=nrx;++i) {
		//sceneManager->addReceiver(i,make_float3(3.72f,2.7f-6.0f, 20.0f),polarization, sphereRadius, sceneManager->printPower);
		sceneManager->addReceiver(i,make_float3(-0.648f,-1.0f, 14.f),polarization, sphereRadius, sceneManager->printPower);
	}
	float zinit=10.0f;
	uint launches=0;
	float xinit=-1.8f;
	for (int i=0;i<floor(100/nrx);++i) {
		for (int j=1;j<=nrx;++j) {
			float3 posrx=make_float3(1.0f,-1.0f,zinit );
			sceneManager->updateReceiver(j, posrx);
			zinit+=0.1f;
		}

		sceneManager->transmit(0, 1.0f, postx, polarizationTx, false);

		++launches;

	} 

	timer.stop();
	std::cout<<"Time="<<timer.getTime()<<". Time/launch="<<(timer.getTime()/launches)<<std::endl;
}
void RDNTests::runDidascalouConductorRDN(int mode) {
	///////////////
	//
	// Requires uncommenting the optix/reflectionFunctions.h to Didascalou perfect conductor
	//
	// ////////////////////

	float freq = 1e9f;
	//OpalSimulationTypes type=OpalSimulationTypes::RDN;
	sceneManager->setSimulation(sim);
	//sceneManager->setSimulationType(type);	
	//TODO: pass as flag
	sceneManager->enableGenerateRaysOnLaunch();	
	sceneManager->initContext(freq);
	Timer timer;
	//Receiver polarization
	optix::float3 polarization = V; 	

	//Transmitter
	optix::float3 postx = make_float3(0.0f,0.0f, 0.0f);
	float3 polarizationTx = V; 

	//Didascalou

	std::cout <<"**** Didascalou Conductor Tunnel RDN ***"<<std::endl;	
	MaterialEMProperties emProp1;
	//emProp1.dielectricConstant = make_float2(5.0f, 0.0f);
	//Set perfect conductor
	emProp1.dielectricConstant=make_float2(0.0f,std::numeric_limits<float>::infinity());
	//emProp1.dielectricConstant = make_float2(5.0f, -60.0f*sceneManager->getChannelParameters().waveLength*0.01f);
	//There is a dependency on the frequency again, we use -15 dB per 203 mm at 5 GHz => -75 dB/m
	emProp1.tattenuation = make_float2(0.1f,-75.f );
	loadCircularTunnel(2.0f,1200.0f, emProp1);
	std::cout<<"Transmitting at "<<postx<<" with polarization="<<polarizationTx<<std::endl;
	std::cout<<"Receiving with radius="<<sphereRadius<<" with polarization="<<polarization<<std::endl;
	sceneManager->setMinEpsilon(1e-4f);
	if (mode==0) {
		didascalouConductorIsotropic(false, polarization,postx, polarizationTx);
	} else if (mode==1) {
		didascalouConductorIsotropic(true, polarization,postx, polarizationTx);
	} else {
		//Sphere scanning
		float initElevation=0.0f;
		float initAzimuth=-90.0f;
		float endElevation=180.0f;
		float endAzimuth=90.0f;
		float deltaEl=10.0f;
		float deltaAz=10.0f;
		float asEl=0.01f;
		float asAz=0.01f;
		//float overlap=0.5f;
		float overlap=0.0f;


		float currentElevation=initElevation;
		float currentAzimuth=initAzimuth;


		//Sectorized transmit
		uint rayD=1000u;
		OpalRaySphereGenerator* gen=sceneManager->getRaySphereGenerator();
		gen->generateRandomUniformOnDevice(currentElevation,currentElevation+deltaEl,currentAzimuth,currentAzimuth+deltaAz,rayD*rayD);
		sceneManager->createRaySphereFromExternalBuffer(rayD,rayD,gen->getDevicePointer());
		//RayDensityNormalizationSimulation* sim=dynamic_cast<RayDensityNormalizationSimulation*>(sceneManager->getSimulation());
		sim->setInitialDensity(sceneManager->getRaySphere().rayCount,currentAzimuth,currentAzimuth+deltaAz,currentElevation, currentElevation+deltaEl);



		sceneManager->finishSceneContext();
		std::cout <<"\t Start Sectorized Didascalou RDN Conductor Tunnel ***"<<std::endl;	
		std::cout<<"Scanning the sphere with ASElevation="<<asEl<< " and ASAzimuth="<<asAz<<std::endl;
		timer.start();



		/** Sequential test. For a really large number of reflections and receivers  (that is, potential hits) have to do a sequential test, to avoid buffer overflows */
		sceneManager->addReceiver(1,make_float3(0.0f,0.0f, 10.f),polarization, sphereRadius, sceneManager->printPower);
		float xinit=0.0f;
		for (int i=0;i<=80;++i) {
			float3 posrx=make_float3(xinit,0.0f,10.0f );
			sceneManager->updateReceiver(1, posrx);
			xinit+=0.01875f;


			//First launch
			std::cout<<"Tracing angle (el/az)="<<(currentElevation-overlap)<<","<<(currentElevation+deltaEl+overlap)<<"/"<<(currentAzimuth-overlap)<<","<<(currentAzimuth+deltaAz+overlap)<<std::endl;
			sceneManager->transmit(0, 1.0f, postx, polarizationTx, true);

			//Now loop to fill the solid angle
			currentAzimuth += deltaAz;
			//Trace all elevations
			while (currentElevation<endElevation) {

				//Trace all azimuth	
				while(currentAzimuth<endAzimuth) {
					std::cout<<"Tracing angle (el/az)="<<(currentElevation-overlap)<<","<<(currentElevation+deltaEl+overlap)<<"/"<<(currentAzimuth-overlap)<<","<<(currentAzimuth+deltaAz+overlap)<<std::endl;
					gen->generateRandomUniformOnDevice(currentElevation,currentElevation+deltaEl,currentAzimuth,currentAzimuth+deltaAz,rayD*rayD);
					sceneManager->createRaySphereFromExternalBuffer(rayD,rayD,gen->getDevicePointer());
					sim->setInitialDensity(sceneManager->getRaySphere().rayCount, currentAzimuth,currentAzimuth+deltaAz,currentElevation, currentElevation+deltaEl);

					sceneManager->transmit(0, 1.0f, postx, polarizationTx, true);
					currentAzimuth += deltaAz;
				}
				currentAzimuth=initAzimuth;
				currentElevation += deltaEl;
			}
			sceneManager->endPartialLaunch(1u);

			currentElevation=initElevation;
			currentAzimuth=initAzimuth;

		} //Multiple zcuts for(n

		timer.stop();
		std::cout<<"Time="<<timer.getTime()<<std::endl;
	}	
}

void RDNTests::didascalouConductorIsotropic(bool half, float3 polarization, float3 postx, float3 polarizationTx) {
	// Isotropic transmit
	///////////////
	//
	// Requires uncommenting the optix/reflectionFunctions.h to Didascalou perfect conductor
	//
	// ////////////////////
	//OpalRaySphereGenerator* gen=sceneManager->getRaySphereGenerator();
	sceneManager->finishSceneContext();
	//uint rayD=4472u; //approx 20e6 rays
	uint rayD=10000u; //approx 100e6 rays
	if (half) { 
		//gen->generateRandomUniformOnDevice(0.0,180.0,-90.0,90.0,rayD*rayD); 
		sceneManager->setRayRange(0.0,180.0,-90.0,90.0,rayD,rayD);
	} else {
		//gen->generateRandomUniformSphereOnDevice(rayD*rayD);
		sceneManager->setRayRange(0.0,180.0,0.0,360.0,rayD,rayD);
	}

	//sceneManager->createRaySphereFromExternalBuffer(rayD,rayD,gen->getDevicePointer());
	//RayDensityNormalizationSimulation* sim=dynamic_cast<RayDensityNormalizationSimulation*>(sceneManager->getSimulation());

	if (half) { 
		sim->setInitialDensity(((float)sceneManager->getRaySphere().rayCount)/(2*M_PIf));
	} else {
		sim->setInitialDensity(((float)sceneManager->getRaySphere().rayCount)/(4*M_PIf));
	}
	sim->setFiltering(2);
	sim->setRayCount(sceneManager->getRaySphere().rayCount);
	Timer timer;
	std::cout <<"\t Start Isotropic Didascalou RDN Conductor Tunnel ***"<<std::endl;	
	timer.start();
	//sceneManager->addReceiver(1,make_float3(0.0f,0.0f, 10.f),polarization, sphereRadius, sceneManager->printPower);
	sceneManager->addReceiver(1,make_float3(1.387499452f,0.0f, 10.f),polarization, sphereRadius, sceneManager->printPower);
	float xinit=0.0f;
	for (int i=0;i<=80;++i) {
		float3 posrx=make_float3(xinit,0.0f,10.0f );
		sceneManager->updateReceiver(1, posrx);
		xinit+=0.01875f;


		//First launch
		sceneManager->transmit(0, 1.0f, postx, polarizationTx, false);

	} //Multiple zcuts for(n
	timer.stop();
	std::cout<<"Time="<<timer.getTime()<<std::endl;


}
void RDNTests::runDidascalouConductor(int mode) {

	float freq = 1e9f;
	//	OpalSimulationTypes type=OpalSimulationTypes::CURVEDWALLS;

	//Change here type of simulation: not good, but ...
	LPCurvedMeshReflectionSimulation* csim=new LPCurvedMeshReflectionSimulation(sceneManager);
	sceneManager->setSimulation(csim);
	//sceneManager->setSimulationType(type);	
	sceneManager->initContext(freq);
	///////////////
	//
	// Requires uncommenting the optix/reflectionFunctions.h to Didascalou perfect conductor
	//
	// ////////////////////
	Timer timer;
	//Receiver polarization
	optix::float3 polarization = V; 	

	//Transmitter
	optix::float3 postx = make_float3(0.0f,0.0f, 0.0f);
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
	//float overlap=0.5f;
	float overlap=0.0f;
	std::cout <<"**** Didascalou Conductor Tunnel with sectorized CurvedWalls simulation***"<<std::endl;	
	MaterialEMProperties emProp1;

	//Didascalou
	//Set perfect conductor
	emProp1.dielectricConstant=make_float2(0.0f,std::numeric_limits<float>::infinity());
	//emProp1.dielectricConstant = make_float2(5.0f, -60.0f*sceneManager->getChannelParameters().waveLength*0.01f);
	//There is a dependency on the frequency again, we use -15 dB per 203 mm at 5 GHz => -75 dB/m
	emProp1.tattenuation = make_float2(0.1f,-75.f );
	loadCircularTunnel(2.0f,1200.0f, emProp1);
	std::cout<<"Transmitting at "<<postx<<" with polarization="<<polarizationTx<<std::endl;
	std::cout<<"Receiving with radius="<<sphereRadius<<" with polarization="<<polarization<<std::endl;
	std::cout<<"Scanning the sphere with ASElevation="<<asEl<< " and ASAzimuth="<<asAz<<std::endl;
	sceneManager->setMinEpsilon(1e-4f);

	float currentElevation=initElevation;
	float currentAzimuth=initAzimuth;
	uint rayD=1000u;
	OpalRaySphereGenerator* gen=sceneManager->getRaySphereGenerator();
	if (mode==0) {
		sceneManager->createRaySphere2D(currentElevation-overlap,asEl,currentElevation+deltaEl+overlap,currentAzimuth-overlap,asAz,currentAzimuth+deltaAz+overlap);
		std::cout <<"**** Using uniform solid angle sampling***"<<std::endl;	
	} else {
		gen->generateRandomUniformOnDevice(currentElevation-overlap,currentElevation+deltaEl+overlap,currentAzimuth-overlap,currentAzimuth+deltaAz+overlap,rayD*rayD);
		sceneManager->createRaySphereFromExternalBuffer(rayD,rayD,gen->getDevicePointer());
		std::cout <<"**** Using random solid angle sampling***"<<std::endl;	
	}

	sceneManager->finishSceneContext();
	timer.start();

	//Angle for separating duplicate rays

	float discriminateAngle=30.0f; //In degrees
	std::cout<<"Set angle discrimination to  "<<discriminateAngle<<std::endl;
	//LPCurvedMeshReflectionSimulation* sim=dynamic_cast<LPCurvedMeshReflectionSimulation*>(sceneManager->getSimulation());
	csim->setMaxAngleForDuplicateRays(discriminateAngle*M_PIf/180.f);

	/** Sequential test. For a really large number of reflections and receivers  (that is, potential hits) have to do a sequential test, to avoid buffer overflows */
	sceneManager->addReceiver(1,make_float3(0.0f,0.0f, 10.f),polarization, sphereRadius, sceneManager->printPower);
	float xinit=0.0f;
	for (int i=0;i<=80;++i) {
		float3 posrx=make_float3(xinit,0.0f,10.0f );
		sceneManager->updateReceiver(1, posrx);
		xinit+=0.01875f;


		//First launch
		//std::cout<<"Tracing angle (el/az)="<<(currentElevation-overlap)<<","<<(currentElevation+deltaEl+overlap)<<"/"<<(currentAzimuth-overlap)<<","<<(currentAzimuth+deltaAz+overlap)<<std::endl;
		sceneManager->transmit(0, 1.0f, postx, polarizationTx, true);

		//Now loop to fill the solid angle
		currentAzimuth += deltaAz;
		//Trace all elevations
		while (currentElevation<endElevation) {

			//Trace all azimuth	
			while(currentAzimuth<endAzimuth) {
				//std::cout<<"Tracing angle (el/az)="<<(currentElevation-overlap)<<","<<(currentElevation+deltaEl+overlap)<<"/"<<(currentAzimuth-overlap)<<","<<(currentAzimuth+deltaAz+overlap)<<std::endl;
				if (mode==0) {
					sceneManager->createRaySphere2D(currentElevation-overlap,asEl,currentElevation+deltaEl+overlap,currentAzimuth-overlap,asAz,currentAzimuth+deltaAz+overlap);
				} else {
					gen->generateRandomUniformOnDevice(currentElevation-overlap,currentElevation+deltaEl+overlap,currentAzimuth-overlap,currentAzimuth+deltaAz+overlap,rayD*rayD);
					sceneManager->createRaySphereFromExternalBuffer(rayD,rayD,gen->getDevicePointer());
				}
				sceneManager->transmit(0, 1.0f, postx, polarizationTx, true);
				currentAzimuth += deltaAz;
			}
			currentAzimuth=initAzimuth;
			currentElevation += deltaEl;
		}
		sceneManager->endPartialLaunch(1u);

		currentElevation=initElevation;
		currentAzimuth=initAzimuth;

	} //Multiple zcuts for(n

	timer.stop();
	std::cout<<"Time="<<timer.getTime()<<std::endl;

}
void RDNTests::runDidascalouDielectricMultipleReceivers(int mode) {
	Timer timer;
	float freq = 1e9f;
	//OpalSimulationTypes type=OpalSimulationTypes::CURVEDWALLS;
	LPCurvedMeshReflectionSimulation* csim=new LPCurvedMeshReflectionSimulation(sceneManager);
	sceneManager->setSimulation(csim);
	//sceneManager->setSimulationType(type);	

	sceneManager->enableGenerateRaysOnLaunch();	


	sceneManager->initContext(freq);


	//sceneManager->getSimulation()->setPrintHits(true);	
	//Receiver polarization
	optix::float3 polarization = V; 	

	//Transmitter
	optix::float3 postx = make_float3(0.0f,1.0f, 0.0f);
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
	//float overlap=0.5f;
	float overlap=0.0f;
	std::cout <<"**** Didascalou Tunnel with CURVEDWALLS simulation ***"<<std::endl;	
	MaterialEMProperties emProp1;

	//Didascalou
	//emProp1.dielectricConstant = make_float2(5.0f, 0.0f);
	//emProp1.dielectricConstant = make_float2(5.0f, -60.0f*sceneManager->getChannelParameters().waveLength*0.01f);
	emProp1.dielectricConstant = make_float2(5.0f, 0.0f);
	//There is a dependency on the frequency again, we use -15 dB per 203 mm at 5 GHz => -75 dB/m
	emProp1.tattenuation = make_float2(0.1f,-75.f );
	loadCircularTunnel(2.0f,1200.0f, emProp1);
	std::cout<<"Transmitting at "<<postx<<" with polarization="<<polarizationTx<<std::endl;
	std::cout<<"Receiving with radius="<<sphereRadius<<" with polarization="<<polarization<<std::endl;
	std::cout<<"Scanning the sphere with ASElevation="<<asEl<< " and ASAzimuth="<<asAz<<std::endl;
	sceneManager->setMinEpsilon(1e-4f);
	sceneManager->finishSceneContext();

	float currentElevation=initElevation;
	float currentAzimuth=initAzimuth;

	uint rayD=1000u;
	OpalRaySphereGenerator* gen=sceneManager->getRaySphereGenerator();
	if (mode==0) {
		sceneManager->createRaySphere2D(currentElevation-overlap,asEl,currentElevation+deltaEl+overlap,currentAzimuth-overlap,asAz,currentAzimuth+deltaAz+overlap);
		std::cout <<"**** Using uniform solid angle sampling***"<<std::endl;	
	} else {
		gen->generateRandomUniformOnDevice(currentElevation-overlap,currentElevation+deltaEl+overlap,currentAzimuth-overlap,currentAzimuth+deltaAz+overlap,rayD*rayD);
		sceneManager->createRaySphereFromExternalBuffer(rayD,rayD,gen->getDevicePointer());
		std::cout <<"**** Using random solid angle sampling***"<<std::endl;	
	}


	//sceneManager->setPrintEnabled(1024*1024*1024, make_uint3(976,552,0));
	timer.start();

	//Angle for separating duplicate rays

	float discriminateAngle=15.0f; //In degrees
	//LPCurvedMeshReflectionSimulation* sim=dynamic_cast<LPCurvedMeshReflectionSimulation*>(sceneManager->getSimulation());
	csim->setMaxAngleForDuplicateRays(discriminateAngle*M_PIf/180.f);


	/******** With angle discrimination ********/
	int nrx=10;
	for (int i=1;i<=nrx;++i) {
		//sceneManager->addReceiver(i,make_float3(3.72f,2.7f-6.0f, 20.0f),polarization, sphereRadius, sceneManager->printPower);
		sceneManager->addReceiver(i,make_float3(1.0f,-1.0f, 10.f),polarization, sphereRadius, sceneManager->printPower);
	}
	float zinit=10.0f;
	uint launches=0;
	float xinit=-1.8f;
	for (int i=0;i<=floor(100/nrx);++i) {
		for (int j=1;j<=nrx;++j) {
			float3 posrx=make_float3(1.0f,-1.0f,zinit );
			sceneManager->updateReceiver(j, posrx);
			zinit+=0.1f;
		}


		//***Single ray transmit****
		//float3 mRay=normalize(make_float3(-0.2487757802, 0.124283649, 0.9605541229));
		//float3 mRay=normalize(make_float3(-0.2477, 0.126362, 0.960561));
		//sceneManager->createRaySphere2D(1,1,&mRay);
		//	sceneManager->transmit(0, 1.0f, postx, polarizationTx, false);
		//	return;
		//*******************************//	

		//First launch
		//std::cout<<"Tracing angle (el/az)="<<(currentElevation-overlap)<<","<<(currentElevation+deltaEl+overlap)<<"/"<<(currentAzimuth-overlap)<<","<<(currentAzimuth+deltaAz+overlap)<<std::endl;
		sceneManager->transmit(0, 1.0f, postx, polarizationTx, true);

		//Now loop to fill the solid angle
		currentAzimuth += deltaAz;
		//Trace all elevations
		while (currentElevation<endElevation) {

			//Trace all azimuth	
			while(currentAzimuth<endAzimuth) {
				std::cout<<"Tracing angle (el/az)="<<(currentElevation-overlap)<<","<<(currentElevation+deltaEl+overlap)<<"/"<<(currentAzimuth-overlap)<<","<<(currentAzimuth+deltaAz+overlap)<<std::endl;
				if (mode==0) {
					sceneManager->createRaySphere2D(currentElevation-overlap,asEl,currentElevation+deltaEl+overlap,currentAzimuth-overlap,asAz,currentAzimuth+deltaAz+overlap);
				} else {
					gen->generateRandomUniformOnDevice(currentElevation-overlap,currentElevation+deltaEl+overlap,currentAzimuth-overlap,currentAzimuth+deltaAz+overlap,rayD*rayD);
					sceneManager->createRaySphereFromExternalBuffer(rayD,rayD,gen->getDevicePointer());
				}

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
void RDNTests::runDidascalouDielectric(int mode) {
	Timer timer;
	float freq = 1e9f;
	//OpalSimulationTypes type=OpalSimulationTypes::CURVEDWALLS;
	LPCurvedMeshReflectionSimulation* csim=new LPCurvedMeshReflectionSimulation(sceneManager);
	sceneManager->setSimulation(csim);
	//sceneManager->setSimulationType(type);	

	sceneManager->enableGenerateRaysOnLaunch();	


	sceneManager->initContext(freq);


	//sceneManager->getSimulation()->setPrintHits(true);	
	//Receiver polarization
	optix::float3 polarization = V; 	

	//Transmitter
	optix::float3 postx = make_float3(0.0f,1.0f, 0.0f);
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
	//float overlap=0.5f;
	float overlap=0.0f;
	std::cout <<"**** Didascalou Tunnel with CURVEDWALLS simulation ***"<<std::endl;	
	MaterialEMProperties emProp1;

	//Didascalou
	//emProp1.dielectricConstant = make_float2(5.0f, 0.0f);
	//emProp1.dielectricConstant = make_float2(5.0f, -60.0f*sceneManager->getChannelParameters().waveLength*0.01f);
	emProp1.dielectricConstant = make_float2(5.0f, 0.0f);
	//There is a dependency on the frequency again, we use -15 dB per 203 mm at 5 GHz => -75 dB/m
	emProp1.tattenuation = make_float2(0.1f,-75.f );
	loadCircularTunnel(2.0f,1200.0f, emProp1);
	std::cout<<"Transmitting at "<<postx<<" with polarization="<<polarizationTx<<std::endl;
	std::cout<<"Receiving with radius="<<sphereRadius<<" with polarization="<<polarization<<std::endl;
	std::cout<<"Scanning the sphere with ASElevation="<<asEl<< " and ASAzimuth="<<asAz<<std::endl;
	sceneManager->setMinEpsilon(1e-4f);

	float currentElevation=initElevation;
	float currentAzimuth=initAzimuth;

	uint rayD=1000u;
	OpalRaySphereGenerator* gen=sceneManager->getRaySphereGenerator();
	if (mode==0) {
		sceneManager->createRaySphere2D(currentElevation-overlap,asEl,currentElevation+deltaEl+overlap,currentAzimuth-overlap,asAz,currentAzimuth+deltaAz+overlap);
		std::cout <<"**** Using uniform solid angle sampling***"<<std::endl;	
	} else {
		gen->generateRandomUniformOnDevice(currentElevation-overlap,currentElevation+deltaEl+overlap,currentAzimuth-overlap,currentAzimuth+deltaAz+overlap,rayD*rayD);
		sceneManager->createRaySphereFromExternalBuffer(rayD,rayD,gen->getDevicePointer());
		std::cout <<"**** Using random solid angle sampling***"<<std::endl;	
	}


	//sceneManager->setPrintEnabled(1024*1024*1024, make_uint3(976,552,0));
	sceneManager->finishSceneContext();
	timer.start();

	//Angle for separating duplicate rays

	float discriminateAngle=15.0f; //In degrees
	//	LPCurvedMeshReflectionSimulation* sim=dynamic_cast<LPCurvedMeshReflectionSimulation*>(sceneManager->getSimulation());
	csim->setMaxAngleForDuplicateRays(discriminateAngle*M_PIf/180.f);


	/******** With angle discrimination ********/
	/** Sequential test. For a really large number of reflections and receivers  (that is, potential hits) have to do a sequential test, to avoid buffer overflows */
	sceneManager->addReceiver(1,make_float3(-0.648f,-1.0f, 14.f),polarization, sphereRadius, sceneManager->printPower);
	float zinit=10.0f;
	uint launches=0;
	float xinit=-1.8f;
	for (int i=0;i<=100;++i) {
		float3 posrx=make_float3(1.0f,-1.0f,zinit );
		sceneManager->updateReceiver(1, posrx);
		zinit+=0.1f;


		//***Single ray transmit****
		//float3 mRay=normalize(make_float3(-0.2487757802, 0.124283649, 0.9605541229));
		//float3 mRay=normalize(make_float3(-0.2477, 0.126362, 0.960561));
		//sceneManager->createRaySphere2D(1,1,&mRay);
		//	sceneManager->transmit(0, 1.0f, postx, polarizationTx, false);
		//	return;
		//*******************************//	

		//First launch
		//std::cout<<"Tracing angle (el/az)="<<(currentElevation-overlap)<<","<<(currentElevation+deltaEl+overlap)<<"/"<<(currentAzimuth-overlap)<<","<<(currentAzimuth+deltaAz+overlap)<<std::endl;
		sceneManager->transmit(0, 1.0f, postx, polarizationTx, true);

		//Now loop to fill the solid angle
		currentAzimuth += deltaAz;
		//Trace all elevations
		while (currentElevation<endElevation) {

			//Trace all azimuth	
			while(currentAzimuth<endAzimuth) {
				std::cout<<"Tracing angle (el/az)="<<(currentElevation-overlap)<<","<<(currentElevation+deltaEl+overlap)<<"/"<<(currentAzimuth-overlap)<<","<<(currentAzimuth+deltaAz+overlap)<<std::endl;
				if (mode==0) {
					sceneManager->createRaySphere2D(currentElevation-overlap,asEl,currentElevation+deltaEl+overlap,currentAzimuth-overlap,asAz,currentAzimuth+deltaAz+overlap);
				} else {
					gen->generateRandomUniformOnDevice(currentElevation-overlap,currentElevation+deltaEl+overlap,currentAzimuth-overlap,currentAzimuth+deltaAz+overlap,rayD*rayD);
					sceneManager->createRaySphereFromExternalBuffer(rayD,rayD,gen->getDevicePointer());
				}

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
void RDNTests::runTest(std::string test) {
	std::vector<int> tokens=parseTestString(test);
	switch(tokens[0]) {
		case 0:	
			runDidascalouDielectricRDN(tokens[1]);
			break;
		case 1:
			runDidascalouConductorRDN(tokens[1]);
			break;
		case 2:
			runDidascalouDielectric(tokens[1]);
			break;
		case 3:	
			runDidascalouConductor(tokens[1]);
			break;
		default:
			std::cout<<"RDNTests: unkown test" <<std::endl;
			break;
	}

}
