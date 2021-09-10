/***************************************************************/
//
//Copyright (c) 2019 Esteban Egea-Lopez http://girtel.upct.es/~eegea
//
/**************************************************************/
#include "dudley.h"
#include "../timer.h"
#include <memory>
#include <fstream>
#include "../curvedMeshSimulation.h"
#include "../curvedFlatMeshSimulation.h"
#include "../rayDensityNormalizationSimulation.h"
#include "../util.h"
using namespace opal;
using namespace optix;
DudleyTests::DudleyTests(OpalSceneManager* sceneManager, float sphereRadius, bool useDepolarization) : TunnelsBase(sceneManager,sphereRadius,useDepolarization) {
}
void DudleyTests::runTest(std::string test) {
	std::vector<float3> Tx(3);
	std::vector<float3> Rx(3);
	std::vector<float> frequency(2);
	std::vector<float3> pol(2);
	float a=2;
	float phi=M_PIf/2.0f;
	frequency[0]=1e9f;
	frequency[1]=200e6;
	Tx[0]=make_float3(0.0f,0.0f, 0.0f);
	Tx[1]=make_float3(1.8f,0.0f,0.0f);//In paper, it is in cylindrical coordinates (0.9a,0,0)
	Tx[2]=make_float3(0.f,1.8f,0.0f); //In paper, it is in cylindrical coordinates (0.9a,pi/2,0)
	Rx[0]=make_float3(1.8f,0.0f,0.0f);
	Rx[1]=make_float3(0.f,1.8f,0.0f);//In paper, cylindrical (0.9a,pi/2,0)
	Rx[2]=make_float3(0.f,0.5f,0.0f);
	pol[0]=H;
	pol[1]=V;
	std::vector<int> tokens=parseTestString(test);
	
	freq=frequency[tokens[0]];	
	postx=Tx[tokens[1]];
	rx=Rx[tokens[2]];
	polarizationTx=pol[tokens[4]];

	if (freq>300e6) {
		std::cout<<"Dielectric er=12"<<std::endl;
		emProp1.dielectricConstant = make_float2(12.0f, -60.0f*sceneManager->getChannelParameters().waveLength*0.02f);
	} else {	
		std::cout<<"Dielectric er=5"<<std::endl;
		emProp1.dielectricConstant = make_float2(5.0f, -60.0f*sceneManager->getChannelParameters().waveLength*0.02f);
	}
	if (tokens[3]==0) {
		runDudleyRDNIsotropic(true);
	} else if (tokens[3]==1) {
		runDudleyRDN();
	} else {
		//Fig 17,
		phi=M_PIf/4.0f;
		freq=200e6;
		postx=0.9f*a*make_float3(cosf(phi),sinf(phi),0.0f);	
		rx=0.9f*a*make_float3(cosf(phi),sinf(phi),0.0f);	
		emProp1.dielectricConstant = make_float2(12.0f, -60.0f*sceneManager->getChannelParameters().waveLength*0.02f);
		//polarizationTx=make_float3(cosf(phi),sinf(phi),0.0f);
		polarizationTx=H;
		runDudleyRDN();
	}
}
void DudleyTests::runDudleyRDN() {
	Timer timer;
	RayDensityNormalizationSimulation* sim = new RayDensityNormalizationSimulation(sceneManager);
	sceneManager->setSimulation(sim);
	sceneManager->enableGenerateRaysOnLaunch(); //No memory used to store the rays: more ray density
	sceneManager->initContext(freq);

	//sceneManager->getSimulation()->setPrintHits(true);
	optix::float3 polarization = H; 	
	float distance=10.0f;

	//Sphere scanning
	float initElevation=0.0f;
	float initAzimuth=-90.0f;
	float endElevation=180.0f;
	float endAzimuth=90.0f;
	float deltaEl=10.0f;
	float deltaAz=10.0f;
	float asEl=0.01f;
	float asAz=0.01f;
	float overlap=0.0f;

	std::cout <<"*** Dudley Tunnel with sectorized RDN ***"<<std::endl;	


	//Transmitter
	//optix::float3 postx = make_float3(0.0f, 0.0f, 0.0f);
	//float3 polarizationTx = H; 

	emProp1.tattenuation = make_float2(0.1f,-75.f );
	loadCircularTunnel(2.0f,1200.0f, emProp1);

	std::cout<<"Transmitting at "<<postx<<" with polarization="<<polarizationTx<<std::endl;
	std::cout<<"Receiving with radius="<<sphereRadius<<" with polarization="<<polarization<<std::endl;
	std::cout<<"Scanning the sphere with ASElevation="<<asEl<< " and ASAzimuth="<<asAz<<std::endl;


	//Change this value if launch enters an infinite loop due to precision errors
	//sceneManager->setMinEpsilon(1e-4f);
	sceneManager->setMinEpsilon(1e-4f);

	float currentElevation=initElevation;
	float currentAzimuth=initAzimuth;
	
	//Ray gen on launch
        float deg2rad=M_PIf/180.0f;
	float rayGoal=1e9;
	float solidAngle=deg2rad*deltaAz*(cosf(deg2rad*currentElevation)-cosf(deg2rad*(currentElevation+deltaEl)));
	int rayD=floor(sqrt(rayGoal*solidAngle)); 
	std::cout<<"\t Scanning the sphere with ASElevation="<<asEl<< " and ASAzimuth="<<asAz<<"rays="<<rayD*rayD<<std::endl;
	sceneManager->finishSceneContext();
	timer.start();

	sceneManager->setRayRange(currentElevation,currentElevation+deltaEl,currentAzimuth,currentAzimuth+deltaAz,rayD,rayD);
	sim->setInitialDensity(sceneManager->getRaySphere().rayCount,currentAzimuth,currentAzimuth+deltaAz,currentElevation, currentElevation+deltaEl);
	sim->setFiltering(2);
	sim->setRayCount(sceneManager->getRaySphere().rayCount);	
	//Alternative sectorized transmit using ray sphere generator

	//uint rayD=10000u;
	//OpalRaySphereGenerator* gen=sceneManager->getRaySphereGenerator();
	//gen->generateRandomUniformOnDevice(currentElevation,currentElevation+deltaEl,currentAzimuth,currentAzimuth+deltaAz,rayD*rayD);
	//sceneManager->createRaySphereFromExternalBuffer(rayD,rayD,gen->getDevicePointer());
	//RayDensityNormalizationSimulation* sim=dynamic_cast<RayDensityNormalizationSimulation*>(sceneManager->getSimulation());
	//sim->setInitialDensity(sceneManager->getRaySphere().rayCount,currentAzimuth,currentAzimuth+deltaAz,currentElevation, currentElevation+deltaEl);
	//sceneManager->finishSceneContext();



	//sceneManager->setPrintEnabled(1024*1024*1024, make_uint3(976,552,0));

	uint nrx=100;	
	float deltas=sphereRadius/50.0f;
	for (int i=1;i<=nrx;++i) {
		//sceneManager->addReceiver(i,make_float3(3.72f,2.7f-6.0f, 20.0f),polarization, sphereRadius, sceneManager->printPower);
		sceneManager->addReceiver(i,rx,polarization, sphereRadius, sceneManager->printPower);
		//sphereRadius += deltas;
	}


	uint ns=2;	
	uint ls=100;
	float zinit=2.0f;
	//float zinit=6.0f;
	uint launches=0;
	rx.z=zinit;
	float rinit=sphereRadius;
	std::cout<<"Starting simulation from 0 to 100"<<std::endl;
	for (int n=0;n<floor(100/nrx);++n) {
		for (int j=1;j<=nrx;++j) {
			rx.z=zinit;	
			sceneManager->updateReceiver(j, rx,rinit);
			zinit+=1.0f;
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
				//std::cout<<"Tracing angle (el/az)="<<(currentElevation-overlap)<<","<<(currentElevation+deltaEl+overlap)<<"/"<<(currentAzimuth-overlap)<<","<<(currentAzimuth+deltaAz+overlap)<<std::endl;
				
				//Alternative ray generation
				//gen->generateRandomUniformOnDevice(currentElevation,currentElevation+deltaEl,currentAzimuth,currentAzimuth+deltaAz,rayD*rayD);
				//sceneManager->createRaySphereFromExternalBuffer(rayD,rayD,gen->getDevicePointer());
				//sim->setInitialDensity(sceneManager->getRaySphere().rayCount, currentAzimuth,currentAzimuth+deltaAz,currentElevation, currentElevation+deltaEl);
				//rayD=floor(sqrt(rayGoal*(cosf(deg2rad*currentElevation)-cosf(deg2rad*(currentElevation+deltaEl)))));
				//sceneManager->setRayRange(currentElevation,currentElevation+deltaEl,currentAzimuth,currentAzimuth+deltaAz,rayD,rayD);
				//sim->setInitialDensity(sceneManager->getRaySphere().rayCount,currentAzimuth,currentAzimuth+deltaAz,currentElevation, currentElevation+deltaEl);
				//sim->setRayCount(sceneManager->getRaySphere().rayCount);	


				solidAngle=deg2rad*deltaAz*(cosf(deg2rad*currentElevation)-cosf(deg2rad*(currentElevation+deltaEl)));
				rayD=floor(sqrt(rayGoal*solidAngle)); 
				sceneManager->setRayRange(currentElevation,currentElevation+deltaEl,currentAzimuth,currentAzimuth+deltaAz,rayD,rayD);
				sceneManager->transmit(0, 1.0f, postx, polarizationTx, true);
				currentAzimuth += deltaAz;
	                        ++launches;
			}
			currentAzimuth=initAzimuth;
			currentElevation += deltaEl;
		}
		sceneManager->endPartialLaunch(1u);

		currentElevation=initElevation;
		currentAzimuth=initAzimuth;

	} //Multiple zcuts for(n

	timer.stop();
	std::cout<<"Time="<<timer.getTime()<<"Time/launch="<<(timer.getTime()/launches)<<std::endl;

}
void DudleyTests::runDudleyRDNIsotropic(bool half) {
	Timer timer;
	RayDensityNormalizationSimulation* sim = new RayDensityNormalizationSimulation(sceneManager);
	sceneManager->setSimulation(sim);
        sceneManager->enableGenerateRaysOnLaunch();	
	sceneManager->initContext(freq);

	//sceneManager->getSimulation()->setPrintHits(true);
	//Receiver polarization
	optix::float3 polarization = H; 	
	float distance=10.0f;




	emProp1.tattenuation = make_float2(0.1f,-75.f );
	loadCircularTunnel(2.0f,1200.0f, emProp1);

	std::cout<<"Transmitting at "<<postx<<" with polarization="<<polarizationTx<<std::endl;
	std::cout<<"Receiving with radius="<<sphereRadius<<" with polarization="<<polarization<<std::endl;


	//Change this value if launch enters an infinite loop due to precision errors
	//sceneManager->setMinEpsilon(1e-4f);
	sceneManager->setMinEpsilon(1e-4f);
	sceneManager->finishSceneContext();

	// Isotropic transmit
	//OpalRaySphereGenerator* gen=sceneManager->getRaySphereGenerator();
	//uint rayD=4472u; //approx 20e6 rays
	uint rayD=10000u; //approx 100e6 rays
	if (half) { 
		//gen->generateRandomUniformOnDevice(0.0,180.0,-90.0,90.0,rayD*rayD); 
	       sceneManager->setRayRange(0.0,180.0,-90.0,90.0,rayD,rayD);
		std::cout <<"**** Dudley Tunnel with  Half Sphere RDN ***"<<std::endl;	
	} else {
		//gen->generateRandomUniformSphereOnDevice(rayD*rayD);
	       sceneManager->setRayRange(0.0,180.0,0.0,360.0,rayD,rayD);
		std::cout <<"**** Dudley Tunnel with Isotropic RDN ***"<<std::endl;	
	}

	//sceneManager->createRaySphereFromExternalBuffer(rayD,rayD,gen->getDevicePointer());
	//RayDensityNormalizationSimulation* sim=dynamic_cast<RayDensityNormalizationSimulation*>(sceneManager->getSimulation());

	if (half) { 
		sim->setInitialDensity(((float)sceneManager->getRaySphere().rayCount)/(2*M_PIf));
	} else {
		sim->setInitialDensity(((float)sceneManager->getRaySphere().rayCount)/(4*M_PIf));
	}

	sim->setFiltering(2);
	sim->setRayCount(rayD*rayD);
	timer.start();

	uint ns=2;	
	uint ls=100;
	float zinit=2.0f;
	//float zinit=6.0f;
	uint launches=1;
	rx.z=zinit;
	float rinit=sphereRadius;
	sceneManager->addReceiver(1,rx,polarization, sphereRadius, sceneManager->printPower);
	//sceneManager->addReceiver(1,make_float3(1.8f,0.0f, zinit),polarization, sphereRadius, sceneManager->printPower);
	std::cout<<"Starting simulation from 0 to 100"<<std::endl;
	for (int n=0;n<100;++n) {
		rx.z=zinit;	
		std::cout<<"radius="<<rinit<<std::endl;
		//float3 posrx=make_float3(1.8f,0.0f,zinit );
		sceneManager->updateReceiver(1, rx,rinit);
		zinit+=1.0f;

		//First launch
		sceneManager->transmit(0, 1.0f, postx, polarizationTx, false);

	} //Multiple zcuts for(n

	timer.stop();
	std::cout<<"Time="<<timer.getTime()<<"Time/launch="<<(timer.getTime()/launches)<<std::endl;

}

