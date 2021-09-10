#include "tunnels.h"
#include "../util.h"
#include "../timer.h"
#include "../flatSimulation.h"
#include "../basicSimulation.h"
#include "../curvedMeshSimulation.h"
#include "../curvedFlatMeshSimulation.h"
#include "../rayDensityNormalizationSimulation.h"

#include <fstream>
#include <iostream>
using namespace optix;
void CubeTunnelTests::xcut(float width, float height, float y, float distance, float3 polarization) {
	//Receivers grid. X cut at distance d
	uint points=ceil(10*width/sceneManager->getChannelParameters().waveLength);
	float xinit=-(width/2.0)+0.0001;
	float xend=(width/2.0)-0.0001;
	float xstep=(xend-xinit)/points;
	optix::float3 posrx;	
	for (int i=0;i<points;++i) {
		//posrx=make_float3(0.0f,xinit+(i*xstep),distance);
		posrx=make_float3(xinit+(i*xstep),y,distance);
		sceneManager->addReceiver(i+1, posrx,polarization, sphereRadius, sceneManager->printPower);
		
		
	}
	
}	
void  CubeTunnelTests::ycut(float width, float height, float x, float distance, float3 polarization) {
	//Receivers grid. Y cut at distance d
	uint points=ceil(10*height/sceneManager->getChannelParameters().waveLength);
	float xinit=-(height/2.0)+0.0001;
	float xend=(height/2.0)-0.0001;
	float xstep=(xend-xinit)/points;
	optix::float3 posrx;	
	for (int i=0;i<points;++i) {
		posrx=make_float3(x,xinit+(i*xstep),distance);
		sceneManager->addReceiver(i+1, posrx,polarization, sphereRadius, sceneManager->printPower);
		
		
	}

}
float CubeTunnelTests::zrun(float zinit, float deltad, float x, float y, float length, float3 polarization, float deltaSphere) {
	
	
	uint points=ceil((length-zinit)/deltad);
	optix::float3 posrx;	
	for (int i=0;i<points;++i) {
		posrx=make_float3(x,y,zinit+(i*deltad));
		if (deltaSphere>0) {
			sceneManager->addReceiver(i+1, posrx,polarization, sphereRadius+(i*deltaSphere), sceneManager->printPower);
		} else {
			//Fixed radius
			sceneManager->addReceiver(i+1, posrx,polarization, sphereRadius, sceneManager->printPower);
		}
		
		
	}
	if (deltaSphere>0) {
		return (sphereRadius+(points*deltaSphere));
	} else {
		return (sphereRadius);
	}

}
void CubeTunnelTests::cubeTunnelRDNIsotropic(float distance) {
	float freq = 900e6f;
	//float freq = 5.9e9f;

//	OpalSimulationTypes type=OpalSimulationTypes::RDN;
//	sceneManager->setSimulationType(type);
	RayDensityNormalizationSimulation* sim=new RayDensityNormalizationSimulation(sceneManager, RDNExecutionMode::HITINFO);
	sim->setComputeMode(ComputeMode::FIELD);
	sceneManager->setSimulation(sim);	
	sceneManager->enableGenerateRaysOnLaunch();
	std::cout<<"Init context"<<std::endl;		
	sceneManager->initContext(freq);


	Timer timer;


	//****Cube tunnel dimensions *****
	float width=8.5f;
	float height=5.0f;
	float length=1000.0f;


	//Receiver polarization
	optix::float3 polarization = V; 	
	//float distance=900.0f;


	//Transmitter
	//optix::float3 postx = make_float3(0.0f, 0.0f, 0.0f);
	optix::float3 postx = make_float3(-3.85f,1.0f, 0.0f);
	float3 polarizationTx = V; 





	//Load tunnel mesh
	MaterialEMProperties emProp1;
	emProp1.dielectricConstant = make_float2(5.0f, -60.0f*sceneManager->getChannelParameters().waveLength*0.01f);
	//There is a dependency on the frequency again, we use -15 dB per 203 mm at 5 GHz => -75 dB/m
	emProp1.tattenuation = make_float2(0.1f,-75.f );
	loadSquareTunnel(width, height, length, emProp1);



	//Change this value if launch enters an infinite loop due to precision errors
	sceneManager->setMinEpsilon(1e-4f);



	//For very large launches, you should enable exceptions at least once and check that no buffer overflow has occurred
	//sceneManager->enableExceptions();	


	//Multiple receivers

	xcut(width, height, -1.0f, distance, polarization);	
	sceneManager->finishSceneContext();

	int rayD=10000;
	//OpalRaySphereGenerator* gen=sceneManager->getRaySphereGenerator();
	//gen->generateRandomUniformOnDevice(0.0,180.0,-90.0,90.0,rayD*rayD); 
	//sceneManager->createRaySphereFromExternalBuffer(rayD,rayD,gen->getDevicePointer());
	//RayDensityNormalizationSimulation* sim=dynamic_cast<RayDensityNormalizationSimulation*>(sceneManager->getSimulation());
	sceneManager->setRayRange(0.0,180.0,-90.0,90.0,rayD,rayD);
	sim->setInitialDensity(((float)sceneManager->getRaySphere().rayCount)/(2*M_PIf));
	sim->setFiltering(2);//Not divided by m	
	//std::cout<<"Tracing angle (el/az)="<<(currentElevation-overlap)<<","<<(currentElevation+deltaEl+overlap)<<"/"<<(currentAzimuth-overlap)<<","<<(currentAzimuth+deltaAz+overlap)<<std::endl;
	//***Single ray transmit****
	//float3 mRay=make_float3(1.526963e-02f, 1.072892e-01f, 9.941105e-01f);
	//float3 mRay=normalize(make_float3(0.57735f,0.57735f,0.57735f));

	//sceneManager->createRaySphere2D(1,1,&mRay);
	//sceneManager->createRaySphere2D(currentElevation-overlap,asEl,currentElevation+deltaEl+overlap,currentAzimuth-overlap,asAz,currentAzimuth+deltaAz+overlap);



	std::cout <<"\t Start Isotropic Cube Tunnel ***"<<std::endl;	

	std::cout<<"Transmitting at "<<postx<<" with polarization="<<polarizationTx<<std::endl;
	std::cout<<"Receiving with radius="<<sphereRadius<<" with polarization="<<polarization<<std::endl;



	timer.start();


	//First launch
	sceneManager->transmit(0, 1.0f, postx, polarizationTx, false);



	timer.stop();
	std::cout<<"Time="<<timer.getTime()<<std::endl;


}

void CubeTunnelTests::cubeTunnelRDN(float distance) {
	float freq = 900e6f;
	//float freq = 5.9e9f;

	//OpalSimulationTypes type=OpalSimulationTypes::RDN;
	//sceneManager->setSimulationType(type);
	RayDensityNormalizationSimulation* sim=new RayDensityNormalizationSimulation(sceneManager);
	sceneManager->enableGenerateRaysOnLaunch();	
	sceneManager->setSimulation(sim);
	std::cout<<"Init context"<<std::endl;		
	sceneManager->initContext(freq);


	Timer timer;


	//****Cube tunnel dimensions *****
	float width=8.5f;
	float height=5.0f;
	float length=1000.0f;


	//Receiver polarization
	optix::float3 polarization = V; 	
	//float distance=900.0f;


	//Transmitter
	//optix::float3 postx = make_float3(0.0f, 0.0f, 0.0f);
	optix::float3 postx = make_float3(-3.85f,1.0f, 0.0f);
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




	//Load tunnel mesh
	MaterialEMProperties emProp1;
	emProp1.dielectricConstant = make_float2(5.0f, -60.0f*sceneManager->getChannelParameters().waveLength*0.01f);
	//There is a dependency on the frequency again, we use -15 dB per 203 mm at 5 GHz => -75 dB/m
	emProp1.tattenuation = make_float2(0.1f,-75.f );
	loadSquareTunnel(width, height, length, emProp1);



	//Change this value if launch enters an infinite loop due to precision errors
	sceneManager->setMinEpsilon(1e-4f);



	//For very large launches, you should enable exceptions at least once and check that no buffer overflow has occurred
	//sceneManager->enableExceptions();	


	//Multiple receivers

	xcut(width, height, -1.0f, distance, polarization);	

	sceneManager->finishSceneContext();

	float currentElevation=initElevation;
	float currentAzimuth=initAzimuth;
	//OpalRaySphereGenerator* gen=sceneManager->getRaySphereGenerator();
	//gen->generateRandomUniformOnDevice(currentElevation,currentElevation+deltaEl,currentAzimuth,currentAzimuth+deltaAz,rayD*rayD);
	//sceneManager->createRaySphereFromExternalBuffer(rayD,rayD,gen->getDevicePointer());
	//RayDensityNormalizationSimulation* sim=dynamic_cast<RayDensityNormalizationSimulation*>(sceneManager->getSimulation());

	float deg2rad=M_PIf/180.0f;
	float rayGoal=1e9;
	float solidAngle=deg2rad*deltaAz*(cosf(deg2rad*currentElevation)-cosf(deg2rad*(currentElevation+deltaEl)));
	int rayD=floor(sqrt(rayGoal*solidAngle)); 
	sceneManager->setRayRange(currentElevation,currentElevation+deltaEl,currentAzimuth,currentAzimuth+deltaAz,rayD,rayD);
	sim->setInitialDensity(sceneManager->getRaySphere().rayCount,currentAzimuth,currentAzimuth+deltaAz,currentElevation, currentElevation+deltaEl);
	sim->setFiltering(2);//Not divided by m	
	//std::cout<<"Tracing angle (el/az)="<<(currentElevation-overlap)<<","<<(currentElevation+deltaEl+overlap)<<"/"<<(currentAzimuth-overlap)<<","<<(currentAzimuth+deltaAz+overlap)<<std::endl;
	//***Single ray transmit****
	//float3 mRay=make_float3(1.526963e-02f, 1.072892e-01f, 9.941105e-01f);
	//float3 mRay=normalize(make_float3(0.57735f,0.57735f,0.57735f));

	//sceneManager->createRaySphere2D(1,1,&mRay);
	//sceneManager->createRaySphere2D(currentElevation-overlap,asEl,currentElevation+deltaEl+overlap,currentAzimuth-overlap,asAz,currentAzimuth+deltaAz+overlap);




	std::cout<<"Transmitting at "<<postx<<" with polarization="<<polarizationTx<<std::endl;
	std::cout<<"Receiving with radius="<<sphereRadius<<" with polarization="<<polarization<<std::endl;
	std::cout<<"Scanning the sphere with ASElevation="<<asEl<< " and ASAzimuth="<<asAz<<std::endl;



	timer.start();


	//First launch
	std::cout<<"Tracing angle (el/az)="<<(currentElevation-overlap)<<","<<(currentElevation+deltaEl+overlap)<<"/"<<(currentAzimuth-overlap)<<","<<(currentAzimuth+deltaAz+overlap)<<std::endl;
	sceneManager->transmit(0, 1.0f, postx, polarizationTx, true);

	//Now loop to fill the solid angle
	currentAzimuth += deltaAz;
	int launches=1;
	//Trace all elevations
	while (currentElevation<endElevation) {

		//Trace all azimuth	
		while(currentAzimuth<endAzimuth) {
			std::cout<<"Tracing angle (el/az)="<<(currentElevation-overlap)<<","<<(currentElevation+deltaEl+overlap)<<"/"<<(currentAzimuth-overlap)<<","<<(currentAzimuth+deltaAz+overlap)<<std::endl;
				//gen->generateRandomUniformOnDevice(currentElevation,currentElevation+deltaEl,currentAzimuth,currentAzimuth+deltaAz,rayD*rayD);
				//sceneManager->createRaySphereFromExternalBuffer(rayD,rayD,gen->getDevicePointer());
				solidAngle=deg2rad*deltaAz*(cosf(deg2rad*currentElevation)-cosf(deg2rad*(currentElevation+deltaEl)));
				rayD=floor(sqrt(rayGoal*solidAngle)); //Missing here the azimuth part of the density...
				sceneManager->setRayRange(currentElevation,currentElevation+deltaEl,currentAzimuth,currentAzimuth+deltaAz,rayD,rayD);
				sim->setInitialDensity(sceneManager->getRaySphere().rayCount,currentAzimuth,currentAzimuth+deltaAz,currentElevation, currentElevation+deltaEl);
			sceneManager->transmit(0, 1.0f, postx, polarizationTx, true);
			currentAzimuth += deltaAz;
			launches++;
		}
		currentAzimuth=initAzimuth;
		currentElevation += deltaEl;
	}
	sceneManager->endPartialLaunch(1u);


	timer.stop();
	std::cout<<"launches="<<launches<<";Time="<<timer.getTime()<<"; time/launch="<<(timer.getTime()/(float)launches)<<std::endl;


}

void CubeTunnelTests::cubeTunnel(int random) {
	float freq = 900e6f;
	//float freq = 5.9e9f;

	//Init context before doing anything else
	if (useDepolarization) {
		std::cout<<"Using Flat simulation with depolarization" <<std::endl;
		LPFlatMeshReflectionSimulation* sim=new LPFlatMeshReflectionSimulation(sceneManager);
		//sim->setComputeMode(ComputeMode::FIELD);
		sceneManager->setSimulation(sim);
	} else {
		BasicFlatMeshReflectionSimulation* sim= new BasicFlatMeshReflectionSimulation(sceneManager);
		sceneManager->setSimulation(sim);
	}
	
	sceneManager->enableGenerateRaysOnLaunch();	

	sceneManager->initContext(freq);
	
//	sceneManager->getSimulation()->setPrintHits(true);	


	Timer timer;


	//****Cube tunnel dimensions *****
	float width=8.5f;
	float height=5.0f;
	float length=1000.0f;


	//Receiver polarization
	optix::float3 polarization = V; 	
	float distance=900.0f;


	//Transmitter
	//optix::float3 postx = make_float3(0.0f, 0.0f, 0.0f);
	optix::float3 postx = make_float3(-3.85f,1.0f, 0.0f);
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




	//Load tunnel mesh
	MaterialEMProperties emProp1;
	emProp1.dielectricConstant = make_float2(5.0f, -60.0f*sceneManager->getChannelParameters().waveLength*0.01f);
	//There is a dependency on the frequency again, we use -15 dB per 203 mm at 5 GHz => -75 dB/m
	emProp1.tattenuation = make_float2(0.1f,-75.f );
	loadSquareTunnel(width, height, length, emProp1);



	//Change this value if launch enters an infinite loop due to precision errors
	sceneManager->setMinEpsilon(1e-4f);



	//For very large launches, you should enable exceptions at least once and check that no buffer overflow has occurred
	//sceneManager->enableExceptions();	


	//Multiple receivers

	xcut(width, height, -1.0f, distance, polarization);	
	sceneManager->finishSceneContext();

	float currentElevation=initElevation;
	float currentAzimuth=initAzimuth;
	OpalRaySphereGenerator* gen=sceneManager->getRaySphereGenerator();
	//std::cout<<"Tracing angle (el/az)="<<(currentElevation-overlap)<<","<<(currentElevation+deltaEl+overlap)<<"/"<<(currentAzimuth-overlap)<<","<<(currentAzimuth+deltaAz+overlap)<<std::endl;
	uint rayD=10000u;	
	if (random==0) {
		std::cout <<"**** Running Cube Tunnel with equispaced sampling sphere at distance "<<distance<<std::endl;	
		sceneManager->createRaySphere2D(currentElevation-overlap,asEl,currentElevation+deltaEl+overlap,currentAzimuth-overlap,asAz,currentAzimuth+deltaAz+overlap);
	} else {
		std::cout <<"**** Running Cube Tunnel with random sphere at distance"<<distance<<std::endl;	
		gen->generateRandomUniformOnDevice(currentElevation,currentElevation+deltaEl,currentAzimuth,currentAzimuth+deltaAz,rayD*rayD);	
		sceneManager->createRaySphereFromExternalBuffer(rayD,rayD,gen->getDevicePointer());
	}
	//***Single ray transmit****
	//float3 mRay=make_float3(0.016752, 0.0167189, 0.99972);
	//float3 mRay=make_float3(-0.0176237, 0.00886388, 0.999805);
	//float3 mRay=make_float3(-0.00209183, 0.00558441, 0.999982);
	//float3 mRay=make_float3(-0.00209182943217456, 0.00558440573513508, 0.999982178211212);
	//float3 mRay=normalize(make_float3(0.57735f,0.57735f,0.57735f));

	//sceneManager->createRaySphere2D(1,1,&mRay);

	//sceneManager->finishSceneContext();
	//sceneManager->transmit(0, 1.0f, postx, polarizationTx, false);
	//return;
	//******* End single ray transmit


	std::cout<<"Transmitting at "<<postx<<" with polarization="<<polarizationTx<<std::endl;
	std::cout<<"Receiving with radius="<<sphereRadius<<" with polarization="<<polarization<<std::endl;
	std::cout<<"Scanning the sphere with ASElevation="<<asEl<< " and ASAzimuth="<<asAz<<std::endl;



	timer.start();


	//First launch
	std::cout<<"Tracing angle (el/az)="<<(currentElevation-overlap)<<","<<(currentElevation+deltaEl+overlap)<<"/"<<(currentAzimuth-overlap)<<","<<(currentAzimuth+deltaAz+overlap)<<std::endl;
	sceneManager->transmit(0, 1.0f, postx, polarizationTx, true);

	//Now loop to fill the solid angle
	currentAzimuth += deltaAz;
	int launches=1;
	//Trace all elevations
	while (currentElevation<endElevation) {

		//Trace all azimuth	
		while(currentAzimuth<endAzimuth) {
			std::cout<<"Tracing angle (el/az)="<<(currentElevation-overlap)<<","<<(currentElevation+deltaEl+overlap)<<"/"<<(currentAzimuth-overlap)<<","<<(currentAzimuth+deltaAz+overlap)<<std::endl;
			if (random==0) {
				sceneManager->createRaySphere2D(currentElevation-overlap,asEl,currentElevation+deltaEl+overlap,currentAzimuth-overlap,asAz,currentAzimuth+deltaAz+overlap);
			} else {
				gen->generateRandomUniformOnDevice(currentElevation,currentElevation+deltaEl,currentAzimuth,currentAzimuth+deltaAz,rayD*rayD);	
				sceneManager->createRaySphereFromExternalBuffer(rayD,rayD,gen->getDevicePointer());
			}
			sceneManager->transmit(0, 1.0f, postx, polarizationTx, true);
			currentAzimuth += deltaAz;
			launches++;
		}
		currentAzimuth=initAzimuth;
		currentElevation += deltaEl;
	}
	sceneManager->endPartialLaunch(1u);


	timer.stop();
	std::cout<<"launches="<<launches<<";Time="<<timer.getTime()<<"; time/launch="<<(timer.getTime()/(float)launches)<<std::endl;


}
void CubeTunnelTests::cubeTunnelSingleReceiver(int random) {
	float freq = 900e6f;
	//float freq = 5.9e9f;

	//Init context before doing anything else
	if (useDepolarization) {
		LPFlatMeshReflectionSimulation* sim=new LPFlatMeshReflectionSimulation(sceneManager);
		sim->setComputeMode(ComputeMode::FIELD);
		sceneManager->setSimulation(sim);
	} else {
		BasicFlatMeshReflectionSimulation* sim= new BasicFlatMeshReflectionSimulation(sceneManager);
		sceneManager->setSimulation(sim);
	}


	sceneManager->initContext(freq);
	
	sceneManager->getSimulation()->setPrintHits(true);	


	Timer timer;


	//****Cube tunnel dimensions *****
	float width=8.5f;
	float height=5.0f;
	float length=1000.0f;


	//Receiver polarization
	optix::float3 polarization = V; 	
	float distance=900.0f;


	//Transmitter
	//optix::float3 postx = make_float3(0.0f, 0.0f, 0.0f);
	optix::float3 postx = make_float3(-3.85f,1.0f, 0.0f);
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




	//Load tunnel mesh
	MaterialEMProperties emProp1;
	emProp1.dielectricConstant = make_float2(5.0f, -60.0f*sceneManager->getChannelParameters().waveLength*0.01f);
	//There is a dependency on the frequency again, we use -15 dB per 203 mm at 5 GHz => -75 dB/m
	emProp1.tattenuation = make_float2(0.1f,-75.f );
	loadSquareTunnel(width, height, length, emProp1);



	//Change this value if launch enters an infinite loop due to precision errors
	sceneManager->setMinEpsilon(1e-4f);



	//For very large launches, you should enable exceptions at least once and check that no buffer overflow has occurred
	//sceneManager->enableExceptions();	


	//Multiple receivers

	//xcut(width, height, -1.0f, distance, polarization);	
	optix::float3 posrx = make_float3(-2.77f,-1.0f, distance);
	sceneManager->addReceiver(1, posrx,polarization, sphereRadius, sceneManager->printPower);

	float currentElevation=initElevation;
	float currentAzimuth=initAzimuth;
	OpalRaySphereGenerator* gen=sceneManager->getRaySphereGenerator();
	//std::cout<<"Tracing angle (el/az)="<<(currentElevation-overlap)<<","<<(currentElevation+deltaEl+overlap)<<"/"<<(currentAzimuth-overlap)<<","<<(currentAzimuth+deltaAz+overlap)<<std::endl;
	uint rayD=1000u;	
	if (random==0) {
		std::cout <<"**** Running Cube Tunnel with equispaced sampling sphere at distance "<<distance<<std::endl;	
		sceneManager->createRaySphere2D(currentElevation-overlap,asEl,currentElevation+deltaEl+overlap,currentAzimuth-overlap,asAz,currentAzimuth+deltaAz+overlap);
	} else {
		std::cout <<"**** Running Cube Tunnel with random sphere at distance"<<distance<<std::endl;	
		gen->generateRandomUniformOnDevice(currentElevation,currentElevation+deltaEl,currentAzimuth,currentAzimuth+deltaAz,rayD*rayD);	
		sceneManager->createRaySphereFromExternalBuffer(rayD,rayD,gen->getDevicePointer());
	}
	//***Single ray transmit****
	//float3 mRay=make_float3(0.016752, 0.0167189, 0.99972);
	//float3 mRay=make_float3(-0.0176237, 0.00886388, 0.999805);
	//float3 mRay=make_float3(-0.00209183, 0.00558441, 0.999982);
	//float3 mRay=make_float3(-0.00209182943217456, 0.00558440573513508, 0.999982178211212);
	//float3 mRay=normalize(make_float3(0.57735f,0.57735f,0.57735f));

	//sceneManager->createRaySphere2D(1,1,&mRay);

	//sceneManager->finishSceneContext();
	//sceneManager->transmit(0, 1.0f, postx, polarizationTx, false);
	//return;
	//******* End single ray transmit


	std::cout<<"Transmitting at "<<postx<<" with polarization="<<polarizationTx<<std::endl;
	std::cout<<"Receiving with radius="<<sphereRadius<<" with polarization="<<polarization<<std::endl;
	std::cout<<"Scanning the sphere with ASElevation="<<asEl<< " and ASAzimuth="<<asAz<<std::endl;



	sceneManager->finishSceneContext();
	timer.start();


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
			if (random==0) {
				sceneManager->createRaySphere2D(currentElevation-overlap,asEl,currentElevation+deltaEl+overlap,currentAzimuth-overlap,asAz,currentAzimuth+deltaAz+overlap);
			} else {
				gen->generateRandomUniformOnDevice(currentElevation,currentElevation+deltaEl,currentAzimuth,currentAzimuth+deltaAz,rayD*rayD);	
				sceneManager->createRaySphereFromExternalBuffer(rayD,rayD,gen->getDevicePointer());
			}
			sceneManager->transmit(0, 1.0f, postx, polarizationTx, true);
			currentAzimuth += deltaAz;
		}
		currentAzimuth=initAzimuth;
		currentElevation += deltaEl;
	}
	sceneManager->endPartialLaunch(1u);


	timer.stop();
	std::cout<<"Time="<<timer.getTime()<<std::endl;


}
void CubeTunnelTests::cubeTunnelWithCurvedSimulation(int random, int simType) {
	float freq = 900e6f;

	if (simType==0) {
		std::cout <<"**** Running Cube Tunnel With CURVEDWALLS***"<<std::endl;	
		LPCurvedMeshReflectionSimulation* sim = new LPCurvedMeshReflectionSimulation(sceneManager);
		sim->setComputeMode(ComputeMode::FIELD);
		sceneManager->setSimulation(sim);
	} else {
		std::cout <<"**** Running Cube Tunnel With CURVEDFLATWALLS***"<<std::endl;	
		LPCurvedFlatMeshReflectionSimulation* sim = new LPCurvedFlatMeshReflectionSimulation(sceneManager);
		sim->setComputeMode(ComputeMode::FIELD);
		sceneManager->setSimulation(sim);
	}
	
	
	sceneManager->enableGenerateRaysOnLaunch();	
	
	sceneManager->initContext(freq);
	if (simType==0) {
	//To make it faster. In fact, we could directly use CURVEDFLATWALLS simulation
		dynamic_cast<LPCurvedMeshReflectionSimulation*>(sceneManager->getSimulation())->disableAngleDiscrimination();
	}

	Timer timer;
	

	//****Cube tunnel dimensions *****
	float width=8.5f;
	float height=5.0f;
	float length=1000.0f;
	
	//****************

	//Receiver polarization
	optix::float3 polarization = V; 	
	float distance=100.0f;


	//Transmitter
	optix::float3 postx = make_float3(-3.85f,1.0f, 0.0f);
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




	//Load tunnel mesh
	MaterialEMProperties emProp1;
	emProp1.dielectricConstant = make_float2(5.0f, -60.0f*sceneManager->getChannelParameters().waveLength*0.01f);
	//There is a dependency on the frequency again, we use -15 dB per 203 mm at 5 GHz => -75 dB/m
	emProp1.tattenuation = make_float2(0.1f,-75.f );
	loadSquareTunnel(width, height, length, emProp1);
	
	
	
	//Change this value if launch enters an infinite loop due to precision errors
	sceneManager->setMinEpsilon(1e-4f);



	//For very large launches, you should enable exceptions at least once and check that no buffer overflow has occurred
	//sceneManager->enableExceptions();	


	//Multiple receivers
	xcut(width, height, -1.0f, distance, polarization);	
	sceneManager->finishSceneContext();
	
	float currentElevation=initElevation;
	float currentAzimuth=initAzimuth;

	OpalRaySphereGenerator* gen=sceneManager->getRaySphereGenerator();
	//std::cout<<"Tracing angle (el/az)="<<(currentElevation-overlap)<<","<<(currentElevation+deltaEl+overlap)<<"/"<<(currentAzimuth-overlap)<<","<<(currentAzimuth+deltaAz+overlap)<<std::endl;
	uint rayD=1000u;	
	if (random==0) {
		std::cout <<"**** Equispaced sampling sphere***"<<std::endl;	
		sceneManager->createRaySphere2D(currentElevation-overlap,asEl,currentElevation+deltaEl+overlap,currentAzimuth-overlap,asAz,currentAzimuth+deltaAz+overlap);
	} else {
		std::cout <<"**** Random sphere***"<<std::endl;	
		gen->generateRandomUniformOnDevice(currentElevation,currentElevation+deltaEl,currentAzimuth,currentAzimuth+deltaAz,rayD*rayD);	
		sceneManager->createRaySphereFromExternalBuffer(rayD,rayD,gen->getDevicePointer());
	}
	
	std::cout<<"Transmitting at "<<postx<<" with polarization="<<polarizationTx<<std::endl;
	std::cout<<"Receiving with radius="<<sphereRadius<<" with polarization="<<polarization<<std::endl;
	std::cout<<"Scanning the sphere with ASElevation="<<asEl<< " and ASAzimuth="<<asAz<<std::endl;
	
	timer.start();
	
	
	//First launch
	std::cout<<"Tracing angle (el/az)="<<(currentElevation-overlap)<<","<<(currentElevation+deltaEl+overlap)<<"/"<<(currentAzimuth-overlap)<<","<<(currentAzimuth+deltaAz+overlap)<<std::endl;
	sceneManager->transmit(0, 1.0f, postx, polarizationTx, true);
	//sceneManager->endPartialLaunch(1u);
	//return sceneManager;	
	
	//Now loop to fill the solid angle
	currentAzimuth += deltaAz;
	//Trace all elevations
	while (currentElevation<endElevation) {

		//Trace all azimuth	
		while(currentAzimuth<endAzimuth) {
			std::cout<<"Tracing angle (el/az)="<<(currentElevation-overlap)<<","<<(currentElevation+deltaEl+overlap)<<"/"<<(currentAzimuth-overlap)<<","<<(currentAzimuth+deltaAz+overlap)<<std::endl;
			if (random==0) {
				sceneManager->createRaySphere2D(currentElevation-overlap,asEl,currentElevation+deltaEl+overlap,currentAzimuth-overlap,asAz,currentAzimuth+deltaAz+overlap);
			} else {
				gen->generateRandomUniformOnDevice(currentElevation,currentElevation+deltaEl,currentAzimuth,currentAzimuth+deltaAz,rayD*rayD);	
				sceneManager->createRaySphereFromExternalBuffer(rayD,rayD,gen->getDevicePointer());
			}
			sceneManager->transmit(0, 1.0f, postx, polarizationTx, true);
			currentAzimuth += deltaAz;
		}
		currentAzimuth=initAzimuth;
		currentElevation += deltaEl;
	}
	sceneManager->endPartialLaunch(1u);
	
	
	
	timer.stop();
	std::cout<<"Time="<<timer.getTime()<<std::endl;


}
//***********
//
//
//
//**********************************



