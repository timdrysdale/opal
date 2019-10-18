#include "tunnels.h"
#include "../util.h"
#include "../timer.h"
using namespace optix;
std::unique_ptr<OpalSceneManager> cubeTunnel(std::unique_ptr<OpalSceneManager> sceneManager, bool print, bool subSteps) {

	Timer timer;

	std::cout << "Simulating crossing streets test" << std::endl;
	//Tunnel is a cube with the XY faces removed (entrance and exit). So tunnel runs on Z axis (front, in Unity)
	std::vector<int> cubeind = loadTrianglesFromFile("meshes/tunnel-i.txt");
	std::vector<float3> cubevert = loadVerticesFromFile("meshes/tunnel-v.txt");
	//std::cout << "indices=" << cubeind.size() << "vertices=" << cubevert.size() << std::endl;
	//Cube(4) NW
	Matrix4x4 tm;
	tm.setRow(0, make_float4(10.0f, 0, 0, 0.0f));
	tm.setRow(1, make_float4(0, 5.0f, 0, 0.0f));
	tm.setRow(2, make_float4(0, 0, 1000.0f, 500.0f));
	tm.setRow(3, make_float4(0, 0, 0, 1));
	MaterialEMProperties emProp1;
	emProp1.dielectricConstant = make_float2(5.0f, -60.0f*sceneManager->getChannelParameters().waveLength*0.01f);
	//There is a dependency on the frequency again, we use -15 dB per 203 mm at 5 GHz => -75 dB/m
	emProp1.tattenuation = make_float2(0.1f,-75.f );
	//emProp1.dielectricConstant = make_float2(3.75f, -0.4576f);
	std::cout << "Adding NW. Em="<< emProp1.dielectricConstant << std::endl;
	sceneManager->addStaticMesh(static_cast<int>(cubevert.size()), cubevert.data(), static_cast<int>(cubeind.size()), cubeind.data(), tm, emProp1);



	
	//receivers

	optix::float3 posrx = make_float3(0.0f, 0.0f, 10.0f);
	optix::float3 polarization = make_float3(0.0f, 1.0f, 0.0f); //Perpendicular to the floor. Assuming as in Unity that forward is z-axis and up is y-axis

	sceneManager->setMinEpsilon(1e-4f);
	int j=1;	
	//sceneManager->addReceiver(j, posrx,polarization, 0.1f, printPower);



	//For very large launches, you should enable exceptions at least once and check that no buffer overflow has occurred
	//sceneManager->enableExceptions();	
	uint points=300;
	float xinit=-5+0.0001;
	float xend=5-0.0001;
	float xstep=(xend-xinit)/points;
	for (int i=0;i<points;++i) {
		posrx=make_float3(xinit+(i*xstep),0.0f,10.0f);
		sceneManager->addReceiver(i+1, posrx,polarization, 1.0f, printPower);
	}
	
//	for (float i=-4.9f;i<=4.9f;i=i+0.1f) {
//		posrx=make_float3(i,0.0f,10.0f);
//		sceneManager->addReceiver(j, posrx,polarization, 2.0f, printPower);
//		j++;
//	}


	/** Sequential test */
//	sceneManager->addReceiver(1, posrx,polarization, 1.0f, printPower);
//	for (int i=0;i<points;++i) {
//		posrx=make_float3(xinit+(i*xstep),0.0f,10.0f);
//		//posrx=make_float3(i,0.0f,10.0f);
//		sceneManager->updateReceiver(1, posrx);
	
	
	float initElevation=0.0f;
	float initAzimuth=-90.0f;
	float endElevation=180.0f;
	float endAzimuth=90.0f;
	float deltaEl=10.0f;
	float deltaAz=10.0f;
	float asEl=0.01f;
	float asAz=0.01f;
	float overlap=0.5f;
	//Create first sphere to avoid trouble with the buffers
	//TODO: we should change this to support late creation of sphere
	float currentElevation=initElevation;
	float currentAzimuth=initAzimuth;
	sceneManager->createRaySphere2D(currentElevation-overlap,asEl,currentElevation+deltaEl+overlap,currentAzimuth-overlap,asAz,currentAzimuth+deltaAz+overlap);
	sceneManager->finishSceneContext();

	if (print) {
		sceneManager->setPrintEnabled(1024 * 1024 * 1024);	
	}
	//sceneManager->setUsageReport();

	optix::float3 postx;
	timer.start();
	postx = make_float3(0.0f, 0.0f, 0.0f);
	//First launch
	sceneManager->transmit(0, 1.0f, postx, polarization, true);
	
	//Now loop to fill the solid angle
	currentAzimuth += deltaAz;
	//Trace all elevations
	while (currentElevation<endElevation) {

		//Trace all azimuth	
		while(currentAzimuth<endAzimuth) {
			std::cout<<"Tracing angle (el/az)="<<(currentElevation-overlap)<<","<<(currentElevation+deltaEl+overlap)<<"/"<<(currentAzimuth-overlap)<<","<<(currentAzimuth+deltaAz+overlap)<<std::endl;
			sceneManager->createRaySphere2D(currentElevation-overlap,asEl,currentElevation+deltaEl+overlap,currentAzimuth-overlap,asAz,currentAzimuth+deltaAz+overlap);
			sceneManager->transmit(0, 1.0f, postx, polarization, true);
			currentAzimuth += deltaAz;
		}
		currentAzimuth=initAzimuth;
		currentElevation += deltaEl;
	}
	//sceneManager->createRaySphere2D(91.f,0.01f,150.0f,-70.0f,0.01f,70.0f);
	//sceneManager->transmit(0, 1.0f, postx, polarization, true);
	sceneManager->endPartialLaunch();
	//* Sequential test */
	//}
//	for (uint i = 1; i <= 50; ++i) {
//
//		sceneManager->setMaxReflections(i);
//		sceneManager->transmit(0, 1.0f, postx, polarization);
//
//
//	}
	
	
//	for (float i=-4.9f;i<=4.9f;i=i+.1f) {
//		posrx=make_float3(i,0.0f,100.0f);
//		sceneManager->updateReceiver(1, posrx);
//		sceneManager->transmit(0, 1.0f, postx, polarization);
//	}
	timer.stop();
	std::cout<<"Time="<<timer.getTime()<<std::endl;

	return sceneManager;

}



