#include "tunnels.h"
#include "../util.h"
#include "../timer.h"
using namespace optix;
void xcut(OpalSceneManager* sceneManager, float width, float height, float y, float distance, float3 polarization, float sphereRadius) {
	//Receivers grid. X cut at distance d
	uint points=ceil(10*width/sceneManager->getChannelParameters().waveLength);
	float xinit=-(width/2.0)+0.0001;
	float xend=(width/2.0)-0.0001;
	float xstep=(xend-xinit)/points;
	optix::float3 posrx;	
	for (int i=0;i<points;++i) {
		//posrx=make_float3(0.0f,xinit+(i*xstep),distance);
		posrx=make_float3(xinit+(i*xstep),y,distance);
		sceneManager->addReceiver(i+1, posrx,polarization, sphereRadius, printPower);
		
		
	}
	
}	
void  ycut(OpalSceneManager* sceneManager, float width, float height, float x, float distance, float3 polarization, float sphereRadius) {
	//Receivers grid. Y cut at distance d
	uint points=ceil(10*height/sceneManager->getChannelParameters().waveLength);
	float xinit=-(height/2.0)+0.0001;
	float xend=(height/2.0)-0.0001;
	float xstep=(xend-xinit)/points;
	optix::float3 posrx;	
	for (int i=0;i<points;++i) {
		posrx=make_float3(x,xinit+(i*xstep),distance);
		sceneManager->addReceiver(i+1, posrx,polarization, sphereRadius, printPower);
		
		
	}

}
void zrun(OpalSceneManager* sceneManager, float zinit, float deltad, float x, float y, float length, float3 polarization, float sphereRadius) {
	uint points=ceil((length-zinit)/deltad);
	optix::float3 posrx;	
	for (int i=0;i<=points;++i) {
		posrx=make_float3(x,y,zinit+(i*deltad));
		sceneManager->addReceiver(i+1, posrx,polarization, sphereRadius+(i*points*5e-6), printPower);
		
		
	}

}
std::unique_ptr<OpalSceneManager> cubeTunnel(std::unique_ptr<OpalSceneManager> sceneManager) {

	Timer timer;

	//Tunnel dimensions
	float width=8.5f;
	float height=5.0f;
	float length=1000.0f;
	
	
	//Polarizations 
	const float3 V=make_float3(0.0f, 1.0f, 0.0f); //Perpendicular to the floor. Assuming as in Unity that forward is z-axis and up is y-axis
	const float3 H=make_float3(1.0f, 0.0f, 0.0f); //Parallel to the floor. Assuming as in Unity that forward is z-axis and up is y-axis
	const float3 VH=make_float3(1.0f, 1.0f, 0.0f); 
	const float3 HZ=make_float3(0.0f, 0.0f, 1.0f); 

	//Receiver polarization
	optix::float3 polarization = H; 	
	float sphereRadius=0.04f;	
	float distance=10.0f;	
	//Transmitter
	//optix::float3 postx = make_float3(0.0f, 0.0f, 0.0f);
	optix::float3 postx = make_float3(-3.85f,1.0f, 0.0f);
	float3 polarizationTx = H; 
	
	
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
	
	std::cout <<"**** Tunnel ***"<<std::endl;	
	std::cout << "Simulating tunnel with width=" <<width<<" and height="<<height<< std::endl;
	std::cout<<"Transmitting at "<<postx<<" with polarization="<<polarizationTx<<std::endl;
	std::cout<<"Receiving with radius="<<sphereRadius<<" with polarization="<<polarization<<std::endl;
	std::cout<<"Scanning the sphere with ASElevation="<<asEl<< " and ASAzimuth="<<asAz<<std::endl;

	//Tunnel is a cube with the XY faces removed (entrance and exit). So tunnel runs on Z axis (front, in Unity)
	std::vector<int> cubeind = loadTrianglesFromFile("meshes/tunnel-i.txt");
	std::vector<float3> cubevert = loadVerticesFromFile("meshes/tunnel-v.txt");
	//std::cout << "indices=" << cubeind.size() << "vertices=" << cubevert.size() << std::endl;
	//Cube(4) NW
	Matrix4x4 tm;
	tm.setRow(0, make_float4(width, 0, 0, 0.0f));
	tm.setRow(1, make_float4(0, height, 0, 0.0f));
	tm.setRow(2, make_float4(0, 0, length, length/2.0f));
	tm.setRow(3, make_float4(0, 0, 0, 1));
	MaterialEMProperties emProp1;
	emProp1.dielectricConstant = make_float2(5.0f, -60.0f*sceneManager->getChannelParameters().waveLength*0.01f);
	//There is a dependency on the frequency again, we use -15 dB per 203 mm at 5 GHz => -75 dB/m
	emProp1.tattenuation = make_float2(0.1f,-75.f );
	//emProp1.dielectricConstant = make_float2(3.75f, -0.4576f);
	std::cout << "Adding NW. Em="<< emProp1.dielectricConstant << std::endl;
	sceneManager->addStaticMesh(static_cast<int>(cubevert.size()), cubevert.data(), static_cast<int>(cubeind.size()), cubeind.data(), tm, emProp1);


	//Change this value if launch enters an infinite loop due to precision errors
	sceneManager->setMinEpsilon(1e-4f);



	//For very large launches, you should enable exceptions at least once and check that no buffer overflow has occurred
	//sceneManager->enableExceptions();	


	//Multiple receivers
	
	xcut(sceneManager.get(), width, height, -1.0f, distance, polarization,sphereRadius);	
	//zrun(sceneManager.get(), 50.0f,10.0f,3.0f, -1.0f,1000.0f,polarization,sphereRadius);	
	
	//Single receiver
//	int j=1;	
//	optix::float3 posrx = make_float3(-3.012f, -1.0f, distance);	
//	//optix::float3 posrx = make_float3(-4.083f, -1.0f, distance);	
//	sceneManager->addReceiver(j, posrx,polarization, sphereRadius, printPower);
	


	/** Sequential test. For a really large number of reflections and receivers  (that is, potential hits) have to do a sequential test, to avoid buffer overflows */
	//sceneManager->addReceiver(1,make_float3(-3.0f,-1.0f, 360.f),polarization, sphereRadius, printPower);
	//uint points=ceil((500.0f-360.0f)/10.0f);
	//for (int i=0;i<=points;++i) {
	//	float3 posrx=make_float3(-3.0f,-1.0f,(10.0f*i) + 360.f);
	//	//posrx=make_float3(i,0.0f,10.0f);
	//	sceneManager->updateReceiver(1, posrx);
	//
	
	//Create first sphere to avoid trouble with the buffers
	//TODO: we should change this to support late creation of sphere
	float currentElevation=initElevation;
	float currentAzimuth=initAzimuth;
	std::cout<<"Tracing angle (el/az)="<<(currentElevation-overlap)<<","<<(currentElevation+deltaEl+overlap)<<"/"<<(currentAzimuth-overlap)<<","<<(currentAzimuth+deltaAz+overlap)<<std::endl;
	
	sceneManager->createRaySphere2D(currentElevation-overlap,asEl,currentElevation+deltaEl+overlap,currentAzimuth-overlap,asAz,currentAzimuth+deltaAz+overlap);
	
	//***Single ray transmit****
	//float3 mRay=make_float3(1.526963e-02f, 1.072892e-01f, 9.941105e-01f);

	//sceneManager->createRaySphere2D(1,1,&mRay);
	//sceneManager->createRaySphere2D(currentElevation-overlap,asEl,currentElevation+deltaEl+overlap,currentAzimuth-overlap,asAz,currentAzimuth+deltaAz+overlap);
	//sceneManager->transmit(0, 1.0f, postx, polarizationTx, true);
	//sceneManager->endPartialLaunch(1u);
	//
	//return sceneManager;	
	//*****************
	
	sceneManager->finishSceneContext();

	//sceneManager->setUsageReport();

	timer.start();
	
	
	
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
	//* Sequential test */
	//}
	
	timer.stop();
	std::cout<<"Time="<<timer.getTime()<<std::endl;

	return sceneManager;

}



