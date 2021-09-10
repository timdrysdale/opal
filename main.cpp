/***************************************************************/
//
//Copyright (c) 2019 Esteban Egea-Lopez http://girtel.upct.es/~eegea
//
/**************************************************************/

#include "timer.h"
#include "Opal.h"
#include <memory>
using namespace opal;
using namespace optix;
//Tests. Compile as exe
#ifdef _WIN32
#else 
#include <unistd.h>
#endif

//Include the file with the tests you want to perform
#include "tests/depolarization.h"
#include "tests/tunnels.h"
#include "tests/tests.h"
#include "tests/rdn.h"
#include "tests/curvature.h"
#include "tests/dudley.h"
#include "tests/diffraction.h"
#include "tests/antennaGain.h"

int main(int argc, char** argv)
{
	try {

		
		std::cout << "Running Opal with:  " ;
		for(int i = 0; i < argc; ++i) {
 		       std::cout << argv[i] << ' ';
		}
		std::cout<<std::endl;
		//float freq = 900e6f;
		//float freq = 5.9e9f;
		//float freq = 1.0e9f;

		//Defaults	
		uint maxReflections=10u;
		bool printEnabled=false;
		bool subSteps=false;
		bool useExactSpeedOfLight=true;
		bool useDepolarization=false;
		bool usePenetration = false;
		bool useMultiGPU=true;
		bool useFastMath=true;
		float radius=0.04f;
#ifdef _WIN32
#else 

		std::string usage="Usage: opal [-options] \n  -r Max reflections E \n -p Enable OptiX rtPrintf on device to debug \n -s Use decimal degrees in angular spacing \n -c Use c=3e8 m/s. Default is c=299 792 458 m/s\n -d Enable depolarization \n -a Enable penetration \n -g Disable multiGPU \n -m disable fast_math \n -h Show help \n -t string to select a particular test (dependent on the program)";

		int c;
		int nr;
		std::string test;

		while ((c = getopt (argc, argv, "r:pscdagmu:ht:")) != -1) {
			switch (c) {
				case 'r':
					//std::cout<<optarg<<std::endl;
					nr=atoi(optarg);
					if (nr>=0) {

						maxReflections=nr;
					} else {
						fprintf(stderr, usage.c_str(),
								argv[0]);
						exit(EXIT_FAILURE);
					}
					break;
				case 'u':
					//std::cout<<"-u="<<optarg<<std::endl;
					radius=atof(optarg);
					break;
				case 't':
					//std::cout<<optarg<<std::endl;
					test=optarg;
					break;
				case 'p':
					printEnabled=true;
					break;
				case 's':
					subSteps=true;
					break;
				case 'c':
					useExactSpeedOfLight=false;
					break;
				case 'd':
					useDepolarization=true;
					break;
				case 'a':
					usePenetration=true;
					break;
				case 'g':
					useMultiGPU=false;
					break;
				case 'm':
					useFastMath=false;
					break;
				case 'h':
					std::cout<<usage<<std::endl;
					exit(0);
					break;

				default: /* '?' */
					fprintf(stderr, usage.c_str(),
							argv[0]);
					exit(EXIT_FAILURE);


			}
		}
#endif
		
		//New way to initialize: first create instance
	
		OpalSceneManager* sceneManager=new OpalSceneManager();


		//Now set desired common features
		if (!useExactSpeedOfLight) {
			sceneManager->useApproximateSpeedLight();
		}

		if (usePenetration) {	
			sceneManager->enablePenetration();
		}
		
		if (useDepolarization) {	
			sceneManager->enableDepolarization();
		}
		if (!useMultiGPU) {
			sceneManager->disableMultiGPU();	
		}
		if (!useFastMath) {
			sceneManager->disableFastMath();
		}
		sceneManager->setMaxReflections(maxReflections);
		
		//sceneManager->setUsageReport();
		if (printEnabled) {
			sceneManager->setPrintEnabled(1024* 1024 * 1024);	
		}

		//Additional features are set in the particular tests and initContext is called then 
		
		

		
		//Finally, run the test

		//Old version
		//sceneManager = rdnBasicTest(std::move(sceneManager),radius,useDepolarization);
		//sceneManager = rdnPlaneTest(std::move(sceneManager),radius,useDepolarization);
		//sceneManager = planeTest(std::move(sceneManager),radius,useDepolarization);
		//sceneManager = planeTestProgressive(std::move(sceneManager),radius,useDepolarization);
		//runTunnel(sceneManager,radius,useDepolarization, test);
		//sceneManager = polarizationPlaneTest(std::move(sceneManager), printEnabled,subSteps);
		//sceneManager = crossingTest(std::move(sceneManager), printEnabled,subSteps, useDepolarization, radius);
		//sceneManager = crossingTestMulti(std::move(sceneManager), printEnabled,subSteps, useDepolarization, radius);
		//sceneManager = freeSpace(std::move(sceneManager),  useDepolarization, radius);
	//	sceneManager = planeTest(std::move(sceneManager),  useDepolarization, radius);


//Basic tests
		//BasicTests basic(sceneManager,radius,useDepolarization);
		//basic.planeTest(0);
		//basic.freeSpace();
		//basic.quadTest(false,false);
		//basic.addCompoundDynamicMeshes();

		//basic.loadScenario();
		//CurvatureTests ct(sceneManager, radius);
		//ct.cylinderTest();
		//ct.symmetricDivergenceTest();
		
//Rectangular Tunnel tests
		//CubeTunnelTests ctt(sceneManager,radius,useDepolarization);
		//ctt.cubeTunnelRDNIsotropic(100);
		//ctt.cubeTunnelRDN(100);
		//ctt.cubeTunnel(0);//No random
		//ctt.cubeTunnelSingleReceiver(0);//No random
		//ctt.cubeTunnelWithCurvedSimulation(0,1);

//RDN tests	
		
		//RDNTests rdn(sceneManager,radius,useDepolarization);
		//rdn.runDidascalouDielectricRDN(1);
		//rdn.runDidascalouConductorRDN(1);
		//rdn.runDidascalouDielectric(0);
		//rdn.runDidascalouDielectricMultipleReceivers(0);
		//rdn.runTest(test);
		//rdn.rdnPlaneTestMultiReceiver();
		//rdn.rdnPlaneTest();
		//rdn.freeSpace();
//Dudley tests
		//DudleyTests dud(sceneManager,radius,useDepolarization);
		//dud.runTest(test);		


//Diffraction tests
		DiffractionTests dif(sceneManager,radius, useDepolarization);
		//dif.semiplaneDiffraction();
		//dif.semiplaneTotal();
		//dif.runSWCorner(false);
		dif.runSWCorner(true); //With multitransmitter
		//dif.runSWAcuteCorner();
		//dif.runCrossing();
		//dif.addCompoundDynamicMeshes();
//AntennaGain test
		//AntennaGainTests ag(sceneManager, radius);
		//ag.freeSpace(useDepolarization);
		//ag.freeSpaceRDN();
		
		delete sceneManager;


		return 0;
	}
	catch (optix::Exception& e) {
		std::cout << "main error occurred with error code "
			<< e.getErrorCode() << " and message "
			<< e.getErrorString() << std::endl;

		return 1;
	}
	catch (opal::Exception& e) {
		std::cout << "main error occurred with  message "
			<< e.getErrorString()
			<< std::endl;

		return 2;
	}

}


