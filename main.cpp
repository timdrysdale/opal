/***************************************************************/
//
//Copyright (c) 2019 Esteban Egea-Lopez http://ait.upct.es/eegea
//
/**************************************************************/

#include "timer.h"
#include "Opal.h"
#include "multitransmitter.h"
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
//#include "tests/tests.h"


int main(int argc, char** argv)
{
	try {

		//std::cout << "Initializing " << std::endl;
		float freq = 5.9e9f;

		//Defaults	
		uint maxReflections=10u;
		bool printEnabled=false;
		bool subSteps=false;
		bool useExactSpeedOfLight=true;
		bool useDepolarization=false;
		bool usePenetration = false;
		bool useMultiGPU=true;
		bool useFastMath=true;
#ifdef _WIN32
#else 

		std::string usage="Usage: opal [-options] \n  -r Max reflections E \n -p Enable OptiX rtPrintf on device to debug \n -s Use decimal degrees in angular spacing \n -c Use c=3e8 m/s. Default is c=299 792 458 m/s\n -d Enable depolarization \n -a Enable penetration \n -g Disable multiGPU \n -m disable fast_math \n -h Show help";

		int c;
		int nr;
		while ((c = getopt (argc, argv, "r:pscdagmu:h")) != -1) {
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
		//std::unique_ptr<OpalSceneManager> sceneManager(new OpalSceneManager(freq,useExactSpeedOfLight));
		
		//New way to initialize: first create instance
		std::unique_ptr<OpalSceneManager> sceneManager(new OpalSceneManager());
	

		//For multiple transmitters simulateneously
		//std::unique_ptr<OpalSceneManagerMultiTransmitter> sceneManager(new OpalSceneManagerMultiTransmitter());

		//Now set desired features
	
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
		

		//Finally, init context
		sceneManager->initContext(freq,useExactSpeedOfLight);
		
		
		//Now, set additional utilities
		//sceneManager->setUsageReport();
		if (printEnabled) {
			sceneManager->setPrintEnabled(1024 * 1024 * 1024);	
		}

		
		//Finally, run the one test
		sceneManager = cubeTunnel(std::move(sceneManager));
		//sceneManager = polarizationPlaneTest(std::move(sceneManager), printEnabled,subSteps);




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


