/***************************************************************/
//
//Copyright (c) 2019 Esteban Egea-Lopez http://ait.upct.es/eegea
//
/**************************************************************/

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

//Callback
void printPower(float power, int txId) {
	std::cout << "PR\t" << power << std::endl;
}

std::vector<float3>  loadVerticesFromFile(const char* file) {
	std::ifstream infile(file);
	float x, y, z;
	//char c;
	std::vector<float3> vertices;
	std::string line;


	while (std::getline(infile, line)) {

		//std::cout << line << std::endl;
		std::string delimiters = "\t";
		size_t current;
		size_t next = -1;
		int p = 0;
		do
		{
			current = next + 1;
			next = line.find_first_of(delimiters, current);
			if (p == 0) {
				x = std::stof(line.substr(current, next - current));
			}
			if (p == 1) {
				y = std::stof(line.substr(current, next - current));
			}
			if (p == 2) {
				z = std::stof(line.substr(current, next - current));
			}

			//std::cout << line.substr(current, next - current) <<"\t"<< std::endl;
			p++;
		} while (next != std::string::npos);

		vertices.push_back(make_float3(x, y, z));
	}
	std::cout << "Loaded " << vertices.size() << " vertices from " << file << std::endl;
	infile.close();

	return vertices;
}
std::vector<int>  loadTrianglesFromFile(const char* file) {
	std::ifstream infile(file);
	int i;
	std::vector<int> triangles;

	while (infile>>i) {
		//std::cout << i << std::endl;
		triangles.push_back(i);
	}
	std::cout << "Loaded " << triangles.size() << "indices from " << file << std::endl;
	infile.close();
	return triangles;
}


//Adding compund dynamic meshes
std::unique_ptr<OpalSceneManager> addCompoundDynamicMeshes(std::unique_ptr<OpalSceneManager> sceneManager) {
	try {

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
		emProp1.dielectricConstant = make_float2(3.75f, -60.0f*sceneManager->defaultChannel.waveLength*0.038f);

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

		std::function<void(float,int)> cb=&printPower;
		sceneManager->addReceiver(1, posrx, 5.0f, cb);


		sceneManager->finishSceneContext();
		sceneManager->setPrintEnabled(1024 * 1024 * 1024);
		optix::float3 polarization = make_float3(0.0f, 1.0f, 0.0f); //Perpendicular to the floor. Assuming as in Unity that forward is z-axis and up is y-axis

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


		return sceneManager;
	}
	catch (optix::Exception& e) {
		std::cout << "addCompoundDynamicMeshes occurred with error code "
			<< e.getErrorCode() << " and message "
			<< e.getErrorString() << std::endl;

		return 0;
	}
	catch (opal::Exception& e) {
		std::cout << "addCompoundDynamicMeshes occurred with  message "
			<< e.getErrorString()
			<< std::endl;

		return 0;
	}


}





//Adding, moving and removing dynamic meshes
std::unique_ptr<OpalSceneManager> addRemoveDynamicMeshes(std::unique_ptr<OpalSceneManager> sceneManager, bool print, bool subSteps) {
	try {

		//Quad
		int quadind[6] = { 0,1,2,1,0,3 };
		optix::float3 quadv[4] = { make_float3(-0.5f,-0.5f,0.f),make_float3(0.5f,0.5f,0.f) ,make_float3(0.5f,-0.5f,0.f) ,make_float3(-0.5f,0.5f,0.f) };



		Matrix4x4 tm;
		tm.setRow(0, make_float4(1, 0, 0, 0.f));
		tm.setRow(1, make_float4(0, 1, 0, 2.f));
		tm.setRow(2, make_float4(0, 0, 1, 75.f));
		tm.setRow(3, make_float4(0, 0, 0.f, 1));
		MaterialEMProperties emProp1;
		emProp1.dielectricConstant = make_float2(3.75f, -60.0f*sceneManager->defaultChannel.waveLength*0.038f);

		//Creation of dynamic meshes  requires calling these 4 functions
		sceneManager->addDynamicMeshGroup(0);
		sceneManager->addMeshToGroup(0,4, quadv, 6, quadind,  emProp1);  //Call for each new mesh in the group
		sceneManager->updateTransformInGroup(0, tm); 
		sceneManager->finishDynamicMeshGroup(0);

		if (subSteps) {
			sceneManager->createRaySphere2DSubstep(1, 1);
		} else {
			sceneManager->createRaySphere2D(1, 1);
		}
		//receivers
		optix::float3 posrx = make_float3(0.0f, 2.0f, 50.0f);


		optix::float3 postx = make_float3(0.0f, 2.0f, 0.0f);

		sceneManager->addReceiver(0, posrx, 5.0f, printPower);


		sceneManager->finishSceneContext();
		if (print) {
			sceneManager->setPrintEnabled(1024 * 1024 * 1024);
		}
		optix::float3 polarization = make_float3(0.0f, 1.0f, 0.0f); //Perpendicular to the floor. Assuming as in Unity that forward is z-axis and up is y-axis

		sceneManager->transmit(1, 1.0f, postx, polarization);

		Matrix4x4 tm1;
		tm1.setRow(0, make_float4(1.f, 0, 0, 0.0f));
		tm1.setRow(1, make_float4(0, 1, 0, 2.f));
		tm1.setRow(2, make_float4(0, 0, 1.0f, -75.f));
		tm1.setRow(3, make_float4(0, 0, 0.f, 1));
		sceneManager->updateTransformInGroup(0, tm1);

		posrx = make_float3(0.0f, 2.0f, -50.0f);
		sceneManager->updateReceiver(0, posrx);

		std::cout << "Symmetric situation if everything has transformed well. Expect the same power as first transmission. Transmit again" << std::endl;
		sceneManager->transmit(1, 1.0f, postx, polarization);

		//Add a new quad 
		sceneManager->addDynamicMeshGroup(1);
		sceneManager->addMeshToGroup(1, 4, quadv, 6, quadind, emProp1);
		sceneManager->updateTransformInGroup(1, tm);
		sceneManager->finishDynamicMeshGroup(1);

		std::cout << "Transmit with new quad. Num quads= "<< sceneManager->dynamicMeshes.size() << std::endl;
		sceneManager->transmit(1, 1.0f, postx, polarization);

		//Remove first quad
		sceneManager->removeDynamicMeshGroup(0);

		posrx = make_float3(0.0f, 2.0f, 50.0f);
		sceneManager->updateReceiver(0, posrx);
		std::cout << "Removing first quad. Expect again the first power. Transmit again.  Num quads= " << sceneManager->dynamicMeshes.size() << std::endl;
		Matrix4x4 mym;
		Matrix4x4 mymi;
		sceneManager->dynamicMeshes.at(1)->transform->getMatrix(0, mym.getData(), mymi.getData());
		std::cout << "Tm of quad 1: " <<  mym<< std::endl;
		sceneManager->transmit(1, 1.0f, postx, polarization);

		//Remove second quad
		sceneManager->removeDynamicMeshGroup(1);

		return sceneManager;
	}
	catch (optix::Exception& e) {
		std::cout << "addRemoveDynamicMeshes occurred with error code "
			<< e.getErrorCode() << " and message "
			<< e.getErrorString() << std::endl;

		return 0;
	}
	catch (opal::Exception& e) {
		std::cout << "addRemoveDynamicMeshes occurred with  message "
			<< e.getErrorString()
			<< std::endl;

		return 0;
	}


}


//Adding and removing dynamic meshes
std::unique_ptr<OpalSceneManager> addRemoveReceivers(std::unique_ptr<OpalSceneManager> sceneManager) {
	try {
		//Horizontal plane
		std::vector<int> planeind = loadTrianglesFromFile("meshes/tri.txt");
		std::vector<float3> planever = loadVerticesFromFile("meshes/vert.txt");
		//std::cout << "indices=" << planeind.size() << "vertices=" << planever.size() << std::endl;
		Matrix4x4 tm;
		tm.setRow(0, make_float4(10.0f, 0, 0, 0.0f));
		tm.setRow(1, make_float4(0, 1, 0, 0.0f));
		tm.setRow(2, make_float4(0, 0, 10.0f, 50.0f));
		tm.setRow(3, make_float4(0, 0, 0, 1));
		MaterialEMProperties emProp1;
		emProp1.dielectricConstant = make_float2(3.75f, -60.0f*sceneManager->defaultChannel.waveLength*0.038f);
		sceneManager->addStaticMesh(static_cast<int>(planever.size()), planever.data(), static_cast<int>(planeind.size()), planeind.data(), tm, emProp1);


		sceneManager->createRaySphere2D(1, 1);
		//sceneManager->createRaySphere2DSubstep(1, 1);
		//receivers
		optix::float3 posrx = make_float3(0.0f, 2.0f, 100.0f);


		optix::float3 postx = make_float3(0.0f, 2.0f, 50.0f);

		sceneManager->addReceiver(1, posrx, 5.0f, printPower);


		sceneManager->finishSceneContext();
		sceneManager->setPrintEnabled(1024 * 1024 * 1024);
		optix::float3 polarization = make_float3(0.0f, 1.0f, 0.0f); //Perpendicular to the floor. Assuming as in Unity that forward is z-axis and up is y-axis
		sceneManager->transmit(0, 1.0f, postx, polarization);

		//Add new receiver
		posrx = make_float3(0.0f, 2.0f, 70.0f);
		sceneManager->addReceiver(2, posrx, 5.0f, printPower);

		std::cout << "transmit again" << std::endl;
		sceneManager->transmit(0, 1.0f, postx, polarization);


		//Remove receiver
		sceneManager->removeReceiver(2);
		std::cout << "transmit again" << std::endl;
		sceneManager->transmit(0, 1.0f, postx, polarization);
		//std::cout << "Launching" << std::endl;

		return sceneManager;
	}
	catch (optix::Exception& e) {
		std::cout << "addRemoveReceivers occurred with error code "
			<< e.getErrorCode() << " and message "
			<< e.getErrorString() << std::endl;

		return 0;
	}
	catch (opal::Exception& e) {
		std::cout << "addRemoveReceivers occurred with  message "
			<< e.getErrorString()
			<< std::endl;

		return 0;
	}


}



//moving receivers
std::unique_ptr<OpalSceneManager> moveReceivers(std::unique_ptr<OpalSceneManager> sceneManager) {
	try {
		//Horizontal plane
		std::vector<int> planeind = loadTrianglesFromFile("D:\\tri.txt");
		std::vector<float3> planever = loadVerticesFromFile("D:\\vert.txt");
		//std::cout << "indices=" << planeind.size() << "vertices=" << planever.size() << std::endl;
		Matrix4x4 tm;
		tm.setRow(0, make_float4(10.0f, 0, 0, 0.0f));
		tm.setRow(1, make_float4(0, 1, 0, 0.0f));
		tm.setRow(2, make_float4(0, 0, 10.0f, 50.0f));
		tm.setRow(3, make_float4(0, 0, 0, 1));
		MaterialEMProperties emProp1;
		emProp1.dielectricConstant = make_float2(3.75f, -60.0f*sceneManager->defaultChannel.waveLength*0.038f);
		sceneManager->addStaticMesh(static_cast<int>(planever.size()), planever.data(), static_cast<int>(planeind.size()), planeind.data(), tm, emProp1);


		//sceneManager->createRaySphere2D(1, 1);
		sceneManager->createRaySphere2DSubstep(1, 1);
		//receivers
		optix::float3 postx = make_float3(0.0f, 2.0f, 100.0f);


		optix::float3 posrx = make_float3(0.0f, 2.0f, 0.0f);
		optix::float3 polarization = make_float3(0.0f, 1.0f, 0.0f); //Perpendicular to the floor. Assuming as in Unity that forward is z-axis and up is y-axis

		sceneManager->addReceiver(1, posrx, 5.0f, printPower);


		sceneManager->finishSceneContext();
		sceneManager->setPrintEnabled(1024 * 1024 * 1024);

		for (size_t i = 0; i < 100; ++i)
		{
			posrx = make_float3(0.0f, 2.0f, 99.0f - i);
			sceneManager->updateReceiver(1, posrx);
			//postx = make_float3(0.0f, 2.0f, 0.f);
			sceneManager->transmit(0, 1.0f, postx, polarization);
			//	postx = make_float3(0.0f, 2.0f, 1.f);
			//sceneManager->transmit(0, 1.0f, postx, polarization);

		}




		return sceneManager;
	}
	catch (optix::Exception& e) {
		std::cout << "moveReceivers occurred with error code "
			<< e.getErrorCode() << " and message "
			<< e.getErrorString() << std::endl;

		return 0;
	}
	catch (opal::Exception& e) {
		std::cout << "moveReceivers occurred with  message "
			<< e.getErrorString()
			<< std::endl;

		return 0;
	}


}

std::unique_ptr<OpalSceneManagerMultiTransmitter> crossingTestMulti(std::unique_ptr<OpalSceneManagerMultiTransmitter> sceneManager, bool print, bool subSteps) {

	Timer timer;

	std::cout << "Simulating crossing streets test with multitransmitter" << std::endl;
	//Cubes
	std::vector<int> cubeind = loadTrianglesFromFile("meshes/tricube.txt");
	std::vector<float3> cubevert = loadVerticesFromFile("meshes/vertcube.txt");
	//std::cout << "indices=" << cubeind.size() << "vertices=" << cubevert.size() << std::endl;
	//Cube(4) NW
	Matrix4x4 tm;
	tm.setRow(0, make_float4(40.0f, 0, 0, -30.0f));
	tm.setRow(1, make_float4(0, 40.0f, 0, 20.0f));
	tm.setRow(2, make_float4(0, 0, 40.0f, 80.0f));
	tm.setRow(3, make_float4(0, 0, 0, 1));
	MaterialEMProperties emProp1;
	emProp1.dielectricConstant = make_float2(3.75f, -60.0f*sceneManager->defaultChannel.waveLength*0.038f);
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
	std::vector<int> planeind = loadTrianglesFromFile("meshes/tri.txt");
	std::vector<float3> planever = loadVerticesFromFile("meshes/vert.txt");
	//std::cout << "indices=" << planeind.size() << "vertices=" << planever.size() << std::endl;

	tm.setRow(0, make_float4(10.0f, 0, 0, 0.0f));
	tm.setRow(1, make_float4(0, 1, 0, 0.0f));
	tm.setRow(2, make_float4(0, 0, 10.0f, 50.0f));
	tm.setRow(3, make_float4(0, 0, 0, 1));

	//emProp1.dielectricConstant = make_float2(3.75f, -60.0f*sceneManager->defaultChannel.waveLength*0.15f);
	std::cout << "Adding Plane. Em=" << emProp1.dielectricConstant << std::endl;
	sceneManager->addStaticMesh(static_cast<int>(planever.size()), planever.data(), static_cast<int>(planeind.size()), planeind.data(), tm, emProp1);

	if (subSteps) {
		sceneManager->createRaySphere2DSubstep(1, 1); //0.1 degree delta step
	} else {
		sceneManager->createRaySphere2D(1, 1); //1 degree delta step
	}

	//receivers: symmetrical configuration, power must be almost equal

	optix::float3 posrx = make_float3(0.0f, 10.0f, 100.0f);
	sceneManager->addReceiver(1, posrx, 5.0f, printPower);
	posrx = make_float3(0.0f, 10.0f, 0.0f);
	sceneManager->addReceiver(3, posrx, 5.0f, printPower);


	//sceneManager->setMaxReflections(3u);
	sceneManager->finishSceneContext();

	if (print) {
		sceneManager->setPrintEnabled(1024 * 1024 * 1024);	
	}
	//sceneManager->setUsageReport();

	optix::float3 postx=make_float3(0.0f,0.0f,0.f);
	optix::float3 polarization = make_float3(0.0f, 1.0f, 0.0f); //Perpendicular to the floor. Assuming as in Unity that forward is z-axis and up is y-axis
	timer.start();
		sceneManager->registerTransmitter(0,postx,polarization,1.0f);
		sceneManager->registerTransmitter(2,postx,polarization,1.0f);

	for (int i = -50; i <= 50; ++i) {

		float x=i;
		postx = make_float3(x, 10.f, 50.0f);
		sceneManager->addTransmitterToGroup(0,1.0f,postx,polarization);
		sceneManager->addTransmitterToGroup(2,1.0f,postx,polarization);
		sceneManager->groupTransmit();
		//sceneManager->transmit(0, 1.0f, postx, polarization);


	}
	timer.stop();
	std::cout<<"Time="<<timer.getTime()<<std::endl;
	//		postx = make_float3(-18.0f, 10.0f, 50.0f);
	//		sceneManager->transmit(0, 1.0f, postx, polarization);

	return sceneManager;

}



//Street crossing test. Cubes are intended to be buildings and a plane is the floor
std::unique_ptr<OpalSceneManager> crossingTest(std::unique_ptr<OpalSceneManager> sceneManager, bool print, bool subSteps) {

	Timer timer;

	std::cout << "Simulating crossing streets test" << std::endl;
	//Cubes
	std::vector<int> cubeind = loadTrianglesFromFile("meshes/tricube.txt");
	std::vector<float3> cubevert = loadVerticesFromFile("meshes/vertcube.txt");
	//std::cout << "indices=" << cubeind.size() << "vertices=" << cubevert.size() << std::endl;
	//Cube(4) NW
	Matrix4x4 tm;
	tm.setRow(0, make_float4(40.0f, 0, 0, -30.0f));
	tm.setRow(1, make_float4(0, 40.0f, 0, 20.0f));
	tm.setRow(2, make_float4(0, 0, 40.0f, 80.0f));
	tm.setRow(3, make_float4(0, 0, 0, 1));
	MaterialEMProperties emProp1;
	emProp1.dielectricConstant = make_float2(3.75f, -60.0f*sceneManager->defaultChannel.waveLength*0.038f);
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
	std::vector<int> planeind = loadTrianglesFromFile("meshes/tri.txt");
	std::vector<float3> planever = loadVerticesFromFile("meshes/vert.txt");
	//std::cout << "indices=" << planeind.size() << "vertices=" << planever.size() << std::endl;

	tm.setRow(0, make_float4(10.0f, 0, 0, 0.0f));
	tm.setRow(1, make_float4(0, 1, 0, 0.0f));
	tm.setRow(2, make_float4(0, 0, 10.0f, 50.0f));
	tm.setRow(3, make_float4(0, 0, 0, 1));

	//emProp1.dielectricConstant = make_float2(3.75f, -60.0f*sceneManager->defaultChannel.waveLength*0.15f);
	std::cout << "Adding Plane. Em=" << emProp1.dielectricConstant << std::endl;
	sceneManager->addStaticMesh(static_cast<int>(planever.size()), planever.data(), static_cast<int>(planeind.size()), planeind.data(), tm, emProp1);

	if (subSteps) {
		sceneManager->createRaySphere2DSubstep(1, 1); //0.1 degree delta step
	} else {
		sceneManager->createRaySphere2D(1, 1); //1 degree delta step
	}

	//receivers

	optix::float3 posrx = make_float3(0.0f, 10.0f, 100.0f);
	sceneManager->addReceiver(1, posrx, 5.0f, printPower);


	//sceneManager->setMaxReflections(3u);

	sceneManager->finishSceneContext();

	if (print) {
		sceneManager->setPrintEnabled(1024 * 1024 * 1024);	
	}
	//sceneManager->setUsageReport();

	optix::float3 postx;
	optix::float3 polarization = make_float3(0.0f, 1.0f, 0.0f); //Perpendicular to the floor. Assuming as in Unity that forward is z-axis and up is y-axis
	timer.start();

	for (int i = -50; i <= 50; ++i) {

		float x=i;
		postx = make_float3(x, 10.f, 50.0f);

		sceneManager->transmit(0, 1.0f, postx, polarization);


	}
	timer.stop();
	std::cout<<"Time="<<timer.getTime()<<std::endl;
	//		postx = make_float3(-18.0f, 10.0f, 50.0f);
	//		sceneManager->transmit(0, 1.0f, postx, polarization);

	return sceneManager;

}


//Horizontal plane test. To validate against a two-ray model
std::unique_ptr<OpalSceneManager> planeTest(std::unique_ptr<OpalSceneManager> sceneManager, bool print, bool subSteps) {
	//Horizontal plane
	std::vector<int> planeind = loadTrianglesFromFile("meshes/tri.txt");
	std::vector<float3> planever = loadVerticesFromFile("meshes/vert.txt");
	//std::cout << "indices=" << planeind.size() << "vertices=" << planever.size() << std::endl;
	Matrix4x4 tm;
	tm.setRow(0, make_float4(10.0f, 0, 0, 0.0f));
	tm.setRow(1, make_float4(0, 1, 0, 0.0f));
	tm.setRow(2, make_float4(0, 0, 10.0f, 50.0f));
	tm.setRow(3, make_float4(0, 0, 0, 1));
	MaterialEMProperties emProp1;
	emProp1.dielectricConstant = make_float2(3.75f, -60.0f*sceneManager->defaultChannel.waveLength*0.038f);
	sceneManager->addStaticMesh(static_cast<int>(planever.size()), planever.data(), static_cast<int>(planeind.size()), planeind.data(), tm, emProp1);


	if (subSteps) {
		sceneManager->createRaySphere2DSubstep(1, 1);
	} else {
		sceneManager->createRaySphere2D(1, 1);
	}
	//receivers
	optix::float3 posrx = make_float3(0.0f, 10.0f, 100.0f);
	//optix::float3 posrx2 = make_float3(10.0f, 10.0f, 100.0f);


	optix::float3 postx = make_float3(0.0f, 2.0f, 50.0f);

	sceneManager->addReceiver(1, posrx, 1.0f, printPower);
	//sceneManager->addReceiver(2, posrx2, 1.0f, printPower);


	sceneManager->finishSceneContext();

	if (print) {
		sceneManager->setPrintEnabled(1024 * 1024 * 1024);
	}
	//	sceneManager->setUsageReport();


	//std::cout << "Launching" << std::endl;


	optix::float3 polarization = make_float3(0.0f, 1.0f, 0.0f); //Perpendicular to the floor. Assuming as in Unity that forward is z-axis and up is y-axis


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
	for (size_t i = 0; i < 100; ++i)
	{
		postx = make_float3(0.0f, 2.0f, 99.0f - i);
		sceneManager->transmit(0, 1.0f, postx, polarization);

	}


	return sceneManager;
}


//Two quads as walls and two overlapping receivers
std::unique_ptr<OpalSceneManager> quadTest(std::unique_ptr<OpalSceneManager> sceneManager, bool print, bool subSteps) {
	//First quad
	int quadind[6] = { 0,1,2,1,0,3 };
	optix::float3 quadv[4] = { make_float3(-0.5f,-0.5f,0.f),make_float3(0.5f,0.5f,0.f) ,make_float3(0.5f,-0.5f,0.f) ,make_float3(-0.5f,0.5f,0.f) };

	//One quad at (0,0,100)

	Matrix4x4 tm;
	tm.setRow(0, make_float4(1, 0, 0, 0.f));
	tm.setRow(1, make_float4(0, 1, 0, 0.f));
	tm.setRow(2, make_float4(0, 0, 1, 100.0f));
	tm.setRow(3, make_float4(0, 0, 0,  1));
	MaterialEMProperties emProp1;
	emProp1.dielectricConstant = make_float2(3.75f, -60.0f*sceneManager->defaultChannel.waveLength*0.038f);

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
	emProp2.dielectricConstant = make_float2(3.75f, -60.0f*sceneManager->defaultChannel.waveLength*0.038f);


	//sceneManager->addStaticMesh(4, quadv2, 6, quadind2, tm2, emProp2);

	optix::float3 posrx = make_float3(0.f, 0.f, 97.0f);
	//sceneManager->addReceiver(1, posrx, 1.0f, printPower);
	sceneManager->addReceiver(1, posrx, 5.0f, printPower);
	posrx=make_float3(0.0f,0.0f,99.0f);
	sceneManager->addReceiver(2, posrx, 5.0f, printPower);
	optix::float3 postx = make_float3(0.0f, 0.f,0.f);

	optix::float3 polarization = make_float3(0.0f, 1.0f, 0.0f); //Perpendicular to the floor. Assuming as in Unity that forward is z-axis and up is y-axis
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
	sceneManager->setUsageReport();
	sceneManager->transmit(0, 1.0f,postx, polarization);
	return sceneManager;
}



//Street crossing test. Cubes are intended to be buildings and a plane is the floor. A complex vehicle mesh is moved
std::unique_ptr<OpalSceneManager> crossingTestAndVehicle(std::unique_ptr<OpalSceneManager> sceneManager) {
	//Cubes
	std::vector<int> cubeind = loadTrianglesFromFile("meshes/tricube.txt");
	std::vector<float3> cubevert = loadVerticesFromFile("meshes/vertcube.txt");
	//std::cout << "indices=" << cubeind.size() << "vertices=" << cubevert.size() << std::endl;
	//Cube(4) NW
	Matrix4x4 tm;
	tm.setRow(0, make_float4(40.0f, 0, 0, -30.0f));
	tm.setRow(1, make_float4(0, 40.0f, 0, 20.0f));
	tm.setRow(2, make_float4(0, 0, 40.0f, 80.0f));
	tm.setRow(3, make_float4(0, 0, 0, 1));
	MaterialEMProperties emProp1;
	emProp1.dielectricConstant = make_float2(3.75f, -60.0f*sceneManager->defaultChannel.waveLength*0.038f);
	//emProp1.dielectricConstant = make_float2(3.75f, -0.4576f);
	std::cout << "Adding NW. Em=" << emProp1.dielectricConstant << std::endl;
	sceneManager->addStaticMesh(static_cast<int>(cubevert.size()), cubevert.data(), static_cast<int>(cubeind.size()), cubeind.data(), tm, emProp1);

	//Cube SW
	tm.setRow(0, make_float4(40.0f, 0, 0, -30.0f));
	tm.setRow(1, make_float4(0, 40.0f, 0, 20.0f));
	tm.setRow(2, make_float4(0, 0, 40.0f, 20.0f));
	tm.setRow(3, make_float4(0, 0, 0, 1));
	std::cout << "Adding SW. Em = " << emProp1.dielectricConstant << std::endl;
	sceneManager->addStaticMesh(static_cast<int>(cubevert.size()), cubevert.data(), static_cast<int>(cubeind.size()), cubeind.data(), tm, emProp1);
	//Cube(2) NE

	tm.setRow(0, make_float4(40.0f, 0, 0, 30.0f));
	tm.setRow(1, make_float4(0, 40.0f, 0, 20.0f));
	tm.setRow(2, make_float4(0, 0, 40.0f, 80.0f));
	tm.setRow(3, make_float4(0, 0, 0, 1));
	std::cout << "Adding NE. Em = " << emProp1.dielectricConstant << std::endl;
	sceneManager->addStaticMesh(static_cast<int>(cubevert.size()), cubevert.data(), static_cast<int>(cubeind.size()), cubeind.data(), tm, emProp1);

	//Cube(1) SE

	tm.setRow(0, make_float4(40.0f, 0, 0, 30.0f));
	tm.setRow(1, make_float4(0, 40.0f, 0, 20.0f));
	tm.setRow(2, make_float4(0, 0, 40.0f, 20.0f));
	tm.setRow(3, make_float4(0, 0, 0, 1));
	std::cout << "Adding SE. Em = " << emProp1.dielectricConstant << std::endl;
	sceneManager->addStaticMesh(static_cast<int>(cubevert.size()), cubevert.data(), static_cast<int>(cubeind.size()), cubeind.data(), tm, emProp1);

	//Horizontal plane
	std::vector<int> planeind = loadTrianglesFromFile("meshes/tri.txt");
	std::vector<float3> planever = loadVerticesFromFile("meshes/vert.txt");
	//std::cout << "indices=" << planeind.size() << "vertices=" << planever.size() << std::endl;

	tm.setRow(0, make_float4(10.0f, 0, 0, 0.0f));
	tm.setRow(1, make_float4(0, 1, 0, 0.0f));
	tm.setRow(2, make_float4(0, 0, 10.0f, 50.0f));
	tm.setRow(3, make_float4(0, 0, 0, 1));

	//emProp1.dielectricConstant = make_float2(3.75f, -60.0f*sceneManager->defaultChannel.waveLength*0.15f);
	std::cout << "Adding Plane. Em=" << emProp1.dielectricConstant << std::endl;
	sceneManager->addStaticMesh(static_cast<int>(planever.size()), planever.data(), static_cast<int>(planeind.size()), planeind.data(), tm, emProp1);



	//Create vehicle group
	std::vector<int> bodyi = loadTrianglesFromFile("meshes/CC_ME_Body_R4-i.txt");
	std::vector<float3> bodyv = loadVerticesFromFile("meshes/CC_ME_Body_R4-v.txt");
	std::vector<int> wheeli = loadTrianglesFromFile("meshes/CC_ME_Wheel_FL-i.txt");
	std::vector<float3> wheelv = loadVerticesFromFile("meshes/CC_ME_Wheel_FL-v.txt");


	//Creation of dynamic meshes  requires calling these 4 functions
	sceneManager->addDynamicMeshGroup(0);
	sceneManager->addMeshToGroup(0, static_cast<int>(bodyv.size()), bodyv.data(), static_cast<int>(bodyi.size()),bodyi.data(), emProp1);  //Call for each new mesh in the group
	sceneManager->addMeshToGroup(0, static_cast<int>(wheelv.size()), wheelv.data(), static_cast<int>(wheeli.size()), wheeli.data(),emProp1);  //Call for each new mesh in the group
	wheeli = loadTrianglesFromFile("meshes/CC_ME_Wheel_FR-i.txt");
	wheelv = loadVerticesFromFile("meshes/CC_ME_Wheel_FR-v.txt");
	sceneManager->addMeshToGroup(0, static_cast<int>(wheelv.size()), wheelv.data(), static_cast<int>(wheeli.size()), wheeli.data(), emProp1);  //Call for each new mesh in the group

	wheeli = loadTrianglesFromFile("meshes/CC_ME_Wheel_BL-i.txt");
	wheelv = loadVerticesFromFile("meshes/CC_ME_Wheel_BL-v.txt");
	sceneManager->addMeshToGroup(0, static_cast<int>(wheelv.size()), wheelv.data(), static_cast<int>(wheeli.size()), wheeli.data(), emProp1);  //Call for each new mesh in the group
	wheeli = loadTrianglesFromFile("meshes/CC_ME_Wheel_BR-i.txt");
	wheelv = loadVerticesFromFile("meshes/CC_ME_Wheel_BR-v.txt");
	sceneManager->addMeshToGroup(0, static_cast<int>(wheelv.size()), wheelv.data(), static_cast<int>(wheeli.size()), wheeli.data(), emProp1);  //Call for each new mesh in the group


	tm.setRow(0, make_float4(0.0f, 0.f, 1.0f, -50.0f));
	tm.setRow(1, make_float4(0.f, 1.0f, 0.f, 0.6f));
	tm.setRow(2, make_float4(-1.f, 0.f, 0.0f, 50.0f));
	tm.setRow(3, make_float4(0, 0, 0, 1));

	/*
	   tm.setRow(0, make_float4(0.0f, 0.f, 0.f, -50.0f));
	   tm.setRow(1, make_float4(0.f, 1.0f, 0.f, 0.6f));
	   tm.setRow(2, make_float4(-1.f, 0.f, 0.0f, 50.0f));
	   tm.setRow(3, make_float4(0, 0, 0, 1));

*/
	sceneManager->updateTransformInGroup(0, tm);
	sceneManager->finishDynamicMeshGroup(0);



	//sceneManager->createRaySphere2D(1, 1);
	sceneManager->createRaySphere2DSubstep(1, 1);

	//receivers

	optix::float3 posrx = make_float3(0.0f, 2.0f, 100.0f);
	sceneManager->addReceiver(1, posrx, 1.f, printPower);


	//sceneManager->setMaxReflections(3u);

	sceneManager->finishSceneContext();


	sceneManager->setPrintEnabled(1024 * 1024 * 1024);


	optix::float3 postx;
	optix::float3 polarization = make_float3(0.0f, 1.0f, 0.0f); //Perpendicular to the floor. Assuming as in Unity that forward is z-axis and up is y-axis

	for (int i = -50; i <= 50; ++i)
	{
		postx = make_float3(i-2, 1.1f, 50.0f);//On top of the vehicle
		tm.setRow(0, make_float4(-1.19209e-07f, 0.f, 1.0f, i));
		tm.setRow(2, make_float4(-1.f, 0.f, -1.19209e-07f, 50.0f));
		//std::cout << "tm=" << tm << std::endl;
		sceneManager->updateTransformInGroup(0, tm);
		sceneManager->transmit(0, 1.0f, postx, polarization);


	}
	//postx = make_float3(-50.0f, 1.43f, 50.0f); //On top of the vehicle
	//postx = make_float3(-50.0f, 3.f, 50.0f); //On top of the vehicle
	//sceneManager->transmit(0, 1.0f, postx, polarization);

	return sceneManager;

}






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
#ifdef _WIN32
#else 

		std::string usage="Usage: opal [-options] \n  -r Max reflections E \n -p Enable OptiX rtPrintf on device to debug \n -s Use decimal degrees in angular spacing \n -c Use c=3e8 m/s. Default is c=299 792 458 m/s \n -h Show help";

		int c;
		int nr;
		while ((c = getopt (argc, argv, "r:psch")) != -1) {
			switch (c) {
				case 'r':
					std::cout<<optarg<<std::endl;
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
		std::unique_ptr<OpalSceneManagerMultiTransmitter> sceneManager(new OpalSceneManagerMultiTransmitter(freq,useExactSpeedOfLight));

		sceneManager->setMaxReflections(maxReflections);
		//sceneManager->setUsageReport();



		//sceneManager = crossingTestAndVehicle(std::move(sceneManager));
		//sceneManager = addRemoveDynamicMeshes(std::move(sceneManager), printEnabled, subSteps);
		//sceneManager = addCompoundDynamicMeshes(std::move(sceneManager));
		//sceneManager = addRemoveReceivers(std::move(sceneManager));

		//sceneManager = planeTest(std::move(sceneManager), printEnabled, subSteps);
		//sceneManager = moveReceivers(std::move(sceneManager));
		//sceneManager = crossingTest(std::move(sceneManager), printEnabled,subSteps);
		//sceneManager = quadTest(std::move(sceneManager),printEnabled,subSteps);
		sceneManager = crossingTestMulti(std::move(sceneManager), printEnabled,subSteps);





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


