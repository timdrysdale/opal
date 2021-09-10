/***************************************************************/
//
//Copyright (c) 2019 Esteban Egea-Lopez http://girtel.upct.es/~eegea
//
/**************************************************************/


#include "Unity.h"
#include <iostream>
#include <stdlib.h>
#include "basicSimulation.h"
#include "flatSimulation.h"
#include "curvedMeshSimulation.h"
#include "curvedFlatMeshSimulation.h"
#include "rayDensityNormalizationSimulation.h"
#include "singleDiffraction.h"

#define CHM() if (sceneManager==nullptr) return OPAL_NO_SCENE_MANAGER; //Check manager


using namespace optix;
namespace opal {

	int handle() {
		try {
			throw;
		}
		catch (opal::Exception& e) {
			//std::ofstream myfile;
			//myfile.open(logFile, std::ifstream::app);
			std::cout << "Init: error occurred with message " << e.getErrorString() << std::endl;
			//myfile.close();

			return OPAL_EXCEPTION;
		}
		catch (optix::Exception& e) {
			//std::ofstream myfile;
			//myfile.open(logFile, std::ifstream::app);
			std::cout << "Init: error occurred with error code "
				<< e.getErrorCode() << " and message "
				<< e.getErrorString() << std::endl;
			//myfile.close();
			return OPTIX_EXCEPTION;
		}

	}

	void printMatrix(opal::UnityMatrix4x4& u, std::ofstream& s) {
		s << "UnityMatrix4x4 -----" << std::endl;

		s << u.m00 << "\t" << u.m01 << "\t" << u.m02 << "\t" << u.m03 << std::endl << u.m10 << "\t" << u.m11 << "\t" << u.m12 << "\t" << u.m13 << std::endl << u.m20 << "\t" << u.m21 << "\t" << u.m22 << "\t" << u.m23 << std::endl << u.m30 << "\t" << u.m31 << "\t" << u.m32 << "\t" << u.m33;

		s << "---" << std::endl;
	}
	void UnityToOptixMatrix4x4(optix::Matrix4x4& m, opal::UnityMatrix4x4& u)
	{

		float4 row0 = make_float4(u.m00, u.m01, u.m02, u.m03);
		float4 row1 = make_float4(u.m10, u.m11, u.m12, u.m13);
		float4 row2 = make_float4(u.m20, u.m21, u.m22, u.m23);
		float4 row3 = make_float4(u.m30, u.m31, u.m32, u.m33);
		m.setRow(0, row0);
		m.setRow(1, row1);
		m.setRow(2, row2);
		m.setRow(3, row3);

	}

	//Initializes Optix.
	//Note that CUDA programs are compiled in execution time with NVRTC. They are read from the OPTIX_SAMPLES_SDK_DIR location as set here (see below). See also sutil.cpp in the Optix SDK, line 848 and 335
	//In that directory you should put all the .cu and the Common.h files. They can be changed without recompiling all the plugin.
	//Any change made in the .cu files in VS is ignored unless copied to that location.

	//When building an executable with Unity, you have either to use a specific script to create the cudaDir and copy the .cu and .h files to it, or just create and copy manually after the build is done
	//In the end, along with the Unity executable and files you need to provide this specific folder and files some way
	OPAL_API int Init(float frequency,int simType, int mode, bool useExactSpeedOfLight, bool useDiffraction, bool enableFastMath, bool generateRaysOnLaunch, bool enableMultiGPU, bool logTrace, bool enableMultitransmitter, bool useAntennaGain, unsigned int maxReflections, float minEpsilon )
	{
		try {
			//First, set the environment to read the CUDA program files from our Plugins directory, in case we use sutil. This is actually not necessary for our programs since we use the ptxHandler

#ifdef _WIN32
			_putenv_s("OPTIX_SAMPLES_SDK_DIR", pluginDir);

#else
			setenv("OPTIX_SAMPLES_SDK_DIR", pluginDir, 1);
#endif // _WIN32


			sceneManager = new OpalSceneManager();
			//TODO: add features here
			sceneManager->setBaseDir(pluginDir);
			if (!useExactSpeedOfLight) {
				sceneManager->useApproximateSpeedLight();
			}
			if (generateRaysOnLaunch) {
				sceneManager->enableGenerateRaysOnLaunch();
			}
			if (!enableMultiGPU) {
				sceneManager->disableMultiGPU();	
			}
			if (!enableFastMath) {
				sceneManager->disableFastMath();
			}
			if (enableMultitransmitter) {
				sceneManager->enableMultitransmitter();
			}	
			sceneManager->setUseAntennaGain(useAntennaGain);
			sceneManager->setMaxReflections(maxReflections);
			sceneManager->setMinEpsilon(minEpsilon);
			OpalSimulation* sim;
			switch (simType) {	
				case 0:
					sim= new BasicFlatMeshReflectionSimulation(sceneManager);
					break;
				case 1:
					sim= new LPFlatMeshReflectionSimulation(sceneManager);
					break;
				case 2: 
					sim= new RayDensityNormalizationSimulation(sceneManager);
					break;
				case 3: 
					sim= new SingleDiffraction(sceneManager);
					break;
			}

			ComputeMode computeMode= ComputeMode::VOLTAGE;
			if (mode==1) {
				computeMode= ComputeMode::FIELD;
			}
			sceneManager->setSimulation(sim);
			sim->setComputeMode(computeMode);
			sim->setEnableTraceLog(logTrace);
			if (useDiffraction) {
				if (simType != 3) {
					SingleDiffraction* simd= new SingleDiffraction(sceneManager);
					sceneManager->setSimulation(simd);
					simd->setComputeMode(computeMode);
					simd->setEnableTraceLog(logTrace);
				}
			}
			sceneManager->initContext(frequency);





			return 0;

		}
		catch (opal::Exception& e) {
#ifdef OPALDEBUG
			std::ofstream myfile;
			myfile.open(logFile, std::ifstream::app);
			myfile << "Init: error occurred with message " << e.getErrorString() << std::endl;
			myfile.close();
#endif // OPALDEBUG

			return OPAL_EXCEPTION;
		}
		catch (optix::Exception& e) {
			std::cout << "Init: error occurred with error code "
				<< e.getErrorCode() << " and message "
				<< e.getErrorString() << std::endl;
#ifdef OPALDEBUG
			std::ofstream myfile;
			myfile.open(logFile, std::ifstream::app);
			myfile << "Init: error occurred with error code "
				<< e.getErrorCode() << " and message "
				<< e.getErrorString() << std::endl;
			myfile.close();
#endif // OPALDEBUG
			return OPTIX_EXCEPTION;
		}

	}
	OPAL_API int Exit()
	{
		try {

			if (sceneManager != nullptr) {

				delete sceneManager;
				sceneManager = nullptr;
			}
			return 0;
		}
		catch (...) {
			handle();
		}
	}
	OPAL_API int SetMaxReflections(unsigned int m)
	{
		CHM();
		sceneManager->setMaxReflections(m);


		return 0;
	}
	OPAL_API int EnablePenetration() {
		CHM();
		sceneManager->enablePenetration();
		return 0;
	}
	OPAL_API int DisablePenetration() {
		CHM();
		sceneManager->disablePenetration();
		return 0;
	}
	OPAL_API int SetAttenuationLimit(float l) {
		CHM();
		sceneManager->setAttenuationLimit(l);
		return 0;
	}
	OPAL_API int AddEdge(optix::float3 p, optix::float3 v, unsigned int faid, unsigned int fbid, optix::float3 face_a, optix::float3 face_b, optix::float3 normal_a, optix::float3 normal_b, UnityMaterialEMProperties emProp, int id) {
		CHM();
		try {
			MaterialEMProperties prop = sceneManager->ITUparametersToMaterial(emProp.a,emProp.b,emProp.c,emProp.d);
			optix::uint2 faces=make_uint2(faid,fbid);
			sceneManager->addEdge(p, v, faces, face_a,  face_b,  normal_a,  normal_b,  prop, id);
			return 0;
		} catch (...) {
			handle();
		}
	}
	OPAL_API int AddEdgeToGroup(optix::float3 p, optix::float3 v, unsigned int faid, unsigned int fbid, optix::float3 face_a, optix::float3 face_b, optix::float3 normal_a, optix::float3 normal_b, UnityMaterialEMProperties emProp, int id, int groupId) {
		CHM();
		try {
			MaterialEMProperties prop = sceneManager->ITUparametersToMaterial(emProp.a,emProp.b,emProp.c,emProp.d);
			optix::uint2 faces=make_uint2(faid,fbid);
			sceneManager->addEdgeToGroup(p, v, faces, face_a,  face_b,  normal_a,  normal_b,  prop, id, groupId);
			return 0;
		} catch (...) {
			handle();
		}
	}
	OPAL_API int AddStaticMeshWithFacesFromUnity(int meshVertexCount, optix::float3* meshVertices, int meshTriangleCount, int* meshTriangles, int faceIdCount, int* faceIds,  UnityMatrix4x4 transformationMatrix, UnityMaterialEMProperties emProp) {
		CHM();
		try {
			optix::Matrix4x4 translationMatrix;
			UnityToOptixMatrix4x4(translationMatrix, transformationMatrix);
			//When transforming to MaterialEMProperties conductivity is multiplied by -60*wavelenght
			MaterialEMProperties prop = sceneManager->ITUparametersToMaterial(emProp.a,emProp.b,emProp.c,emProp.d);
			std::vector<std::pair<optix::int3, unsigned int>> triangleIndexFaceBuffer;
			//Lots of copying again, but it is supposed to be done during intialization
			for (int i=0; i<meshTriangleCount; i=i+3) {
				optix::int3 tri=make_int3(meshTriangles[i],meshTriangles[i+1],meshTriangles[i+2]);
				triangleIndexFaceBuffer.push_back(std::pair<optix::int3, unsigned int>(tri,static_cast<unsigned int>(faceIds[i])));
				
				
			}
			std::vector<optix::float3> v;
			for (int j=0; j<meshVertexCount; ++j) {
				v.push_back(meshVertices[j]);
			}
			sceneManager->addStaticMeshWithFaces(v,triangleIndexFaceBuffer, translationMatrix, prop);

			return 0;
		} catch (...) {
			handle();
		}
	}
	OPAL_API int AddStaticCurvedMeshFromUnity(int meshVertexCount, optix::float3* meshVertices, int meshTriangleCount, int* meshTriangles, int pd1Count, optix::float4* pd1, int pd2Count, optix::float4* pd2,  UnityMatrix4x4 transformationMatrix, UnityMaterialEMProperties emProp, bool makeSingleFace, int faceId) {
		CHM();
		try {
			optix::Matrix4x4 translationMatrix;
			UnityToOptixMatrix4x4(translationMatrix, transformationMatrix);
			//When transforming to MaterialEMProperties conductivity is multiplied by -60*wavelenght
			MaterialEMProperties prop = sceneManager->ITUparametersToMaterial(emProp.a,emProp.b,emProp.c,emProp.d);
			std::vector<optix::float3> vertices(meshVertices, meshVertices+meshVertexCount);
			std::vector<int> triangles(meshTriangles, meshTriangles+meshTriangleCount); 
			std::vector<optix::float4> pd1d(pd1, pd1+pd1Count);
			std::vector<optix::float4> pd2d(pd2, pd2+pd2Count);
			sceneManager->addStaticCurvedMesh(vertices, triangles,pd1d,pd2d, translationMatrix,prop, makeSingleFace, faceId);

			return 0;
		} catch (...) {
			handle();
		}
}
	OPAL_API int AddStaticMeshFromUnity(int meshVertexCount, optix::float3* meshVertices, int meshTriangleCount, int* meshTriangles, UnityMatrix4x4 transformationMatrix, UnityMaterialEMProperties emProp) {
		CHM();
		try {
			optix::Matrix4x4 translationMatrix;
			UnityToOptixMatrix4x4(translationMatrix, transformationMatrix);
			//When transforming to MaterialEMProperties conductivity is multiplied by -60*wavelenght
			MaterialEMProperties prop = sceneManager->ITUparametersToMaterial(emProp.a,emProp.b,emProp.c,emProp.d);
			/*float relativePermitivity;
			  if (emProp.b==0) {
			  relativePermitivity = emProp.a;
			  }
			  else {
			  relativePermitivity=emProp.a*powf((sceneManager->defaultChannel.frequency / 1.0e9f), emProp.b); //Frequency in GHz
			  }
			  float conductivity;
			  if (emProp.d == 0) {
			  conductivity = emProp.c;
			  }
			  else {
			  conductivity = emProp.c*powf((sceneManager->defaultChannel.frequency / 1.0e9f), emProp.d); //Frequency in GHz
			  }
			  prop.dielectricConstant = make_float2(relativePermitivity,-60.0f*sceneManager->defaultChannel.waveLength*conductivity);
			  */
			sceneManager->addStaticMesh(meshVertexCount, meshVertices, meshTriangleCount, meshTriangles, translationMatrix, prop);

			return 0;
		}
		catch (opal::Exception& e) {
#ifdef OPALDEBUG
			std::ofstream myfile;
			myfile.open(logFile, std::ifstream::app);
			myfile << "AddStaticMeshFromUnity: error occurred with message " << e.getErrorString() << std::endl;
			myfile.close();
#endif // OPALDEBUG

			return OPAL_EXCEPTION;
		}
		catch (optix::Exception& e) {
#ifdef OPALDEBUG
			std::ofstream myfile;
			myfile.open(logFile, std::ifstream::app);
			myfile << "AddStaticMeshFromUnity: error occurred with error code "
				<< e.getErrorCode() << " and message "
				<< e.getErrorString() << std::endl;
			myfile.close();
#endif // OPALDEBUG
			return OPTIX_EXCEPTION;
		}
	}

	OPAL_API int AddDynamicMeshGroup(int id)
	{
		CHM();
		try {
			sceneManager->addDynamicMeshGroup(id);

			return 0;
		}
		catch (opal::Exception& e) {
#ifdef OPALDEBUG
			std::ofstream myfile;
			myfile.open(logFile, std::ifstream::app);
			myfile << "AddDynamicMeshGroup: error occurred with message " << e.getErrorString() << std::endl;
			myfile.close();
#endif // OPALDEBUG

			return OPAL_EXCEPTION;
		}
		catch (optix::Exception& e) {
#ifdef OPALDEBUG
			std::ofstream myfile;
			myfile.open(logFile, std::ifstream::app);
			myfile << "AddDynamicMeshGroup: error occurred with error code "
				<< e.getErrorCode() << " and message "
				<< e.getErrorString() << std::endl;
			myfile.close();
#endif // OPALDEBUG
			return OPTIX_EXCEPTION;
		}
	}

	OPAL_API int AddMeshWithFacesToGroupFromUnity(int id, int meshVertexCount, optix::float3* meshVertices, int meshTriangleCount, int* meshTriangles, int faceIdCount, int* faceIds,  UnityMaterialEMProperties emProp) {
		CHM();
		try {
			//When transforming to MaterialEMProperties conductivity is multiplied by -60*wavelenght
			MaterialEMProperties prop = sceneManager->ITUparametersToMaterial(emProp.a,emProp.b,emProp.c,emProp.d);
			std::vector<std::pair<optix::int3, unsigned int>> triangleIndexFaceBuffer;
			//TODO: Lots of copying again, this can be used more during execution, change to more efficient
			for (int i=0; i<meshTriangleCount; i=i+3) {
				optix::int3 tri=make_int3(meshTriangles[i],meshTriangles[i+1],meshTriangles[i+2]);
				triangleIndexFaceBuffer.push_back(std::pair<optix::int3, unsigned int>(tri,static_cast<unsigned int>(faceIds[i])));
				
				
			}
			std::vector<optix::float3> v;
			for (int j=0; j<meshVertexCount; ++j) {
				v.push_back(meshVertices[j]);
			}
			sceneManager->addMeshWithFacesToGroup(id, v,triangleIndexFaceBuffer,  prop);

			return 0;
		} catch (...) {
			handle();
		}
	}
	OPAL_API int AddMeshToGroupFromUnity(int id, int meshVertexCount, optix::float3 * meshVertices, int meshTriangleCount, int * meshTriangles, UnityMaterialEMProperties emProp)
	{
		CHM();
		try {

			//When transforming to MaterialEMProperties conductivity is multiplied by -60*wavelenght
			MaterialEMProperties prop = sceneManager->ITUparametersToMaterial(emProp.a,emProp.b,emProp.c,emProp.d);

			sceneManager->addMeshToGroup(id, meshVertexCount, meshVertices, meshTriangleCount, meshTriangles,  prop);

			return 0;
		}
		catch (opal::Exception& e) {
#ifdef OPALDEBUG
			std::ofstream myfile;
			myfile.open(logFile, std::ifstream::app);
			myfile << "AddMeshToGroupFromUnity: error occurred with message " << e.getErrorString() << std::endl;
			myfile.close();
#endif // OPALDEBUG

			return OPAL_EXCEPTION;
		}
		catch (optix::Exception& e) {
#ifdef OPALDEBUG
			std::ofstream myfile;
			myfile.open(logFile, std::ifstream::app);
			myfile << "AddMeshToGroupFromUnity: error occurred with error code "
				<< e.getErrorCode() << " and message "
				<< e.getErrorString() << std::endl;
			myfile.close();
#endif // OPALDEBUG
			return OPTIX_EXCEPTION;
		}
	}

	OPAL_API int UpdateTransformInGroup(int id, UnityMatrix4x4 transformationMatrix)
	{
		CHM();
		try {
			optix::Matrix4x4 translationMatrix;
			UnityToOptixMatrix4x4(translationMatrix, transformationMatrix);


			sceneManager->updateTransformInGroup(id, translationMatrix);  

			return 0;
		}
		catch (opal::Exception& e) {
#ifdef OPALDEBUG
			std::ofstream myfile;
			myfile.open(logFile, std::ifstream::app);
			myfile << "UpdateTransformInGroup: error occurred with message " << e.getErrorString() << std::endl;
			myfile.close();
#endif // OPALDEBUG

			return OPAL_EXCEPTION;
		}
		catch (optix::Exception& e) {
#ifdef OPALDEBUG
			std::ofstream myfile;
			myfile.open(logFile, std::ifstream::app);
			myfile << "UpdateTransformInGroup: error occurred with error code "
				<< e.getErrorCode() << " and message "
				<< e.getErrorString() << std::endl;
			myfile.close();
#endif // OPALDEBUG
			return OPTIX_EXCEPTION;
		}
	}

	OPAL_API int SetPrintEnabled(int bufferSize)
	{
		CHM();
		try {

			sceneManager->setPrintEnabled(bufferSize);
			return 0;
		}
		catch (opal::Exception& e) {
#ifdef OPALDEBUG
			std::ofstream myfile;
			myfile.open(logFile, std::ifstream::app);
			myfile << "SetPrintEnabled: error occurred with message " << e.getErrorString() << std::endl;
			myfile.close();
#endif // OPALDEBUG

			return OPAL_EXCEPTION;
		}
		catch (optix::Exception& e) {
#ifdef OPALDEBUG
			std::ofstream myfile;
			myfile.open(logFile, std::ifstream::app);
			myfile << "SetPrintEnabled: error occurred with error code "
				<< e.getErrorCode() << " and message "
				<< e.getErrorString() << std::endl;
			myfile.close();
#endif // OPALDEBUG
			return OPTIX_EXCEPTION;
		}
	}
	OPAL_API int SetUsageReport()
	{
		CHM();
		try {
			sceneManager->setUsageReport();
		}
		catch (...) {
			handle();
		}
	}
	OPAL_API int FillRaySphere2D(int elevationSteps, int azimuthSteps, optix::float3* rayDirections)
	{
		CHM();
		try {

			sceneManager->createRaySphere2D(elevationSteps, azimuthSteps, rayDirections);
			return 0;
		}
		catch (opal::Exception& e) {
#ifdef OPALDEBUG
			std::ofstream myfile;
			myfile.open(logFile, std::ifstream::app);
			myfile << "CreateRaySphere2D: error occurred with message " << e.getErrorString() << std::endl;
			myfile.close();
#endif // OPALDEBUG

			return OPAL_EXCEPTION;
		}
		catch (optix::Exception& e) {
#ifdef OPALDEBUG
			std::ofstream myfile;
			myfile.open(logFile, std::ifstream::app);
			myfile << "CreateRaySphere2D: error occurred with error code "
				<< e.getErrorCode() << " and message "
				<< e.getErrorString() << std::endl;
			myfile.close();
#endif // OPALDEBUG
			return OPTIX_EXCEPTION;
		}
	}
	OPAL_API int SetRayRange(float initElevation, float endElevation, float initAzimuth,  float endAzimuth, int rayD) {
		CHM();
		try {
			sceneManager->setRayRange(initElevation,endElevation,initAzimuth,endAzimuth,rayD,rayD);
			return 0;
			
		} catch (...) {
			handle();
		}
	}
	OPAL_API int CreateRaySphereRange(float initElevation, float elevationDelta, float endElevation, float initAzimuth, float azimuthDelta, float endAzimuth) {
		CHM();
		try {
			sceneManager->createRaySphere2D(initElevation, elevationDelta, endElevation, initAzimuth,azimuthDelta,endAzimuth);
			return 0;
			
		} catch (...) {
			handle();
		}
	}
	OPAL_API int CreateRaySphere2D(int elevationDelta, int azimuthDelta)
	{
		CHM();
		try {

			sceneManager->createRaySphere2D(elevationDelta, azimuthDelta);
			return 0;

		}
		catch (opal::Exception& e) {
#ifdef OPALDEBUG
			std::ofstream myfile;
			myfile.open(logFile, std::ifstream::app);
			myfile << "CreateRaySphere2D: error occurred with message " << e.getErrorString() << std::endl;
			myfile.close();
#endif // OPALDEBUG

			return OPAL_EXCEPTION;
		}
		catch (optix::Exception& e) {
#ifdef OPALDEBUG
			std::ofstream myfile;
			myfile.open(logFile, std::ifstream::app);
			myfile << "CreateRaySphere2D: error occurred with error code "
				<< e.getErrorCode() << " and message "
				<< e.getErrorString() << std::endl;
			myfile.close();
#endif // OPALDEBUG
			return OPTIX_EXCEPTION;
		}
	}
	OPAL_API int CreateRaySphere2DSubstep(int elevationDelta, int azimuthDelta)
	{
		CHM();
		try {

			sceneManager->createRaySphere2DSubstep(elevationDelta, azimuthDelta);
			return 0;

		}
		catch (opal::Exception& e) {
#ifdef OPALDEBUG
			std::ofstream myfile;
			myfile.open(logFile, std::ifstream::app);
			myfile << "CreateRaySphere2D: error occurred with message " << e.getErrorString() << std::endl;
			myfile.close();
#endif // OPALDEBUG

			return OPAL_EXCEPTION;
		}
		catch (optix::Exception& e) {
#ifdef OPALDEBUG
			std::ofstream myfile;
			myfile.open(logFile, std::ifstream::app);
			myfile << "createRaySphere2DSubstep: error occurred with error code "
				<< e.getErrorCode() << " and message "
				<< e.getErrorString() << std::endl;
			myfile.close();
#endif // OPALDEBUG
			return OPTIX_EXCEPTION;
		}
	}
	OPAL_API float GetReceivedPower(int rxId, int txId)
	{
		CHM();
		try 
			{
			return sceneManager->getReceivedPower(rxId, txId);
		} catch (...) {
			handle();
		}
	}
	OPAL_API int GetReceivedE(int rxId, int txId, float* Exr, float* Exi, float* Eyr, float* Eyi,float* Ezr, float* Ezi)
	{
		CHM();
		try 
			{
			EComponents e= sceneManager->getReceivedE(rxId, txId);
			(*Exr)=e.Ex.x;
			(*Exi)=e.Ex.y;
			(*Eyr)=e.Ey.x;
			(*Eyi)=e.Ey.y;
			(*Ezr)=e.Ez.x;
			(*Ezi)=e.Ez.y;
			std::cout<<"GetReceivedE() "<<e.Ex<<","<<e.Ey<<","<<e.Ez<<std::endl;
			return 0;

		} catch (...) {
			handle();
		}
	}


	OPAL_API int AddReceiverFromUnity(int id, float3  position, float3 polarization, float radius)
	{
		CHM();
		try {
			std::cout<<"AddReceiverFromUnity "<<id<<": position="<<position<<"polarization="<<polarization<<";radius="<<radius<<std::endl;
			sceneManager->addReceiver(id, position, polarization, radius, nullptr);
			return 0;
		}
		catch (opal::Exception& e) {
#ifdef OPALDEBUG
			std::ofstream myfile;
			myfile.open(logFile, std::ifstream::app);
			myfile << "AddReceiverFromUnity: error occurred with message " << e.getErrorString() << std::endl;
			myfile.close();
#endif // OPALDEBUG

			return OPAL_EXCEPTION;
		}
		catch (optix::Exception& e) {
#ifdef OPALDEBUG
			std::ofstream myfile;
			myfile.open(logFile, std::ifstream::app);
			myfile << "AddReceiverFromUnity: error occurred with error code "
				<< e.getErrorCode() << " and message "
				<< e.getErrorString() << std::endl;
			myfile.close();
#endif // OPALDEBUG
			return OPTIX_EXCEPTION;
		}
	}
	OPAL_API int Transmit(int txId, float txPower, optix::float3 origin, optix::float3 polarization) {
		CHM();
		try {
			sceneManager->transmit(txId, txPower, origin, polarization);
			return 0;
		}
		catch (opal::Exception& e) {
			std::cout << "Transmit: error occurred with message " << e.getErrorString() << std::endl;

			return OPAL_EXCEPTION;
		}
		catch (optix::Exception& e) {
			std::cout << "Transmit: error occurred with error code "
				<< e.getErrorCode() << " and message "
				<< e.getErrorString() << std::endl;
			return OPTIX_EXCEPTION;
		}

	}

	OPAL_API int FinishSceneContext() {
		CHM();
		try {

			sceneManager->finishSceneContext();
			return 0;
		}
		catch (opal::Exception& e) {
#ifdef OPALDEBUG
			std::ofstream myfile;
			myfile.open(logFile, std::ifstream::app);
			myfile << "FinishSceneContext: error occurred with message " << e.getErrorString() << std::endl;
			myfile.close();
#endif // OPALDEBUG

			return OPAL_EXCEPTION;
		}
		catch (optix::Exception& e) {
#ifdef OPALDEBUG
			std::ofstream myfile;
			myfile.open(logFile, std::ifstream::app);
			myfile << "FinishSceneContext: error occurred with error code "
				<< e.getErrorCode() << " and message "
				<< e.getErrorString() << std::endl;
			myfile.close();
#endif // OPALDEBUG
			return OPTIX_EXCEPTION;
		}
	}
	OPAL_API int UpdateReceiver(int id, optix::float3 position) {
		CHM();
		try {

			sceneManager->updateReceiver(id, position);
			return 0;
		}
		catch (opal::Exception& e) {
#ifdef OPALDEBUG
			std::ofstream myfile;
			myfile.open(logFile, std::ifstream::app);
			myfile << "UpdateReceiver: error occurred with message " << e.getErrorString() << std::endl;
			myfile.close();
#endif // OPALDEBUG

			return OPAL_EXCEPTION;
		}
		catch (optix::Exception& e) {
#ifdef OPALDEBUG
			std::ofstream myfile;
			myfile.open(logFile, std::ifstream::app);
			myfile << "UpdateReceiver: error occurred with error code "
				<< e.getErrorCode() << " and message "
				<< e.getErrorString() << std::endl;
			myfile.close();
#endif // OPALDEBUG
			return OPTIX_EXCEPTION;
		}
	}

	OPAL_API int UpdateReceiverWithRadius(int id, optix::float3 position, optix::float3 polarization, float radius) {
		CHM();
		try {

			sceneManager->updateReceiver(id, position,polarization, radius);
			return 0;
		}
		catch (opal::Exception& e) {
#ifdef OPALDEBUG
			std::ofstream myfile;
			myfile.open(logFile, std::ifstream::app);
			myfile << "UpdateReceiver: error occurred with message " << e.getErrorString() << std::endl;
			myfile.close();
#endif // OPALDEBUG

			return OPAL_EXCEPTION;
		}
		catch (optix::Exception& e) {
#ifdef OPALDEBUG
			std::ofstream myfile;
			myfile.open(logFile, std::ifstream::app);
			myfile << "UpdateReceiver: error occurred with error code "
				<< e.getErrorCode() << " and message "
				<< e.getErrorString() << std::endl;
			myfile.close();
#endif // OPALDEBUG
			return OPTIX_EXCEPTION;
		}
	}
	OPAL_API int RemoveReceiverFromUnity(int id) {
		CHM();
		try {

			sceneManager->removeReceiver(id);
			return 0;
		}
		catch (opal::Exception& e) {
#ifdef OPALDEBUG
			std::ofstream myfile;
			myfile.open(logFile, std::ifstream::app);
			myfile << "RemoveReceiverFromUnity:"<<id<<" error occurred with message: " << e.getErrorString() << std::endl;
			myfile.close();
#endif // OPALDEBUG

			return OPAL_EXCEPTION;
		}
		catch (optix::Exception& e) {
#ifdef OPALDEBUG
			std::ofstream myfile;
			myfile.open(logFile, std::ifstream::app);
			myfile << "RemoveReceiverFromUnity:" << id<<" error occurred with error code "
				<< e.getErrorCode() << " and message "
				<< e.getErrorString() << std::endl;
			myfile.close();
#endif // OPALDEBUG
			return OPTIX_EXCEPTION;
		}
	}
	OPAL_API int FinishDynamicMeshGroup(int id) {
		CHM();
		try {

			sceneManager->finishDynamicMeshGroup(id);
			return 0;
		}
		catch (opal::Exception& e) {
#ifdef OPALDEBUG
			std::ofstream myfile;
			myfile.open(logFile, std::ifstream::app);
			myfile << "FinishDynamicMeshGroup:" << id << " error occurred with message: " << e.getErrorString() << std::endl;
			myfile.close();
#endif // OPALDEBUG

			return OPAL_EXCEPTION;
		}
		catch (optix::Exception& e) {
#ifdef OPALDEBUG
			std::ofstream myfile;
			myfile.open(logFile, std::ifstream::app);
			myfile << "FinishDynamicMeshGroup:" << id << " error occurred with error code "
				<< e.getErrorCode() << " and message "
				<< e.getErrorString() << std::endl;
			myfile.close();
#endif // OPALDEBUG
			return OPTIX_EXCEPTION;
		}
	}
	OPAL_API int RemoveDynamicMeshGroup(int id) {
		CHM();
		try {

			sceneManager->removeDynamicMeshGroup(id);
			return 0;
		}
		catch (opal::Exception& e) {
#ifdef OPALDEBUG
			std::ofstream myfile;
			myfile.open(logFile, std::ifstream::app);
			myfile << "RemoveDynamicMeshGroup:" << id << " error occurred with message: " << e.getErrorString() << std::endl;
			myfile.close();
#endif // OPALDEBUG

			return OPAL_EXCEPTION;
		}
		catch (optix::Exception& e) {
#ifdef OPALDEBUG
			std::ofstream myfile;
			myfile.open(logFile, std::ifstream::app);
			myfile << "RemoveDynamicMeshGroup:" << id << " error occurred with error code "
				<< e.getErrorCode() << " and message "
				<< e.getErrorString() << std::endl;
			myfile.close();
#endif // OPALDEBUG
			return OPTIX_EXCEPTION;
		}
	}
	OPAL_API int RegisterTransmitter(int txId, optix::float3 origin, optix::float3 polarization, float transmitPower) 
	{
		CHM();
		try {
			sceneManager->getTransmitterManager()->registerTransmitter(txId, origin, polarization, transmitPower);
			return 0;
		}
		catch (...) {
			handle();
		}
	}
	OPAL_API int RemoveTransmitter(int txId) 
	{
		CHM();
		try {

			sceneManager->getTransmitterManager()->removeTransmitter(txId);
			return 0;
		}
		catch (...) {
			handle();
		}
	}
	OPAL_API int AddTransmitterToGroup(int txId,float transmitPower, optix::float3 origin,optix::float3 polarization)
	{
		CHM();
		try {

			sceneManager->getTransmitterManager()->addTransmitterToGroup(txId,transmitPower,origin,polarization);
			return 0;
		}
		catch (...) {
			handle();
		}
	}

	OPAL_API int ClearGroup()
	{
		CHM();
		try {

			sceneManager->getTransmitterManager()->clearGroup();
			return 0;
		}
		catch (...) {
			handle();
		}
	}
	OPAL_API int GroupTransmit()
	{
		CHM();
		try {

			sceneManager->groupTransmit();
			return 0;
		}
		catch (...) {
			handle();
		}
	}
	OPAL_API int LoadAndRegisterGain(const char* path) 
	{
		CHM();
		try {
				std::cout<<"Loading from Unity gain file "<<path<<std::endl;
				AntennaGain gains=sceneManager->loadGainsFromFileIndBPower(path);
				int id=sceneManager->registerAntennaGain(gains);

				return id;
		}
		catch (...) {
			handle();
		}
	}
	OPAL_API int RegisterReceiverGain(int rxId, int gainId)  
	{
		CHM();
		try {
                        sceneManager->registerReceiverGain(rxId,gainId);
			return 0;
		}
		catch (...) {
			handle();
		}
	}
	OPAL_API int RegisterTransmitterGain(int txId, int gainId)  
	{
		CHM();
		try {
                        sceneManager->registerTransmitterGain(txId,gainId);
			return 0;
		}
		catch (...) {
			handle();
		}
	}
}
