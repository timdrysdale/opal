#include "Unity.h"
#include <iostream>
#include <stdlib.h>

#define CHM() if (sceneManager==nullptr) return OPAL_NO_SCENE_MANAGER; //Check manager
using namespace optix;
namespace opal {
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
	//Note that CUDA programs are compiled in execution time with NVRTC. They are read from the OPTIX_SAMPLES_SDK_DIR location as set here (see below). See also sutil.cpp line 848 and 335
	//In that directory you should put all the .cu and the Common.h files. They can be changed without recompiling all the plugin.
	//Any change made in the .cu files in VS is ignored unless copied to that location.

	//When building an executable with Unity, you have either to use a specific script to create the cudaDir and copy the .cu and .h files to it, or just create and copy manually after the build is done
	//In the end, along with the Unity executable and files you need to have this specific folder and files some way
	OPAL_API int Init(float frequency, bool holdReflections)
	{
		try {
			//First, set the environment to read the CUDA program files from our Plugins directory.

#ifdef _WIN32
			_putenv_s("OPTIX_SAMPLES_SDK_DIR", cudaDir);
			
#else
			setenv("OPTIX_SAMPLES_SDK_DIR", cudaDir);
#endif // _WIN32


#ifdef OPALDEBUG
			sceneManager = new OpalSceneManager(frequency, holdReflections);
#else
			sceneManager = new OpalSceneManager(frequency);
#endif


			
			
			
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

		if (sceneManager != nullptr) {
			
			delete sceneManager;
		}
		return 0;
	}
	OPAL_API int SetMaxReflections(unsigned int m)
	{
		CHM();
		sceneManager->setMaxReflections(m);

		
		return 0;
	}
	OPAL_API int AddStaticMeshFromUnity(int meshVertexCount, optix::float3* meshVertices, int meshTriangleCount, int* meshTriangles, UnityMatrix4x4 transformationMatrix, UnityMaterialEMProperties emProp) {
		CHM();
		try {
			optix::Matrix4x4 translationMatrix;
			UnityToOptixMatrix4x4(translationMatrix, transformationMatrix);
			//When transforming to MaterialEMProperties conductivity is multiplied by -60*wavelenght
			MaterialEMProperties prop;
			prop.dielectricConstant = make_float2(emProp.relativePermitivity,-60.0f*sceneManager->defaultChannel.waveLength*emProp.conductivity);
		
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

	OPAL_API int AddMeshToGroupFromUnity(int id, int meshVertexCount, optix::float3 * meshVertices, int meshTriangleCount, int * meshTriangles, UnityMaterialEMProperties emProp)
	{
		CHM();
		try {
			
			//When transforming to MaterialEMProperties conductivity is multiplied by -60*wavelenght
			MaterialEMProperties prop;
			prop.dielectricConstant = make_float2(emProp.relativePermitivity, -60.0f*sceneManager->defaultChannel.waveLength*emProp.conductivity);

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

	/*OPAL_API int SetDuplicateBlockSize(optix::uint elevationBlockSize, optix::uint azimuthBlockSize)
	{
		CHM();
		try {

			sceneManager->setDuplicateBlockSize(elevationBlockSize, azimuthBlockSize);
			return 0;
		}
		catch (opal::Exception& e) {
#ifdef OPALDEBUG
			std::ofstream myfile;
			myfile.open(logFile, std::ifstream::app);
			myfile << "SetDuplicateBlockSize: error occurred with message " << e.getErrorString() << std::endl;
			myfile.close();
#endif // OPALDEBUG

			return OPAL_EXCEPTION;
		}
		catch (optix::Exception& e) {
#ifdef OPALDEBUG
			std::ofstream myfile;
			myfile.open(logFile, std::ifstream::app);
			myfile << "SetDuplicateBlockSize: error occurred with error code "
				<< e.getErrorCode() << " and message "
				<< e.getErrorString() << std::endl;
			myfile.close();
#endif // OPALDEBUG
			return OPTIX_EXCEPTION;
		}
	}

	*/
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

	OPAL_API int AddReceiverFromUnity(int id, float3  position, float radius, receiverCallback callback)
	{
		CHM();
		try {
			sceneManager->addReceiver(id, position, radius, callback);
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
#ifdef OPALDEBUG
			std::ofstream myfile;
			myfile.open(logFile, std::ifstream::app);
			myfile << "Transmit: error occurred with message " << e.getErrorString() << std::endl;
			myfile.close();
#endif // OPALDEBUG

			return OPAL_EXCEPTION;
		}
		catch (optix::Exception& e) {
#ifdef OPALDEBUG
			std::ofstream myfile;
			myfile.open(logFile, std::ifstream::app);
			myfile << "Transmit: error occurred with error code "
				<< e.getErrorCode() << " and message "
				<< e.getErrorString() << std::endl;
			myfile.close();
#endif // OPALDEBUG
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

	OPAL_API int UpdateReceiverWithRadius(int id, optix::float3 position, float radius) {
		CHM();
		try {

			sceneManager->updateReceiver(id, position,radius);
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
}
