#pragma once
#ifdef _WIN32
#ifdef OPAL_EXPORTS
#define OPAL_API  __declspec(dllexport)   
#else  
#define OPAL_API  __declspec(dllimport)   
#endif  
#else
#define OPAL_API 
#endif

#include "Opal.h"
//#include <optixpp_namespace.h>
#include <fstream>

namespace opal {





	static const int OPAL_NO_SCENE_MANAGER = 1;
	static const int OPAL_EXCEPTION= 2;
	static const int OPTIX_EXCEPTION = 3;
	static const char* logFile = "unity_opal.txt";

	//Location of Optix Programs
	static const char* cudaDir = "./Assets/Plugins/Opal";


	// -------------------------------------------------------------------------------------
	// Matrix Data Variables to be marshalled from Unity
	// -------------------------------------------------------------------------------------
	struct UnityMatrix4x4 {
	public:
		float m00;

		float m01;

		float m02;

		float m03;

		float m10;

		float m11;

		float m12;

		float m13;

		float m20;

		float m21;

		float m22;

		float m23;

		float m30;

		float m31;

		float m32;

		float m33;



	};

	struct UnityMaterialEMProperties {
		//ITU parameters: depend on frequency
		//RelativePermitivity
		 float a;
		 float b;
		//Conductivity
		 float c;
		 float d;
		
	};

	static  OpalSceneManager* sceneManager = nullptr;

	void printMatrix(UnityMatrix4x4& u, std::ofstream& s);
	void UnityToOptixMatrix4x4(optix::Matrix4x4& m, UnityMatrix4x4& u);

	typedef void(__stdcall *receiverCallback)(float, int);


	
	extern "C" OPAL_API int Transmit(int txId, float txPower, optix::float3 origin, optix::float3 polarization);

	extern "C" OPAL_API int FinishSceneContext();

	extern "C" OPAL_API int Init(float frequency, bool holdReflections);

	extern "C" OPAL_API int SetMaxReflections(unsigned int  m);

	extern "C" OPAL_API int Exit();

	extern "C" OPAL_API int SetPrintEnabled(int bufferSize);

	extern "C" OPAL_API int SetUsageReport();

	extern "C" OPAL_API int FillRaySphere2D(int elevationSteps, int azimuthSteps, optix::float3 * rayDirections);

	extern "C" OPAL_API int CreateRaySphere2D(int elevationDelta, int azimuthDelta);

	extern "C" OPAL_API int CreateRaySphere2DSubstep(int elevationDelta, int azimuthDelta);

	extern "C" OPAL_API int UpdateReceiver(int id, optix::float3 position);

	extern "C" OPAL_API int UpdateReceiverWithRadius(int id, optix::float3 position, float radius);

	extern "C" OPAL_API int AddReceiverFromUnity(int id, optix::float3 position, float radius, receiverCallback callback);

	extern "C" OPAL_API int RemoveReceiverFromUnity(int id);

	extern "C" OPAL_API int AddStaticMeshFromUnity(int meshVertexCount, optix::float3* meshVertices, int meshTriangleCount, int* meshTriangles, UnityMatrix4x4 transformationMatrix, UnityMaterialEMProperties emProp);
	

	extern "C" OPAL_API int  AddDynamicMeshGroup(int id);

	extern "C" OPAL_API int  AddMeshToGroupFromUnity(int id, int meshVertexCount, optix::float3* meshVertices, int meshTriangleCount, int* meshTriangles, UnityMaterialEMProperties emProp);

	extern "C" OPAL_API int  UpdateTransformInGroup(int id, UnityMatrix4x4 transformationMatrix);

	extern "C" OPAL_API int  FinishDynamicMeshGroup(int id);

	extern "C" OPAL_API int  RemoveDynamicMeshGroup(int id);
	




}

