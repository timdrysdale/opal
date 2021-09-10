/***************************************************************/
//
//Copyright (c) 2019 Esteban Egea-Lopez http://girtel.upct.es/~eegea
//
/**************************************************************/


#ifndef UNITY_H
#define UNITY_H

#ifdef _WIN32
#if defined(OPAL_EXPORTS) || defined(opal_s_EXPORTS)
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
	static const int OPAL_NO_MULTI_TRANSMITTER_MANAGER = 4;
	static const char* logFile = "unity_opal.txt";

	//Location of Optix Programs in the Unity plugins folder
	static const char* pluginDir = "./Assets/Plugins/Opal";


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
	static bool MultiTransmitter = false;
	void printMatrix(UnityMatrix4x4& u, std::ofstream& s);
	void UnityToOptixMatrix4x4(optix::Matrix4x4& m, UnityMatrix4x4& u);
#ifdef _WIN32

	typedef void(__stdcall *receiverCallback)(float, int);
#else
	typedef  std::function<void(float,int)> receiverCallback;
#endif //_WIN32


	extern "C" OPAL_API int Transmit(int txId, float txPower, optix::float3 origin, optix::float3 polarization);

	extern "C" OPAL_API int FinishSceneContext();

	extern "C" OPAL_API int Init(float frequency,int simType, int computeMode, bool useExactSpeedOfLight, bool useDiffraction, bool enableFastMath, bool generateRaysOnLaunch, bool enableMultiGPU, bool logTrace, bool enableMultitransmitter, bool useAntennaGain, unsigned int maxReflections, float minEpsilon );

	extern "C" OPAL_API int SetMaxReflections(unsigned int  m);
	
	extern "C" OPAL_API int EnablePenetration();
	
	extern "C" OPAL_API int DisablePenetration();
	
	extern "C" OPAL_API int SetAttenuationLimit(float l);


	extern "C" OPAL_API int Exit();

	extern "C" OPAL_API int SetPrintEnabled(int bufferSize);

	extern "C" OPAL_API int SetUsageReport();

	extern "C" OPAL_API int FillRaySphere2D(int elevationSteps, int azimuthSteps, optix::float3 * rayDirections);

	extern "C" OPAL_API int SetRayRange(float initElevation,  float endElevation, float initAzimuth,  float endAzimuth, int rayD);
	
	extern "C" OPAL_API int CreateRaySphereRange(float initElevation, float elevationDelta, float endElevation, float initAzimuth, float azimuthDelta, float endAzimuth);

	extern "C" OPAL_API int CreateRaySphere2D(int elevationDelta, int azimuthDelta);

	extern "C" OPAL_API int CreateRaySphere2DSubstep(int elevationDelta, int azimuthDelta);

	extern "C" OPAL_API int UpdateReceiver(int id, optix::float3 position);

	extern "C" OPAL_API int UpdateReceiverWithRadius(int id, optix::float3 position, optix::float3 polarization, float radius);

	//extern "C" OPAL_API int AddReceiverFromUnity(int id, optix::float3 position, optix::float3 polarization,  float radius, receiverCallback callback);
	
	extern "C" OPAL_API int AddReceiverFromUnity(int id, optix::float3 position, optix::float3 polarization, float radius);

	extern "C" OPAL_API int RemoveReceiverFromUnity(int id);
	
	extern "C" OPAL_API int AddEdge(optix::float3 p, optix::float3 v, unsigned int faid, unsigned int fbid, optix::float3 face_a, optix::float3 face_b, optix::float3 normal_a, optix::float3 normal_b, UnityMaterialEMProperties emProp, int id);

	extern "C" OPAL_API int AddEdgeToGroup(optix::float3 p, optix::float3 v, unsigned int faid, unsigned int fbid, optix::float3 face_a, optix::float3 face_b, optix::float3 normal_a, optix::float3 normal_b, UnityMaterialEMProperties emProp, int id, int groupId);
	extern "C" OPAL_API int AddStaticMeshWithFacesFromUnity(int meshVertexCount, optix::float3* meshVertices, int meshTriangleCount, int* meshTriangles, int faceIdCount, int* faceIds,  UnityMatrix4x4 transformationMatrix, UnityMaterialEMProperties emProp);

	extern "C" OPAL_API int AddStaticMeshFromUnity(int meshVertexCount, optix::float3* meshVertices, int meshTriangleCount, int* meshTriangles, UnityMatrix4x4 transformationMatrix, UnityMaterialEMProperties emProp);
	
	extern "C" OPAL_API int AddStaticCurvedMeshFromUnity(int meshVertexCount, optix::float3* meshVertices, int meshTriangleCount, int* meshTriangles, int pd1Count, optix::float4* pd1, int pd2Count, optix::float4* pd2,  UnityMatrix4x4 transformationMatrix, UnityMaterialEMProperties emProp, bool makeSingleFace, int faceId);

	extern "C" OPAL_API float GetReceivedPower(int rxId, int txId);
	
	extern "C" OPAL_API int GetReceivedE(int rxId, int txId, float* Exr, float* Exi, float* Eyr, float* Eyi,float* Ezr, float* Ezi);

	extern "C" OPAL_API int  AddDynamicMeshGroup(int id);

	extern "C" OPAL_API int  AddMeshToGroupFromUnity(int id, int meshVertexCount, optix::float3* meshVertices, int meshTriangleCount, int* meshTriangles, UnityMaterialEMProperties emProp);
	
	extern "C" OPAL_API int  AddMeshWithFacesToGroupFromUnity(int id, int meshVertexCount, optix::float3* meshVertices, int meshTriangleCount, int* meshTriangles, int faceIdCount, int* faceIds, UnityMaterialEMProperties emProp);

	extern "C" OPAL_API int  UpdateTransformInGroup(int id, UnityMatrix4x4 transformationMatrix);

	extern "C" OPAL_API int  FinishDynamicMeshGroup(int id);

	extern "C" OPAL_API int  RemoveDynamicMeshGroup(int id);
	
        extern "C" OPAL_API int  LoadAndRegisterGain(const char* path);
        
        extern "C" OPAL_API int  RegisterReceiverGain(int rxId ,int gainId);
        
 	extern "C" OPAL_API int  RegisterTransmitterGain(int txId ,int gainId);

	//Multi-transmitter functions

	//Register transmitter in Opal. Add to transmitters map
	extern "C" OPAL_API int	 RegisterTransmitter(int txId, optix::float3 origin, optix::float3 polarization, float transmitPower) ;
	//Find transmitter and remove from Opal. Cannot transmit anymore 
	extern "C" OPAL_API int	 RemoveTransmitter(int txId) ;
	//Add transmitter to next parallel transmission
	extern "C" OPAL_API int AddTransmitterToGroup(int txId,float transmitPower, optix::float3 origin,optix::float3 polarization);
	//Clear current transmit group
	extern "C" OPAL_API int	 ClearGroup();
	//Transmit simultaneously all transmitters in group
	extern "C" OPAL_API int	 GroupTransmit() ;



}
#endif

