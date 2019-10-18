/***************************************************************/
//
//Copyright (c) 2019 Esteban Egea-Lopez http://ait.upct.es/eegea
//
/**************************************************************/

#ifndef OPAL_H
#define OPAL_H



#include "tutils.h"
#include <optix_world.h>
#include <map>
#include <fstream>
#include <sstream>
#include <functional>
#include <vector>
#include <memory>
#include "Common.h"

namespace opal {



	class Exception : public std::exception {
		public:
			/// Create exception
			Exception(const std::string& message)
				: m_message(message) {}

			/// Virtual destructor (needed for virtual function calls inherited from
			/// std::exception).
			virtual ~Exception() throw() {}

			/// Retrieve the error message
			const std::string& getErrorString() const { return m_message; }

			/// From std::exception
			virtual const char* what() const throw() { return getErrorString().c_str(); }
		private:
			std::string m_message;

	};


	struct OpalMesh
	{
		optix::GeometryInstance      geom_instance;
		optix::float3                bbox_min;
		optix::float3                bbox_max;
		int                          num_triangles;
	};

	class OpalDynamicMeshGroup
	{
		public:
			optix::GeometryGroup		geom_group;
			optix::Transform	transform;
			unsigned int childIndex;
	};


	struct ChannelParameters {
		float frequency;
		float waveLength;
		float k;
		float eA;
	};

	struct RaySphere {
		optix::Buffer raySphereBuffer;
		optix::uint elevationSteps;
		optix::uint azimuthSteps;
		optix::uint  rayCount;

	};


	class SphereReceiver {
		public:
			optix::float3 position;
			optix::float3 polarization;
			float radius;
			optix::GeometryInstance geomInstance;
			optix::Program closestHitProgram;
			std::function<void(float, int)> callback;
			int externalId;
			bool dirty;
	};

	class BaseTransmitter {
		public:
			optix::float3 origin;
			optix::float3 polarization;
			int externalId;
			float transmitPower;
	};

	//Compare mesh faces. Differences below epsilon are consider equal to avoid precision problems
	struct face_compare
	{
		const float epsilon = 1e-6f;
		bool operator() (const optix::float3 first, const optix::float3 second) {
			float xa = fabs(first.x - second.x);
			float ya = fabs(first.y - second.y);
			float za = fabs(first.z - second.z);
			if (xa <= epsilon && ya <= epsilon && za <= epsilon) {
				return false;
			}
			else {
				return (std::tie(first.x, first.y, first.z) < std::tie(second.x, second.y, second.z));
			}

		}
	};


	typedef std::map<optix::float3, unsigned int, face_compare> FaceMap; //Used to extract the faces of the meshes


	//System information
	
	struct DeviceInformation {
		char name[256];
		int computeCapability[2] = {0, 0};
		RTsize totalMemory = 0;
		int clockRate = 0;
		int maxThreadsPerBlock = 0;
		int smCount = 0;
		int executionTimeoutEnabled = 0;
		int maxHardwareTextureCount = 0 ;
		int tccDriver = 0;
		int cudaDeviceOrdinal = 0;
	};
	struct SystemInformation {
		unsigned int optixVersion;
		unsigned int major = optixVersion / 1000; // Check major with old formula.
		unsigned int minor;
		unsigned int micro;
		unsigned int numberOfDevices = 0;
		std::vector<DeviceInformation> devices;		
		void getSystemInformation();
		std::string printSystemInformation();
	};

	class OpalSceneManager {
		protected:
			//Source directory in SDK, used by sutil to find .cu programs
			std::string cudaProgramsDir;
			
			optix::Context context;


			//Internal buffers
			typedef struct {
				optix::uint elevation;
				optix::uint azimuth;
				optix::uint tx;
				optix::uint rx;
				optix::uint reflections; 
			} InternalBuffersParameters;	 //Internal buffers depend on these parameters, used to keep track of the changes

			InternalBuffersParameters currentInternalBuffersState;

			optix::Buffer globalHitInfoBuffer;
			optix::Buffer atomicIndexBuffer;
			optix::Buffer txOriginBuffer;

			float fractionMemGlobalBufferSize;
			uint maxGlobalBufferSize;
			//Scene graph variables
			optix::GeometryGroup receiversGroup;
			optix::Group rootGroup;
			optix::GeometryGroup staticMeshesGroup;
			std::vector<OpalMesh> staticMeshes;

			//External to internal info
			std::map<int, unsigned int> receiverExtToBufferId; //External id to receptionInfo buffer Id
			std::vector<SphereReceiver*> receivers; //receivers info (mapped to receptionInfo buffer)
			std::map<int, BaseTransmitter*> transmitterExtToBase; //Map from externalId to transmitterBase
		
			bool sceneGraphCreated;
			bool sceneFinished;
			int transmissionLaunches;
			RaySphere raySphere;
			ChannelParameters defaultChannel;
			std::map<int, OpalDynamicMeshGroup*> dynamicMeshes; //externalId to dynamic mesh

			//Control parameters
			unsigned int numberOfFaces;
			bool usePenetration;
			bool useDepolarization;
			bool useMultiGPU;
			float attenuationLimit;	

			unsigned int maxReflections; //Default 10
			float minEpsilon; //Default 1.e-3f
			bool useExactSpeedOfLight; //speed of light (can be exact or approximated)
			std::ostringstream configInfo;
#ifdef OPALDEBUG
			std::ofstream outputFile;
#endif // OPALDEBUG


			//Launch variables
			Transmitter currentTransmitter;
			opalthrustutils::PartialLaunchState* partialLaunchState;
			float radioReductionFraction;
			
			
			//Default programs

			std::map<std::string,optix::Program> defaultPrograms;
			optix::Material defaultMeshMaterial;
			float deg2rad;   //Degrees to radians
			
			//System information
			SystemInformation sysInfo;
			std::vector<int> enabledDevices;
				


		public:
		//Init
			OpalSceneManager();
			OpalSceneManager(float f,  bool useExactSpeedOfLight=true);
			virtual ~OpalSceneManager();
			void initContext(float f, bool useExactSpeedOfLight=true);
			virtual void initMembers();
			ChannelParameters getChannelParameters() const;
			void setEnabledDevices();	
		//Static meshes	
			void setMeshEMProperties(optix::GeometryInstance geom_instance, MaterialEMProperties emProp);
			OpalMesh createMesh(int meshVertexCount, optix::float3* meshVertices, int meshTriangleCount, int* meshTriangles, optix::Program intersectionProgram, optix::Program boundingBoxProgram, optix::Material material);
			OpalMesh addStaticMesh(int meshVertexCount, optix::float3* meshVertices, int meshTriangleCount, int* meshTriangles, optix::Matrix4x4 transformationMatrix, MaterialEMProperties emProp);
			void addStaticMesh(OpalMesh mesh);
			MaterialEMProperties ITUparametersToMaterial(float a, float b, float c, float d);
		//Moving meshes functions
			OpalDynamicMeshGroup* addDynamicMeshGroup(int id);
			void removeDynamicMeshGroup(int id);
			void  addMeshToGroup(int id, int meshVertexCount, optix::float3* meshVertices, int meshTriangleCount, int* meshTriangles,  MaterialEMProperties emProp);
			void updateTransformInGroup(int id, optix::Matrix4x4 transformationMatrix);
			void finishDynamicMeshGroup(int id);
			const std::map<int, OpalDynamicMeshGroup*> &  getDynamicMeshes() const;
			//Create and arbitrary 2D ray sphere, with ray directions provided by the user
			void createRaySphere2D(int elevationSteps, int azimutSteps, optix::float3 * rayDirections);
			//Create a 2D ray sphere in discrete steps of elevation and azimuth
			void createRaySphere2D(int elevationDelta, int azimuthDelta);
			//Create a ray sphere with fractions of degree
			//Now elevation delta and azimuthDelta are a decimal fraction of degree, that is, every unit is 0.1 degree, so elevationDelta=1 means 0.1 degree, elevationDelta=2 means 0.2 degree
			void createRaySphere2DSubstep(int elevationDelta, int azimuthDelta);
			//Create a 2D ray sphere with arbitray angular separation and solid angle coverage
			void createRaySphere2D(float initElevation, float elevationDelta, float endElevation, float initAzimuth, float azimuthDelta, float endAzimuth);
		//Receivers
			void addReceiver(int id, optix::float3  position, optix::float3 polarization, float radius, std::function<void(float, int)>  callback);
			void removeReceiver(int id);
			void updateReceiver(int id, optix::float3 position);
			void updateReceiver(int id, optix::float3 position, float radius);
		//Transmit
			void transmit(int txId, float txPower, optix::float3 origin, optix::float3 polarization, bool partial=false);

	
		//Finish building scene
			void finishSceneContext();

		//Configure launch
			void enableMultiGPU();
			void disableMultiGPU();
			void enablePenetration();
			void disablePenetration();
			void enableDepolarization(); 
			void disableDepolarization();
			void setFrequency(float f);
			//Set max attenuation for penetration in dB. Rays with a higher attenuation are not traced for penetration 
			void setAttenuationLimit(float f);
			void setMaxReflections(unsigned int m);
			void setMinEpsilon(float f);
			
			void endPartialLaunch();
			
			void enableExceptions();
		//Log
			void setPrintEnabled(int bufferSize);
			void setUsageReport();
			virtual std::string printInternalBuffersState();
			virtual std::string printSceneReport();
			virtual std::string printContextInformation();
		protected:
		//Scene
			void extractFaces(optix::float3* meshVertices, std::vector<std::pair<optix::int3, unsigned int>> &triangleIndexBuffer);


			void createSceneContext();
			void buildSceneGraph();

		

		//Default programs
			void setDefaultPrograms();
			optix::Material createMeshMaterial(unsigned int ray_type_index, optix::Program closestHitProgram);

			optix::Program createClosestHitMesh();

			virtual optix::Program createClosestHitReceiver();


			optix::Program createBoundingBoxTriangle();
			#ifdef OPAL_USE_TRI
			optix::Program createTriangleAttributesProgram();
			#endif

			optix::Program createIntersectionTriangle();

			optix::Program createBoundingBoxSphere();

			optix::Program createIntersectionSphere();

			virtual optix::Program createMissProgram();

			virtual optix::Program createRayGenerationProgram();


		//Launches 
			void executeTransmitLaunch(uint numTransmitters, bool partial);
		//Hits processing
			virtual void  processHits(HitInfo* host_hits, uint hits);

		//Internal buffers
			virtual void setInternalBuffers();
			virtual void checkInternalBuffers();
			virtual void clearInternalBuffers();

			virtual optix::Buffer setGlobalHitInfoBuffer(optix::uint ele, optix::uint azi, optix::uint rx, optix::uint reflections);
			void resizeGlobalHitInfoBuffer(optix::uint ele, optix::uint azi, optix::uint rx, optix::uint reflections);
			optix::Buffer setAtomicIndexBuffer();
			optix::Buffer setTransmitterBuffer(optix::uint tx);

		//Utils
			static void callbackUsageReport(int level, const char* tag, const char* msg, void* cbdata);
			void computeReceivedPower(optix::float2 E, unsigned int index, int txId, float txPower, optix::float3 origin);
	};



} //namespace opal
#endif

