/***************************************************************/
//
//Copyright (c) 2019 Esteban Egea-Lopez http://girtel.upct.es/~eegea
//
/**************************************************************/

#ifndef OPAL_H
#define OPAL_H



//Do not change the order of includes, unless you want to fix many dependencies
#include "tutils.h"
#include "Common.h"
#include <map>
#include <fstream>
#include <sstream>
#include <functional>
#include <vector>
#include <memory>
#include <tuple>
#include <set>
#include "opalSimulation.h" 
#include "transmitterManager.h"
#include "raySphere.h"
#include <optix_world.h>

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
			std::vector<std::pair<Edge,Edge*>> edges;	
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

		struct EComponents {
			optix::float2 E;
			optix::float2 Ex;
			optix::float2 Ey;
			optix::float2 Ez;
			unsigned int index;
			unsigned int hits;
		};

	class SphereReceiver {
		public:
			optix::float3 position;
			optix::float3 polarization;
			float radius;
			optix::GeometryInstance geomInstance;
			//optix::Program closestHitProgram;
			std::function<void(float, int)> callback;
			int externalId;
			bool dirty;
			std::vector<std::tuple<int,float3>> lastReceivedPower;
			std::vector<std::tuple<int,EComponents>> lastReceivedE;
			EComponents getLastReceivedE(int txId);  
			float getLastReceivedPower(int txId);  
			void setLastReceivedResult(int txId, float3 r);
			void setLastReceivedE(int txId, EComponents e);
			void clearLastResult();
			int antennaGainId;	
			
	};


	//Compare mesh faces. Note that for us a face is not just a mesh triangle. Each face is used to define an equal interaction element used for ray filtering.
	//For example a wall (made of many triangles). All rays hitting that same wall (equal face id) will be considered the same EM wave for filtering purposes
	//Differences below epsilon are consider equal to avoid precision problems
	struct face_compare
	{
		const float epsilon = 1e-6f;
                bool esentiallyEqual(float a,  float b) {
			return fabs(a - b) <= ( (fabs(a) > fabs(b) ? fabs(b) : fabs(a)) * epsilon);
		}
		bool operator() (const optix::float3 first, const optix::float3 second) {
			//const float xa = fabs(first.x - second.x);
			//const float ya = fabs(first.y - second.y);
			//const float za = fabs(first.z - second.z);
			if (esentiallyEqual(first.x, second.x)  && esentiallyEqual(first.y, second.y) && esentiallyEqual(first.z, second.z)  ) {
			//if (xa <= epsilon && ya <= epsilon && za <= epsilon) {
				return false;
			}
			else {
				return (std::tie(first.x, first.y, first.z) < std::tie(second.x, second.y, second.z));
			}

		}
	};


	typedef std::map<optix::float3, unsigned int, face_compare> FaceMap; //Used to extract the faces of the meshes


	typedef std::vector<std::vector<float> > AntennaGain; //Azimuth/Elevation gain in linear units


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
	
	struct ConfigurationOptions {
		bool useExactSpeedOfLight; //speed of light (can be exact or approximated)
		bool usePenetration;
		bool useDepolarization;
		bool useMultiGPU;
		bool useFastMath;
		bool useMultiChannel;
		bool useAntennaGain;
		bool executeCallbacks;
	
		//Improves performance if ray sphere changes from launch to launch. Otherwise create once a ray sphere
		bool generateRaysOnLaunch;
		std::tuple<bool,int,bool,optix::uint3> printEnabled;
	};
		
	class FieldInfo {
		protected:
		std::map<int, std::map<int,EComponents>*>  Emap;
		//std::map<int, std::map<int,EComponents>> Ext;   // Complex
		public:
		void updateTransmitters(std::vector<Transmitter*>& activeTransmitters);
		void reset();
		void updateField(optix::float2 Ee, int rxId, int txId, unsigned int index, unsigned int raysHit);
		void updateField(optix::float2 Ex, optix::float2 Ey, optix::float2 Ez,  int rxId, int txId, unsigned int index, unsigned int raysHit);
		std::map<int, std::map<int,EComponents>*> getE();	
	};	

	//Forward declarations
	class TransmitterManager;
	class OpalSimulation;
	class OpalRaySphereGenerator;

	class OpalSceneManager {
		protected:
			//Source directory in SDK, used by sutil to find .cu programs
			std::string baseDir;
			std::string optixProgramsDir;
			std::string cudaProgramsDir;
			std::string currentDir;
			
			optix::Context context;


			optix::Buffer txOriginBuffer;
			optix::Buffer rayRangeBuffer;
			optix::Buffer raySphereParametersBuffer;
			
			//Gains
			std::map<int, optix::Buffer > antennaGainBuffers;
			//Scene graph variables
			optix::GeometryGroup receiversGroup;
			optix::Group rootGroup;
			optix::GeometryGroup staticMeshesGroup;
			std::vector<OpalMesh> staticMeshes;

			//Diffraction support
			std::vector<Edge*> edges;	


			//External to internal info
			std::map<int, unsigned int> receiverExtToBufferId; //External id to receptionInfo buffer Id
			std::vector<SphereReceiver*> receivers; //receivers info (mapped to receptionInfo buffer)
			
			//Transmitters
			TransmitterManager* transmitterManager;
		
			bool sceneGraphCreated;
			bool sceneFinished;
			bool contextInitialized;
		
			bool usingCurvedMeshes;	
			RaySphere raySphere;
			ChannelParameters defaultChannel;
			std::map<int, OpalDynamicMeshGroup*> dynamicMeshes; //externalId to dynamic mesh

			OpalRaySphereGenerator* rayGenerator;
			
			
			//Configuration parameters
			ConfigurationOptions options;
			
			unsigned int numberOfFaces;
			std::set<unsigned int> globalFaces;
			float attenuationLimit;	
			//PtxUtil*  ptxHandler;

			unsigned int maxReflections; //Default 10
			float minEpsilon; //Default 1.e-3f
			std::ostringstream configInfo;

#ifdef OPALDEBUG
			std::ofstream outputFile;
#endif // OPALDEBUG


			//Launch variables
			Transmitter currentTransmitter;
			
			
			//Default programs

			std::map<std::string,optix::Program> defaultPrograms;
			optix::Material defaultMeshMaterial;
			optix::Material defaultReceiverMaterial;
			optix::Material defaultCurvedMeshMaterial;
			//Default programs can be overriden in simulation classes
			void createDefaultPrograms();	
			std::vector<OpalSimulation*> simulations;
			float deg2rad;   //Degrees to radians
			PtxUtil*  ptxHandler;
			
			#ifdef OPAL_USE_TRI
			virtual optix::Program createTriangleAttributesProgram();
			virtual optix::Program createCurvedTriangleAttributesProgram();
			#endif

			virtual optix::Program createIntersectionTriangle();
			virtual optix::Program createBoundingBoxSphere();
			virtual optix::Program createIntersectionSphere();
			virtual optix::Program createBoundingBoxTriangle();
			virtual optix::Program createExceptionReflectionProgram();
					

	
			//System information
			SystemInformation sysInfo;
			std::vector<int> enabledDevices;
			
			//Accumulate results	
			FieldInfo* info;


		public:
		//Init
			OpalSceneManager();
			OpalSceneManager(float f,  bool useExactSpeedOfLight=true);
			virtual ~OpalSceneManager();
			void initContext(float f);
			virtual void initMembers();
		//State
			ChannelParameters getChannelParameters() const;
			std::string getBaseDirectory() const;
			RaySphere getRaySphere() const; 
			optix::Context getContext() const;
			std::vector<int> getEnabledDevices() const;
			SystemInformation getSystemInformation() const;
			void setEnabledDevices();
			
			std::vector<Transmitter*> getActiveTransmitters();
			unsigned int getNumberOfActiveTransmitters() const;
			std::vector<SphereReceiver*> getReceivers();
			SphereReceiver* getReceiver(int externalId);
			Transmitter* getTransmitter(int externalId);
			unsigned int getTransmitterIndex(int externalId);
			unsigned int getReceiverIndex(int externalId);
			unsigned int getNumberOfReceivers() const;
			OpalSimulation* getSimulation() const;
			OpalSimulation* getSimulation(uint index) const;
			optix::Program getDefaultProgram(std::string p);	
			unsigned int getNumberOfEdges () const;
			void removeEdge(Edge* e);
			std::vector<Edge*> getEdges() ; 	
			Edge* addEdge(optix::float3 p, optix::float3 v, optix::uint2 faces, optix::float3 face_a, optix::float3 face_b, optix::float3 normal_a, optix::float3 normal_b, MaterialEMProperties emProp, int id=-1);
		//Static meshes	
			void setMeshEMProperties(optix::GeometryInstance geom_instance, MaterialEMProperties emProp);
			OpalMesh createMesh(int meshVertexCount, optix::float3* meshVertices, int meshTriangleCount, int* meshTriangles, optix::Program intersectionProgram, optix::Program boundingBoxProgram,  bool makeSingleFace = false);
			
			//Static mesh with default closest hit programs
			OpalMesh addStaticMesh(std::vector<optix::float3>& meshVertices, std::vector<int>& meshTriangles, optix::Matrix4x4 transformationMatrix, MaterialEMProperties emProp, bool makeSingleFace=false); 
			OpalMesh addStaticMesh(int meshVertexCount, optix::float3* meshVertices, int meshTriangleCount, int* meshTriangles, optix::Matrix4x4 transformationMatrix, MaterialEMProperties emProp, bool makeSingleFace = false);
			OpalMesh addStaticMeshWithFaces(std::vector<optix::float3> &meshVertices,std::vector<std::pair<optix::int3, unsigned int>> &triangleIndexFaceBuffer, optix::Matrix4x4 transformationMatrix, MaterialEMProperties emProp); 
			OpalMesh addStaticCurvedMesh(std::vector<optix::float3>& meshVertices, std::vector<int>& meshTriangles, std::vector<optix::float4>& pd1, std::vector<optix::float4>& pd2, optix::Matrix4x4 transformationMatrix, MaterialEMProperties emProp, bool makeSingleFace, int faceId = -1);
			void addStaticMesh(OpalMesh mesh);
			void setMeshFaceId(OpalMesh mesh, uint id);
			OpalMesh setMeshOnDevice(int meshVertexCount, optix::float3* meshVertices,std::vector<std::pair<optix::int3, unsigned int>> &triangleIndexBuffer, optix::Program intersectionProgram, optix::Program boundingBoxProgram); 
			MaterialEMProperties ITUparametersToMaterial(float a, float b, float c, float d);

			bool checkFaceIds(std::vector<std::pair<optix::int3, unsigned int>> &triangleIndexFaceBuffer, int &uniqueFaces);
		//Moving meshes functions
			OpalDynamicMeshGroup* addDynamicMeshGroup(int groupId);
			void removeDynamicMeshGroup(int groupId);
			void  addMeshToGroup(int id, int meshVertexCount, optix::float3* meshVertices, int meshTriangleCount, int* meshTriangles,  MaterialEMProperties emProp);
			void addMeshWithFacesToGroup(int groupId, std::vector<optix::float3> &meshVertices,std::vector<std::pair<optix::int3, unsigned int>> &triangleIndexFaceBuffer,  MaterialEMProperties emProp); 
			void updateTransformInGroup(int groupId, optix::Matrix4x4 transformationMatrix);
			void finishDynamicMeshGroup(int groupId);
			const std::map<int, OpalDynamicMeshGroup*> &  getDynamicMeshes() const;
			void transformEdge(const Edge& original, Edge* e, optix::Matrix4x4 t); 
			Edge* addEdgeToGroup(optix::float3 p, optix::float3 v, optix::uint2 faces, optix::float3 face_a, optix::float3 face_b, optix::float3 normal_a, optix::float3 normal_b, MaterialEMProperties emProp, int id, int groupId); 
			
		//Ray spheres
		
			OpalRaySphereGenerator* getRaySphereGenerator() const;	
			void createRaySphereFromExternalBuffer(int elevationSteps, int azimuthSteps, optix::float3*  bpointer); 
			//Create and arbitrary 2D ray sphere, with ray directions provided by the user
			void createRaySphere2D(int elevationSteps, int azimutSteps, optix::float3 * rayDirections);
			//Create a 2D ray sphere in discrete steps of elevation and azimuth
			void createRaySphere2D(int elevationDelta, int azimuthDelta);
			//Create a ray sphere with fractions of degree
			//Now elevation delta and azimuthDelta are a decimal fraction of degree, that is, every unit is 0.1 degree, so elevationDelta=1 means 0.1 degree, elevationDelta=2 means 0.2 degree
			void createRaySphere2DSubstep(int elevationDelta, int azimuthDelta);
			//Create a 2D ray sphere with arbitray angular separation and solid angle coverage
			void createRaySphere2D(float initElevation, float elevationDelta, float endElevation, float initAzimuth, float azimuthDelta, float endAzimuth);
		
		//Antenna Gains
			int registerAntennaGain(AntennaGain& gain);
			void registerReceiverGain(int rxId, int gainId); 
			void registerTransmitterGain(int txId, int gainId); 
			optix::Buffer getAntennaGainBuffer(int gainId);
			optix::Matrix<4,4> computeMatrixFromWorldToPolarization(float3 pol); 


		//Receivers
			void addReceiver(int id, optix::float3  position, optix::float3 polarization, float radius, std::function<void(float, int)>  callback);
			void removeReceiver(int id);
			void updateReceiver(int id, optix::float3 position);
			void updateReceiver(int id, optix::float3 position, optix::float3 polarization, float radius);
			void updateReceiver(int id, optix::float3 position, float radius);
			void clearReceivers();
		//Transmitters
			TransmitterManager* getTransmitterManager() ;
		//Transmit
			void transmit(int txId, float txPower, optix::float3 origin, optix::float3 polarization, bool partial=false);
			void groupTransmit(bool partial=false);
			
	
		//Finish building scene
			void finishSceneContext();

		//Configure launch
			ConfigurationOptions getConfigurationOptions() const;
			void setSimulation(OpalSimulation* sim);
			void setUseAntennaGain(bool use);
			void setExecuteCallback(bool execute);
			void enableMultitransmitter();
			void enableMultiChannel();
			void disableMultiChannel();
			void enableMultiGPU();
			void disableMultiGPU();
			void enablePenetration();
			void disablePenetration();
			void enableDepolarization(); 
			void disableDepolarization();
			void setFrequency(float f);
			void useApproximateSpeedLight();
			//Set max attenuation for penetration in dB. Rays with a higher attenuation are not traced for penetration 
			void setAttenuationLimit(float f);
			void setMaxReflections(unsigned int m);
			unsigned int  getMaxReflections() const {return this->maxReflections;};
			void setMinEpsilon(float f);
			//Disabling fast math results in higher accuracy but worse performance
			void enableFastMath();
			void disableFastMath();
			
			void setRayRange(float initElevation, float elevationDelta,  float initAzimuth, float azimuthDelta, unsigned int elevationSteps,unsigned int azimuthSteps);
			void setRaySphereParameters(unsigned int elevationSteps, unsigned int azimuthSteps, unsigned int standardSphere);
			
			// To generate rays directly on the launch.  Improves performance when rays change from launch to launch. It is also necessary when the number of rays is high,
			// since no memory is allocated to them
			// Otherwise, create once a ray sphere. TODO: test whether using a ray sphere is better than using always generation on launch 
			void enableGenerateRaysOnLaunch();
			void disableGenerateRaysOnLaunch();
			
			void setBaseDir(std::string b);
			
			void endPartialLaunch(uint numTransmitters);
			
			void enableExceptions();

			uint getNumberOfFaces();
			void setInitialHash(uint h);
			//void setMaxAngleForDuplicateRays(float angle); //In radians, all rays whose direction at departure is separated less than angle are considered duplicates when filtering
			std::map<std::string,optix::Program>& getDefaultPrograms();
			optix::Material getDefaultMeshMaterial() ;
			optix::Material getDefaultCurvedMeshMaterial() ;
			optix::Material getDefaultReceiverMaterial() ;


			


		//Log
			void setPrintEnabled(int bufferSize);
			void setPrintEnabled(int bufferSize, optix::uint3 index);
			void setUsageReport();
			virtual std::string printSceneReport();
			virtual std::string printContextInformation();
			virtual std::string printInternalBuffersState();
		//Util
			optix::float2 getAngles(optix::float3 const ray);
			//This should probably be somewhere else, but VS refuses to link it in other places and it is at so many places now that it is annoying to refactor, so we put it here to 
			std::vector<float3>  loadVerticesFromFile(const char* file); 
			std::vector<int>  loadTrianglesFromFile(const char* file) ;
			std::vector<float4>  loadPDFromFile(const char* file); 
			std::vector<float3>  loadRaysFromFile(const char* file);
			void writeMeshToPLYFile(std::string fileName, std::vector<optix::float3>& vertices, std::vector<int>& indices, optix::Matrix4x4& transformationMatrix); 
			static void printPower(float power, int txId ); 
			AntennaGain loadGainsFromFileIndBPower(const char* file);
			//Sign function
			template <typename T> int sgn(T val) {
			    return (T(0) < val) - (val < T(0));
			}; 
			float signedAngle(optix::float3 from, optix::float3 to, optix::float3 axis); 
			bool isUsingCurvedMeshes() const;
		//Results
			void computeTotalReceivedPower(bool callCallback);
			float computeReceivedPower(optix::float2 E, unsigned int index, unsigned int txIndex,  uint raysHit); 
			void printFieldComponents(optix::float2 Ex, optix::float2 Ey, optix::float2 Ez, unsigned int index,unsigned int txIndex, uint raysHit);
			FieldInfo* getFieldInfo() {return info;};
			float getReceivedPower(int rxId, int txId);
			EComponents getReceivedE(int rxId, int txId);


				
		protected:
		//Scene
			void extractFaces(optix::float3* meshVertices, std::vector<std::pair<optix::int3, unsigned int>> &triangleIndexBuffer);


			void createSceneContext();
			void buildSceneGraph();



		//Internal buffers
			void setInternalBuffers();
			optix::Buffer setTransmitterBuffer(optix::uint tx);
			optix::Buffer setRayRangeBuffer();
			optix::Buffer setRaySphereParametersBuffer();

		//Utils
			static void callbackUsageReport(int level, const char* tag, const char* msg, void* cbdata);
		//TODO: rewrite result collection in a better way...
	};



} //namespace opal
#endif

