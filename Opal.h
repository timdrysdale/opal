#pragma once


#include <optix_world.h>
#include <map>
#include <fstream>
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

	/*struct Duplicates {
		optix::Buffer duplicatesBuffer;
		optix::uint2  duplicateBlockSize; //Block size for [elevation, azimuth]. Default (3,3). Consider changing it 
	};
	*/

	class SphereReceiver {
	public:
		optix::float3 position;
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

	const float deg2rad = 0.017453292f;  //Degrees to radians

	class OpalSceneManager {
	protected:
		optix::Context context;
		

		//Internal buffers
		typedef struct {
			optix::uint elevation;
	     	     	optix::uint azimuth;
	     	     	optix::uint tx;
	     	     	optix::uint rx;
	     	     	optix::uint faces;
	     	     	optix::uint reflections; 
		} InternalBuffersParameters;	 //Internal buffers depend on these parameters, used to keep track of the changes

		InternalBuffersParameters currentInternalBuffersState;

		optix::Buffer receptionInfoBuffer;
		optix::Buffer internalRaysBuffer;
		std::vector<optix::Buffer> internalRays;
		
		optix::Buffer facesMinDBuffer;
		optix::Buffer facesMinEBuffer;

		//Scene graph variables
		optix::GeometryGroup receiversGroup;
		optix::Group rootGroup;
		optix::GeometryGroup staticMeshesGroup;
		std::vector<OpalMesh> staticMeshes;

		//External to internal info
		std::map<int, unsigned int> receiverExtToBufferId; //External id to receptionInfo buffer Id
		std::vector<SphereReceiver*> receivers; //receivers info (mapped to receptionInfo buffer)
		std::map<int, BaseTransmitter*> transmitterExtToBase; //Map from externalId to transmitterBase
	public:
		bool sceneGraphCreated;
		bool sceneFinished;
		int transmissionLaunches;
		RaySphere raySphere;
		ChannelParameters defaultChannel;
		std::map<int, OpalDynamicMeshGroup*> dynamicMeshes; //externalId to dynamic mesh

		//Control parameters
		//Face-based duplicates
		unsigned int numberOfFaces;
	
	

		unsigned int maxReflections; //Default 10
		float minEpsilon; //Default 1.e-4f

#ifdef OPALDEBUG
		std::ofstream outputFile;
#endif // OPALDEBUG
		bool holdReflections;
		bool useInternalTracing;	


		float radioReductionFraction;
		//Default programs

		std::map<std::string,optix::Program> defaultPrograms;
		optix::Material defaultMeshMaterial;


	public:
		OpalSceneManager();
		OpalSceneManager(float f, bool useInternalTracing, bool holdReflections=false);
		virtual ~OpalSceneManager();
		void initContext(float f,bool useInternalTracing, bool holdReflections=false);
		virtual void initMembers();
		void setFrequency(float f);
		void setMaxReflections(unsigned int m);
		void setMeshEMProperties(optix::GeometryInstance geom_instance, MaterialEMProperties emProp);
		OpalMesh createStaticMesh(int meshVertexCount, optix::float3* meshVertices, int meshTriangleCount, int* meshTriangles, optix::Matrix4x4 transformationMatrix, optix::Program intersectionProgram, optix::Program boundingBoxProgram, optix::Material material);
		OpalMesh createMesh(int meshVertexCount, optix::float3* meshVertices, int meshTriangleCount, int* meshTriangles, optix::Program intersectionProgram, optix::Program boundingBoxProgram, optix::Material material);

		OpalDynamicMeshGroup* addDynamicMeshGroup(int id);
		void removeDynamicMeshGroup(int id);
		void  addMeshToGroup(int id, int meshVertexCount, optix::float3* meshVertices, int meshTriangleCount, int* meshTriangles,  MaterialEMProperties emProp);
		void updateTransformInGroup(int id, optix::Matrix4x4 transformationMatrix);
		void finishDynamicMeshGroup(int id);

		OpalMesh addStaticMesh(int meshVertexCount, optix::float3* meshVertices, int meshTriangleCount, int* meshTriangles, optix::Matrix4x4 transformationMatrix, MaterialEMProperties emProp);
		void addStaticMesh(OpalMesh mesh);
		MaterialEMProperties ITUparametersToMaterial(float a, float b, float c, float d);


		void createRaySphere2D(int elevationSteps, int azimutSteps, optix::float3 * rayDirections);
		void createRaySphere2D(int elevationDelta, int azimuthDelta);

		void createRaySphere2DSubstep(int elevationDelta, int azimuthDelta);

		//void setDuplicateBlockSize(unsigned int elevationBlockSize, unsigned int azimuthBlockSize);
		void addReceiver(int id, optix::float3  position, float radius, std::function<void(float, int)>  callback);
		void removeReceiver(int id);
		void updateReceiver(int id, optix::float3 position);
		void updateReceiver(int id, optix::float3 position, float radius);
		
		void transmit(int txId, float txPower, optix::float3 origin, optix::float3 polarization);
		

		void finishSceneContext();

		void setPrintEnabled(int bufferSize);
		void setUsageReport();
		virtual std::string printInternalBuffersState();
		virtual std::string printSceneReport();
	
	protected:
		
		optix::float2 sumReflections(unsigned int receiver);
		void extractFaces(optix::float3* meshVertices, std::vector<std::pair<optix::int3, unsigned int>> &triangleIndexBuffer);


		void createSceneContext();
		void buildSceneGraph();

		//Default programs
		void setDefaultPrograms();
		optix::Material createMeshMaterial(unsigned int ray_type_index, optix::Program closestHitProgram);
			
		optix::Program createClosestHitMesh();

		virtual optix::Program createClosestHitReceiver();
		virtual optix::Program createClosestHitInternalRay();

		virtual optix::Program createClosestHitReceiverHoldReflections();

		optix::Program createBoundingBoxTriangle();

		optix::Program createIntersectionTriangle();

		optix::Program createBoundingBoxSphere();

		optix::Program createIntersectionSphere();

		virtual optix::Program createMissProgram();

		virtual optix::Program createRayGenerationProgram();

		virtual optix::Program createInitializationProgram();

		//Complex functions and arithmetic programs
		optix::Program createComplexScaProd();
		optix::Program createComplexExpImaginary();
		optix::Program createComplexSqrt();
		optix::Program createComplexDiv();
		optix::Program createComplexProd();


		virtual void setInternalBuffers();
		virtual void checkInternalBuffers();
		virtual void clearInternalBuffers();
		optix::Buffer setReceptionInfoBuffer(optix::uint rx);
		optix::Buffer setInternalRaysBuffer(optix::uint rx,  optix::uint elevationSteps, optix::uint azimuthSteps); 
		optix::Buffer setFacesMinDBuffer(optix::uint rx, optix::uint reflections, optix::uint faces);
		optix::Buffer setFacesMinEBuffer(optix::uint rx,  optix::uint reflections, optix::uint faces);
		optix::Buffer setFacesMinEBufferHoldReflections(optix::uint rx,  optix::uint reflections, optix::uint faces);
		
		virtual void updateReceiverBuffers(optix::uint oldReceivers, optix::uint newReceivers);
		void recreateReceiverBuffers();
		void updateFacesBuffers();
		void updateTransmitterBuffers(unsigned int tx); 
		static void callbackUsageReport(int level, const char* tag, const char* msg, void* cbdata);
	};



	//Derived class for multi-transmitter and multi-receiver operation. Supports parallel transmission of multiple nodes 
	class OpalSceneManagerMultiTransmitter : public OpalSceneManager {
		public:
		virtual void initMembers() override;
		OpalSceneManagerMultiTransmitter(float f, bool holdReflections=false);

		
		//Register transmitter in Opal. Add to map
		void addTransmitter(int txId, optix::float3 origin, optix::float3 polarization, float transmitPower) ;
		//Find transmitter and remove from Opal. Cannot transmit anymore 
		void removeTransmitter(int txId) ;
		//Add transmitter to next parallel transmission
		void addTransmitterToGroup(int txId,float transmitPower, optix::float3 origin,optix::float3 polarization); 
		//Add transmitter to next parallel transmission
		void addTransmitterToGroup(int txId,float transmitPower, optix::float3 origin); 
		//Clear current transmit group
		void clearGroup(); 
		//Transmit simultaneously all transmitters in group
		void groupTransmit() ;
		
		virtual std::string printInternalBuffersState() override;
		virtual std::string printSceneReport() override;
		
		protected:


		//We have to split the minimum-distance electric field internal buffer (for duplicate removal) into two buffers: for real and imaginary parts of the Electric filed. 
		//Otherwise we get invalid memory access on the device side. See the multiTransmitter.cu for an explanation
		optix::Buffer facesMinExBuffer;
		optix::Buffer facesMinEyBuffer;
		std::vector<optix::Buffer> rxFacesMinExBuffers;
		std::vector<optix::Buffer> rxFacesMinEyBuffers;
		std::vector<optix::Buffer> rxFacesMinDBuffers;


		std::vector<BaseTransmitter*> activeTransmitters; //Map from launchIndex.z to transmitterBase. Used for grouping transmissions (batches)
		optix::Buffer txOriginBuffer;

		optix::float2 sumReflections(unsigned int tx, unsigned int receiver);
		
		virtual optix::Program createClosestHitReceiver() override;
		virtual optix::Program createClosestHitReceiverHoldReflections() override;
		virtual optix::Program createMissProgram() override;

		virtual optix::Program createRayGenerationProgram() override;

		virtual optix::Program createInitializationProgram() override;
		
		virtual void setInternalBuffers() override;
		virtual void checkInternalBuffers() override;
		virtual void clearInternalBuffers() override;
		
		optix::Buffer setReceptionInfoBuffer(optix::uint receivers, optix::uint tx);
		optix::Buffer setTransmittersOriginBuffer(optix::uint tx);
		optix::Buffer setInternalRaysBuffer(optix::uint rx, optix::uint tx, optix::uint elevationSteps, optix::uint azimuthSteps); 
		optix::Buffer setFacesMinDBuffer(optix::uint rx, optix::uint tx, optix::uint reflections, optix::uint faces);
		optix::Buffer setFacesMinEBuffers(optix::uint rx, optix::uint tx, optix::uint reflections, optix::uint faces);
		optix::Buffer setFacesMinEBuffersHoldReflections(optix::uint rx, optix::uint tx, optix::uint reflections, optix::uint faces);
		virtual void updateReceiverBuffers(optix::uint oldReceivers, optix::uint newReceivers) override;
	};	

#ifndef TIMER_H
#define TIMER_H

#if defined(_WIN32)
#include <Windows.h>
#else
#include <sys/time.h>
#endif


	/*! \brief A simple timer class.
	* This timer class can be used on Windows and Linux systems to
	* measure time intervals in seconds.
	* The timer can be started and stopped several times and accumulates
	* time elapsed between the start() and stop() calls. */
	class Timer
	{
	public:
		//! Default constructor. Constructs a Timer, but does not start it yet. 
		Timer();

		//! Default destructor.
		~Timer();

		//! Starts the timer.
		void start();

		//! Stops the timer.
		void stop();

		//! Resets the timer.
		void reset();

		//! Resets the timer and starts it.
		void restart();

		//! Returns the current time in seconds.
		double getTime() const;

		//! Return whether the timer is still running.
		bool isRunning() const { return m_running; }

	private:
#if defined(_WIN32)
		typedef LARGE_INTEGER Time;
#else
		typedef timeval Time;
#endif

	private:
		double calcDuration(Time begin, Time end) const;

	private:
#if defined(_WIN32)
		LARGE_INTEGER m_freq;
#endif
		Time   m_begin;
		bool   m_running;
		double m_seconds;
	};

#endif // TIMER_H
} //namespace opal

  //Tests
std::vector<optix::float3>  loadVerticesFromFile(const char* file);
std::vector<int>  loadTrianglesFromFile(const char* file);
std::unique_ptr<opal::OpalSceneManager> crossingTest(std::unique_ptr<opal::OpalSceneManager> sceneManager, bool print, bool subSteps);
std::unique_ptr<opal::OpalSceneManager> planeTest(std::unique_ptr<opal::OpalSceneManager> sceneManager, bool print, bool subSteps);
std::unique_ptr<opal::OpalSceneManager> quadTest(std::unique_ptr<opal::OpalSceneManager> sceneManager, bool print);
std::unique_ptr<opal::OpalSceneManager> addRemoveReceivers(std::unique_ptr<opal::OpalSceneManager> sceneManager);
std::unique_ptr<opal::OpalSceneManager> moveReceivers(std::unique_ptr<opal::OpalSceneManager> sceneManager);
std::unique_ptr<opal::OpalSceneManager> addRemoveDynamicMeshes(std::unique_ptr<opal::OpalSceneManager> sceneManager, bool print, bool subSteps);
std::unique_ptr<opal::OpalSceneManager> addCompoundDynamicMeshes(std::unique_ptr<opal::OpalSceneManager> sceneManager);
std::unique_ptr<opal::OpalSceneManager> crossingTestAndVehicle(std::unique_ptr<opal::OpalSceneManager> sceneManager);
std::unique_ptr<opal::OpalSceneManagerMultiTransmitter> seqParallelTxTest(std::unique_ptr<opal::OpalSceneManagerMultiTransmitter> sceneManager);
