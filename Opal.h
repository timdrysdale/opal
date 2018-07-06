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
	};

	class BaseTransmitter {
	public:
		optix::float3 origin;
		optix::float3 polarization;
		int externalId;
		bool isDirty;
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
		std::vector<OpalMesh> staticMeshes;
		

		
		optix::Buffer receptionInfoBuffer;
		optix::Buffer txPowerBuffer;
		optix::Buffer txOriginBuffer;
		//optix::Buffer minReflectionE;
		optix::Buffer internalRaysBuffer;
		std::vector<optix::Buffer> internalRays;
		//optix::Buffer facesBuffer;
		optix::Buffer facesMinDBuffer;
		optix::Buffer facesMinEBuffer;

		//Scene structure variables
		optix::GeometryGroup receiversGroup;
		optix::Group rootGroup;
		optix::GeometryGroup staticMeshesGroup;

		std::map<int, unsigned int> receiverExtToBufferId; //External id to receptionInfo buffer Id
		std::vector<SphereReceiver*> receivers; //receivers info (mapped to receptionInfo buffer)
		std::map<int, BaseTransmitter*> transmitters; //Map from launchIndex.z to transmitter
	public:
		bool sceneGraphCreated;
		bool sceneFinished;
		int transmissions;
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
		bool holdReflections;
#endif // OPALDEBUG


		


		//Default programs

		std::map<std::string,optix::Program> defaultPrograms;
		optix::Material defaultMeshMaterial;


	public:
		OpalSceneManager(float f, bool holdReflections=false);
		virtual ~OpalSceneManager();
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
	
	protected:
		
		optix::float2 sumReflections(DuplicateReflection* ref_host, unsigned int receiver);
		void extractFaces(optix::float3* meshVertices, std::vector<std::pair<optix::int3, unsigned int>> &triangleIndexBuffer);


		void createSceneContext();
		void buildSceneGraph();

		//Default programs
		void setDefaultPrograms();
		optix::Material createMeshMaterial(unsigned int ray_type_index, optix::Program closestHitProgram);
			
		optix::Program createClosestHitMesh();

		optix::Program createClosestHitReceiver();

		optix::Program createClosestHitReceiverHoldReflections();

		optix::Program createBoundingBoxTriangle();

		optix::Program createIntersectionTriangle();

		optix::Program createBoundingBoxSphere();

		optix::Program createIntersectionSphere();

		optix::Program createMissProgram();

		optix::Program createRayGenerationProgram();

		optix::Program createInitializationProgram();

		//Complex functions and arithmetic programs
		optix::Program createComplexScaProd();
		optix::Program createComplexExpImaginary();
		optix::Program createComplexSqrt();
		optix::Program createComplexDiv();
		optix::Program createComplexProd();



		optix::Buffer setReceptionInfoBuffer(optix::uint receivers, optix::uint transmitters);
		optix::Buffer setTransmittersOriginBuffer(optix::uint transmitters);
		//optix::Buffer setDuplicatesBuffer(optix::uint receivers, optix::uint transmitters, optix::uint2 duplicateBlockSize);
		optix::Buffer setInternalRaysBuffer(optix::uint receivers, optix::uint transmitters);
		optix::Buffer setFacesBuffer(optix::uint receivers);
		optix::Buffer setFacesMinDBuffer(optix::uint receivers);
		optix::Buffer setFacesMinEBuffer(optix::uint receivers);
		optix::Buffer setFacesMinEBuffersHoldReflections(optix::uint receivers);
		void updateReceiverBuffers(optix::uint oldReceivers, optix::uint newReceivers);
		void recreateReceiverBuffers();
		void updateFacesBuffers();
		void printInternalBuffersState();
		void printSceneReport();
		static void callbackUsageReport(int level, const char* tag, const char* msg, void* cbdata);
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
std::unique_ptr<opal::OpalSceneManager> crossingTest(std::unique_ptr<opal::OpalSceneManager> sceneManager);
std::unique_ptr<opal::OpalSceneManager> planeTest(std::unique_ptr<opal::OpalSceneManager> sceneManager);
std::unique_ptr<opal::OpalSceneManager> quadTest(std::unique_ptr<opal::OpalSceneManager> sceneManager);
std::unique_ptr<opal::OpalSceneManager> addRemoveReceivers(std::unique_ptr<opal::OpalSceneManager> sceneManager);
std::unique_ptr<opal::OpalSceneManager> moveReceivers(std::unique_ptr<opal::OpalSceneManager> sceneManager);
std::unique_ptr<opal::OpalSceneManager> addRemoveDynamicMeshes(std::unique_ptr<opal::OpalSceneManager> sceneManager);
std::unique_ptr<opal::OpalSceneManager> addCompoundDynamicMeshes(std::unique_ptr<opal::OpalSceneManager> sceneManager);
std::unique_ptr<opal::OpalSceneManager> crossingTestAndVehicle(std::unique_ptr<opal::OpalSceneManager> sceneManager);