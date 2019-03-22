/***************************************************************/
//
//Copyright (c) 2019 Esteban Egea-Lopez http://ait.upct.es/eegea
//
/**************************************************************/


#pragma once


#include "tutils.h"
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
			optix::Buffer resultHitInfoBuffer;
			optix::Buffer atomicIndexBuffer;


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
			unsigned int numberOfFaces;



			unsigned int maxReflections; //Default 10
			float minEpsilon; //Default 1.e-4f
			bool useExactSpeedOfLight; //speed of light (can be exact or approximated)

#ifdef OPALDEBUG
			std::ofstream outputFile;
#endif // OPALDEBUG


			float radioReductionFraction;
			//Default programs

			std::map<std::string,optix::Program> defaultPrograms;
			optix::Material defaultMeshMaterial;


		public:
		//Init
			OpalSceneManager();
			OpalSceneManager(float f,  bool useExactSpeedOfLight=true);
			virtual ~OpalSceneManager();
			void initContext(float f, bool useExactSpeedOfLight=true);
			virtual void initMembers();
			void setFrequency(float f);
			void setMaxReflections(unsigned int m);
			
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
		//Ray sphere
			void createRaySphere2D(int elevationSteps, int azimutSteps, optix::float3 * rayDirections);
			void createRaySphere2D(int elevationDelta, int azimuthDelta);

			void createRaySphere2DSubstep(int elevationDelta, int azimuthDelta);
		//Receivers
			void addReceiver(int id, optix::float3  position, float radius, std::function<void(float, int)>  callback);
			void removeReceiver(int id);
			void updateReceiver(int id, optix::float3 position);
			void updateReceiver(int id, optix::float3 position, float radius);
		//Transmit
			void transmit(int txId, float txPower, optix::float3 origin, optix::float3 polarization);

	
		//Finish building scene
			void finishSceneContext();
		//Log
			void setPrintEnabled(int bufferSize);
			void setUsageReport();
			virtual std::string printInternalBuffersState();
			virtual std::string printSceneReport();

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
			virtual optix::Program createClosestHitInternalRay();


			optix::Program createBoundingBoxTriangle();
			#ifdef OPAL_USE_TRI
			optix::Program createTriangleAttributesProgram();
			#endif

			optix::Program createIntersectionTriangle();

			optix::Program createBoundingBoxSphere();

			optix::Program createIntersectionSphere();

			virtual optix::Program createMissProgram();

			virtual optix::Program createRayGenerationProgram();


		//Complex functions and arithmetic programs
			optix::Program createComplexScaProd();
			optix::Program createComplexExpImaginary();
			optix::Program createComplexSqrt();
			optix::Program createComplexDiv();
			optix::Program createComplexProd();

		//Internal buffers
			virtual void setInternalBuffers();
			virtual void checkInternalBuffers();
			virtual void clearInternalBuffers();

			virtual optix::Buffer setGlobalHitInfoBuffer(optix::uint ele, optix::uint azi, optix::uint rx, optix::uint reflections);
			void resizeGlobalHitInfoBuffer(optix::uint ele, optix::uint azi, optix::uint rx, optix::uint reflections);
			virtual optix::Buffer setResultHitInfoBuffer(optix::uint rx, optix::uint reflections);
			optix::Buffer setAtomicIndexBuffer();

		//Utils
			static void callbackUsageReport(int level, const char* tag, const char* msg, void* cbdata);
			void computeReceivedPower(optix::float2 E, unsigned int index, int txId, float txPower, optix::float3 origin);
	};



	//From NVIDIA samples
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

//Load meshes
/*std::vector<optix::float3>  loadVerticesFromFile(const char* file);
std::vector<int>  loadTrianglesFromFile(const char* file);

//Tests

std::unique_ptr<opal::OpalSceneManager> crossingTest(std::unique_ptr<opal::OpalSceneManager> sceneManager, bool print, bool subSteps);
std::unique_ptr<opal::OpalSceneManager> planeTest(std::unique_ptr<opal::OpalSceneManager> sceneManager, bool print, bool subSteps);
std::unique_ptr<opal::OpalSceneManager> quadTest(std::unique_ptr<opal::OpalSceneManager> sceneManager, bool print, bool subSteps);
std::unique_ptr<opal::OpalSceneManager> addRemoveReceivers(std::unique_ptr<opal::OpalSceneManager> sceneManager);
std::unique_ptr<opal::OpalSceneManager> moveReceivers(std::unique_ptr<opal::OpalSceneManager> sceneManager);
std::unique_ptr<opal::OpalSceneManager> addRemoveDynamicMeshes(std::unique_ptr<opal::OpalSceneManager> sceneManager, bool print, bool subSteps);
std::unique_ptr<opal::OpalSceneManager> addCompoundDynamicMeshes(std::unique_ptr<opal::OpalSceneManager> sceneManager);
std::unique_ptr<opal::OpalSceneManager> crossingTestAndVehicle(std::unique_ptr<opal::OpalSceneManager> sceneManager);
*/
