
/***************************************************************/
//
//Copyright (c) 2019 Esteban Egea-Lopez http://girtel.upct.es/~eegea
//
/**************************************************************/

#ifndef SINGLEDIFFRACTION_H
#define SINGLEDIFFRACTION_H
#include "opalSimulation.h"

namespace opal {
	class  SingleDiffraction: public  OpalSimulation {
		protected:
			virtual void setOtherDirectory() override;
			virtual optix::Buffer setEdgeBuffer();
			virtual optix::Buffer setReceiverPositionsBuffer();
			virtual optix::Buffer setHitBuffer();
			virtual void fillEdgeBuffer();
			virtual void fillReceiverPositionsBuffer();
			virtual void fillAntennaGainIdBuffer(); 
			optix::Buffer edgeBuffer;
			optix::Buffer receiverPositionsBuffer;
			optix::Buffer hitBuffer;
			optix::Buffer antennaGainIdBuffer;
			optix::Buffer transformToPolarizationBuffer;
			bool updateReceiverBuffer;
			bool updateEdgeBuffer;
			unsigned int diffractionEntryIndex;
			unsigned int diffractionRayIndex;
			optix::Program  createComputeSimpleDiffractionProgram();
			optix::Program createMissDiffractionProgram() ;
			optix::Program createExceptionDiffractionProgram() ;
			optix::Program createClosestHitMeshDiffractionProgram(); 
			optix::Program createClosestHitCurvedMeshDiffractionProgram(); 
			virtual void createClosestHitPrograms() override;
			virtual void addReceiver(int id, float3  position, float3 polarization, float radius, std::function<void(float, int)>  callback, std::vector<optix::Material>& materials) override;
			//virtual void addReceiver(int id, float3  position, float3 polarization, float radius, std::function<void(float, int)>  callback) override;
			virtual void removeReceiver(int id) override;
			virtual void updateReceiver(int id, float3 position, float3 polarization, float radius) override;
			virtual void processDiffractionLaunch();
			virtual void saveTraceToFile(thrust::host_vector<LogTraceHitInfo> trace, std::string fileName) override;
			void processTraceLog(unsigned int maxTraceSize);
			void checkBuffersSize(uint nrx, uint ntx, uint ne); 
		public:	
			SingleDiffraction(OpalSceneManager*  m);
			virtual std::string printConfigInfo() const override;
			virtual void init() override; 
			//virtual void setDefaultPrograms(std::map<std::string,optix::Program>& defaultPrograms, optix::Material& defaultMeshMaterial) override;
			virtual void setInternalBuffers() override;
			virtual void executeTransmitLaunch(uint numTransmitters, bool partial) override;
			virtual void endPartialLaunch(uint numTransmitters) override;
			virtual void finishSceneContext() override;
			virtual std::string printInternalBuffersState() override;
			virtual void addStaticMesh(opal::OpalMesh& mesh, std::vector<optix::Material>& materials) override;
			virtual void addStaticCurvedMesh(opal::OpalMesh& mesh, std::vector<optix::Material>& materials) override ;
			virtual void registerReceiverGain(int rxId, int gainId) override; 
			virtual void transformEdge(Edge* e, optix::Matrix4x4 t) override;	
	};
}
#endif //SIMPLEDIFFRACTION_H
