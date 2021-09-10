/***************************************************************/
//
//Copyright (c) 2019 Esteban Egea-Lopez http://girtel.upct.es/~eegea
//
/**************************************************************/

#ifndef BASICSIMULATION_H
#define BASICSIMULATION_H
#include "ptxUtil.h"
#include "opalSimulation.h"

namespace opal {
	//Simulation for scenarios with only flat elements (walls) and only vertical (0,1,0) or horizontal (1,0,0) polarization
	class  BasicFlatMeshReflectionSimulation: public  OpalSimulation {
		protected:
			
			opalthrustutils::PartialLaunchState<HitInfo>* partialLaunchState;
			
	//		virtual void processLaunch(HitInfo* host_hits, uint hits, uint numTransmitters);
		//Hits processing
	//	#ifdef OPAL_EXTENDED_HITINFO
	//		virtual void  processHitsExtended(HitInfo* host_hits, uint hits);
	//	#else	
	//		virtual void  processHits(HitInfo* host_hits, uint hits);
	//	#endif

		public:
			BasicFlatMeshReflectionSimulation(OpalSceneManager*  m);
			virtual ~BasicFlatMeshReflectionSimulation();
			virtual void setOtherDirectory() override;
			//virtual void setDefaultPrograms() override;
			//virtual void setDefaultPrograms(std::map<std::string,optix::Program>& defaultPrograms, optix::Material& defaultMeshMaterial) override;
			virtual std::string printConfigInfo() const override; 
			virtual void executeTransmitLaunch(uint numTransmitters, bool partial) override;
			virtual void endPartialLaunch(uint numTransmitters) override;
			virtual void updateReceiver(int id, float3 position, float3 polarization, float radius) override;
			virtual void addReceiver(int id, float3  position, float3 polarization, float radius, std::function<void(float, int)>  callback, std::vector<optix::Material>& materials) override;
	};


}


#endif
