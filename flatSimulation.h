/***************************************************************/
//
//Copyright (c) 2019 Esteban Egea-Lopez http://girtel.upct.es/~eegea
//
/**************************************************************/

#ifndef FLATSIMULATION_H
#define FLATSIMULATION_H
#include "ptxUtil.h"
#include "basicSimulation.h"

namespace opal {

	//Simulation for scenarios with only flat elements (walls) and arbitrary linear polarization
	class  LPFlatMeshReflectionSimulation : public  BasicFlatMeshReflectionSimulation {
		protected:
			virtual void setOtherDirectory() override;
		public:	
			LPFlatMeshReflectionSimulation(OpalSceneManager*  m);
			virtual std::string printConfigInfo() const override; 
			virtual void updateReceiver(int id, float3 position, float3 polarization, float radius) override;
			virtual void addReceiver(int id, float3  position, float3 polarization, float radius, std::function<void(float, int)>  callback, std::vector<optix::Material>& materials) override;
	};
}
#endif

