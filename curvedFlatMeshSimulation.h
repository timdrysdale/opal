/***************************************************************/
//
//Copyright (c) 2019 Esteban Egea-Lopez http://girtel.upct.es/~eegea
//
/**************************************************************/

#ifndef CURVEDFLATMESHSIMULATION_H
#define CURVEDFLATMESHSIMULATION_H
#include "curvedMeshSimulation.h"

namespace opal {
	//Simulation for scenarios with a mix of curved surfaces and  flat elements (walls) and arbitrary linear polarization
	class  LPCurvedFlatMeshReflectionSimulation : public LPCurvedMeshReflectionSimulation {
		public:
			LPCurvedFlatMeshReflectionSimulation(OpalSceneManager*  m);
			virtual std::string printConfigInfo() const override; 
			virtual void executeTransmitLaunch(uint numTransmitters, bool partial) override;
			virtual void endPartialLaunch(uint numTransmitters) override;

	};


}


#endif
