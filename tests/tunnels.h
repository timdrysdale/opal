/***************************************************************/
//
//Copyright (c) 2019 Esteban Egea-Lopez http://girtel.upct.es/~eegea
//
/**************************************************************/


#ifndef TUNNELS_H
#define TUNNELS_H
#include "../Opal.h"
#include "tunnelsBase.h"
#include <memory>
using namespace opal;
namespace opal {
//Simulate propagation inside a tunnel. Used to exemplify partial launches with low angular separation (precise angular sampling). To be run with a high number of reflections, such as 20 or 30
	class CubeTunnelTests : public TunnelsBase {
		protected:
			void xcut( float width, float height, float y, float distance, float3 polarization ); 
			void ycut( float width, float height, float x, float distance, float3 polarization) ;
			float zrun(float zinit, float deltad, float x, float y, float length, float3 polarization, float deltaSphere) ;
		public:
			CubeTunnelTests(OpalSceneManager* sceneManager, float sphereRadius, bool useDepolarization) : TunnelsBase(sceneManager,sphereRadius,useDepolarization) {}; 
			void cubeTunnelRDN(float distance);
			void cubeTunnelRDNIsotropic(float distance);
			void cubeTunnelSingleReceiver(int random);
			void cubeTunnel(int random);
			void cubeTunnelWithCurvedSimulation(int random, int simType); 
	};

}
	
#endif

