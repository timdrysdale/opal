
/***************************************************************/
//
//Copyright (c) 2019 Esteban Egea-Lopez http://girtel.upct.es/~eegea
//
/**************************************************************/
#ifndef TUNNELSBASE_H
#define TUNNELSBASE_H
#include "../Opal.h"
#include <string>
namespace opal {
	class TunnelsBase {
		protected:
			OpalSceneManager* sceneManager;
			float sphereRadius;
			bool useDepolarization;
			void loadCircularTunnel(float radius,  float length, MaterialEMProperties emProp1);
			void loadSquareTunnel(float width, float height, float length, MaterialEMProperties emProp1);
			void loadHalfCylinder(float radius,  float length, float height, MaterialEMProperties emProp1);
		public:
			TunnelsBase(OpalSceneManager* sceneManager, float sphereRadius, bool useDepolarization); 
			std::vector<int> parseTestString(std::string test);
	};
}
#endif
