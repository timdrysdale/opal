/***************************************************************/
//
//Copyright (c) 2021 Esteban Egea-Lopez http://girtel.upct.es/~eegea
//
/**************************************************************/
#ifndef DIFFRACTION_H
#define DIFFRACTION_H
#include "../Opal.h"
namespace opal {
	class DiffractionTests {
		protected:
			OpalSceneManager* sceneManager;
			float sphereRadius;
			bool useDepolarization;
			void addCubeEdges(float3 sw,float length, uint index, MaterialEMProperties emProp1);
			void loadSWCornerAsGroup(float xsize, float ysize, float zsize, MaterialEMProperties emProp1);
			void loadSWCorner(float xsize, float ysize, float zsize, MaterialEMProperties emProp1);
			void loadSWAcuteCorner(float xsize, float ysize, float zsize, float aperture,MaterialEMProperties emProp1);
			void loadRaggedCorners(float xsize, float ysize, float zsize, float aperture,MaterialEMProperties emProp1);
			void loadCrossing(MaterialEMProperties emProp1);

		public:
			DiffractionTests(OpalSceneManager* sceneManager, float sphereRadius, bool useDepolarization); 
			void semiplaneDiffraction();
			void semiplaneTotal();
			void runSWCornerDynamicMesh(); 
			void runSWCorner(bool multitransmitter); 
			void runSWAcuteCorner(); 
			void runCrossing(); 
			void  addCompoundDynamicMeshes(); 
	};
}
#endif


