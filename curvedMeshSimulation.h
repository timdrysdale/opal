/***************************************************************/
//
//Copyright (c) 2019 Esteban Egea-Lopez http://girtel.upct.es/~eegea
//
/**************************************************************/

#ifndef CURVEDMESHSIMULATION_H
#define CURVEDMESHSIMULATION_H
#include "ptxUtil.h"
#include "opalSimulation.h"
namespace opal {
	//Simulation for scenarios with a only curved surfaces  and arbitrary linear polarization


	//Used to compare hits by initial ray direction	
	struct CheckAngle {
		HitInfo current;
		float cosAngleDiscrimination;
		bool IsInAngle(HitInfo& other) {
			//	bool inangle=optix::dot(current.rayDir,other.rayDir)>=cosAngleDiscrimination;
			//		std::cout<<"\t current "<<current.rayDir<<"\t"<<current.r<<"\t"<<current.thrd.y<<"\t"<<current.cosAngleDiscrimination<<"\t"<<current.thrd<<std::endl;
			//		std::cout<<"\t other"<<other.rayDir<<"\t"<<current.r<<"\t"<<current.thrd.y<<"\t"<<current.cosAngleDiscrimination<<"\t"<<current.thrd<<std::endl;
			//		std::cout<<"inangle="<<inangle<<std::endl;
			//std::cout<<"a="<<(acosf(optix::dot(current.rayDir,other.rayDir))*180/M_PI)<<"ina="<<inangle<<std::endl;
			return (optix::dot(make_float3(current.rdud),make_float3(other.rdud))>=cosAngleDiscrimination);
			//return inangle;
		}
	};
	
	class  LPCurvedMeshReflectionSimulation: public  OpalSimulation {
		protected:
			opalthrustutils::PartialLaunchState<HitInfo>* partialLaunchState;
			bool useAngleDiscrimination;
			virtual void setOtherDirectory() override;
			float cosAngleDuplicateRays;
			
		//	virtual void processLaunch(HitInfo* host_hits, uint hits, uint numTransmitters);
			virtual void createIntersectionPrograms() override;
			virtual void createClosestHitPrograms() override;
			
			virtual optix::Material createDefaultMeshMaterial(unsigned int ray_type_index, optix::Program closestHitProgram) override;

			virtual optix::Program createClosestHitMesh() override;

			virtual optix::Program createClosestHitReceiver() override;

		//For debug	
			optix::Buffer invalidRaysBuffer;

			virtual	optix::Program createClosestHitCurvedMesh();
			virtual	optix::Program createCurvedTriangleAttributesProgram();
			virtual	optix::Program createIntersectionTriangleCurved();



			
			void getUniqueByAngle(std::vector<HitInfo>& filtered, std::vector<HitInfo>& bucket);
			std::vector<HitInfo> filterByAngle(HitInfo* host_hits, uint hits); 
		//Hits processing
		//#ifdef OPAL_EXTENDED_HITINFO
		//	virtual void  processHitsExtended(HitInfo* host_hits, uint hits);
		//#else	
		//	virtual void  processHits(HitInfo* host_hits, uint hits);
		//#endif
		public:
			LPCurvedMeshReflectionSimulation(OpalSceneManager*  m);
			virtual ~LPCurvedMeshReflectionSimulation();
			//virtual void setDefaultPrograms() override;
			//virtual void setDefaultPrograms(std::map<std::string,optix::Program>& defaultPrograms, optix::Material& defaultMeshMaterial) override;
			virtual std::string printConfigInfo() const override; 
			virtual void executeTransmitLaunch(uint numTransmitters, bool partial) override;
			virtual void endPartialLaunch(uint numTransmitters) override;
			virtual void disableAngleDiscrimination();
			virtual void enableAngleDiscrimination();
			virtual void setInternalBuffers() override;
			virtual std::string printInternalBuffersState() override;
			//Specific for curved surface simulation
			void setMaxAngleForDuplicateRays(float angle); //In radians, all rays whose direction at departure is separated less than angle are considered duplicates when filtering
			void addStaticCurvedMesh(OpalMesh& mesh, std::vector<optix::Material>& materials) override;
		
	};


}


#endif
