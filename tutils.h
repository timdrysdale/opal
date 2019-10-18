/***************************************************************/
//
//Copyright (c) 2019 Esteban Egea-Lopez http://ait.upct.es/eegea
//
/**************************************************************/

#ifndef TUTILS_H
#define TUTILS_H




#pragma once
#include <cuda_runtime.h>
#include "device_launch_parameters.h"
#include <optix_world.h>
#include <optixu/optixu_math_namespace.h>
#include <optixu/optixu_matrix_namespace.h> 
#include <optixu/optixpp_namespace.h>
#include <thrust/host_vector.h>
#include <thrust/device_vector.h>
#include "Common.h"
#include <vector>
//Functions to filter duplicate hits on device with the Thrust library

namespace opalthrustutils {
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
	class PartialLaunchState {
		thrust::device_vector<HitInfo>* previousHits;
		uint partialLaunchGlobalIndex;
		public:
			PartialLaunchState();
			virtual ~PartialLaunchState();
			void reset();
			uint  getIndex() const {return partialLaunchGlobalIndex;};
			thrust::device_vector<HitInfo>*  getDeviceHits()  {return previousHits;};
			void setIndex(uint index) { partialLaunchGlobalIndex=index;};
			std::vector<HitInfo> getHits();
		       	uint getDeviceHitsBufferSize() const {return previousHits->size();};	


	};
	//Full launches: all hits are sorted, filtered and transferred to host (or prepared to be transferred)
	uint filterHitsWithCopy(optix::Buffer hitBuffer, optix::Buffer resultBuffer, unsigned int bsize); 
	uint filterHitsWithCopyResize(optix::Buffer hitBuffer, optix::Buffer resultBuffer, unsigned int bsize); 
	thrust::host_vector<HitInfo> filterHitsAndTransferMultiGPU( optix::Buffer hitBuffer, optix::Buffer aIndex, const std::vector<int> &devices); 
	
	//For partial launches
	//Just filter but do not copy for transfer. Used for partial launches
	uint filterHits(optix::Buffer hitBuffer, uint bsize); 
	//Just copy for transfer. Used for partial launches, to transfer when partial launch is finished. Assume hits have been sorted and filtered before
	uint copyHitsToTransfer(optix::Buffer hitBuffer, optix::Buffer resultBuffer, uint bsize);
	//uint filterHitsMultiGPU( optix::Buffer hitBuffer,  optix::Buffer aIndex, const std::vector<int> &devices,  thrust::device_vector<HitInfo> &vt, uint previousSize );
	uint filterHitsMultiGPU( optix::Buffer hitBuffer,  optix::Buffer aIndex, const std::vector<int> &devices,  PartialLaunchState* state, uint maxGlobalBufferSize );
	
}
#endif

