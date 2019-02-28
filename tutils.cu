/***************************************************************/
//
//Copyright (c) 2019 Esteban Egea-Lopez http://ait.upct.es/eegea
//
/**************************************************************/
#include "Common.h"
#include "tutils.h"
#include <thrust/host_vector.h>
#include <thrust/device_vector.h>
#include <thrust/device_ptr.h>
#include <thrust/generate.h>
#include <thrust/sort.h>
#include <thrust/copy.h>
#include <thrust/functional.h>
#include <algorithm>
#include <cstdlib>
#include <ostream>
#include "sutil.h"
//#include <optix_world.h>
//#include <optixu/optixpp_namespace.h>
namespace opalthrustutils {


	uint filterHitsWithCopy(optix::Buffer hitBuffer, optix::Buffer resultBuffer, uint bsize) {
		//Get device pointer to global buffer
		HitInfo* raw_ptr =static_cast<HitInfo*>( hitBuffer->getDevicePointer(0));
		thrust::device_ptr<HitInfo> dev_ptr=thrust::device_pointer_cast(raw_ptr);
		//		if (raw_ptr==NULL) {
		//			std::cout<<"Null buffer pointer"<<std::endl;
		//		}
		//std::cout<<"buffer pointer="<<raw_ptr<<"size="<<bsize<<std::endl;

		//Get device pointer to final result buffer
		HitInfo* raw_ptr_r =static_cast<HitInfo*>( resultBuffer->getDevicePointer(0));
		thrust::device_ptr<HitInfo> dev_ptr_r=thrust::device_pointer_cast(raw_ptr_r);
		//Sort	
		thrust::sort(dev_ptr, dev_ptr+bsize);
		//Keep closest one and copy
		thrust::device_ptr<HitInfo> new_end=thrust::unique_copy(dev_ptr, dev_ptr+bsize, dev_ptr_r);
		//DOES NOT WORK



		return (new_end-dev_ptr_r);



	}
	uint filterHitsWithCopyResize(optix::Buffer hitBuffer, optix::Buffer resultBuffer, uint bsize) {
		//Get device pointer to global buffer

		HitInfo* raw_ptr =static_cast<HitInfo*>( hitBuffer->getDevicePointer(0));
		thrust::device_ptr<HitInfo> dev_ptr=thrust::device_pointer_cast(raw_ptr);
		//		if (raw_ptr==NULL) {
		//			std::cout<<"Null buffer pointer"<<std::endl;
		//		}
		//std::cout<<"buffer pointer="<<raw_ptr<<"size="<<bsize<<std::endl;

		//Sort	
		thrust::sort(dev_ptr, dev_ptr+bsize);
		//Keep closest one
		thrust::device_ptr<HitInfo> new_end=thrust::unique(dev_ptr, dev_ptr+bsize);

		//Resize and copy on device
		resultBuffer->setSize( (new_end-dev_ptr));
		//Get device pointer to final result buffer
		HitInfo* raw_ptr_r =static_cast<HitInfo*>( resultBuffer->getDevicePointer(0));
		//std::cout<<"raw_ptr_r="<<raw_ptr_r<<std::endl;
		thrust::device_ptr<HitInfo> dev_ptr_r=thrust::device_pointer_cast(raw_ptr_r);
		//Copy
		thrust::device_ptr<HitInfo> copy_end=thrust::copy(dev_ptr, new_end, dev_ptr_r );
		return (copy_end-dev_ptr_r);

	}


}
