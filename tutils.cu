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

	thrust::host_vector<HitInfo> filterHitsMultiGPU( optix::Buffer hitBuffer,  optix::Buffer aIndex, const std::vector<int> &devices) {
		
		//Declare vector for devices
		uint enabledDevices=devices.size();
		std::vector<uint*> ai_ptrs(enabledDevices);
		std::vector<HitInfo*> raw_ptrs(enabledDevices);
		thrust::device_vector<thrust::device_ptr<HitInfo>> dev_ptrs(enabledDevices);
		thrust::device_vector<thrust::device_ptr<uint>> dev_ai(enabledDevices);
		std::vector<uint> ai_size(enabledDevices);

		uint totalHitsSize=0;
		for (uint i=0; i<enabledDevices; ++i) {
			//Get device pointer to atomic index buffer (per GPU)
			uint* ai_ptr=	static_cast<uint*>( aIndex->getDevicePointer(devices[i]));
			thrust::device_ptr<uint> dev_ptr_ai = thrust::device_pointer_cast(ai_ptr);
			//Copy memory from device...
			uint s= dev_ptr_ai[0];
			ai_ptrs[i]=ai_ptr;
			//Get device pointer to global buffer (per GPU)
			raw_ptrs[i]=static_cast<HitInfo*>( hitBuffer->getDevicePointer(devices[i]));
			dev_ptrs[i]=thrust::device_pointer_cast(raw_ptrs[i]);
			ai_size[i]=s;
			dev_ai[i]=dev_ptr_ai;
			
			totalHitsSize +=s;
		}	




		//Create vector on device to filter results
		thrust::device_vector<HitInfo> vt(totalHitsSize);
		
		//Fill the vector with the launch results
		uint aux=0;
		for (uint i=0; i<enabledDevices; ++i) {
			uint c_buf_size=ai_size[i];
			//std::cout<<"aux="<<aux<<"c_buf_size="<<c_buf_size<<std::endl;
			cudaMemcpy(thrust::raw_pointer_cast(vt.data())+aux, raw_ptrs[i],(c_buf_size)*sizeof(HitInfo),cudaMemcpyDeviceToDevice);
			aux += c_buf_size;

			//Init atomic index for next launch
			thrust::device_ptr<uint> dai=	dev_ai[i];
			dai[0]=0u;

		}
		
		//Sort the vector
		thrust::sort(vt.begin(), vt.end());
		//Filter the results: keep lower distance hit per sequence
		thrust::device_vector<HitInfo>::iterator new_end=thrust::unique(vt.begin(), vt.end());

		//Transfer to host
		thrust::host_vector<HitInfo> hi(vt.begin(),new_end);
		return hi;

	}


}
