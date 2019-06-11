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

	thrust::host_vector<HitInfo> filterHitsMultiGPU( optix::Buffer hitBuffer,  optix::Buffer aIndex, const std::vector<int> &devices) {
		
		uint enabledDevices=devices.size();
		std::vector<uint*> ai_ptrs(enabledDevices);
		std::vector<HitInfo*> raw_ptrs(enabledDevices);
		thrust::device_vector<thrust::device_ptr<HitInfo>> dev_ptrs(enabledDevices);
		thrust::device_vector<thrust::device_ptr<uint>> dev_ai(enabledDevices);
		std::vector<uint> ai_size(enabledDevices);

		uint totalHitsSize=0;
		//Get device pointer to global buffer
		for (uint i=0; i<enabledDevices; ++i) {
			uint* ai_ptr=	static_cast<uint*>( aIndex->getDevicePointer(devices[i]));
			thrust::device_ptr<uint> dev_ptr_ai = thrust::device_pointer_cast(ai_ptr);
			//Copy memory from device...
			uint s= dev_ptr_ai[0];
			ai_ptrs[i]=ai_ptr;
			raw_ptrs[i]=static_cast<HitInfo*>( hitBuffer->getDevicePointer(devices[i]));
			dev_ptrs[i]=thrust::device_pointer_cast(raw_ptrs[i]);
			ai_size[i]=s;
			dev_ai[i]=dev_ptr_ai;
			
			totalHitsSize +=s;
		}	




//		uint* ai_ptr =static_cast<uint*>( aIndex->getDevicePointer(0));
//		uint* ai_ptr2 =static_cast<uint*>( aIndex->getDevicePointer(1));
//		HitInfo* raw_ptr =static_cast<HitInfo*>( hitBuffer->getDevicePointer(0));
//		HitInfo* raw_ptr2 =static_cast<HitInfo*>( hitBuffer->getDevicePointer(1));
//		
//		thrust::device_ptr<HitInfo> dev_ptr=thrust::device_pointer_cast(raw_ptr);
//		thrust::device_ptr<HitInfo> dev_ptr2=thrust::device_pointer_cast(raw_ptr2);
//		thrust::device_ptr<uint> dev_ai=thrust::device_pointer_cast(ai_ptr);
//		thrust::device_ptr<uint> dev_ai2=thrust::device_pointer_cast(ai_ptr2);
		//		if (raw_ptr==NULL) {
		//			std::cout<<"Null buffer pointer"<<std::endl;
		//		}
		//std::cout<<"buffer pointer="<<raw_ptr<<std::endl;	
		//uint as=dev_ai[0];
		//uint as2=dev_ai2[0];

		
		//std::cout<<"as="<<as<<"as2="<<as2<<std::endl;
		//thrust::device_vector<HitInfo> vt(as+as2);
		thrust::device_vector<HitInfo> vt(totalHitsSize);
		//std::cout<<"as="<<as<<"as2="<<as2<<std::endl;
		//thrust::device_vector<HitInfo> vt2(as2);
		//std::cout<<"as="<<as<<"as2="<<as2<<std::endl;
		uint aux=0;
		for (uint i=0; i<enabledDevices; ++i) {
			uint c_buf_size=ai_size[i];
			//std::cout<<"aux="<<aux<<"c_buf_size="<<c_buf_size<<std::endl;
			cudaMemcpy(thrust::raw_pointer_cast(vt.data())+aux, raw_ptrs[i],(c_buf_size)*sizeof(HitInfo),cudaMemcpyDeviceToDevice);
			aux += c_buf_size;

			//Init for next launch
			thrust::device_ptr<uint> dai=	dev_ai[i];
			dai[0]=0u;

		}
		//cudaMemcpy(thrust::raw_pointer_cast(vt.data()), raw_ptr,as*sizeof(HitInfo),cudaMemcpyDeviceToDevice);
		//cudaMemcpy(thrust::raw_pointer_cast(vt.data())+as, raw_ptr2,as2*sizeof(HitInfo),cudaMemcpyDeviceToDevice);
		//std::cout<<"as="<<as<<"vt.size="<<vt.size()<<std::endl;
		//thrust::copy_n(dev_ptr,as,vt.begin());
		//thrust::copy_n(dev_ptr2,as2,vt2.begin());
		thrust::sort(vt.begin(), vt.end());
		//std::cout<<"sort="<<as<<"as2="<<as2<<std::endl;
	//	thrust::sort(vt2.begin(), vt2.end());
		//std::cout<<"sort2="<<as<<"as2="<<as2<<std::endl;
	//	thrust::device_vector<HitInfo> merged;
	//	thrust::merge(vt.begin(), vt.end(),vt2.begin(), vt2.end(),merged.begin());
		//Sort	
		//thrust::sort(dev_ptr, dev_ptr+bsize);
		//thrust::sort(dev_ptr2, dev_ptr+bsize);
		//Keep closest one
		//thrust::device_ptr<HitInfo> new_end=thrust::unique(dev_ptr, dev_ptr+bsize);
		//thrust::device_vector<HitInfo>::iterator new_end=thrust::unique(merged.begin(), merged.end());
		thrust::device_vector<HitInfo>::iterator new_end=thrust::unique(vt.begin(), vt.end());

		//std::cout<<"unique="<<as<<"as2="<<as2<<std::endl;
		////Resize and copy on device
		//resultBuffer->setSize( (new_end-vt.begin()));
		////Get device pointer to final result buffer
		//HitInfo* raw_ptr_r =static_cast<HitInfo*>( resultBuffer->getDevicePointer(0));
		////std::cout<<"raw_ptr_r="<<raw_ptr_r<<std::endl;
		//thrust::device_ptr<HitInfo> dev_ptr_r=thrust::device_pointer_cast(raw_ptr_r);
		////Copy
		//thrust::device_ptr<HitInfo> copy_end=thrust::copy(vt.begin(), new_end, dev_ptr_r );
		//std::cout<<"copy to result"<<std::endl;
		thrust::host_vector<HitInfo> hi(vt.begin(),new_end);
		//for (int i=0; i<hi.size(); i++) {
		//	std::cout<<hi[i].E<<std::endl;
		//}
		//return (copy_end-dev_ptr_r);
		//return vt.size();
		//dev_ai[0]=0u;
		//dev_ai2[0]=0u;
		return hi;

	}


}
