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
#include <thrust/transform.h>
#include <thrust/functional.h>
#include <algorithm>
#include <cstdlib>
#include <ostream>
#include "sutil.h"
//#include <optix_world.h>
//#include <optixu/optixpp_namespace.h>
namespace opalthrustutils {
	PartialLaunchState::PartialLaunchState() :  partialLaunchGlobalIndex(0u)  {
		this->previousHits = new thrust::device_vector<HitInfo>();
	}
	PartialLaunchState::~PartialLaunchState()   {
		delete previousHits;
	}
	void PartialLaunchState::reset() {
		partialLaunchGlobalIndex=0u;
		previousHits->clear();
	}
	
	std::vector<HitInfo> PartialLaunchState::getHits() {
		std::vector<HitInfo> h(partialLaunchGlobalIndex);
		if (partialLaunchGlobalIndex>0) {
			thrust::copy_n(previousHits->begin(),partialLaunchGlobalIndex,h.begin());
		}
		return h;
	}


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

	//Just filter but do not copy for transfer. Used for partial launches
	uint filterHits(optix::Buffer hitBuffer,  uint bsize) {

		//Get device pointer to global buffer

		HitInfo* raw_ptr =static_cast<HitInfo*>( hitBuffer->getDevicePointer(0));
		thrust::device_ptr<HitInfo> dev_ptr=thrust::device_pointer_cast(raw_ptr);

		//Sort	
		thrust::sort(dev_ptr, dev_ptr+bsize);
		//Keep closest ones
		thrust::device_ptr<HitInfo> new_end=thrust::unique(dev_ptr, dev_ptr+bsize);
		//Return size to update the atomic index
		return (new_end-dev_ptr);
	}
	//Just copy for transfer. Used for partial launches, to transfer when partial launch is finished. Assume hits have been sorted and filtered before
	uint copyHitsToTransfer(optix::Buffer hitBuffer, optix::Buffer resultBuffer, uint bsize) {
		//Get device pointer to global buffer

		HitInfo* raw_ptr =static_cast<HitInfo*>( hitBuffer->getDevicePointer(0));
		thrust::device_ptr<HitInfo> dev_ptr=thrust::device_pointer_cast(raw_ptr);


		//Resize and copy on device
		resultBuffer->setSize( bsize);
		//Get device pointer to final result buffer
		HitInfo* raw_ptr_r =static_cast<HitInfo*>( resultBuffer->getDevicePointer(0));
		//cudaMemcpy(raw_ptr, raw_ptr_r,(bsize)*sizeof(HitInfo),cudaMemcpyDeviceToDevice);
		//std::cout<<"raw_ptr_r="<<raw_ptr_r<<std::endl;
		thrust::device_ptr<HitInfo> dev_ptr_r=thrust::device_pointer_cast(raw_ptr_r);
		//Copy
		thrust::copy_n(dev_ptr, bsize, dev_ptr_r );
		return bsize;
	}
thrust::host_vector<HitInfo> filterHitsAndTransferMultiGPU( optix::Buffer hitBuffer,  optix::Buffer aIndex, const std::vector<int> &devices) {

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
		//It is not copied twice, one for transfer and then when we return the result. Apparently copy elision works here.
		//Anyway, an alternative would be to pass a vector as reference, resize it and copy as below
		//hv.resize((new_end-vt.begin()));
		//thrust::copy(vt.begin(),new_end,hv.begin());
		

	}

	//For partial launches
	//Filter the hits but do not transfer, they keep stored in a device vector provided by the host
	//uint filterHitsMultiGPU( optix::Buffer hitBuffer,  optix::Buffer aIndex, const std::vector<int> &devices, thrust::device_vector<HitInfo> &vt, uint previousSize) {
	uint filterHitsMultiGPU( optix::Buffer hitBuffer,  optix::Buffer aIndex, const std::vector<int> &devices, PartialLaunchState* state, uint maxGlobalBufferSize) {
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
			if (s>=maxGlobalBufferSize) {
				throw opalthrustutils::Exception("filterHitsMultiGPU(): globalHitBuffer overflow");
			
			}	
			ai_size[i]=s;
			dev_ai[i]=dev_ptr_ai;

			totalHitsSize +=s;
		}	


		thrust::device_vector<HitInfo>* vt =state->getDeviceHits();
		uint previousSize=state->getIndex();
		

		//Merge with the previously filtered hits
		
		//Manage memory, otherwise we run out of memory soon for large launches
		float totalMem=sizeof(HitInfo)*totalHitsSize/(1024.f*1024.f); //In MiB

		//If 'low' memory size keep it simple
		if (totalMem<1000.0f) {	
		//	std::cout<<"Simple Memory: previousSize="<<previousSize<<"totalHitsSize="<<totalHitsSize<<"total memory for hits="<<totalMem<<" MiB"<<std::endl;


			vt->resize(totalHitsSize+previousSize);
			//std::cout<<"resized"<<std::endl;
			//Fill the vector with the launch results
			uint aux=previousSize;
			for (uint i=0; i<enabledDevices; ++i) {
				uint c_buf_size=ai_size[i];
			//	std::cout<<"aux="<<aux<<"c_buf_size="<<c_buf_size<<"data="<<&vt<<std::endl;
				cudaMemcpy(thrust::raw_pointer_cast(vt->data())+aux, raw_ptrs[i],(c_buf_size)*sizeof(HitInfo),cudaMemcpyDeviceToDevice);
				aux += c_buf_size;

			//	std::cout<<i<<" copied "<<c_buf_size<<std::endl;
				//Init atomic index for next launch
				thrust::device_ptr<uint> dai=	dev_ai[i];
				dai[0]=0u;

			}

			//std::cout<<"vt.size="<<vt->size()<<std::endl;
			//Sort the vector
			thrust::sort(vt->begin(), vt->end());
			
			//std::cout<<"sorted "<<std::endl;
			
			//Filter the results: keep lower distance hit per sequence
			thrust::device_vector<HitInfo>::iterator new_end=thrust::unique(vt->begin(), vt->end());
			//std::cout<<"filtered "<<std::endl;
			
			//The device vector has all the hits, but only those up to new_end are properly sorted. They are kept for the next launch to be sorted and filtered again with the new ones
			//std::cout<<"vt.size="<<vt->size()<<"hits="<<(new_end-vt->begin())<<std::endl;
			return (new_end-vt->begin());
		} else {
			//std::cout<<"Large Memory: previousSize="<<previousSize<<"memory="<<(previousSize*sizeof(HitInfo)/(1024.f*1024.f))<<" MiB; totalHitsSize="<<totalHitsSize<<"total memory for hits="<<(sizeof(HitInfo)*totalHitsSize/(1024.f*1024.f))<<" MiB"<<std::endl;
			
			//Large number of hits, process in chunks
			//Create chunks per device
			//Have to find a balance between available memory and potential hits you may have. You can use a higher fraction in setGlobalHitInfoBuffer (0.5-0.7) and a lower value for chunkElements here if 
			//many hits are potential (enought to overflow the globalHitInfoBuffer) or, conversely, a lower fraction there and a higher value here, which means better performance here (fewer iterations)
			//uint chunkElements=33554432u; //Number of elements for 1GiB of HitInfo assuming sizeof(HitInfo)=32
			uint chunkElements=16777216u; //Number of elements for 0.5GiB of HitInfo assuming sizeof(HitInfo)=32

			for (uint i=0; i<enabledDevices; ++i) {

				uint chunks=ai_size[i]/chunkElements;
				uint remainder=ai_size[i]%chunkElements;
				//std::cout<<"Current buffer for device "<<i<<"="<<ai_size[i]<<"; memory="<<(sizeof(HitInfo)*ai_size[i]/(1024.f*1024.f))<<" MiB; chunks="<<chunks<<"remainder="<<remainder<<"previousSize="<<previousSize<<std::endl;
				for (uint j=0; j<chunks;++j) {
					vt->resize(previousSize+chunkElements);
					//std::cout<<"Chunk filter device "<<i<<":  resized. chunk="<<j<<std::endl;
					//Fill the vector with the launch results
					//std::cout<<"aux="<<aux<<"c_buf_size="<<c_buf_size<<"data="<<&vt<<std::endl;
					
					cudaMemcpy(thrust::raw_pointer_cast(vt->data())+previousSize, raw_ptrs[i],(chunkElements)*sizeof(HitInfo),cudaMemcpyDeviceToDevice);
					
					//std::cout<<"Chunk filter device "<<i<<":  copied. chunk="<<j<<std::endl;
					//Sort the vector
					
					thrust::sort(vt->begin(), vt->end());
					
					//std::cout<<"Chunk filter device "<<i<<":  sorted. chunk="<<j<<std::endl;
					//Filter the results: keep lower distance hit per sequence
					thrust::device_vector<HitInfo>::iterator new_end=thrust::unique(vt->begin(), vt->end());
					
					//std::cout<<"Chunk filter device "<<i<<":  filtered. chunk="<<j<<std::endl;
					//Keep the already filtered ones in our vector
					previousSize =(new_end-vt->begin()); 
					
					//std::cout<<"Chunk filter device "<<i<<":  filtered. new previousSizes="<<previousSize<<std::endl;
				}
				//Finish the remainding ones
					vt->resize(previousSize+remainder);
					//std::cout<<"Chunk filter device "<<i<<":  resized. chunks="<<chunks<<"remainder="<<remainder<<std::endl;
					
					
					//Fill the vector with the launch results
					//std::cout<<"aux="<<aux<<"c_buf_size="<<c_buf_size<<"data="<<&vt<<std::endl;
					
					cudaMemcpy(thrust::raw_pointer_cast(vt->data())+previousSize, raw_ptrs[i],(remainder)*sizeof(HitInfo),cudaMemcpyDeviceToDevice);
					
					
					//std::cout<<"Chunk filter device "<<i<<":  copied. chunks="<<chunks<<"remainder="<<remainder<<std::endl;
					//Sort the vector
					thrust::sort(vt->begin(), vt->end());
					
					//std::cout<<"Chunk filter device "<<i<<":  sorted. chunks="<<chunks<<"remainder="<<remainder<<std::endl;
					//Filter the results: keep lower distance hit per sequence
					thrust::device_vector<HitInfo>::iterator new_end=thrust::unique(vt->begin(), vt->end());
					
					
					//std::cout<<"Chunk filter device "<<i<<":  filtered. chunks="<<chunks<<"remainder="<<remainder<<std::endl;
					//Keep the already filtered ones in our vector
					previousSize =(new_end-vt->begin()); 
					
					//std::cout<<"Chunk filter device "<<i<<":  filtered. new previousSizes="<<previousSize<<"remainder="<<remainder<<std::endl;
					//Init atomic index for next launch
					thrust::device_ptr<uint> dai=	dev_ai[i];
					dai[0]=0u;
			}
			//std::cout<<"vt.size="<<vt->size()<<"hits="<<previousSize<<std::endl;
			return previousSize;
		}
	}

}
