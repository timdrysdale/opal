/***************************************************************/
//
//Copyright (c) 2019 Esteban Egea-Lopez http://girtel.upct.es/~eegea
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
#include <thrust/partition.h>
#include <thrust/transform_reduce.h>
#include <algorithm>
#include <cstdlib>
#include <ostream>
#include "sutil.h"
#include <thrust/iterator/zip_iterator.h>

#define CUDA_CALL(x) do { if((x)!=cudaSuccess) { \
	printf("Error at %s:%d\n",__FILE__,__LINE__);\
}} while(0)
#define CURAND_CALL(x) do { curandStatus_t st=x; if(st!=CURAND_STATUS_SUCCESS) { \
	printf("Error %d at %s:%d\n",st,__FILE__,__LINE__);\
}} while(0)
//#include <optix_world.h>
//#include <optixu/optixpp_namespace.h>
namespace opalthrustutils {

	thrust::host_vector<LogTraceHitInfo> getLogTraceOrderer( optix::Buffer traceBuffer,  optix::Buffer aIndex, const std::vector<int> &devices, uint maxGlobalBufferSize) {

		uint enabledDevices=devices.size();

		BufferPointers<LogTraceHitInfo> bp(enabledDevices);
		bp.fillMembers(traceBuffer,aIndex,devices,maxGlobalBufferSize); 
		long totalMemory=bp.getTotalHitsSize()*sizeof(HitInfo);
		float GiB=1048576.0f*1024.0f;
		std::cout<<"Total memory="<<(totalMemory/GiB)<<std::endl;
		if (!bp.checkMemory(traceBuffer,totalMemory)) {
			throw opalthrustutils::Exception("getLogTraceOrdered(): Not enough memory to sort. Try reducing the number of receivers or increasing the angular sampling of rays");

		} else {


			//Create vector on device to filter results
			thrust::device_vector<LogTraceHitInfo> vt(bp.getTotalHitsSize());

			//Fill the vector with the launch results
			uint aux=0;
			for (uint i=0; i<enabledDevices; ++i) {
				uint c_buf_size=bp.getAtomicIndex(i);

				//	std::cout<<"aux="<<aux<<"c_buf_size="<<c_buf_size<<"data="<<&vt<<std::endl;
				//cudaMemcpy(thrust::raw_pointer_cast(vt->data())+aux, raw_ptrs[i],(c_buf_size)*sizeof(HitInfo),cudaMemcpyDeviceToDevice);
				cudaMemcpy(thrust::raw_pointer_cast(vt.data())+aux, bp.getRawPointerToHitBuffer(i),(c_buf_size)*sizeof(LogTraceHitInfo),cudaMemcpyDeviceToDevice);
				aux += c_buf_size;

				//	std::cout<<i<<" copied "<<c_buf_size<<std::endl;
				//Init atomic index for next launch
				//thrust::device_ptr<uint> dai=	dev_ai[i];
				thrust::device_ptr<uint> dai=	bp.getPointerToAtomicIndexBuffer(i);
				dai[0]=0u;
				//uint c_buf_size=ai_size[i];
				////std::cout<<"aux="<<aux<<"c_buf_size="<<c_buf_size<<std::endl;
				//cudaMemcpy(thrust::raw_pointer_cast(vt.data())+aux, raw_ptrs[i],(c_buf_size)*sizeof(HitInfo),cudaMemcpyDeviceToDevice);
				//aux += c_buf_size;

				////Init atomic index for next launch
				//thrust::device_ptr<uint> dai=	dev_ai[i];
				//dai[0]=0u;

			}

			//Sort the vector
			thrust::sort(vt.begin(), vt.end());
			//Transfer to host
			thrust::host_vector<LogTraceHitInfo> hi(vt.begin(),vt.end());
			return hi;
		}
	}

	//Single GPU utility functions. Not used
	//Just filter but do not copy for transfer.
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


	//Single and multi-GPU functions

	thrust::host_vector<HitInfo> filterHitsAndTransfer( optix::Buffer hitBuffer,  optix::Buffer aIndex, const std::vector<int> &devices, uint maxGlobalBufferSize) {



		uint enabledDevices=devices.size();

		BufferPointers<HitInfo> bp(enabledDevices);
		bp.fillMembers(hitBuffer,aIndex,devices,maxGlobalBufferSize); 


		//Create vector on device to filter results
		thrust::device_vector<HitInfo> vt(bp.getTotalHitsSize());

		//Fill the vector with the launch results
		uint aux=0;
		for (uint i=0; i<enabledDevices; ++i) {
			uint c_buf_size=bp.getAtomicIndex(i);
			//uint c_buf_size=ai_size[i];
			std::cout<<"aux="<<aux<<"c_buf_size="<<c_buf_size<<std::endl;
			//cudaMemcpy(thrust::raw_pointer_cast(vt.data())+aux, raw_ptrs[i],(c_buf_size)*sizeof(HitInfo),cudaMemcpyDeviceToDevice);
			cudaMemcpy(thrust::raw_pointer_cast(vt.data())+aux, bp.getRawPointerToHitBuffer(i),(c_buf_size)*sizeof(HitInfo),cudaMemcpyDeviceToDevice);
			aux += c_buf_size;

			//Init atomic index for next launch
			thrust::device_ptr<uint> dai=	bp.getPointerToAtomicIndexBuffer(i);
			//thrust::device_ptr<uint> dai=	dev_ai[i];
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
	thrust::host_vector<RDNHit> getReceivedFieldMultiGPU( optix::Buffer hitBuffer,const std::vector<int> &devices,  uint nrx) {
		thrust::host_vector<RDNHit> h(nrx);
	//	for (uint i=0; i<nrx; ++i) {
	//		std::cout<<"h="<<h[i]<<std::endl;
	//	}
		uint enabledDevices=devices.size();
		for (uint i=0; i<enabledDevices; ++i) {
			cudaSetDevice(devices[i]);
			RDNHit* pr=static_cast<RDNHit*>(hitBuffer->getDevicePointer(devices[i]));
			thrust::device_ptr<RDNHit> p=thrust::device_pointer_cast(pr);
			for (uint j=0; j<nrx; ++j) {
				RDNHit myHit=p[j];
				h[j].EEx +=myHit.EEx;
				h[j].EyEz +=myHit.EyEz;
				RDNHit empty={make_float4(0.0f,0.0f,0.0f,0.0f),make_float4(0.0f,0.0f,0.0f,0.0f)}; 	
				p[j]=empty;
				
	//	#ifdef OPAL_EXTENDED_HITINFO
	//			RDNHit myHit=p[j];
	//			h[j].Ex +=myHit.Ex;
	//			h[j].Ey +=myHit.Ey;
	//			h[j].Ez +=myHit.Ez;
	//			RDNHit empty={make_float2(0.0f,0.0f),make_float2(0.0f,0.0f),make_float2(0.0f,0.0f)}; 	
	//			p[j]=empty;

	//	#else
	//			h[j] +=p[j];
	//			//Init
	//			p[j]=make_float2(0.0f,0.0f);
	//	#endif
				
 			}
		}
		//for (uint i=0; i<nrx; ++i) {
		//	std::cout<<"ho="<<h[i]<<std::endl;
		//}
		return h;
		
	}
	//#ifndef OPAL_EXTENDED_HITINFO
	optix::float2 getReducedFieldMultiGPU( optix::Buffer hitBuffer,  optix::Buffer aIndex, const std::vector<int> &devices, uint maxGlobalBufferSize) {
		uint enabledDevices=devices.size();

		BufferPointers<HitInfo> bp(enabledDevices);
		bp.fillMembers(hitBuffer,aIndex,devices,maxGlobalBufferSize); 
		long totalMemory=bp.getTotalHitsSize()*sizeof(HitInfo);
		float GiB=1048576.0f*1024.0f;
		std::cout<<"Total memory="<<(totalMemory/GiB)<<std::endl;
               	GetRealE unary_op_r;
               	GetImE unary_op_i;
		thrust::plus<float> binary_op;
		float init=0;
                float2 E=make_float2(0.0f,0.0f); 
		for (uint i=0; i<enabledDevices; ++i) {
			cudaSetDevice(i);
			uint c_buf_size=bp.getAtomicIndex(i);
			thrust::device_ptr<HitInfo> dp=bp.getPointerToHitBuffer(i);
                        float Er=thrust::transform_reduce(dp, dp+c_buf_size,unary_op_r,init,binary_op);
                        float Ey=thrust::transform_reduce(dp, dp+c_buf_size,unary_op_i,init,binary_op);
			E.x += Er;
			E.y +=Ey;
			thrust::device_ptr<uint> dai=	bp.getPointerToAtomicIndexBuffer(i);
			dai[0]=0u;
		}
		return E;
	}
	//#endif
	thrust::host_vector<HitInfo> copyHitsToHostMultiGPU( optix::Buffer hitBuffer,  optix::Buffer aIndex, const std::vector<int> &devices, uint maxGlobalBufferSize) {

		uint enabledDevices=devices.size();

		BufferPointers<HitInfo> bp(enabledDevices);
		bp.fillMembers(hitBuffer,aIndex,devices,maxGlobalBufferSize); 
		long totalMemory=bp.getTotalHitsSize()*sizeof(HitInfo);
		float GiB=1048576.0f*1024.0f;
		std::cout<<"Total memory="<<(totalMemory/GiB)<<std::endl;


		//Create host vector to copy 
		thrust::host_vector<HitInfo> hv(bp.getTotalHitsSize());

			//Fill the vector with the launch results
			uint aux=0;
			for (uint i=0; i<enabledDevices; ++i) {
				uint c_buf_size=bp.getAtomicIndex(i);

				//	std::cout<<"aux="<<aux<<"c_buf_size="<<c_buf_size<<"data="<<&vt<<std::endl;
				//cudaMemcpy(thrust::raw_pointer_cast(vt->data())+aux, raw_ptrs[i],(c_buf_size)*sizeof(HitInfo),cudaMemcpyDeviceToDevice);
				cudaMemcpy(thrust::raw_pointer_cast(hv.data())+aux, bp.getRawPointerToHitBuffer(i),(c_buf_size)*sizeof(HitInfo),cudaMemcpyDeviceToHost);
				aux += c_buf_size;

				//	std::cout<<i<<" copied "<<c_buf_size<<std::endl;
				//Init atomic index for next launch
				//thrust::device_ptr<uint> dai=	dev_ai[i];
				thrust::device_ptr<uint> dai=	bp.getPointerToAtomicIndexBuffer(i);
				dai[0]=0u;
				//uint c_buf_size=ai_size[i];
				////std::cout<<"aux="<<aux<<"c_buf_size="<<c_buf_size<<std::endl;
				//cudaMemcpy(thrust::raw_pointer_cast(vt.data())+aux, raw_ptrs[i],(c_buf_size)*sizeof(HitInfo),cudaMemcpyDeviceToDevice);
				//aux += c_buf_size;

				////Init atomic index for next launch
				//thrust::device_ptr<uint> dai=	dev_ai[i];
				//dai[0]=0u;

			}

			//Transfer to host
			return hv;
		
	}
	thrust::host_vector<HitInfo> getAllHitsOrderedMultiGPU( optix::Buffer hitBuffer,  optix::Buffer aIndex, const std::vector<int> &devices, uint maxGlobalBufferSize, bool sort) {
		//		//Declare vector for devices
		//		uint enabledDevices=devices.size();
		//		std::vector<uint*> ai_ptrs(enabledDevices);
		//		std::vector<HitInfo*> raw_ptrs(enabledDevices);
		//		thrust::device_vector<thrust::device_ptr<HitInfo>> dev_ptrs(enabledDevices);
		//		thrust::device_vector<thrust::device_ptr<uint>> dev_ai(enabledDevices);
		//		std::vector<uint> ai_size(enabledDevices);
		//
		//		uint totalHitsSize=0;
		//		for (uint i=0; i<enabledDevices; ++i) {
		//			//Get device pointer to atomic index buffer (per GPU)
		//			uint* ai_ptr=	static_cast<uint*>( aIndex->getDevicePointer(devices[i]));
		//			thrust::device_ptr<uint> dev_ptr_ai = thrust::device_pointer_cast(ai_ptr);
		//			//Copy memory from device...
		//			uint s= dev_ptr_ai[0];
		//			ai_ptrs[i]=ai_ptr;
		//			//Get device pointer to global buffer (per GPU)
		//			raw_ptrs[i]=static_cast<HitInfo*>( hitBuffer->getDevicePointer(devices[i]));
		//			dev_ptrs[i]=thrust::device_pointer_cast(raw_ptrs[i]);
		//			ai_size[i]=s;
		//			dev_ai[i]=dev_ptr_ai;
		//
		//			totalHitsSize +=s;
		//		}	
		//

		uint enabledDevices=devices.size();

		BufferPointers<HitInfo> bp(enabledDevices);
		bp.fillMembers(hitBuffer,aIndex,devices,maxGlobalBufferSize); 
		long totalMemory=bp.getTotalHitsSize()*sizeof(HitInfo);
		float GiB=1048576.0f*1024.0f;
		std::cout<<"Total memory="<<(totalMemory/GiB)<<std::endl;
		if (sort && !bp.checkMemory(hitBuffer,totalMemory)) {
			throw opalthrustutils::Exception("orderOnGlobalBufferAndTransfer(): Not enough memory to sort. Try reducing the number of receivers or increasing the angular sampling of rays");

		} else {


			//Create vector on device to filter results
			thrust::device_vector<HitInfo> vt(bp.getTotalHitsSize());

			//Fill the vector with the launch results
			uint aux=0;
			for (uint i=0; i<enabledDevices; ++i) {
				uint c_buf_size=bp.getAtomicIndex(i);

				//	std::cout<<"aux="<<aux<<"c_buf_size="<<c_buf_size<<"data="<<&vt<<std::endl;
				//cudaMemcpy(thrust::raw_pointer_cast(vt->data())+aux, raw_ptrs[i],(c_buf_size)*sizeof(HitInfo),cudaMemcpyDeviceToDevice);
				cudaMemcpy(thrust::raw_pointer_cast(vt.data())+aux, bp.getRawPointerToHitBuffer(i),(c_buf_size)*sizeof(HitInfo),cudaMemcpyDeviceToDevice);
				aux += c_buf_size;

				//	std::cout<<i<<" copied "<<c_buf_size<<std::endl;
				//Init atomic index for next launch
				//thrust::device_ptr<uint> dai=	dev_ai[i];
				thrust::device_ptr<uint> dai=	bp.getPointerToAtomicIndexBuffer(i);
				dai[0]=0u;
				//uint c_buf_size=ai_size[i];
				////std::cout<<"aux="<<aux<<"c_buf_size="<<c_buf_size<<std::endl;
				//cudaMemcpy(thrust::raw_pointer_cast(vt.data())+aux, raw_ptrs[i],(c_buf_size)*sizeof(HitInfo),cudaMemcpyDeviceToDevice);
				//aux += c_buf_size;

				////Init atomic index for next launch
				//thrust::device_ptr<uint> dai=	dev_ai[i];
				//dai[0]=0u;

			}

			if (sort) {
				//Sort the vector
				thrust::sort(vt.begin(), vt.end());
			}
			//Transfer to host
			thrust::host_vector<HitInfo> hi(vt.begin(),vt.end());
			return hi;
		}
	}

	//For partial launches
	//Filter the hits but do not transfer, they keep stored in a device vector provided by the host
	//uint filterHitsMultiGPU( optix::Buffer hitBuffer,  optix::Buffer aIndex, const std::vector<int> &devices, thrust::device_vector<HitInfo> &vt, uint previousSize) {
	uint filterHitsMultiGPU( optix::Buffer hitBuffer,  optix::Buffer aIndex, const std::vector<int> &devices, PartialLaunchState<HitInfo>* state, uint maxGlobalBufferSize) {
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
			if (s>0) {
				std::cout<<"filterHitsMultiGPU(): s="<<s<<std::endl;
			}
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
			thrust::sort(thrust::device,vt->begin(), vt->end());

			//std::cout<<"sorted "<<std::endl;

			//Filter the results: keep lower distance hit per sequence
			thrust::device_vector<HitInfo>::iterator new_end=thrust::unique(thrust::device,vt->begin(), vt->end());
			//std::cout<<"filtered "<<std::endl;

			//The device vector has all the hits, but only those up to new_end are properly sorted. They are kept for the next launch to be sorted and filtered again with the new ones
			//std::cout<<"vt.size="<<vt->size()<<"hits="<<(new_end-vt->begin())<<std::endl;

			//Should we resize the vt vector to free memory...?
			return (new_end-vt->begin());
		} else {
			std::cout<<"Large Memory: previousSize="<<previousSize<<"memory="<<(previousSize*sizeof(HitInfo)/(1024.f*1024.f))<<" MiB; totalHitsSize="<<totalHitsSize<<"total memory for hits="<<(sizeof(HitInfo)*totalHitsSize/(1024.f*1024.f))<<" MiB"<<std::endl;

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

					thrust::sort(thrust::device,vt->begin(), vt->end());

					//std::cout<<"Chunk filter device "<<i<<":  sorted. chunk="<<j<<std::endl;
					//Filter the results: keep lower distance hit per sequence
					thrust::device_vector<HitInfo>::iterator new_end=thrust::unique(thrust::device,vt->begin(), vt->end());

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
				thrust::sort(thrust::device,vt->begin(), vt->end());

				//std::cout<<"Chunk filter device "<<i<<":  sorted. chunks="<<chunks<<"remainder="<<remainder<<std::endl;
				//Filter the results: keep lower distance hit per sequence
				thrust::device_vector<HitInfo>::iterator new_end=thrust::unique(thrust::device,vt->begin(), vt->end());


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

	//TODO: we do not really know yet how to to this in parallel, so for the moment just order (if we can) and send to host
	uint getAllHitsOrderedMultiGPU( optix::Buffer hitBuffer,  optix::Buffer aIndex, const std::vector<int> &devices, PartialLaunchState<HitInfo>* state, uint maxGlobalBufferSize) {

		uint enabledDevices=devices.size();

		BufferPointers<HitInfo> bp(enabledDevices);
		bp.fillMembers(hitBuffer,aIndex,devices,maxGlobalBufferSize); 
		//uint totalHitsSize=0;

		//Memory management and checks
		float GiB=1048576.0f*1024.0f;
		std::vector<RTsize> memoryInDevice(enabledDevices);
		for (uint i=0; i<enabledDevices; ++i) {
			memoryInDevice[i]=hitBuffer->getContext()->getAvailableDeviceMemory(i);
		}
		thrust::device_vector<HitInfo>* vt =state->getDeviceHits();
		uint previousSize=state->getIndex();

		long totalMemory=(bp.getTotalHitsSize()+previousSize)*sizeof(HitInfo);

		//Check if we have memory enough or throw exception
		bool enoughMemory=false;
		for (int i=0; i<enabledDevices; ++i) {
			if (totalMemory<memoryInDevice[i]){
				std::cout<<"totalMemory="<<(totalMemory/GiB)<<" memoryInDevice["<<i<<"]="<<(memoryInDevice[i]/GiB)<<std::endl;
				enoughMemory=true;
				break;
			}	
		}
		if (!enoughMemory) {
			//TODO: fallback to host, transfer and sort in host
			throw opalthrustutils::Exception("getAllHitsOrderedMultiGPU(): Not enough memory to sort. Try reducing the number of receivers or increasing the angular sampling of rays");
		} else {
			vt->resize(bp.getTotalHitsSize()+previousSize);
			//std::cout<<" resize mfc0="<<((hitBuffer->getContext()->getAvailableDeviceMemory(0))/GiB)<<"mfc1="<<((hitBuffer->getContext()->getAvailableDeviceMemory(1))/GiB)<<std::endl;
			//Fill the vector with the launch results
			uint aux=previousSize;
			for (uint i=0; i<enabledDevices; ++i) {
				//uint c_buf_size=ai_size[i];
				uint c_buf_size=bp.getAtomicIndex(i);

				//	std::cout<<"aux="<<aux<<"c_buf_size="<<c_buf_size<<"data="<<&vt<<std::endl;
				//cudaMemcpy(thrust::raw_pointer_cast(vt->data())+aux, raw_ptrs[i],(c_buf_size)*sizeof(HitInfo),cudaMemcpyDeviceToDevice);
				cudaMemcpy(thrust::raw_pointer_cast(vt->data())+aux, bp.getRawPointerToHitBuffer(i),(c_buf_size)*sizeof(HitInfo),cudaMemcpyDeviceToDevice);
				aux += c_buf_size;

				//	std::cout<<i<<" copied "<<c_buf_size<<std::endl;
				//Init atomic index for next launch
				//thrust::device_ptr<uint> dai=	dev_ai[i];
				thrust::device_ptr<uint> dai=	bp.getPointerToAtomicIndexBuffer(i);
				dai[0]=0u;

			}


			//	std::cout<<"merge mfc0="<<((hitBuffer->getContext()->getAvailableDeviceMemory(0))/GiB)<<"mfc1="<<((hitBuffer->getContext()->getAvailableDeviceMemory(1))/GiB)<<std::endl;
			//TODO: This merge only works with 2 devices, we should extend this for more devices
			//	if (enabledDevices==1) {
			//		//Just copy
			//		cudaMemcpy(thrust::raw_pointer_cast(vt->data()), raw_ptrs[0],(ai_size[0])*sizeof(HitInfo),cudaMemcpyDeviceToDevice);
			//	} else if (enabledDevices==2) {
			//		thrust::device_ptr<HitInfo> dp0=(thrust::device_ptr<HitInfo>)dev_ptrs[0];
			//		thrust::device_ptr<HitInfo> dp1=(thrust::device_ptr<HitInfo>)dev_ptrs[1];

			//		thrust::merge(dp0,dp0+ai_size[0],dp1,dp1+ai_size[1],vt->begin());
			//	} else {
			//			throw opalthrustutils::Exception("getAllHitsOrderedMultiGPU: merge not implemented for more than 2 devices");
			//	}

			thrust::sort(vt->begin(), vt->end());
			return (vt->end()-vt->begin());
		}
	}

	uint getMixedHitsMultiGPU( optix::Buffer hitBuffer,  optix::Buffer aIndex, const std::vector<int> &devices, PartialLaunchState<HitInfo>* state, uint maxGlobalBufferSize) {

		uint enabledDevices=devices.size();

		BufferPointers<HitInfo> bp(enabledDevices);
		bp.fillMembers(hitBuffer,aIndex,devices,maxGlobalBufferSize); 
		thrust::device_vector<HitInfo>* vt =state->getDeviceHits();
		uint previousSize=state->getIndex();

		long totalMemory=(bp.getTotalHitsSize()+previousSize)*sizeof(HitInfo);

		if (!bp.checkMemory(hitBuffer,totalMemory)) {
			//TODO: fallback to host, transfer and sort in host
			throw opalthrustutils::Exception("getMixedHitsMultiGPU(): Not enough memory to sort. Try reducing the number of receivers or increasing the angular sampling of rays");
		} else {
			//Fill the vector with the launch results
			uint aux=previousSize;

			//First, partition between hits that have touched at least one curved mesh and those which not
			//Do the partition on the global buffer and keep the hits on curved on global buffer
			std::vector<uint> indexPartition(enabledDevices);
			std::vector<uint> numberNonCurvedHits(enabledDevices);

			for (uint i=0; i<enabledDevices; ++i) {
				cudaSetDevice(i);
				//uint c_buf_size=ai_size[i];
				uint c_buf_size=bp.getAtomicIndex(i);
				//Partition
				thrust::device_vector<HitInfo>::iterator  pit=thrust::partition(thrust::device,bp.getPointerToHitBuffer(i),bp.getPointerToHitBuffer(i)+c_buf_size,hit_on_curved());	
				HitInfo* beg=bp.getRawPointerToHitBuffer(i);
				//Mmmm, do we really have to do as below... see thrust/system/cuda/detail/reduce.h:963
				HitInfo* pitr=thrust::raw_pointer_cast(&(*pit));
				indexPartition[i]=pitr-beg;
				numberNonCurvedHits[i]=(bp.getRawPointerToHitBuffer(i)+c_buf_size)-pitr;
				//std::cout<<"noncurved["<<i<<"]="<<numberNonCurvedHits[i]<<";indexPartition["<<i<<"]="<<indexPartition[i]<<"c_buf_size="<<c_buf_size<<std::endl;
				thrust::device_ptr<uint> dai=	bp.getPointerToAtomicIndexBuffer(i);
				//Reuse global buffer positions with non-curved hits for next launch
				dai[0]=indexPartition[i];
			}
			uint nonCurvedHits=0u;
			for (uint i=0; i<enabledDevices; ++i) {
				nonCurvedHits += numberNonCurvedHits[i];
			}

			vt->resize(previousSize+nonCurvedHits);
			//Now copy and filter hits on non-curved walls
			for (uint i=0; i<enabledDevices; ++i) {
				//	std::cout<<"aux="<<aux<<"c_buf_size="<<c_buf_size<<"data="<<&vt<<std::endl;
				cudaMemcpy(thrust::raw_pointer_cast(vt->data())+aux, (bp.getRawPointerToHitBuffer(i)+indexPartition[i]),(numberNonCurvedHits[i])*sizeof(HitInfo),cudaMemcpyDeviceToDevice);
				aux += numberNonCurvedHits[i];

				//	std::cout<<i<<" copied "<<c_buf_size<<std::endl;

			}

			//Sort the non-curved hits
			thrust::sort(thrust::device,vt->begin(), vt->end());

			//Filter the results: keep lower distance hit per sequence
			thrust::device_vector<HitInfo>::iterator new_end=thrust::unique(thrust::device,vt->begin(), vt->end());
			return (new_end-vt->begin());
		}
	}
	//	thrust::host_vector<HitInfo> getMixedHitsMultiGPU( optix::Buffer hitBuffer,  optix::Buffer aIndex, const std::vector<int> &devices,  uint maxGlobalBufferSize, uint& curved) {
	//	
	//		uint enabledDevices=devices.size();
	//		
	//		BufferPointers<HitInfo> bp(enabledDevices);
	//		bp.fillMembers(hitBuffer,aIndex,devices,maxGlobalBufferSize); 
	//		
	//		long totalMemory=bp.getTotalHitsSize()*sizeof(HitInfo);
	//
	//		if (!bp.checkMemory(hitBuffer,totalMemory)) {
	//			//TODO: fallback to host, transfer and sort in host
	//			throw opalthrustutils::Exception("getMixedHitsMultiGPU(): Not enough memory to sort. Try reducing the number of receivers or increasing the angular sampling of rays");
	//		} else {
	//			//Create vector on device to filter results
	//			thrust::device_vector<HitInfo> vt(bp.getTotalHitsSize());
	//			//Fill the vector with the launch results. 
	//			//bp.copyToVector(vt,0u);
	//			//First, partition between hits that have touched at least one curved mesh and those which not
	//			//Do the partition on the global buffer and keep the hits on curved on global buffer
	//			std::vector<uint> indexPartition(enabledDevices);
	//			std::vector<uint> numberNonCurvedHits(enabledDevices);
	//			for (uint i=0u; i<enabledDevices; ++i) {
	//				//Partition
	//				uint c_buf_size=bp.getAtomicIndex(i);
	//				thrust::device_vector<HitInfo>::iterator  pit=thrust::partition(thrust::device,bp.getPointerToHitBuffer(i),bp.getPointerToHitBuffer(i)+c_buf_size,hit_on_curved());	
	//				HitInfo* beg=bp.getRawPointerToHitBuffer(i);
	//				//Mmmm, do we really have to do as below... see thrust/system/cuda/detail/reduce.h:963
	//				HitInfo* pitr=thrust::raw_pointer_cast(&(*pit));
	//				indexPartition[i]=pitr-beg;
	//				numberNonCurvedHits[i]=(bp.getRawPointerToHitBuffer(i)+c_buf_size)-pitr;
	//				std::cout<<"noncurved["<<i<<"]="<<numberNonCurvedHits[i]<<";indexPartition["<<i<<"]="<<indexPartition[i]<<"c_buf_size="<<c_buf_size<<std::endl;
	//				thrust::device_ptr<uint> dai=	bp.getPointerToAtomicIndexBuffer(i);
	//				//Init hitBuffer for next launch
	//				dai[0]=0u;
	//			}
	//
	//			uint nonCurvedHits=0u;
	//			uint curvedHits=0u;
	//			for (uint i=0; i<enabledDevices; ++i) {
	//				nonCurvedHits += numberNonCurvedHits[i];
	//				curvedHits += indexPartition[i];
	//			}
	//		    
	//			std::cout<<"curvedHits="<<curvedHits<<"\tnon curved hits before filtering ="<<nonCurvedHits<<"\ttotal="<<(curvedHits+nonCurvedHits)<<"getTotalHitsSize()="<<bp.getTotalHitsSize()<<std::endl;
	//			
	//			//Now copy and filter hits on non-curved walls
	//			uint vtIndex=0;
	//			for (uint i=0; i<indexPartition.size(); ++i) {
	//				//	std::cout<<"aux="<<aux<<"c_buf_size="<<c_buf_size<<"data="<<&vt<<std::endl;
	//				//First copy curved
	//				cudaMemcpy(thrust::raw_pointer_cast(vt.data())+vtIndex, bp.getRawPointerToHitBuffer(i),(indexPartition[i])*sizeof(HitInfo),cudaMemcpyDeviceToDevice);
	//				vtIndex += indexPartition[i];
	//
	//				//	std::cout<<i<<" copied "<<c_buf_size<<std::endl;
	//
	//			}
	//			for (uint i=0; i<numberNonCurvedHits.size(); ++i) {
	//				//	std::cout<<"aux="<<aux<<"c_buf_size="<<c_buf_size<<"data="<<&vt<<std::endl;
	//				//First copy curved
	//				cudaMemcpy(thrust::raw_pointer_cast(vt.data())+vtIndex, (bp.getRawPointerToHitBuffer(i)+indexPartition[i]),(numberNonCurvedHits[i])*sizeof(HitInfo),cudaMemcpyDeviceToDevice);
	//				vtIndex += numberNonCurvedHits[i];
	//
	//				//	std::cout<<i<<" copied "<<c_buf_size<<std::endl;
	//
	//			}
	//	                //Sort the curved hits
	//			thrust::sort(thrust::device,vt.begin(), vt.begin()+curvedHits);
	//
	//			//Sort the non-curved hits
	//			thrust::sort(thrust::device,vt.begin()+curvedHits, vt.end());
	//
	//			//Filter the non-curved results: keep lower distance hit per sequence
	//			thrust::device_vector<HitInfo>::iterator new_end=thrust::unique(thrust::device,vt.begin()+curvedHits, vt.end());
	//
	//			curved=curvedHits;
	//			//Transfer to host
	//			thrust::host_vector<HitInfo> hi(vt.begin(),new_end);
	//			return hi;
	//		}
	//	}
	thrust::host_vector<HitInfo> getMixedHitsMultiGPU( optix::Buffer hitBuffer,  optix::Buffer aIndex, const std::vector<int> &devices,  uint maxGlobalBufferSize, uint& curved) {

		uint enabledDevices=devices.size();

		BufferPointers<HitInfo> bp(enabledDevices);
		bp.fillMembers(hitBuffer,aIndex,devices,maxGlobalBufferSize); 

		long totalMemory=bp.getTotalHitsSize()*sizeof(HitInfo);

		if (!bp.checkMemory(hitBuffer,totalMemory)) {
			//TODO: fallback to host, transfer and sort in host
			throw opalthrustutils::Exception("getMixedHitsMultiGPU(): Not enough memory to sort. Try reducing the number of receivers or increasing the angular sampling of rays");
		} else {
			//Create vector on device to filter results
			thrust::device_vector<HitInfo> vt(bp.getTotalHitsSize());
			//Fill the vector with the launch results. 
			bp.copyToVector(vt,0u);
			//First, partition between hits that have touched at least one curved mesh and those which not
			//Do the partition on the global buffer and keep the hits on curved on global buffer
			std::vector<uint> indexPartition(enabledDevices);
			std::vector<uint> numberNonCurvedHits(enabledDevices);
			for (uint i=0u; i<enabledDevices; ++i) {
				cudaSetDevice(i);
				//Partition
				uint c_buf_size=bp.getAtomicIndex(i);
				thrust::device_vector<HitInfo>::iterator  pit=thrust::partition(thrust::device,bp.getPointerToHitBuffer(i),bp.getPointerToHitBuffer(i)+c_buf_size,hit_on_curved());	
				HitInfo* beg=bp.getRawPointerToHitBuffer(i);
				//Mmmm, do we really have to do as below... see thrust/system/cuda/detail/reduce.h:963
				HitInfo* pitr=thrust::raw_pointer_cast(&(*pit));
				indexPartition[i]=pitr-beg;
				numberNonCurvedHits[i]=(bp.getRawPointerToHitBuffer(i)+c_buf_size)-pitr;
				std::cout<<"noncurved["<<i<<"]="<<numberNonCurvedHits[i]<<";indexPartition["<<i<<"]="<<indexPartition[i]<<"c_buf_size="<<c_buf_size<<std::endl;
				thrust::device_ptr<uint> dai=	bp.getPointerToAtomicIndexBuffer(i);
				//Init hitBuffer for next launch
				dai[0]=0u;
			}

			uint nonCurvedHits=0u;
			uint curvedHits=0u;
			for (uint i=0; i<enabledDevices; ++i) {
				nonCurvedHits += numberNonCurvedHits[i];
				curvedHits += indexPartition[i];
			}

			std::cout<<"curvedHits="<<curvedHits<<"\tnon curved hits before filtering ="<<nonCurvedHits<<"\ttotal="<<(curvedHits+nonCurvedHits)<<"getTotalHitsSize()="<<bp.getTotalHitsSize()<<std::endl;

			//Now copy and filter hits on non-curved walls
			uint vtIndex=0;
			for (uint i=0; i<indexPartition.size(); ++i) {
				//	std::cout<<"aux="<<aux<<"c_buf_size="<<c_buf_size<<"data="<<&vt<<std::endl;
				//First copy curved
				cudaMemcpy(thrust::raw_pointer_cast(vt.data())+vtIndex, bp.getRawPointerToHitBuffer(i),(indexPartition[i])*sizeof(HitInfo),cudaMemcpyDeviceToDevice);
				vtIndex += indexPartition[i];

				//	std::cout<<i<<" copied "<<c_buf_size<<std::endl;

			}
			for (uint i=0; i<numberNonCurvedHits.size(); ++i) {
				//	std::cout<<"aux="<<aux<<"c_buf_size="<<c_buf_size<<"data="<<&vt<<std::endl;
				//First copy curved
				cudaMemcpy(thrust::raw_pointer_cast(vt.data())+vtIndex, (bp.getRawPointerToHitBuffer(i)+indexPartition[i]),(numberNonCurvedHits[i])*sizeof(HitInfo),cudaMemcpyDeviceToDevice);
				vtIndex += numberNonCurvedHits[i];

				//	std::cout<<i<<" copied "<<c_buf_size<<std::endl;

			}
			//Sort the curved hits
			thrust::sort(thrust::device,vt.begin(), vt.begin()+curvedHits);

			//Sort the non-curved hits
			thrust::sort(thrust::device,vt.begin()+curvedHits, vt.end());

			//Filter the non-curved results: keep lower distance hit per sequence
			thrust::device_vector<HitInfo>::iterator new_end=thrust::unique(thrust::device,vt.begin()+curvedHits, vt.end());

			curved=curvedHits;
			//Transfer to host
			thrust::host_vector<HitInfo> hi(vt.begin(),new_end);
			return hi;
		}
	}


	void initializeGenerators(curandGenerator_t* gen, unsigned long long seed1, curandGenerator_t* gen2, unsigned long long seed2) {
		std::cout<<"Initializing generators"<<std::endl;
		CURAND_CALL(curandCreateGenerator(gen, 
					CURAND_RNG_PSEUDO_DEFAULT));

		/* Set seed */
		CURAND_CALL(curandSetPseudoRandomGeneratorSeed(*gen, 
					seed1));
		//CURAND_CALL(curandSetPseudoRandomGeneratorSeed(*gen, 
		//			1234ULL));
		CURAND_CALL(curandCreateGenerator(gen2, 
					CURAND_RNG_PSEUDO_DEFAULT));

		/* Set seed */
		CURAND_CALL(curandSetPseudoRandomGeneratorSeed(*gen2, 
					seed2));
		//CURAND_CALL(curandSetPseudoRandomGeneratorSeed(*gen2, 
		//			4357ULL));
	} 
	void generateRandomUniformRays(float eli,float elf,float azi, float aze, long r, DVector<float3>* dv, curandGenerator_t& gen, curandGenerator_t& gen2)
	{
		thrust::device_vector<float3>* result=dv->getDeviceVector();
		result->resize(r);
		//Generate sequences 
		//curandGenerator_t gen;
		//float *devData;
		//curandGenerator_t gen2; //float *devData2;

		/* Allocate n floats on device */
		//CUDA_CALL(cudaMalloc((void **)&devData, r*sizeof(float)));
		thrust::device_vector<float> s1(r);
		thrust::device_vector<float> s2(r);

		/* Create pseudo-random number generator */
		//CURAND_CALL(curandCreateGenerator(&gen, CURAND_RNG_PSEUDO_DEFAULT));

		/* Set seed */
		//CURAND_CALL(curandSetPseudoRandomGeneratorSeed(gen, 1234ULL));

		/* Generate n floats on device */
		//	CURAND_CALL(curandGenerateUniform(gen, devData, r));	
		CURAND_CALL(curandGenerateUniform(gen, s1.data().get(), r));	


		/* Allocate n floats on device */
		//	CUDA_CALL(cudaMalloc((void **)&devData2, r*sizeof(float)));

		/* Create pseudo-random number generator */
		//CURAND_CALL(curandCreateGenerator(&gen2, CURAND_RNG_PSEUDO_DEFAULT));

		/* Set seed */
		//	CURAND_CALL(curandSetPseudoRandomGeneratorSeed(gen2, 4357ULL));

		/* Generate n floats on device */
		//CURAND_CALL(curandGenerateUniform(gen2, devData2, r));	
		CURAND_CALL(curandGenerateUniform(gen2, s2.data().get(), r));

		//for (auto s : s1) {
		//	std::cout<<s<<std::endl;
		//}
		//for (auto s : s2) {
		//	std::cout<<s<<std::endl;
		//}
		//rr.eli=eli;
		//rr.elf=elf;
		//rr.azi=azi;
		//rr.aze=aze;
		//Create a zip iterator and transform to get the ray
		thrust::transform( thrust::make_zip_iterator(thrust::make_tuple(s1.begin(), s2.begin())),
				thrust::make_zip_iterator(thrust::make_tuple(s1.end(),   s2.end())),
				result->begin(),
				RandomRay(eli, elf, azi, aze) );	
	}

	void generateRandomUniformRaysOnSphere(long r, DVector<float3>* dv, curandGenerator_t& gen, curandGenerator_t& gen2)
	{
		thrust::device_vector<float3>* result=dv->getDeviceVector();
		result->resize(r);
		//Generate sequences 
		//curandGenerator_t gen;
		//float *devData;
		//curandGenerator_t gen2; //float *devData2;

		/* Allocate n floats on device */
		//CUDA_CALL(cudaMalloc((void **)&devData, r*sizeof(float)));
		thrust::device_vector<float> s1(r);
		thrust::device_vector<float> s2(r);

		/* Create pseudo-random number generator */
		//CURAND_CALL(curandCreateGenerator(&gen, CURAND_RNG_PSEUDO_DEFAULT));

		/* Set seed */
		//CURAND_CALL(curandSetPseudoRandomGeneratorSeed(gen, 1234ULL));

		/* Generate n floats on device */
		//	CURAND_CALL(curandGenerateUniform(gen, devData, r));	
		CURAND_CALL(curandGenerateUniform(gen, s1.data().get(), r));	


		/* Allocate n floats on device */
		//	CUDA_CALL(cudaMalloc((void **)&devData2, r*sizeof(float)));

		/* Create pseudo-random number generator */
		//CURAND_CALL(curandCreateGenerator(&gen2, CURAND_RNG_PSEUDO_DEFAULT));

		/* Set seed */
		//	CURAND_CALL(curandSetPseudoRandomGeneratorSeed(gen2, 4357ULL));

		/* Generate n floats on device */
		//CURAND_CALL(curandGenerateUniform(gen2, devData2, r));	
		CURAND_CALL(curandGenerateUniform(gen2, s2.data().get(), r));

		//for (auto s : s1) {
		//	std::cout<<s<<std::endl;
		//}
		//for (auto s : s2) {
		//	std::cout<<s<<std::endl;
		//}
		//rr.eli=eli;
		//rr.elf=elf;
		//rr.azi=azi;
		//rr.aze=aze;
		//Create a zip iterator and transform to get the ray
		thrust::transform( thrust::make_zip_iterator(thrust::make_tuple(s1.begin(), s2.begin())),
				thrust::make_zip_iterator(thrust::make_tuple(s1.end(),   s2.end())),
				result->begin(),
				RandomRayOnSphere() );	
	}

}
