/***************************************************************/
//
//Copyright (c) 2019 Esteban Egea-Lopez http://girtel.upct.es/~eegea
//
/**************************************************************/

#ifndef TUTILS_H
#define TUTILS_H


//TODO:clean this mess and change to templates

//DO NOT MESS WITH THE ORDER OF INCLUDES OR THE INCLUDED HEADERS HERE
//TODO: I have not found yet why changing the order of some includes break either the compilation or the nvrtc compilation..

#include <cuda_runtime.h>
//#include "device_launch_parameters.h"
#include <optix_world.h>
#include <optixu/optixu_math_namespace.h> 
#include <optixu/optixu_matrix_namespace.h> 
#include <optixu/optixpp_namespace.h>
#include "Common.h"
#include <thrust/host_vector.h>
#include <thrust/device_vector.h>
#include <thrust/sort.h>
#include <thrust/unique.h>
#include <thrust/partition.h>
#include <vector>
#include <curand.h>
#include <curand_kernel.h>

//Visual studio does not know uint
#ifdef _WIN32
 typedef unsigned int uint;
#endif

#include <assert.h>
//Functions to filter duplicate hits and other post launch processing on device with the Thrust library

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
	template<typename T>
		class PartialLaunchState {
			thrust::device_vector<T>* previousHits;
			uint partialLaunchGlobalIndex;
			public:
			PartialLaunchState() :  partialLaunchGlobalIndex(0u)  {
				this->previousHits = new thrust::device_vector<T>();
			}
			virtual ~PartialLaunchState() {
				delete previousHits;
			}

			uint  getIndex() const {return partialLaunchGlobalIndex;};
			thrust::device_vector<T>*  getDeviceHits()  {return previousHits;};
			void setIndex(uint index) { partialLaunchGlobalIndex=index;};
			std::vector<T> getHits() {
				std::vector<T> h(partialLaunchGlobalIndex);
				if (partialLaunchGlobalIndex>0) {
					thrust::copy_n(previousHits->begin(),partialLaunchGlobalIndex,h.begin());
				}
				return h;
			}
			uint getDeviceHitsBufferSize() const {return previousHits->size();};	


		};
	//Utility class to handle dev pointers and keep all thrust related here. Otherwise we get a nightmare of include errors
	template<typename T>
		class DVector {
			thrust::device_vector<T>* dv;
			public:
			DVector()   {
				//Force to create 
				cudaSetDevice(0);
				this->dv = new thrust::device_vector<T>();
			}
			virtual ~DVector() {
				delete dv;
			}

			thrust::device_vector<T>*  getDeviceVector()  {return dv;};
			uint getDeviceVectorSize() const {return dv->size();};	
			T* getPointer() { return dv->data().get();}; 


		};


	template<typename T>
		class BufferPointers {
			uint enabledDevices;
			std::vector<uint*> ai_ptrs;
			std::vector<T*> raw_ptrs;
			thrust::device_vector<thrust::device_ptr<T>> dev_ptrs;
			thrust::device_vector<thrust::device_ptr<uint>> dev_ai;
			std::vector<uint> ai_size;

			uint totalHitsSize;
			public:
			BufferPointers(uint n) : enabledDevices(n), ai_ptrs(n), raw_ptrs(n), dev_ptrs(n), dev_ai(n), ai_size(n), totalHitsSize(0) {
			}
			void fillMembers(optix::Buffer hitBuffer, optix::Buffer aIndex, const std::vector<int> &devices, uint maxGlobalBufferSize) {
				for (uint i=0; i<enabledDevices; ++i) {
					//Get device pointer to atomic index buffer (per GPU)
					uint* ai_ptr=	static_cast<uint*>( aIndex->getDevicePointer(devices[i]));
					thrust::device_ptr<uint> dev_ptr_ai = thrust::device_pointer_cast(ai_ptr);
					//Copy memory from device...
					uint s= dev_ptr_ai[0];
					ai_ptrs[i]=ai_ptr;
					//Get device pointer to global buffer (per GPU)
					raw_ptrs[i]=static_cast<T*>( hitBuffer->getDevicePointer(devices[i]));
					dev_ptrs[i]=thrust::device_pointer_cast(raw_ptrs[i]);
					if (s>0) {
						std::cout<<i<<" BufferPointers::fillMembers(): s="<<s<<"; mem="<<(s*sizeof(T)/(1024*1024))<<" MiB"<<std::endl;
					}
					if (s>maxGlobalBufferSize) {
						throw opalthrustutils::Exception("BufferPointers::fillMembers(): globalHitBuffer overflow. Try changing fractionMemGlobalBufferSize or reducing the ray density or sphere radius ");

					}	
					ai_size[i]=s;
					dev_ai[i]=dev_ptr_ai;

					totalHitsSize +=s;
				}	
			}	
			T* getRawPointerToHitBuffer(uint i) {
				return raw_ptrs[i];
			}
			thrust::device_ptr<T> getPointerToHitBuffer(uint i) {	
				return dev_ptrs[i];
			}
			thrust::device_ptr<uint> getPointerToAtomicIndexBuffer(uint i) {
				return dev_ai[i];
			}
			uint getAtomicIndex(uint i) {
				return ai_size[i];
			}
			uint getTotalHitsSize() const { return totalHitsSize;};
			uint getEnabledDevices() const { return enabledDevices;};
			bool checkMemory(optix::Buffer hitBuffer, long totalMemory) {
				float GiB=1048576.0f*1024.0f;
				std::vector<RTsize> memoryInDevice(enabledDevices);
				for (uint i=0; i<enabledDevices; ++i) {
					memoryInDevice[i]=hitBuffer->getContext()->getAvailableDeviceMemory(i);
				}


				bool enoughMemory=false;
				for (int i=0; i<enabledDevices; ++i) {
					if (totalMemory<memoryInDevice[i]){
						enoughMemory=true;
						break;
					} else {
						std::cout<<"bp.checkMemory(): totalMemory="<<(totalMemory/GiB)<<" GBi, memoryInDevice["<<i<<"]="<<(memoryInDevice[i]/GiB)<<" GBi"<<std::endl;
					}	
				}
				return enoughMemory;
			}
			void copyToVector(thrust::device_vector<T>& vt, uint offset) {
				uint aux=offset;
				for (uint i=0; i<enabledDevices; ++i) {
					uint c_buf_size=getAtomicIndex(i);
					cudaMemcpy(thrust::raw_pointer_cast(vt.data())+aux, getRawPointerToHitBuffer(i),(c_buf_size)*sizeof(T),cudaMemcpyDeviceToDevice);
					aux += c_buf_size;
				}
			}
		};


	//Full launches: all hits are sorted, filtered and transferred to host (or prepared to be transferred)

	//Basic filtering. It removes all duplicated hits on device. The result (filtered hits) is transferred to host 
	//A duplicate  is defined by T operator< for sorting and operator== for unique, see Common.h
	//For instance, first with sort we order by hash and distance to receiver and with unique we keep the hit with lowest distance

	thrust::host_vector<HitInfo> filterHitsAndTransfer( optix::Buffer hitBuffer, optix::Buffer aIndex, const std::vector<int> &devices, uint maxGlobalBufferSize); 


	thrust::host_vector<HitInfo> getAllHitsOrderedMultiGPU( optix::Buffer hitBuffer, optix::Buffer aIndex, const std::vector<int> &devices, uint maxGlobalBufferSize, bool sort=true); 
	thrust::host_vector<HitInfo> copyHitsToHostMultiGPU( optix::Buffer hitBuffer,  optix::Buffer aIndex, const std::vector<int> &devices, uint maxGlobalBufferSize); 
	thrust::host_vector<HitInfo> getMixedHitsMultiGPU( optix::Buffer hitBuffer,  optix::Buffer aIndex, const std::vector<int> &devices, uint maxGlobalBufferSize, uint& curved);

	//For partial launches
	//Just filter but do not copy for transfer. Used for partial launches
	uint filterHits(optix::Buffer hitBuffer, uint bsize); 
	//Just copy for transfer. Used for partial launches, to transfer when partial launch is finished. Assume hits have been sorted and filtered before
	uint copyHitsToTransfer(optix::Buffer hitBuffer, optix::Buffer resultBuffer, uint bsize);
	//uint filterHitsMultiGPU( optix::Buffer hitBuffer,  optix::Buffer aIndex, const std::vector<int> &devices,  thrust::device_vector<HitInfo> &vt, uint previousSize );
	uint filterHitsMultiGPU( optix::Buffer hitBuffer,  optix::Buffer aIndex, const std::vector<int> &devices,  PartialLaunchState<HitInfo>* state, uint maxGlobalBufferSize );
	uint getAllHitsOrderedMultiGPU( optix::Buffer hitBuffer,  optix::Buffer aIndex, const std::vector<int> &devices, PartialLaunchState<HitInfo>* state, uint maxGlobalBufferSize); 
	void initializeGenerators(curandGenerator_t* gen, unsigned long long seed1, curandGenerator_t* gen2, unsigned long long seed2); 
	uint getMixedHitsMultiGPU( optix::Buffer hitBuffer,  optix::Buffer aIndex, const std::vector<int> &devices, PartialLaunchState<HitInfo>* state, uint maxGlobalBufferSize);
	
	thrust::host_vector<RDNHit> getReceivedFieldMultiGPU( optix::Buffer hitBuffer,const std::vector<int> &devices,  uint nrx); 


	//Log traces	
	thrust::host_vector<LogTraceHitInfo> getLogTraceOrderer( optix::Buffer traceBuffer,  optix::Buffer aIndex, const std::vector<int> &devices, uint maxGlobalBufferSize);


	struct hit_on_curved
	{
		__host__ __device__
			bool operator()(const HitInfo &x)
			{
				uint curved = (x.thrd.w >> FLAG_CURVED_MESH_POSITION) & 1u;
				return (curved==1u);
			}
	};
	typedef thrust::tuple<float,float> rngPair;

	//Random ray generation

	//See http://www.pbr-book.org/3ed-2018/Monte_Carlo_Integration/2D_Sampling_with_Multidimensional_Transformations.html
	//General method is explained after (13.12): "first compute the marginal density to isolate one particular variable and draw a sample from that density using standard 1D techniques. Once that sample is drawn, one can then compute the conditional density function given that value and draw a sample from that distribution, again using standard 1D sampling techniques."
	//That is Generate X1: from p(x)=int[p(x,y)dy], Then F(x<=X)=int[p(x)dx]=U1, The p(y|x)=p(x,y)/p(x) F[y<=Y|x)=U2
	void generateRandomUniformRays(float eli,float elf,float azi, float aze, long r, DVector<float3>* v, curandGenerator_t& gen, curandGenerator_t& gen2);	
	void generateRandomUniformRaysOnSphere(long r, DVector<float3>* v, curandGenerator_t& gen, curandGenerator_t& gen2);	

	struct RandomRay 
	{
		float eli; 
		float elf;
		float azi;
		float aze; 
		RandomRay(float a, float b, float c, float d) : eli(a), elf(b), azi(c), aze(d) {};	
		__host__ __device__
			float3 operator()(const rngPair& a) const
			{
				float cosEl=cosf(eli)+ thrust::get<0>(a)*(cosf(elf)-cosf(eli) );
				float sinEl=sqrt(1.0 -(cosEl*cosEl));
				float azr=(aze-azi)*thrust::get<1>(a)  + azi;

				return  make_float3(sinEl*sinf(azr), cosEl, sinEl*cosf(azr) );        
				//return  make_float3(sinEl*cosf(azr),  sinEl*sinf(azr), cosEl );   //Left-handed with elevation from Z and azimuth clockwise from X to Y 
			}
	};

	struct RandomRayOnSphere 
	{
		__host__ __device__
			float3 operator()(const rngPair& a) const
			{
				float cosEl=1-2*thrust::get<0>(a);
				float sinEl =sqrtf(1.0f- (cosEl*cosEl)); 
				float azr=2*M_PIf*thrust::get<1>(a);

				return  make_float3(sinEl*sinf(azr), cosEl, sinEl*cosf(azr) );        
			}
	};
	//#ifndef OPAL_EXTENDED_HITINFO
	optix::float2 getReducedFieldMultiGPU( optix::Buffer hitBuffer,  optix::Buffer aIndex, const std::vector<int> &devices, uint maxGlobalBufferSize);
	struct GetRealE 
	{
		__host__ __device__
			float operator()(const HitInfo& a) const
			{
				return  a.EEx.x  ;        
			}
	};

	struct GetImE 
	{
		__host__ __device__
			float operator()(const HitInfo& a) const
			{
				return a.EEx.y  ;        
			}
	};

//	#endif
} //namespace opalthrustutils
#endif

