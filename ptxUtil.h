/***************************************************************/
//
//Copyright (c) 2019 Esteban Egea-Lopez http://girtel.upct.es/~eegea
//
/**************************************************************/


//NVIDIA copyright below

/*
 * Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *  * Neither the name of NVIDIA CORPORATION nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS ``AS IS'' AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY
 * OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef PTXUTIL_H
#define PTXUTIL_H
#include <map>
#include <vector>
#include <string>
namespace opal {
	//We copy some functions from sutil.cpp in order to be able to disable fast math and other NVRTC compiler options at runtime without modifying the SDK.
	class PtxUtil {	
		struct PtxSourceCache
		{
			std::map<std::string, std::string *> map;
			~PtxSourceCache()
			{
				for( std::map<std::string, std::string *>::const_iterator it = map.begin(); it != map.end(); ++it )
					delete it->second;
			}
		};

		PtxSourceCache g_ptxSourceCache;

		std::string g_nvrtcLog;
		std::vector<const char *> nvrtcOptions;	
		public: 
		PtxUtil(std::vector<const char *> options);

		//For some reason this is not part of sutil namespace
		bool readSourceFile( std::string &str, const std::string &filename );
		void getPtxStringFromFile( std::string &ptx, const char* sample_name, const char* filename );


		const char* getPtxString( const char* sample, const char* filename, const char** log=NULL );

		void getCuStringFromFile( std::string &cu, std::string& location, const char* sample_name, const char* filename );

		void getPtxFromCuString( std::string &ptx, const char* sample_name, const char* cu_source, const char* name, const char** log_string);
	};
}
#endif

