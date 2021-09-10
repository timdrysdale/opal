#include "ptxUtil.h"
#include <nvrtc.h>
#include <fstream>
#include <sstream>
#include <sampleConfig.h>
#include <sutil.h>
#include <optix_world.h>

using namespace optix;

#define STRINGIFY(x) STRINGIFY2(x)
#define STRINGIFY2(x) #x
#define LINE_STR STRINGIFY(__LINE__)

// Error check/report helper for users of the C API
#define NVRTC_CHECK_ERROR_OPAL( func )                             \
	do {                                                             \
		nvrtcResult code = func;                                       \
		if( code != NVRTC_SUCCESS )                                    \
		throw Exception( "ERROR: " __FILE__ "(" LINE_STR "): " +     \
				std::string( nvrtcGetErrorString( code ) ) );            \
	} while( 0 )

namespace opal {
	
	PtxUtil::PtxUtil(std::vector<const char *> options) {
		this->nvrtcOptions=options;
	}	
	
	//sutil functions
	bool PtxUtil::readSourceFile( std::string &str, const std::string &filename )
	{
		// Try to open file
		std::ifstream file( filename.c_str() );
		if( file.good() )
		{
			// Found usable source file
			std::stringstream source_buffer;
			source_buffer << file.rdbuf();
			str = source_buffer.str();
			return true;
		}
		return false;
	}
	void PtxUtil::getPtxStringFromFile( std::string &ptx, const char* sample_name, const char* filename )
	{
		std::string source_filename;
		if (sample_name)
			source_filename = std::string( sutil::samplesPTXDir() ) + "/" + std::string( sample_name ) + "_generated_" + std::string( filename ) + ".ptx";
		else
			source_filename = std::string( sutil::samplesPTXDir() ) + "/cuda_compile_ptx_generated_" + std::string( filename ) + ".ptx";

		// Try to open source PTX file
		if (!readSourceFile( ptx, source_filename ))
			throw Exception( "Couldn't open source file " + source_filename );
	}



	const char* PtxUtil::getPtxString( const char* sample, const char* filename, const char** log )
	{
		if (log)
			*log = NULL;

		std::string *ptx, cu;
		std::string key = std::string( filename ) + ";" + ( sample ? sample : "" );
		std::map<std::string, std::string *>::iterator elem = g_ptxSourceCache.map.find( key );

		if( elem == g_ptxSourceCache.map.end() )
		{
			ptx = new std::string();
#if CUDA_NVRTC_ENABLED
			std::string location;
			getCuStringFromFile( cu, location, sample, filename );
			getPtxFromCuString( *ptx, sample, cu.c_str(), location.c_str(), log  );
#else
			getPtxStringFromFile( *ptx, sample, filename );
#endif
			g_ptxSourceCache.map[key] = ptx;
		}
		else
		{
			ptx = elem->second;
		}

		return ptx->c_str();
	}


	void PtxUtil::getCuStringFromFile( std::string &cu, std::string& location, const char* sample_name, const char* filename )
	{
		std::vector<std::string> source_locations;

		std::string base_dir = std::string( sutil::samplesDir() );

		// Potential source locations (in priority order)
	
		source_locations.push_back(std::string(sample_name) + "/"  + std::string(filename) );
		if( sample_name ) {
			source_locations.push_back( base_dir + "/" + sample_name + "/" + filename );
		}
		source_locations.push_back( base_dir + "/cuda/" + filename );

		for( std::vector<std::string>::const_iterator it = source_locations.begin(); it != source_locations.end(); ++it ) {
			// Try to get source code from file
			if( readSourceFile( cu, *it ) )
			{
				location = *it;
				return;
			}

		}

		// Wasn't able to find or open the requested file
		throw Exception( "Couldn't open source file " + std::string( filename ) );
	}

	void PtxUtil::getPtxFromCuString( std::string &ptx, const char* sample_name, const char* cu_source, const char* name, const char** log_string)
	{
		// Create program
		nvrtcProgram prog = 0;
		NVRTC_CHECK_ERROR_OPAL( nvrtcCreateProgram( &prog, cu_source, name, 0, NULL, NULL ) );

		// Gather NVRTC options
		std::vector<const char *> options;

		std::string base_dir = std::string( sutil::samplesDir() );

		// Set sample dir as the primary include path
		std::string sample_dir;
		if( sample_name )
		{
			sample_dir = std::string( "-I" ) + base_dir + "/" + sample_name;
			options.push_back( sample_dir.c_str() );
		}

		// Collect include dirs
		std::vector<std::string> include_dirs;
		const char *abs_dirs[] = { SAMPLES_ABSOLUTE_INCLUDE_DIRS };
		const char *rel_dirs[] = { SAMPLES_RELATIVE_INCLUDE_DIRS };

		const size_t n_abs_dirs = sizeof( abs_dirs ) / sizeof( abs_dirs[0] );
		for( size_t i = 0; i < n_abs_dirs; i++ )
			include_dirs.push_back(std::string( "-I" ) + abs_dirs[i]);
		const size_t n_rel_dirs = sizeof( rel_dirs ) / sizeof( rel_dirs[0] );
		for( size_t i = 0; i < n_rel_dirs; i++ )
			include_dirs.push_back(std::string( "-I" ) + base_dir + rel_dirs[i]);
		for( std::vector<std::string>::const_iterator it = include_dirs.begin(); it != include_dirs.end(); ++it )
			options.push_back( it->c_str() );

		// Collect NVRTC options
		for( size_t i = 0; i < nvrtcOptions.size(); i++ ) {
		  options.push_back( nvrtcOptions[i] );
		}
		//std::cout<< "** NVRTC options for "<<name<<std::endl;
		//for (int i=0; i<options.size(); ++i) {
		//	std::cout<<options[i]<<std::endl;
		//}
		// JIT compile CU to PTX
		const nvrtcResult compileRes = nvrtcCompileProgram( prog, (int) options.size(), options.data() );

		// Retrieve log output
		size_t log_size = 0;
		NVRTC_CHECK_ERROR_OPAL( nvrtcGetProgramLogSize( prog, &log_size ) );
		g_nvrtcLog.resize( log_size );
		if( log_size > 1 )
		{
			NVRTC_CHECK_ERROR_OPAL( nvrtcGetProgramLog( prog, &g_nvrtcLog[0] ) );
			if( log_string )
				*log_string = g_nvrtcLog.c_str();
		}
		if( compileRes != NVRTC_SUCCESS )
			throw Exception( "NVRTC Compilation failed.\n" + g_nvrtcLog );

		// Retrieve PTX code
		size_t ptx_size = 0;
		NVRTC_CHECK_ERROR_OPAL( nvrtcGetPTXSize( prog, &ptx_size ) );
		ptx.resize( ptx_size );
		NVRTC_CHECK_ERROR_OPAL( nvrtcGetPTX( prog, &ptx[0] ) );

		// Cleanup
		NVRTC_CHECK_ERROR_OPAL( nvrtcDestroyProgram( &prog ) );
	}


}
