

#
# Copyright (c) 2016, NVIDIA CORPORATION. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
#  * Neither the name of NVIDIA CORPORATION nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS ``AS IS'' AND ANY
# EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
# PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR
# CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
# EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
# PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
# PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY
# OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#
function(OPTIX_add_sample_library target_name)

  # These calls will group PTX and CUDA files into their own directories in the Visual
  # Studio projects.
  if (NOT CUDA_NVRTC_ENABLED)
    source_group("PTX Files"  REGULAR_EXPRESSION ".+\\.ptx$")
  endif()
  source_group("CUDA Files" REGULAR_EXPRESSION ".+\\.cu$")

  # Separate the sources from the CMake and CUDA options fed to the macro.  This code
  # comes from the CUDA_COMPILE_PTX macro found in FindCUDA.cmake.  We are copying the
  # code here, so that we can use our own name for the target.  target_name is used in the
  # creation of the output file names, and we want this to be unique for each target in
  # the SDK.
  CUDA_GET_SOURCES_AND_OPTIONS(source_files cmake_options options ${ARGN})

  if (CUDA_NVRTC_ENABLED)
  
    # Isolate OBJ target files. NVCC should only process these files and leave PTX targets for NVRTC
    set(cu_obj_source_files)
    foreach(file ${source_files})
      get_source_file_property(_cuda_source_format ${file} CUDA_SOURCE_PROPERTY_FORMAT)
      if(${_cuda_source_format} MATCHES "OBJ")
        list(APPEND cu_obj_source_files ${file})
      endif()
    endforeach()

    # Create the rules to build the OBJ from the CUDA files.
    CUDA_WRAP_SRCS( ${target_name} OBJ generated_files ${cu_obj_source_files} ${cmake_options} OPTIONS ${options} )
  else()
  
    # Create the rules to build the PTX and OBJ from the CUDA files.
    CUDA_WRAP_SRCS( ${target_name} PTX generated_files ${source_files} ${cmake_options} OPTIONS ${options} )
  endif()
  

  # Here is where we create the rule to make the executable.  We define a target name and
  # list all the source files used to create the target.  In addition we also pass along
  # the cmake_options parsed out of the arguments.
  add_library(${target_name}
    ${source_files}
    ${generated_files}
    ${cmake_options}
    )

  # Most of the samples link against the sutil library and the optix library.  Here is the
  # rule that specifies this linkage.
  target_link_libraries( ${target_name}
    sutil_sdk
    optix
    ${optix_rpath}
    )

  if( UNIX AND NOT APPLE )
    # Force using RPATH instead of RUNPATH on Debian
    target_link_libraries( ${target_name} "-Wl,--disable-new-dtags" )
  endif()
# Use gcc rather than g++ to link if we are linking statically against libgcc_s and libstdc++
if(USING_GNU_C OR USING_GNU_CXX)
  if(GCC_LIBSTDCPP_HACK)
    set_target_properties(${target_name} PROPERTIES LINKER_LANGUAGE "C")
    target_link_libraries(${target_name} LINK_PRIVATE ${STATIC_LIBSTDCPP})
  endif()
endif()
  if(USING_GNU_CXX)
    target_link_libraries( ${target_name} m ) # Explicitly link against math library (C samples don't do that by default)
  endif()
target_link_libraries(${target_name}
  optix
  optixu
  sutil_sdk
  )

  if(USING_GNU_CXX)
    target_link_libraries( ${target_name} m ) # Explicitly link against math library (C samples don't do that by default)
  endif()
endfunction()


set_source_files_properties(${CMAKE_CURRENT_SOURCE_DIR}/tutils.cu
  PROPERTIES CUDA_SOURCE_PROPERTY_FORMAT OBJ
  )

# See top level CMakeLists.txt file for documentation of OPTIX_add_sample_executable.
OPTIX_add_sample_executable( opal 
  Opal.cpp
  Opal.h
  Common.h
  Complex.h
  Unity.h
  Unity.cpp
  tests.cpp
  traceFunctions.h
  multitransmitter.h
  multitransmitter.cpp
  generation.cu
  receiver.cu
  optixGeometryTriangles.cu
  triangle.cu
  sphere.cu
  tutils.cu
  tutils.h
  #To see it properly in Visual Studio we should add the following
   polarization/generation.cu
  polarization/receiver.cu
  polarization/triangle.cu
  polarization/tracePolarizationFunctions.h
  polarization/sphere.cu
  polarization/optixGeometryTriangles.cu

  )

target_link_libraries( opal 
     ${CUDA_LIBRARIES}
  optix
  optixu
  sutil_sdk
  )
set(sources
  Opal.cpp
  Opal.h
  Common.h
  Complex.h
  Unity.h
  Unity.cpp
  tests.cpp
  multitransmitter.h
  multitransmitter.cpp
  traceFunctions.h
  generation.cu
  receiver.cu
  optixGeometryTriangles.cu
  triangle.cu
  sphere.cu
  tutils.cu
  tutils.h
   polarization/generation.cu
  polarization/receiver.cu
  polarization/triangle.cu
  polarization/tracePolarizationFunctions.h
  polarization/sphere.cu
  polarization/optixGeometryTriangles.cu

)
OPTIX_add_sample_library( opal_s 
  Opal.cpp
  Opal.h
  Common.h
  Complex.h
  Unity.h
  Unity.cpp
  tests.cpp
  multitransmitter.h
  multitransmitter.cpp
  traceFunctions.h
  generation.cu
  receiver.cu
  optixGeometryTriangles.cu
  triangle.cu
  sphere.cu
  tutils.cu
  tutils.h
   polarization/generation.cu
  polarization/receiver.cu
  polarization/triangle.cu
  polarization/tracePolarizationFunctions.h
  polarization/sphere.cu
  polarization/optixGeometryTriangles.cu

  )

target_link_libraries( opal_s 
     ${CUDA_LIBRARIES}
  optix
  optixu
  sutil_sdk
  )
set(sources
  Opal.cpp
  Opal.h
  Common.h
  Complex.h
  Unity.h
  Unity.cpp
  tests.cpp
  multitransmitter.h
  multitransmitter.cpp
  traceFunctions.h
  generation.cu
  receiver.cu
  optixGeometryTriangles.cu
  triangle.cu
  sphere.cu
  tutils.cu
  tutils.h
   polarization/generation.cu
  polarization/receiver.cu
  polarization/triangle.cu
  polarization/tracePolarizationFunctions.h
  polarization/sphere.cu
  polarization/optixGeometryTriangles.cu

)
