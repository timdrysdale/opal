

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
    #mlpack
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
  #mlpack
  )

  if(USING_GNU_CXX)
    target_link_libraries( ${target_name} m ) # Explicitly link against math library (C samples don't do that by default)
  endif()
endfunction()


set_source_files_properties(${CMAKE_CURRENT_SOURCE_DIR}/tutils.cu
  PROPERTIES CUDA_SOURCE_PROPERTY_FORMAT OBJ
 )
set(OPAL_SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}")
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/Config.h.in ${CMAKE_CURRENT_SOURCE_DIR}/Config.h @ONLY)
 

# See top level CMakeLists.txt file for documentation of OPTIX_add_sample_executable.
OPTIX_add_sample_executable( opal 
  Opal.cpp
  Opal.h
  Common.h
  Config.h
  util.cpp
  util.h
  Unity.h
  Unity.cpp
  main.cpp
  transmitterManager.h
  transmitterManager.cpp
  opalSimulation.h
  opalSimulation.cpp
  basicSimulation.h
  basicSimulation.cpp
  flatSimulation.h
  flatSimulation.cpp
  curvedMeshSimulation.h
  curvedMeshSimulation.cpp
  curvedFlatMeshSimulation.h
  curvedFlatMeshSimulation.cpp
  rayDensityNormalizationSimulation.h
  rayDensityNormalizationSimulation.cpp
  tutils.cu
  tutils.h
  timer.h
  timer.cpp
  tests/rdn.h
  tests/rdn.cpp
  tests/tests.h
  tests/tests.cpp
  tests/tunnels.cpp
  tests/tunnels.h
  tests/depolarization.h
  tests/depolarization.cpp
  tests/curvature.h
  tests/curvature.cpp
  tests/tunnelsBase.h
  tests/tunnelsBase.cpp
  tests/dudley.h
  tests/dudley.cpp
  tests/diffraction.h
  tests/diffraction.cpp
  tests/antennaGain.h
  tests/antennaGain.cpp
  ptxUtil.h
  ptxUtil.cpp
  raySphere.h
  raySphere.cpp
  singleDiffraction.h
  singleDiffraction.cpp
  #To see it properly in Visual Studio we should add the following. 
  optix/intersect/sphere.cu
  optix/intersect/optixGeometryTriangles.cu
  optix/intersect/optixTriangle.cu
  optix/intersect/optixGeometryTrianglesCurved.cu
  optix/intersect/optixTriangleCurved.cu
  optix/intersect/exception.cu
  optix/basic/generation.cu
  optix/basic/generationGPU.cu
  optix/basic/triangle.cu
  optix/basic/receiver.cu
  optix/polarization/generation.cu
  optix/polarization/generationGPU.cu
  optix/polarization/receiver.cu
  optix/polarization/triangle.cu
  optix/polarization/linearPolarizationFunctions.h
  optix/polarization/curved/curved.cu
  optix/polarization/curved/triangle.cu
  optix/polarization/curved/receiver.cu
  optix/polarization/curved/generation.cu
  optix/polarization/curved/generationGPU.cu
  optix/polarization/rdn/curved.cu
  optix/polarization/rdn/receiver.cu
  optix/polarization/rdn/triangle.cu
  optix/polarization/rdn/generation.cu
  optix/polarization/rdn/generationGPU.cu
  optix/log/generation.cu
  optix/log/triangle.cu
  optix/log/receiver.cu
  optix/log/curved.cu
  optix/Complex.h
  optix/penetrationFunctions.h
  optix/traceFunctions.h
  optix/curvedFunctions.h
  optix/receiverFunctions.h
  optix/triangleFunctions.h
  optix/reflectionFunctions.h
  )

target_link_libraries( opal
     ${CUDA_LIBRARIES}
    ${CUDA_curand_LIBRARY}
  optix
  optixu
  sutil_sdk
  #mlpack
  )
set(sources
  json.hpp
  Opal.cpp
  Opal.h
  Common.h
  Config.h
  util.cpp
  util.h
  #Complex.h
  Unity.h
  Unity.cpp
  main.cpp
  #traceFunctions.h
  #penetrationFunctions.h
  transmitterManager.h
  transmitterManager.cpp
  opalSimulation.h
  opalSimulation.cpp
  basicSimulation.h
  basicSimulation.cpp
  flatSimulation.h
  flatSimulation.cpp
  curvedMeshSimulation.h
  curvedMeshSimulation.cpp
  curvedFlatMeshSimulation.h
  curvedFlatMeshSimulation.cpp
  rayDensityNormalizationSimulation.h
  rayDensityNormalizationSimulation.cpp
  tutils.cu
  tutils.h
  timer.h
  timer.cpp
  tests/rdn.h
  tests/rdn.cpp
  tests/tests.h
  tests/tests.cpp
  tests/tunnels.cpp
  tests/tunnels.h
  tests/depolarization.h
  tests/depolarization.cpp
  tests/curvature.h
  tests/curvature.cpp
  tests/tunnelsBase.h
  tests/tunnelsBase.cpp
  tests/dudley.h
  tests/dudley.cpp
  tests/diffraction.h
  tests/diffraction.cpp
  tests/antennaGain.h
  tests/antennaGain.cpp
  ptxUtil.h
  ptxUtil.cpp
  raySphere.h
  raySphere.cpp
  singleDiffraction.h
  singleDiffraction.cpp
  json.hpp
  #To see it properly in Visual Studio we should add the following. 
  optix/intersect/sphere.cu
  optix/intersect/optixGeometryTriangles.cu
  optix/intersect/optixTriangle.cu
  optix/intersect/optixGeometryTrianglesCurved.cu
  optix/intersect/optixTriangleCurved.cu
  optix/intersect/exception.cu
  optix/basic/generation.cu
  optix/basic/generationGPU.cu
  optix/basic/triangle.cu
  optix/basic/receiver.cu
  optix/polarization/generation.cu
  optix/polarization/generationGPU.cu
  optix/polarization/receiver.cu
  optix/polarization/triangle.cu
  optix/polarization/linearPolarizationFunctions.h
  optix/polarization/curved/curved.cu
  optix/polarization/curved/triangle.cu
  optix/polarization/curved/receiver.cu
  optix/polarization/curved/generation.cu
  optix/polarization/curved/generationGPU.cu
  optix/polarization/rdn/curved.cu
  optix/polarization/rdn/receiver.cu
  optix/polarization/rdn/triangle.cu
  optix/polarization/rdn/generation.cu
  optix/polarization/rdn/generationGPU.cu
  optix/log/generation.cu
  optix/log/triangle.cu
  optix/log/receiver.cu
  optix/log/curved.cu
  optix/Complex.h
  optix/penetrationFunctions.h
  optix/traceFunctions.h
  optix/curvedFunctions.h
  optix/receiverFunctions.h
  optix/triangleFunctions.h
  optix/reflectionFunctions.h
)
OPTIX_add_sample_library( opal_s 
  Opal.cpp
  Opal.h
  Common.h
  Config.h
  util.cpp
  util.h
  Unity.h
  Unity.cpp
  main.cpp
  transmitterManager.h
  transmitterManager.cpp
  opalSimulation.h
  opalSimulation.cpp
  basicSimulation.h
  basicSimulation.cpp
  flatSimulation.h
  flatSimulation.cpp
  curvedMeshSimulation.h
  curvedMeshSimulation.cpp
  curvedFlatMeshSimulation.h
  curvedFlatMeshSimulation.cpp
  rayDensityNormalizationSimulation.h
  rayDensityNormalizationSimulation.cpp
  tutils.cu
  tutils.h
  timer.h
  timer.cpp
  tests/rdn.h
  tests/rdn.cpp
  tests/tests.h
  tests/tests.cpp
  tests/tunnels.cpp
  tests/tunnels.h
  tests/depolarization.h
  tests/depolarization.cpp
  tests/curvature.h
  tests/curvature.cpp
  tests/tunnelsBase.h
  tests/tunnelsBase.cpp
  tests/dudley.h
  tests/dudley.cpp
  tests/diffraction.h
  tests/diffraction.cpp
  tests/antennaGain.h
  tests/antennaGain.cpp
  ptxUtil.h
  ptxUtil.cpp
  raySphere.h
  raySphere.cpp
  singleDiffraction.h
  singleDiffraction.cpp
  json.hpp
  #dkm.h 
  #To see it properly in Visual Studio we should add the following. 
  optix/intersect/sphere.cu
  optix/intersect/optixGeometryTriangles.cu
  optix/intersect/optixTriangle.cu
  optix/intersect/optixGeometryTrianglesCurved.cu
  optix/intersect/optixTriangleCurved.cu
  optix/intersect/exception.cu
  optix/basic/generation.cu
  optix/basic/generationGPU.cu
  optix/basic/triangle.cu
  optix/basic/receiver.cu
  optix/polarization/generation.cu
  optix/polarization/generationGPU.cu
  optix/polarization/receiver.cu
  optix/polarization/triangle.cu
  optix/polarization/linearPolarizationFunctions.h
  optix/polarization/curved/curved.cu
  optix/polarization/curved/triangle.cu
  optix/polarization/curved/receiver.cu
  optix/polarization/curved/generation.cu
  optix/polarization/curved/generationGPU.cu
  optix/polarization/rdn/curved.cu
  optix/polarization/rdn/receiver.cu
  optix/polarization/rdn/triangle.cu
  optix/polarization/rdn/generation.cu
  optix/polarization/rdn/generationGPU.cu
  optix/log/generation.cu
  optix/log/triangle.cu
  optix/log/receiver.cu
  optix/log/curved.cu
  optix/Complex.h
  optix/penetrationFunctions.h
  optix/traceFunctions.h
  optix/curvedFunctions.h
  optix/receiverFunctions.h
  optix/triangleFunctions.h
  optix/reflectionFunctions.h
  )

target_link_libraries( opal_s 
     ${CUDA_LIBRARIES}
    ${CUDA_curand_LIBRARY}
  optix
  optixu
  sutil_sdk
  )
set(sources
  Opal.cpp
  Opal.h
  Common.h
  Config.h
  util.cpp
  util.h
  #Complex.h
  Unity.h
  Unity.cpp
  main.cpp
  #traceFunctions.h
  #penetrationFunctions.h
  transmitterManager.h
  transmitterManager.cpp
  opalSimulation.h
  opalSimulation.cpp
  basicSimulation.h
  basicSimulation.cpp
  flatSimulation.h
  flatSimulation.cpp
  curvedMeshSimulation.h
  curvedMeshSimulation.cpp
  curvedFlatMeshSimulation.h
  curvedFlatMeshSimulation.cpp
  rayDensityNormalizationSimulation.h
  rayDensityNormalizationSimulation.cpp
  tutils.cu
  tutils.h
  timer.h
  timer.cpp
  tests/rdn.h
  tests/rdn.cpp
  tests/tests.h
  tests/tests.cpp
  tests/tunnels.cpp
  tests/tunnels.h
  tests/depolarization.h
  tests/depolarization.cpp
  tests/curvature.h
  tests/curvature.cpp
  tests/tunnelsBase.h
  tests/tunnelsBase.cpp
  tests/dudley.h
  tests/dudley.cpp
  tests/diffraction.h
  tests/diffraction.cpp
  tests/antennaGain.h
  tests/antennaGain.cpp
  ptxUtil.h
  ptxUtil.cpp
  raySphere.h
  raySphere.cpp
  singleDiffraction.h
  singleDiffraction.cpp
  json.hpp
  #To see it properly in Visual Studio we should add the following. 
  optix/intersect/sphere.cu
  optix/intersect/optixGeometryTriangles.cu
  optix/intersect/optixTriangle.cu
  optix/intersect/optixGeometryTrianglesCurved.cu
  optix/intersect/optixTriangleCurved.cu
  optix/intersect/exception.cu
  optix/basic/generation.cu
  optix/basic/generationGPU.cu
  optix/basic/triangle.cu
  optix/basic/receiver.cu
  optix/polarization/generation.cu
  optix/polarization/generationGPU.cu
  optix/polarization/receiver.cu
  optix/polarization/triangle.cu
  optix/polarization/linearPolarizationFunctions.h
  optix/polarization/curved/curved.cu
  optix/polarization/curved/triangle.cu
  optix/polarization/curved/receiver.cu
  optix/polarization/curved/generation.cu
  optix/polarization/curved/generationGPU.cu
  optix/polarization/rdn/curved.cu
  optix/polarization/rdn/receiver.cu
  optix/polarization/rdn/triangle.cu
  optix/polarization/rdn/generation.cu
  optix/polarization/rdn/generationGPU.cu
  optix/log/generation.cu
  optix/log/triangle.cu
  optix/log/receiver.cu
  optix/log/curved.cu
  optix/Complex.h
  optix/penetrationFunctions.h
  optix/traceFunctions.h
  optix/curvedFunctions.h
  optix/receiverFunctions.h
  optix/triangleFunctions.h
  optix/reflectionFunctions.h
)
