include(def_library)
include(export_package)

set(GSLIC_VERSION_MAJOR 1)
set(GSLIC_VERSION_MINOR 0)
set(GSLIC_VERSION ${GSLIC_VERSION_MAJOR}.${GSLIC_VERSION_MINOR})

option(BUILD_SHARED_LIBS "Build Shared Library" ON)

find_package(CUDA REQUIRED)

#################################################
# Append all includes.
list(APPEND GSLIC_INC_DIRS
    ${CMAKE_CURRENT_SOURCE_DIR}
   )

# Add include directories to library.
include_directories(${GSLIC_INC_DIRS})
include_directories(${GSLIC_INC_DIRS}/gSLIC)


#################################################
# Library headers and sources.
set(GSLIC_HDRS
    gSLIC/FastImgSeg.h
   )

set(GSLIC_SRCS
    gSLIC/FastImgSeg.cpp
   )


#################################################
# Define library.

set(CUDA_FILES gSLIC/cudaDefines.h
               gSLIC/cudaImgTrans.h gSLIC/cudaImgTrans.cu
               gSLIC/cudaSegEngine.h gSLIC/cudaSegEngine.cu
               gSLIC/cudaSegSLIC.h gSLIC/cudaSegSLIC.cu
               gSLIC/cudaUtil.h gSLIC/cudaUtil.cu)

set(CUDA_PROPAGATE_HOST_FLAGS OFF)
#set(CUDA_NVCC_FLAGS "-use_fast_math -gencode arch=compute_30,code=sm_30" )
set(CUDA_NVCC_FLAGS "${CUDA_NVCC_FLAGS} -arch=sm_30")
cuda_add_library(gslic_cuda ${CUDA_FILES})

install(TARGETS gslic_cuda
  EXPORT GSLIC
  RUNTIME DESTINATION ${CMAKE_INSTALL_PREFIX}/bin
  LIBRARY DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
  ARCHIVE DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
)


def_library(gslic
  SOURCES ${GSLIC_HDRS} ${GSLIC_SRCS}
  LINK_LIBS gslic_cuda
  PACKAGE GSLIC)


#################################################
# Export library to cmake.
export_package(GSLIC
  TARGETS gslic
  VERSION ${GSLIC_VERSION}
  LIBRARY gslic
  LIBRARIES gslic
  INCLUDE_DIRS ${GSLIC_INC_DIRS}
  )

