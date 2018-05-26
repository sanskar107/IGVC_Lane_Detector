cmake_minimum_required(VERSION 3.1)
project(lane_detector)

set (CMAKE_CXX_STANDARD 11)


find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport 
  roscpp
  std_msgs
  rospy
  tf
  message_filters
)

find_package(OpenCV REQUIRED)
find_package(CUDA REQUIRED)


# catkin_python_setup()

catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  include/lane_detector/
  ${CUDA_INCLUDE_DIRS}
  "src/library/"
)
# add_subdirectory(src/ORUtils)

set(GSLICR_LIB
src/gSLICr_Lib/engines/gSLICr_core_engine.h
src/gSLICr_Lib/engines/gSLICr_seg_engine.h
src/gSLICr_Lib/engines/gSLICr_seg_engine_GPU.h
src/gSLICr_Lib/engines/gSLICr_seg_engine_shared.h
src/gSLICr_Lib/engines/gSLICr_core_engine.cpp
src/gSLICr_Lib/engines/gSLICr_seg_engine.cpp
src/gSLICr_Lib/engines/gSLICr_seg_engine_GPU.cu
src/gSLICr_Lib/objects/gSLICr_settings.h
src/gSLICr_Lib/objects/gSLICr_spixel_info.h
src/gSLICr_Lib/gSLICr_defines.h
src/gSLICr_Lib/gSLICr.h
src/library/gSLIC/FastImgSeg.h
src/library/gSLIC/FastImgSeg.cpp
src/library/gSLIC/cudaDefines.h
src/library/gSLIC/cudaImgTrans.cu
src/library/gSLIC/cudaImgTrans.h
src/library/gSLIC/cudaSegEngine.cu
src/library/gSLIC/cudaSegEngine.h
src/library/gSLIC/cudaSegSLIC.cu
src/library/gSLIC/cudaSegSLIC.h
src/library/gSLIC/cudaUtil.cu
src/library/gSLIC/cudaUtil.h
)

list(APPEND "-std=c++11 -ftree-vectorize")
SOURCE_GROUP(engines FILES ${GSLICR_LIB})

cuda_add_library(gSLICr_lib
      ${GSLICR_LIB}
      src/NVTimer.h
      OPTIONS -gencode arch=compute_30,code=compute_30)

target_link_libraries(gSLICr_lib ${CUDA_LIBRARY})

add_executable(lanes src/lanes_chalne_wala.cpp)
add_executable(lanes_sid src/lanes_chalne_wala_siddhant.cpp)

add_library(svm include/lane_detector/svm.cpp)

target_link_libraries(lanes
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  svm
  gSLICr_lib
)

target_link_libraries(lanes_sid
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  svm
  gSLICr_lib
)