cmake_minimum_required(VERSION 2.8.7)
project(object_depth_module)

## ZED package
find_package(ZED 1.2 REQUIRED)


## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  image_transport
  roscpp
  rosconsole
  rospy
  pcl_conversions
  std_msgs
)

SET(CUDA_USE_STATIC_CUDA_RUNTIME OFF)				# To get rid of -lopencv_dep_cudart error

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)

exec_program(uname ARGS -p OUTPUT_VARIABLE CMAKE_SYSTEM_NAME2)

## OpenCV package
SET(OCV_VERSION "2.4")
find_package(OpenCV ${OCV_VERSION} COMPONENTS core highgui imgproc features2d REQUIRED)

## CUDA package
SET(CUDA_VERSION "8.0")
find_package(CUDA ${CUDA_VERSION} REQUIRED)

## PCL package
find_package(PCL REQUIRED)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES opencv_project
  CATKIN_DEPENDS rosconsole
		opencv
		image_transport
		roscpp
		rospy
		std_msgs
		pcl_conversions
#  DEPENDS system_lib
)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs
# )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###########
## Build ##
###########

###############################################################################
# INCLUDES
## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${CUDA_INCLUDE_DIRS}
  ${ZED_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

link_directories(include)
link_directories(${ZED_LIBRARY_DIR})
link_directories(${CUDA_LIBRARY_DIRS})
link_directories(${OpenCV_LIBRARY_DIRS})
link_directories(${PCL_LIBRARY_DIRS})

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(opencv_project ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_definitions(-std=c++11)# -m64) #-Wall)

SET(LINK_LIBRARIES
  ${catkin_LIBRARIES}
  ${ZED_LIBRARIES}
  ${CUDA_LIBRARIES} ${CUDA_nppi_LIBRARY} ${CUDA_npps_LIBRARY} ${CUDA_CUBLAS_LIBRARIES} ${CUDA_curand_LIBRARY}
  ${OpenCV_LIBS}
  ${PCL_LIBRARIES})

## Declare a C++ library. Note: If you have any local cpp files, you can add here. But don't forget to link them later (Follow this marker ***)
#SET(ALL_SOURCES src/activation_layer.c src/activations.c src/art.c src/avgpool_layer.c
#		src/batchnorm_layer.c src/blas.c  src/box.c src/captcha.c src/cifar.c src/classifier.c
#		src/coco.c src/col2im.c  src/compare.c src/connected_layer.c  src/convolutional_layer.c
#		src/cost_layer.c src/crnn_layer.c src/crop_layer.c  src/cuda.c src/darknet.c src/data.c
#		src/deconvolutional_layer.c src/demo.c src/depth_computations.cpp src/detection_layer.c
#		src/detector.c src/dice.c src/dropout_layer.c  src/gemm.c src/go.c src/gru_layer.c
#		src/im2col.c src/image.c src/kernel_window src/layer.c src/list.c src/local_layer.c
#		src/matrix.c src/maxpool_layer.c  src/network.c  src/nightmare.c src/normalization_layer.c
#		src/option_list.c src/parser.c src/region_layer.c src/reorg_layer.c src/rnn.c
#		src/rnn_layer.c src/rnn_vid.c src/route_layer.c src/shortcut_layer.c src/softmax_layer.c src/super.c
#		src/swag.c src/tag.c src/tree.c src/utils.c src/voxel.c src/writing.c src/yolo.c src/detector.c
#)

SET(CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS} ; -gencode arch=compute_30,code=sm_30 -gencode arch=compute_35,code=sm_35 -gencode arch=compute_50,code=[sm_50,compute_50] -gencode arch=compute_52,code=[sm_52,compute_52])
SET(CUDA_VERBOSE_BUILD ON CACHE BOOL "nvcc verbose" FORCE)
SET(ALL_CUDA_SRC src/activation_kernels.cu src/avgpool_layer_kernels.cu src/blas_kernels.cu
		src/col2im_kernels.cu src/convolutional_kernels.cu src/crop_layer_kernels.cu
		src/dropout_layer_kernels.cu src/deconvolutional_kernels.cu src/network_kernels.cu
		src/im2col_kernels.cu src/maxpool_layer_kernels.cu
)

#CUDA_COMPILE(CUDA_COMPILE_CU_O SHARED ${ALL_CUDA_SRC})
#CUDA_COMPILE(activation_kernels_o src/activation_kernels.cu)
#CUDA_COMPILE(avgpool_layer_kernels_o src/avgpool_layer_kernels.cu)
#CUDA_COMPILE(blas_kernels_o src/blas_kernels.cu)
#CUDA_COMPILE(col2im_kernels_o src/col2im_kernels.cu)
#CUDA_COMPILE(convolutional_kernels_o src/convolutional_kernels.cu)
#CUDA_COMPILE(crop_layer_kernels_o src/crop_layer_kernels.cu)
#CUDA_COMPILE(deconvolutional_kernels_o src/deconvolutional_kernels.cu)
#CUDA_COMPILE(dropout_layer_kernels_o src/dropout_layer_kernels.cu)
#CUDA_COMPILE(im2col_kernels_o src/im2col_kernels.cu)
#CUDA_COMPILE(maxpool_layer_kernels_o src/maxpool_layer_kernels.cu)
#CUDA_COMPILE(network_kernels_o src/network_kernels.cu)


#SET(LIB_TYPE MODULE)
#CUDA_ADD_LIBRARY(OBJECT_CUDA_LIB ${activation_kernels_o} ${avgpool_layer_kernels_o}
#		${blas_kernels_o} ${col2im_kernels_o} ${convolutional_kernels_o}
#		${crop_layer_kernels_o} ${deconvolutional_kernels_o} ${dropout_layer_kernels_o}
#		${im2col_kernels_o} ${maxpool_layer_kernels_o} ${network_kernels_o}
#		${LIB_TYPE}
#)
#target_link_libraries(OBJECT_CUDA_LIB ${LINK_LIBRARIES})

#add_library(object_depth_module )

## Declare a C++ executable
add_executable(object_depth_module_node src/object_depth_module_node.cpp
		src/object_depth_interface.cpp src/depth_computations.cpp
		src/object_detection.cpp src/activation_layer.cpp src/activations.cpp src/art.cpp src/avgpool_layer.cpp
		src/batchnorm_layer.cpp src/blas.cpp  src/box.cpp src/captcha.cpp src/cifar.cpp src/classifier.cpp
		src/coco.cpp src/col2im.cpp  src/compare.cpp src/connected_layer.cpp  src/convolutional_layer.cpp
		src/cost_layer.cpp src/crnn_layer.cpp src/crop_layer.cpp  src/cuda.cpp src/darknet.cpp src/data.cpp
		src/deconvolutional_layer.cpp src/demo.cpp src/detection_layer.cpp src/detector.cpp src/dice.cpp src/dropout_layer.cpp  src/gemm.cpp src/go.cpp src/gru_layer.cpp
		src/im2col.cpp src/image.cpp src/kernel_window src/layer.cpp src/list.cpp src/local_layer.cpp
		src/matrix.cpp src/maxpool_layer.cpp  src/network.cpp  src/nightmare.cpp src/normalization_layer.cpp
		src/option_list.cpp src/parser.cpp src/region_layer.cpp src/reorg_layer.cpp src/rnn.cpp
		src/rnn_layer.cpp src/rnn_vid.cpp src/route_layer.cpp src/shortcut_layer.cpp src/softmax_layer.cpp src/super.cpp
		src/swag.cpp src/tag.cpp src/tree.cpp src/utils.cpp src/voxel.cpp src/writing.cpp src/yolo.cpp src/detector.cpp
		src/activation_kernels.cu src/avgpool_layer_kernels.cu src/blas_kernels.cu
		src/col2im_kernels.cu src/convolutional_kernels.cu src/crop_layer_kernels.cu
		src/dropout_layer_kernels.cu src/deconvolutional_kernels.cu src/network_kernels.cu
		src/im2col_kernels.cu src/maxpool_layer_kernels.cu
)

## Add cmake target dependencies of the executable
## same as for the library above
#${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS}
#add_dependencies(object_depth_module_node ${LINE_PROFILER_CU_O})

## Specify libraries to link a library or executable target against
target_link_libraries(object_depth_module_node ${LINK_LIBRARIES})

#Add all files in subdirectories of the project in
# a dummy_target so qtcreator have access to all files
FILE(GLOB_RECURSE extra_files ${CMAKE_SOURCE_DIR}/*)
add_custom_target(dummy_${PROJECT_NAME} SOURCES ${extra_files})

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS opencv_project opencv_project_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_opencv_project.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
