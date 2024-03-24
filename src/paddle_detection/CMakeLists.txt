cmake_minimum_required(VERSION 3.0.2)
project(paddle_detection)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
  image_transport
  geometry_msgs
)

option(WITH_MKL ON)
option(WITH_GPU OFF)
option(WITH_TENSORRT OFF)
option(WITH_KEYPOINT OFF)
option(WITH_MOT OFF)

SET(PADDLE_DIR "/home/lhy/paddle_inference")
SET(PADDLE_LIB_NAME "libpaddle_inference")
SET(OPENCV_DIR "")
SET(CUDA_LIB "")
SET(CUDNN_LIB "")
SET(TENSORRT_INC_DIR "/usr/include/x86_64-linux-gnu")
SET(TENSORRT_LIB_DIR "/usr/lib/x86_64-linux-gnu")

include(cmake/yaml-cpp.cmake)

include_directories("${CMAKE_SOURCE_DIR}/")
include_directories("${CMAKE_CURRENT_BINARY_DIR}/ext/yaml-cpp/src/ext-yaml-cpp/include")
link_directories("${CMAKE_CURRENT_BINARY_DIR}/ext/yaml-cpp/lib")

if (WITH_KEYPOINT)
    set(SRCS src/main_keypoint.cc src/preprocess_op.cc src/object_detector.cc src/picodet_postprocess.cc src/utils.cc src/keypoint_detector.cc src/keypoint_postprocess.cc)
elseif (WITH_MOT)
    set(SRCS src/main_jde.cc src/preprocess_op.cc src/object_detector.cc src/jde_detector.cc src/tracker.cc src/trajectory.cc src/lapjv.cpp src/picodet_postprocess.cc src/utils.cc)
else ()
    set(SRCS src/main.cc src/preprocess_op.cc src/object_detector.cc src/picodet_postprocess.cc src/utils.cc)
endif()


macro(safe_set_static_flag)
    foreach(flag_var
        CMAKE_CXX_FLAGS CMAKE_CXX_FLAGS_DEBUG CMAKE_CXX_FLAGS_RELEASE
        CMAKE_CXX_FLAGS_MINSIZEREL CMAKE_CXX_FLAGS_RELWITHDEBINFO)
      if(${flag_var} MATCHES "/MD")
        string(REGEX REPLACE "/MD" "/MT" ${flag_var} "${${flag_var}}")
      endif(${flag_var} MATCHES "/MD")
    endforeach(flag_var)
endmacro()

if (WITH_MKL)
    ADD_DEFINITIONS(-DUSE_MKL)
endif()

if (NOT DEFINED PADDLE_DIR OR ${PADDLE_DIR} STREQUAL "")
    message(FATAL_ERROR "please set PADDLE_DIR with -DPADDLE_DIR=/path/paddle_influence_dir")
endif()
message("PADDLE_DIR IS:" ${PADDLE_DIR})

#if (NOT DEFINED OPENCV_DIR OR ${OPENCV_DIR} STREQUAL "")
#    message(FATAL_ERROR "please set OPENCV_DIR with -DOPENCV_DIR=/path/opencv")
#endif()

include_directories("${CMAKE_SOURCE_DIR}/")
include_directories("${PADDLE_DIR}/")
include_directories("${PADDLE_DIR}/third_party/install/protobuf/include")
include_directories("${PADDLE_DIR}/third_party/install/glog/include")
include_directories("${PADDLE_DIR}/third_party/install/gflags/include")
include_directories("${PADDLE_DIR}/third_party/install/xxhash/include")
if (EXISTS "${PADDLE_DIR}/third_party/install/snappy/include")
    include_directories("${PADDLE_DIR}/third_party/install/snappy/include")
endif()
if(EXISTS "${PADDLE_DIR}/third_party/install/snappystream/include")
    include_directories("${PADDLE_DIR}/third_party/install/snappystream/include")
endif()
include_directories("${PADDLE_DIR}/third_party/boost")
include_directories("${PADDLE_DIR}/third_party/eigen3")

if (EXISTS "${PADDLE_DIR}/third_party/install/snappy/lib")
    link_directories("${PADDLE_DIR}/third_party/install/snappy/lib")
endif()
if(EXISTS "${PADDLE_DIR}/third_party/install/snappystream/lib")
    link_directories("${PADDLE_DIR}/third_party/install/snappystream/lib")
endif()

link_directories("${PADDLE_DIR}/third_party/install/protobuf/lib")
link_directories("${PADDLE_DIR}/third_party/install/glog/lib")
link_directories("${PADDLE_DIR}/third_party/install/gflags/lib")
link_directories("${PADDLE_DIR}/third_party/install/xxhash/lib")
link_directories("${PADDLE_DIR}/third_party/install/paddle2onnx/lib")
link_directories("${PADDLE_DIR}/third_party/install/onnxruntime/lib")
link_directories("${PADDLE_DIR}/paddle/lib/")
link_directories("${CMAKE_CURRENT_BINARY_DIR}")



if (WIN32)
  include_directories("${PADDLE_DIR}/paddle/fluid/inference")
  include_directories("${PADDLE_DIR}/paddle/include")
  link_directories("${PADDLE_DIR}/paddle/fluid/inference")
  find_package(OpenCV REQUIRED PATHS ${OPENCV_DIR}/build/ NO_DEFAULT_PATH)

else ()
  find_package(OpenCV 4 REQUIRED)
  MESSAGE(STATUS "OpenCV version: "${OpenCV_VERSION})

  find_package(OpenCV REQUIRED PATHS ${OPENCV_DIR}/share/OpenCV NO_DEFAULT_PATH)
  include_directories("${PADDLE_DIR}/paddle/include")
  link_directories("${PADDLE_DIR}/paddle/lib")
endif ()
include_directories(${OpenCV_INCLUDE_DIRS})

if (WIN32)
    add_definitions("/DGOOGLE_GLOG_DLL_DECL=")
    set(CMAKE_C_FLAGS_DEBUG   "${CMAKE_C_FLAGS_DEBUG} /bigobj /MTd")
    set(CMAKE_C_FLAGS_RELEASE  "${CMAKE_C_FLAGS_RELEASE} /bigobj /MT")
    set(CMAKE_CXX_FLAGS_DEBUG  "${CMAKE_CXX_FLAGS_DEBUG} /bigobj /MTd")
    set(CMAKE_CXX_FLAGS_RELEASE   "${CMAKE_CXX_FLAGS_RELEASE} /bigobj /MT")
else()
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -o2 -fopenmp -std=c++11")
    set(CMAKE_STATIC_LIBRARY_PREFIX "")
endif()

# TODO let users define cuda lib path
if (WITH_GPU)
    if (NOT DEFINED CUDA_LIB OR ${CUDA_LIB} STREQUAL "")
        message(FATAL_ERROR "please set CUDA_LIB with -DCUDA_LIB=/path/cuda-8.0/lib64")
    endif()
    if (NOT WIN32)
        if (NOT DEFINED CUDNN_LIB)
            message(FATAL_ERROR "please set CUDNN_LIB with -DCUDNN_LIB=/path/cudnn_v7.4/cuda/lib64")
        endif()
    endif(NOT WIN32)
endif()


if (NOT WIN32)
  if (WITH_TENSORRT AND WITH_GPU)
	  include_directories("${TENSORRT_INC_DIR}/")
	  link_directories("${TENSORRT_LIB_DIR}/")
  endif()
endif(NOT WIN32)

if (NOT WIN32)
    set(NGRAPH_PATH "${PADDLE_DIR}/third_party/install/ngraph")
    if(EXISTS ${NGRAPH_PATH})
        include(GNUInstallDirs)
        include_directories("${NGRAPH_PATH}/include")
        link_directories("${NGRAPH_PATH}/${CMAKE_INSTALL_LIBDIR}")
        set(NGRAPH_LIB ${NGRAPH_PATH}/${CMAKE_INSTALL_LIBDIR}/libngraph${CMAKE_SHARED_LIBRARY_SUFFIX})
    endif()
endif()

if(WITH_MKL)
  include_directories("${PADDLE_DIR}/third_party/install/mklml/include")
  if (WIN32)
    set(MATH_LIB ${PADDLE_DIR}/third_party/install/mklml/lib/mklml.lib
            ${PADDLE_DIR}/third_party/install/mklml/lib/libiomp5md.lib)
  else ()
    set(MATH_LIB ${PADDLE_DIR}/third_party/install/mklml/lib/libmklml_intel${CMAKE_SHARED_LIBRARY_SUFFIX}
            ${PADDLE_DIR}/third_party/install/mklml/lib/libiomp5${CMAKE_SHARED_LIBRARY_SUFFIX})
    execute_process(COMMAND cp -r ${PADDLE_DIR}/third_party/install/mklml/lib/libmklml_intel${CMAKE_SHARED_LIBRARY_SUFFIX} /usr/lib)
  endif ()
  set(MKLDNN_PATH "${PADDLE_DIR}/third_party/install/mkldnn")
  if(EXISTS ${MKLDNN_PATH})
    include_directories("${MKLDNN_PATH}/include")
    if (WIN32)
      set(MKLDNN_LIB ${MKLDNN_PATH}/lib/mkldnn.lib)
    else ()
      set(MKLDNN_LIB ${MKLDNN_PATH}/lib/libmkldnn.so.0)
    endif ()
  endif()
else()
  set(MATH_LIB ${PADDLE_DIR}/third_party/install/openblas/lib/libopenblas${CMAKE_STATIC_LIBRARY_SUFFIX})
endif()


if (WIN32)
    if(EXISTS "${PADDLE_DIR}/paddle/fluid/inference/${PADDLE_LIB_NAME}${CMAKE_STATIC_LIBRARY_SUFFIX}")
        set(DEPS
            ${PADDLE_DIR}/paddle/fluid/inference/${PADDLE_LIB_NAME}${CMAKE_STATIC_LIBRARY_SUFFIX})
    else()
        set(DEPS
            ${PADDLE_DIR}/paddle/lib/${PADDLE_LIB_NAME}${CMAKE_STATIC_LIBRARY_SUFFIX})
    endif()
endif()


if (WIN32)
    set(DEPS ${PADDLE_DIR}/paddle/lib/${PADDLE_LIB_NAME}${CMAKE_STATIC_LIBRARY_SUFFIX})
else()
    set(DEPS ${PADDLE_DIR}/paddle/lib/${PADDLE_LIB_NAME}${CMAKE_SHARED_LIBRARY_SUFFIX})
endif()

message("PADDLE_LIB_NAME:" ${PADDLE_LIB_NAME})
message("DEPS:" $DEPS)

if (NOT WIN32)
    set(DEPS ${DEPS}
        ${MATH_LIB} ${MKLDNN_LIB}
        glog gflags protobuf z xxhash yaml-cpp
        )
    if(EXISTS "${PADDLE_DIR}/third_party/install/snappystream/lib")
        set(DEPS ${DEPS} snappystream)
    endif()
    if (EXISTS "${PADDLE_DIR}/third_party/install/snappy/lib")
        set(DEPS ${DEPS} snappy)
    endif()
else()
    set(DEPS ${DEPS}
        ${MATH_LIB} ${MKLDNN_LIB}
        glog gflags_static libprotobuf xxhash libyaml-cppmt)
    set(DEPS ${DEPS} libcmt shlwapi)
    if (EXISTS "${PADDLE_DIR}/third_party/install/snappy/lib")
        set(DEPS ${DEPS} snappy)
    endif()
    if(EXISTS "${PADDLE_DIR}/third_party/install/snappystream/lib")
        set(DEPS ${DEPS} snappystream)
    endif()
endif(NOT WIN32)

if(WITH_GPU)
  if(NOT WIN32)
    if (WITH_TENSORRT)
	    set(DEPS ${DEPS} ${TENSORRT_LIB_DIR}/libnvinfer${CMAKE_SHARED_LIBRARY_SUFFIX})
	    set(DEPS ${DEPS} ${TENSORRT_LIB_DIR}/libnvinfer_plugin${CMAKE_SHARED_LIBRARY_SUFFIX})
    endif()
    set(DEPS ${DEPS} ${CUDA_LIB}/libcudart${CMAKE_SHARED_LIBRARY_SUFFIX})
    set(DEPS ${DEPS} ${CUDNN_LIB}/libcudnn${CMAKE_SHARED_LIBRARY_SUFFIX})
  else()
    set(DEPS ${DEPS} ${CUDA_LIB}/cudart${CMAKE_STATIC_LIBRARY_SUFFIX} )
    set(DEPS ${DEPS} ${CUDA_LIB}/cublas${CMAKE_STATIC_LIBRARY_SUFFIX} )
    set(DEPS ${DEPS} ${CUDNN_LIB}/cudnn${CMAKE_STATIC_LIBRARY_SUFFIX})
  endif()
endif()

if (NOT WIN32)
    set(EXTERNAL_LIB "-ldl -lrt -lgomp -lz -lm -lpthread")
    set(DEPS ${DEPS} ${EXTERNAL_LIB})
endif()

set(DEPS ${DEPS} ${OpenCV_LIBS})


## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


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
##   * add a build_depend and a exec_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a exec_depend tag for "message_runtime"
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
add_message_files(
  FILES
  signal.msg
)

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
generate_messages(
  DEPENDENCIES
  std_msgs
)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
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

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES paddle_detection
  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${CMAKE_SOURCE_DIR}/paddle_detection/include
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/paddle_detection.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# add_executable(${PROJECT_NAME}_node src/paddle_detection_node.cpp)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )

add_executable(paddle_detection_main ${SRCS})
ADD_DEPENDENCIES(paddle_detection_main ext-yaml-cpp ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
message("DEPS:" $DEPS)
target_link_libraries(paddle_detection_main ${DEPS} ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# catkin_install_python(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
# install(TARGETS ${PROJECT_NAME}_node
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark libraries for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
# install(TARGETS ${PROJECT_NAME}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_paddle_detection.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
