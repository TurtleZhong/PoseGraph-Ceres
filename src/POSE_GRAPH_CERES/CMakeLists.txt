cmake_minimum_required(VERSION 2.8)
project(POSE_GRAPH_CERES)

IF(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE Release)
ENDIF()

MESSAGE("Build type: " ${CMAKE_BUILD_TYPE})

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall  -O3 -march=native ")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall   -O3 -march=native")

# Check C++11 or C++0x support
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
   add_definitions(-DCOMPILEDWITHC11)
   message(STATUS "Using flag -std=c++11.")
elseif(COMPILER_SUPPORTS_CXX0X)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
   add_definitions(-DCOMPILEDWITHC0X)
   message(STATUS "Using flag -std=c++0x.")
else()
   message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)



find_package(OpenCV 3.0 QUIET)
if(NOT OpenCV_FOUND)
   find_package(OpenCV 2.4.3 QUIET)
   if(NOT OpenCV_FOUND)
      message(FATAL_ERROR "OpenCV > 2.4.3 not found.")
   endif()
endif()

find_package(Eigen3 3.1.0 REQUIRED)
find_package(Pangolin REQUIRED)
# Sophus
find_package( Sophus REQUIRED )
include_directories( ${Sophus_INCLUDE_DIRS} )
# ceres
find_package(Ceres REQUIRED)
include_directories(${CERES_INCLUDE_DIRS})

# g2o
SET( G2O_ROOT /usr/local/include/g2o )
FIND_PACKAGE( G2O REQUIRED )
# CSparse
FIND_PACKAGE( CSparse REQUIRED )
INCLUDE_DIRECTORIES( ${G2O_INCLUDE_DIR} ${CSPARSE_INCLUDE_DIR} )

include_directories(
${PROJECT_SOURCE_DIR}
${PROJECT_SOURCE_DIR}/include
${PROJECT_SOURCE_DIR}/test
${EIGEN3_INCLUDE_DIR}
)

#ADD_SUBDIRECTORY( ${PROJECT_SOURCE_DIR}/test )

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)

add_library(${PROJECT_NAME} SHARED
    src/config.cc
    src/converter.cc
    src/GroundTruth.cc
    src/camera.cc
    src/Frame.cc
    src/MapPoint.cc
    src/ORBextractor.cc
    src/ORBmatcher.cc
    src/SequenceRun.cc

)

target_link_libraries(${PROJECT_NAME}
${OpenCV_LIBS}
${Sophus_LIBRARIES}
${EIGEN3_LIBS}
${CERES_LIBRARIES}
)

# Build examples

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)

add_executable(param_reader test/param_reader.cpp )
target_link_libraries(param_reader ${PROJECT_NAME} g2o_core g2o_types_slam3d)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)

add_executable(frame_test test/frame_test.cpp )
target_link_libraries(frame_test ${PROJECT_NAME} g2o_core g2o_types_slam3d)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)

add_executable(pose_graph_try1 test/pose_graph_try1.cpp )
target_link_libraries(pose_graph_try1 ${PROJECT_NAME} g2o_core g2o_types_slam3d g2o_solver_csparse g2o_stuff g2o_csparse_extension ${CSPARSE_LIBRARY})

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)

add_executable(feature_match test/feature_match.cpp )
target_link_libraries(feature_match ${PROJECT_NAME} g2o_core g2o_types_slam3d)

add_executable(feature_match2 test/feature_match2.cpp )
target_link_libraries(feature_match2 ${PROJECT_NAME} g2o_core g2o_types_slam3d)

add_executable(pose_graph_try2 test/pose_graph_try2.cpp )
target_link_libraries(pose_graph_try2 ${PROJECT_NAME} g2o_core g2o_types_slam3d g2o_solver_csparse g2o_stuff g2o_csparse_extension ${CSPARSE_LIBRARY})

add_executable(pose_graph_try3 test/pose_graph_try3.cpp )
target_link_libraries(pose_graph_try3 ${PROJECT_NAME} g2o_core g2o_types_slam3d g2o_solver_csparse g2o_stuff g2o_csparse_extension ${CSPARSE_LIBRARY})

add_executable(match_test test/match_test.cpp )
target_link_libraries(match_test ${PROJECT_NAME})

add_executable(match_test2 test/match_test2.cpp )
target_link_libraries(match_test2 ${PROJECT_NAME})

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)

add_executable(pose_graph_ceres test/pose_graph_ceres.cpp )
target_link_libraries(pose_graph_ceres ${PROJECT_NAME} g2o_core g2o_types_slam3d)

