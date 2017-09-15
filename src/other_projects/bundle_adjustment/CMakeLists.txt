cmake_minimum_required(VERSION 2.8)

project(bundle_adjustment)

find_package(Ceres REQUIRED)
find_package(OpenCV REQUIRED)


include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
set(CMAKE_BUILD_TYPE "Release") 
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O3")

include_directories("/usr/include/eigen3" )
include_directories(${CERES_INCLUDE_DIRS} )

include_directories(${OpenCV_INCLUDE_DIRS} )



add_executable(${PROJECT_NAME} bundle_adjustment.cpp)

target_link_libraries(${PROJECT_NAME}  ${CERES_LIBRARIES}  ${OpenCV_LIBS})
