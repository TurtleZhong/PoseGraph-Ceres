# OpenCV
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

# Sophus 
find_package( Sophus REQUIRED )
include_directories( ${Sophus_INCLUDE_DIRS} )

# Eigen
include_directories( "/usr/include/eigen3" )


# for test
add_library(libtest 
    config.cpp
    camera.cpp
    )
target_link_libraries(libtest ${OpenCV_LIBS} ${Sophus_LIBRARIES})

add_executable( vo0_3 vo0_3.cpp )
target_link_libraries( vo0_3 myslam )
