set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)

add_executable(param_readtest param_readtest.cpp )
target_link_libraries(param_readtest ${PROJECT_NAME} )

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)

add_executable(visotest1 visotest1.cpp )
target_link_libraries(visotest1 ${PROJECT_NAME} )
