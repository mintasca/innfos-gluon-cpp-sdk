cmake_minimum_required(VERSION 3.0)

project(EXAMPLE)
 
link_directories(${CMAKE_CURRENT_SOURCE_DIR}/../sdk/sca_sdk/lib/linux_x86_64)
link_directories(${CMAKE_CURRENT_SOURCE_DIR}/../sdk/sdk/lib)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../sdk/sca_sdk/include)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../sdk/sdk/include)

add_compile_options(-std=c++11)
add_compile_options(-fPIC)
add_compile_options(-pthread)
add_compile_options(-DASIO_STANDALONE)
add_compile_options(-DASIO_HAS_STD_CHRONO)

SET(EXAMPLE_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/src)

LINK_LIBRARIES("${CMAKE_CURRENT_SOURCE_DIR}/../sdk/sca_sdk/lib/linux_x86_64/libActuatorController.so")


SET(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/bin/basic)
ADD_EXECUTABLE(startup ${EXAMPLE_SOURCES}/basic/startup.cpp)
target_link_libraries(startup kdl pthread ActuatorController sqlite3)
ADD_EXECUTABLE(shutdown ${EXAMPLE_SOURCES}/basic/shutdown.cpp)
target_link_libraries(shutdown kdl pthread ActuatorController sqlite3)

SET(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/bin/advanced)
ADD_EXECUTABLE(motion ${EXAMPLE_SOURCES}/advanced/motion.cpp)
target_link_libraries(motion kdl pthread ActuatorController sqlite3)


