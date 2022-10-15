cmake_minimum_required(VERSION 3.23)
project(MyEnergy)

set(CMAKE_CXX_STANDARD 17)
include_directories(${CMAKE_SOURCE_DIR}/)
include_directories(${CMAKE_SOURCE_DIR}/include/)
aux_source_directory(${CMAKE_SOURCE_DIR}/ DIR)
aux_source_directory(${CMAKE_SOURCE_DIR}/src SRC)

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
link_directories(${OpenCV_LIBRARY_DIRS})

add_executable(MyEnergy ${DIR} ${SRC} include/EnergyDetector.h src/EnergyDetector.cpp)

target_link_libraries(MyEnergy ${OpenCV_LIBS})