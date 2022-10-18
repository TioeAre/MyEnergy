cmake_minimum_required(VERSION 3.20)
project(MyEnergy)

set(CMAKE_CXX_STANDARD 17)
include_directories(.)
include_directories(/usr/include)
include_directories(${CMAKE_SOURCE_DIR}/include)
aux_source_directory(${CMAKE_SOURCE_DIR} DIR)
aux_source_directory(${CMAKE_SOURCE_DIR}/src SRC)

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
link_directories(${OpenCV_LIBRARY_DIRS})

find_package(Ceres 2 REQUIRED)
include_directories(${CERES_INCLUDE_DIRS})
link_directories(${CERES_LIBRARY_DIRS})

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIRS})

add_executable(MyEnergy ${DIR} ${SRC})

target_link_libraries(MyEnergy ${OpenCV_LIBS} ${CERES_LIBRARIES})