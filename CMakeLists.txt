cmake_minimum_required(VERSION 3.25)
project(embree3Pathtracing3)

set(CMAKE_CXX_STANDARD 17)

# Add the include directory for Embree
message("$ENV{EMBREE_INCLUDE_PATH3}")
include_directories($ENV{EMBREE_INCLUDE_PATH3})


add_executable(embree3Pathtracing3 main.cpp Color.h)

find_package(Eigen3 CONFIG REQUIRED)
target_link_libraries(${PROJECT_NAME}  Eigen3::Eigen)

find_package(embree 3.0 REQUIRED)
target_link_libraries(${PROJECT_NAME} embree)

#for OpenMP
find_package(OpenMP)
if(OPENMP_CXX_FOUND)
    # add flags for OpenMP
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} ${OpenMP_SHARED_LINKER_FLAGS}")
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()