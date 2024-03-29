cmake_minimum_required(VERSION 3.5 FATAL_ERROR)

project(surface)

include_directories(include)
find_package(PCL 1.12 REQUIRED PATHS usr/local/lib)
find_package(OpenMP REQUIRED)
find_package(MPI REQUIRED)
FIND_PACKAGE( Boost REQUIRED )
INCLUDE_DIRECTORIES( ${Boost_INCLUDE_DIR} )
if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    set (CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()
message(STATUS "Finding partA")
include_directories(${PCL_INCLUDE_DIRS})
include_directories(SYSTEM ${MPI_INCLUDE_PATH})

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (surface src/main.cpp src/mls_mpi.cpp)
target_link_libraries (surface ${PCL_LIBRARIES})
target_link_libraries(surface ${MPI_LIBRARIES})
TARGET_LINK_LIBRARIES(surface LINK_PUBLIC ${Boost_LIBRARIES} )

install(TARGETS surface
  DESTINATION build)
