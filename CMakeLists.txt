cmake_minimum_required(VERSION 3.5 FATAL_ERROR)

project(surface)

set(CMAKE_EXE_LINKER_FLAGS_INIT "-Wl,--copy-dt-needed-entries")
set (CMAKE_SHARED_LINKER_FLAGS_INIT "-Wl,--copy-dt-needed-entries")
set (CMAKE_MODULE_LINKER_FLAGS_INIT "-Wl,--copy-dt-needed-entries")

include_directories(include)
find_package(OpenMP REQUIRED)
find_package(MPI REQUIRED)
FIND_PACKAGE( Boost REQUIRED )

INCLUDE_DIRECTORIES( ${Boost_INCLUDE_DIR} )
INCLUDE_DIRECTORIES(/home/m/projects/source_code/pcl/surface/include/pcl/surface
        /home/m/projects/source_code/pcl/common/include
        /home/m/projects/source_code/pcl/build/include
        /home/m/Documents/CUSTOM_PCL/eigen
        /home/m/Documents/CUSTOM_PCL/pcl/io/include
        /home/m/projects/source_code/pcl/search/include
        /home/m/projects/source_code/pcl/kdtree/include
        /home/m/projects/source_code/pcl/octree/include
        /home/m/projects/source_code/pcl/surface/include
        /home/m/projects/source_code/pcl/visualization/include
        /home/m/projects/source_code/pcl/geometry/include
        /usr/include/vtk-7.1
        )
add_library(mls SHARED IMPORTED)
add_library(common SHARED IMPORTED)
add_library(kdtree SHARED IMPORTED)
add_library(pcl_search SHARED IMPORTED)
add_library(octree SHARED IMPORTED)
add_library(visualization SHARED IMPORTED)
add_library(geometry SHARED IMPORTED)
add_library(boost_filesystem SHARED IMPORTED)



set_property(TARGET mls PROPERTY IMPORTED_LOCATION "/home_edu/edu-cmc-sqi19/edu-cmc-sqi19-24/PCL/pcl/build/lib/libpcl_surface.so.1.9.1")
set_property(TARGET common PROPERTY IMPORTED_LOCATION "/home_edu/edu-cmc-sqi19/edu-cmc-sqi19-24/PCL/pcl/build/lib/libpcl_common.so.1.9.1")
set_property(TARGET kdtree PROPERTY IMPORTED_LOCATION "/home_edu/edu-cmc-sqi19/edu-cmc-sqi19-24/PCL/pcl/build/lib/libpcl_kdtree.so.1.9.1")
set_property(TARGET pcl_search PROPERTY IMPORTED_LOCATION "/home_edu/edu-cmc-sqi19/edu-cmc-sqi19-24/PCL/pcl/build/lib/libpcl_search.so.1.9.1")
set_property(TARGET octree PROPERTY IMPORTED_LOCATION "/home_edu/edu-cmc-sqi19/edu-cmc-sqi19-24/PCL/pcl/build/lib/libpcl_octree.so.1.9.1")
set_property(TARGET visualization PROPERTY IMPORTED_LOCATION "/home_edu/edu-cmc-sqi19/edu-cmc-sqi19-24/PCL/pcl/build/lib/libpcl_visualization.so.1.9.1")
set_property(TARGET geometry PROPERTY IMPORTED_LOCATION "/home_edu/edu-cmc-sqi19/edu-cmc-sqi19-24/PCL/pcl/build/lib/libpcl_visualization.so.1.9.1")
set_property(TARGET boost_filesystem PROPERTY IMPORTED_LOCATION "/usr/lib/x86_64-linux-gnu/libboost_filesystem.so.1.71.0")



if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    set (CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()

include_directories(SYSTEM ${MPI_INCLUDE_PATH})

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (surface src/main.cpp src/mls_mpi.cpp)
target_link_libraries (surface mls)
target_link_libraries (surface common)
target_link_libraries (surface kdtree)
target_link_libraries (surface pcl_search)
target_link_libraries (surface octree)
target_link_libraries (surface visualization)
target_link_libraries (surface boost_filesystem)


target_link_libraries(surface ${MPI_LIBRARIES})
TARGET_LINK_LIBRARIES(surface LINK_PUBLIC ${Boost_LIBRARIES} )

install(TARGETS surface
  DESTINATION build)
