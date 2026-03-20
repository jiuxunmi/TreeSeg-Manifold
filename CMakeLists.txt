cmake_minimum_required(VERSION 3.10)
project(TreeSegManifold)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(PCL 1.10 REQUIRED)
find_package(Armadillo 9.8 REQUIRED)
find_package(Boost REQUIRED COMPONENTS system filesystem)

include_directories(
    ${PCL_INCLUDE_DIRS} 
    ${ARMADILLO_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
    ${CMAKE_SOURCE_DIR}/include
)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_library(treeseg SHARED ${CMAKE_SOURCE_DIR}/src/treeseg.cpp)
target_link_libraries(treeseg ${PCL_LIBRARIES} ${ARMADILLO_LIBRARIES} ${Boost_LIBRARIES})

add_executable(downsample ${CMAKE_SOURCE_DIR}/src/downsample.cpp)
target_link_libraries(downsample treeseg ${PCL_LIBRARIES})

add_executable(getdtmslice ${CMAKE_SOURCE_DIR}/src/getdtmslice.cpp)
target_link_libraries(getdtmslice treeseg ${PCL_LIBRARIES})

add_executable(findstems ${CMAKE_SOURCE_DIR}/src/findstems.cpp)
target_link_libraries(findstems treeseg ${PCL_LIBRARIES})

add_executable(segmentstem ${CMAKE_SOURCE_DIR}/src/segmentstem.cpp)
target_link_libraries(segmentstem treeseg ${PCL_LIBRARIES})

add_executable(step1_supervoxelizer ${CMAKE_SOURCE_DIR}/src/step1_supervoxelizer.cpp)
target_link_libraries(step1_supervoxelizer treeseg ${PCL_LIBRARIES})

add_executable(step2_graph_grower ${CMAKE_SOURCE_DIR}/src/step2_graph_grower.cpp)
target_link_libraries(step2_graph_grower treeseg ${PCL_LIBRARIES})