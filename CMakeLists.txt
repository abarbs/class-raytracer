cmake_minimum_required (VERSION 2.8)
project (RAYTRACER)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake")

include_directories(${RAYTRACER_SOURCE_DIR}/src)

if(CMAKE_COMPILER_IS_GNUCXX)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -msse2")
  set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -DNDEBUG -DBOOST_DISABLE_ASSERTS -Ofast")
  set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG}")
endif(CMAKE_COMPILER_IS_GNUCXX)

if (MSVC)
  add_definitions(/arch:SSE2)
  set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} /DNDEBUG")
endif(MSVC)

find_package(Boost COMPONENTS unit_test_framework regex REQUIRED)
if(NOT Boost_FOUND)
  message(FATAL_ERROR "Please point the environment variable CMAKE_PREFIX_PATH to where it can find your Boost installation. (e.g. /path/to/boost_1_53_0)")
endif(NOT Boost_FOUND)
include_directories(${Boost_INCLUDE_DIR})
message(STATUS ${Boost_INCLUDE_DIR})

set(EIGEN3_INCLUDE_DIR $ENV{EIGEN3_INCLUDE_DIR})
message(STATUS ${EIGEN3_INCLUDE_DIR})
if( NOT EIGEN3_INCLUDE_DIR )
    message( FATAL_ERROR "Please point the environment variable EIGEN3_INCLUDE_DIR to the include directory of your Eigen3 installation.")
endif(NOT EIGEN3_INCLUDE_DIR)
include_directories (${EIGEN3_INCLUDE_DIR})

find_package(TBB)
if (NOT TBB_FOUND)
  message(FATAL_ERROR "TBB not found, please install TBB")
endif()
include_directories(${TBB_INCLUDE_DIRS})
link_directories(${TBB_LIBRARY_DIRS})

add_executable(rayTracer src/rayTracer.cpp)
target_link_libraries(rayTracer ${Boost_LIBRARIES} ${TBB_LIBRARIES})

add_executable(test test/tests.cpp src/rayTracer.cpp)
set_target_properties(test PROPERTIES COMPILE_DEFINITIONS "_UNIT_TESTS")
target_link_libraries(test ${Boost_LIBRARIES} ${TBB_LIBRARIES})
