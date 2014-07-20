cmake_minimum_required (VERSION 2.8)
project (RAYTRACER)

include_directories(${RAYTRACER_SOURCE_DIR})

if(CMAKE_COMPILER_IS_GNUCXX)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -msse2")
  set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -DNDEBUG -DBOOST_DISABLE_ASSERTS -Ofast")
  set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG}")
endif(CMAKE_COMPILER_IS_GNUCXX)

if (MSVC)
  add_definitions(/arch:SSE2)
  set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} /DNDEBUG")  
endif(MSVC)

find_package(OpenCV REQUIRED)

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

add_executable(rayTracer rayTracer.cpp)
target_link_libraries(rayTracer ${Boost_LIBRARIES} ${OpenCV_LIBS})

add_executable(test tests.cpp rayTracer.cpp)
set_target_properties(test PROPERTIES COMPILE_DEFINITIONS "_UNIT_TESTS")
target_link_libraries(test ${Boost_LIBRARIES} ${OpenCV_LIBS})


