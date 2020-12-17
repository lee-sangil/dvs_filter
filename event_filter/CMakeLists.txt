# Project
cmake_minimum_required(VERSION 2.8.3)
set(PROJECT event_filter)
project(${PROJECT})

set(DEBUG false)

find_package(catkin_simple REQUIRED)
catkin_simple()

# For gprof
if(${DEBUG})
	add_compile_options(-pg)
	set(catkin_LIBRARIES ${catkin_LIBRARIES} -pg)
	include_directories(${catkin_INCLUDE_DIRS})
endif()

# Set build flags
if(${DEBUG})
  set(CMAKE_BUILD_TYPE DEBUG) # Debug, RelWithDebInfo
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -pedantic -Wall -O0 -lcaer")
else()
  set(CMAKE_BUILD_TYPE RELEASE)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -march=native -lcaer -pedantic -O2 -fopenmp -msse2 -mavx")
endif()
message(STATUS "Build type: " ${CMAKE_BUILD_TYPE})

find_package(OpenCV REQUIRED)

# Set include/library paths for successive compilation.
include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
)

# Search for all source files
file(GLOB sources_files RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}/src" "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp")
list(TRANSFORM sources_files PREPEND "src/")

# Make the executable
cs_add_executable(${PROJECT}
  ${sources_files}
)

# Link the executable to the necesarry libs
target_link_libraries(${PROJECT}
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)
