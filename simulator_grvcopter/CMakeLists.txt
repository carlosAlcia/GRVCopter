cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

# Find Gazebo
find_package(gazebo REQUIRED)
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

# Build our plugin
add_library(simulator_grvcopter SHARED simulator_grvcopter.cc)
target_link_libraries(simulator_grvcopter ${GAZEBO_LIBRARIES})

add_executable(sender sender.cc)

target_sources(simulator_grvcopter PRIVATE Messages_GRVCopter.cpp)
if (${gazebo_VERSION_MAJOR} LESS 6)
  # These two
  include(FindBoost)
  find_package(Boost ${MIN_BOOST_VERSION} REQUIRED system filesystem regex)
  target_link_libraries(sender ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES})
else()
  target_link_libraries(sender ${GAZEBO_LIBRARIES})
endif()
