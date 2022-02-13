cmake_minimum_required(VERSION 3.5)
set(PROJECT_NAME as2_trajectory_generator)
project(${PROJECT_NAME})

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# set Release as default
if (NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

add_subdirectory(subpackages/dynamic_trajectory_generator)

# find dependencies
set(PROJECT_DEPENDENCIES
  ament_cmake
  rclcpp
  as2_core
  std_msgs
  as2_msgs
  std_srvs
  trajectory_msgs
  geometry_msgs
)

foreach(DEPENDENCY ${PROJECT_DEPENDENCIES})
  find_package(${DEPENDENCY} REQUIRED)
endforeach()

message("DYNAMIC_TRAJECTORY_GENERATOR_FOUND:")
message("${dynamic_trajectory_generator_INCLUDE_DIRS}")

include_directories(
  include
  include/${PROJECT_NAME}
)

set(SOURCE_CPP_FILES 
  src/as2_trajectory_generator.cpp
)

# set(INCLUDE_HPP_FILES 
#   include/${PROJECT_NAME}/as2_trajectory_generator_libs.hpp
# )

add_executable(${PROJECT_NAME}_node src/as2_trajectory_generator_node.cpp ${SOURCE_CPP_FILES})
target_link_libraries(${PROJECT_NAME}_node dynamic_trajectory_generator)
ament_target_dependencies(${PROJECT_NAME}_node ${PROJECT_DEPENDENCIES})


install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME})

install(TARGETS
  ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME})
    
ament_package()
