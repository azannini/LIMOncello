cmake_minimum_required(VERSION 3.5)
project(limoncello)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if (NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type (Release or Debug)" FORCE)
endif()

# Set C++ standard and compiler flags
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=native -pthread -fexceptions -O3 -lm -lblas -llapack")

# Find Boost and TBB
find_package(Boost REQUIRED COMPONENTS system)
find_package(TBB REQUIRED)

# Include Boost directories
include_directories(${Boost_INCLUDE_DIRS})

# Find Eigen3 and PCL
find_package(Eigen3 REQUIRED)
find_package(PCL 1.8 REQUIRED)

# Find ROS packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)

# Include directories
include_directories(
  include
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

# Manif
add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/external/manif EXCLUDE_FROM_ALL)

# Link libraries
set(LINK_LIBRARIES
  PRIVATE manif
  ${PCL_LIBRARIES}
  ${Boost_LIBRARIES}
  ${TBB_IMPORTED_TARGETS}
)

# Create executable
add_executable(${PROJECT_NAME} src/main.cpp)

ament_target_dependencies(${PROJECT_NAME} 
    PUBLIC rclcpp 
    PUBLIC pcl_conversions
    PUBLIC sensor_msgs 
    PUBLIC nav_msgs 
    PUBLIC tf2 
    PUBLIC tf2_ros 
    PUBLIC tf2_eigen
)

# Link the executable with libraries
target_link_libraries(${PROJECT_NAME} ${LINK_LIBRARIES})

install(TARGETS 
  ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME} 
)


install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()