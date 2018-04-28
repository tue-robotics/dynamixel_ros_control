cmake_minimum_required(VERSION 2.8.3)
project(dynamixel_ros_control)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED
  ros_control_boilerplate
  dynamixel_workbench_toolbox
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
 INCLUDE_DIRS include
 LIBRARIES dynamixel_hw_interface
 CATKIN_DEPENDS ros_control_boilerplate
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)


file(GLOB HEADERS include/*/*.h)

# Dynamixel Hardware Interface
add_library(dynamixel_hw_interface
  ${HEADERS}
  src/dynamixel_hw_interface.cpp
)
target_link_libraries(dynamixel_hw_interface
  ${catkin_LIBRARIES}
)

# Main control executable
add_executable(dynamixel_hw_main src/dynamixel_hw_main.cpp)
target_link_libraries(dynamixel_hw_main
  dynamixel_hw_interface
  ${catkin_LIBRARIES}
)

## Install ------------------------------------------------------------

# Install libraries
install(TARGETS
    dynamixel_hw_interface
  LIBRARY DESTINATION
    ${CATKIN_PACKAGE_LIB_DESTINATION}
)

# Install executables
install(TARGETS
    dynamixel_hw_main
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Install header files
install(DIRECTORY include/dynamixel_ros_control/   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
