cmake_minimum_required(VERSION 2.8.3)
project(velma_hardware)

### ROS Dependencies ###
# Find the RTT-ROS package (this transitively includes the Orocos CMake macros)
find_package(catkin REQUIRED COMPONENTS
  rtt_ros cmake_modules
  # ADDITIONAL ROS PACKAGES
  )

find_package(Eigen REQUIRED)
find_package(Xenomai REQUIRED)

### Orocos Dependencies ###
# Note that orocos_use_package() does not need to be called for any dependency
# listed in the package.xml file

include_directories(${catkin_INCLUDE_DIRS} ${USE_OROCOS_INCLUDE_DIRS} ${Eigen_INCLUDE_DIRS})

### Orocos Targets ###
if(${OROCOS_TARGET} STREQUAL "xenomai" )
orocos_component(torso_controller src/torso.cpp src/MotorController.cpp src/CANDev.cpp)
target_link_libraries(torso_controller ${catkin_LIBRARIES} ${USE_OROCOS_LIBRARIES} ${Xenomai_RTDM_LIBRARIES})
endif()

### Orocos Package Exports and Install Targets ###

# Generate install targets for header files

orocos_install_headers(DIRECTORY include/${PROJECT_NAME})

# Export package information (replaces catkin_package() macro) 
orocos_generate_package(
  INCLUDE_DIRS include
  DEPENDS rtt_ros
)
