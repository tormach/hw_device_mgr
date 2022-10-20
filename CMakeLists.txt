cmake_minimum_required(VERSION 3.0.2)
project(hw_device_mgr)

#* Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS hal_hw_interface
                                        rospy)
# Find Machinekit HAL
find_package(HAL)
include(UseHALComp)

# Use setup.py
catkin_python_setup()

# catkin configuration
catkin_package(CATKIN_DEPENDS hal_hw_interface rospy)

# Build HAL component
# - Specify additional header locations
include_directories(${HAL_INCLUDE_PATH} ${catkin_INCLUDE_DIRS})
hal_add_instcomp(hw_device_mgr/latency/multilatency)

# Install python executables
catkin_install_python(PROGRAMS scripts/ecat_pcap_decode scripts/halsampler_decode
                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

# Tests
if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  find_package(ros_pytest REQUIRED)
  add_rostest(tests/test.launch)
endif()
