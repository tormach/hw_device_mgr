cmake_minimum_required(VERSION 3.0.2)
project(hal_402_device_mgr)

#* Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

#* Find catkin macros and libraries
#* if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
#* is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS message_generation rospy)

# Use setup.py
catkin_python_setup()

#***********************************************
#* Declare ROS messages, services and actions **
#***********************************************

#* Generate messages in the 'msg' folder
add_message_files(FILES msg_error.msg msg_status.msg)

#* Generate services in the 'srv' folder
add_service_files(FILES srv_robot_state.srv)

#* Generate added messages and services with any dependencies listed here
generate_messages()

#**********************************
#* catkin specific configuration **
#**********************************
catkin_package(CATKIN_DEPENDS message_runtime rospy)

#************
#* Install **
#************

# Install python executables
catkin_install_python(PROGRAMS scripts/hal_402_mgr DESTINATION
                      ${CATKIN_PACKAGE_BIN_DESTINATION})
