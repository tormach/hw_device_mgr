cmake_minimum_required(VERSION 3.5)
project(hw_device_mgr)

# Find dependencies
find_package(ament_cmake REQUIRED)
# - RT
find_package(hal_hw_interface REQUIRED)
# - Python
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
# - Msg
find_package(rosidl_default_generators REQUIRED)

#***********************************************
#* Declare ROS messages, services and actions **
#***********************************************

rosidl_generate_interfaces(${PROJECT_NAME}
  msg/MsgError.msg
  ADD_LINTER_TESTS
)

#************
#* Install **
#************

# Install python modules
ament_python_install_package(${PROJECT_NAME})

# Install python scripts
install(PROGRAMS
  scripts/hw_device_mgr
  DESTINATION lib/${PROJECT_NAME}
)

#**********
#* Tests **
#**********

if(BUILD_TESTING)

  find_package(ament_cmake_flake8 REQUIRED)
  ament_flake8()
  find_package(ament_cmake_pep257 REQUIRED)
  ament_pep257()
  find_package(ament_cmake_lint_cmake REQUIRED)
  ament_lint_cmake()
  find_package(ament_cmake_xmllint REQUIRED)
  ament_xmllint()
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_modules "hw_device_mgr")

endif()

ament_export_dependencies(rosidl_default_runtime)

ament_package()
