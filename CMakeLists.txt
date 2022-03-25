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
  find_package(ament_cmake_lint_cmake REQUIRED)
  ament_lint_cmake()
  # FIXME disable until this is resolved, either by loading the xsd
  # file or by moving the ESI files out of this repo or by excluding
  # ESI files from the linter.
  #
  #   warning: failed to load external entity "EtherCATInfo.xsd"
  #   Schemas parser error : Failed to locate the main schema resource at 'EtherCATInfo.xsd'.
  #   WXS schema EtherCATInfo.xsd failed to compile
  #
  # find_package(ament_cmake_xmllint REQUIRED)
  # ament_xmllint()
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_modules "hw_device_mgr" TIMEOUT 120)

endif()

ament_export_dependencies(rosidl_default_runtime)

ament_package()
