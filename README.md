# `hw_device_mgr`:  Manage hardware devices in ROS and Machinekit HAL

This project manages EtherCAT and CANopen hardware devices:
discovery, configuration, enable/disable, fault management, and more.
It provides interfaces for Machinekit HAL and ROS applications.

The Python classes are structured for maximum flexibility, making it
easy to add new device types and new interfaces to other software.

[![CI](https://github.com/tormach/hw_device_mgr/actions/workflows/ci.yaml/badge.svg)](https://github.com/tormach/hw_device_mgr/actions/workflows/ci.yaml)
