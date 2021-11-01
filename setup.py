#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

dev_data_dir = "src/hw_device_mgr/devices"
# Packages like hw_device_mgr.{pkg}.tests.bogus_devices
pkgs_bd = [
    "cia_301",
    "cia_402",
    "errors",
    "ethercat",
    "hal",
    "lcec",
    "mgr",
    "mgr_hal",
    "mgr_ros",
]
# Packages like hw_device_mgr.{pkg}.tests
pkgs_t = ["devices", "mgr_ros_hal"] + pkgs_bd
# Generate lists
packages = (
    [
        "hw_device_mgr",
        "hw_device_mgr.tests",
        "hw_device_mgr.tests.bogus_devices",
    ]
    + [f"hw_device_mgr.{p}" for p in pkgs_t]
    + [f"hw_device_mgr.{p}.tests" for p in pkgs_t]
    + [f"hw_device_mgr.{p}.tests.bogus_devices" for p in pkgs_bd]
)

d = generate_distutils_setup(
    packages=packages,
    package_dir={"": "src"},
    package_data={
        "": [  # Within any package, install:
            # ESI files
            "device_xml/*.xml",
            # Error descriptions
            "device_err/*.yaml",
            # Test configs
            "tests/*.yaml",
            "bogus_devices/*.yaml",
        ],
    },
)

setup(**d)
