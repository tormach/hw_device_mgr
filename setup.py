#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

dev_data_dir = "src/hw_device_mgr/devices"
d = generate_distutils_setup(
    packages=["hw_device_mgr"],
    package_dir={"": "src"},
    package_data=[
        (
            "drive_err",
            [
                f"{dev_data_dir}/IS620N.yaml",
                f"{dev_data_dir}/SV660N.yaml",
                # (No errors for iTegva_E7x)
            ],
        ),
        (
            "drive_xml",
            [
                f"{dev_data_dir}/IS620N_v2.6.7.xml",
                f"{dev_data_dir}/SV660_EOE_1Axis_V9.12.xml",
                f"{dev_data_dir}/iTegva_E7x_Series.xml",
            ],
        ),
    ],
)

setup(**d)
