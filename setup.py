from setuptools import setup

package_name = "hw_device_mgr"

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

setup(
    name=package_name,
    version="0.2.0",
    packages=packages,
    package_dir={"": "src"},
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        ("share/" + package_name, ["package.xml"]),
    ],
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
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="John Morris",
    maintainer_email="john@zultron.com",
    description="Machinekit HAL interface to robot hardware and I/O",
    license="BSD",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "scripts/hw_device_mgr",
        ],
    },
)
