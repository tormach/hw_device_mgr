from setuptools import setup
from setuptools.command.install import install
from warnings import warn
import subprocess
import os

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
    "mgr_ros_hal",
]
# Packages like hw_device_mgr.{pkg}.tests
pkgs_t = [
    "devices",
    "logging",
    *pkgs_bd,
]
# Generate lists
packages = (
    [
        "hw_device_mgr",
        "hw_device_mgr.latency",
        "hw_device_mgr.tests",
        "hw_device_mgr.tests.bogus_devices",
    ]
    + [f"hw_device_mgr.{p}" for p in pkgs_t]
    + [f"hw_device_mgr.{p}.tests" for p in pkgs_t]
    + [f"hw_device_mgr.{p}.tests.bogus_devices" for p in pkgs_bd]
    + [
        "hw_device_mgr.devices.device_xml",
        "hw_device_mgr.devices.device_err",
    ]
)


class CustomInstall(install):
    user_options = install.user_options + [
        (
            "sudo-halcompile=",
            None,
            "If needed to install HAL components, sudo executable (ROS 2 only).",
        ),
    ]

    def initialize_options(self):
        super().initialize_options()
        self.sudo_halcompile = None

    def finalize_options(self):
        super().finalize_options()
        # You can add validation or default value logic here
        if self.sudo_halcompile:
            print(f"Prepending to halcompile: {self.sudo_halcompile}")

    def run(self):
        """Run halcompile on `multilatency.comp`."""
        if os.environ.get("ROS_VERSION", None) == "2":
            # ROS1 builds comp from CMakeFile
            comp_src = "hw_device_mgr/latency/multilatency.comp"
            cmd = ["/usr/bin/env", "halcompile", "--install", comp_src]
            if self.sudo_halcompile:
                cmd.insert(0, self.sudo_halcompile)
            res = subprocess.run(cmd, capture_output=True)
            if res.returncode != 0:
                warn(f"Command failed, {repr(cmd)}")
                for line in res.stderr.decode().splitlines():
                    warn(f"stderr:  {line}")
        super().run()


setup_kwargs = dict()
if os.environ.get("ROS_VERSION", None) != "1":
    # catkin doesn't support zip_safe or entry_points
    setup_kwargs["zip_safe"] = True
    setup_kwargs["entry_points"] = {
        "console_scripts": [
            "hw_device_mgr = hw_device_mgr.mgr_ros_hal.__main__:main",
            "ecat_pcap_decode = hw_device_mgr.latency.ecat_pcap_decode:main",
            "halsampler_decode = hw_device_mgr.latency.halsampler_decode:main",
        ]
    }

setup(
    name=package_name,
    version="0.2.0",
    packages=packages,
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
            "*.xml",
            # Error descriptions
            "device_err/*.yaml",
            # Test configs
            "tests/*.yaml",
            "bogus_devices/*.yaml",
        ],
    },
    install_requires=["setuptools"],
    maintainer="John Morris",
    maintainer_email="john@zultron.com",
    description="Machinekit HAL interface to robot hardware and I/O",
    license="BSD",
    tests_require=["pytest"],
    cmdclass={"install": CustomInstall},
    **setup_kwargs,
)
