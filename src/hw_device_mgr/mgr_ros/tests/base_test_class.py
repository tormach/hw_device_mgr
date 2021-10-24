from ...mgr.tests.base_test_class import BaseMgrTestClass
from .bogus_devices.mgr import BogusROSHWDeviceMgr
import pytest
from .mock_rospy import MockRospy


###############################
# Test class


class BaseROSMgrTestClass(BaseMgrTestClass):
    """Base test class for `ROSHWDeviceMgr` class"""

    patch_mock_rospy = ("hw_device_mgr.mgr_ros.mgr.rospy",)

    # Manager class
    device_class = BogusROSHWDeviceMgr

    # Data types
    data_type_class = BogusROSHWDeviceMgr.data_type_class

    # Attached device classes
    device_model_classes = BogusROSHWDeviceMgr.device_classes

    @pytest.fixture
    def mock_rospy(self):
        """Fixture for mocking `rospy`

        Because this mocks the entire top-level `rospy` module, the test
        object (or class) must have a `patch_rospy` attribute, a `str` or
        `tuple` of `str`, with the object to patch, e.g. if your module
        `mymodule` has `import rospy`, then the test object must have
        `patch_rospy` attribute `mymodule.rospy`.

        The mock object is passed as test function arg and also test
        object `mock_rospy` attribute.  It mocks the following methods:

        - `init_node()`:  Stores name for use in other methods
        - `is_shutdown()`:  Returns `True` if accessor `set_shutdown()`
          was called
        - `has_param()`, `get_param()`:  See below for mocking ROS params
        - `log*()`:  Prints log to stdout

        The above methods are regular `MagicMock` objects and can be
        tested with `assert_called()`, etc. methods.

        ROS params may be mocked in two ways:
        - The test object's 'ros_params_yaml' attribute may name a `.yaml`
          file in the current directory from which to read parameters
        - The test object's `ros_params` attribute may contain a `dict`
        object containing parameters
        """

        if not hasattr(self, "ros_params"):
            self.ros_params = dict()
        yield from MockRospy.fixture("mock_rospy", self, self.ros_params)

    @pytest.fixture
    def manager_ros_params(self, mock_rospy, mgr_config, global_config):
        """ROS params for the device manager"""
        hdm_params = dict(
            manager_config=mgr_config,
            device_config=global_config,
            update_rate=20,
            sim=True,
        )
        self.ros_params.update(dict(hw_device_mgr=hdm_params))

    @pytest.fixture
    def device_cls(self, config_cls, manager_ros_params):
        """Fixture for ROS Device classes"""
        self.device_class.clear_devices()
        yield self.device_class
