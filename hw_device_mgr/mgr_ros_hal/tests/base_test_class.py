from ...mgr_hal.tests.base_test_class import BaseLCEC402MgrTestClass
from ...mgr_ros.tests.base_test_class import BaseROSMgrTestClass
from ..mgr import ROSHALHWDeviceMgr
import pytest


###############################
# Test class


class BaseROSHALMgrTestClass(BaseLCEC402MgrTestClass, BaseROSMgrTestClass):
    """Base test class for `ROSHWDeviceMgr` class"""

    # Managed device types
    device_class = ROSHALHWDeviceMgr
    data_type_class = ROSHALHWDeviceMgr.data_type_class
    device_model_classes = ROSHALHWDeviceMgr.device_classes

    # Test configuration
    global_config_yaml = "devices/tests/config.yaml"
    device_data_yaml = "devices/tests/device_config.yaml"
    device_sdos_yaml = "devices/tests/sdos.yaml"

    @pytest.fixture
    def device_cls(self, config_cls, manager_ros_params, manager_lcec):
        """Fixture for ROS + LCEC Device classes"""
        self.device_class.clear_devices()
        yield self.device_class
