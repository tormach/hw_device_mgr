from ...mgr_hal.tests.base_test_class import BaseLCEC402MgrTestClass
from ...mgr_ros.tests.base_test_class import BaseROSMgrTestClass
from .bogus_devices.mgr import BogusHALROSHWDeviceMgr
from ...devices.tests.test_devices_402 import Test402Devices
import pytest


###############################
# Test class


class BaseROSHALMgrTestClass(BaseLCEC402MgrTestClass, BaseROSMgrTestClass):
    """Base test class for `ROSHWDeviceMgr` class."""

    data_type_class = BogusHALROSHWDeviceMgr.data_type_class
    device_class = BogusHALROSHWDeviceMgr
    device_base_class = BogusHALROSHWDeviceMgr.device_base_class
    device_model_classes = BogusHALROSHWDeviceMgr.device_classes

    device_config_yaml = Test402Devices.device_config_yaml
    device_data_yaml = Test402Devices.device_data_yaml
    device_sdos_yaml = Test402Devices.device_sdos_yaml

    device_model_sdo_clone = None

    @pytest.fixture
    def extra_fixtures(
        self, manager_ros_params, mock_hal, mock_ethercat_command
    ):
        pass
