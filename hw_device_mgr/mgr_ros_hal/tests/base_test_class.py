from ...mgr_hal.tests.base_test_class import BaseHALMgrTestClass
from ...mgr_ros.tests.base_test_class import BaseROSMgrTestClass
from .bogus_devices.mgr import ROSHWDeviceMgrTest
import pytest


class BaseROSHALMgrTestClass(BaseROSMgrTestClass, BaseHALMgrTestClass):
    """Base test class for `ROSHALHWDeviceMgr` class."""

    data_type_class = ROSHWDeviceMgrTest.data_type_class
    device_class = ROSHWDeviceMgrTest
    device_base_class = ROSHWDeviceMgrTest.device_base_class
    device_model_classes = ROSHWDeviceMgrTest.device_classes

    @pytest.fixture
    def extra_fixtures(
        self,
        manager_ros_params,
        sim_device_data_path,
        device_config_path,
        mock_hal,
        mock_ethercat_command,
    ):
        pass
