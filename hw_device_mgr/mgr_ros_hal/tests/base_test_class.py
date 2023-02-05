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
    def category_extra_fixtures(
        self, mock_hal, ethercat_extra_fixtures, ros_extra_fixtures
    ):
        pass
