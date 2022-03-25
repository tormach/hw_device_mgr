from ...mgr.tests.base_test_class import BaseMgrTestClass
from ...lcec.tests.base_test_class import BaseLCECTestClass
from .bogus_devices.mgr import HALHWDeviceMgrTest
import pytest


class BaseHALMgrTestClass(BaseMgrTestClass, BaseLCECTestClass):
    """Base test class for `HALHWDeviceMgr` class."""

    # Manager class
    device_class = HALHWDeviceMgrTest

    # Data types
    data_type_class = HALHWDeviceMgrTest.data_type_class

    # Base class for attached devices
    device_base_class = HALHWDeviceMgrTest.device_base_class

    # Attached device classes
    device_model_classes = HALHWDeviceMgrTest.device_classes

    @pytest.fixture
    def extra_fixtures(self, mock_hal, mock_ethercat_command):
        pass
