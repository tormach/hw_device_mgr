from ...mgr.tests.base_test_class import BaseMgrTestClass
from ...lcec.tests.base_test_class import BaseLCECTestClass
from .bogus_devices.mgr import BogusLCEC402HWDeviceMgr
import pytest


class BaseLCEC402MgrTestClass(BaseMgrTestClass, BaseLCECTestClass):
    """Base test class for `HALHWDeviceMgr` class"""

    # Manager class
    device_class = BogusLCEC402HWDeviceMgr

    # Data types
    data_type_class = BogusLCEC402HWDeviceMgr.data_type_class

    # Attached device classes
    device_model_classes = BogusLCEC402HWDeviceMgr.device_classes

    @pytest.fixture
    def manager_lcec(self, mock_hal, mock_ethercat_command):
        """Mock HAL and `ethercat` command for the device manager"""

    @pytest.fixture
    def device_cls(self, config_cls, manager_lcec):
        """Fixture for HAL Device classes"""
        self.device_class.clear_devices()
        yield self.device_class
