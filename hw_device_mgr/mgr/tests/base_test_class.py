import pytest
from ...devices.tests.base_test_class import BaseDevicesTestClass
from .bogus_devices.mgr import (
    HWDeviceMgrTest,
    HwMgrTestDevices,
    HwMgrTestElmoGold420,
    HwMgrTestElmoGold520,
    HwMgrTestInovanceIS620N,
    HwMgrTestInovanceSV660N,
)


class BaseMgrTestClass(BaseDevicesTestClass):
    """Base test class for `HWDeviceMgr` class."""

    # The HWDeviceMgr class under test inherits from the `Device` *base class*,
    # but this base test class inherits from classes used to test `Device`
    # *subclasses*, e.g. the `CiA301Device` class.  Those classes provide
    # fixtures for the sim devices, device config, SDOs, etc.  (And this is a
    # major reason for the separate test base classes:  to provide relevant
    # fixtures without dragging in irrelevant tests.)

    # test_read_update_write() configuration
    read_update_write_yaml = "mgr/tests/read_update_write.cases.yaml"

    # Manager configuration
    mgr_config_yaml = "mgr/tests/bogus_devices/mgr_config.yaml"

    # Device model SDOs; for test fixture
    device_sdos_yaml = "devices/tests/sim_sdo_data.yaml"

    # Manager class
    device_class = HWDeviceMgrTest

    # Base class for attached devices
    device_base_class = HwMgrTestDevices

    # Attached device classes
    device_model_classes = (
        HwMgrTestElmoGold420,
        HwMgrTestElmoGold520,
        HwMgrTestInovanceIS620N,
        HwMgrTestInovanceSV660N,
    )

    @pytest.fixture
    def mgr_config(self):
        self.mgr_config = self.load_yaml(self.mgr_config_yaml)
        return self.mgr_config

    @pytest.fixture
    def device_cls(self, device_config, extra_fixtures):
        # Don't init device_model_classes device_config; HWDeviceMgr does that
        self.init_sim()
        yield self.device_class

    @pytest.fixture
    def category_cls(self):
        """Fixture for Device class category."""
        yield self.device_base_class
