import pytest
from ...devices.tests.base_test_class import BaseDevicesTestClass
from .bogus_devices.mgr import (
    HWDeviceMgrTest,
    HwMgrTestDevices,
    HwMgrTestElmoGold420,
    HwMgrTestElmoGold520,
    HwMgrTestInovanceIS620N,
    HwMgrTestInovanceSV660N,
    HwMgrTestEVEXCRE,
)


class BaseMgrTestClass(BaseDevicesTestClass):
    """Base test class for `HWDeviceMgr` class."""

    # The HWDeviceMgr class under test inherits from the `Device` *base class*,
    # but this base test class inherits from classes used to test `Device`
    # *subclasses*, e.g. the `CiA301Device` class.  Those classes provide
    # fixtures for the sim devices, device config, SDOs, etc.  (And this is a
    # major reason for the separate test base classes:  to provide relevant
    # fixtures without dragging in irrelevant tests.)

    # test_read_update_write() configuration:
    # CiA NMT init online & operational status
    read_update_write_package = "hw_device_mgr.mgr.tests"

    # Manager configuration
    mgr_config_package = "hw_device_mgr.mgr.tests.bogus_devices"
    mgr_config_yaml = "mgr_config.yaml"

    # Device model SDOs; for test fixture
    device_sdos_package = "hw_device_mgr.devices.tests"

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
        HwMgrTestEVEXCRE,
    )

    @pytest.fixture
    def mgr_config(self):
        rsrc = self.mgr_config_package, self.mgr_config_yaml
        mgr_config = self.load_yaml_resource(*rsrc)
        assert mgr_config, f"Empty YAML package resource {rsrc}"
        return mgr_config

    @classmethod
    def init_sim(cls, **kwargs):
        """Create sim device objects with configured SDOs."""
        super().init_sim(**cls.init_sim_sdo_kwargs(**kwargs))

    @pytest.fixture
    def device_cls(self, device_config, extra_fixtures):
        # Don't init device_model_classes device_config; HWDeviceMgr does that
        self.init_sim()
        yield self.device_class

    @pytest.fixture
    def category_cls(self):
        """Fixture for Device class category."""
        yield self.device_base_class
