import pytest
from ...ethercat.tests.base_test_class import BaseEtherCATTestClass
from .bogus_devices.mgr import (
    BogusHWDeviceMgr,
    BogusEtherCAT402Device,
    BogusEtherCAT402Servo,
    BogusOtherCAT402Servo,
)


class BaseMgrTestClass(BaseEtherCATTestClass):
    """Base test class for `HWDeviceMgr` class."""

    # test_read_update_write() configuration
    read_update_write_yaml = "mgr/tests/read_update_write.cases.yaml"

    # Manager configuration
    mgr_config_yaml = "mgr/tests/bogus_devices/mgr_config.yaml"

    # Device model SDOs; for test fixture
    device_sdos_yaml = "devices/tests/sim_sdo_data.yaml"

    # Manager class
    device_class = BogusHWDeviceMgr

    # Base class for attached devices
    device_base_class = BogusEtherCAT402Device

    # Attached device classes
    device_model_classes = (
        BogusEtherCAT402Servo,
        BogusOtherCAT402Servo,
    )

    @pytest.fixture
    def mgr_config(self):
        self.mgr_config = self.load_yaml(self.mgr_config_yaml)
        return self.mgr_config
