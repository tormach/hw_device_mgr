from ...ethercat.tests.base_test_class import BaseEtherCATTestClass
from .devices import (
    DevicesForTest,
    ElmoGold420ForTest,
    ElmoGold520ForTest,
    InovanceIS620NForTest,
    InovanceSV660NForTest,
    EVEXCREForTest,
)


class BaseDevicesTestClass(BaseEtherCATTestClass):

    device_class = DevicesForTest
    # device_class = DevicesForTest
    device_model_classes = (
        ElmoGold420ForTest,
        ElmoGold520ForTest,
        InovanceIS620NForTest,
        InovanceSV660NForTest,
        EVEXCREForTest,
    )

    device_config_package = "hw_device_mgr.devices.tests"
    sim_device_data_package = "hw_device_mgr.devices.tests"
    device_sdos_package = "hw_device_mgr.devices.tests"
    device_dcs_package = "hw_device_mgr.devices.tests"
