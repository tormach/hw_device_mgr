from ...lcec.tests.base_test_class import BaseLCECTestClass
from .devices import (
    DevicesForTest,
    ElmoGold420ForTest,
    ElmoGold520ForTest,
    InovanceIS620NForTest,
    InovanceSV660NForTest,
)


class BaseDevicesTestClass(BaseLCECTestClass):

    device_class = DevicesForTest
    # device_class = DevicesForTest
    device_model_classes = (
        ElmoGold420ForTest,
        ElmoGold520ForTest,
        InovanceIS620NForTest,
        InovanceSV660NForTest,
    )

    device_config_yaml = "devices/tests/device_config.yaml"
    sim_device_data_yaml = "devices/tests/sim_devices.yaml"
    device_sdos_yaml = "devices/tests/sim_sdo_data.yaml"
