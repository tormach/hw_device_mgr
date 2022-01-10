# Test and test Device base classes
from ...lcec.tests.test_device_402 import (
    TestLCEC402Device as _TestLCEC402Device,
)
from ...lcec.device import LCECSimDevice
from ...cia_402.device import CiA402SimDevice

# Device model classes
from ..elmo_gold import ElmoGold420, ElmoGold520
from ..inovance_is620n import InovanceIS620N
from ..inovance_sv660 import InovanceSV660


class DevicesForTest(LCECSimDevice, CiA402SimDevice):
    category = "devices_for_test"


class ElmoGold420Category(DevicesForTest):
    category = "elmo_gold_420_test"


class ElmoGold420ForTest(ElmoGold420, ElmoGold420Category):
    product_code = 0x10030924


class ElmoGold520Category(DevicesForTest):
    category = "elmo_gold_520_test"


class ElmoGold520ForTest(ElmoGold520, ElmoGold520Category):
    product_code = 0x10030925


class InovanceIS620NCategory(DevicesForTest):
    category = "inovance_is620n_test"


class InovanceIS620NForTest(InovanceIS620N, InovanceIS620NCategory):
    product_code = 0x100C0108


class InovanceSV660NCategory(DevicesForTest):
    category = "inovance_sv660n_test"


class InovanceSV660NForTest(InovanceSV660, InovanceSV660NCategory):
    product_code = 0x100C010D


class Test402Devices(_TestLCEC402Device):
    expected_mro = [
        "DevicesForTest",
        "LCECSimDevice",
        "LCECDevice",
        "EtherCATSimDevice",
        "EtherCATDevice",
        "CiA402SimDevice",
        "CiA402Device",
        "CiA301SimDevice",
        "CiA301Device",
        "HALPinDevice",
        "SimDevice",
        "Device",
        "ABC",
        "HALMixin",
        "object",
    ]

    device_class = DevicesForTest
    device_model_classes = (
        ElmoGold420ForTest,
        ElmoGold520ForTest,
        InovanceIS620NForTest,
        InovanceSV660NForTest,
    )
    device_model_sdo_clone = None

    device_config_yaml = "devices/tests/device_config.yaml"
    device_data_yaml = "devices/tests/sim_devices.yaml"
    device_sdos_yaml = "devices/tests/sim_sdo_data.yaml"
