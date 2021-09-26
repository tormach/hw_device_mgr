# Test and test Device base classes
from ...ethercat.tests.test_device_402 import (
    TestEtherCAT402Device as _TestEtherCAT402Device,
)
from ...ethercat.tests.bogus_devices.device_402 import BogusEtherCAT402Device

# Device model classes
from ..elmo_gold import ElmoGold420, ElmoGold520
from ..inovance_is620n import InovanceIS620N
from ..inovance_sv660 import InovanceSV660


class ElmoGold420ForTest(ElmoGold420, BogusEtherCAT402Device):
    allow_rereg = True


class ElmoGold520ForTest(ElmoGold520, BogusEtherCAT402Device):
    allow_rereg = True


class InovanceIS620NForTest(InovanceIS620N, BogusEtherCAT402Device):
    allow_rereg = True


class InovanceSV660NForTest(InovanceSV660, BogusEtherCAT402Device):
    allow_rereg = True


class Test402Devices(_TestEtherCAT402Device):
    expected_mro = [
        "BogusEtherCAT402Device",
        "EtherCATDevice",
        "CiA402Device",
        "BogusCiA301Device",
        "CiA301Device",
        "Device",
        "ABC",
        "object",
    ]

    device_class = BogusEtherCAT402Device
    device_model_classes = (
        ElmoGold420ForTest,
        ElmoGold520ForTest,
        InovanceIS620NForTest,
        InovanceSV660NForTest,
    )

    global_config_yaml = "devices/tests/config.yaml"
    device_data_yaml = "devices/tests/device_config.yaml"
    device_sdos_yaml = "devices/tests/sdos.yaml"
