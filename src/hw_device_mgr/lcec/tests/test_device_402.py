from .test_device import TestLCECDevice as _TestLCECDevice
from ...ethercat.tests.test_device_402 import (
    TestEtherCAT402Device as _TestEtherCAT402Device,
)
from .bogus_devices.device_402 import (
    BogusLCEC402Device,
    BogusLCEC402Servo,
    BogusLCEC402Servo2,
)


class TestLCEC402Device(_TestLCECDevice, _TestEtherCAT402Device):

    device_class = BogusLCEC402Device
    device_model_classes = (BogusLCEC402Servo, BogusLCEC402Servo2)

    expected_mro = [
        "BogusLCEC402Device",
        "LCECDevice",
        "BogusEtherCAT402Device",
        "EtherCATDevice",
        "CiA402Device",
        "BogusCiA301Device",
        "CiA301Device",
        "HALPinDevice",
        "Device",
        "ABC",
        "HALMixin",
        "object",
    ]


[
    "BogusLCEC402Device",
    "LCECDevice",
    "BogusEtherCAT402Device",
    "EtherCATDevice",
    "CiA402Device",
    "BogusCiA301Device",
    "CiA301Device",
    "HALPinDevice",
    "Device",
    "ABC",
    "HALMixin",
    "object",
]
