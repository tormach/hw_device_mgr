from .test_device import TestLCECDevice as _TestLCECDevice
from ...ethercat.tests.test_device_402 import (
    TestEtherCAT402Device as _TestEtherCAT402Device,
)
from .bogus_devices.device_402 import (
    BogusLCEC402Device,
    BogusV1LCEC402Servo,
    BogusV2LCEC402Servo,
)


class TestLCEC402Device(_TestLCECDevice, _TestEtherCAT402Device):

    device_class = BogusLCEC402Device
    device_model_classes = (BogusV1LCEC402Servo, BogusV2LCEC402Servo)

    expected_mro = [
        "BogusLCEC402Device",
        "LCECSimDevice",
        "LCECDevice",
        "EtherCATSimDevice",
        "EtherCATDevice",
        "BogusCiA402DeviceCategory",
        "CiA402SimDevice",
        "CiA402Device",
        "BogusCiA301DeviceCategory",
        "CiA301SimDevice",
        "CiA301Device",
        "HALPinDevice",
        "BogusDevice",
        "SimDevice",
        "Device",
        "ABC",
        "HALMixin",
        "object",
    ]
