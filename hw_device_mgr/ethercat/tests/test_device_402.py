from ...cia_402.tests.test_device import TestCiA402Device as _TestCiA402Device
from .base_test_class import BaseEtherCATTestClass
from .bogus_devices.device_402 import (
    BogusEtherCAT402Device,
    BogusEtherCAT402Servo,
    BogusOtherCAT402Servo,
)


class TestEtherCAT402Device(BaseEtherCATTestClass, _TestCiA402Device):
    # This class tests EtherCAT devices with DS402 drive profile

    expected_mro = [
        "BogusEtherCAT402Device",
        "EtherCATDevice",
        *_TestCiA402Device.expected_mro[1:],  # Remove BogusCiA402Device
    ]

    device_class = BogusEtherCAT402Device
    device_model_classes = (
        BogusEtherCAT402Servo,
        BogusOtherCAT402Servo,
    )
