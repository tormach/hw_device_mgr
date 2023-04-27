from ...tests.base_test_class import BaseTestClass
from .bogus_devices.device import (
    BogusErrorDevice,
    BogusErrorV1Servo,
    BogusErrorV2Servo,
    BogusErrorV1IO,
)


class ErrorBaseTestClass(BaseTestClass):
    device_class = BogusErrorDevice
    device_model_classes = BogusErrorV1Servo, BogusErrorV2Servo, BogusErrorV1IO
