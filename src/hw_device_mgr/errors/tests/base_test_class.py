from ...tests.base_test_class import BaseTestClass
from .bogus_devices.device import (
    BogusErrorDevice,
    BogusErrorServo,
    BogusErrorIO,
)


class ErrorBaseTestClass(BaseTestClass):

    device_class = BogusErrorDevice
    device_model_classes = BogusErrorServo, BogusErrorIO
