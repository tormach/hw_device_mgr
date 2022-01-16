from ...tests.base_test_class import BaseTestClass
from .bogus_devices.device import (
    BogusErrorDevice,
    BogusV1ErrorServo,
    BogusV2ErrorServo,
    BogusV1ErrorIO,
)


class ErrorBaseTestClass(BaseTestClass):

    device_class = BogusErrorDevice
    device_model_classes = BogusV1ErrorServo, BogusV2ErrorServo, BogusV1ErrorIO
