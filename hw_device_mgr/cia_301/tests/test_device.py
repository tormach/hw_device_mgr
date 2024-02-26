from .base_test_class import BaseCiA301TestClass
from ...tests.test_device import TestDevice as _TestDevice


class TestCiA301Device(BaseCiA301TestClass, _TestDevice):
    expected_mro = [
        "CiA301SimDevice",
        "CiA301Device",
        *_TestDevice.expected_mro,
    ]
