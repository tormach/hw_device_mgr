from .base_test_class import BaseLCECTestClass
from ...ethercat.tests.test_command import (
    TestEtherCATCommand as _TestEtherCATCommand,
)


class TestLCECCommand(BaseLCECTestClass, _TestEtherCATCommand):
    halcomp_name = "lcec_command"
