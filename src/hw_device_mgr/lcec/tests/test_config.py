from ...ethercat.tests.test_config import (
    TestEtherCATConfig as _TestEtherCATConfig,
)
from .base_test_class import BaseLCECTestClass


class TestLCECConfig(BaseLCECTestClass, _TestEtherCATConfig):
    pass
