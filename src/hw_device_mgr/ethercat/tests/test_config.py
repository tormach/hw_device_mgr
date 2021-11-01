from ...cia_301.tests.test_config import TestCiA301Config as _TestCiA301Config
from .base_test_class import BaseEtherCATTestClass
import pytest


class TestEtherCATConfig(BaseEtherCATTestClass, _TestCiA301Config):
    @pytest.fixture
    def obj(self, device_data, sdo_data, config_cls):
        obj = config_cls(
            address=device_data["address"], model_id=device_data["model_id"]
        )
        yield obj
