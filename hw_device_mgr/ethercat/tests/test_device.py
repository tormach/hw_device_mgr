import pytest
from ...cia_301.tests.test_device import TestCiA301Device as _TestCiA301Device
from .base_test_class import BaseEtherCATTestClass


class TestEtherCATDevice(BaseEtherCATTestClass, _TestCiA301Device):

    expected_mro = [
        "BogusEtherCATDevice",
        "EtherCATDevice",
        *_TestCiA301Device.expected_mro,
    ]

    def test_xml_description_path(self):
        for cls in self.device_model_classes:
            esi_path = cls.xml_description_path()
            print(esi_path)
            assert esi_path.exists()

    @pytest.fixture
    def obj(self, device_cls, device_data, sdo_data):
        self.obj = self.device_model_cls(
            address=device_data["address"], sim=self.sim
        )
        self.obj.init()
        self.obj.add_device_sdos()
        yield self.obj
