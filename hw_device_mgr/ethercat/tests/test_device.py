from ...cia_301.tests.test_device import TestCiA301Device as _TestCiA301Device
from .base_test_class import BaseEtherCATTestClass


class TestEtherCATDevice(BaseEtherCATTestClass, _TestCiA301Device):

    expected_mro = [
        "BogusEtherCATDevice",
        "EtherCATSimDevice",
        "EtherCATDevice",
        *_TestCiA301Device.expected_mro[1:],  # Lop off BogusCiA301Device
    ]

    def test_xml_description_path(self):
        for cls in self.device_model_classes:
            esi_path = cls.xml_description_path()
            print(esi_path)
            assert esi_path.exists()
