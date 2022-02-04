from ...cia_402.tests.test_device import TestCiA402Device as _TestCiA402Device
from .base_test_class import BaseEtherCATTestClass


class TestEtherCATDevice(BaseEtherCATTestClass, _TestCiA402Device):

    expected_mro = [
        "RelocatableESIDevice",
        "EtherCATSimDevice",
        "EtherCATDevice",
        *_TestCiA402Device.expected_mro,
    ]

    def test_xml_description_path(self):
        for cls in self.device_model_classes:
            esi_path = cls.xml_description_path()
            print(esi_path)
            assert esi_path.exists()
