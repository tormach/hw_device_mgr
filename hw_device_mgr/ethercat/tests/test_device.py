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
            assert cls.xml_description_fname
            if cls.xml_description_package is None:
                assert "/" in cls.xml_description_fname
            else:
                assert "/" not in cls.xml_description_fname
