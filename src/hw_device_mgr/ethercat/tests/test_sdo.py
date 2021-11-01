from ...cia_301.tests.test_sdo import TestCiA301SDO as _TestCiA301SDO
from .base_test_class import BaseEtherCATTestClass


class TestEtherCATSDO(BaseEtherCATTestClass, _TestCiA301SDO):
    pass
