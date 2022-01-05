from ...cia_301.tests.test_command import (
    TestCiA301Command as _TestCiA301Command,
)
from .base_test_class import BaseEtherCATTestClass


class TestEtherCATCommand(BaseEtherCATTestClass, _TestCiA301Command):
    pass
