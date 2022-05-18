import pytest
from .. import Logging


class TestLogging:
    tc = Logging

    @pytest.fixture
    def obj(self):
        return self.tc("test")

    def test_init(self, obj):
        assert hasattr(obj, "_logger")

    def test_getLevel_setLevel(self, obj):
        obj.setLevel("error")
        assert obj.getLevel() == obj.ERROR

    def test_getLogger(self):
        logger = self.tc.getLogger("test")
        assert isinstance(logger, self.tc)
