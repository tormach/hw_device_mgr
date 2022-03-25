import pytest
from ...tests.test_device import TestDevice as _TestDevice
from .base_test_class import ErrorBaseTestClass


class TestErrorDevice(ErrorBaseTestClass, _TestDevice):
    expected_mro = [
        "ErrorSimDevice",
        "ErrorDevice",
        *_TestDevice.expected_mro,
    ]

    @pytest.fixture
    def obj(self):
        self.obj = self.device_class()
        self.obj.init()
        yield self.obj

    def test_error_descriptions(self):
        for cls in self.device_model_classes:
            print("cls:", cls)
            print("yaml:", cls.error_descriptions_yaml())
            errs = cls.error_descriptions()
            assert isinstance(errs, dict)
            assert len(errs) > 0
            for err_code, data in errs.items():
                assert isinstance(err_code, int)
                assert "description" in data
                assert isinstance(data["description"], str)
                assert isinstance(data.get("advice", ""), str)

    def test_set_feedback(self, obj):
        errs = self.device_class.error_descriptions()
        for err_code, data in errs.items():
            obj.set_feedback(error_code=err_code)
            fb = obj.feedback()
            for key, val in data.items():
                assert fb[key] == val
