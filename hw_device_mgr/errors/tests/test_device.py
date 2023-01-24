import pytest
from ...tests.test_device import TestDevice as _TestDevice
from .base_test_class import ErrorBaseTestClass


class TestErrorDevice(ErrorBaseTestClass, _TestDevice):
    expected_mro = [
        "ErrorSimDevice",
        "ErrorDevice",
        *_TestDevice.expected_mro,
        "ConfigIO",
    ]

    def test_error_descriptions(self, obj):
        print("cls:", type(obj))
        print("obj:", obj)
        print("yaml:", obj.device_error_package, obj.device_error_yaml)
        errs = obj.error_descriptions()
        assert isinstance(errs, dict)
        assert len(errs) > 0
        for err_code, data in errs.items():
            assert isinstance(err_code, int)
            assert "description" in data
            assert isinstance(data["description"], str)
            assert isinstance(data.get("advice", ""), str)
            print(f"   code {err_code}:  {data['description']}")

    def test_set_feedback(self, obj):
        errs = obj.error_descriptions()
        assert len(errs) > 0
        for err_code, data in errs.items():
            obj.set_feedback(error_code=err_code)
            fb = obj.feedback()
            for key, val in data.items():
                assert fb[key] == val
        assert 0
