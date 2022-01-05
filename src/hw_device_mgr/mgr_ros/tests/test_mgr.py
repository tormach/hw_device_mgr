from .base_test_class import BaseROSMgrTestClass
from ...mgr.tests.test_mgr import TestHWDeviceMgr as _TestHWDeviceMgr
import pytest
from pprint import pformat


class TestROSDeviceMgr(BaseROSMgrTestClass, _TestHWDeviceMgr):

    expected_mro = [
        "BogusROSHWDeviceMgr",
        "ROSHWDeviceMgr",
        "HWDeviceMgr",
        "FysomGlobalMixin",
        "Device",
        "ABC",
        "object",
    ]

    @pytest.fixture
    def obj(self, device_cls):
        # init() and init_devices() signatures changed
        self.obj = device_cls(sim=self.sim)
        self.obj.init()
        self.obj.init_devices()
        yield self.obj

    def test_ros_params(self, obj):
        print(f"self.rosparams:\n{pformat(self.rosparams)}")
        assert self.rosparams["sim"] is True  # Fixture sanity
        assert obj.sim is True  # Defaults to False
        assert obj.update_rate == 20  # Defaults to 10
        assert hasattr(obj, "mgr_config")
        assert "init_timeout" in obj.mgr_config
        assert hasattr(obj, "device_config")
        assert isinstance(obj.device_config, list)
        assert "param_values" in obj.device_config[0]
