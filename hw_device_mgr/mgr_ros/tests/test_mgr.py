from .base_test_class import BaseROSMgrTestClass
from ...mgr.tests.test_mgr import TestHWDeviceMgr as _TestHWDeviceMgr
import pytest
from pprint import pformat
import yaml


class TestROSHWDeviceMgr(BaseROSMgrTestClass, _TestHWDeviceMgr):

    expected_mro = [
        "BogusROSHWDeviceMgr",
        "SimROSHWDeviceMgr",
        "ROSHWDeviceMgr",
        "BogusHWDeviceMgr",
        "SimHWDeviceMgr",
        "HWDeviceMgr",
        "FysomGlobalMixin",
        "SimDevice",
        "Device",
        "ABC",
        "object",
    ]
    rclpy_patches = [
        "hw_device_mgr.mgr_ros.mgr.rclpy",
    ]

    @pytest.fixture
    def obj(self, device_cls, device_config_path, device_data_path):
        # init_sim() and init_devices() signatures changed, so can't
        # use parent test class obj fixture
        self.obj = device_cls(sim=self.sim)
        self.obj.init(list())
        self.obj.init_sim()
        self.obj.init_devices()
        yield self.obj

    def test_ros_params(self, obj):
        print(f"self.rosparams:\n{pformat(self.rosparams)}")
        assert self.rosparams["use_sim"] is True  # Fixture sanity
        assert obj.sim is True  # Defaults to False
        assert obj.update_rate == 20  # Defaults to 10
        assert hasattr(obj, "mgr_config")
        assert "init_timeout" in obj.mgr_config
        assert hasattr(obj, "device_config")
        assert isinstance(obj.device_config, list)
        for devc in obj.device_config:
            if devc["category"] == "bogus_v1_io":
                continue
            assert "param_values" in devc
