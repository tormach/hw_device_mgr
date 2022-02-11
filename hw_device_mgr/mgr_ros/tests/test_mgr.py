from .base_test_class import BaseROSMgrTestClass
from ...mgr.tests.test_mgr import TestHWDeviceMgr as _TestHWDeviceMgr
import pytest
from pprint import pformat
import yaml


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
    rclpy_patches = [
        "hw_device_mgr.mgr_ros.mgr.rclpy",
    ]

    @pytest.fixture
    def obj(self, device_cls, global_config, tmp_path):
        # init() and init_devices() signatures changed, so can't use
        # parent test class obj fixture
        tmpfile = tmp_path / "global_config.yaml"
        with open(tmpfile, "w") as f:
            f.write(yaml.dump(global_config))
        self.obj = device_cls()
        self.obj.init(list())
        self.obj.init_devices(tmpfile)
        print("device_config:", self.obj.device_config)
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
        assert "param_values" in obj.device_config[0]
