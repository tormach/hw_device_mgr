from .base_test_class import BaseROSMgrTestClass
from ...mgr.tests.test_mgr import TestHWDeviceMgr as _TestHWDeviceMgr
import pytest
from pprint import pformat


class TestROSHWDeviceMgr(BaseROSMgrTestClass, _TestHWDeviceMgr):

    expected_mro = [
        "ROSHWDeviceMgrTestCategory",
        "ROSSimHWDeviceMgr",
        "ROSHWDeviceMgr",
        *_TestHWDeviceMgr.expected_mro[1:],
        "ConfigIO",
    ]
    rclpy_patches = [
        "hw_device_mgr.mgr_ros.mgr.rclpy",
    ]

    @pytest.fixture
    def obj(self, category_cls):
        # init_sim() and init_devices() signatures changed, so can't
        # use parent test class obj fixture
        self.obj = self.device_class()
        self.obj.init(argv=list())
        yield self.obj

    def test_ros_params(self, obj):
        print(f"self.rosparams:\n{pformat(self.rosparams)}")
        assert obj.mgr_config.get("update_rate", None) == 20  # Defaults to 10
        assert hasattr(obj, "mgr_config")
        assert "init_timeout" in obj.mgr_config
        assert hasattr(obj, "device_config")
        assert isinstance(obj.device_config, list)
        for devc in obj.device_config:
            if devc["category"] == "bogus_v1_io":
                continue
            assert "param_values" in devc
