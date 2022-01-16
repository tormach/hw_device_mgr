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
    def obj(self, device_cls, device_config, tmp_path, all_device_data):
        # init() and init_devices() signatures changed, so can't use
        # parent test class obj fixture
        tmpfile = tmp_path / "device_config.yaml"
        # Munge data for pyyaml
        device_config = [dc.copy() for dc in device_config]
        for dc in device_config:
            dc["product_code"] = int(dc["product_code"])
            dc["vendor_id"] = int(dc["vendor_id"])
        with open(tmpfile, "w") as f:
            f.write(yaml.safe_dump(device_config))
        self.obj = device_cls(device_data=all_device_data.values())
        self.obj.init(list())
        self.obj.init_devices(tmpfile)
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
