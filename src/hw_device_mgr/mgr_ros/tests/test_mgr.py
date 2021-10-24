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

    def test_mock_rospy_fixture(self, mock_rospy):
        from ..mgr import rospy

        assert rospy.get_param is mock_rospy.get_param
        assert rospy.init_node is mock_rospy.init_node

    def test_ros_params(self, obj):
        print(f"self.ros_params:\n{pformat(self.ros_params)}")
        assert self.ros_params["hw_device_mgr"]["sim"] is True  # Fixture sanity
        assert obj.sim is True  # Defaults to False
        assert obj.update_rate == 20  # Defaults to 10
        assert hasattr(obj, "mgr_config")
        assert "init_timeout" in obj.mgr_config
        assert hasattr(obj, "device_config")
        assert isinstance(obj.device_config, list)
        assert "param_values" in obj.device_config[0]
