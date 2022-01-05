from ...mgr.tests.base_test_class import BaseMgrTestClass
from .bogus_devices.mgr import BogusROSHWDeviceMgr
import pytest


###############################
# Test class


class BaseROSMgrTestClass(BaseMgrTestClass):
    """Base test class for `ROSHWDeviceMgr` class"""

    # Manager class
    device_class = BogusROSHWDeviceMgr

    # Data types
    data_type_class = BogusROSHWDeviceMgr.data_type_class

    # Attached device classes
    device_model_classes = BogusROSHWDeviceMgr.device_classes

    @pytest.fixture
    def manager_ros_params(
        self, mock_rclpy, mgr_config, global_config, request
    ):
        """ROS params for the device manager"""
        hdm_params = dict(
            manager_config=mgr_config,
            device_config=global_config,
            update_rate=20,
            sim=True,
        )
        if not hasattr(request.instance, "rosparams"):
            request.instance.rosparams = dict()
        request.instance.rosparams.update(hdm_params)

    @pytest.fixture
    def device_cls(self, config_cls, manager_ros_params):
        """Fixture for ROS Device classes"""
        self.device_class.clear_devices()
        yield self.device_class

    def test_mock_rclpy_fixture(self, mock_rclpy):
        from ..mgr import rclpy

        node = rclpy.create_node("foo")
        assert node is self.node

        decl = node.declare_parameter("bar", 13)
        assert decl.value == 13
        decl.value = 42
        assert decl.value == 42

        pub = node.create_publisher("foo_pub", "foo_pub_topic")
        print("publishers:", self.publishers)
        assert pub.msg_type == "foo_pub"
        assert self.publishers["foo_pub_topic"] is pub

        sub = node.create_subscription("foo_sub", "foo_sub_topic", "foo_sub_cb")
        print("subscriptions:", self.subscriptions)
        assert sub.msg_type == "foo_sub"
        assert sub.cb == "foo_sub_cb"
        assert self.subscriptions["foo_sub_topic"] is sub

        srv = node.create_service("foo_srv", "foo_srv", "foo_srv_cb")
        print("services:", self.services)
        assert srv.srv_type == "foo_srv"
        assert srv.cb == "foo_srv_cb"
        assert self.services["foo_srv"] is srv
