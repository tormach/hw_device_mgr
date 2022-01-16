from ...mgr.tests.base_test_class import BaseMgrTestClass
from .bogus_devices.mgr import BogusROSHWDeviceMgr
import pytest


###############################
# Test class


class BaseROSMgrTestClass(BaseMgrTestClass):
    """Base test class for `ROSHWDeviceMgr` class."""

    rclpy_patches = ("hw_device_mgr.mgr_ros.mgr.rclpy", "bogus")

    # Manager class
    device_class = BogusROSHWDeviceMgr

    # Data types
    data_type_class = BogusROSHWDeviceMgr.data_type_class

    # Base class for attached devices
    device_base_class = BogusROSHWDeviceMgr.device_base_class

    # Attached device classes
    device_model_classes = BogusROSHWDeviceMgr.device_classes

    @pytest.fixture
    def extra_fixtures(self, manager_ros_params, mock_rclpy):
        pass

    @pytest.fixture
    def manager_ros_params(self, mock_rclpy, mgr_config):
        """ROS params for the device manager."""
        hdm_params = dict(
            update_rate=20,
            use_sim=True,
        )
        hdm_params.update(mgr_config)
        hdm_params.pop("devices")
        self.rosparams.update(hdm_params)

    def test_mock_rclpy_fixture(self, mock_rclpy):
        from ..mgr import rclpy

        node = rclpy.create_node("foo")
        assert node is self.node

        decl = node.declare_parameter("bar", 13)
        assert decl.value == 13
        decl.value = 42
        assert decl.value == 42

        pub = node.create_publisher(int, "foo_pub_topic", 1)
        print("publishers:", self.publishers)
        assert pub.msg_type is int
        assert self.publishers["foo_pub_topic"] is pub

        sub = node.create_subscription(float, "foo_sub_topic", "foo_sub_cb")
        print("subscriptions:", self.subscriptions)
        assert sub.msg_type is float
        assert sub.cb == "foo_sub_cb"
        assert self.subscriptions["foo_sub_topic"] is sub

        srv = node.create_service(str, "foo_srv", "foo_srv_cb")
        print("services:", self.services)
        assert srv.srv_type is str
        assert srv.cb == "foo_srv_cb"
        assert self.services["foo_srv"] is srv
