from ...mgr.tests.base_test_class import BaseMgrTestClass
import pytest

try:
    import rclpy  # noqa: F401
except ModuleNotFoundError:
    pytest.skip(allow_module_level=True)
from .bogus_devices.mgr import ROSHWDeviceMgrTest


###############################
# Test class


class BaseROSMgrTestClass(BaseMgrTestClass):
    """Base test class for `ROSHWDeviceMgr` class."""

    rclpy_patches = ("hw_device_mgr.mgr_ros.mgr.rclpy", "bogus")

    # Manager class
    device_class = ROSHWDeviceMgrTest

    # Data types
    data_type_class = ROSHWDeviceMgrTest.data_type_class

    # Base class for attached devices
    device_base_class = ROSHWDeviceMgrTest.device_base_class

    # Attached device classes
    device_model_classes = ROSHWDeviceMgrTest.device_classes

    @pytest.fixture
    def extra_fixtures(
        self, manager_ros_params, sim_device_data_path, device_config_path
    ):
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
        print(f"manager_ros_params fixture:  rosparams = {self.rosparams}")

    @pytest.fixture
    def sim_device_data_path(self, tmp_path, mock_rclpy):
        # Dump sim_device_data into YAML file & point `sim_device_data_path` ROS
        # param to it
        tmpfile = tmp_path / "sim_devices.yaml"
        print(f"Sim device data written to {tmpfile}")
        sim_device_data = self.init_sim_device_data()
        # Clean up for YAML dumper
        for d in sim_device_data:
            d["product_code"] = int(d["product_code"])
            d["vendor_id"] = int(d["vendor_id"])
        self.dump_yaml_path(tmpfile, sim_device_data)
        self.rosparams["sim_device_data_path"] = tmpfile
        yield tmpfile

    @pytest.fixture
    def device_config_path(self, tmp_path, device_config, mock_rclpy):
        # Clean data types from device config to dump into YAML file &
        # set `device_config_path` ROS param
        device_config = [dc.copy() for dc in device_config]
        assert device_config
        for dc in device_config:
            dc["product_code"] = int(dc["product_code"])
            dc["vendor_id"] = int(dc["vendor_id"])
        tmpfile = tmp_path / "device_config.yaml"
        self.dump_yaml_path(tmpfile, device_config)
        self.rosparams["device_config_path"] = tmpfile
        print(f"Cleaned device config written to {tmpfile}")
        yield tmpfile

    def test_mock_rclpy_fixture(self, mock_rclpy):
        from ..mgr import rclpy  # noqa:  F811

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
