from ..mgr.mgr import HWDeviceMgr, SimHWDeviceMgr
import rclpy
import yaml
import os
import traceback


class ROSHWDeviceMgr(HWDeviceMgr):

    name = "ros_hw_device_mgr"

    def get_param(self, name, default=None):
        if self.ros_node.has_parameter(name):
            param = self.ros_node.get_parameter(name)
        else:
            param = self.ros_node.declare_parameter(name, value=default)
        return param.value

    def init(self, args, **kwargs):
        """
        Initialize manager instance.

        Init ROS node, shutdown callback and rate object, and read
        manager config from ROS params, in addition to base class init
        """
        # - Init ROS node
        node_kwargs = dict(
            allow_undeclared_parameters=True,
            automatically_declare_parameters_from_overrides=True,
        )

        rclpy.init(args=args)
        self.ros_node = rclpy.create_node(self.name, **node_kwargs)
        self.ros_context = rclpy.utilities.get_default_context()
        self.logger.info(f"Initializing '{self.name}' ROS node")
        # - Sim mode
        # FIXME Use this ROS param when HAL ros_launch implemented
        # self.sim = self.get_param("use_sim", False)
        # - ROS update rate
        self.update_rate = self.get_param("update_rate", 10)
        # - mgr_config
        mgr_config = dict(
            goal_state_timeout=self.get_param("goal_state_timeout", 2),
            init_timeout=self.get_param("init_timeout", 5),
        )
        super().init(mgr_config=mgr_config, **kwargs)
        self.logger.info(f"Initialized '{self.name}' ROS node")

    def init_devices(self, **kwargs):
        device_config_path = self.get_param("device_config_path")
        assert device_config_path, "No 'device_config_path' param defined"
        self.logger.info(f"Reading device config from '{device_config_path}'")
        assert os.path.exists(device_config_path)
        with open(device_config_path, "r") as f:
            device_config = yaml.safe_load(f)
        assert device_config
        super().init_devices(device_config=device_config, **kwargs)

    def read_update_write(self):
        """
        Add interrupt handling and fast-tracking to `read_update_write`.

        The `rclpy.node.Node` spinner can handle the main loop and
        shutdown.  This method moves generic `Exception` handling and
        fast-tracking here from the non-ROS `run` loop.
        """
        loop = True
        while loop:
            try:
                super().read_update_write()
            except Exception:
                # Ignore other exceptions & enter fault mode in
                # hopes we can recover
                self.logger.error("Ignoring unexpected exception; details:")
                for line in traceback.format_exc().splitlines():
                    self.logger.error(line)
                self.command_in.set(
                    state_cmd="fault", msg_log="Unexpected exception"
                )
            if self.fast_track:
                # This update included a state transition; skip
                # the `sleep()` before the next update
                loop = True
                self.fast_track = False
            else:
                loop = False

    def run(self):
        """
        Program main loop.

        Let `rclpy.node.Node` spinner manage looping and exit
        """
        self.ros_node.create_timer(1 / self.update_rate, self.read_update_write)
        rclpy.spin(self.ros_node)
        self.logger.info("Shutting down")
        rclpy.shutdown()


class SimROSHWDeviceMgr(ROSHWDeviceMgr, SimHWDeviceMgr):
    name = "sim_ros_hw_device_mgr"

    def init_sim(self):
        device_data_path = self.get_param("device_data_path")
        assert device_data_path, "No 'device_data_path' param defined"
        assert os.path.exists(
            device_data_path
        ), f"Device data path doesn't exist:  '{device_data_path}'"
        self.logger.info(f"Reading sim device config from {device_data_path}")
        with open(device_data_path, "r") as f:
            device_data = yaml.safe_load(f)
        super().init_sim(device_data=device_data)
