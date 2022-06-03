from ..mgr.mgr import HWDeviceMgr, SimHWDeviceMgr
from ..config_io import ConfigIO
import rclpy
import traceback


class ROSHWDeviceMgr(HWDeviceMgr, ConfigIO):
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
        device_config = self.load_yaml_path(device_config_path)
        assert device_config, f"Empty YAML file '{device_config_path}'"
        super().init_devices(device_config=device_config, **kwargs)

    def init_sim_from_rosparams(self, **kwargs):
        sim_device_data_path = self.get_param("sim_device_data_path")
        assert sim_device_data_path, "No 'sim_device_data_path' param defined"
        sim_device_data = self.load_yaml_path(sim_device_data_path)
        assert sim_device_data, f"Empty YAML file '{sim_device_data_path}'"
        self.init_sim(sim_device_data=sim_device_data, **kwargs)

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
        try:
            rclpy.spin(self.ros_node)
        except KeyboardInterrupt:
            self.logger.warning("Caught KeyboardInterrupt")
        self.logger.info("Shutting down")
        rclpy.shutdown()


class ROSSimHWDeviceMgr(ROSHWDeviceMgr, SimHWDeviceMgr):
    # For tests
    pass
