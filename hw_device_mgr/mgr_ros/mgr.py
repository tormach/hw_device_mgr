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

    def init(self, /, argv, **kwargs):
        """
        Initialize manager instance.

        Init ROS node, shutdown callback and rate object, and read manager
        config and device config YAML path from ROS params, in addition to base
        class init.

        The ROS param `device_config_path` must be a `str` containing the path
        to the device configuration YAML file.

        If the ROS param `sim_device_data_path` is non-empty, it must be a `str`
        containing the path to a YAML file with the sim device configuration.
        """
        # - Init ROS node
        node_kwargs = dict(
            allow_undeclared_parameters=True,
            automatically_declare_parameters_from_overrides=True,
        )

        rclpy.init(args=argv)
        self.ros_node = rclpy.create_node(self.name, **node_kwargs)
        self.ros_context = rclpy.utilities.get_default_context()
        self.logger.info(f"Initializing '{self.name}' ROS node")
        # - mgr_config
        if "mgr_config" in kwargs:
            raise TypeError("unexpected 'mgr_config' argument")
        mgr_config = dict(
            goal_state_timeout=self.get_param("goal_state_timeout", 2),
            init_timeout=self.get_param("init_timeout", 5),
            update_rate=self.get_param("update_rate", 10),
        )
        # - device_config
        if "device_config" in kwargs:
            raise TypeError("unexpected 'device_config' argument")
        device_config_path = self.get_param("device_config_path")
        assert device_config_path, "No 'device_config_path' param defined"
        self.logger.info(f"Reading device config from '{device_config_path}'")
        device_config = self.load_yaml_path(device_config_path)
        assert device_config, f"Empty YAML file '{device_config_path}'"
        # - sim device data
        if "sim_device_data" in kwargs:
            raise TypeError("unexpected 'sim_device_data' argument")
        sim_device_data_path = self.get_param("sim_device_data_path", None)
        if sim_device_data_path is not None:
            sim_device_data = self.load_yaml_path(sim_device_data_path)
            assert sim_device_data, f"Empty YAML file '{sim_device_data_path}'"
            kwargs["sim_device_data"] = sim_device_data
        #
        super().init(
            mgr_config=mgr_config, device_config=device_config, **kwargs
        )
        self.logger.info(f"Initialized '{self.name}' ROS node")

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
        update_period = 1.0 / self.mgr_config.get("update_rate", 10.0)
        self.ros_node.create_timer(update_period, self.read_update_write)
        try:
            rclpy.spin(self.ros_node)
        except KeyboardInterrupt:
            self.logger.warning("Caught KeyboardInterrupt")
        self.logger.info("Shutting down")
        rclpy.shutdown()


class ROSSimHWDeviceMgr(ROSHWDeviceMgr, SimHWDeviceMgr):
    # For tests
    pass
