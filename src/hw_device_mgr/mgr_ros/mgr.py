from ..mgr.mgr import HWDeviceMgr
import rclpy
import traceback


class ROSHWDeviceMgr(HWDeviceMgr):

    ros_param_base = "hw_device_mgr"

    def get_param(self, name, default=None):
        return self.ros_node.declare_parameter(name, value=default).value

    def init(self, ros_args=[], **kwargs):
        """Initialize manager instance

        Init ROS node, shutdown callback and rate object, and read
        manager config from ROS params, in addition to base class init
        """
        # - Init ROS node
        rclpy.init(args=ros_args)
        self.ros_node = rclpy.create_node(self.name)
        self.ros_context = rclpy.utilities.get_default_context()
        self.logger.info(f"Initializing '{self.name}' ROS node")
        # - ROS update rate
        self.update_rate = self.get_param("update_rate", 10)
        # - ROS params
        self.sim = self.get_param("sim", False)
        mgr_config = self.get_param("manager_config")
        super().init(mgr_config=mgr_config, **kwargs)
        self.logger.info(f"Initialized '{self.name}' ROS node")

    def init_devices(self, **kwargs):
        """Read device config from ROS params, and init devices as usual"""
        device_config = self.get_param("device_config")
        super().init_devices(device_config=device_config, **kwargs)

    def read_update_write(self):
        """
        Add interrupt handling and fast-tracking to
        `read_update_write()`

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
        """Program main loop

        Let `rclpy.node.Node` spinner manage looping and exit"""
        self.ros_node.create_timer(1 / self.update_rate, self.read_update_write)
        rclpy.spin(self.ros_node)
        self.logger.info("Shutting down")
        rclpy.shutdown()
