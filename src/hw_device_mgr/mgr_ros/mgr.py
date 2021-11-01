from ..mgr.mgr import HWDeviceMgr
import rospy


class ROSHWDeviceMgr(HWDeviceMgr):

    ros_param_base = "hw_device_mgr"

    def init(self, **kwargs):
        """Initialize manager instance

        Init ROS node, shutdown callback and rate object, and read
        manager config from ROS params, in addition to base class init
        """
        # - Init ROS node
        rospy.init_node(self.name)
        rospy.on_shutdown(self.ros_shutdown)
        ns = rospy.get_namespace()
        rospy.loginfo(f"Initializing '{self.name}' ROS node, ns={ns}")
        # - ROS update rate
        self.update_rate = rospy.get_param(
            f"{self.ros_param_base}/update_rate", 10
        )
        self.rate = rospy.Rate(self.update_rate)
        # - ROS params
        self.sim = rospy.get_param(f"{self.ros_param_base}/sim", False)
        mgr_config = rospy.get_param(f"{self.ros_param_base}/manager_config")
        rospy.loginfo(f"Initialized '{self.name}' ROS node")
        super().init(mgr_config=mgr_config, **kwargs)

    def init_devices(self, **kwargs):
        """Read device config from ROS params, and init devices as usual"""
        device_config = rospy.get_param(f"{self.ros_param_base}/device_config")
        super().init_devices(device_config=device_config, **kwargs)

    def read_update_write(self):
        """Add ROS interrupt handling to `read_update_write()`"""
        try:
            super().read_update_write()
        except rospy.exceptions.ROSInterruptException as e:
            self.logger.info(str(e))
            self.shutdown = True

    def ros_shutdown(self):
        # need to unload the userland component here?
        self.logger.info("Stopping on ROS shutdown signal")
        self.shutdown = True
