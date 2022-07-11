import sys
from hw_device_mgr.mgr_ros_hal.mgr import ROSHALHWDeviceMgr


def main(args=None):
    argv = list(sys.argv[1:])  # Knock off executable name
    mgr = ROSHALHWDeviceMgr()
    mgr.init(argv=argv)
    mgr.run()


if __name__ == "__main__":
    sys.exit(main())
