import sys
from hw_device_mgr.mgr_ros_hal.mgr import ROSHALHWDeviceMgr


def main(args=None):
    argv = list(sys.argv[1:])  # Knock off executable name
    mgr = ROSHALHWDeviceMgr()
    mgr.init(argv=argv)
    try:
        res = mgr.run()
    except Exception as e:
        res = 1
        raise e
    finally:
        mgr.exit()
        return res


if __name__ == "__main__":
    sys.exit(main())
