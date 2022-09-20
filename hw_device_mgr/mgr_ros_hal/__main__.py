import sys
from hw_device_mgr.mgr_ros_hal.mgr import ROSHALHWDeviceMgr


def main(args=None):
    argv = list(sys.argv[1:])  # Knock off executable name
    sim = "--sim" in argv
    mgr = ROSHALHWDeviceMgr()
    mgr.init(argv)
    if sim:
        mgr.init_sim_from_rosparams()
    mgr.init_devices()
    mgr.run()


if __name__ == "__main__":
    sys.exit(main())
