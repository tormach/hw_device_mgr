import logging

try:
    import rosgraph

    if rosgraph.is_master_online():
        import rospy

        HAVE_ROSPY = True
    else:
        HAVE_ROSPY = False
except Exception:
    HAVE_ROSPY = False


class Logging:
    """Logger class that flexibly uses `rospy` when available, falling
    back to `logging`
    """

    # Dispatchers for logging methods
    _dispatcher = {
        True: dict(  # rospy available:  `rospy` module methods
            debug="logdebug",
            info="loginfo",
            warn="logwarn",
            err="logerr",
            fatal="logfatal",
        ),
        False: dict(  # rospy unavailable:  `logging` instance methods
            debug="debug",
            info="info",
            warn="warning",
            err="error",
            fatal="critical",
        ),
    }

    # Map `rospy` log levels to `logging` levels
    _rospy_level_map = dict(
        DEBUG="DEBUG",
        INFO="INFO",
        WARN="WARNING",
        ERR="ERROR",
        FATAL="CRITICAL",
    )

    def __init__(self, name="root", level="INFO"):
        self.name = name
        if HAVE_ROSPY:
            self.use_rospy = rosgraph.is_master_online()
        else:
            self.use_rospy = False
        self._logger = None
        self.setLevel(level)

    def _get_logger(self):
        if self._logger is not None:
            return self._logger
        if self.use_rospy:
            logger = rospy
        else:
            logger = logging.getLogger(self.name)
            if hasattr(logger, "setFormatter"):  # RospyLogger doesn't
                formatter = logging.Formatter(
                    "%(name)s:%(levelname)s: %(message)s"
                )
                logger.setFormatter(formatter)
        self._logger = logger
        return logger

    def setLevel(self, level):
        if self.use_rospy:
            # rospy log level is set externally
            return
        logging_level = self._rospy_level_map[level]
        self._get_logger().setLevel(logging_level)
        # The ROS environment causes `logging.getLogger()` to
        # return `<RospyLogger hw_device_mgr.params.commands (level)>`
        # objects, which don't work without the following line.
        # :P
        logging.basicConfig(level=logging_level)

    def __getattr__(self, name):
        if name == "logger":
            return self._get_logger()
        if name in self._dispatcher[True]:
            return getattr(self.logger, self._dispatcher[self.use_rospy][name])
        raise AttributeError(f"'Logging' object has no attribute '{name}'")

    @classmethod
    def getLogger(cls, name):
        # Look like Python Logging.getLogger()
        return cls(name)
