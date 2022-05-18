from rclpy import logging
from . import Logging
from rclpy.logging import LoggingSeverity


class ROSLogging(Logging):
    """Wrapper for `rclpy.logging` object."""

    def __init__(self, name):
        self._logger = logging.get_logger(name)

    # Translate Python str levels to rclpy str levels (int levels are
    # the same)
    _rclpy_level_map = dict(
        critical=LoggingSeverity.FATAL,
        error=LoggingSeverity.ERROR,
        warning=LoggingSeverity.WARN,
        info=LoggingSeverity.INFO,
        debug=LoggingSeverity.DEBUG,
        notset=LoggingSeverity.UNSET,
    )

    def setLevel(self, level):
        if isinstance(level, str):
            level = self._rclpy_level_map.get(level.lower(), None)
            assert level is not None, f"Invalid log level '{level}'"
        self._logger.set_level(level)

    def getLevel(self):
        return self._logger.get_effective_level()
