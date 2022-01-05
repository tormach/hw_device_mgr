from rclpy import logging


class Logging:
    """Wrapper for `rclpy.logging` object"""

    def __init__(self, name):
        self._logger = logging.get_logger(name)

    # Translate Python str levels to rclpy str levels (int levels are
    # the same)
    _rclpy_level_map = dict(
        critical="fatal",
        error="error",
        warning="warn",
        info="info",
        debug="debug",
        notset="unset",
    )

    def setLevel(self, level):
        if isinstance(level, str):
            level = self._rclpy_level_map.get(level.upper(), level).upper()
        self._logger.set_logger_level(level)

    def __getattr__(self, name):
        if name in self._rclpy_level_map:
            return getattr(self._logger, self._rclpy_level_map[name])
        raise AttributeError(f"'Logging' object has no attribute '{name}'")

    @classmethod
    def getLogger(cls, name):
        return cls(name)
