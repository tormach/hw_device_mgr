import logging


class Logging:
    """Wrapper for `logging` module."""

    _logging_class = logging

    def __init__(self, name):
        lc = self._logging_class
        lo = self._logger = lc.getLogger(name)
        lh = self._log_handler = lc.StreamHandler()
        lf = lc.Formatter("%(asctime)s [%(levelname)s]%(name)s: %(message)s")
        lh.setFormatter(lf)
        lo.addHandler(lh)

    # Translate Python str levels to str levels (int levels are the same)
    _level_map = dict(
        fatal="fatal",
        error="error",
        warning="warning",
        info="info",
        debug="debug",
        notset="notset",
    )

    def setLevel(self, level):
        if isinstance(level, str):
            level = self._level_map.get(level.upper(), level).upper()
        self._logger.setLevel(level)

    def getLevel(self):
        return self._logger.getEffectiveLevel()

    def __getattr__(self, name):
        if name in self._level_map:
            return getattr(self._logger, self._level_map[name])
        if name.lower() in self._level_map:
            attr = self._level_map[name.lower()].upper()
            return getattr(self._logging_class, attr)
        raise AttributeError(f"'Logging' object has no attribute '{name}'")

    @classmethod
    def getLogger(cls, name):
        return cls(name)
