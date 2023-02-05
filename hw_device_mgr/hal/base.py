import hal


class HALMixin:
    """
    A Machinekit HAL-related mixin class.

    Provides `hal` enums as class attrs, e.g. `HAL_IN`, `HAL_OUT`,
    `HAL_BIT`, `HAL_U32`, and methods for smoother HAL integration.
    """

    # - Set class attrs: HAL_IN, HAL_OUT, HAL_BIT, HAL_U32, etc. `hal` enums
    # - Set up int to str map
    _hal_enum_to_str_map = dict()
    for name in dir(hal):
        val = getattr(hal, name)
        if not name.startswith("HAL_") or not isinstance(val, int):
            continue
        locals()[name] = val
        _hal_enum_to_str_map[val] = name

    @classmethod
    def hal_enum(cls, x):
        """
        Translate a `string` or `int` to a HAL enum.

        E.g. `in`, `IN` or `16` -> `hal.HAL_IN`.
        """
        return getattr(hal, f"HAL_{x.upper()}") if isinstance(x, str) else x

    @classmethod
    def hal_enum_str(cls, x):
        """
        Translate a HAL enum to a `string`.

        E.g. `3`, `hal.HAL_S32` -> `HAL_S32`
        """
        return cls._hal_enum_to_str_map[x]

    @staticmethod
    def pname_to_attr(pname):
        """
        Munge a HAL pin name to be suitable as a Python name.

        E.g. `control-word` -> `control_word`
        """
        return pname.replace("-", "_")

    def hal_component(self, name):
        self.comp = hal.component(name)
        self.compname = name

    def hal_ready(self):
        self.comp.ready()
