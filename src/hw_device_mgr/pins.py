import hal


class HALBase:
    # - Set class attrs: HAL_IN, HAL_OUT, HAL_BIT, HAL_U32, etc. `hal` enums
    # - Set up int to str map
    _hal_enum_to_str_map = dict()
    for name in dir(hal):
        val = getattr(hal, name)
        if not name.startswith("HAL_") or not isinstance(val, int):
            continue  # pragma: no cover
        locals()[name] = val
        _hal_enum_to_str_map[val] = name

    # Normalize 'in', 'IN' or hal.HAL_IN to hal.HAL_IN
    @classmethod
    def hal_enum(cls, x):
        return getattr(hal, f"HAL_{x.upper()}") if isinstance(x, str) else x

    # Translate 3 to 'HAL_S32'
    @classmethod
    def hal_name(cls, x):
        return cls._hal_enum_to_str_map[x]

    @staticmethod
    def pname_to_attr(pname):
        return pname.replace("-", "_")


class HALPin(HALBase):
    # A `hal.Pin` wrapper whose value updates only after `read()` or
    # `write()` and whose value's changes are tracked
    def __init__(self, comp, pname, ptype, pdir):
        self.comp = comp
        self.name = pname
        self.ptype = self.hal_enum(ptype)
        self.pdir = self.hal_enum(pdir)
        self.hal_pin = comp.newpin(pname, self.ptype, self.pdir)
        self.val = self.old_val = 0

    def set(self, val):
        self.val = val

    def get(self):
        return self.val

    def read(self):
        if self.pdir == self.HAL_OUT:
            return
        self.old_val = self.val
        self.val = self.hal_pin.get()

    def write(self):
        if self.pdir == self.HAL_IN:
            return
        self.old_val = self.val
        self.hal_pin.set(self.val)

    @property
    def changed(self):
        return self.old_val != self.val

    def __repr__(self):
        ptype = self.hal_name(self.ptype)
        pdir = self.hal_name(self.pdir)
        return f"<HALPin {self.name} {ptype} {pdir}>"


class HALPins(HALBase):
    def __init__(self, comp, pin_specs, prefix=""):
        self.comp = comp
        self.prefix = prefix
        self.pin_specs = pin_specs

    def init_pins(self):
        self.pin_dict = pin_dict = dict()
        for pname, spec in self.pin_specs.items():
            pin = HALPin(
                self.comp, self.prefix + pname, spec["ptype"], spec["pdir"]
            )
            setattr(self, self.pname_to_attr(pname), pin)
            pin_dict[pname] = pin

    def set(self, pname, val):
        self.pin_dict[pname].set(val)

    def get(self, pname):
        return self.pin_dict[pname].get()

    def read_all(self):
        for p in self.pin_dict.values():
            p.read()

    def write_all(self):
        for p in self.pin_dict.values():
            p.write()
