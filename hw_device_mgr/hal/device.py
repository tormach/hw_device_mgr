from ..device import Device, SimDevice
from .base import HALMixin
from .data_types import HALDataType
from functools import cached_property


class HALPinDevice(Device, HALMixin):
    """A `Device` with HAL pins attached to feedback, goal and command."""

    data_type_class = HALDataType

    # For these interfaces, create HAL pins with (direction, prefix)
    pin_interfaces = dict(
        feedback_in=(HALMixin.HAL_IN, ""),
        command_out=(HALMixin.HAL_OUT, ""),
    )

    @cached_property
    def compname(self):
        return self.comp.getprefix()

    def pin_name(self, interface, pname):
        return self.pin_prefix + self.pin_interfaces[interface][1] + pname

    @cached_property
    def pin_prefix(self):
        """
        HAL pin prefix for this device.

        Pin prefix is computed by separating numeric components of the
        device `address` string with `.` and adding a final `.`, e.g.
        `(0,5)` -> `0.5.`.
        """
        return f"{self.addr_slug}{self.slug_separator}"

    def init(self, *, comp=None, **kwargs):
        # Set (or ensure) self.comp
        if comp is not None:
            self.comp = comp
        else:
            assert hasattr(self, "comp")  # HALCompDevice already set

        # Run parent init() to populate interfaces
        super().init(**kwargs)

        # Create HAL pins for pin interfaces
        self.pins = {i: dict() for i in self.pin_interfaces}
        self.no_pin_keys = {i: set() for i in self.pins}  # Attrs w/o pins
        for intf_name, intf_pins in self.pins.items():
            self.logger.debug(f"{self} Init HAL pins;  interface {intf_name}:")
            intf = self.interface(intf_name)
            for base_pname in intf.keys():
                dtype = intf.get_data_type(base_pname)
                if not hasattr(dtype, "hal_type"):
                    self.logger.debug(
                        f"Interface '{intf_name}' key '{base_pname}' type"
                        f" '{dtype.name}' not HAL compatible; not creating pin"
                    )
                    self.no_pin_keys[intf_name].add(base_pname)
                    continue
                pname = self.pin_name(intf_name, base_pname)
                ptype = dtype.hal_type
                pdir = self.pin_interfaces[intf_name][0]
                try:
                    pin = self.comp.newpin(pname, ptype, pdir)
                except Exception as e:
                    raise RuntimeError(
                        f"Exception creating pin {self.compname}.{pname}:  {e}"
                    )
                ptypes, pdirs = (self.hal_enum_str(i) for i in (ptype, pdir))
                intf_pins[base_pname] = pin
                self.logger.debug(f"  {pname}:  {ptypes} {pdirs}")

    def read(self):
        # Read from HAL pins
        super().read()
        pins = self.pins["feedback_in"]
        vals = {p: pins[p].get() for p in pins.keys()}
        self.interface("feedback_in").update(**vals)
        if not getattr(self, "read_once", False):
            self.read_once = True
            self.logger.info(
                f"HAL pins read for {self} feedback_in:  {list(pins.keys())}"
            )
            self.logger.info(
                "   Interface keys:  "
                f"{list(self.interface('feedback_in').keys())}"
            )

    def write(self):
        # Write to output pins
        super().write()
        for iface, pins in self.pins.items():
            if self.pin_interfaces[iface][0] == self.HAL_IN:
                continue  # Only write HAL_OUT, HAL_IO pins
            vals = self.interface(iface).get()
            for name, pin in pins.items():
                pin.set(vals[name])


class HALPinSimDevice(HALPinDevice, SimDevice):
    """A `HalPinDevice` with HAL pins attached to sim feedback."""

    # For these interfaces, create HAL pins with (direction, prefix)
    pin_interfaces = dict(
        sim_feedback=(HALMixin.HAL_OUT, "sim_"),
        **HALPinDevice.pin_interfaces,
    )


class HALCompDevice(HALPinDevice):
    """A `Device` with HAL component."""

    hal_comp_name = None

    def init(self, **kwargs):
        self.comp = self.hal.component(self.hal_comp_name or self.name)
        super().init(**kwargs)
        self.comp.ready()
        self.logger.info(f"HAL component '{self.compname}' ready")
