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

    def init(self, /, comp, **kwargs):
        super().init(**kwargs)
        self.comp = comp

        # Get specs for all pins in all interfaces; shared pin names must match,
        # except for direction, which becomes HAL_IO if different
        all_specs = dict()
        for iface, params in self.pin_interfaces.items():
            for base_pname, new_spec in self.iface_pin_specs(iface).items():
                if base_pname not in all_specs:
                    all_specs[base_pname] = new_spec
                    continue
                spec = all_specs[base_pname]
                for key, new_val in new_spec.items():
                    if key == "pdir" and spec["pdir"] != new_val:
                        spec["pdir"] = self.HAL_IO
                        continue
                    if spec[key] != new_val:
                        raise RuntimeError(
                            f"Two interfacess' pin '{base_pname}'"
                            f" spec '{key}' differs,"
                            f" '{spec[key]}' != '{new_val}'"
                        )
                all_specs[base_pname] = spec

        # Create all pins from specs
        self.pins = dict()
        for base_pname, specs in all_specs.items():
            pname, ptype, pdir = specs["pname"], specs["ptype"], specs["pdir"]
            try:
                pin = comp.newpin(pname, ptype, pdir)
            except Exception as e:
                raise RuntimeError(
                    f"Exception creating pin {comp.getprefix()}.{pname}:  {e}"
                )
            ptypes, pdirs = (self.hal_enum_str(i) for i in (ptype, pdir))
            self.pins[base_pname] = pin
            self.logger.debug(f"Created HAL pin {pname} {ptypes} {pdirs}")

    def iface_pin_specs(self, iface):
        iface_pdir = self.pin_interfaces[iface][0]
        data_types = self.merge_dict_attrs(f"{iface}_data_types")
        res = dict()
        for base_pname, data_type_name in data_types.items():
            pname = self.pin_name(iface, base_pname)
            ptype = self.data_type_class.by_shared_name(data_type_name).hal_type
            res[pname] = dict(pname=pname, ptype=ptype, pdir=iface_pdir)
        return res

    def read(self):
        # Read from HAL pins
        for pin_iface, params in self.pin_interfaces.items():
            if params[0] == self.HAL_OUT:
                continue  # Only read HAL_IN, HAL_IO pins
            iface_vals = {
                p: self.pins[self.pin_name(pin_iface, p)].get()
                for p in self.interface(pin_iface).get()
            }
            self.interface(pin_iface).set(**iface_vals)

    def write(self):
        # Write to output pins
        super().write()
        for pin_iface, params in self.pin_interfaces.items():
            if params[0] == self.HAL_IN:
                continue  # Only write HAL_OUT, HAL_IO pins
            for name, val in self.interface(pin_iface).get().items():
                pname = self.pin_name(pin_iface, name)
                self.pins[pname].set(val)


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
        self.logger.info(f"Initialized '{self.compname}' HAL component")
        super().init(comp=self.comp, **kwargs)

    def hal_comp_ready(self):
        self.comp.ready()
        self.logger.info("%s: HAL component ready" % self.name)
