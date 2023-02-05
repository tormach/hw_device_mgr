from ...tests.fixtures import MockFixture


class MockHALPin(MockFixture):
    """A `MockFixture` subclass that emulates a HAL pin."""

    fixture_name = "MockHALPin"
    instance_attr = "pname"
    mock_methods = ["get", "set"]

    def __init__(self, pname, ptype, pdir, storage=None):
        """Emulate `hal_comp.newpin()`."""
        self.pname = pname
        self.ptype = ptype
        self.pdir = pdir

    def _set_storage(self, storage):
        # Init storage outside constructor to reduce test logging mess
        self.storage = storage
        self.storage[self.pname] = 0

    def set(self, val):
        """Mock HAL pin setter."""
        print(f"MockHALPin {self.pname}: set({val}/0x{val:X})")
        self.storage[self.pname] = val

    def get(self):
        """Mock HAL pin getter."""
        val = self.storage[self.pname]
        print(f"MockHALPin {self.pname}: get() -> {val}/0x{val:X}")
        return val


class MockHALComponent(MockFixture):
    """A `MockFixture` subclass that emulates a HAL component."""

    fixture_name = "MockHALComponent"
    instance_attr = "name"
    mock_methods = [
        "ready",
        "getprefix",
        "newpin",
        "pin_names",
        "get_pin",
        "set_pin_val",
        "get_pin_val",
        "get_pin_type",
        "get_pin_dir",
    ]
    mock_attrs = ["is_ready"]
    patches = "hal.component"

    def __init__(self, name, pin_vals):
        """Emulate `hal.component()`."""
        self.name = name
        self.is_ready = False
        self.pins = {}
        self.pin_vals = pin_vals
        print(f"Created {self}")

    def ready(self):
        """Emulate `hal_comp.ready`."""
        print(f"MockHALComponent {self.name}: ready()")
        self.is_ready = True

    def getprefix(self):
        """Emulate `hal_comp.getprefix`."""
        # print(f"MockHALComponent {self.name}: getprefix() -> {self.name}")
        return self.name

    def newpin(self, *args):
        """Emulate `hal_comp.newpin`."""
        mock_obj = MockHALPin.get_mock(*args)
        pin = mock_obj.obj
        pin._set_storage(self.pin_vals)
        self.pins[pin.pname] = mock_obj
        print(f"MockHALComponent {self.name}: newpin({args})")
        return mock_obj

    def pin_names(self):
        """Fixture method:  Return component's pins' names."""
        return self.pins.keys()

    def get_pin(self, pname):
        """Fixture method:  Return a component's pin's object."""
        return self.pins.get(pname, None)

    def set_pin_val(self, pname, val):
        """Fixture method:  Set a component's pin's value."""
        print(f"fixture set_pin_val({pname}, {val})")
        self.pin_vals[pname] = val

    def get_pin_val(self, pname):
        """Fixture method:  Get a component's pin's value."""
        val = self.pin_vals[pname]
        print(f"fixture get_pin_val({pname}) = {val}")
        return val

    def get_pin_type(self, pname):
        """Fixture method:  Get a component's pin's type."""
        return self.pins[pname].obj.ptype

    def get_pin_dir(self, pname):
        """Fixture method:  Get a component's pin's direction."""
        return self.pins[pname].obj.pdir

    @classmethod
    def tweak_fixture(cls, mock_obj, obj):
        """
        Add `get_pin(pname)` and `set_pin(pname, val)`.

        For testing object instance.
        """
        obj.request_inst.get_pin = obj.get_pin_val
        obj.request_inst.set_pin = obj.set_pin_val
        assert not hasattr(obj, "halcomp_mockobj")
        obj.halcomp_mockobj = mock_obj
