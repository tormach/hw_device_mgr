import pytest
from hw_device_mgr.pins import HALBase, HALPin, HALPins


class TestHALBase:
    tc = HALBase

    hal_enums = dict(
        HAL_BIT=1,
        HAL_FLOAT=2,
        HAL_S32=3,
        HAL_U32=4,
        HAL_S64=5,
        HAL_U64=6,
        HAL_IN=16,
        HAL_OUT=32,
        HAL_IO=48,
        HAL_RO=64,
        HAL_RW=192,
    )

    def test_hal_enum_values(self):
        for name, val in self.hal_enums.items():
            assert getattr(self.tc, name) == val
            assert self.tc._hal_enum_to_str_map[val] == name

    def test_hal_enum(self):
        for name, val in self.hal_enums.items():
            # Ints should stay ints
            assert self.tc.hal_enum(val) == val
            # 'in' -> 16
            assert self.tc.hal_enum(name[4:].lower()) == val
            # 'IN' -> 16
            assert self.tc.hal_enum(name[4:].upper()) == val

    def test_hal_name(self):
        for name, val in self.hal_enums.items():
            assert self.tc.hal_name(val) == name


class TestHALPin:
    tc = HALPin

    hal_enums = TestHALBase.hal_enums

    @pytest.fixture
    def obj1(self, mock_halcomp):
        self.name1 = 'pin-a'
        self.ptype1 = self.tc.HAL_S32
        self.ptype_raw1 = 'S32'
        self.pdir1 = self.tc.HAL_IN
        self.pdir_raw1 = 'in'
        obj1 = self.tc(
            mock_halcomp, self.name1, self.ptype_raw1, self.pdir_raw1
        )
        self.obj1 = obj1
        yield obj1

    @pytest.fixture
    def obj2(self, mock_halcomp):
        self.name2 = 'my-hal-pin1'
        self.ptype2 = self.tc.HAL_BIT
        self.ptype_raw2 = 'bit'
        self.pdir_raw2 = self.pdir2 = self.tc.HAL_OUT
        obj2 = self.tc(
            mock_halcomp, self.name2, self.ptype_raw2, self.pdir_raw2
        )
        self.obj2 = obj2
        yield obj2

    @pytest.fixture
    def obj3(self, mock_halcomp):
        self.name3 = 'x'
        self.ptype3 = self.tc.HAL_FLOAT
        self.ptype_raw3 = 'float'
        self.pdir_raw3 = self.pdir3 = self.tc.HAL_IO
        obj3 = self.tc(
            mock_halcomp, self.name3, self.ptype_raw3, self.pdir_raw3
        )
        self.obj3 = obj3
        yield obj3

    def test_init(self, obj1, obj2, obj3):
        for i in range(1, 4):
            obj = getattr(self, f'obj{i}')
            ptype = getattr(self, f'ptype{i}')
            pdir = getattr(self, f'pdir{i}')
            name = getattr(self, f'name{i}')
            print(obj, ptype, pdir, name)
            assert obj.comp is self.mock_halcomp
            assert obj.ptype == ptype
            assert obj.pdir == pdir
            assert obj.hal_pin is self.mock_halcomp.get_pin(name)
            self.mock_halcomp.newpin.assert_any_call(name, ptype, pdir)

    def test_set(self, obj1):
        assert obj1.val == 0  # Initial value
        obj1.set(42)
        assert obj1.val == 42
        obj1.set(4)
        assert obj1.val == 4
        assert self.mock_halcomp.get_pin_val(self.name1) == 0

    def test_get(self, obj2):
        obj2.set(True)
        assert obj2.val is True
        obj2.set(False)
        assert obj2.val is False
        obj2.set(True)
        assert self.mock_halcomp.get_pin_val(self.name2) == 0

    def test_read_write(self, obj1, obj2, obj3):
        for i in range(1, 4):
            obj = getattr(self, f'obj{i}')
            name = getattr(self, f'name{i}')

            # Test initial values
            assert (obj.val, obj.old_val) == (0, 0)
            assert self.mock_halcomp.get_pin_val(name) == 0

            # Set obj & pin values differently, read & test
            self.mock_halcomp.set_pin_val(name, 1)
            obj.set(0)
            obj.read()
            if obj.pdir == obj.HAL_OUT:
                obj.hal_pin.get.assert_not_called()
                assert obj.get() == 0
            else:
                obj.hal_pin.get.assert_called()
                assert obj.get() == 1

            # Set obj & pin values differently, write & test
            self.mock_halcomp.set_pin_val(name, 1)
            obj.set(0)
            obj.write()
            if obj.pdir == obj.HAL_IN:
                obj.hal_pin.set.assert_not_called()
                assert self.mock_halcomp.get_pin_val(name) == 1
            else:
                obj.hal_pin.set.assert_called()
                assert self.mock_halcomp.get_pin_val(name) == 0

    def test_changed_in(self, obj1):
        # Input pins change value from HAL pin after read()
        self.mock_halcomp.set_pin_val(self.name1, 999)
        obj1.read()
        assert obj1.changed
        self.mock_halcomp.set_pin_val(self.name1, 888)
        obj1.read()
        assert obj1.changed
        obj1.read()
        assert not obj1.changed
        obj1.read()
        assert not obj1.changed

    def test_changed_out(self, obj2):
        # Output pins change HAL pin value after write()
        obj2.set(999)
        assert obj2.changed
        obj2.write()
        print(f"obj2.val {obj2.val}  obj2.old_val {obj2.old_val}")
        assert not obj2.changed
        obj2.set(888)
        assert obj2.changed
        obj2.write()
        assert not obj2.changed
        obj2.write()
        assert not obj2.changed

    def test_changed_io(self, obj3):
        # Output pins change HAL pin value after read() and write()
        obj3.set(999)
        assert obj3.changed
        obj3.write()
        print(f"obj3.val {obj3.val}  obj3.old_val {obj3.old_val}")
        assert not obj3.changed

        self.mock_halcomp.set_pin_val(obj3.name, 888)
        obj3.read()
        assert obj3.changed
        obj3.read()
        assert not obj3.changed

    def test_repr(self, obj1):
        assert isinstance(obj1.__repr__(), str)


class TestHALPins:
    tc = HALPins

    hal_enums = TestHALBase.hal_enums

    pin_specs = {
        'in-s32': dict(ptype='s32', pdir='in'),
        'out-float': dict(ptype='float', pdir='out'),
        'io-bit': dict(ptype='bit', pdir='io'),
    }
    pin_prefix = 'prefix-'

    test_cases = [
        dict(in_s32=17, out_float=True),
        dict(in_s32=0, out_float=False),
        dict(in_s32=-30, out_float=False),
        dict(in_s32=-31000, out_float=True),
    ]

    def test_init(self, mock_halcomp):
        pins = self.tc(mock_halcomp, self.pin_specs, prefix=self.pin_prefix)
        assert pins.comp is mock_halcomp
        assert pins.pin_specs is self.pin_specs
        assert pins.prefix == self.pin_prefix

    def test_pname_to_attr(self):
        test_cases = [
            ('pin', 'pin'),
            ('PIN1', 'PIN1'),
            ('pin-x-y', 'pin_x_y'),
        ]
        for pname, attr in test_cases:
            assert self.tc.pname_to_attr(pname) == attr

    @pytest.fixture
    def obj(self, mock_halcomp):
        obj = self.tc(mock_halcomp, self.pin_specs, prefix=self.pin_prefix)
        obj.init_pins()
        yield obj

    def test_init_pins(self, obj):
        assert hasattr(obj, 'pin_dict')
        assert isinstance(obj.pin_dict, dict)
        for pname in self.pin_specs:
            data = self.pin_specs[pname]
            (ptype, pdir) = (
                self.tc.hal_enum(data[a]) for a in ('ptype', 'pdir')
            )

            assert pname in obj.pin_dict
            assert (
                getattr(obj, self.tc.pname_to_attr(pname), None)
                is obj.pin_dict[pname]
            )

            pin = obj.pin_dict[pname]
            assert pin.name == self.pin_prefix + pname
            assert pin.ptype == ptype
            assert pin.pdir == pdir
            assert pin.comp is self.mock_halcomp
            assert self.pin_prefix + pname in self.mock_halcomp.pin_names()

    def test_set(self, obj):
        for test_case in self.test_cases:
            for pattr, val in test_case.items():
                pin = getattr(obj, pattr)
                pname = pin.name[len(self.pin_prefix) :]
                obj.set(pname, val)
                assert pin.val == val

    def test_get(self, obj):
        for test_case in self.test_cases:
            for pattr, val in test_case.items():
                pin = getattr(obj, pattr)
                pin.set(val)
                assert obj.get(pin.name[len(self.pin_prefix) :]) == val

    def test_read_all(self, obj):
        for test_case in self.test_cases:
            for pattr, val in test_case.items():
                pin = getattr(obj, pattr)
                pin.hal_pin.set(val)  # HAL pin val copied to pin obj
            obj.read_all()
            for pattr, val in test_case.items():
                pin = getattr(obj, pattr)
                if pin.pdir != pin.HAL_OUT:
                    assert pin.get() == val
                else:
                    pin.hal_pin.get.assert_not_called()
                    assert pin.get() == 0

    def test_write_all(self, obj):
        for test_case in self.test_cases:
            for pattr, val in test_case.items():
                pin = getattr(obj, pattr)
                pin.set(val)  # Pin obj val copied to HAL pin
            obj.write_all()
            for pattr, val in test_case.items():
                pin = getattr(obj, pattr)
                if pin.pdir != pin.HAL_IN:
                    pin.hal_pin.set.assert_called()
                    assert pin.hal_pin.get() == val
                else:
                    pin.hal_pin.set.assert_not_called()
                    assert pin.hal_pin.get() == 0
