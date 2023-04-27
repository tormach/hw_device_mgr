from .base_test_class import BaseTestClass


class TestDataType(BaseTestClass):
    # All shared types in the base `DataType` class
    all_shared_types = {
        "bit",
        "int8",
        "int16",
        "int32",
        "int64",
        "uint8",
        "uint16",
        "uint32",
        "uint64",
        "float",
        "double",
        "str",
    }

    # Shared types with keys in test class `subtype_data` attribute
    defined_shared_types = all_shared_types

    # Triplets:  (raw_value, value, string value)
    conv_cases = dict(
        bit=(
            (True, 1, "True"),
            (1, 1, "True"),
            (100, 1, "True"),
            ("1", 1, "True"),
            (False, 0, "False"),
            (0, 0, "False"),
            ("0", 0, "False"),
        ),
        int8=(
            (-5, -5, "-5"),
            ("-4", -4, "-4"),
            ("0xff", 255, "255"),
        ),
        int16=(
            (15, 15, "15"),
            ("0xffff", 65535, "65535"),
        ),
        int32=(
            (-150000, -150000, "-150000"),
            ("0xffffffff", 4294967295, "4294967295"),
        ),
        int64=(
            (1500000000, 1500000000, "1500000000"),
            ("0xffffffffffff", 281474976710655, "281474976710655"),
        ),
        uint8=(
            (5, 5, "0x05"),
            ("24", 24, "0x18"),
            ("0xff", 255, "0xFF"),
        ),
        uint16=(
            (15, 15, "0x000F"),
            ("0xffff", 65535, "0xFFFF"),
        ),
        uint32=(
            (150000, 150000, "0x000249F0"),
            ("0xffffffff", 4294967295, "0xFFFFFFFF"),
        ),
        uint64=(
            (0, 0, "0x0000000000000000"),
            ("0xffffffffffff", 281474976710655, "0x0000FFFFFFFFFFFF"),
        ),
        float=(
            (0, 0, "0.0"),
            ("2.6", 2.6, "2.6"),
            ("1e10", 1e10, "10000000000.0"),
        ),
        double=(
            (-400, -400, "-400.0"),
            ("1.55e-2", 1.55e-2, "0.0155"),
            ("1e99", 1e99, "1e+99"),
        ),
        str=(
            ("", "", ""),
            ("foo", "foo", "foo"),
        ),
    )

    def test_class_generation(self):
        # Test some of the funky inheritance and class generation
        # going on here
        print(f"test class:  {self.data_type_class}")
        parent_base_cls = super(self.data_type_class, self.data_type_class)
        print(f"parent class:  {parent_base_cls}")
        for shared_name in self.all_shared_types:
            print()
            print(f"shared_name  {shared_name}")
            assert shared_name in self.data_type_class._shared_name_registry
            assert shared_name in parent_base_cls._shared_name_registry
            data_cls = self.data_type_class._shared_name_registry[shared_name]
            print(f"  data class:         {data_cls}")
            parent_data_cls = parent_base_cls._shared_name_registry[shared_name]
            print(f"  data class parent:  {parent_data_cls}")
            assert data_cls is not parent_data_cls
            assert issubclass(data_cls, parent_data_cls)

    def test_type_registries(self):
        # Test *_name*_registry attrs, by_*name() methods
        dtc = self.data_type_class
        for shared_name in self.defined_shared_types:
            t = dtc._shared_name_registry[shared_name]
            print(f"{t}  {t.name}  {t.shared_name}")
            assert dtc._shared_name_registry[t.shared_name] is t
            assert dtc.by_shared_name(t.shared_name) is t
            if t.name is not None:
                assert dtc.by_name(t.name) is t

    def test_conv_str(self):
        for shared_name in self.defined_shared_types:
            cases = self.conv_cases[shared_name]
            data_type = self.data_type_class.by_shared_name(shared_name)
            print(f"{shared_name}:  {data_type}")
            for d_in, d_val, d_str in cases:
                print("   ", *[repr(i) for i in (d_in, d_val, d_str)])
                d = data_type(d_in)
                assert d == d_val
                assert str(d) == d_str
