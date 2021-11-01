from .base_test_class import BaseCiA301TestClass


class TestCiA301SDO(BaseCiA301TestClass):
    def test_init(self, sdo_data):
        for sd in sdo_data.values():
            print("SDO data:", sd)
            obj = self.sdo_class(**sd)

            print("sdo:", repr(obj))
            assert (sd["index"], sd["subindex"]) == (obj.index, obj.subindex)
            print(obj.data_type)
            assert issubclass(obj.data_type, self.data_type_class)
            assert obj.name == sd["name"]
            assert obj.ro == sd.get("ro", False)

    def test_parse_idx_str(self):
        test_data = (
            ("2004-04h", (0x2004, 0x04)),
            ("6040h", (0x6040, 0x00)),
            ("ffff-ffh", (0xFFFF, 0xFF)),
            ("FFFF-FFh", (0xFFFF, 0xFF)),
            ("FFff-fFH", (0xFFFF, 0xFF)),
        )
        for arg, expected in test_data:
            print("arg:", arg, "expected:", expected)
            assert self.sdo_class.parse_idx_str(arg) == expected
