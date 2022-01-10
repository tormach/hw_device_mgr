from .base_test_class import BaseCiA301TestClass


class TestCiA301SDO(BaseCiA301TestClass):
    def test_init(self, sdo_data):
        for sdo in sdo_data.values():
            print("SDO data:", sdo)
            obj = self.sdo_class(
                index=sdo.index,
                subindex=sdo.subindex,
                data_type=sdo.data_type.shared_name,
                name=sdo.name,
                index_name=sdo.index_name,
                ro=sdo.ro,
                pdo_mapping=sdo.pdo_mapping,
                default_value=sdo.default_value,
                min_value=sdo.min_value,
                max_value=sdo.max_value,
            )
            print("sdo:", repr(obj))
            assert (sdo.index, sdo.subindex) == (obj.index, obj.subindex)
            print(obj.data_type)
            assert issubclass(obj.data_type, self.data_type_class)
            assert obj.name == sdo.name
            assert obj.ro == sdo.ro

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
