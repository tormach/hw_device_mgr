import pytest
from unittest.mock import MagicMock, patch
from ...ethercat.tests.base_test_class import BaseEtherCATTestClass
from ...hal.tests.base_test_class import BaseHALTestClass
from ..data_types import LCECDataType
from ..sdo import LCECSDO
from ..command import LCECSimCommand
from ..config import LCECSimConfig
from .bogus_devices.device import (
    BogusLCECDevice,
    BogusLCECV1Servo,
    BogusLCECV2Servo,
    BogusLCECV1IO,
)


class BaseLCECTestClass(BaseEtherCATTestClass, BaseHALTestClass):
    # Classes under test in this module
    data_type_class = LCECDataType
    sdo_class = LCECSDO
    command_class = LCECSimCommand
    config_class = LCECSimConfig
    device_class = BogusLCECDevice
    device_model_classes = BogusLCECV1Servo, BogusLCECV2Servo, BogusLCECV1IO

    @classmethod
    def lcec_data_type(cls, type_str):
        if not hasattr(cls, "_lcec_data_type_map"):
            assert issubclass(
                cls.data_type_class, LCECDataType
            ), f"{cls.data_type_class} is not subclass of LCECDataType"
            dtm = cls._lcec_data_type_map = dict()
            for dt in cls.data_type_class.all_types():
                if hasattr(dt, "igh_type"):
                    dtm[dt.igh_type] = dt
        return cls._lcec_data_type_map[type_str]

    @pytest.fixture
    def mock_ethercat_command(self):
        """
        Emulate IgH EtherCAT master `ethercat` command-line utility.

        Realistic results from `slaves`, `upload` and `download`
        commands.  Patches `subprocess.check_output()`.
        """

        def emulate_ethercat_command(args, **kwargs):
            # kwargs might be set by `subprocess.check_output`; check known args
            kwargs.pop("stderr", None)
            assert not kwargs, f"Non-empty kwargs:  {kwargs}"
            print(f'mocking command: {" ".join(args)}')
            print(f"  subprocess.check_output kwargs: {repr(kwargs)}")
            # Parse out args, kwargs
            assert args.pop(0) == "ethercat"
            cmd = args.pop(0)
            kwargs = dict()
            for ix, arg in reversed(list(enumerate(args))):
                if not arg.startswith("--"):
                    continue
                if "=" in arg:
                    k, v = arg[2:].split("=")
                else:
                    k, v = (arg[2:], True)
                kwargs[k] = v
                args.pop(ix)
            if "alias" not in kwargs:
                kwargs["alias"] = "0"
            # Emulate commands
            if cmd in ("upload", "download"):
                ix = tuple(int(x, 16) for x in (args.pop(0), args.pop(0)))
                dtc = self.data_type_class
                ix = tuple([dtc.uint16(ix[0]), dtc.uint8(ix[1])])
                dt = self.lcec_data_type(kwargs["type"])
                sdo = self.sdo_class(
                    index=ix[0], subindex=ix[1], data_type=dt.shared_name
                )
                address = tuple(
                    int(kwargs[k]) for k in ("master", "position", "alias")
                )
                sim_sdo_values = self.command_class.sim_sdo_values
                key = self.config_class.address_in_canon_addresses(
                    address, sim_sdo_values
                )
                assert key is not None
                params = sim_sdo_values[key]
                if cmd == "download":
                    val = args.pop(0)
                    assert val is not None
                    val = sdo.data_type(val)
                    print(f"  download {sdo} = {val}")
                    params[ix] = val
                    return b""
                else:  # upload
                    val = params[ix] or 0
                    res = (
                        f"0x{val:04x} {val:d}"
                        if isinstance(val, int)
                        else str(val)
                    )
                    print(f'  upload {sdo} = {val} "{res}"')
                    return res.encode()
            elif cmd == "slaves":
                res = ""
                for data in self.command_class.sim_device_data.values():
                    _res = "=== Master {address[0]}, Slave {address[1]} ===\n"
                    if data["address"][2]:
                        _res += "Alias: {address[2]}\n"
                    _res += "Identity:\n"
                    _res += "  Vendor Id:       0x{vendor_id:08x}\n"
                    _res += "  Product code:    0x{product_code:08x}\n"
                    res += _res.format(**data)
                print(f"  slaves:\n{res}")
                return res.encode()
            else:
                print(f"{cmd}:  (unhandled)")
                return b""

        # Patch in a mock object to run check_output()
        mock_cmd = MagicMock(
            name="mock_ethercat_command", side_effect=emulate_ethercat_command
        )
        patch("subprocess.check_output", new=mock_cmd).start()
        yield mock_cmd
        patch.stopall()

    @pytest.fixture
    def ethercat_extra_fixtures(self, device_xml, mock_ethercat_command):
        yield

    @pytest.fixture
    def category_extra_fixtures(
        self, ethercat_extra_fixtures, hal_extra_fixtures, log_debug
    ):
        pass
