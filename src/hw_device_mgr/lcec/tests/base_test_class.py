import pytest
from unittest.mock import MagicMock, patch
from ...ethercat.tests.base_test_class import BaseEtherCATTestClass
from ..data_types import LCECDataType
from ..sdo import LCECSDO
from .bogus_devices.command import BogusLCECCommand
from .bogus_devices.config import BogusLCECConfig
from .bogus_devices.device import (
    BogusLCECDevice,
    BogusLCECServo,
    BogusLCECServo2,
    BogusLCECIO,
)


class BaseLCECTestClass(BaseEtherCATTestClass):

    # Classes under test in this module
    data_type_class = LCECDataType
    sdo_class = LCECSDO
    command_class = BogusLCECCommand
    config_class = BogusLCECConfig
    device_class = BogusLCECDevice
    device_model_classes = BogusLCECServo, BogusLCECServo2, BogusLCECIO

    @classmethod
    def lcec_data_type(cls, type_str):
        if not hasattr(cls, "_lcec_data_type_map"):
            dtm = cls._lcec_data_type_map = dict()
            for dt in cls.data_type_class.all_types():
                if hasattr(dt, "igh_type"):
                    dtm[dt.igh_type] = dt
        return cls._lcec_data_type_map[type_str]

    @pytest.fixture
    def mock_ethercat_command(self, all_device_data):
        """Emulate IgH EtherCAT master `ethercat` command-line utility

        Realistic results from `slaves`, `upload` and `download`
        commands.  Patches `subprocess.check_output()`.
        """

        def emulate_ethercat_command(args):
            print(f'mocking ethercat command: {" ".join(args)}')
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
            # Emulate commands
            if cmd in ("upload", "download"):
                ix = tuple(int(x, 16) for x in (args.pop(0), args.pop(0)))
                dtc = self.data_type_class
                ix = tuple([dtc.uint16(ix[0]), dtc.uint8(ix[1])])
                dt = self.lcec_data_type(kwargs["type"])
                sdo = self.sdo_class(index=ix[0], subindex=ix[1], data_type=dt)
                address = (int(kwargs["master"]), int(kwargs["position"]))
                params = self.dev_data[address]["params"]
                if cmd == "download":
                    val = sdo.to_data_type_value(args.pop(0))
                    print(f"  download {sdo} = {val}")
                    params[ix] = val
                    return b""
                else:  # upload
                    val = params[ix]
                    res = (
                        f"0x{val:04x} {val:d}"
                        if val.base_type is int
                        else str(val)
                    )
                    print(f'  upload {sdo} = {val} "{res}"')
                    return res.encode()
            elif cmd == "slaves":
                res = ""
                for data in all_device_data:
                    _res = "=== Master {bus}, Slave {position} ===\n"
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
    def command_cls(self, all_device_data, all_sdo_data, mock_ethercat_command):
        """Side-load bus scan data into command class

        Emulate IgH `ethercat` utility"""
        self.command_class.init(all_device_data, all_sdo_data)
        yield self.command_class
