from .base_test_class import BaseMgrTestClass
from ...tests.test_device import TestDevice as _TestDevice
import re
import pytest


class TestHWDeviceMgr(BaseMgrTestClass, _TestDevice):
    expected_mro = [
        "BogusHWDeviceMgr",
        "HWDeviceMgr",
        "FysomGlobalMixin",
        "Device",
        "ABC",
        "object",
    ]

    tc_is_category = False

    # Test CiA NMT init:  online & operational status
    read_update_write_yaml = "mgr/tests/read_update_write.cases.yaml"

    @pytest.fixture
    def obj(self, device_cls, mgr_config, global_config):
        self.obj = device_cls(sim=self.sim)
        self.obj.init(mgr_config=mgr_config)
        self.obj.init_devices(device_config=global_config)
        yield self.obj

    drive_key_re = re.compile(r"^drive_([x0-9])_(.*)$")

    def read_update_write_conv_test_data(self):
        uint16 = self.device_class.data_type_class.uint16
        dbc = self.device_class.device_base_class
        for data in (self.test_data, self.ovr_data):
            for intf, intf_data in data.items():
                for key in intf_data.keys():
                    # # Test mgr keys
                    # if key == "drive_control_mode":
                    #     intf_data[key] = dbc.control_mode_int(intf_data[key])
                    # Test drive keys
                    match = self.drive_key_re.match(key)
                    if not match:
                        continue  # Only testing drive keys
                    dkey = match.group(2)
                    if dkey == "control_mode" and intf == "command_out":
                        # Translate control_mode, e.g. MODE_CSP -> 8
                        intf_data[key] = dbc.control_mode_int(intf_data[key])
                    elif dkey == "control_mode_fb" and intf in (
                        "sim_feedback",
                        "feedback_in",
                    ):
                        # Translate control_mode, e.g. MODE_CSP -> 8
                        intf_data[key] = dbc.control_mode_int(intf_data[key])
                    elif dkey in ("status_word", "control_word"):
                        # Format status_word, control_word for
                        # readability, e.g. 0x000F
                        intf_data[key] = uint16(intf_data[key])

    def munge_test_case_data(self, test_case, dst, suffix=""):
        # Expand drive_x_foo -> drive_1_foo, etc.; don't clobber
        # specific case
        for intf in self.device_class.interface_names:
            values = test_case.get(intf + suffix, dict())
            intf_dst = dst.setdefault(intf, dict())
            for key, val in values.items():
                match = self.drive_key_re.match(key)
                if not match:
                    intf_dst[key] = val
                    continue
                d_ix, d_key = match.groups()
                if d_ix == "x":
                    for i, dev in enumerate(self.obj.devices):
                        dev_key = f"drive_{i}_{d_key}"
                        if dev_key not in values:
                            intf_dst[dev_key] = val
                else:
                    intf_dst[key] = val

    def split_drive_data(self, data):
        # The manager is a device, and each of obj.devices is a
        # device.  The test cases are crammed together with device
        # keys prefixed with `drive_x_`, meaning it applies to all
        # drives, or prefixed with `drive_0_`, meaning it applies to
        # the first drive, or prefixed with nothing, meaning it
        # applies to the manager.
        mgr_data = data.copy()
        device_data = list([dict() for d in self.obj.devices])
        for drive_key in list(mgr_data.keys()):
            match = self.drive_key_re.match(drive_key)
            if not match:
                continue  # Not a drive_key
            val = mgr_data.pop(drive_key)
            index, key = match.groups()
            device_data[int(index)][key] = val
        return mgr_data, device_data

    def override_interface_param(self, interface, key, val):
        match = self.drive_key_re.match(key)
        if match:
            index, key = match.groups()
            intf = self.obj.devices[int(index)].interface(interface)
            intf.update(**{key: val})
        else:
            super().override_interface_param(interface, key, val)

    def check_interface_values(self, interface, indent=4):
        # Prepare expected data
        expected = self.test_data[interface]
        mgr_expected, device_expected = self.split_drive_data(expected)
        # self.print_dict(mgr_expected, f"Expected {interface}", indent=2)
        # for i, d in enumerate(device_expected):
        #     self.print_dict(d, f"drive_{i}", indent=4)

        # Check actual against expected data
        passing = True
        # - Mgr
        actual = self.obj.interface(interface).get()
        # self.print_dict(actual, f"Actual {interface}", indent=2)
        passing &= self.check_data_values(
            interface, mgr_expected, actual, indent=indent
        )

        # - Devices
        for i, device in enumerate(self.obj.devices):
            actual = device.interface(interface).get()
            # self.print_dict(actual, f"drive_{i}", indent=4)
            passing &= self.check_data_values(
                "", device_expected[i], actual, indent, f"drive_{i}_"
            )

        if not passing:
            print(f"FAILURE at {self.test_desc}")
        return passing

    def set_command_and_check(self):
        print("\n*** Running object set_command()")
        # Mgr command_in is internally generated; no args to set_command()
        # (Drive command is set by mgr; no need to call that separately)
        self.obj.set_command()
        success = self.check_interface_values("command_in")
        success = self.check_interface_values("command_out") and success
        assert success
        print("\n*** Overriding command_out")
        self.override_data("command_out")
        # self.print_dict(self.test_data, "Test data (after override)")
