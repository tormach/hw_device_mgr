import pytest
from .base_test_class import BaseTestClass


class TestDeviceRUW(BaseTestClass):
    @pytest.fixture
    def obj(self, device_cls, sim_device_data):
        assert device_cls.name
        self.obj = device_cls(address=sim_device_data["address"])
        self.obj.init()
        yield self.obj

    #########################################
    # Test read()/update()/write() integration
    #
    # Run object through test cases defined in external .yaml file:
    # - Optionally set goal and/or override feedback
    # - Check expected feedback & command, in & out

    # Configuration
    # - YAML test cases package resource
    read_update_write_package = None  # Skip tests if None
    read_update_write_yaml = "read_update_write.cases.yaml"
    # - Translate feedback/command test input params from values
    #   human-readable in .yaml to values matching actual params
    read_update_write_translate_feedback_in = dict()
    read_update_write_translate_feedback_out = dict()
    read_update_write_translate_command_in = dict()
    read_update_write_translate_command_out = dict()
    read_update_write_translate_sim_feedback = dict()

    def read_update_write_conv_test_data(self):
        # Subclasses may massage data
        pass

    #
    # Setup
    #

    # Print intro and merge test case updates
    def setup_test(self, test_case):
        self.test_desc = test_case["desc"]
        print()
        print("*" * 80)
        print("    ", test_case["desc"])
        print("*" * 80)
        print(f"obj:  {self.obj}")
        # Throw separator into pytest log output
        self.obj.logger.info(f"Step -> {self.test_desc}")

        # Expected feedback/command in/out data:  Update values from
        # last iteration
        #
        self.test_data = getattr(self, "test_data", dict())
        self.munge_test_case_data(test_case, self.test_data)

        # Override feedback/command/sim_feedback data & mgr state
        #
        # Unlike check data, override data isn't carried over from
        # previous iterations
        #
        self.set_override_data(test_case)

        # Nested dictionaries of { interface : { attr : { model_id, ... } } }:
        # model_id must have interface attribute; for other models, missing OK
        self.missing_not_ok = {
            i: dict() for i in self.device_class.interface_names
        }

        self.read_update_write_conv_test_data()
        # self.print_dict(self.test_data, "Test data")
        # self.print_dict(self.ovr_data, "Override data")

    def set_override_data(self, test_case):
        self.ovr_data = dict()  # Clear overrides from previous round
        self.munge_test_case_data(test_case, self.ovr_data, suffix="_set")

    def munge_test_case_data(self, test_case, dst, suffix=""):
        for intf in self.device_class.interface_names:
            values = test_case.get(intf + suffix, dict())
            dst.setdefault(intf, dict()).update(values)

    #
    # Read, get_feedback, set_command, write
    #
    def read_and_check(self):
        print("\n*** Overriding sim_feedback")
        self.override_data("sim_feedback")
        print("\n*** Running object read() and checking feedback")
        self.pre_read_actions()
        self.obj.read()
        self.post_read_actions()
        assert self.check_interface_values("feedback_in")

    def pre_read_actions(self):
        """Provide hook for inserting actions before `read()`."""
        pass

    def post_read_actions(self):
        """Provide hook for inserting actions after `read()`."""
        pass

    def get_feedback_and_check(self):
        print("\n*** Overriding feedback_in")
        self.override_data("feedback_in")
        print("\n*** Overriding command_in")
        self.override_data("command_in")
        # self.print_dict(self.test_data, "Test data (after override)")
        print("\n*** Running object get_feedback()")
        self.obj.get_feedback()
        # self.print_dict(
        #     self.obj.interface("feedback_out").get(), "feedback_out", indent=2
        # )
        assert self.check_interface_values("feedback_out")

    def munge_interface_data(self, interface):
        # Do any test data manipulation before sending to interface; subclasses
        # may override
        return self.test_data[interface]

    def set_command_and_check(self):
        print("\n*** Overriding feedback_out")
        self.override_data("feedback_out")
        print("\n*** Running object set_command()")
        self.obj.set_command(**self.munge_interface_data("command_in"))
        assert self.check_interface_values("command_in")
        assert self.check_interface_values("command_out")
        print("\n*** Overriding command_out")
        self.override_data("command_out")
        # self.print_dict(self.test_data, "Test data (after override)")

    def write_and_check(self):
        print("\n*** Running object write() and checking sim_feedback")
        self.obj.write()
        assert self.check_interface_values("sim_feedback")
        self.post_write_actions()

    def post_write_actions(self):
        """Provide hook for inserting actions after `write()`."""
        pass

    #
    # Utilities
    #

    def override_interface_param(self, interface, ovr_data):
        intf = self.obj.interface(interface)
        intf.update(**ovr_data)

    def override_data(self, interface):
        ovr_data = self.ovr_data.get(interface, dict())
        if not ovr_data:
            print(f"  {interface}:  {{}}  (no overrides)")
            return
        self.override_interface_param(interface, ovr_data)
        self.print_dict(ovr_data, interface, indent=2)
        # self.print_dict(intf_data, interface, indent=2)

    def print_dict(self, d, name, indent=0, prefix=""):
        # Print in format that can be pasted right back into .yaml file
        if isinstance(d, dict):
            if name:
                print(f"{' ' * indent}{name}:")
            for k, v in d.items():
                if isinstance(v, dict):
                    if v:
                        self.print_dict(v, k, indent=indent + 2)
                    else:
                        print(f"{' ' * (indent + 2)}{prefix}{k}:  {{}}")
                else:
                    print(f"{' ' * (indent + 2)}{prefix}{k}:  {v}")
        else:
            if d is None:
                d = "null"
            print(f"{' ' * indent}{prefix}{name}:  {d}")

    def check_interface_values(self, interface, indent=4):
        # Prepare expected data
        expected = self.test_data[interface]
        # self.print_dict(expected, f"Expected {interface}", indent=2)

        # Prepare actual data
        actual = self.obj.interface(interface).get()
        # self.print_dict(actual, "Actual", indent=2)

        # Check actual against expected data
        passing = self.check_data_values(
            interface, expected, actual, indent=indent
        )
        if not passing:
            print(f"FAILURE at {self.test_desc}")
        return passing

    def check_data_values(
        self, interface, expected, actual, indent=4, prefix=""
    ):
        # Destructive operations ahead
        actual, expected = actual.copy(), expected.copy()
        passing = True
        MISSING = dict()  # Sentinel object
        mno = self.missing_not_ok.get(interface, dict())
        model_id = getattr(self.obj, "model_id", None)
        mno_dfl = set([model_id])  # By default, missing params not OK
        if interface:
            print(f"  {interface}:")
        for param in list(expected.keys()):
            expected_val = expected.pop(param)
            actual_val = actual.pop(param, MISSING)
            missing_not_ok = model_id in mno.get(param, mno_dfl)
            # Print debug info
            if actual_val is MISSING:
                msg = "MISSING" if missing_not_ok else "(missing; ok)"
                print(f'{" "*indent}{prefix}{param}:  {msg}')
            elif isinstance(expected_val, dict):
                if actual_val:
                    self.print_dict(actual_val, prefix + param, indent=indent)
                else:
                    print(f'{" "*indent}{prefix}{param}:  {{}}')
            else:
                self.print_dict(actual_val, param, indent=indent, prefix=prefix)
            # Check param actual vs expected
            if actual_val != expected_val and missing_not_ok:
                print(f"  ****MISMATCH****  expected:  {expected_val}")
                passing = False
        # Check expected data is comprehensive, all params checked
        self.check_untested_params(actual)

        return passing

    def check_untested_params(self, params):
        if params:
            print(f"  ****ERROR****  Params not checked:  {params}")
            raise KeyError(f"Params not checked:  {params}")

    def load_test_cases(self):
        rsrc = (self.read_update_write_package, self.read_update_write_yaml)
        rsrc_str = self.resource_path(*rsrc)
        test_cases = self.load_yaml_resource(*rsrc)
        assert test_cases, f"Empty YAML from package resource {rsrc_str}"
        print(f"Read test cases from package resource {rsrc_str}")
        return test_cases

    #
    # Main function
    #

    def read_update_write_loop(self, test_case):
        self.setup_test(test_case)
        self.read_and_check()
        self.get_feedback_and_check()
        self.set_command_and_check()
        self.write_and_check()

    def test_read_update_write(self, obj):
        if self.read_update_write_package is None:
            return  # No test cases defined for this class
        test_cases = self.load_test_cases()
        # Start by overriding feedback_out and command_out
        print("\n*** Overriding feedback_out and command_out")
        self.set_override_data(test_cases[0])
        self.override_data("feedback_out")
        self.override_data("command_out")
        # Now loop over cases
        for test_case in test_cases:
            self.read_update_write_loop(test_case)
