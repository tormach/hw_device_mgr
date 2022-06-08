import pytest
from .base_test_class import BaseTestClass
from ..device import Device
import subprocess
from pprint import pformat
import ruamel.yaml


class TestDevice(BaseTestClass):
    # Expected class MRO
    expected_mro = [
        "SimDevice",
        "Device",
        "ABC",
    ]

    #
    # Class tests
    #

    def test_mro(self, device_cls):
        mro = [cls.__name__ for cls in device_cls.__mro__[1:-1]]
        print("actual MRO:  ", mro)
        print("expected MRO:", self.expected_mro)
        assert mro == self.expected_mro

    def test_category_registry(self, device_cls):
        # Check category_cls(), and if base class is a category,
        # it is in registry
        base = device_cls
        # Test category registries
        print(f"base cls {base}")
        print(f"category {base.category}")
        print(f"category registry:  {base._category_registry}")
        assert base.category in base._category_registry
        if base.category == "hw_device_mgr":
            # hw_device_mgr is its own device...
            assert "category" not in base.__dict__
            assert base.category_cls(base.category) is not base
            assert issubclass(base, base.category_cls(base.category))
        else:
            # ...whereas other devices are tested together in a category
            assert "category" in base.__dict__
            assert base.category == base.__dict__["category"]
            assert base.category_cls() is base
            assert base.category_cls(base.category) is base
        category_cls = base.category_cls()
        assert category_cls.category == base.category
        assert issubclass(base, category_cls)
        category_cls_super = super(category_cls, category_cls)
        assert getattr(category_cls_super, "category ", None) != base.category

    def test_model_registries(self, category_cls):
        print(f"Registry log:\n{pformat(self.device_class._registry_log)}")

        for model_cls in self.device_model_classes:
            model_id = model_cls.device_model_id()
            id_registry = category_cls._model_id_registry
            name = model_cls.name
            name_registry = category_cls._model_name_registry
            print(f"model_cls:  {model_cls}")
            print(f"model_cls model_id:  {model_id}")
            print(f"model_cls name:  {name}")
            print(f"dev cls category: {category_cls.category}")
            print(f"dev cls id registry:\n{pformat(id_registry)}")
            print(f"dev cls name registry:\n{pformat(name_registry)}")
            assert category_cls.category in id_registry
            assert model_id in id_registry[category_cls.category]
            assert category_cls.category in name_registry
            assert name in name_registry[category_cls.category]
            id_registered = False
            name_registered = False
            for parent_cls in category_cls.__mro__:
                if not hasattr(parent_cls, "category"):
                    break  # Parent class of `Device`
                print(f"category: {parent_cls.category}")
                # Check model_id_registry
                assert parent_cls.category in id_registry
                id_cat_reg = id_registry[parent_cls.category]
                print(f"model_id_registry:\n{pformat(id_cat_reg)}")
                assert (
                    id_registered or parent_cls.get_model(model_id) is model_cls
                )
                if "category" in parent_cls.__dict__:
                    assert model_id in id_cat_reg
                    assert id_registered or id_cat_reg[model_id] is model_cls
                    id_registered = True
                # Check model name registry
                assert parent_cls.category in name_registry
                name_cat_reg = name_registry[parent_cls.category]
                print(f"category: {parent_cls.category}")
                print(f"model_name_registry:\n{pformat(name_cat_reg)}")
                assert (
                    name_registered
                    or parent_cls.get_model_by_name(name) is model_cls
                )
                if "category" in parent_cls.__dict__:
                    assert name in name_cat_reg
                    assert name_registered or name_cat_reg[name] is model_cls
                    name_registered = True

            assert id_registered
            assert name_registered

    def test_scan_devices(self, device_cls, all_device_data):
        devs = device_cls.scan_devices()
        for obj, data in zip(devs, all_device_data.values()):
            print(f"Dev:  {obj}")
            assert obj.name == data["test_name"]
            assert obj.address == data["test_address"]
            assert obj.model_id == data["model_id"]

    #
    # Instance tests
    #

    @pytest.fixture
    def obj(self, device_cls, sim_device_data):
        self.sim_device_data = sim_device_data
        self.obj = device_cls(address=sim_device_data["test_address"])
        self.obj.init()
        yield self.obj

    def test_init(self, obj):
        assert hasattr(obj, "index")

    def test_set_sim_feedback(self, obj):
        res = obj.set_sim_feedback()
        assert res.__class__.__name__ == "Interface"

    #########################################
    # Test read()/update()/write() integration
    #
    # Run object through test cases defined in external .yaml file:
    # - Optionally set goal and/or override feedback
    # - Check expected feedback & command, in & out

    # Configuration
    # - Path to .yaml test cases (relative to `tests/` directory)
    read_update_write_yaml = None
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

        # Override feedback/command/sim_feedback data
        #
        # Unlike check data, override data isn't carried over from
        # previous iterations
        #
        self.ovr_data = dict()
        self.munge_test_case_data(test_case, self.ovr_data, suffix="_set")

        self.read_update_write_conv_test_data()
        # self.print_dict(self.test_data, "Test data")
        # self.print_dict(self.ovr_data, "Override data")

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
        # self.print_dict(self.test_data, "Test data (after override)")
        print("\n*** Running object get_feedback()")
        self.obj.get_feedback()
        # self.print_dict(
        #     self.obj.interface("feedback_out").get(), "feedback_out", indent=2
        # )
        assert self.check_interface_values("feedback_out")

    def set_command_and_check(self):
        print("\n*** Running object set_command()")
        self.obj.set_command(**self.test_data["command_in"])
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

    def override_interface_param(self, interface, key, val):
        intf = self.obj.interface(interface)
        intf.update(**{key: val})

    def override_data(self, interface):
        ovr_data = self.ovr_data.get(interface, dict())
        if not ovr_data:
            print(f"  {interface}:  No overrides")
            return
        for key, val in ovr_data.items():
            self.override_interface_param(interface, key, val)
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
        if interface:
            print(f"  {interface}:")
        for param in list(expected.keys()):
            expected_val = expected.pop(param)
            actual_val = actual.pop(param, MISSING)
            # Print debug info
            if actual_val is MISSING:
                print(f'{" "*indent}{prefix}{param}:  MISSING')
            elif isinstance(expected_val, dict):
                if actual_val:
                    self.print_dict(actual_val, prefix + param, indent=indent)
                else:
                    print(f'{" "*indent}{prefix}{param}:  {{}}')
            else:
                self.print_dict(actual_val, param, indent=indent, prefix=prefix)
            # Check param actual vs expected
            if actual_val != expected_val:
                print(f"  ****MISMATCH****  expected:  {expected_val}")
                passing = False
        # Check expected data is comprehensive, all params checked
        self.check_untested_params(actual)

        return passing

    def check_untested_params(self, params):
        if params:
            print(f"  ****ERROR****  Params not checked:  {params}")
            raise KeyError(f"Params not checked:  {params}")

    #
    # Main function
    #

    def read_update_write_loop(self, test_case):
        self.setup_test(test_case)
        self.read_and_check()
        self.get_feedback_and_check()
        self.set_command_and_check()
        self.write_and_check()

    def test_read_update_write(self, obj, fpath):
        test_cases_yaml = getattr(self, "read_update_write_yaml", None)
        if test_cases_yaml is None:
            return  # No test cases defined for this class
        with open(fpath(test_cases_yaml)) as f:
            yaml = ruamel.yaml.YAML()
            test_cases = yaml.load(f)
        print(f"Read test cases from {fpath(test_cases_yaml)}")

        for test_case in test_cases:
            self.read_update_write_loop(test_case)

    def test_dot(self, tmp_path):
        # Test class diagram
        gv_file = tmp_path / f"{self.device_class.category}.gv"
        assert not gv_file.exists()
        with gv_file.open("w") as f:
            f.write(self.device_class.dot())
        subprocess.check_call(["dot", "-Tpng", "-O", gv_file])
        # All class diagrams
        gv_file = tmp_path / ".." / "all.gv"
        with gv_file.open("w") as f:
            f.write(Device.dot())
        subprocess.check_call(["dot", "-Tpng", "-O", gv_file])
