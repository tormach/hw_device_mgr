import pytest
from .base_test_class import BaseTestClass
from ..device import Device
import subprocess
from pprint import pformat


class TestDevice(BaseTestClass):
    # Expected class MRO
    expected_mro = [
        "SimDevice",
        "Device",
        "LoggingMixin",
        "CachedAttrMixin",
        "ABC",
    ]

    def test_fixture(self, device_cls, sim_device_data, category_cls):
        # Fixture sanity check
        print("device_cls:", device_cls)
        print("sim_device_data:\n", pformat(sim_device_data))
        assert sim_device_data["device_cls"] is device_cls
        assert hasattr(device_cls, "name")
        assert device_cls in self.device_model_classes
        assert issubclass(device_cls, category_cls)

    #
    # Class tests
    #

    def test_mro(self):
        mro = [cls.__name__ for cls in self.device_class.__mro__[1:-1]]
        print("actual MRO:  ", mro)
        print("expected MRO:", self.expected_mro)
        assert mro == self.expected_mro

    def test_category_registry(self, category_cls):
        # Check category_cls(), and if base class is a category,
        # it is in registry
        base = category_cls
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

    def test_scan_devices(self, category_cls, all_device_data):
        devs = category_cls.scan_devices()
        for obj, data in zip(devs, all_device_data.values()):
            print(f"Dev:  {obj}")
            assert obj.name == data["test_name"]
            assert obj.address == data["address"]
            assert obj.model_id == data["model_id"]

    def test_dot(self, category_cls, tmp_path):
        # Test class diagram
        gv_file = tmp_path / f"{category_cls.category}.gv"
        assert not gv_file.exists()
        with gv_file.open("w") as f:
            f.write(category_cls.dot())
        subprocess.check_call(["dot", "-Tpng", "-O", gv_file])
        # All class diagrams
        gv_file = tmp_path / ".." / "all.gv"
        with gv_file.open("w") as f:
            f.write(Device.dot())
        subprocess.check_call(["dot", "-Tpng", "-O", gv_file])

    #
    # Instance tests
    #

    @pytest.fixture
    def obj(self, device_cls, sim_device_data):
        assert device_cls.name
        self.obj = device_cls(address=sim_device_data["address"])
        self.obj.init()
        yield self.obj

    def test_init(self, obj):
        pass  # Base class init() method does nothing

    def test_set_sim_feedback(self, obj):
        res = obj.set_sim_feedback()
        assert res.__class__.__name__ == "DebugInterface"

    def test_check_and_set_timeout(self, obj, mock_time):
        fb_out = obj.feedback_out
        self.now = 10000  # Fake value returned by time.time()

        def fb_get():
            return fb_out.get("goal_reached"), fb_out.get("fault")

        def fb_changed():
            return fb_out.changed("goal_reached"), fb_out.changed("fault")

        def fb_set(goal_reached, fault, latch_old=True):
            fb_out.set(goal_reached=goal_reached, fault=fault)
            assert fb_get() == (goal_reached, fault)  # Sanity
            if latch_old:
                # Run again so that `changed()` is False
                fb_out.set(goal_reached=goal_reached, fault=fault)
                assert fb_changed() == (False, False)  # Sanity

        def do_test(expected):
            # Run once to possibly set timer
            obj.check_and_set_timeout()
            # Let time pass
            self.now += 1000
            # Run again and check against expected result
            if expected is True:
                assert obj.check_and_set_timeout()
            else:
                assert not obj.check_and_set_timeout()

        # Steady state tests
        # - Goal reached and no fault:  clear timer
        fb_set(True, False)
        do_test(False)
        # - Goal not reached and fault:  clear timer
        fb_set(False, True)
        do_test(False)
        # - Goal not reached and no fault:  Set timer
        fb_set(False, False)
        do_test(True)

        # Test transitions
        # - Goal not reached and no fault sets timer, but fault clears it
        fb_set(False, False)
        obj.check_and_set_timeout()
        self.now += 1000
        fb_set(False, True)
        assert not obj.check_and_set_timeout()
        # - Goal not reached and no fault sets timer, but goal reached clears it
        fb_set(False, False)
        obj.check_and_set_timeout()
        self.now += 1000
        fb_set(True, False)
        assert not obj.check_and_set_timeout()
