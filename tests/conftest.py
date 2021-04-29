import pytest
from unittest.mock import MagicMock, PropertyMock, patch
from weakref import WeakSet
import yaml
import os


###############################
# A boilerplate abstract class for stateful fixtures that mock complex
# behaviors


class MockFixture:
    fixture_name = 'MockFixture'
    instance_attr = 'name'
    mock_methods = []
    mock_attrs = []
    patches = None
    subordinate_classes = []

    _cache = {}

    @classmethod
    def instances(cls):
        if not hasattr(cls, "_instances"):
            cls._instances = WeakSet()
        return cls._instances

    @classmethod
    def clear(cls):
        for c in cls.subordinate_classes:
            c.clear()
        cls.instances().clear()

    def __new__(cls, *args, **kwargs):
        obj = object.__new__(cls)
        cls.instances().add(obj)
        return obj

    @property
    def instance_name(self):
        return getattr(self, self.instance_attr, "(anonymous)")

    def __repr__(self):
        return f"<{self.fixture_name} '{self.instance_name}'>"

    def _make_mock_obj(self):
        m = MagicMock(name=self.__repr__())
        m.obj = self
        for method in self.mock_methods:
            getattr(m, method).side_effect = getattr(self, method)
        for attr in self.mock_attrs:
            setattr(
                type(m),
                attr,
                # Use m as sentinel object to avoid 1 extra SLOC!
                PropertyMock(
                    side_effect=lambda x=m: getattr(self, attr)
                    if x is m
                    else setattr(self, attr, x)
                ),
            )
        return m

    @property
    def mock_obj(self):
        if not hasattr(self, '_mock_obj'):
            self._mock_obj = self._make_mock_obj()
        return self._mock_obj

    @classmethod
    def get_mock(cls, *args):
        obj = cls(*args)
        return obj.mock_obj

    @classmethod
    def get_instance(cls, singleton=True):
        # Return an arbitrary instance; if singleton is True, must be
        # *only* instance
        if singleton:
            assert len(cls._instances) == 1
        for inst in cls._instances:  # Get arbitrary (only?) element
            break
        return inst

    @classmethod
    def fixture(cls, name, request, *args):
        mock_obj = cls.get_mock(*args)
        mock_obj.obj.request_inst = request.instance
        setattr(request.instance, name, mock_obj)
        if cls.patches is not None:
            patches = cls.patches
        else:
            patches = getattr(request.instance, f'patch_{name}', None)
        if patches is None:
            raise RuntimeError(f"Unable to determine which {name} to patch")
        if isinstance(patches, str):
            patches = (patches,)
        for p in patches:
            print(f'Patching {name} target {p}')
            patch(p, new=mock_obj).start()
        cls.tweak_fixture(mock_obj, mock_obj.obj)
        yield mock_obj
        patch.stopall()

    @classmethod
    def tweak_fixture(cls, mock_obj, obj):
        # Hook for subclasses to further customize fixture
        pass


###############################
# Misc. fixtures


@pytest.fixture
def fpath(request):
    """Fixture that returns test directory"""

    def func(base_name=None):
        cwd = os.path.dirname(os.path.realpath(__file__))
        if base_name is None:
            return cwd
        else:
            return os.path.join(cwd, base_name)

    return func


###############################
# Mock HAL


class MockHALPin(MockFixture):
    fixture_name = 'MockHALPin'
    instance_attr = 'pname'
    mock_methods = ['get', 'set']

    def __init__(self, pname, ptype, pdir):
        self.pname = pname
        self.ptype = ptype
        self.pdir = pdir
        self.val = 0

    def set(self, val):
        print(f"MockHALPin {self.pname}: set({val}/0x{val:X})")
        self.val = val

    def get(self):
        print(f"MockHALPin {self.pname}: get() -> {self.val}/0x{self.val:X}")
        return self.val


class MockHALComponent(MockFixture):
    fixture_name = 'MockHALComponent'
    instance_attr = 'name'
    mock_methods = [
        'ready',
        'getprefix',
        'newpin',
        'pin_names',
        'get_pin',
        'set_pin_val',
        'get_pin_val',
        'get_pin_type',
        'get_pin_dir',
    ]
    mock_attrs = ['is_ready']
    patches = 'hal.component'

    def __init__(self, name):
        self.name = name
        self.is_ready = False
        self.pins = {}
        print(f"Created {self}")

    def ready(self):
        print(f"MockHALComponent {self.name}: ready()")
        self.is_ready = True

    def getprefix(self):
        print(f"MockHALComponent {self.name}: getprefix() -> {self.name}")
        return self.name

    def newpin(self, *args):
        mock_obj = MockHALPin.get_mock(*args)
        pin = mock_obj.obj
        self.pins[pin.pname] = mock_obj
        print(f"MockHALComponent {self.name}: newpin({args})")
        return mock_obj

    def pin_names(self):
        return self.pins.keys()

    def get_pin(self, pname):
        return self.pins.get(pname, None)

    def set_pin_val(self, pname, val):
        self.pins[pname].obj.val = val

    def get_pin_val(self, pname):
        return self.pins[pname].obj.val

    def get_pin_type(self, pname):
        return self.pins[pname].obj.ptype

    def get_pin_dir(self, pname):
        return self.pins[pname].obj.pdir


@pytest.fixture
def mock_halcomp(request):
    """Fixture for mocking a HAL component

    The mock object is passed as test function arg and also test
    object `mock_halcomp` attribute.  It mocks the following methods:

    - `getprefix()`:  Returns component name passed to constructor
    - `newpin()`:  Returns mock HAL pin object; see below
    - `ready()`:  Sets `is_ready` attribute `True`

    Mock HAL pins `set()` and `get()` methods operate on a value
    accessible out-of-band with `mock_halcomp.set_pin_val(pin_name,
    val)` and `mock_halcomp.get_pin_val(pin_name)`.  The pin type and
    direction are accessible via `mock_halcomp.get_pin_type(pin_name)`
    and `mock_halcomp.get_pin_dir(pin_name)`, respectively.
    """
    comp_name = getattr(request.instance, 'halcomp_name', 'foocomp')
    yield from MockHALComponent.fixture('mock_halcomp', request, comp_name)


@pytest.fixture()
def mock_hal(request):
    """Fixture for mocking `hal.component`

    This fixture patches `hal.component` and returns the mock class in
    test function `mock_hal` argument and test object `mock_hal`
    attribute.  See the `mock_halcomp` fixture for more information
    about the mock instances of this mock class.
    """
    mock_hal = MagicMock()
    mock_hal.component = MagicMock(
        name="mock_hal_component", side_effect=MockHALComponent.get_mock
    )
    request.instance.mock_hal = mock_hal
    request.instance.components = MockHALComponent
    patch('hal.component', side_effect=mock_hal.component).start()
    yield mock_hal
    MockHALComponent.clear()
    patch.stopall()


###############################
# Mock rospy


@pytest.fixture
def ros_params(request, fpath):
    """Fixture for mocking ROS params; used with `mock_rospy` fixture."""

    if hasattr(request.instance, 'ros_params_yaml'):
        # Load .yaml file from current directory
        yaml_file_base = getattr(request.instance, 'ros_params_yaml', None)
        yaml_file = fpath(yaml_file_base)
        with open(yaml_file) as f:
            return yaml.safe_load(f)
    else:
        # Look for params in test object attrs
        return getattr(request.instance, 'ros_params', {})


class MockRospy(MockFixture):
    fixture_name = 'MockRospy'
    mock_methods = [
        'reset',
        'init_node',
        'is_shutdown',
        'has_param',
        'get_param',
        'logdebug',
        'loginfo',
        'logwarn',
        'logerr',
        'Publisher',
        'Service',
        # Out-of-band accessors
        'set_shutdown',
    ]
    mock_attrs = [
        'is_shutdown_max_cycles',
        'is_shutdown_behavior',
    ]

    def __init__(self, params):
        self.params = params
        self.is_shutdown_max_cycles = 3
        self.is_shutdown_behavior = 'shutdown'
        self.reset()

    def reset(self):
        self.shutdown = False
        self.cycles = 0
        if hasattr(self, 'mock_obj'):
            self.mock_obj.reset_mock()

    def init_node(self, name):
        print(f'rospy.init_node({name})')
        self.name = name

    def is_shutdown(self):
        self.cycles += 1
        if self.shutdown:
            res = True
        elif self.cycles >= self.is_shutdown_max_cycles:
            if self.is_shutdown_behavior == 'shutdown':
                res = True
            elif self.is_shutdown_behavior == 'raise':
                exc = self.mock_obj.exceptions.ROSInterruptException
                print(f'rospy.is_shutdown():  Raise {exc}')
                raise exc('ROS Interrupt')
        else:
            res = False
        print(f'rospy.is_shutdown() = {res}')
        return res

    def set_shutdown(self):
        self.shutdown = True

    def _lookup(self, name, default=None, params=None):
        if params is None:
            params = self.params
        split = [yaml.safe_load(s) for s in name.split('/', 1)]
        if split is None:
            return default
        if split[0] is None:  # Leading /
            return self._lookup(split[1], default=default)
        elif len(split) == 2:
            if split[0] in params:
                return self._lookup(
                    split[1], default=default, params=params[split[0]]
                )
            else:
                return default
        else:
            if split[0] in params:
                return params.get(split[0], default)
            else:
                return default

    def has_param(self, name):
        res = self._lookup(name) is not None
        print(f'rospy.has_param({name}) -> {res}')
        return res

    def get_param(self, name, default=None):
        res = self._lookup(name, default)
        print(f'rospy.get_param({name}, {default}) -> {str(res)[:30]}')
        return res

    def logdebug(self, msg):
        print(f"rospy.logdebug:  '{msg}'")

    def loginfo(self, msg):
        print(f"rospy.loginfo:  '{msg}'")

    def logwarn(self, msg):
        print(f"rospy.logwarn:  '{msg}'")

    def logerr(self, msg):
        print(f"rospy.logerr:  '{msg}'")

    def Publisher(self, name, msg_type, queue_size=0, latch=False):
        print(f'rospy.Publisher({name}, {msg_type}, {queue_size}, {latch})')
        return MagicMock(name=f'rospy.Publisher({name})')

    def Service(self, name, msg_type, callback):
        print(f'rospy.Service({name}, {msg_type}, {callback})')
        return MagicMock(name=f'rospy.Publisher({name})')

    @classmethod
    def tweak_fixture(cls, mock_obj, obj):
        # `rospy.ROSInterruptException()` sets test object
        # `raised_ros_interrupt_exception` attribute to True.
        from rospy.exceptions import ROSInterruptException

        class NewROSInterruptException(ROSInterruptException):
            def __init__(self, *args, **kwargs):
                super().__init__(*args, **kwargs)
                obj.request_inst.raised_ros_interrupt_exception = True

        mock_obj.exceptions.ROSInterruptException = NewROSInterruptException


@pytest.fixture
def mock_rospy(ros_params, request):
    """Fixture for mocking `rospy`

    Because this mocks the entire top-level `rospy` module, the test
    object (or class) must have a `patch_rospy` attribute, a `str` or
    `tuple` of `str`, with the object to patch, e.g. if your module
    `mymodule` has `import rospy`, then the test object must have
    `patch_rospy` attribute `mymodule.rospy`.

    The mock object is passed as test function arg and also test
    object `mock_halcomp` attribute.  It mocks the following methods:

    - `init_node()`:  Stores name for use in other methods
    - `is_shutdown()`:  Returns `True` if accessor `set_shutdown()`
      was called
    - `has_param()`, `get_param()`:  See below for mocking ROS params
    - `log*()`:  Prints log to stdout

    The above methods are regular `MagicMock` objects and can be
    tested with `assert_called()`, etc. methods.

    ROS params may be mocked in two ways:
    - The test object's 'ros_params_yaml' attribute may name a `.yaml`
      file in the current directory from which to read parameters
    - The test object's `ros_params` attribute may contain a `dict`
    object containing parameters
    """

    yield from MockRospy.fixture('mock_rospy', request, ros_params)
