import abc
from importlib.resources import path as imp_path
from .logging import Logging
from .interface import Interface
from .data_types import DataType


class Device(abc.ABC):
    """Base device class for both device categories and device models"""

    category = "all"  # Device category sets up registry for models
    name = None  # Concrete device model subclasses must define

    logger = Logging.getLogger(__name__)

    data_type_class = DataType

    feedback_in_data_types = dict()
    feedback_out_data_types = dict()
    command_in_data_types = dict()
    command_out_data_types = dict()
    sim_feedback_data_types = dict()

    feedback_in_defaults = dict()
    feedback_out_defaults = dict(goal_reached=True, goal_reason="Reached")
    command_in_defaults = dict()
    command_out_defaults = dict()
    sim_feedback_defaults = dict()

    interface_names = {
        "feedback_in",
        "feedback_out",
        "command_in",
        "command_out",
        "sim_feedback",
    }

    def __init__(self, address=None, sim=False):
        self.address = address
        self.sim = sim
        self.init_interfaces()

    def init(self, index=None):
        """Subclasses may implement `init()` for extra initialization outside
        the constructor.  Implementations should always call
        `super().init()`.
        """
        self.index = index

    @classmethod
    def merge_dict_attrs(cls, attr):
        """Scan through class and parent classes for `attr`, a `dict`, and
        return merged `dict`"""
        res = dict()
        for c in cls.__mro__:
            c_attr = c.__dict__.get(attr, dict())
            # Overlap not allowed
            assert not (set(res.keys()) & set(c_attr.keys()))
            res.update(c_attr)
        return res

    def init_interfaces(self):
        intfs = self._interfaces = dict()
        dt_name2cls = self.data_type_class.by_shared_name
        for name in self.interface_names:
            defaults = self.merge_dict_attrs(f"{name}_defaults")
            for k, v in defaults.items():
                if isinstance(v, dict):
                    defaults[k] = v.copy()
            dt_names = self.merge_dict_attrs(f"{name}_data_types")
            data_types = {k: dt_name2cls(v) for k, v in dt_names.items()}
            intfs[name] = Interface(name, defaults, data_types)

    def interface(self, name):
        return self._interfaces[name]

    def __getattr__(self, name):
        """Provide attributes for each interface"""
        if name not in self._interfaces:
            cname = self.__class__.__name__
            raise AttributeError(f"'{cname}' object has no attribute '{name}'")
        return self._interfaces[name]

    def set_interface(self, what, **kwargs):
        self._interfaces[what].set(**kwargs)

    def update_interface(self, what, **kwargs):
        self._interfaces[what].update(**kwargs)

    def interface_changed(self, what, key, return_vals=False):
        return self._interfaces[what].changed(key, return_vals=return_vals)

    def read(self):
        """Read `feedback_in` from hardware interface"""
        if self.sim:
            sfb = self._interfaces["sim_feedback"].get()
            self._interfaces["feedback_in"].set(**sfb)

    def get_feedback(self):
        """Process `feedback_in` and return `feedback_out` interface"""
        fb_in = self._interfaces["feedback_in"].get()
        self._interfaces["feedback_out"].set(**fb_in)
        return self._interfaces["feedback_out"]

    def set_command(self, **kwargs):
        """Process `command_in` and return `command_out` interface"""
        self._interfaces["command_in"].set(**kwargs)
        self._interfaces["command_out"].set()  # Set defaults
        return self._interfaces["command_out"]

    def set_sim_feedback(self):
        """Process any of command and feedback and set `sim_feedback`
        interface"""
        sfb = self._interfaces["sim_feedback"]
        sfb.set()
        return sfb

    def write(self):
        """Write `command_out` to hardware interface"""
        if self.sim:
            self.set_sim_feedback()

    def log_status(self):
        pass

    @classmethod
    def pkg_path(cls, path):
        """A `pathlib.Path` object for this package's directory"""
        # Find cls's module & package
        pkg = ".".join(cls.__module__.split(".")[:-1])
        with imp_path(pkg, path) as p:
            return p

    def __str__(self):
        return f"<{self.name}@{self.address}>"

    def __repr__(self):
        return self.__str__()

    ########################################
    # Device category and model registries

    # Top-level registry for device category classes
    _category_registry = dict()

    # Allow reregistering devices or not
    allow_rereg = False

    def __init_subclass__(cls, /, **kwargs):
        # Add device type implementations to applicable registries
        cls._register_model()

    @classmethod
    def device_type_key(cls):
        """A unique key by which a detected device's model class may be looked
        up, e.g. `(manufacturer_id, model)`
        """
        return cls.name

    # Record class registrations; for debugging registry
    _registry_log = list()

    @classmethod
    def _register_model(cls):
        # Register model in all parent categories
        if not cls.name:
            return  # Not a model
        key = cls.device_type_key()
        for supercls in cls.__mro__:
            if "category" not in supercls.__dict__:
                continue
            category = supercls.category
            # Ensure category is registered
            supercls._category_registry.setdefault(category, supercls)
            # Check & restister device type
            if "_model_registry" not in supercls.__dict__:
                supercls._model_registry = dict()
            if not cls.allow_rereg and key in supercls._model_registry:
                print(f"allow_rereg:  {cls.allow_rereg}")
                raise KeyError(
                    f"Attempt to re-register {cls} as {supercls.category}!"
                )
            supercls._model_registry[key] = cls
            cls._registry_log.append((category, supercls, key, cls))

    @classmethod
    def category_cls(cls, category=None):
        return cls._category_registry[category or cls.category]

    @classmethod
    def get_model(cls, key=None, category=None):
        category_cls = cls.category_cls(category)
        if key is None:  # Return list of all model classes
            return list(category_cls._model_registry.values())
        if key not in category_cls._model_registry:
            raise NotImplementedError(f'Unrecognized device type "{key}"')
        return category_cls._model_registry[key]

    ########################################
    # Device identifier registry and instance factory

    # keys: device identifiers; values:  device objects
    _address_registry = dict()

    @classmethod
    def get_device(cls, address=None, sim=False, **kwargs):
        registry = cls._address_registry.setdefault(cls.name, dict())
        if address in registry:
            return registry[address]
        device_obj = cls(address=address, sim=sim, **kwargs)
        registry[address] = device_obj
        return device_obj

    @classmethod
    def clear_devices(cls):
        """Clear out device instance registry (for tests)"""
        cls._address_registry.clear()

    @classmethod
    @abc.abstractmethod
    def scan_devices(cls, sim=True):
        """Scan attached devices and return a list of objects

        Typically each device on a bus is scanned for its device type
        key and its device ID.  The type key is used by
        `get_model(key)` to obtain the device class, and the device ID
        is used by `get_device_obj(address)` to obtain the device
        instance.
        """
