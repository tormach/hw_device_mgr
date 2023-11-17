import abc
from .logging import LoggingMixin
from .interface import Interface
from .data_types import DataType
from functools import cached_property
import re
import time


class Device(LoggingMixin, abc.ABC):
    """Base device class for both device categories and device models."""

    category = "all"  # Device category sets up registry for models
    name = None  # Concrete device model subclasses must define

    data_type_class = DataType
    interface_class = Interface

    feedback_in_data_types = dict()
    feedback_out_data_types = dict(
        goal_reached="bit",
        goal_reason="str",
        fault="bit",
        fault_desc="str",
    )
    command_in_data_types = dict()
    command_out_data_types = dict()

    feedback_in_defaults = dict()
    feedback_out_defaults = dict(
        goal_reached=True,
        goal_reason="Reached",
        fault=False,
        fault_desc="",
    )
    command_in_defaults = dict()
    command_out_defaults = dict()

    interface_names = {
        "feedback_in",
        "feedback_out",
        "command_in",
        "command_out",
    }

    goal_reached_timeout = 10  # seconds

    @classmethod
    def canon_address(cls, address):
        """Canonicalize a device address."""
        # For the base class, just ensure that it's `None` or a tuple.
        return None if address is None else tuple(address)

    def __init__(self, address=None):
        self.address = self.canon_address(address)
        self._timeout = None

    def logging_name(self):
        return f"{self.category}.{self}"

    def init(self):
        """
        Initialize device and interfaces.

        Subclasses may implement `init()` for extra initialization
        outside the constructor.  Implementations should always call
        `super().init()`.
        """
        self.init_interfaces()

    def exit(self):
        """Clean up before exiting."""
        pass

    @classmethod
    def merge_dict_attrs(cls, attr):
        """
        Merge `dict` attributes across class hierarchy.

        Scan through class and parent classes for `attr`, a `dict`, and
        return merged `dict`.
        """
        res = dict()
        for c in cls.__mro__:
            c_attr = c.__dict__.get(attr, dict())
            # Overlap not allowed
            assert not (set(res.keys()) & set(c_attr.keys()))
            res.update(c_attr)
        return res

    slug_separator = "."

    @cached_property
    def addr_slug(self):
        """
        Return a slug generated from the device address.

        The slug is computed by separating numeric components of the
        device `address` string with the `slug_separator` character,
        default `.`, e.g. `(0,5)` -> `0.5`.  This is intended to be
        useful for inclusion into identifiers.
        """
        addr_prefix = re.sub(r"[^0-9]+", self.slug_separator, str(self.address))
        return addr_prefix.strip(self.slug_separator)

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
            intfs[name] = self.interface_class(name, defaults, data_types)

    def interface(self, name):
        return self._interfaces[name]

    def __getattr__(self, name):
        """Provide attributes for each interface."""
        if name not in self.interface_names:
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
        """Read `feedback_in` from hardware interface."""
        self._interfaces["feedback_in"].set()

    def get_feedback(self) -> Interface:
        """Process `feedback_in` and return `feedback_out` interface."""
        fb_in = self._interfaces["feedback_in"].get()
        timeout = self.check_and_set_timeout()
        fb_out = self._interfaces["feedback_out"]
        fb_out.set(**fb_in)
        if timeout:
            fb_out.update(fault=True, fault_desc=timeout)
        return fb_out

    def check_and_set_timeout(self):
        """Set fault if feedback_out goal_reached is False for too long."""
        # This is still data from previous cycle
        fb_out = self._interfaces["feedback_out"]
        old_goal_reached = fb_out.get("goal_reached")
        old_fault = fb_out.get("fault")

        # Cancel timer & return if goal reached or fault in previous cycle
        if old_goal_reached or old_fault:
            if self._timeout is not None:
                self._timeout = None
                self.logger.debug("Cleared timeout")
            return False

        # Otherwise, a timer should be running and monitored
        if self._timeout is None:
            # No timer running; set one
            self.set_timeout(self.goal_reached_timeout)
            self.logger.debug(f"Set {self.goal_reached_timeout}s timeout")
        elif time.time() > self._timeout:
            # Timeout; set fault
            reason = fb_out.get_old("goal_reason")
            msg = f"Timeout ({self.goal_reached_timeout}s):  {reason}"
            self.logger.error(msg)
            return msg
        return False

    def set_timeout(self, timeout_seconds):
        """Set timeout for `timeout_seconds` in the future."""
        self._timeout = time.time() + timeout_seconds

    def set_command(self, **kwargs) -> Interface:
        """Process `command_in` and return `command_out` interface."""
        self._interfaces["command_in"].set(**kwargs)
        self._interfaces["command_out"].set()  # Set defaults
        return self._interfaces["command_out"]

    def write(self):
        """Write `command_out` to hardware interface."""

    def __str__(self):
        return f"{self.name}@{str(self.address).replace(' ', '')}"

    def __repr__(self):
        return f"<{self.__str__()}>"

    ########################################
    # Device category and model registries

    # Top-level registry for device category classes
    _category_registry = dict()

    # Allow reregistering devices or not
    allow_rereg = False

    def __init_subclass__(cls, /, **kwargs):  # noqa:  E225
        # Add device type implementations to applicable registries
        cls._register_model()

    @classmethod
    def device_model_id(cls):
        """
        Return unique device model identifier.

        A unique ID that may be generated from bus scan results by which
        a detected device's model class may be looked up, e.g.
        `(manufacturer_id, model)`.
        """
        if not hasattr(cls, "model_id"):
            return None
        return cls.data_type_class.uint32(cls.model_id)

    # Record class registrations; for debugging registry
    _registry_log = list()

    # { category : { model_id : device_class } }
    _model_id_registry = dict()

    # { category : { model_name : device_class } }
    _model_name_registry = dict()

    @classmethod
    def _register_model(cls):
        # Register model in all parent categories
        if not cls.name:
            # Not a concrete device; skip
            cls._registry_log.append(("no_name", cls))
            return  # Not a model
        model_id = cls.device_model_id()
        registered = False
        for supercls in cls.category_classes():
            category = supercls.category
            # Ensure category is registered
            cls._category_registry.setdefault(category, supercls)
            # Check & register device id
            reg = cls._model_id_registry.setdefault(category, dict())
            # Be sure model is registered in at least one category, but don't
            # clobber earlier registrations
            assert (
                model_id not in reg or registered
            ), f"Duplicate model_id {model_id}"
            if model_id not in reg:
                registered = True
                reg[model_id] = cls
            # Register device name
            reg = cls._model_name_registry.setdefault(category, dict())
            assert cls.name not in reg, f"{cls.name} in {category} registry"
            reg[cls.name] = cls
            cls._registry_log.append(
                ("cat", cls.name, model_id, category, cls, supercls)
            )

    @classmethod
    def category_classes(cls, model_cls=None):
        if model_cls is None:
            model_cls = cls
        return [c for c in model_cls.__mro__ if "category" in c.__dict__]

    @classmethod
    def category_cls(cls, category=None):
        return cls._category_registry.get(category or cls.category, None)

    @classmethod
    def get_model(cls, model_id=None):
        category = cls.category
        assert (
            category in cls._model_id_registry
        ), f"{category} not in {cls._model_id_registry}"
        model_registry = cls._model_id_registry[category]
        if model_id is None:  # Return set of all model classes
            return set(model_registry.values())
        if model_id not in model_registry:
            return None
        return model_registry[model_id]

    @classmethod
    def get_model_by_name(cls, name):
        category = cls.category
        assert category in cls._model_name_registry
        model_registry = cls._model_name_registry[category]
        assert name in model_registry, f"{name} not in {model_registry}"
        return model_registry[name]

    ########################################
    # Device identifier registry and instance factory

    # keys: device identifiers; values:  device objects
    _address_registry = dict()

    @classmethod
    def get_device(cls, address=None, **kwargs):
        address = cls.canon_address(address)
        registry = cls._address_registry.setdefault(cls.name, dict())
        if address in registry:
            return registry[address]
        device_obj = cls(address=address, **kwargs)
        registry[address] = device_obj
        return device_obj

    @classmethod
    def clear_devices(cls):
        """Clear out device instance registry (for tests)."""
        cls._address_registry.clear()

    @classmethod
    @abc.abstractmethod
    def scan_devices(cls):
        """
        Scan attached devices and return a list of objects.

        Typically each device on a bus is scanned for its device type
        key and its device ID.  The type key is used by `get_model(key)`
        to obtain the device class, and the device ID is used by
        `get_device_obj(address)` to obtain the device instance.
        """

    @classmethod
    def dot(cls):
        classes = set()
        links = set()
        for dev_cls in cls.get_model():
            cls.dot_link(dev_cls, classes, links)
        rank_same_node = "; ".join(
            c.dot_str()
            for c in classes
            if "category" in c.__dict__ and c.category != "all"
        )
        rank_same_leaf = "; ".join(c.dot_str() for c in classes if c.name)
        gv = "strict digraph Devices {\n"
        gv += "  rankdir=LR\n"
        gv += "  subgraph {\n"
        for dev_cls in classes:
            gv += f"    {dev_cls.dot_str()}"
            gv += f"      [style=filled, fillcolor={dev_cls.dot_color()}];\n"
        for parent, child in links:
            gv += f"    {parent.dot_str()} -> {child.dot_str()};\n"
        gv += f"  {{rank = same; {rank_same_leaf};}}\n"
        gv += f"  {{rank = same; {rank_same_node};}}\n"
        gv += "  }\n"
        gv += "}\n"
        return gv

    @classmethod
    def dot_link(cls, dev_cls, classes, links):
        classes.add(dev_cls)
        for base in dev_cls.__bases__:
            if not issubclass(base, Device):
                continue  # Ignore non-Device subclasses
            classes.add(base)
            links.add((base, dev_cls))
            cls.dot_link(base, classes, links)

    @classmethod
    def dot_str(cls):
        if cls.name:
            type_name = f"{cls.name}\\n"
        elif "category" in cls.__dict__:
            type_name = f"{cls.category}\\n"
        else:
            type_name = ""
        return f'"{type_name}{cls.__name__}"'

    @classmethod
    def dot_color(cls):
        if cls.name:
            return "green"
        elif "category" in cls.__dict__:
            return "white"
        else:
            return "lightblue"


class SimDevice(Device):
    sim_feedback_data_types = dict()
    sim_feedback_defaults = dict()

    interface_names = {
        "feedback_in",
        "feedback_out",
        "command_in",
        "command_out",
        "sim_feedback",
    }

    _sim_device_data = dict()

    @classmethod
    def sim_device_data_class(cls, sim_device_data):
        return cls.get_model(sim_device_data["model_id"])

    @classmethod
    def sim_device_data_address(cls, sim_device_data):
        return cls.canon_address(sim_device_data["address"])

    @classmethod
    def init_sim(cls, /, sim_device_data):
        """
        Create sim device objects for tests.

        Construct sim device objects with device class, address, etc.
        from `sim_device_data`.
        """
        cls_sim_data = cls._sim_device_data[cls.category] = dict()

        for dev in sim_device_data:
            device_cls = cls.sim_device_data_class(dev)
            address = cls.sim_device_data_address(dev)

            # Set sparse keys
            updates = dict(
                model_id=device_cls.device_model_id(),
                name=device_cls.name,
                address=address,
            )
            cls_sim_data[address] = {**dev, **updates}

        assert cls_sim_data

    @classmethod
    def scan_devices(cls, **kwargs):
        res = list()
        cls_sim_data = cls._sim_device_data[cls.category]
        for data in cls_sim_data.values():
            dev_type = cls.get_model(data["model_id"])
            address = data["address"]
            dev = dev_type.get_device(address=address, **kwargs)
            res.append(dev)
        return res

    def read(self):
        """Read `feedback_in` from hardware interface."""
        super().read()
        sfb = self._interfaces["sim_feedback"].get()
        self._interfaces["feedback_in"].update(**sfb)

    def set_sim_feedback(self):
        """Simulate feedback from command and feedback."""
        sfb = self._interfaces["sim_feedback"]
        sfb.set()
        return sfb

    def write(self):
        """Write `command_out` to hardware interface."""
        super().write()
        self.set_sim_feedback()
