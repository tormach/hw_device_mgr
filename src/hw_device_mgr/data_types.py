import re

__all__ = ("DataType",)


class DataType:
    """Base class for data types

    CAN 301 and ETG.2000 each define a group of standard data types:
    a bit type and various numeric and string types.  The two
    standards share underlying data types, but name them differently.

    This module contains several subclasses representing such a group
    of those underlying data types.  This abstract parent class
    provides a scheme for defining the characteristics of a single
    data type, such as bit length and signedness for an `int` type.
    It also maintains registries for looking up the member types.

    The CiA 301 and ETG.2000 standard data type groups are implemented
    as abstract subclasses of this class.  Their group members carry
    the standard's name in the `name` attribute, and the underlying
    type's commmon name in the `shared_name` attribute, for example
    the `Int8Type` defined in this module has `shared_name` attribute
    `uint8`; its subclass with the same `shared_name`,
    `hw_device_mgr.ethercat.data_types.EtherCATInt8Type` representing
    the ETG.2000 type, has `name` `USINT`; and its counterpart,
    `hw_device_mgr.cia_301.data_types.CiA301Int8Type` has
    `shared_name` `UINT8`.  This scheme enables code to be reused
    while being agnostic to CAN or EtherCAT and to the specific data
    type.

    A concrete data type subclass will define the following attributes:
    - `shared_name`:  The underlying data type name (described above)
    - `shared_name_re`:  An optional regex for matching `shared_name`
    - `name`:  The group-specific name (described above)
    - `name_re`:  An optional regex for matching `name`
    - `base_type`:  The base Python type
    - `num_bits`:  Number of bits; `None` for variable

    The data type subclasses in this module are defined as
    conventional Python classes, but this class contains convenience
    code for generating data type subclasses of a new group from
    shorthand parent class attributes; see the CAN or EtherCAT data
    type modules for examples.
    """

    # Subclass instance attributes
    name = None  # String for type subclass-specific name
    name_re = None  # Regex to match subclass-specific name; default: name
    shared_name = None  # String for type shared name
    shared_name_re = None  # Regex to match shared name
    base_type = None  # Base Python type
    num_bits = None  # Number of bits
    subtype_data = dict()  # Attributes added to generated datatype subclasses

    # Registries for base `DataType` class only
    _name_re_registry = dict()
    _shared_name_registry = dict()
    _shared_name_re_registry = dict()

    def __init_subclass__(cls, /, **kwargs):
        if "subtype_prefix" in cls.__dict__:
            # Abstract (base) subclass
            cls._generate_subtypes()
        else:
            # Data type subclass
            cls._shared_name_registry[cls.shared_name] = cls
            shared_re_compiled = re.compile(r"^" + cls.shared_name_re + r"$")
            cls._shared_name_re_registry[shared_re_compiled] = cls
            for supercls in cls.__mro__:  # Find abstract class
                if "_shared_name_registry" in supercls.__dict__:
                    setattr(supercls, cls.shared_name, cls)
                    break
            if cls.name is not None:  # Device-specific data type
                name_re = cls.name_re if cls.name_re else cls.name
                re_compiled = re.compile(r"^" + name_re + r"$")
                cls._name_re_registry[re_compiled] = cls

    @classmethod
    def _generate_subtypes(cls):
        # Registries mapping type `[shared_]name` attrs/regexes to subclasses
        cls._name_re_registry = dict()
        cls._shared_name_registry = dict()
        cls._shared_name_re_registry = dict()

        # Name prefix for generated data type classes
        subtype_prefix = getattr(cls, "subtype_prefix", None)
        if subtype_prefix is None:
            return  # Nothing to do
        # List of parent classes to inherit from
        parent_cls = super(cls, cls)
        if not hasattr(parent_cls, "_shared_name_re_registry"):
            return  # Base `DataType` class, or unexpected MRO scheme
        # Generate subclasses
        for (
            shared_name,
            parent_subtype,
        ) in parent_cls._shared_name_registry.items():
            # subtype_data may be sparse (not all types supported); be
            # complete anyway
            attributes = cls.subtype_data.get(shared_name, dict())
            # Parent class subtypes to inherit from
            parent_subtypes = []
            for i in cls.__mro__:
                if "_shared_name_registry" not in i.__dict__:
                    continue
                if shared_name not in i._shared_name_registry:
                    continue
                parent_subtypes.append(i._shared_name_registry[shared_name])
            bases = tuple((cls, *parent_subtypes))
            # Create the class; __init_subclass__ will handle the rest
            type(
                # Subtype class name
                subtype_prefix + parent_subtype.__name__,
                # Bases
                bases,
                # Data members + methods
                attributes,
            )

    def __new__(cls, raw_val, *args, **kwargs):
        if isinstance(raw_val, cls):
            return raw_val  # No need for new instance
        return cls.base_type.__new__(cls, cls._conv(raw_val), *args, **kwargs)

    @classmethod
    def _conv(cls, raw_val):
        # Default conversion:  Use base type constructor
        return cls.base_type(raw_val)

    @classmethod
    def by_name(cls, name):
        for test_re, test_cls in cls._name_re_registry.items():
            if test_re.match(name):
                return test_cls
        raise KeyError(f'{cls.__name__}:  Unknown data type name "{name}"')

    @classmethod
    def by_shared_name(cls, name):
        for test_re, test_cls in cls._shared_name_re_registry.items():
            if test_re.match(name):
                return test_cls
        raise KeyError(f'Unknown data type shared name "{name}"')

    @classmethod
    def all_types(cls):
        return cls._shared_name_registry.values()

    def __str__(self):
        return str(self.base_type(self))


class BitType(DataType, int):
    shared_name = "bit"
    shared_name_re = r"bit"
    base_type = int
    num_bits = 1

    @staticmethod
    def _conv(x):
        return 1 if int(x) else 0


class Int8Type(DataType, int):
    shared_name = "int8"
    shared_name_re = r"int8"
    base_type = int
    signed = False
    num_bits = 8

    def _conv(x):
        return int(x, 0) if isinstance(x, (str, bytes, bytearray)) else int(x)


class Int16Type(Int8Type):
    shared_name = "int16"
    shared_name_re = r"int16"
    num_bits = 16


class Int32Type(Int8Type):
    shared_name = "int32"
    shared_name_re = r"int32"
    num_bits = 32


class Int64Type(Int8Type):
    shared_name = "int64"
    shared_name_re = r"int64"
    num_bits = 64


class UInt8Type(Int8Type):
    shared_name = "uint8"
    shared_name_re = r"uint8"
    signed = True
    num_bits = 8

    def __str__(self):
        # Print unsigned ints in fixed-width hex representation
        return ("0x{0:0%dX}" % int(self.num_bits / 4)).format(self)

    def __repr__(self):
        return self.__str__()


class UInt16Type(UInt8Type):
    shared_name = "uint16"
    shared_name_re = r"uint16"
    num_bits = 16


class UInt32Type(UInt8Type):
    shared_name = "uint32"
    shared_name_re = r"uint32"
    num_bits = 32


class UInt64Type(UInt8Type):
    shared_name = "uint64"
    shared_name_re = r"uint64"
    num_bits = 64


class FloatType(DataType, float):
    shared_name = "float"
    shared_name_re = r"float"
    base_type = float
    num_bits = 32


class DoubleType(FloatType):
    shared_name = "double"
    shared_name_re = r"double"
    num_bits = 64


class StringType(DataType, str):
    shared_name = "str"
    shared_name_re = r"str"
    base_type = str
    num_bits = None  # Variable length

    def __str__(self):
        return self
