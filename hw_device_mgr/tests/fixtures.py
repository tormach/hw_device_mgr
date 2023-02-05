from unittest.mock import MagicMock, PropertyMock, patch
from weakref import WeakSet


###############################
# A boilerplate abstract class for stateful fixtures that mock complex
# behaviors


class MockFixture:
    fixture_name = "MockFixture"
    instance_attr = "name"
    mock_methods = []
    mock_attrs = []
    patches = None
    subordinate_classes = []

    _cache = {}

    @classmethod
    def instances(cls):
        """Return a `WeakSet` of instances."""
        if not hasattr(cls, "_instances"):
            cls._instances = WeakSet()
        return cls._instances

    @classmethod
    def clear(cls):
        for c in cls.subordinate_classes:
            c.clear()
        cls.instances().clear()

    def __new__(cls, *args, **kwargs):
        """Cache newly-created instances."""
        obj = object.__new__(cls)
        cls.instances().add(obj)
        return obj

    @property
    def instance_name(self):
        return getattr(self, self.instance_attr, "(anonymous)")

    def __repr__(self):
        return f"<{self.fixture_name} '{self.instance_name}'>"

    def _make_mock_obj(self):
        """
        Build a mock object.

        Object has methods and attributes named by the `mock_methods`
        and `mock_attrs` attributes.
        """
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
        """Build and cache a mock object."""
        if not hasattr(self, "_mock_obj"):
            self._mock_obj = self._make_mock_obj()
        return self._mock_obj

    @classmethod
    def get_mock(cls, *args, **kwargs):
        """Create a new instance and return the mock object."""
        print(f"get_mock:  Creating {cls.fixture_name}:  args {args} {kwargs}")
        obj = cls(*args, **kwargs)
        return obj.mock_obj

    @classmethod
    def get_instance(cls, singleton=True):
        """
        Return an arbitrary, cached instance of the class.

        If `singleton` is set, it is an error if there is not exactly
        one instance.
        """
        # Return an arbitrary instance; if singleton is True, must be
        # *only* instance
        if singleton:
            assert len(cls._instances) == 1
        for inst in cls._instances:  # Get arbitrary (only?) element
            break
        return inst

    @classmethod
    def fixture(cls, name, test_obj, *args):
        """
        Provide a main interface for fixture functions.

        Factory method; creates the mock instance, applies patches,
        yields the mock object, and stops patches.
        """
        mock_obj = cls.get_mock(*args)
        mock_obj.obj.request_inst = test_obj
        if cls.patches is not None:
            patches = cls.patches
        else:
            patches = getattr(test_obj, f"patch_{name}", None)
        if patches is None:
            raise RuntimeError(f"Unable to determine which {name} to patch")
        if isinstance(patches, str):
            patches = (patches,)
        for p in patches:
            print(f"Patching {name} target {p}")
            patch(p, new=mock_obj).start()
        cls.tweak_fixture(mock_obj, mock_obj.obj)
        yield mock_obj
        patch.stopall()

    @classmethod
    def tweak_fixture(cls, mock_obj, obj):
        """
        Provide hook for subclasses to further customize fixture.

        Called during the `fixture` method.
        """
        pass
