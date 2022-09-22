import pytest

try:
    import rclpy  # noqa: F401
except ModuleNotFoundError:
    pytest.skip(allow_module_level=True)

from ..ros import ROSLogging
from .test_logging import TestLogging as _TestLogging


class TestROSLogging(_TestLogging):
    tc = ROSLogging
