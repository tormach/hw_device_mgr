import pytest
from ...cia_402.tests.base_test_class import BaseCiA402TestClass
from ..data_types import HALDataType
from .bogus_devices.device import (
    BogusHALDevice,
    BogusHALV1Servo,
    BogusHALV2Servo,
    BogusHALV1IO,
)
from .mock_hal import MockHALComponent


class BaseHALTestClass(BaseCiA402TestClass):

    # Classes under test in this module
    data_type_class = HALDataType
    device_class = BogusHALDevice
    device_model_classes = BogusHALV1Servo, BogusHALV2Servo, BogusHALV1IO

    @pytest.fixture
    def mock_halcomp(self):
        """
        Fixture for mocking a HAL component.

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
        comp_name = getattr(self, "halcomp_name", "foocomp")
        self.pin_vals = dict()
        yield from MockHALComponent.fixture(
            "mock_halcomp", self, comp_name, self.pin_vals
        )

    @pytest.fixture()
    def mock_hal(self, mocker):
        """
        Fixture for mocking `hal.component`.

        This fixture patches `hal.component` and returns the mock class
        in test function `mock_hal` argument and test object `mock_hal`
        attribute.  See the `mock_halcomp` fixture for more information
        about the mock instances of this mock class.
        """

        def mock_component(comp_name):
            comp = MockHALComponent.get_mock(comp_name, self.pin_vals)
            self.mock_halcomp = comp
            self.get_pin = comp.obj.get_pin_val
            self.set_pin = comp.obj.set_pin_val
            return comp

        self.pin_vals = dict()
        mock_hal = mocker.MagicMock()
        mock_hal.component = mocker.MagicMock(
            name="mock_hal_component", side_effect=mock_component
        )
        self.mock_hal = mock_hal
        self.components = MockHALComponent
        with mocker.patch("hal.component", side_effect=mock_hal.component):
            yield mock_hal
        MockHALComponent.clear()

    @pytest.fixture
    def extra_fixtures(self, mock_hal):
        pass
