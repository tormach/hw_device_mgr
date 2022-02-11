from ...cia_301.tests.base_test_class import (
    BaseCiA301TestClass as _BaseCiA301TestClass,
)
from .bogus_devices.device import (
    BogusCiA402Device,
    BogusCiA402Servo,
    BogusCiA402Servo2,
)


class BaseCiA402TestClass(_BaseCiA301TestClass):

    # Device scan data; for test fixture
    device_data_yaml = "cia_402/tests/bogus_devices/device.yaml"

    # test_read_update_write() configuration
    read_update_write_yaml = "cia_402/tests/read_update_write.cases.yaml"

    # Classes under test in this module
    device_class = BogusCiA402Device
    device_model_classes = (BogusCiA402Servo, BogusCiA402Servo2)
