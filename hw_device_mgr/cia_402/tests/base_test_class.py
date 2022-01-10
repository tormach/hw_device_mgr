from ...cia_301.tests.base_test_class import (
    BaseCiA301TestClass as _BaseCiA301TestClass,
)
from .bogus_devices.device import (
    BogusCiA402Device,
    BogusV1CiA402Servo,
    BogusV2CiA402Servo,
)


class BaseCiA402TestClass(_BaseCiA301TestClass):

    # test_read_update_write() configuration
    read_update_write_yaml = "cia_402/tests/read_update_write.cases.yaml"

    # Classes under test in this module
    device_class = BogusCiA402Device
    device_model_classes = (BogusV1CiA402Servo, BogusV2CiA402Servo)
