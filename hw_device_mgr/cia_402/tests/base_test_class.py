from ...cia_301.tests.base_test_class import BaseCiA301TestClass
from .bogus_devices.device import (
    BogusCiA402Device,
    BogusCiA402V1Servo,
    BogusCiA402V2Servo,
    BogusCiA402V1IO,
)


class BaseCiA402TestClass(BaseCiA301TestClass):

    # test_read_update_write_402() configuration
    read_update_write_402_yaml = "cia_402/tests/read_update_write.cases.yaml"

    # Classes under test in this module
    device_class = BogusCiA402Device
    device_model_classes = (
        BogusCiA402V1Servo,
        BogusCiA402V2Servo,
        BogusCiA402V1IO,
    )
