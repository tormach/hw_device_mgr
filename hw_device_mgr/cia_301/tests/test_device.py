from .base_test_class import BaseCiA301TestClass
from ...tests.test_device import TestDevice as _TestDevice
import pytest
import time


class TestCiA301Device(BaseCiA301TestClass, _TestDevice):
    expected_mro = [
        "CiA301SimDevice",
        "CiA301Device",
        *_TestDevice.expected_mro,
    ]

    # CiA NMT init online & operational status test cases
    read_update_write_package = "hw_device_mgr.cia_301.tests"

    @pytest.fixture
    def obj(self, device_cls, sim_device_data):
        self.obj = self.device_model_cls(address=sim_device_data["address"])
        self.obj.init()
        yield self.obj

    def get_feedback_and_check(self):
        super().get_feedback_and_check()
        # Asynch param download causes a race condition.  When
        # feedback_out.param_state == PARAM_STATE_UPDATING, wait for the
        # param download to complete before the next cycle
        if not hasattr(self.device_class, "PARAM_STATE_UPDATING"):
            # mgr class needs to skip the below
            return
        updating = self.device_class.PARAM_STATE_UPDATING
        if self.test_data["feedback_out"].get("param_state", None) != updating:
            return
        # Spin while we wait on the worker
        timeout, incr = 1, 0.01
        for i in range(int(timeout / incr)):
            if self.obj.config.initialize_params():
                break
            time.sleep(incr)
        else:
            print("initialize_params() never returned True!")
