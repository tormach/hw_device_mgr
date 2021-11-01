from .base_test_class import BaseROSHALMgrTestClass
from ...mgr_ros.tests.test_mgr import TestROSDeviceMgr as _TestROSDeviceMgr
from ...mgr_hal.tests.test_mgr import TestHALHWDeviceMgr as _TestHALHWDeviceMgr
import pytest


class TestROSDeviceMgr(
    BaseROSHALMgrTestClass, _TestHALHWDeviceMgr, _TestROSDeviceMgr
):

    expected_mro = [
        "ROSHALHWDeviceMgr",
        "ROSHWDeviceMgr",
        "HALHWDeviceMgr",
        "HWDeviceMgr",
        "FysomGlobalMixin",
        "HALCompDevice",
        "HALPinDevice",
        "Device",
        "ABC",
        "HALMixin",
        "object",
    ]

    @pytest.fixture
    def obj(self, device_cls):
        # init() and init_devices() signatures changed
        self.obj = device_cls(sim=self.sim)
        self.obj.init()
        self.obj.init_devices()
        yield self.obj
