from .base_test_class import BaseROSMgrTestClass
from ...mgr.tests.test_mgr_read_update_write import (
    TestHWDeviceMgrRUW as _TestHWDeviceMgrRUW,
)
import pytest


class TestROSHWDeviceMgrRUW(BaseROSMgrTestClass, _TestHWDeviceMgrRUW):
    rclpy_patches = [
        "hw_device_mgr.mgr_ros.mgr.rclpy",
    ]

    @pytest.fixture
    def obj(self, category_cls):
        # init_sim() and init_devices() signatures changed, so can't
        # use parent test class obj fixture
        self.obj = self.device_class()
        self.obj.init(argv=list())
        yield self.obj
