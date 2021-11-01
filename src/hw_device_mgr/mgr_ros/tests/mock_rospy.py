from ...tests.fixtures import MockFixture
from unittest.mock import MagicMock
import yaml


class MockRospy(MockFixture):
    fixture_name = "MockRospy"
    mock_methods = [
        "reset",
        "init_node",
        "is_shutdown",
        "has_param",
        "get_param",
        "logdebug",
        "loginfo",
        "logwarn",
        "logerr",
        "Publisher",
        "Service",
        # Out-of-band accessors
        "set_shutdown",
    ]
    mock_attrs = [
        "is_shutdown_max_cycles",
        "is_shutdown_behavior",
    ]

    def __init__(self, params):
        self.params = params
        self.is_shutdown_max_cycles = 3
        self.is_shutdown_behavior = "shutdown"
        self.reset()

    def reset(self):
        self.shutdown = False
        self.cycles = 0
        if hasattr(self, "mock_obj"):
            self.mock_obj.reset_mock()

    def init_node(self, name):
        print(f"rospy.init_node({name})")
        self.name = name

    def is_shutdown(self):
        self.cycles += 1
        if self.shutdown:
            res = True
        elif self.cycles >= self.is_shutdown_max_cycles:
            if self.is_shutdown_behavior == "shutdown":
                res = True
            elif self.is_shutdown_behavior == "raise":
                exc = self.mock_obj.exceptions.ROSInterruptException
                print(f"rospy.is_shutdown():  Raise {exc}")
                raise exc("ROS Interrupt")
        else:
            res = False
        print(f"rospy.is_shutdown() = {res}")
        return res

    def set_shutdown(self):
        self.shutdown = True

    def _lookup(self, name, default=None, params=None):
        if params is None:
            params = self.params
        split = [yaml.safe_load(s) for s in name.split("/", 1)]
        if split is None:
            return default
        if split[0] is None:  # Leading /
            return self._lookup(split[1], default=default)
        elif len(split) == 2:
            if split[0] in params:
                return self._lookup(
                    split[1], default=default, params=params[split[0]]
                )
            else:
                return default
        else:
            if split[0] in params:
                return params.get(split[0], default)
            else:
                return default

    def has_param(self, name):
        res = self._lookup(name) is not None
        print(f"rospy.has_param({name}) -> {res}")
        return res

    def get_param(self, name, default=None):
        res = self._lookup(name, default=default)
        print(f"rospy.get_param({name}, {default}) -> {str(res)[:30]}")
        return res

    def logdebug(self, msg):
        print(f"rospy.logdebug:  '{msg}'")

    def loginfo(self, msg):
        print(f"rospy.loginfo:  '{msg}'")

    def logwarn(self, msg):
        print(f"rospy.logwarn:  '{msg}'")

    def logerr(self, msg):
        print(f"rospy.logerr:  '{msg}'")

    def Publisher(self, name, msg_type, queue_size=0, latch=False):
        print(f"rospy.Publisher({name}, {msg_type}, {queue_size}, {latch})")
        return MagicMock(name=f"rospy.Publisher({name})")

    def Service(self, name, msg_type, callback):
        print(f"rospy.Service({name}, {msg_type}, {callback})")
        return MagicMock(name=f"rospy.Publisher({name})")

    @classmethod
    def tweak_fixture(cls, mock_obj, obj):
        # `rospy.ROSInterruptException()` sets test object
        # `raised_ros_interrupt_exception` attribute to True.
        from rospy.exceptions import ROSInterruptException

        class NewROSInterruptException(ROSInterruptException):
            def __init__(self, *args, **kwargs):
                super().__init__(*args, **kwargs)
                obj.request_inst.raised_ros_interrupt_exception = True

        mock_obj.exceptions.ROSInterruptException = NewROSInterruptException
