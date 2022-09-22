from ..device import Device, SimDevice
from ..data_types import DataType
import ruamel.yaml


class ErrorDevice(Device):
    """
    Abstract class representing a device error code handling.

    Error code is fed into `error_code` feedback.

    The `set_feedback()` method looks up the error code in the
    `device_err/{name}.yaml` file and adds `description` and `advice`
    strings to feedback.
    """

    device_error_dir = "device_err"

    feedback_in_data_types = dict(error_code="uint32")
    feedback_in_defaults = dict(error_code=0)

    feedback_out_defaults = dict(
        error_code=0, description="No error", advice="No error"
    )

    no_error = feedback_out_defaults

    data_type_class = DataType

    _error_descriptions = dict()

    @classmethod
    def error_descriptions_yaml(cls):
        return cls.pkg_path(cls.device_error_dir) / f"{cls.name}.yaml"

    @classmethod
    def error_descriptions(cls):
        """
        Return dictionary of error code data.

        Data is read from YAML file `{device_error_dir}/{name}.yaml` and
        cached.
        """
        if cls.name not in cls._error_descriptions:
            errs = cls._error_descriptions[cls.name] = dict()
            path = cls.error_descriptions_yaml()
            if path.exists():
                yaml = ruamel.yaml.YAML()
                with path.open() as f:
                    err_yaml = yaml.load(f)
                for err_code_str, err_data in err_yaml.items():
                    errs[int(err_code_str, 0)] = err_data
        return cls._error_descriptions[cls.name]

    def get_feedback(self):
        fb_out = super().get_feedback()
        error_code = self.feedback_in.get("error_code")
        if not error_code:
            self.feedback_out.update(**self.no_error)
            return fb_out

        error_info = self.error_descriptions().get(error_code, None)
        if error_info is None:
            fb_out.update(
                description=f"Unknown error code {error_code}",
                advice="Please consult with hardware vendor",
                error_code=error_code,
            )
            return fb_out
        else:
            fb_out.update(error_code=error_code, **error_info)
        if fb_out.changed("error_code"):
            msg = "error code {}:  {}".format(
                error_code, fb_out.get("description")
            )
            self.logger.error(msg)
        return fb_out


class ErrorSimDevice(ErrorDevice, SimDevice):
    """Abstract class representing a device simulated error code handling."""
