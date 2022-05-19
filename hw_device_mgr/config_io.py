import ruamel.yaml
from pathlib import Path
from importlib.resources import open_binary


class ConfigIO:
    @classmethod
    def open_path(cls, path, *args, **kwargs):
        """Return open file object for `path`."""
        path_obj = Path(path)
        return path_obj.open(*args, **kwargs)

    @classmethod
    def open_resource(cls, package, resource):
        """Return open file object for importlib package resource."""
        return open_binary(package, resource)

    @classmethod
    def load_yaml_path(cls, path):
        """Read and return `data` from YAML formatted file `path`."""
        yaml = ruamel.yaml.YAML()
        with cls.open_path(path) as f:
            data = yaml.load(f)
        return data

    @classmethod
    def dump_yaml_path(cls, path, data):
        """Dump `data` in YAML format to `path`."""
        yaml = ruamel.yaml.YAML()
        with cls.open_path(path, "w") as f:
            yaml.dump(data, f)

    @classmethod
    def load_yaml_resource(cls, package, resource):
        """Load YAML from importlib package resource."""
        yaml = ruamel.yaml.YAML()
        with cls.open_resource(package, resource) as f:
            data = yaml.load(f)
        return data
