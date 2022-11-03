class Interface:
    """Represents an interface, e.g. feedback or command."""

    def __init__(self, name, defaults=None, data_types=None):
        self.name = name
        self.data_types = data_types.copy() if data_types else dict()
        self.defaults = self.set_types(**(defaults or dict()))
        self.values = dict()  # Make attribute exist
        self.set(**defaults)  # Set values to defaults
        self.set(**defaults)  # Set old values to defaults

    def add_attribute(self, attr, default, data_type):
        assert (
            attr not in self.defaults
        ), f"Attempt to redefine attribute '{attr}' in interface {self.name}"
        self.data_types[attr] = data_type
        kwargs = {attr: default}
        self.defaults.update(self.set_types(**kwargs))
        self.set(**kwargs)  # Set value to default
        self.set(**kwargs)  # Set old value to default

    def set_types(self, **values):
        """Set data types for values if a type is defined for that value."""
        for key, data_type in self.data_types.items():
            if key in values:
                values[key] = data_type(values[key])
        return values

    def keys(self):
        return self.values.keys()

    def set(self, **values):
        self.values_old = self.values
        self.values = self.defaults.copy()
        self.values.update(self.set_types(**values))

    def update(self, **values):
        self.values.update(self.set_types(**values))

    def get(self, key=None):
        return self.values if key is None else self.values[key]

    def get_data_type(self, key):
        return self.data_types[key]

    def changed(self, key, return_vals=False):
        val = self.values[key]
        val_old = self.values_old[key]
        return (val, val_old) if return_vals else val != val_old

    def rising_edge(self, key):
        return self.values[key] and not self.values_old[key]

    def __str__(self):
        res = f"Interface {self.name} ("
        res += ", ".join([f"{k}={v}" for k, v in self.values.items()])
        res += ")"
        return res

    def __repr__(self):
        return f"<{str(self)}>"
