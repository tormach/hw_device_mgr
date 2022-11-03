from ..interface import Interface


class DebugInterface(Interface):
    """Interface for testing that verifies `update()` keys."""

    def update(self, **values):
        for key in values:
            assert key in self.defaults, f"Unknown interface key {key}"
        super().update(**values)
