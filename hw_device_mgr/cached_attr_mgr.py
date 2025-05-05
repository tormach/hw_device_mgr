class CachedAttrMixin:

    def clear_cached_properties(self, *args):
        """
        Clear `cached_property` and `lru_func` values.

        Subclasses should remove cached values defined in their class by
        overloading this function:

            def clear_cached_properties(self, *args):
                super().clear_cached_properties("prop1", "prop2", *args)
        """
        props = set(args)
        for prop in props:
            self.__dict__.pop(prop, None)
