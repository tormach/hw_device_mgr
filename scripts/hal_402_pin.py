# -*- coding: utf-8 -*-
class pin_402(object):
    def __init__(self, name, dir, type, bit_pos):
        self.name = name
        self.dir = dir
        self.type = type
        self.bit_pos = bit_pos
        self.halpin = None
        self.local_pin_value = None

    def set_parent_comp(self, component):
        self.parent_comp = component

    def create_halpin(self):
        self.halpin = self.parent_comp.newpin(self.name, self.type, self.dir)

    def set_local_value(self):
        # put HAL pin value in lacal_pin_value
        self.local_pin_value = self.halpin.get()
