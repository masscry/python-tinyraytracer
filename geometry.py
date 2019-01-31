#!/usr/bin/env python
# -*- coding: utf-8 -*-

from math import sqrt
from math import pi
from math import tan

class vec4(object):

    def __init__(self, x, y, z, w):
        self.data = (x, y, z, w)

    def __getitem__(self, index):
        return self.data[index]


class vec3(object):

    def __init__(self, x, y, z):
        self.data = (x, y, z)

    def __getitem__(self, index):
        return self.data[index]

    def get_x(self):
        return self.data[0]

    def get_y(self):
        return self.data[1]

    def get_z(self):
        return self.data[2]

    def set_x(self, val):
        self.data = (val, self.data[1], self.data[2])

    def set_y(self, val):
        self.data = (self.data[0], val, self.data[2])

    def set_z(self, val):
        self.data = (self.data[0], self.data[1], val)

    x = property(get_x, set_x)
    y = property(get_y, set_y)
    z = property(get_z, set_z)

    def norm(self):
        return sqrt(self.data[0]*self.data[0]
            + self.data[1]*self.data[1]
            + self.data[2]*self.data[2]
        )

    def normalize(self):
        len = self.norm()

        self.data = (
            self.data[0] / len,            
            self.data[1] / len,            
            self.data[2] / len 
        )

        return self

    def __mul__(self, other):
        if type(other) is float:
            return vec3(
                self[0]*other,
                self[1]*other,
                self[2]*other
            )
        return self[0]*other[0] + self[1]*other[1] + self[2]*other[2]

    def __add__(self, other):
        return vec3(
            self[0] + other[0],
            self[1] + other[1],
            self[2] + other[2]
        )

    def __sub__(self, other):
        return vec3(
            self[0] - other[0],
            self[1] - other[1],
            self[2] - other[2]
        )

    def __neg__(self):
        return vec3(
            -self[0],
            -self[1],
            -self[2]
        )
    
    def cross(self, other):
        return vec3(
            self[1]*other[2] - self[2]*other[1],
            self[2]*other[0] - self[0]*other[2],
            self[0]*other[1] - self[1]*other[0]
        )

    def __str__(self):
        return '{} {} {}'.format(*self.data)
    