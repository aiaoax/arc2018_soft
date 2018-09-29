#!/usr/bin/python
# -*- coding: utf-8 -*-
# license removed for brevity

from enum import IntEnum
class Direction(IntEnum):
    AHEAD = 1
    BACK  = 2
    RIGHT = 3
    LEFT  = 4

class Speed(IntEnum):
    LOW    = 1 
    MIDDLE = 2
    HILH   = 3
