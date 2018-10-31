#!/usr/bin/python
# -*- coding: utf-8 -*-
# license removed for brevity

from enum import IntEnum
class Direction(IntEnum):
    AHEAD = 1
    BACK  = 2
#    RIGHT = 3
#    LEFT  = 4
#    STOP  = 5

class Speed(IntEnum):
    LOW    = 1 
    MIDDLE = 2
    HIGH   = 3

class Arm(IntEnum):
    PLUS  = 1
    NONE  = 2
    MINUS = 3

class Mode(IntEnum):
    BOOT    = 1
    HERVEST = 2
    BULB    = 3
