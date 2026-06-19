import math

class Angle:
    def __init__(self):
        self._angleRad = 0

    @property
    def degrees(self): # convert radians to degrees
        return self._angleRad * 180 / math.pi
    
    @degrees.setter
    def degrees(self, value): # convert degrees to radians
        self._angleRad = value/180 * math.pi
    
    @property
    def radians(self):
        return self._angleRad
    
    @radians.setter
    def radians(self, value):
        self._angleRad = value
