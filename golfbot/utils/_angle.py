import math

class Angle:
    def __init__(self, valueRad = 0.0):
        self._angleRad = valueRad

    def __str__(self):
        return f"{self.radians}"

    @property
    def degrees(self) -> float: # convert radians to degrees
        return self._angleRad * 180 / math.pi
    
    @degrees.setter
    def degrees(self, value): # convert degrees to radians
        self._angleRad = value/180 * math.pi
    
    @property
    def radians(self) -> float:
        return self._angleRad
    
    @radians.setter
    def radians(self, value):
        self._angleRad = value
