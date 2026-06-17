import numpy as np
from collections.abc import Sequence
from typing import Self


class Vector2(Sequence[float]):    
    def __init__(self, x: float, y: float):
        self._arr = np.array([x, y])
        self._idx = 0

    def __add__(self, other: Sequence[float]) -> Self:
        if len(other) != 2:
            raise ValueError("Length of array must be 2")
        
        added = np.add(self, other)
        return Vector2(added[0], added[1])
    
    def __sub__(self, other: Sequence[float]) -> Self:
        if len(other) != 2:
            raise ValueError("Length of array must be 2")

        subtracted = np.subtract(self, other)
        return Vector2(subtracted[0], subtracted[1])
    
    def __mul__(self, scalar: float) -> Self:
        multiplied = np.multiply(self._arr, scalar)
        return Vector2(multiplied[0], multiplied[1])
        
    def __neg__(self):
        negaed = 0 - self._arr
        return Vector2(negaed[0], negaed[1])

    def __len__(self):
        return 2
    
    def __getitem__(self, index: int) -> float:
        return self._arr[index]
    
    def __eq__(self, other: Sequence[float]):
        return len(other) == 2 and self._arr[0] == other[0] and self._arr[1] == other[1]

    def __str__(self):
        return str(self._arr)
    
    def __format__(self, format_spec):
        return format(self._arr, format_spec)

    def rotate(self, angle: float) -> Self:
        """
        Calculates the rotated vector of self when being rotated by angle

        Args:
            angle (float): The angle to rotate by in radian

        Returns:
            A new rotated vector
        """
        rotated = np.array(np.matrix([[np.cos(angle), np.sin(angle)],
                   [-np.sin(angle), np.cos(angle)]]) @ self._arr)
        return Vector2(rotated[0][0], rotated[0][1])

    @staticmethod
    def dot(a: Sequence[float], b: Sequence[float]) -> float:
        """
        Calculates the dot product between vectors a and b

        Args:
            a (ArrayLike): An iterable object with length 2
            b (ArrayLike): An iterable object with length 2

        Returns:
            The dot product between the vectors
        """
        if len(a) != len(b) or len(a) != 2 or len(b) != 2:
            raise ValueError("Length of arrays must be 2")
        return np.dot(a, b)

    @staticmethod
    def project(a: Sequence[float], b: Sequence[float]) -> Self:
        """
        Projects the vector a onto b

        Args:
            a (ArrayLike): The vector to project onto b
            b (ArrayLike): The vector being projected on
        
        Returns:
            The resulting projected vector
        """
        if len(a) != len(b) or len(a) != 2 or len(b) != 2:
            raise ValueError("Length of arrays must be 2")

        dotProd = np.dot(a, b)/np.dot(b, b)

        return Vector2(b[0]*dotProd, b[1]*dotProd)

    @staticmethod
    def unsignedAngle(a: Sequence[float], b: Sequence[float]) -> float:
        """
        Calculates the unsigned angle between 2 vectors in radians

        Args:
            a (ArrayLike): The first array
            b (ArrayLike): The second array
        
        Returns:
            The angle in range of [0, pi]
        """
        if len(a) != len(b) or len(a) != 2 or len(b) != 2:
            raise ValueError("Length of arrays must be 2")

        vecA = Vector2(a[0], a[1])
        vecB = Vector2(b[0], b[1])
        return np.arccos(Vector2.dot(vecA, vecB)/(vecA.magnitude*vecB.magnitude))
        
    @staticmethod
    def signedAngle(a: Sequence[float], b: Sequence[float]) -> float:
        """
        Calculates the signed angle between 2 vectors in radians

        Args:
            a (ArrayLike): The first array
            b (ArrayLike): The second array

        Returns:
            Angle in range of [-pi, pi]
        """
        if len(a) != len(b) or len(a) != 2 or len(b) != 2:
            raise ValueError("Length of arrays must be 2")
        
        return np.atan2(
            a[0]*b[1] - a[1]*b[0],
            a[0]*b[0] + a[1]*b[1])
    
    @property
    def x(self) -> float:
        return self._arr[0]
    
    @property
    def y(self) -> float:
        return self._arr[1]
    
    @property
    def magnitude(self) -> float:
        """
        The magnitude of the vector
        """
        return np.sqrt(self._arr @ self._arr)
    
    @property
    def normalized(self) -> Self:
        """
        The normalized vector
        """
        normalized = self._arr/self.magnitude
        return Vector2(normalized[0], normalized[1])
    