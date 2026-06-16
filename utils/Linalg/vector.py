import numpy as np
from collections.abc import Sequence
from typing import Self


class Vector2(Sequence[float]):
    def __init__(self, arr: Sequence[float]):
        if(len(arr) != 2):
            raise ValueError("Length of array must be 2")
        
        self._arr = np.array([arr[0], arr[1]])
    
    def __init__(self, x: float, y: float):
        self._arr = np.array([x, y])

    def __add__(self, other: Sequence[float]) -> Self:
        if len(other) != 2:
            raise ValueError("Length of array must be 2")
        
        return Vector2(np.add(self, other))
    
    def __sub__(self, other: Sequence[float]) -> Self:
        if len(other) != 2:
            raise ValueError("Length of array must be 2")

        return Vector2(np.subtract(self, other))
    
    def __mul__(self, scalar: float) -> Self:
        return Vector2(np.multiply(self._arr, scalar))
        
    def __neg__(self):
        return Vector2(0 - self._arr)

    def __iter__(self):
        iter(self._arr)
        return self

    def __next__(self):
        return next(self._arr)

    def __len__(self):
        return 2
    
    def __getitem__(self, index: int) -> float:
        return self._arr[index]

    def rotate(self, angle: float) -> Self:
        """
        Calculates the rotated vector of self when being rotated by angle

        Args:
            angle (float): The angle to rotate by in radian

        Returns:
            A new rotated vector
        """
        return np.matrix([[np.cos(angle), np.sin(angle)],
                   [-np.sin(angle), np.cos(angle)]]) @ self._arr

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
        
        return a @ b

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

        dotProd = Vector2.dot(a, b)/(Vector2.dot(b, b))
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

        vecA = Vector2(a)
        vecB = Vector2(b)
        np.arccos(Vector2.dot(vecA, vecB)/(vecA.magnitude*vecB.magnitude))
        
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
        Vector2.signedAngle()
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
        return Vector2(self._arr/self.magnitude)
    