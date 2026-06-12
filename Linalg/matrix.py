import numpy as np
from numpy.typing import ArrayLike
from typing import Self
from .vector import Vector2

class Matrix22(ArrayLike):
    def __init__(self, matrix: ArrayLike):
        np_mat = np.matrix(matrix)
        if np_mat.shape != (2, 2):
            return ValueError("Matrix must have shape 2x2")
        self._data = np_mat

    def __init__(self, a11: float, a12: float, a21: float, a22: float):
        self._data = np.matrix([[a11, a12], [a21, a22]])

    def __add__(self, other: ArrayLike) -> Self:
        np_mat = np.matrix(other)
        if np_mat.shape != (2, 2):
            return ValueError("Matrix must have shape 2x2")

        return Matrix22(self._data + np_mat)
    
    def __sub__(self, other: ArrayLike) -> Self:
        np_mat = np.matrix(other)
        if np_mat.shape != (2, 2):
            return ValueError("Matrix must have shape 2x2")

        return Matrix22(self._data - np_mat)
    
    def __mul__(self, other: ArrayLike) -> Self | Vector2:
        np_mat = np.matrix(other)

        if np_mat.shape == (2, 2):
            return Matrix22(self._data @ np_mat)
        elif np_mat.shape == (2,):
            return Vector2(self._data @ np_mat)
        else:
            return ValueError("Matrix must have shape 2x2 or 2x1")
    
    def __neg__(self) -> Self:
        return Matrix22(-self._data)
    
    def __pow__(self, a: float) -> Self:
        return Matrix22(self._data**a)
    
    def __iter__(self):
        iter(self._data)
        return self
    
    def __next__(self):
        return next(self._data)
    
    def __getitem__(self, row: int) -> float:
        return self._data[row]

    @property
    def a11(self) -> float:
        return self._data[0,0]
    
    @property
    def a12(self) -> float:
        return self._data[0,1]
    
    @property
    def a21(self) -> float:
        return self._data[1,0]

    @property
    def a22(self) -> float:
        return self._data[1,1]
