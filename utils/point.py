from collections.abc import Sequence

class Point(Sequence[int]):
    def __init__(self, x, y):
        self.x = int(round(x))
        self.y = int(round(y))

    def __getitem__(self, index):
        match index:
            case 0:
                return self.x
            case 1:
                return self.y
            case _:
                raise IndexError("Index out of range of point")

    def __len__(self):
        return 2