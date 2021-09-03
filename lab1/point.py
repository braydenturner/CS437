class Point:
    """
    Used to denote x, y coordinated
    """
    x: int
    y: int

    def __init__(self, x: float, y: float):
        self.x = int(x)
        self.y = int(y)

    def __add__(self, other):
        return Point(self.x + other.x,  self.y + other.y)

    def __str__(self):
        return f"({self.x},{self.y})"
