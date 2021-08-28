import picar_4wd as fc
import random
from enum import Enum
import time

class Direction(Enum):
    Left = 0
    Right = 1

    def turn(self):
        if self == Direction.Left:
            turn_left(2)
        else:
            turn_right(2)


def get_distance() -> int:
    """
    Gets distance in front of car
    :return: distance in cm
    """
    distance: int = fc.us.get_distance() #cm
    print(f"Distance: {distance}cm")


def scan(step: int = 30) -> list:
    """
    Scans in front of the car and returns a list
    :param step: the angle (degree) change of each scan
    :return:
    """

    return []


def turn_random_direction():
    """
    Picks a random direction and turns car that way
    :return: None
    """
    direction: Direction = random.choice(list(Direction))
    print(f"Turning {direction}")
    direction.turn()

def turn_left(seconds: int, power: int = 50):
    """Move backward"""
    fc.forward(power)


def turn_right(seconds: int, power: int = 50):
    """Move backward"""
    fc.forward(power)


def move_backward(power: int = 25):
    """Move backward"""
    fc.backward(power)


def move_forward(power: int = 50):
    """Move backward"""
    fc.forward(power)


def main():
    while True:
        move_forward()
        distanceCM = get_distance()
        if distanceCM < 10:
            move_backward()
            time.sleep(1)
            turn_random_direction()


if __name__ == "__main__":
    try:
        main()
    finally:
        fc.stop()
