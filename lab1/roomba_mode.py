import picar_4wd as fc
import random
from enum import Enum
import time

class Direction(Enum):
    Left = 0
    Right = 1

    def turn(self):
        """
        Picks a random time in seconds between [0, 2]
        and turns in that direction
        :return: None
        """
        seconds: float = random.random() * 2
        if self == Direction.Left:
            turn_left(seconds)
        else:
            turn_right(seconds)
        time.sleep(seconds)


def get_distance() -> int:
    """
    Gets distance in front of car
    :return: distance in cm
    """
    distance: int = fc.us.get_distance() #cm
    print(f"Distance: {distance}cm")

    return distance

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


def turn_left(power: int = 50):
    """Move backward"""
    fc.turn_left(power)


def turn_right(power: int = 50):
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
        distance = get_distance()
        if distance < 10:
            move_backward()
            time.sleep(1)
            turn_random_direction()


if __name__ == "__main__":
    try:
        main()
    finally:
        fc.stop()
