import picar_4wd as fc
import random
from enum import Enum
import time

step = 20
current_angle = 0

# Init motors
left_front = fc.Motor(fc.PWM("P13"), fc.Pin("D4"), is_reversed=False) # motor 1
right_front = fc.Motor(fc.PWM("P12"), fc.Pin("D5"), is_reversed=False) # motor 2
left_rear = fc.Motor(fc.PWM("P8"), fc.Pin("D11"), is_reversed=False) # motor 3
right_rear = fc.Motor(fc.PWM("P9"), fc.Pin("D15"), is_reversed=False) # motor 4

class Direction(Enum):
    Left = 0
    Right = 1

    def turn(self):
        """
        Picks a random time in seconds between [.5, 2]
        and turns in that direction
        :return: None
        """
        seconds: float = random.uniform(.25, 1)
        if self == Direction.Left:
            turn_left()
        else:
            turn_right()
        time.sleep(seconds)


def get_distance() -> int:
    """
    Gets distance in front of car
    :return: distance in cm
    """
    distance: int = fc.us.get_distance() #cm
    print(f"Distance: {distance}cm")

    return distance


def scan() -> float:
    """
    Scans in front of the car and returns the distance
    :return: distance in cm
    """
    global current_angle, step
    angle_range = 135
    max_angle = angle_range / 2
    min_angle = max_angle * -1

    current_angle += step
    if current_angle > max_angle:
        current_angle = max_angle
        step *= -1
    elif current_angle < min_angle:
        current_angle = min_angle
        step *= -1

    fc.servo.set_angle(current_angle)
    time.sleep(.1)
    distance = get_distance()
    return distance


def turn_random_direction():
    """
    Picks a random direction and turns car that way
    :return: None
    """
    direction: Direction = random.choice(list(Direction))
    print(f"Turning {direction}")
    direction.turn()


def move_forward(power):
    print("Driving")
    left_front.set_power(power)
    left_rear.set_power(power)
    right_front.set_power(power)
    right_rear.set_power(power)

def move_backward(power):
    print("Backing up")
    left_front.set_power(-power)
    left_rear.set_power(-power)
    right_front.set_power(-power)
    right_rear.set_power(-power)

def turn_left(power):
    left_front.set_power(-power)
    left_rear.set_power(-power)
    right_front.set_power(power)
    right_rear.set_power(power)

def turn_right(power):
    left_front.set_power(power)
    left_rear.set_power(power)
    right_front.set_power(-power)
    right_rear.set_power(-power)


def main():
    fc.servo.set_angle(current_angle)
    move_forward()
    while True:
        distance = get_distance()
        if distance < 10:
            fc.stop()
            move_backward()
            time.sleep(.75)
            fc.stop()
            turn_random_direction()
            fc.stop()
            move_forward()


if __name__ == "__main__":
    try:
        main()
    finally:
        fc.stop()
