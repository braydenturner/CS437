from enum import Enum
from os import system
from matplotlib import pyplot as plt
import picar_4wd as fc
import numpy as np
import time
import sys


np.set_printoptions(threshold=sys.maxsize)


class Point:
    x: int
    y: int

    def __init__(self, x: int, y: int):
        self.x = int(x)
        self.y = int(y)

    def __add__(self, other):
        return Point(self.x + other.x,  self.y + other.y)

    def __str__(self):
        return f"({self.x},{self.y})"


# [y, x]
side_length = 100
world_map = np.zeros((side_length, side_length))
step = 5
current_servo_angle = 0
current_car_angle = 0
curr_position = Point(50, 0)


class Ultrasonic:
    """
    Collection of functions to control and compute data from ultrasonic sensor
    """

    @staticmethod
    def find_objects():
        measurements = Ultrasonic.scan()
        points = [Ultrasonic.compute_point(dist=measurement[0], angle=measurement[1]) for measurement in measurements]

        global world_map
        last_point = None

        # Run through points found by sensor
        for point in points:

            # If point exists, mark as obstacle
            if point is not None:
                Ultrasonic.mark_point(point)

                # If last point was marked, mark points in between
                if last_point is not None:
                    points_in_between = Ultrasonic.interpolate_points(point, last_point)
                    for pnt in points_in_between:
                        Ultrasonic.mark_point(pnt)
            else:
                print("Missed")

            # Set this point as last checked point for interpolation
            last_point = point

    @staticmethod
    def scan() -> [float]:
        """
        Scans in front of the car
        :return: list of distance and angles (cm, degrees)
        """
        global step
        angle_range = 180
        max_angle: int = int(angle_range / 2)
        min_angle = max_angle * -1

        # (distance, angle)
        measurements: [(float, int)] = []

        # Sweep from min to max angle along step
        for angle in range(min_angle, max_angle, step):
            print(f"Taking measurement at {angle}")
            fc.servo.set_angle(angle)
            time.sleep(0.2)
            distance = Ultrasonic.get_distance()
            measurements.append((distance, angle))

        return measurements

    @staticmethod
    def mark_point(point: Point):
        global side_length

        if point.x < side_length and point.y < side_length:
            print(f"Marking point ({point})")
            # Swapped in matrix
            world_map[point.y][point.x] = 1
        else:
            print(f"Point({point}) out of bounds")

    @staticmethod
    def compute_point(dist: float, angle: int) -> Point:
        """
        Computes where the point is in space that is detected given the angle and curr position
        relative_point = (dist * sin(angle), dist * cos (angle))
        absolute_point = relative_point + curr_position
        :return: (x, y) coordinate
        """
        global curr_position, current_car_angle

        # filter out sensor limit readings
        if np.abs(100 - dist) <= 25 or dist < 0:
            print(f"Filtering out {dist}")
            return None
        radians = np.deg2rad(angle)
        relative_point = Point(dist * np.sin(radians), dist * np.cos(radians))
        absolute_point = relative_point + curr_position

        return absolute_point

    @staticmethod
    def get_distance() -> int:
        """
        Gets distance from sensor
        :return: distance in cm
        """
        distance: int = fc.us.get_distance()  # cm
        print(f"Distance: {distance}cm")

        return distance

    @staticmethod
    def interpolate_points(p1: Point, p2: Point) -> [Point]:
        print(f'Interpolating {p1} and {p2}')
        if p2.x - p1.x == 0:
            print(f'Infinite slope')
            return []
        slope = (p2.y - p1.y) / (p2.x - p1.x)
        y_intercept = p1.y - slope * p1.x
        print (f"Slope {slope} and y-intercept {y_intercept}")
        points_to_fill_in = []
        sorted_x = sorted([p1.x, p2.x])

        # find all points between 2 points to fill in
        for x in range(sorted_x[0], sorted_x[1]):
            y = slope * x + y_intercept
            new_pnt = Point(x, y)
            print(f"New Point {new_pnt}")
            points_to_fill_in.append(new_pnt)

        return points_to_fill_in


class Location:
    """
    Maintains location of car in space
    """

    @staticmethod
    def update_location():
        """

        :return:
        """
        global current_car_angle, curr_position

    @staticmethod
    def distance_traveled() -> int:
        """
        speed / time
        :return:
        """

    @staticmethod
    def speed() -> float:
        """
        :return: speed in cm/s
        """
        speed_reading = fc.speed_val()
        print(f"Current speed: {speed_reading} cm/s")
        return speed_reading


class Movement:
    """
    Moves the car in each direction
    """

    class Direction(Enum):

        Left = 0
        Right = 1

        def turn(self):
            """
            Picks random direction and turns 90 degrees
            and turns in that direction
            :return: None
            """

            if self == Movement.Direction.Left:
                Movement.turn_left()
            else:
                Movement.turn_right()

    @staticmethod
    def turn_left(power: int = 50):
        fc.turn_left(power)

    @staticmethod
    def turn_right(power: int = 50):
        fc.turn_right(power)

    # 100 power over 1s is 1cm
    # distance (cm) = time * (power / 100 ) ?
    @staticmethod
    def move_backward(power: int = 15):
        fc.backward(power)

    @staticmethod
    def move_forward(power: int = 50):
        fc.forward(power)


def main():
    while True:
        # Scan 180 FOV, Update map, interpolate points in between
        Ultrasonic.find_objects()

        plt.imshow(world_map, interpolation='nearest')
        plt.savefig("map.png")
        # plt.show()

        time.sleep(5)
        # Move in direction until object is hit

        # Turn


if __name__ == "__main__":
    try:
        main()
    finally:
        fc.stop()
