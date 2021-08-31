from enum import Enum
from os import system
import picar_4wd as fc
import numpy as np
import time

# [y, x]
side_length = 100
world_map = np.zeros((side_length, side_length))
step = 18
current_servo_angle = 0
current_car_angle = 0
curr_position = [50, 0]


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
            fc.servo.set_angle(angle)
            distance = Ultrasonic.get_distance()
            measurements.append((distance, angle))

        return measurements

    @staticmethod
    def mark_point(point: [int, int]):
        global side_length
        x, y = point[0], point[1]

        if x < side_length and y < side_length:
            print(f"Marking point({x},{y})")
            # Swapped in matrix
            world_map[y][x] = 1
        else:
            print(f"Point({x},{y}) out of bounds")


    @staticmethod
    def compute_point(dist: float, angle: int) -> [[int, int]]:
        """
        Computes where the point is in space that is detected given the angle and curr position
        relative_point = (dist * sin(angle), dist * cos (angle))
        absolute_point = relative_point + curr_position
        :return: (x, y) coordinate
        """
        global curr_position, current_car_angle

        # filter out sensor limit readings
        if np.abs(100 - dist) <= 10 :
            return None
        radians = np.deg2rad(angle)
        relative_point = [dist * np.sin(radians), dist * np.cos(radians)]
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
    def interpolate_points(p1: (int, int), p2: (int, int)) -> [(int, int)]:
        x_coord, y_coord = zip(p1, p2)
        coefficients = np.polyfit(x_coord, y_coord, 1)
        slope, y_intercept = coefficients[0], coefficients[1]

        points_to_fill_in = []

        # find all points between 2 points to fill in
        for x in range(p1[0], p2[0]):
            y = int(slope * x + y_intercept)
            points_to_fill_in.append((x, y))

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

        system('clear')
        print(world_map)
        time.sleep(5)
        # Move in direction until object is hit

        # Turn

        return


if __name__ == "__main__":
    try:
        main()
    finally:
        fc.stop()
