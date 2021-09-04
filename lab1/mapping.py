from enum import Enum
from point import Point
from matplotlib import pyplot as plt
from webpage import WepPage
from typing import *
from a_star_search import AStar
import picar_4wd as fc
import numpy as np
import time
import sys


np.set_printoptions(threshold=sys.maxsize)

class Orientation(Enum):
    North = 0
    East = 1
    South = 2
    West = 3


# [y, x]
side_length = 100
world_map = np.zeros((side_length, side_length))
step = 10
current_servo_angle = 0
curr_position = Point(50, 0)
curr_orientation = Orientation.North


class Ultrasonic:
    """
    Collection of functions to control and compute data from ultrasonic sensor
    """

    @staticmethod
    def find_objects():
        measurements = Ultrasonic.mapping_scan()
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
    def mapping_scan() -> [float]:
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

        # filter out sensor limit readings
        if np.abs(100 - dist) <= 50 or dist < 0:
            print(f"Filtering out {dist}")
            return None


        global curr_position, curr_orientation

        radians = np.deg2rad(angle)
        # x and y from car's reference frame
        x = dist * np.sin(radians)
        y = dist * np.cos(radians)

        """
        Below are the transformations done to the map reference
        For example, if the car is facing east and sees something 
        directly in front of it at 13cm, this translates to a +13cm
        in the x direction on the board (locked to the world)
        
        """
        if curr_orientation == Orientation.North:
            # (x, y)
            relative_point = Point(x, y)
        elif curr_orientation == Orientation.East:
            # (y, -x)
            relative_point = Point(y, x * -1)
        elif curr_orientation == Orientation.South:
            # (-x, -y)
            relative_point = Point(-1 * x, -1 * y)
        elif curr_orientation == Orientation.West:
            # (-y, x)
            relative_point = Point(-1 * y, x)

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

    @staticmethod
    def avoidance_scan() -> float:
        """
        Scans in front of the car and returns the distance
        :return: distance in cm
        """
        global current_servo_angle, step
        angle_range = 135
        max_angle = angle_range / 2
        min_angle = max_angle * -1

        current_servo_angle += step
        if current_servo_angle > max_angle:
            current_angle = max_angle
            step *= -1
        elif current_servo_angle < min_angle:
            current_angle = min_angle
            step *= -1

        fc.servo.set_angle(current_servo_angle)
        time.sleep(.1)
        distance = fc.us.get_distance()
        return distance

    @staticmethod
    def pad_world_map():
        '''
        Adds extra passing to obstacle found to help with car clearance
        :return:
        '''
        global world_map
        for x in range(5):
            temp_map = np.copy(world_map)
            for row_i, row in enumerate(world_map):
                for col_i, col in enumerate(row):
                    if col == 1:
                        neighbors = AStar.neighbors(temp_map, Point(col_i, row_i))
                        for neighbor in neighbors:
                            temp_map[neighbor.y][neighbor.x] = 1
            world_map = temp_map

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

    class Move:
        class Type(Enum):
            Forward = 0
            Left = 1
            Right = 2
            Backward = 3

        def __init__(self, type_of_move: Type, amount: int):
            self.type = type_of_move
            self.amount = amount

    @staticmethod
    def turn_left(power: int = 50):
        fc.turn_left(power)
        fc.stop()

    @staticmethod
    def turn_right(power: int = 50):
        fc.turn_right(power)
        time.sleep(.55)
        fc.stop()

    # 100 power over 1s is 1cm
    # distance (cm) = time * (power / 100 ) ?
    @staticmethod
    def move_backward(power: int = 10):
        fc.backward(power)

    @staticmethod
    def move_forward(power: int = 10):
        fc.forward(power)

    @staticmethod
    def compute_moves(path: [Point]) -> [Move]:
        global curr_orientation

        last_point = path.pop()
        distance_forward = 0
        while len(path) > 0:
            next_point = path.pop


class Location:
    """
    Maintains location of car in space
    """

    @staticmethod
    def monitor_location(stop_at: int):
        speeds = []
        start_time = time.perf_counter()
        while True:
            speeds.append(Location.speed())
            elapsed_time = time.perf_counter() - start_time
            distance = Location.distance_traveled(elapsed_time, speeds)
            if abs(distance - stop_at) < 1:
                break
        fc.stop()
        fc.left_rear_speed.deinit()
        fc.right_rear_speed.deinit()

        Location.update_location(distance)

    @staticmethod
    def update_location(new_location: float):
        """

        :return:
        """

        global curr_orientation, curr_position
        if curr_orientation == Orientation.North:
            # (x, y)
            relative_point = Point(0, new_location)
        elif curr_orientation == Orientation.East:
            # (y, -x)
            relative_point = Point(new_location, 0)
        elif curr_orientation == Orientation.South:
            # (-x, -y)
            relative_point = Point(0, -1 * new_location)
        elif curr_orientation == Orientation.West:
            # (-y, x)
            relative_point = Point(new_location, 0)

        curr_position += relative_point

        print(f"New position {curr_position}")

    @staticmethod
    def update_orientation(turn_direction: Movement.Direction):
        """

        :return:
        """

        global curr_orientation

        if turn_direction == Movement.Direction.Left:
            curr_orientation = (curr_orientation - 1) % 4
        else:
            curr_orientation = (curr_orientation + 1) % 4

    @staticmethod
    def distance_traveled(time_elapsed, speed_intervals) -> int:
        """
        speed / time
        :return: distance in cm
        """
        if len(speed_intervals) == 0:
            return 0
        mean_speed = np.mean(speed_intervals)
        distance =  mean_speed * time_elapsed
        print(f"Distance traveled {distance}cm")

        return distance

    @staticmethod
    def speed() -> float:
        """
        :return: speed in cm/s
        """
        speed_reading = fc.speed_val()
        print(f"Current speed: {speed_reading} cm/s")
        return speed_reading


def main():
    # Process(target=WepPage.run).start()
    x = 0
    fc.start_speed_thread()
    while x < 4:
        # Scan 180 FOV, Update map, interpolate points in between
        Ultrasonic.find_objects()
        plt.imshow(world_map, interpolation='nearest')
        plt.savefig("/home/pi/Desktop/map_no_padding.png")

        Ultrasonic.pad_world_map()
        plt.imshow(world_map, interpolation='nearest')
        plt.savefig("/home/pi/Desktop/map_padding.png")

        new_maze = np.full(np.shape(world_map), -1)

        end = Point(50, side_length - 1)
        came_from, cost_so_far = AStar.search(world_map, curr_position, end)

        for point, cost in cost_so_far.items():
            new_maze[point.y][point.x] = cost

        last_elm = end
        path_forward = [last_elm]
        while last_elm is not None:
            last_elm = came_from[last_elm]
            path_forward.append(last_elm)

        cmap = plt.cm.gray
        norm = plt.Normalize(world_map.min(), world_map.max())
        rgba = cmap(norm(world_map))

        path_forward.reverse()
        for point in path_forward:
            if point is not None:
                rgba[point.y][point.x] = 1, 0, 0, 1
        plt.imshow(rgba, interpolation='nearest')
        plt.savefig("/home/pi/Desktop/map_search.png")

        # Move in direction until object is hit, measuring distance
        Movement.move_forward()
        Location.monitor_location(stop_at=end.y)
        Movement.turn_right()

        x+=1

        # Turn



if __name__ == "__main__":
    try:
        main()
    finally:
        fc.stop()
