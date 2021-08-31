from enum import Enum

import numpy

import picar_4wd as fc
import typing as T
import types
import random
import numpy as np
import time


world_map = np.zeros(100 * np.zeros(100))
step = 18
current_servo_angle = 0
current_car_angle = 0
curr_position = (0,0)


'''
Collection of functions to control and compute data from ultrasonic sensor
'''
class Ultrasonic:

    @staticmethod
    def scan() -> float:
        """
        Scans in front of the car and returns the distance
        :return: distance in cm
        """
        global current_angle, step
        angle_range = 180
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
        distance = Ultrasonic.get_distance()
        return distance


    @staticmethod
    def compute_point(dist: float):
        """
        Computes where the point is in space that is detected given the angle and curr position
        relative_point = (dist * sin(angle), dist * cos (angle))
        absolute_point = relative_point + curr_position
        :return: None
        """
        global curr_position, current_servo_angle, current_car_angle

        relative_point = (dist * np.sin(current_servo_angle), dist * np.cos(current_servo_angle))
        absolute_point = relative_point + curr_position


    @staticmethod
    def get_distance() -> int:
        """
        Gets distance from sensor
        :return: distance in cm
        """
        distance: int = fc.us.get_distance() #cm
        print(f"Distance: {distance}cm")

        return distance


'''
Maintains location of car in space
'''
class Location:

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


'''
Moves the car in each direction
'''
class Movement:
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


    def turn_left(power: int = 50):
        fc.turn_left(power)


    def turn_right(power: int = 50):
        fc.turn_right(power)


    # 100 power over 1s is 1cm
    # distance (cm) = time * (power / 100 ) ?
    def move_backward(power: int = 15):
        fc.backward(power)


    def move_forward(power: int = 50):
        fc.forward(power)



def main():
    fc.start_speed_thread()
    while True:
        # Scan 180 FOV
        # Update map, interpolating points in between
        #
        #
        return


if __name__ == "__main__":
    try:
        main()
    finally:
        fc.stop()
