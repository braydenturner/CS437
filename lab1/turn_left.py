import picar_4wd as fc
import time


if __name__ == "__main__":
    fc.turn_left(100)
    time.sleep(1)
    fc.stop()
    time.sleep(3)
    fc.turn_left(50)
    time.sleep(1)
    fc.stop()
    time.sleep(3)
    fc.turn_left(25)
    time.sleep(1)
    fc.stop()
    time.sleep(3)


    time.sleep(5)
    fc.forward(100)
    time.sleep(1)
    fc.stop()
    time.sleep(3)
    fc.forward(50)
    fc.stop()
    time.sleep(3)
    time.sleep(1)
    fc.forward(25)
    fc.stop()
    time.sleep(3)
    time.sleep(1)
