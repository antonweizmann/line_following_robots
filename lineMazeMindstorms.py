from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor, ColorSensor
from pybricks.parameters import Port, Direction, Color
from pybricks.tools import wait

hub = PrimeHub()

left_motor = Motor(Port.C, Direction.COUNTERCLOCKWISE)

right_motor = Motor(Port.D)

color_sensor = ColorSensor(Port.E)


# Kp = 5
# Ki = 0.0
# Kd = 0.8

# Kp = 7
# Ki = 0.003
# Kd = 5.4

Kp = 6
Ki = 0.002
Kd = 5
integral = 0
last_error = 0

def follow_line():
    """A complete PID line follower for SPIKE Prime."""
    global integral, last_error

    reflection = color_sensor.reflection()

    target = 50

    error = reflection -target

    # base_speed = 200
    base_speed = 300
    P = error * Kp

    integral += error
    if integral > 50:
        integral = 50
    elif integral < -50:
        integral = -50
    I = integral * Ki

    derivative = error - last_error
    D = derivative * Kd

    last_error = error

    turn_rate = P + I + D

    left_speed = base_speed - turn_rate
    right_speed = base_speed + turn_rate

    left_motor.run(left_speed)
    right_motor.run(right_speed)

while True:
    follow_line()
    wait(10)
