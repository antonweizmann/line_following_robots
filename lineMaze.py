from robots import *
from coppeliasim_zmqremoteapi_client import RemoteAPIClient

client = RemoteAPIClient()
sim = client.require("sim")

# HANDLES FOR ACTUATORS AND SENSORS
left_motor = Motor(sim, DeviceNames.MOTOR_LEFT_LINE, Direction.CLOCKWISE)
right_motor = Motor(sim, DeviceNames.MOTOR_RIGHT_LINE, Direction.CLOCKWISE)
color_sensor = ImageSensor(sim, DeviceNames.IMAGE_SENSOR_LINE)


Kp = 0.12
Ki = 0.01
Kd = 0.05

integral = 0
last_error = 0

def is_red_detected(color_sensor):
    """
    Calculates the relative intensity of the red channel compared to
    other channels
    """
    red_ratio_threshold = 1.5
    red, green, blue = color_sensor.rgb()
    print(red, green, blue)
    red_intensity = red / (green + blue)

    return red_intensity > red_ratio_threshold


def is_blue_detected(color_sensor):
    """
       Calculates the relative intensity of the blue channel compared to
       other channels
       """
    blue_ratio_threshold = 1.5
    red, green, blue = color_sensor.rgb()
    blue_intensity = blue / (red + green)

    return blue_intensity > blue_ratio_threshold

def follow_line():
    """
    A very simple line follower that should be improved.
    """
    global integral, last_error
    color_sensor._update_image() # Updates the internal image

    reflection = color_sensor.reflection() # Gets the reflection from the image
    print(reflection)
    target = 50

    error = reflection - target
    base_speed = 7
    P = error * Kp

    integral += error
    if integral > 500:
        integral = 500
    elif integral < -500:
        integral = -500
    I = integral * Ki

    derivative = error - last_error
    D = derivative * Kd

    last_error = error
    turn_rate = P + I + D

    left_speed = base_speed - turn_rate
    right_speed = base_speed + turn_rate
    left_motor.run(speed=left_speed)
    right_motor.run(speed=right_speed)

# Starts coppeliasim simulation if not done already
sim.startSimulation()

# MAIN CONTROL LOOP
while True:
	follow_line()
