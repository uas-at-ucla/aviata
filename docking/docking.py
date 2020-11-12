import asyncio
import argparse
import cv2
from mavsdk import System
from mavsdk.offboard import (OffboardError, VelocityNedYaw)

# Camera simulator-related imports
import math
import pygame
#import constants
from CameraSimulator import CameraSimulator
from PIL import Image

# Other scripts
from image_analyzer import getErrors

north = 0
east = 0
down = -5

async def dock():
    global north
    global east
    global down

    # temporary arguments to get the static image we want to parse
    parser = argparse.ArgumentParser(
        description='get the image that the drone should follow')

    parser.add_argument('-i', '--image', metavar='IMAGE', type=str, required=False,
                        help='path to the image to use')

    options = parser.parse_args()

    image_filename = options.image
    if image_filename == None:
        print("Using default image 'sample_apriltags/apriltag-se.jpg'")
        image_filename = 'sample_apriltags/apriltag-se.jpg'
    else: 
        print(f"Using specified image {image_filename}")

    # Start the camera simulator
    camera_simulator = CameraSimulator() # position of target

    # connect to the drone
    drone = System()
    await drone.connect(system_address="udp://:14540")

    # wait until drone is connected and do basic setup checks
    async for state in drone.core.connection_state():
        if state.is_connected:
            print(f"Connected to drone with UUID: {state.uuid}")
            break

    async for global_lock in drone.telemetry.health():
        if global_lock.is_global_position_ok:
            print("-- Global position state is ok")
            break

    print("-- Arming")
    await drone.action.arm()

    # Takeoff the vehicle (later, we can assume that the drone is already flying when we run this script)
    print("-- Taking off")
    await drone.action.set_takeoff_altitude(5.0) # meters
    await drone.action.takeoff()
    await asyncio.sleep(8)

    # start offboard mode (requires setting initial setpoint)
    print("-- Setting initial setpoint")
    await drone.offboard.set_velocity_ned(
        VelocityNedYaw(0.0, 0.0, 0.0, 0.0))

    print("-- Starting offboard")
    try:
        await drone.offboard.start()
    except OffboardError as error:
        print(f"Starting offboard mode failed with error code: \
              {error._result.result}")
        print("-- Disarming")
        await drone.action.disarm()
        return

    await drone.offboard.set_velocity_ned(
        VelocityNedYaw(0.0, 0.5, 0.0, 60.0))
    print("Flying away for a bit")
    await asyncio.sleep(2)

    # Get telemetry periodically from the drone
    telemetry_task = asyncio.ensure_future(get_telemetry(drone))

    while True:
        # 1. get image from camera
        # img = cv2.imread(image_filename) # eventually, send telemetry to camera simulator and get back a frame to process
        img = camera_simulator.updateCurrentImage(east, north, down * -1.0, 0)
        # print(north, east)
        # 2. process image to get errors for center/height/rotation
        errs = getErrors(img)
        if errs is None:
            continue

        x_err, y_err, alt_err, rot_err = errs
        # 3. move according to errors
        north_velocity = y_err * 4 # no I or D yet
        east_velocity = x_err * 4 # no I or D yet
        down_velocity = alt_err * .2
        print(f"Setting velocities to: {north_velocity} north, {east_velocity} east, {down_velocity} down")

        await drone.offboard.set_velocity_ned(
            VelocityNedYaw(north_velocity, east_velocity, down_velocity, -1.0 * rot_err)) # north, east, down (all m/s), yaw (degrees, north is 0, positive for clockwise)

        # if y_err < .10 and x_err < .10:
        #     print("Close enough to target, landing")
        #     break

        await asyncio.sleep(.1) # update drone at 10Hz
    
    # Safely stop the drone
    print("-- Stopping offboard")
    try:
        await drone.offboard.stop()
    except OffboardError as error:
        print(f"Stopping offboard mode failed with error code: \
              {error._result.result}")

    print("-- Landing drone")
    await drone.action.land()


async def get_telemetry(drone):
    global north
    global east
    global down

    # set update rate
    await drone.telemetry.set_rate_position_velocity_ned(20) # hertz

    # get telemetry updates at the rate specified above
    async for position in drone.telemetry.position_velocity_ned():
        # print(position.position)
        p = position.position
        north = p.north_m
        east = p.east_m
        down = p.down_m
        print(east, " ", north, " ", down, " ", 0)

if __name__ == "__main__":
    loop = asyncio.get_event_loop()
    loop.run_until_complete(dock())