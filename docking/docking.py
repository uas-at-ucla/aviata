import asyncio
import argparse
import cv2
from mavsdk import System
from mavsdk.offboard import (OffboardError, VelocityNedYaw)
from image_analyzer import getErrors

async def dock():

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
        VelocityNedYaw(0.0, 0.0, -1.0, 60.0))

    print_position_task = asyncio.ensure_future(print_position(drone))

    for i in range(0, 5):
        # 1. get image from camera
        img = cv2.imread(image_filename) # eventually, send telemetry to camera simulator and get back a frame to process
        # 2. process image to get errors for center/height/rotation
        x_err, y_err = getErrors(img)
        # 3. move according to errors
        north_velocity = y_err * 2 # no I or D yet
        east_velocity = x_err * 2 # no I or D yet
        print(f"Setting velocities to: {north_velocity} north, {east_velocity} east")

        await drone.offboard.set_velocity_ned(
            VelocityNedYaw(north_velocity, east_velocity, 0.0, 0.0)) # north, east, down (all m/s), yaw (degrees, north is 0, positive for clockwise)

        await asyncio.sleep(1)
    
    # Safely stop the drone
    print("-- Stopping offboard")
    try:
        await drone.offboard.stop()
    except OffboardError as error:
        print(f"Stopping offboard mode failed with error code: \
              {error._result.result}")

    print("-- Landing drone")
    await drone.action.land()

async def print_position(drone):
    i = 0
    async for position in drone.telemetry.position_velocity_ned():
        i += 1
        if i > 20: # throttle print rate a little bit
            print(position.position)
            i = 0

if __name__ == "__main__":
    loop = asyncio.get_event_loop()
    loop.run_until_complete(dock())