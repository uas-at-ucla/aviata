import asyncio
import random
import time
from mavsdk import System

# Test set of manual inputs. Format: [roll, pitch, throttle, yaw]
manual_inputs = [
    [0, 0, 0.5, 0],  # no movement
    [-1, 0, 0.5, 0],  # minimum roll
    [1, 0, 0.5, 0],  # maximum roll
    [0, -1, 0.5, 0],  # minimum pitch
    [0, 1, 0.5, 0],  # maximum pitch
    [0, 0, 0.5, -1],  # minimum yaw
    [0, 0, 0.5, 1],  # maximum yaw
    [0, 0, 1, 0],  # max throttle
    [0, 0, 0, 0],  # minimum throttle
]

async def manual_controls():
    # connect to the drone
    drone = System()
    await drone.connect(system_address="udp://:14540")

    # wait until drone is connected
    async for state in drone.core.connection_state():
        if state.is_connected:
                print(f"Connected to drone with UUID: {state.uuid}")
                break
    

    # Checking if Global Position Estimate is ok
    async for global_lock in drone.telemetry.health():
        if global_lock.is_global_position_ok:
            print("-- Global position state is ok")
            break

    # set the manual control input after arming
    await drone.manual_control.set_manual_control_input(
        float(0), float(0), float(0.5), float(0)
    )

    # Arming the drone
    print("-- Arming")
    await drone.action.arm()

    # Takeoff the vehicle
    print("-- Taking off")
    await drone.action.takeoff()
    await asyncio.sleep(5)

        # set the manual control input after arming
    await drone.manual_control.set_manual_control_input(
        float(0), float(0), float(0.5), float(0)
    )

    # start manual control
    print("-- Starting manual control")
    await drone.manual_control.start_position_control()

    print_position_task = asyncio.ensure_future(print_position(drone))

    while True:

        # I made the index 2 so that it would only have roll
        # this was to test using north/east/down telemetry (position_velocity_ned())
        # if you run it, you can see that the drone moves east only
        # it also prints the position, and you can see that the north/down values don't change, and east always increases
        # this seems to be exactly what we want as inputs to the camera simulator
        input_index = 2 #random.randint(0, len(manual_inputs) - 1) # grabs a random input from the test list
        input_list = manual_inputs[input_index]

        # get current state of roll axis (between -1 and 1)
        roll = float(input_list[0])
        # get current state of pitch axis (between -1 and 1)
        pitch = float(input_list[1])
        # get current state of throttle axis (between -1 and 1, but between 0 and 1 is expected)
        throttle = float(input_list[2])
        # get current state of yaw axis (between -1 and 1)
        yaw = float(input_list[3])

        # thinking ahead: can store roll/pitch/throttle/yaw
        # this loop needs to be streamlined; according to docs, set_manual_control_input() "requires manual control to be sent regularly"
        # update these values in a different thread? should be fine since this thread only reads (no need to worry about volatile since single-threaded)
        # then set_manual_control_input() just continually sets r/p/t/y while we're changing them

        await drone.manual_control.set_manual_control_input(roll, pitch, throttle, yaw)

        await asyncio.sleep(0.1)

async def print_position(drone):
    # here we could send the telemetry to the camera simulator and get back a frame to process
    # (when we actually start using a real drone/camera we can replace this to simply query for the next camera frame)

    # then using the frame we just got, update the roll/pitch/yaw/throttle
    # throttle will probably be a function of how close we are/somehow use this for altitude as well
    # roll/pitch will be for centering
    # yaw for rotation

    # later: think about how to allow for manual override in the real world
    async for position in drone.telemetry.position_velocity_ned():
        print(position.position)

if __name__ == "__main__":

    loop = asyncio.get_event_loop()
    loop.run_until_complete(manual_controls())