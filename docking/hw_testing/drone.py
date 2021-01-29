# standard python library imports
import asyncio
import time
import math
import numpy as np

# mavsdk
from mavsdk import System
from mavsdk.offboard import (OffboardError, VelocityBodyYawspeed, VelocityNedYaw)

# aviata modules
import util
from image_analyzer import ImageAnalyzer
from pid_controller import PIDController
from log import Log

boom_length = 1 # meter
fov = 48.8
altitude_disp = boom_length / 2 / math.tan(math.radians(fov / 2)) * 2 # add 100% tolerance

class Drone:

    def __init__(self, target):
        # self.camera_simulator = CameraSimulator(target) # later, use camera here
        self.image_analyzer = ImageAnalyzer()
        self.north = 0
        self.east = 0
        self.down = 0
        self.yaw = 0
        self.takeoff_altitude = 2 # initial altitude
        self.target = target 
        self.dt = 0.05 # delta t update frequency, 20hz
        self.MAX_ATTEMPTS = 3 # number of unsuccessful attempts before aborting docking
        self.MAX_HEIGHT = 10 # max height above central target before failure
        self.MAX_HEIGHT_STAGE_2 = 2
        self.STAGE_1_TOLERANCE = 0.1 
        self.STAGE_2_TOLERANCE = 0.05
        self.log=Log()
        self.logging=True

        self.in_air = False

    async def connect_gazebo(self):
        """
        Connect to gazebo and perform basic setup checks.
        Does not return until connection to drone is verified.
        """
        # Wait for connection
        self.drone = System()
        await self.drone.connect(system_address="udp://:14540")

        async for state in self.drone.core.connection_state():
            if state.is_connected:
                self.log.write_literal(f"Connected to drone with UUID: {state.uuid}")
                break

        # Basic setup checks
        async for global_lock in self.drone.telemetry.health():
            if global_lock.is_global_position_ok:
                self.log.write_literal("Global position state is ok")
                break

    async def takeoff(self, test_number):
        """
        Takeoff and start offboard mode, then initiate docking.
        Return true if success, false otherwise
        """
        self.log.set_tagline("Takeoff")
        await self.drone.action.arm()

        # Get telemetry periodically from the drone
        telemetry_task = asyncio.create_task(self.get_telemetry_position())
        rotation_task = asyncio.create_task(self.get_telemetry_rotation())
        in_air_task = asyncio.create_task(self.get_telemetry_in_air())

        # Takeoff the vehicle (later, we can assume that the drone is already flying when we run this script)
        self.log.write_literal("-- Taking off")
        await self.drone.action.set_takeoff_altitude(2.0) # meters
        await self.drone.action.takeoff()
        
        while not self.in_air:
            await asyncio.sleep(0.1)
        
        await asyncio.sleep(8)

        # start offboard mode (requires setting initial setpoint)
        self.log.write_literal("Setting initial setpoint")
        await self.drone.offboard.set_velocity_body(VelocityBodyYawspeed(0.0, 0.0, 0.0, 0))

        self.log.write_literal("Starting offboard")
        try:
            await self.drone.offboard.start()
        except OffboardError as error:
            self.log.write_literal(f"Starting offboard mode failed with error code: {error._result.result}")
            self.log.write_literal("Disarming")
            await self.drone.action.disarm()
            return False

        self.log.write_literal(f"Takeoff complete, ascended to: {abs(self.down)}")

        # do tests here
        if test_number == 1:
            self.test1()
        elif test_number == 2:
            self.test2()

        await asyncio.sleep(3)
        await self.land()

        telemetry_task.cancel()
        rotation_task.cancel()

        return True

    def test1(self):
        # don't forget to set async if necessary
        self.log.set_tagline("Test 1")
        self.log.write_literal("We are in test 1")

    def test2(self):
        # don't forget to set async if necessary
        self.log.set_tagline("Test 2")
        self.log.write_literal("We are in test 2")

    async def get_telemetry_rotation(self):
        """Poll to obtain the drone's yaw"""
        await self.drone.telemetry.set_rate_attitude(20) # update at 20Hz 
        async for angles in self.drone.telemetry.attitude_euler():
            self.yaw = angles.yaw_deg # 0 degrees is north, positive cw, negative ccw, between -180 and +180


    async def get_telemetry_position(self):
        """Poll to obtain the drone's x, y, and z position"""
        await self.drone.telemetry.set_rate_position_velocity_ned(20) # update at 20Hz 
        async for position in self.drone.telemetry.position_velocity_ned():
            p = position.position
            self.north = p.north_m
            self.east = p.east_m
            self.down = p.down_m

    async def get_telemetry_in_air(self):
        async for is_in_air in self.drone.telemetry.in_air():
            self.in_air = is_in_air

    async def land(self):
        """Safely stop the drone and stop offboard mode"""
        self.log.set_tagline("Landing")
        self.log.write_literal("Stopping offboard")
        try:
            await self.drone.offboard.stop()
        except OffboardError as error:
            self.log.write_literal(f"Stopping offboard mode failed with error code: \
                {error._result.result}")

        self.log.write_literal("-- Landing drone")
        await self.drone.action.land()
