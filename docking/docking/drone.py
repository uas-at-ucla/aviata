# standard python library imports
import asyncio

# mavsdk
from mavsdk import System
from mavsdk.offboard import (OffboardError, VelocityNedYaw)

# aviata modules
from camera_simulator import CameraSimulator
from image_analyzer import ImageAnalyzer

class Drone:

    def __init__(self):
        self.camera_simulator = CameraSimulator(1, 2, 0, 90) # later, use camera here
        self.image_analyzer = ImageAnalyzer()
        self.north = 0
        self.east = 0
        self.down = -5
        self.yaw = 0

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
                print(f"Connected to drone with UUID: {state.uuid}")
                break

        # Basic setup checks
        async for global_lock in self.drone.telemetry.health():
            if global_lock.is_global_position_ok:
                print("-- Global position state is ok")
                break

    async def takeoff(self):
        """
        Takeoff and start offboard mode.
        Return true if success, false otherwise
        """
        print("-- Arming")
        await self.drone.action.arm()

        # Takeoff the vehicle (later, we can assume that the drone is already flying when we run this script)
        print("-- Taking off")
        await self.drone.action.set_takeoff_altitude(self.down * -1.0) # meters
        await self.drone.action.takeoff()
        await asyncio.sleep(8)

        # start offboard mode (requires setting initial setpoint)
        print("-- Setting initial setpoint")
        await self.drone.offboard.set_velocity_ned(
            VelocityNedYaw(0.0, 0.0, 0.0, 0.0))

        print("-- Starting offboard")
        try:
            await self.drone.offboard.start()
        except OffboardError as error:
            print(f"Starting offboard mode failed with error code: \
                {error._result.result}")
            print("-- Disarming")
            await self.drone.action.disarm()
            return False

        return True

    async def initiate_docking(self):

        # Get telemetry periodically from the drone
        telemetry_task = asyncio.create_task(self.get_telemetry_position())
        rotation_task = asyncio.create_task(self.get_telemetry_rotation())

        await self.stage1()
        await self.stage2()
        await self.stage3()

        telemetry_task.cancel()
        rotation_task.cancel()

    async def stage1(self):
        """
        Position the drone above the large central target
        (unfinished)
        """
        print("Docking stage 1")

        while True:
            img = self.camera_simulator.updateCurrentImage(self.east, self.north, self.down * -1.0, self.yaw)
            
            errs = self.image_analyzer.process_image(img)
            if errs is None:
                continue
            x_err, y_err, alt_err, rot_err = errs

            north_velocity = y_err * 4 / 5 # no I or D yet
            east_velocity = x_err * 4 / 5 # no I or D yet
            down_velocity = alt_err * .2 / 5
            rot_angle = self.yaw + rot_err # yaw is in degrees, not degrees per second. must be set absolutely

            await self.drone.offboard.set_velocity_ned(
                VelocityNedYaw(north_velocity, east_velocity, down_velocity, rot_angle)) # north, east, down (all m/s), yaw (degrees, north is 0, positive for clockwise)

            await asyncio.sleep(.1) # update drone at 10Hz

    async def stage2(self):
        """Fly from the central target to the peripheral target"""
        print("Docking stage 2")
    
    async def stage3(self):
        """Position the drone above the peripheral target and descend"""
        print("Docking stage 3")
    
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

    async def land(self):
        """Safely stop the drone and stop offboard mode"""
        print("-- Stopping offboard")
        try:
            await drone.offboard.stop()
        except OffboardError as error:
            print(f"Stopping offboard mode failed with error code: \
                {error._result.result}")

        print("-- Landing drone")
        await drone.action.land()
