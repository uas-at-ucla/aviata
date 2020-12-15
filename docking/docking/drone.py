# standard python library imports
import asyncio

# mavsdk
from mavsdk import System
from mavsdk.offboard import (OffboardError, VelocityNedYaw)

# aviata modules
from camera_simulator import CameraSimulator
from image_analyzer import ImageAnalyzer
from debug_window import DebugWindow

class Drone:

    def __init__(self, target_number):
        self.camera_simulator = CameraSimulator(0, 0, 0, 0) # later, use camera here
        self.image_analyzer = ImageAnalyzer()
        self.north = 0
        self.east = 0
        self.down = -5
        self.yaw = 0
        self.target_number = target_number

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

        telemetry_task.cancel()
        rotation_task.cancel()

    async def stage1(self):
        """
        Position the drone above the large central target
        (unfinished)
        """
        print("Docking stage 1")

        debug_window=DebugWindow(1,1,2,0,90)

        frames_elapsed = 0
        total_time_elapsed = 0
        prev_errs = (0, 0, 0)
        sum_errs = (0, 0, 0)
        dt = 0.1 # delta t
        while True:
            img = self.camera_simulator.updateCurrentImage(self.east, self.north, self.down * -1.0, self.yaw)
            
            errs = self.image_analyzer.process_image(img, 0)
            if errs is None:
                frames_elapsed = frames_elapsed + 1

                if frames_elapsed > 10: # haven't detected target in 1 second
                    await self.drone.offboard.set_velocity_ned(
                        VelocityNedYaw(0, 0, -0.2, self.yaw)) # north, east, down (all m/s), yaw (degrees, north is 0, positive for clockwise)

                await asyncio.sleep(.1)
                continue

            debug_window.updateWindow(self.east,self.north,self.down*-1.0,self.yaw, 0) # 0 tags detected for now, change later
            frames_elapsed = 0
            x_err, y_err, alt_err, rot_err = errs # errors all in meters
            alt_err = alt_err - 4 # so it's centered around 4 instead of 0

            ku_nv = 2.0
            ku_ev = 2.15
            ku_dv = 3

            tu_nv = 0.8
            tu_ev = 0.8
            tu_dv = 0.8

            kp_nv = 0.6 * ku_nv
            kp_ev = 0.6 * ku_ev
            kp_dv = 0.2 * ku_dv

            kd_nv = 0.075 * ku_nv * tu_nv
            kd_ev = 0.075 * ku_ev * tu_ev
            kd_dv = 0.066666 * ku_dv * tu_dv

            ki_nv = 1.2 * ku_nv / tu_nv
            ki_ev = 1.2 * ku_ev / tu_ev
            ki_dv = .4 * ku_dv / tu_dv

            east_velocity = 0 # x_err * kp_ev + (x_err - prev_errs[0]) * kd_ev # + sum_errs[0] * ki_ev
            north_velocity = 0 # y_err * kp_nv + (y_err - prev_errs[1]) * kd_nv # + sum_errs[1] * ki_nv
            down_velocity = alt_err * kp_dv + (alt_err - prev_errs[2]) / dt * kd_dv + sum_errs[2] * dt * ki_dv
            print(alt_err)
            rot_angle = self.yaw + rot_err # yaw is in degrees, not degrees per second. must be set absolutely

            prev_errs = (x_err, y_err, alt_err)
            sum_errs = (sum_errs[0] + x_err, sum_errs[1] + y_err, sum_errs[2] + alt_err)

            await self.drone.offboard.set_velocity_ned(
                VelocityNedYaw(north_velocity, east_velocity, down_velocity, rot_angle)) # north, east, down (all m/s), yaw (degrees, north is 0, positive for clockwise)

            total_time_elapsed = total_time_elapsed + dt
            await asyncio.sleep(dt) # 10Hz

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
