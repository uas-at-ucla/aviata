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
from camera_simulator import CameraSimulator
from image_analyzer import ImageAnalyzer
from debug_window import DebugWindow
from pid_controller import PIDController
from log import Log

boom_length = 1 # meter
fov = 48.8
altitude_disp = boom_length / 2 / math.tan(math.radians(fov / 2)) * 2 # add 100% tolerance

class Drone:

    def __init__(self, target):
        self.camera_simulator = CameraSimulator(target) # later, use camera here
        self.image_analyzer = ImageAnalyzer()
        self.north = 0
        self.east = 0
        self.down = -8 # initial altitude
        self.yaw = 0
        self.target = target 
        self.dt = 0.05 # delta t update frequency, 20hz
        self.MAX_ATTEMPTS = 3 # number of unsuccessful attempts before aborting docking
        self.MAX_HEIGHT = 10 # max height above central target before failure
        self.MAX_HEIGHT_STAGE_2 = 2
        self.STAGE_1_TOLERANCE = 0.1 
        self.STAGE_2_TOLERANCE = 0.05
        self.log=Log()
        self.logging=True

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
        await self.drone.action.set_takeoff_altitude(3.0) # meters
        await self.drone.action.takeoff()
        await asyncio.sleep(8)

        # start offboard mode (requires setting initial setpoint)
        print("-- Setting initial setpoint")
        await self.drone.offboard.set_velocity_ned(
            VelocityNedYaw(0.0, 0.0, 0.0, 135))

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

    async def initiate_docking(self, id):

        # Get telemetry periodically from the drone
        telemetry_task = asyncio.create_task(self.get_telemetry_position())
        rotation_task = asyncio.create_task(self.get_telemetry_rotation())

        if await self.stage1(id):
            await self.stage2(id)
            await self.land()
        else:
            print("Docking failed: Max height exceeded")
            await self.safe_land()

        telemetry_task.cancel()
        rotation_task.cancel()

    async def stage1(self, id):
        """Position the drone above the large central target, returns true if successful"""
        print("Docking stage 1")
        self.log.writeLiteral("Docking stage 1")

        debug_window=DebugWindow(1,self.target)

        failed_frames = 0 # Number of frames since we detected the target
        successful_frames = 0 # Number of frames within tolerance 
        pid_controller = PIDController(self.dt)

        while True:
            start_millis = int(round(time.time() * 1000))
            img = self.camera_simulator.updateCurrentImage(self.east, self.north, self.down * -1.0, self.yaw)
            errs = self.image_analyzer.process_image(img, 0, self.yaw)
            
            if errs is None:
                self.log.writeLiteral("No Errors")
                failed_frames = failed_frames + 1

                if self.down * -1.0 - self.target.getAlt() > self.MAX_HEIGHT: # drone moves above max height, docking fails
                    return False

                if failed_frames > 1 / self.dt: # haven't detected target in 1 second
                    await self.drone.offboard.set_velocity_body(
                        VelocityBodyYawspeed(0, 0, -0.2, 0))

                await asyncio.sleep(self.dt)
                continue

            failed_frames = 0
            x_err, y_err, alt_err, rot_err, tags_detected = errs

            debug_window.updateWindow(self.east, self.north, self.down * -1.0, self.yaw, tags_detected)
            if self.logging:
                self.log.write(self.east, self.north, self.down * -1.0, self.yaw, tags_detected,successful_frames,x_err,y_err,alt_err,rot_err)

            x_err, y_err, alt_err = self.offset_errors(x_err, y_err, alt_err, rot_err, id)
            east_velocity, north_velocity, down_velocity = pid_controller.get_velocities(x_err, y_err, alt_err, 0.4)
            
            # Checks if drone is aligned with central target
            if util.is_between_symm(alt_err, self.STAGE_1_TOLERANCE) and util.is_between_symm(x_err, self.STAGE_1_TOLERANCE) and util.is_between_symm(y_err, self.STAGE_1_TOLERANCE):
                successful_frames += 1
            else:
                successful_frames = 0

            # Ends loop if aligned for 1 second
            if successful_frames == 1 / self.dt:
                break

            await self.drone.offboard.set_velocity_ned(
                VelocityNedYaw(north_m_s=north_velocity, east_m_s=east_velocity, down_m_s=down_velocity, yaw_deg=rot_err)) # north, east, down (all m/s), yaw (degrees, north is 0, positive for clockwise)

            current_millis = int(round(time.time() * 1000))
            if current_millis - start_millis < self.dt:
                await asyncio.sleep(self.dt - (current_millis - start_millis))
        
        debug_window.destroyWindow()
        return True

    async def stage2(self, id):
        """Position the drone above the peripheral target and descend"""
        print("Docking stage 2")
        self.log.writeLiteral("Docking stage 2")

        debug_window = DebugWindow(2,self.target)
        pid_controller = PIDController(self.dt)
        successful_frames = 0
        while True:
        
            img = self.camera_simulator.updateCurrentImage(self.east, self.north, self.down * -1.0, self.yaw,id)
            errs = self.image_analyzer.process_image(img, id, self.yaw)
            checked_frames = 0
            docking_attempts = 0

            # Waits for one second to detect target tag, ascends to find central target if fails
            while errs is None:
                checked_frames += 1
                self.log.writeLiteral("No errors, checked frames: "+str(checked_frames))
                await self.drone.offboard.set_velocity_body(VelocityBodyYawspeed(0, 0, -0.1, 0))

                if checked_frames > 1 / self.dt:
                    docking_attempts += 1
                    if docking_attempts > self.MAX_ATTEMPTS:
                        print("Docking failed")
                        self.log.writeLiteral("Docking failed")
                        await self.safe_land()
                        return

                    img = self.camera_simulator.updateCurrentImage(self.east, self.north, self.down * -1.0, self.yaw,id)
                    errs = self.image_analyzer.process_image(img,0,self.yaw)
                    
                    # Ascends until maximum height or until peripheral target detected
                    while errs is None and self.down * -1.0 - self.target.getAlt() < self.MAX_HEIGHT_STAGE_2:
                        await self.drone.offboard.set_velocity_body(VelocityBodyYawspeed(0, 0, -0.2, 0))
                        img = self.camera_simulator.updateCurrentImage(self.east, self.north, self.down * -1.0, self.yaw, id)
                        errs = self.image_analyzer.process_image(img, id, self.yaw)

                    if not errs is None:
                        break

                    # If peripheral tag not found, ascends until maximum height or until central target detected
                    while errs is None and self.down * -1.0 - self.target.getAlt() < self.MAX_HEIGHT:
                        await self.drone.offboard.set_velocity_body(VelocityBodyYawspeed(0, 0, -0.2, 0))
                        img = self.camera_simulator.updateCurrentImage(self.east, self.north, self.down * -1.0, self.yaw)
                        errs = self.image_analyzer.process_image(img,0, self.yaw)
                    
                    # Re-attempts stage 1
                    if not errs is None:
                        await self.stage1(id)
                        img = self.camera_simulator.updateCurrentImage(self.east, self.north, self.down * -1.0, self.yaw)
                        errs = self.image_analyzer.process_image(img, id, self.yaw)
                        checked_frames = 0
                    else: # If the central target cannot be found at maximum height (maybe could have re-attempt stage 1 again, not sure)
                        print("Docking failed")
                        await self.safe_land()
                        return 
                        

                await asyncio.sleep(self.dt)
                img = self.camera_simulator.updateCurrentImage(self.east, self.north, self.down * -1.0, self.yaw,id)
                errs = self.image_analyzer.process_image(img, id,self.yaw)

            x_err, y_err, alt_err, rot_err, tags_detected = errs
            alt_err = alt_err - .05 
            if self.logging:
                self.log.write(self.east, self.north, self.down * -1.0, self.yaw, tags_detected,successful_frames,x_err,y_err,alt_err,rot_err)

            # Checks if drone is aligned with docking
            if alt_err < self.STAGE_2_TOLERANCE * 2 and alt_err > -1 * self.STAGE_2_TOLERANCE and rot_err < 2.0 and rot_err > -2.0 and x_err > -1 * self.STAGE_2_TOLERANCE and x_err < 1 * self.STAGE_2_TOLERANCE and y_err > -1 * self.STAGE_2_TOLERANCE and y_err<self.STAGE_2_TOLERANCE:
                successful_frames += 1
            else:
                successful_frames = 0

            # Adjusts errors for distance to target to prevent overshoot
            # Adjusted errors asymptote to 1 as alt_err increases, goes to 0 as alt_err decreases
            OVERSHOOT_CONSTANT=.6 # use to adjust speed of descent, higher constant means faster, lower means less overshooting
            # x_err*=np.tanh(OVERSHOOT_CONSTANT*alt_err*alt_err)
            # y_err*=np.tanh(OVERSHOOT_CONSTANT*alt_err*alt_err)
            # rot_err*=np.tanh(OVERSHOOT_CONSTANT*alt_err*alt_err)
            # alt_err*=np.tanh(OVERSHOOT_CONSTANT*alt_err*alt_err)

            # Ends loop if aligned for 1 second
            if successful_frames == 1 / self.dt:
                break

            debug_window.updateWindow(self.east, self.north, self.down * -1.0, self.yaw, tags_detected)

            east_velocity, north_velocity, down_velocity = pid_controller.get_velocities(x_err, y_err, alt_err, 0.1)

            await self.drone.offboard.set_velocity_ned(
                VelocityNedYaw(north_m_s=north_velocity, east_m_s=east_velocity, down_m_s=down_velocity, yaw_deg=rot_err)) # north, east, down (all m/s), yaw (degrees, north is 0, positive for clockwise)
            
            await asyncio.sleep(self.dt)

    def offset_errors(self, x, y, alt, rot, target):
        # maps target 3 to 0 degrees, target 2 to 45 degrees, 1 to 90, 8 to 135, 7 to 180, ... , 4 to 315
        # note that peripheral targets are numbered so that the north one is 1, then increasing clockwise
        target_offset = abs(target - 3) * 45 if target <= 3 else (11 - target) * 45

        x = x + boom_length / 2 * math.cos(math.radians(target_offset - rot))
        y = y + boom_length / 2 * math.sin(math.radians(target_offset - rot))
        
        alt = alt - altitude_disp
        return x, y, alt
    
    # Moves drone up and away from current location, then ends offboard control and lands
    async def safe_land(self):
        dt=0.05
        for i in range (5/0.05): #Drone flies away up and to the north for 5 seconds 
            await self.drone.offboard.set_velocity_ned(
                        VelocityNedYaw(0.4, 0, -0.4, 0.0)) #These numbers can be changed to get out of GPS error margin
            asyncio.sleep(dt)
        
        await self.land()

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
            await self.drone.offboard.stop()
        except OffboardError as error:
            print(f"Stopping offboard mode failed with error code: \
                {error._result.result}")

        print("-- Landing drone")
        await self.drone.action.land()
