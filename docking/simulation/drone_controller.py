from dronekit import connect, Command, LocationGlobal
from pymavlink import mavutil
import time, sys, argparse, math


class DroneController:
    """
    Interface for controlling the drone
    Uses MAVLink messages sent through DroneKit
    """

    MAV_MODE_AUTO = 4

    connection_string = "127.0.0.1:14540"
    vehicle = connect(connection_string, wait_ready=True)
    home_position_set = False
    home = vehicle.location.global_relative_frame
    cmds = vehicle.commands

    def __init__(self):
        print("Initializing drone...")
        self.cmds.clear()
        while not self.home_position_set:
            print("Waiting for home position...")
            time.sleep(1)
        print("Home position received, ready to continue")

    def takeoff(self):
        self.PX4setMode(self.MAV_MODE_AUTO)
        waypoint = self.get_location_offset_meters(self.home, 0, 0, 20)
        
        # Takeoff command
        cmd = Command(0, 0, 0, 
                      mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                      mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
                      0, 1, 0, 0, 0, 0,
                      waypoint.lat, waypoint.lon, waypoint.alt)
        self.cmds.add(cmd)
        
        # Loiter command -> temporary
        cmd = Command(0, 0, 0,
                      mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                      mavutil.mavlink.MAV_CMD_NAV_LOITER_UNLIM,
                      0, 1, 0, 0, 0, 0,
                      0, 0, 0) # lat, long, alt empty -> current loc
        self.cmds.add(cmd)
        self.cmds.upload()
        time.sleep(2)

        self.vehicle.armed = True
        while not self.vehicle.armed:
            print("Arming...")
            time.sleep(1)

        while len(self.vehicle.commands) > 0:
            time.sleep(1)
        
        self.vehicle.armed = False
        self.vehicle.close()
    
    def PX4setMode(self, mavMode):
        self.vehicle._master.mav.command_long_send(self.vehicle._master.target_system, self.vehicle._master.target_component, mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0, mavMode, 0, 0, 0, 0, 0, 0)

    def get_location_offset_meters(self, original_location, dNorth, dEast, alt):
        """
        Returns a LocationGlobal object containing the latitude/longitude `dNorth` and `dEast` metres from the
        specified `original_location`. The returned Location adds the entered `alt` value to the altitude of the `original_location`.
        The function is useful when you want to move the vehicle around specifying locations relative to
        the current vehicle position.
        The algorithm is relatively accurate over small distances (10m within 1km) except close to the poles.
        For more information see:
        http://gis.stackexchange.com/questions/2951/algorithm-for-offsetting-a-latitude-longitude-by-some-amount-of-meters
        """
        earth_radius = 6378137.0 # radius of "spherical" earth

        # Coordinate offsets in radians
        dLat = dNorth/earth_radius
        dLon = dEast/(earth_radius*math.cos(math.pi*original_location.lat/180))

        # New position in decimal degrees
        newlat = original_location.lat + (dLat * 180/math.pi)
        newlon = original_location.lon + (dLon * 180/math.pi)
        return LocationGlobal(newlat, newlon, original_location.alt + alt)

# Listeners
@DroneController.vehicle.on_message('HOME_POSITION')
def listener(self, name, home_position):
    DroneController.home_position_set = True
    