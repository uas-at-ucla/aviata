from dronekit import connect, VehicleMode

# Connect to vehicle
connection_string = "127.0.0.1:14540"
print("connecting to the vehicle on: %s" % connection_string)
vehicle = connect(connection_string, wait_ready=True)
print("connected to vehicle!")

# Get some vehicle attributes
gps = vehicle.gps_0
battery = vehicle.battery
status = vehicle.system_status.state
mode = vehicle.mode.name
print("GPS: %s" % gps)
print("battery: %s" % battery)
print("status: %s" % status)
print("mode: %s" % mode)

# print("\nPrint all parameters (iterate `vehicle.parameters`):")
# for item in vehicle.parameters.items():
#     print("Key:%s Value:%s" % (item[0], item[1]))

# Cleanup
print("cleaning up")
vehicle.close()
