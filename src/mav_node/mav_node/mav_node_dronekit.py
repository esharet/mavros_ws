import time
from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative

connection_string = "udp:127.0.0.1:14560"
print(f'Connecting to vehicle on: {connection_string}' )
vehicle = connect(connection_string, wait_ready=True)

while not vehicle.is_armable:
    print(" Waiting for vehicle to initialise...")
    time.sleep(1)

print("Arming motors")
vehicle.mode = VehicleMode("MANUAL")
vehicle.armed = True

while not vehicle.armed:
    print(" Waiting for arming...")
    time.sleep(1)

THROTTLE = 1700
WHEEL = 1700

# Must  send all values
# Each value in separated command
print(f"\nChannel overrides: {vehicle.channels}")
vehicle.channels.overrides = {'1': WHEEL, '2': 65535,'3': THROTTLE,'4':65535, '5':65535,'6':65535,'7':65535,'8':65535}
# vehicle.channels.overrides = {'1': WHEEL, '2': 65535,'3': 65535,'4':65535, '5':65535,'6':65535,'7':65535,'8':65535}

time.sleep(2)