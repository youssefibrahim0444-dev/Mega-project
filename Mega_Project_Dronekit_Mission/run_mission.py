from dronekit import connect, VehicleMode, Command
from pymavlink import mavutil
import time

# -------- CONNECT TO VEHICLE --------
print("Connecting...")
vehicle = connect('127.0.0.1:14551', wait_ready=True)

# -------- UPLOAD MISSION --------
def upload_mission(filename):
    print("Uploading mission...")
    cmds = vehicle.commands
    cmds.clear()

    with open(filename) as f:
        lines = f.readlines()

    for line in lines:
        if line.startswith("QGC WPL") or line.strip() == "":
            continue

        parts = line.split("\t")
        if len(parts) < 12:
            continue  # skip invalid line

        cmd = Command(
            0, 0, 0,
            int(parts[2]),  # frame
            int(parts[3]),  # command
            int(parts[1]),  # current (0 or 1)
            int(parts[11]), # autocontinue
            float(parts[4]), # param1
            float(parts[5]), # param2
            float(parts[6]), # param3
            float(parts[7]), # param4
            float(parts[8]), # x (lat)
            float(parts[9]), # y (lon)
            float(parts[10]) # z (alt)
        )
        cmds.add(cmd)

    cmds.upload()
    print("Mission uploaded!")

# -------- MAIN --------
mission_file = "MegaProjectMission.waypoints"
upload_mission(mission_file)

# -------- ARM AND TAKEOFF --------
print("Waiting for vehicle to be armable...")
while not vehicle.is_armable:
    time.sleep(1)

print("Arming motors...")
vehicle.mode = VehicleMode("GUIDED")
vehicle.armed = True

while not vehicle.armed:
    print(" Waiting for arming...")
    time.sleep(1)

print("Taking off...")
vehicle.simple_takeoff(10)  

time.sleep(10)

print("Switching to AUTO...")
vehicle.mode = VehicleMode("AUTO")

# -------- MONITOR MISSION --------
while True:
    next_wp = vehicle.commands.next
    print(f"Next WP: {next_wp}")
    time.sleep(2)
