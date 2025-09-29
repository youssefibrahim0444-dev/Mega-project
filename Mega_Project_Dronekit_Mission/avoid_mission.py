from dronekit import connect, VehicleMode, LocationGlobalRelative, Command
from pymavlink import mavutil
import math
import time

# ---------------- CONNECT ----------------
print("Connecting...")
vehicle = connect('127.0.0.1:14551', wait_ready=True)

# ---------------- CONSTANTS ----------------
CLEARANCE_DISTANCE = 20      # meters left/right shift
REJOIN_DISTANCE = 15         # meters ahead of obstacle
PROXIMITY_TRIGGER = 10       # meters before obstacle to trigger avoidance (will add obstacle radius)
OBSTACLE_HEIGHT = 100        # meters
TAKEOFF_ALT = 20             # takeoff altitude (meters)
REACH_THRESHOLD = 2.0        # m threshold for reaching GUIDED targets

# ---------------- OBSTACLES (midpoints) ----------------
# Format: (lat, lon, id, radius_m)
obstacles = [
    (38.31452985, -76.54215575, 1, 5),
    (38.31409635, -76.5434647,  2, 4),
    (38.3149045,  -76.54934945, 3, 3),
    (38.31681115, -76.5522355,  4, 5),
    (38.31789705, -76.5483785,  5, 4),
    (38.31676485, -76.545192,   6, 3),
    (38.31620505, -76.5473753,  7, 5),
    (38.3164618,  -76.5512538,  8, 4),
    (38.3157042,  -76.5494621,  9, 3),
]

cleared_obstacles = set()   # store IDs of cleared obstacles

# ---------------- HELPERS ----------------
def get_distance_metres(aLocation1, aLocation2):
    # Accept either LocationGlobalRelative or tuple-like objects
    lat1 = getattr(aLocation1, 'lat', None) or (aLocation1[0] if isinstance(aLocation1, (tuple, list)) else None)
    lon1 = getattr(aLocation1, 'lon', None) or (aLocation1[1] if isinstance(aLocation1, (tuple, list)) else None)
    lat2 = getattr(aLocation2, 'lat', None) or (aLocation2[0] if isinstance(aLocation2, (tuple, list)) else None)
    lon2 = getattr(aLocation2, 'lon', None) or (aLocation2[1] if isinstance(aLocation2, (tuple, list)) else None)
    dlat = lat2 - lat1
    dlong = lon2 - lon1
    return math.sqrt((dlat * 1.113195e5)**2 + (dlong * 1.113195e5)**2)

def location_offset_meters(original_location, dNorth, dEast):
    earth_radius = 6378137.0
    dLat = dNorth / earth_radius
    dLon = dEast / (earth_radius * math.cos(math.pi * original_location.lat / 180))
    newlat = original_location.lat + (dLat * 180 / math.pi)
    newlon = original_location.lon + (dLon * 180 / math.pi)
    return LocationGlobalRelative(newlat, newlon, original_location.alt)

def goto(location):
    vehicle.simple_goto(location)

# ---------------- ARM & TAKEOFF ----------------
def arm_and_takeoff(aTargetAltitude):
    print("Arming motors...")
    while not vehicle.is_armable:
        print(" Waiting for vehicle to initialise...")
        time.sleep(1)

    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

    while not vehicle.armed:
        print(" Waiting for arming...")
        time.sleep(1)

    print(f"Taking off to {aTargetAltitude}m...")
    vehicle.simple_takeoff(aTargetAltitude)

    while True:
        alt = vehicle.location.global_relative_frame.alt
        print(f" Altitude: {alt:.1f}m")
        if alt >= aTargetAltitude * 0.95:
            print("Reached target altitude")
            break
        time.sleep(1)

# ---------------- MISSION UPLOAD ----------------
def upload_mission_from_file(filename):
    print(f"Uploading mission from {filename}...")
    cmds = vehicle.commands
    cmds.clear()

    with open(filename) as f:
        lines = f.readlines()

    for line in lines:
        if line.startswith("QGC WPL") or line.strip() == "":
            continue
        parts = line.split("\t")
        if len(parts) < 12:
            continue
        cmd = Command(
            0, 0, 0,
            int(parts[2]), int(parts[3]),
            int(parts[1]), int(parts[11]),
            float(parts[4]), float(parts[5]), float(parts[6]), float(parts[7]),
            float(parts[8]), float(parts[9]), float(parts[10])
        )
        cmds.add(cmd)

    cmds.upload()
    print("✅ Mission uploaded!")

# ---------------- OBSTACLE AVOIDANCE ----------------
def avoid_obstacle(obs_lat, obs_lon, obs_id, obs_radius):
    if obs_id in cleared_obstacles:
        return  # already handled

    print(f"⚠️ Obstacle {obs_id} detected near ({obs_lat:.6f}, {obs_lon:.6f}) radius={obs_radius}m")

    current = vehicle.location.global_relative_frame
    obs_location = LocationGlobalRelative(obs_lat, obs_lon, current.alt)

    # Decide left/right dynamically by checking clearance to other obstacles
    left_clear = True
    right_clear = True

    # left offset and right offset points relative to obstacle center (using simple east offset)
    left_point = location_offset_meters(obs_location, 0, CLEARANCE_DISTANCE)
    right_point = location_offset_meters(obs_location, 0, -CLEARANCE_DISTANCE)

    for (lat, lon, oid, radius) in obstacles:
        if oid == obs_id:
            continue
        other = LocationGlobalRelative(lat, lon, current.alt)
        # if other obstacle is too close to left or right candidate, mark not clear
        if get_distance_metres(left_point, other) < (radius + obs_radius + 1.0):
            left_clear = False
        if get_distance_metres(right_point, other) < (radius + obs_radius + 1.0):
            right_clear = False

    # prefer side with more clearance
    if left_clear and not right_clear:
        shift_east = CLEARANCE_DISTANCE
        side = "LEFT"
    elif right_clear and not left_clear:
        shift_east = -CLEARANCE_DISTANCE
        side = "RIGHT"
    else:
        # Both clear or both blocked: choose the side with larger min-distance to other obstacles
        min_left = min(get_distance_metres(left_point, LocationGlobalRelative(lat, lon, current.alt)) - r for (lat, lon, _, r) in obstacles)
        min_right = min(get_distance_metres(right_point, LocationGlobalRelative(lat, lon, current.alt)) - r for (lat, lon, _, r) in obstacles)
        if min_left >= min_right:
            shift_east = CLEARANCE_DISTANCE
            side = "LEFT"
        else:
            shift_east = -CLEARANCE_DISTANCE
            side = "RIGHT"

    # compute bypass and rejoin points (bypass is lateral, rejoin is forward)
    bypass = location_offset_meters(obs_location, 0, shift_east)
    rejoin = location_offset_meters(obs_location, REJOIN_DISTANCE, 0)

    print(f"Switching to GUIDED to bypass obstacle {obs_id} -> {side}")
    vehicle.mode = VehicleMode("GUIDED")
    time.sleep(1)

    # Fly to bypass
    print(f"→ Guiding to bypass point {bypass.lat:.6f},{bypass.lon:.6f}")
    goto(bypass)
    t0 = time.time()
    while get_distance_metres(vehicle.location.global_relative_frame, bypass) > REACH_THRESHOLD:
        if time.time() - t0 > 30:
            print("  Timeout reaching bypass point; proceeding to rejoin.")
            break
        time.sleep(0.8)

    # Fly to rejoin
    print(f"→ Guiding to rejoin point {rejoin.lat:.6f},{rejoin.lon:.6f}")
    goto(rejoin)
    t1 = time.time()
    while get_distance_metres(vehicle.location.global_relative_frame, rejoin) > REACH_THRESHOLD:
        if time.time() - t1 > 40:
            print("  Timeout reaching rejoin point; switching back to AUTO.")
            break
        time.sleep(0.8)

    print(f"✅ Cleared obstacle {obs_id}, resuming AUTO")
    vehicle.mode = VehicleMode("AUTO")

    cleared_obstacles.add(obs_id)

# ---------------- MISSION MONITOR ----------------
def monitor_mission():
    cmds = vehicle.commands
    cmds.download()
    cmds.wait_ready()
    total_wps = cmds.count

    while True:
        nextwp = vehicle.commands.next
        loc = vehicle.location.global_relative_frame
        print(f"Next WP: {nextwp}/{total_wps}, Mode: {vehicle.mode.name}, Pos: ({loc.lat:.6f},{loc.lon:.6f}), Alt:{loc.alt:.1f}")

        # Mission completed → RTL
        if nextwp == total_wps:
            print("✅ Mission complete. Switching to RTL...")
            vehicle.mode = VehicleMode("RTL")
            break

        # Check obstacles (use each obstacle radius in trigger)
        for (lat, lon, oid, radius) in obstacles:
            if oid in cleared_obstacles:
                continue
            obs_location = LocationGlobalRelative(lat, lon, loc.alt)
            d = get_distance_metres(loc, obs_location)
            if d < (PROXIMITY_TRIGGER + radius) and loc.alt <= OBSTACLE_HEIGHT:
                avoid_obstacle(lat, lon, oid, radius)

        time.sleep(1.0)

# ---------------- RUN ----------------
MISSION_FILE = "MegaProjectMission.waypoints"
upload_mission_from_file = None 


def upload_mission_from_file(filename):
    print(f"Uploading mission from {filename}...")
    cmds = vehicle.commands
    cmds.clear()

    with open(filename) as f:
        lines = f.readlines()

    for line in lines:
        if line.startswith("QGC WPL") or line.strip() == "":
            continue
        parts = line.split("\t")
        if len(parts) < 12:
            continue
        cmd = Command(
            0, 0, 0,
            int(parts[2]), int(parts[3]),
            int(parts[1]), int(parts[11]),
            float(parts[4]), float(parts[5]), float(parts[6]), float(parts[7]),
            float(parts[8]), float(parts[9]), float(parts[10])
        )
        cmds.add(cmd)

    cmds.upload()
    print("✅ Mission uploaded!")

# Upload mission, arm, takeoff, start AUTO, and monitor
upload_mission_from_file(MISSION_FILE)
arm_and_takeoff(TAKEOFF_ALT)

print("Switching to AUTO to start mission...")
vehicle.mode = VehicleMode("AUTO")
time.sleep(1)

print("Monitoring mission and avoiding obstacles...")
monitor_mission()

# Close vehicle connection
vehicle.close()
