import math

# Home location data from ArduPilot SITL console output (replace with your values)
home_latitude_deg = -35.363262
home_longitude_deg = 149.165237
home_altitude_m = 584.0
home_heading_deg = 353.0

# Constants
DEG_TO_RAD = math.pi / 180.0
EARTH_RADIUS_M = 6378137.0  # Earth's radius in meters (for ECEF coordinate system)

def ecef_to_enu(ecef_x, ecef_y, ecef_z):
    # Convert ECEF (Earth-Centered, Earth-Fixed) coordinates to ENU (East-North-Up) frame
    # using the "home" location as the reference point.

    # Convert latitude and longitude to radians
    latitude_rad = home_latitude_deg * DEG_TO_RAD
    longitude_rad = home_longitude_deg * DEG_TO_RAD

    # Calculate the ECEF components of the "home" location
    home_x, home_y, home_z = lla_to_ecef(home_latitude_deg, home_longitude_deg, home_altitude_m)

    # Perform coordinate transformation from ECEF to ENU
    dx = ecef_x - home_x
    dy = ecef_y - home_y
    dz = ecef_z - home_z

    enu_x = -math.sin(longitude_rad) * dx + math.cos(longitude_rad) * dy
    enu_y = -math.sin(latitude_rad) * math.cos(longitude_rad) * dx - math.sin(latitude_rad) * math.sin(longitude_rad) * dy + math.cos(latitude_rad) * dz
    enu_z = math.cos(latitude_rad) * math.cos(longitude_rad) * dx + math.cos(latitude_rad) * math.sin(longitude_rad) * dy + math.sin(latitude_rad) * dz

    return enu_x, enu_y, enu_z

def lla_to_ecef(latitude_deg, longitude_deg, altitude_m):
    # Convert latitude, longitude, and altitude to ECEF (Earth-Centered, Earth-Fixed) coordinates.
    # Returns ECEF X, Y, Z in meters.

    latitude_rad = latitude_deg * DEG_TO_RAD
    longitude_rad = longitude_deg * DEG_TO_RAD

    # Calculate ECEF components
    N = EARTH_RADIUS_M / math.sqrt(1.0 - (math.sin(latitude_rad) ** 2) * 0.00669437999014)
    ecef_x = (N + altitude_m) * math.cos(latitude_rad) * math.cos(longitude_rad)
    ecef_y = (N + altitude_m) * math.cos(latitude_rad) * math.sin(longitude_rad)
    ecef_z = (N * (1.0 - 0.00669437999014) + altitude_m) * math.sin(latitude_rad)

    return ecef_x, ecef_y, ecef_z
