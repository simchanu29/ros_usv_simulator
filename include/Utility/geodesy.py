import pyproj as proj
import numpy as np
import geometry as geo

CONVERSION_FACTOR_GPS = 1852.0  # in meters/min

# Define a projection with Proj4 notation, in this case an Icelandic grid
isn2004 = proj.Proj("+proj=lcc +lat_1=64.25 +lat_2=65.75 +lat_0=65 +lon_0=-19 +x_0=1700000 +y_0=300000 +no_defs +a=6378137 +rf=298.257222101 +to_meter=1")

# Define some common projections using EPSG codes
# spatialreference.org
wgs84 = proj.Proj("+init=EPSG:4326")  # LatLon with WGS84 datum used by GPS units and Google Earth
osgb36 = proj.Proj("+init=EPSG:27700")  # UK Ordnance Survey, 1936 datum
UTM26N = proj.Proj("+init=EPSG:32626")  # UTM coords, zone 26N, WGS84 datum
UTM27N = proj.Proj("+init=EPSG:32627")  # UTM coords, zone 27N, WGS84 datum
UTM28N = proj.Proj("+init=EPSG:32628")
UTM29N = proj.Proj("+init=EPSG:32629")
UTM30N = proj.Proj("+init=EPSG:32630")
UTM31N = proj.Proj("+init=EPSG:32631")


def ned2enu_yaw_rad(yaw):
    return geo.angle_rad(np.pi/2.0, - yaw)


def ned2enu_yaw_deg(yaw):
    return geo.angle_deg(90.0, - yaw)


def dist_m(x1, y1, x2, y2):
    return np.sqrt((x2-x1)**2 + (y2-y1)**2)
