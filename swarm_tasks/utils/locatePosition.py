# Import Necessary Packages
import math
from functools import lru_cache
from scipy import interpolate


def destination_location(homeLattitude, homeLongitude, distance, bearing):
    R = 6371e3 #Radius of earth in metres
    rlat1 = homeLattitude * (math.pi/180) 
    rlon1 = homeLongitude * (math.pi/180)
    d = distance
    bearing = bearing * (math.pi/180) #Converting bearing to radians
    rlat2 = math.asin((math.sin(rlat1) * math.cos(d/R)) + (math.cos(rlat1) * math.sin(d/R) * math.cos(bearing)))
    rlon2 = rlon1 + math.atan2((math.sin(bearing) * math.sin(d/R) * math.cos(rlat1)) , (math.cos(d/R) - (math.sin(rlat1) * math.sin(rlat2))))
    rlat2 = rlat2 * (180/math.pi) #Converting to degrees
    rlon2 = rlon2 * (180/math.pi) #converting to degrees
    location = [rlat2, rlon2]
    return location

def distance_bearing(homeLattitude, homeLongitude, destinationLattitude, destinationLongitude):
    R = 6371e3 #Radius of earth in metres
    rlat1 = homeLattitude * (math.pi/180)
    rlat2 = destinationLattitude * (math.pi/180) 
    rlon1 = homeLongitude * (math.pi/180) 
    rlon2 = destinationLongitude * (math.pi/180) 
    dlat = (destinationLattitude - homeLattitude) * (math.pi/180)
    dlon = (destinationLongitude - homeLongitude) * (math.pi/180)
    #haversine formula to find distance
    a = (math.sin(dlat/2) * math.sin(dlat/2)) + (math.cos(rlat1) * math.cos(rlat2) * (math.sin(dlon/2) * math.sin(dlon/2)))
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
    distance = R * c #distance in metres
    #formula for bearing
    y = math.sin(rlon2 - rlon1) * math.cos(rlat2)
    x = math.cos(rlat1) * math.sin(rlat2) - math.sin(rlat1) * math.cos(rlat2) * math.cos(rlon2 - rlon1)
    bearing = math.atan2(y, x) #bearing in radians
    bearingDegrees = bearing * (180/math.pi)
    out = [distance, bearingDegrees]
    return out

@lru_cache(maxsize=32)
def _geo_transform(origin_lat, origin_lon, endDistance):
    """Precomputes everything geoToCart()/cartToGeo() need for a given
    (origin, endDistance) pair: the two destination_location() calls
    (pure trig, depending only on origin/endDistance) and the four
    scipy interp1d objects built from them.

    origin and endDistance are effectively constant for an entire
    mission run (origin is read once from rectangles.yaml at startup;
    endDistance is rarely edited), but geoToCart/cartToGeo are called
    several times per bot per tick in the live guidance loop
    (swarm.py's searchSub/goalSub/...). Without this cache, every one
    of those calls was rebuilding four scipy interpolator objects from
    scratch just to evaluate each once -- measured at ~110us/call, most
    of it this redundant setup rather than the actual interpolation.
    maxsize=32 is generous for how many distinct origins a single
    long-running process would ever see.
    """
    origin = (origin_lat, origin_lon)
    rEndDistance = math.sqrt(2 * (endDistance ** 2))
    bearing = 45
    lEnd = destination_location(origin[0], origin[1], rEndDistance, 180 + bearing)
    rEnd = destination_location(origin[0], origin[1], rEndDistance, bearing)

    x_cart = y_cart = [-endDistance, 0, endDistance]
    x_lon = [lEnd[1], origin[1], rEnd[1]]
    y_lat = [lEnd[0], origin[0], rEnd[0]]

    geo_to_cart_lat = interpolate.interp1d(y_lat, y_cart)
    geo_to_cart_lon = interpolate.interp1d(x_lon, x_cart)
    cart_to_geo_lat = interpolate.interp1d(y_cart, y_lat)
    cart_to_geo_lon = interpolate.interp1d(x_cart, x_lon)
    return geo_to_cart_lat, geo_to_cart_lon, cart_to_geo_lat, cart_to_geo_lon


def geoToCart(origin, endDistance, geoLocation):
    # The initial point of rectangle in (x,y) is (0,0) so considering the current
    # location as origin and retreiving the latitude and longitude from the GPS
    # origin = (12.948048, 80.139742) Format
    f_lat, f_lon, _, _ = _geo_transform(origin[0], origin[1], endDistance)

    # Converting (latitude, longitude) to (x,y) using interpolation function
    y, x = f_lat(geoLocation[0]), f_lon(geoLocation[1])
    return (float(x),float(y))

def cartToGeo(origin, endDistance, cartLocation):
    # The initial point of rectangle in (x,y) is (0,0) so considering the current
    # location as origin and retreiving the latitude and longitude from the GPS
    # origin = (12.948048, 80.139742) Format
    _, _, f_lat, f_lon = _geo_transform(origin[0], origin[1], endDistance)

    # Converting (x,y) to (latitude, longitude) using interpolation function
    lat, lon = f_lat(cartLocation[1]), f_lon(cartLocation[0])
    return (lat, lon)
