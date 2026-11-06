import math
import numpy as np
from scipy import interpolate

"""
Single, canonical lat/lon <-> local x,y converter, anchored to one fixed
origin per swarm.

LatLonConversion (below) is a great-circle-based converter: it builds a
local Cartesian frame by taking the haversine/destination-point formulas
out to the two diagonal corners of a square region around the origin
(bearing 45 deg / 225 deg, out to `endDistance`), then linearly
interpolates lat<->y and lon<->x against those three points (SW corner,
origin, NE corner). Because geoToCart's interpolators and cartToGeo's
interpolators are built from the exact same three (lat,lon)<->(x,y) pairs
with domain/range swapped, the two directions are exact inverses of each
other (verified: round-trip error is 0 up to float rounding) -- PROVIDED
both directions go through the same LatLonConversion instance (i.e. the
same `origin`).

This is exactly the failure mode that caused the original bot.x/bot.y
mismatch: if two different converters (or two slightly different origins)
end up used for the forward and reverse conversion anywhere in the
pipeline, the round-trip guarantee above no longer holds. Keeping this as
ONE class, constructed ONCE per swarm and shared everywhere (see
hardware/ardupilot_link.py), is what removes that possibility.

+x = East, +y = North -- matches the sim's math-convention heading frame
(theta=0 points along +x, increasing counter-clockwise), and 1 unit of
x/y corresponds to ~1 metre near the origin.
"""


class LatLonConversion:
	def __init__(self, origin, endDistance=50000):
		"""
		origin: [lat, lon] in degrees -- the shared local-tangent-plane
		origin every x,y in the swarm is measured from. Use the SAME
		LatLonConversion instance everywhere x,y is computed for a given
		swarm; two instances built from even slightly different origins
		will not agree with each other even though each is internally
		exact.

		endDistance: metres -- half-width of the square region the
		geoToCart/cartToGeo interpolators are built over (their domain
		only has 3 points: -endDistance, 0, +endDistance along each
		axis, at bearing 45/225 from origin). Must be >= the largest x
		or y this converter will ever be asked to convert -- e.g. for a
		world/mission area W metres across, pass endDistance >= W/2 (with
		some margin) so no coordinate falls outside the interpolators'
		domain. Defaults to 50000 (50km), matching this class's original
		hardcoded value -- large enough for the up-to-10km-class missions
		this codebase has used so far, but pass an explicit value for
		bigger operational areas instead of silently extrapolating.
		"""
		self.origin = origin
		self.endDistance = endDistance

	def destination_location(self, distance, bearing):
		R = 6371e3  # Radius of earth in metres
		rlat1 = self.origin[0] * (math.pi / 180)
		rlon1 = self.origin[1] * (math.pi / 180)
		d = distance
		bearing = bearing * (math.pi / 180)  # Converting bearing to radians
		rlat2 = math.asin(
			(math.sin(rlat1) * math.cos(d / R))
			+ (math.cos(rlat1) * math.sin(d / R) * math.cos(bearing))
		)
		rlon2 = rlon1 + math.atan2(
			(math.sin(bearing) * math.sin(d / R) * math.cos(rlat1)),
			(math.cos(d / R) - (math.sin(rlat1) * math.sin(rlat2))),
		)
		rlat2 = rlat2 * (180 / math.pi)  # Converting to degrees
		rlon2 = rlon2 * (180 / math.pi)  # converting to degrees
		location = [rlat2, rlon2]
		return location

	def distance_bearing(self, destinationLattitude, destinationLongitude):
		R = 6371e3  # Radius of earth in metres
		rlat1 = self.origin[0] * (math.pi / 180)
		rlat2 = destinationLattitude * (math.pi / 180)
		rlon1 = self.origin[1] * (math.pi / 180)
		rlon2 = destinationLongitude * (math.pi / 180)
		dlat = (destinationLattitude - self.origin[0]) * (math.pi / 180)
		dlon = (destinationLongitude - self.origin[1]) * (math.pi / 180)
		# haversine formula to find distance
		a = (math.sin(dlat / 2) * math.sin(dlat / 2)) + (
			math.cos(rlat1) * math.cos(rlat2) * (math.sin(dlon / 2) * math.sin(dlon / 2))
		)
		c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
		distance = R * c  # distance in metres
		# formula for bearing
		y = math.sin(rlon2 - rlon1) * math.cos(rlat2)
		x = math.cos(rlat1) * math.sin(rlat2) - math.sin(rlat1) * math.cos(
			rlat2
		) * math.cos(rlon2 - rlon1)
		bearing = math.atan2(y, x)  # bearing in radians
		bearingDegrees = bearing * (180 / math.pi)
		out = [distance, bearingDegrees]
		return out

	def geoToCart(self, geoLocation):
		# The initial point of rectangle in (x,y) is (0,0) so considering the current
		# location as origin and retreiving the latitude and longitude from the GPS
		# origin = (12.948048, 80.139742) Format

		# Calculating the hypot end point for interpolating the latitudes and longitudes
		rEndDistance = math.sqrt(2 * (self.endDistance**2))

		# The bearing for the hypot angle is 45 degrees considering coverage area as square
		bearing = 45

		# Determining the Latitude and Longitude of Middle point of the sqaure area
		# and hypot end point of square area for interpolating latitude and longitude
		lEnd, rEnd = self.destination_location(
			rEndDistance, 180 + bearing
		), self.destination_location(rEndDistance, bearing)

		# Array of (x,y)
		x_cart, y_cart = [-self.endDistance, 0, self.endDistance], [-self.endDistance, 0, self.endDistance]

		# Array of (latitude, longitude)
		x_lon, y_lat = [lEnd[1], self.origin[1], rEnd[1]], [lEnd[0], self.origin[0], rEnd[0]]

		# Latitude interpolation function
		f_lat = interpolate.interp1d(y_lat, y_cart)

		# Longitude interpolation function
		f_lon = interpolate.interp1d(x_lon, x_cart)

		# Converting (latitude, longitude) to (x,y) using interpolation function
		y, x = f_lat(geoLocation[0]), f_lon(geoLocation[1])
		return float(x), float(y)

	def cartToGeo(self, cartLocation):
		# The initial point of rectangle in (x,y) is (0,0) so considering the current
		# location as origin and retreiving the latitude and longitude from the GPS
		# origin = (12.948048, 80.139742) Format

		# Calculating the hypot end point for interpolating the latitudes and longitudes
		rEndDistance = math.sqrt(2 * (self.endDistance**2))

		# The bearing for the hypot angle is 45 degrees considering coverage area as square
		bearing = 45

		# Determining the Latitude and Longitude of Middle point of the sqaure area
		# and hypot end point of square area for interpolating latitude and longitude
		lEnd, rEnd = self.destination_location(
			rEndDistance, 180 + bearing
		), self.destination_location(rEndDistance, bearing)

		# Array of (x,y)
		x_cart, y_cart = [-self.endDistance, 0, self.endDistance], [-self.endDistance, 0, self.endDistance]

		# Array of (latitude, longitude)
		x_lon, y_lat = [lEnd[1], self.origin[1], rEnd[1]], [lEnd[0], self.origin[0], rEnd[0]]

		# Latitude interpolation function
		f_lat = interpolate.interp1d(y_cart, y_lat)

		# Longitude interpolation function
		f_lon = interpolate.interp1d(x_cart, x_lon)

		# Converting (latitude, longitude) to (x,y) using interpolation function
		lat, lon = f_lat(cartLocation[1]), f_lon(cartLocation[0])
		return float(lat), float(lon)

	#--- adapter methods -----------------------------------------------------
	#Thin wrappers matching the (lat, lon)/(x, y) scalar-pair interface used
	#elsewhere in swarm_tasks (e.g. hardware/ardupilot_link.py), so this
	#class is a drop-in replacement without touching call sites beyond
	#construction. All three of these still go through the ONE set of
	#interpolators above (rebuilt fresh each call from self.origin, which
	#never changes after __init__) -- there is no second code path here
	#that could drift out of sync with geoToCart/cartToGeo.

	@property
	def origin_lat(self):
		return self.origin[0]

	@property
	def origin_lon(self):
		return self.origin[1]

	def latlon_to_xy(self, lat, lon):
		"""lat/lon (degrees) -> local (x, y) metres from the origin."""
		return self.geoToCart([lat, lon])

	def xy_to_latlon(self, x, y):
		"""local (x, y) metres from the origin -> lat/lon (degrees)."""
		return self.cartToGeo([x, y])

	def round_trip_error(self, lat, lon):
		"""
		Sanity check: lat/lon -> x,y -> lat/lon and returns the
		(dlat, dlon) discrepancy in degrees. Should be ~0 (float
		rounding only) as long as this is the only LatLonConversion
		instance used for a given swarm/origin -- if you see anything
		bigger, two different converters/origins are being mixed
		somewhere between the two conversions.
		"""
		x, y = self.latlon_to_xy(lat, lon)
		lat2, lon2 = self.xy_to_latlon(x, y)
		return lat2 - lat, lon2 - lon

	def xy_round_trip_error(self, x, y):
		"""Same check, starting from x,y instead of lat/lon. Returns (dx, dy) in metres."""
		lat, lon = self.xy_to_latlon(x, y)
		x2, y2 = self.latlon_to_xy(lat, lon)
		return x2 - x, y2 - y

	@staticmethod
	def compass_to_math_heading(compass_deg):
		"""
		MAVLink compass bearing (0=N, clockwise+) -> sim math-convention
		theta (0=+x/East, counter-clockwise+), wrapped to [0, 2*pi).
		"""
		theta = np.pi/2 - np.radians(compass_deg)
		return theta % (2*np.pi)

	#--- earth-geometry verification ------------------------------------------
	#These check the x,y frame against real Earth geometry AT THE BOT'S
	#ACTUAL LOCATION (not just near the origin) -- useful for pure-sim bots
	#too, not only ones bridged to real hardware via ArduPilotLink. See
	#verify_xy_movement() below and Bot.set_geo_check()/step() in
	#utils/robot.py for how a bot can run this automatically every step.

	@staticmethod
	def haversine_distance(lat1, lon1, lat2, lon2):
		"""
		True great-circle distance (metres) between two arbitrary
		(lat, lon) points -- NOT anchored to this converter's origin,
		unlike distance_bearing() above. This is the ground truth used
		by verify_xy_movement() to check a movement that may be
		happening far from the origin.
		"""
		R = 6371e3
		rlat1, rlat2 = math.radians(lat1), math.radians(lat2)
		dlat = math.radians(lat2-lat1)
		dlon = math.radians(lon2-lon1)
		a = math.sin(dlat/2)**2 + math.cos(rlat1)*math.cos(rlat2)*math.sin(dlon/2)**2
		c = 2*math.atan2(math.sqrt(a), math.sqrt(max(0.0, 1-a)))
		return R*c

	def verify_xy_movement(self, xy1, xy2):
		"""
		Checks a simulated x,y movement (e.g. one Bot.step()) against
		real Earth geometry, evaluated at wherever xy1/xy2 actually are
		-- not just near the origin. Converts both endpoints to
		lat/lon via this converter, then compares:
			- sim_distance_m: the flat Euclidean |xy2-xy1| the
			  simulation thinks it moved (its whole notion of "metres")
			- true_distance_m: the real great-circle distance between
			  the two corresponding lat/lon points on the actual Earth
		The two will only match exactly at the origin; elsewhere they
		diverge slightly due to the same chord-vs-local-derivative
		approximation documented in the class docstring above (bigger
		at high latitude, and grows somewhat with distance from the
		origin -- see verify_xy_movement()'s use in Bot.step()).

		Args:
			xy1, xy2: (x, y) tuples/pairs, the start and end of the
				movement being checked, in this converter's x,y frame.

		Returns: dict with keys
			sim_distance_m, true_distance_m, error_m, error_pct,
			latlon1 (tuple), latlon2 (tuple)
		"""
		x1, y1 = xy1
		x2, y2 = xy2
		sim_distance_m = math.hypot(x2-x1, y2-y1)

		lat1, lon1 = self.xy_to_latlon(x1, y1)
		lat2, lon2 = self.xy_to_latlon(x2, y2)
		true_distance_m = self.haversine_distance(lat1, lon1, lat2, lon2)

		error_m = sim_distance_m-true_distance_m
		error_pct = (error_m/true_distance_m*100.0) if true_distance_m > 1e-9 else 0.0

		return {
			'sim_distance_m': sim_distance_m,
			'true_distance_m': true_distance_m,
			'error_m': error_m,
			'error_pct': error_pct,
			'latlon1': (lat1, lon1),
			'latlon2': (lat2, lon2),
		}


#Backward-compatible alias: earlier revision of this module exposed the
#converter as `LatLonConverter(origin_lat, origin_lon)` (two scalar args)
#rather than `LatLonConversion(origin=[lat, lon])`. Kept so any code still
#importing the old name keeps working, without introducing a second
#implementation of the actual conversion math.
class LatLonConverter(LatLonConversion):
	def __init__(self, origin_lat, origin_lon, endDistance=50000):
		super().__init__([origin_lat, origin_lon], endDistance=endDistance)