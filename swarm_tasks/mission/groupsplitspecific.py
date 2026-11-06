import csv, os, sys
import simplekml
from geopy.distance import distance
from geopy.point import Point
from swarm_tasks.utils.locatePosition import geoToCart, cartToGeo
from utils import mission_dir
import numpy as np
# matplotlib is only used by plot_curve() below (a debug/manual-use-only
# plotter, never called from the real mission path) -- imported there,
# not here, so the headless backend's import of this module never pulls
# in matplotlib/Qt at all.


class SpecificSplitMission:
    def __init__(
        self, origin, center_lat_lons, drone_array, grid_spacing, coverage_area,run_id,
        drone_speeds=None,
    ):
        self.origin = origin
        self.center_lat_lons = center_lat_lons
        self.drone_array = drone_array
        self.grid_spacing = grid_spacing
        self.coverage_area = coverage_area
        # Hybrid swarm: {uav_id: cruise speed (m/s)} -- see
        # AutoSplitMission's identical field (groupsplitauto.py) for why.
        # None (default): falls back to an equal split, same as before.
        self.drone_speeds = drone_speeds
        self.mission_dir = mission_dir("specific_split",run_id)
        self.initial_heading = np.radians(0)  # Initial heading angle in radians
        self.G = 9.81  # Gravity (m/s²)
        self.MAX_BANK_ANGLE = np.radians(30)  # 40 degrees in radians
        self.SPEED = 18  # Aircraft speed in m/s
        self.TURN_RATE = (self.G * np.tan(self.MAX_BANK_ANGLE)) / self.SPEED  # rad/s
        self.sample_points = []
        self.path = []
        self.waypoints = []
        self.filePaths = []

    def _grid_csv(self, uav_id):
        return os.path.join(self.mission_dir, f"uav_{uav_id}_grid.csv")

    def _path_csv(self, uav_id):
        return os.path.join(self.mission_dir, f"uav_{uav_id}_path.csv")

    def _grid_kml(self, uav_id):
        return os.path.join(self.mission_dir, f"uav_{uav_id}_grid.kml")

    def _path_kml(self, uav_id):
        return os.path.join(self.mission_dir, f"uav_{uav_id}.kml")

    def _strip_heights(self, uav_ids, full_height):
        """See AutoSplitMission._strip_heights() (groupsplitauto.py) --
        identical weighted-by-cruise-speed split, falling back to equal
        shares when speed data isn't available for every drone here."""
        n = len(uav_ids)
        if not self.drone_speeds or any(
            self.drone_speeds.get(d) is None for d in uav_ids
        ):
            return [full_height / n] * n
        weights = [max(float(self.drone_speeds[d]), 1e-6) for d in uav_ids]
        total = sum(weights)
        return [full_height * w / total for w in weights]

    def CreateGridsForSpecifiedAreaAndSpecifiedDrones(
        self,
        center_latitude: float,
        center_longitude: float,
        uav_ids: list,
        grid_space: int,
        coverage_area: int,
    ) -> None:

        center_lat = center_latitude
        center_lon = center_longitude

        grid_spacing = grid_space
        meters_for_extended_lines = 250
        full_width, full_height = coverage_area, coverage_area

        strip_heights = self._strip_heights(uav_ids, full_height)

        center_point = Point(center_lat, center_lon)

        west_edge = distance(meters=full_width / 2).destination(center_point, 270)

        cumulative_height = 0.0
        for i in range(len(uav_ids)):
            rectangle_height = strip_heights[i]
            top_offset = (
                cumulative_height - (full_height / 2) + (rectangle_height / 2)
            )
            cumulative_height += rectangle_height

            top_center = distance(meters=top_offset).destination(center_point, 0)
            top = distance(meters=rectangle_height / 2).destination(top_center, 0)
            bottom = distance(meters=rectangle_height / 2).destination(top_center, 180)

            kml = simplekml.Kml()

            csv_data = []

            current_lat = bottom.latitude
            line_number = 0
            line = kml.newlinestring()
            line.altitudemode = simplekml.AltitudeMode.clamptoground
            line.style.linestyle.color = simplekml.Color.black
            line.style.linestyle.width = 2
            waypoint_number = 1

            while current_lat <= top.latitude:
                line_number += 1
                current_point = Point(current_lat, west_edge.longitude)
                east_point = distance(meters=full_width).destination(current_point, 90)
                if line_number % 2 == 1:
                    csv_data.append((current_point.latitude, current_point.longitude))
                    csv_data.append((east_point.latitude, east_point.longitude))

                    line.coords.addcoordinates(
                        [
                            (current_point.longitude, current_point.latitude),
                            (east_point.longitude, east_point.latitude),
                        ]
                    )
                    kml.newpoint(
                        name=f"{waypoint_number}",
                        coords=[(current_point.longitude, current_point.latitude)],
                    )
                    waypoint_number += 1
                    kml.newpoint(
                        name=f"{waypoint_number}",
                        coords=[(east_point.longitude, east_point.latitude)],
                    )
                    waypoint_number += 1
                else:
                    csv_data.append((east_point.latitude, east_point.longitude))
                    csv_data.append((current_point.latitude, current_point.longitude))

                    line.coords.addcoordinates(
                        [
                            (east_point.longitude, east_point.latitude),
                            (current_point.longitude, current_point.latitude),
                        ]
                    )
                    kml.newpoint(
                        name=f"{waypoint_number}",
                        coords=[(east_point.longitude, east_point.latitude)],
                    )
                    waypoint_number += 1
                    kml.newpoint(
                        name=f"{waypoint_number}",
                        coords=[(current_point.longitude, current_point.latitude)],
                    )
                    waypoint_number += 1

                if line_number % 2 == 1:
                    point_135 = distance(meters=meters_for_extended_lines).destination(
                        east_point, 135
                    )
                    csv_data.append((point_135.latitude, point_135.longitude))
                    line.coords.addcoordinates(
                        [(point_135.longitude, point_135.latitude)]
                    )
                    kml.newpoint(
                        name=f"{waypoint_number}",
                        coords=[(point_135.longitude, point_135.latitude)],
                    )
                    waypoint_number += 1
                else:
                    point_225 = distance(meters=meters_for_extended_lines).destination(
                        current_point, 225
                    )
                    csv_data.append((point_225.latitude, point_225.longitude))
                    line.coords.addcoordinates(
                        [(point_225.longitude, point_225.latitude)]
                    )
                    kml.newpoint(
                        name=f"{waypoint_number}",
                        coords=[(point_225.longitude, point_225.latitude)],
                    )
                    waypoint_number += 1

                current_lat = (
                    distance(meters=grid_spacing).destination(current_point, 0).latitude
                )

            uav_id = uav_ids[i]
            kml.save(self._grid_kml(uav_id))
            xy = []
            for data in csv_data:
                x, y = geoToCart(self.origin, 500000, data)
                xy.append((x, y))
            with open(
                self._grid_csv(uav_id),
                mode="w",
                newline="",
            ) as file:
                writer = csv.writer(file)
                writer.writerows(xy)
                self.generate_bezier_curve(xy, uav_id)

    def write_kml(self, data, num):
        kml = simplekml.Kml()
        line = kml.newlinestring()
        line.altitudemode = simplekml.AltitudeMode.clamptoground
        line.style.linestyle.color = simplekml.Color.blue
        line.style.linestyle.width = 2
        kml_data = []
        if len(data) == 0:
            print("No Mission Data")
            return
        for i, cmd in enumerate(data):
            lat, lon = cartToGeo(self.origin, 5000000, [cmd[0], cmd[1]])
            kml_data.append([lat, lon])
        for i in range(len(kml_data) - 1):
            line.coords.addcoordinates(
                [
                    (kml_data[i][1], kml_data[i][0]),
                    (kml_data[i + 1][1], kml_data[i + 1][0]),
                ]
            )
            kml.newpoint(name="{}".format(i), coords=[(kml_data[i][1], kml_data[i][0])])
        kml.save(self._path_kml(num))

    def get_heading_to_target(self, current_pos, target_pos):
        """Compute the heading angle required to face the target waypoint."""
        dx, dy = target_pos[0] - current_pos[0], target_pos[1] - current_pos[1]
        return np.arctan2(dy, dx)

    def normalize_angle(self, angle):
        """Ensure angles stay within -π to π range."""
        return (angle + np.pi) % (2 * np.pi) - np.pi

    def predict_path_with_waypoints(
        self,
        initial_pos,
        initial_heading,
        speed,
        turn_rate,
        waypoints,
        dt=0.1,
        max_iter=5000,
    ):
        """
        Predicts the aircraft's movement through multiple waypoints.

        Parameters:
        - initial_pos: (x, y) tuple for the start position
        - initial_heading: Initial heading in radians
        - speed: Constant velocity (m/s)
        - turn_rate: Max turn rate (rad/s)
        - waypoints: List of (x, y) waypoints
        - dt: Time step (s)
        - max_iter: Prevent infinite loops by limiting iterations

        Returns:
        - A list of (x, y) positions representing the predicted path.
        """
        x, y = initial_pos
        theta = initial_heading
        path = [(x, y)]

        for target in waypoints:
            iteration = 0
            while np.hypot(target[0] - x, target[1] - y) > speed * dt:
                if iteration > max_iter:
                    print(
                        f"Warning: Exceeded max iterations while moving to waypoint {target}, skipping!"
                    )
                    break  # Prevent infinite loop

                desired_theta = self.get_heading_to_target((x, y), target)

                heading_diff = self.normalize_angle(
                    desired_theta - theta
                )  # Normalize angle difference

                # Adjust heading smoothly within the turn rate limit
                theta += np.clip(heading_diff, -turn_rate * dt, turn_rate * dt)

                # Move the aircraft forward
                x += speed * np.cos(theta) * dt
                y += speed * np.sin(theta) * dt

                path.append((x, y))
                iteration += 2
        return np.array(path)

    def generate_bezier_curve(self, waypoints, index):
        self.waypoints.append(waypoints)
        result = [waypoints[0]]
        alternative = False

        for i in range(1, len(waypoints) - 1, 3):
            if i + 2 < len(waypoints) - 1:  # Ensure we don't include the last line
                if alternative:
                    heading = 180
                    alternative = False
                else:
                    heading = self.initial_heading
                    alternative = True
                data = []
                path1 = self.predict_path_with_waypoints(
                    waypoints[i],
                    heading,
                    self.SPEED,
                    self.TURN_RATE,
                    [waypoints[i], waypoints[i + 1], waypoints[i + 2]],
                )
                sampled_indices = np.linspace(0, len(path1) - 1, 10, dtype=int)
                sampled_points = path1[sampled_indices]
                for sample_point in sampled_points:
                    data.append(sample_point.tolist())
                data.append(waypoints[i + 2])
                self.sample_points.extend(data)
                result.extend(data)
            else:
                j = i
                while j <= len(waypoints) - 1:
                    result.append(waypoints[j])
                    j += 1
        self.path.append(result)
        self.write_kml(result, index)
        self.write_to_csv(result, index)
        return result

    def plot_curve(self):
        import matplotlib.pyplot as plt

        plt.figure(figsize=(8, 6))
        num_of_drones = 0
        for num in self.drone_array:
            num_of_drones += len(num)
        for num in range(num_of_drones):
            predict_path = np.array(self.path[num])
            sampled_points = np.array(self.sample_points)
            waypoints = self.waypoints[num]
            plt.plot(predict_path[:, 0], predict_path[:, 1], "r-", linewidth=2)
            plt.scatter(*zip(*waypoints), color="blue", s=100, marker="X")

            plt.scatter(
                sampled_points[:, 0],
                sampled_points[:, 1],
                color="black",
                marker="o",
                s=40,
            )

        # Annotate the sampled Bézier points with t-values
        for i, (x, y) in enumerate(sampled_points):
            plt.text(
                x,
                y,
                f"w",
                fontsize=10,
                verticalalignment="top",
                horizontalalignment="left",
            )

        plt.xlabel("X Position (m)")
        plt.ylabel("Y Position (m)")
        plt.title("Aircraft Path Prediction with 40° Roll Limit")
        plt.legend()
        plt.grid()
        plt.axis("equal")

        # --- Ensure Plot Opens Correctly ---
        plt.show(block=True)  # Ensures the window stays open

    def write_to_csv(self, data, num):
        file = self._path_csv(num)
        self.filePaths.append(file)
        with open(file, "w", newline="") as csvfile:
            csv_writer = csv.writer(csvfile)
            for row in data:
                csv_writer.writerow(row)

    def GroupSplitting(
        self, center_lat_lons, drone_array, grid_spacing, coverage_area
    ) -> bool:
        for i in range(len(center_lat_lons)):
            if len(drone_array[i]) == 0:
                continue
            self.CreateGridsForSpecifiedAreaAndSpecifiedDrones(
                center_lat_lons[i][0],
                center_lat_lons[i][1],
                drone_array[i],
                grid_spacing[i],
                coverage_area[i],
            )

        return True


"""
center_latlon = [
    [13.391341, 80.236145],
    [13.386840, 80.257992],
    [13.393423, 80.224792],
    [13.383977, 80.236774],
    [13.373029, 80.236966],
]
origin = ( 13.210665, 80.099739) #[13.375812,80.225549]
drone_array = [[2,4],[5,7],[1],[6],[3,8]]
grid_spacing = [50,100,50,100,50]
coverage_area = [1000,2000,2000,1000,1000]
split = SpecificSplitMission(origin=origin,center_lat_lons=center_latlon, drone_array=drone_array, grid_spacing=grid_spacing,
                         coverage_area=coverage_area)
isDone = split.GroupSplitting(
    center_lat_lons=center_latlon,
    drone_array=drone_array,
    grid_spacing=grid_spacing,
    coverage_area=coverage_area,
)
split.plot_curve()
print(isDone)

"""
