import numpy as np
import csv, os, sys
import simplekml
from geopy.distance import distance
from geopy.point import Point
from swarm_tasks.utils.locatePosition import geoToCart, cartToGeo
from utils import mission_dir
# matplotlib is only used by plot_curve() below (a debug/manual-use-only
# plotter, never called from the real mission path) -- imported there,
# not here, so the headless backend's import of this module never pulls
# in matplotlib/Qt at all.


GAP_BETWEEN_RECTANGLES_M = 50


def check_coverage_area(num_of_drones, grid_space, coverage_area):
    """Raise ValueError unless every drone's strip is at least one grid
    line tall. GridFormation() cuts the coverage area into one strip per
    drone with a gap between strips; when the area is smaller than the
    gaps alone the strips get negative height and no waypoints, which
    used to surface much later as an IndexError in generate_bezier_curve().
    """
    needed = (num_of_drones - 1) * GAP_BETWEEN_RECTANGLES_M + num_of_drones * grid_space
    if coverage_area < needed:
        raise ValueError(
            f"coverage area {coverage_area:g} m is too small for "
            f"{num_of_drones} drone(s) with {grid_space:g} m grid spacing: "
            f"need at least {needed:g} m "
            f"({GAP_BETWEEN_RECTANGLES_M} m gap between strips + one grid line each)"
        )


class BezierCurveMultiple:
    def __init__(
        self,
        origin,
        center_latitude,
        center_longitude,
        num_of_drones,
        grid_space,
        coverage_area,
        run_id
    ):
        self.initial_heading = np.radians(0)  # Initial heading angle in radians
        self.G = 9.81  # Gravity (m/s²)
        self.MAX_BANK_ANGLE = np.radians(20)  # 20 degrees in radians
        self.SPEED = 18  # Aircraft speed in m/s
        self.TURN_RATE = (self.G * np.tan(self.MAX_BANK_ANGLE)) / self.SPEED  # rad/s
        self.mission_dir = mission_dir("search",run_id)
        self.origin = origin
        self.center_latitude = center_latitude
        self.center_longitude = center_longitude
        self.num_of_drones = num_of_drones
        self.grid_space = grid_space
        self.coverage_area = coverage_area
        self.path = []
        self.filePaths = []
        self.waypoints = []
        self.grid_path = []
        self.sample_points = []

    def _grid_csv(self, drone_num):
        return os.path.join(self.mission_dir, f"drone_{drone_num}_grid.csv")

    def _path_csv(self, drone_num):
        return os.path.join(self.mission_dir, f"drone_{drone_num}_path.csv")

    def _grid_kml(self, drone_num):
        return os.path.join(self.mission_dir, f"drone_{drone_num}_grid.kml")

    def _path_kml(self, drone_num):
        return os.path.join(self.mission_dir, f"drone_{drone_num}.kml")

    def write_to_csv(self, data, num):
        file = self._path_csv(num)
        if file not in self.filePaths:
            self.filePaths.append(file)
            with open(file, "w", newline="") as csvfile:
                csv_writer = csv.writer(csvfile)
                for row in data:
                    csv_writer.writerow(row)

    def get_heading_to_target(self, current_pos, target_pos):
        """Compute the heading angle required to face the target waypoint."""
        dx, dy = target_pos[0] - current_pos[0], target_pos[1] - current_pos[1]
        return np.arctan2(dy, dx)

    def normalize_angle(self, angle):
        """Ensure angles stay within -π to π range."""
        return (angle + np.pi) % (2 * np.pi) - np.pi

    def GridFormation(self):
        center_lat = self.center_latitude
        center_lon = self.center_longitude

        num_rectangles = self.num_of_drones
        grid_spacing = self.grid_space
        meters_for_extended_lines = 250
        gap_between_rectangles = GAP_BETWEEN_RECTANGLES_M

        check_coverage_area(num_rectangles, grid_spacing, self.coverage_area)
        full_width, full_height = self.coverage_area, self.coverage_area

        total_gap_height = (num_rectangles - 1) * gap_between_rectangles
        available_height = full_height - total_gap_height
        rectangle_height = available_height / num_rectangles

        center_point = Point(center_lat, center_lon)

        csv_datas = []

        west_edge = distance(meters=full_width / 2).destination(center_point, 270)

        for i in range(num_rectangles):
            top_offset = (
                (i * rectangle_height) - (full_height / 2) + (rectangle_height / 2)
            )

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
                        east_point, 110
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
                        current_point, 240
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
            kml.save(self._grid_kml(i + 1))
            csv_datas.append(csv_data)

        minimum_waypoints = len(min(csv_datas, key=len))
        for i in range(len(csv_datas)):
            number_of_waypoints = 0
            path = self._grid_csv(i + 1)
            self.grid_path.append(path)
            with open(
                self._grid_csv(i + 1),
                mode="w",
                newline="",
            ) as file:
                for j in range(len(csv_datas[i])):
                    if number_of_waypoints < minimum_waypoints:
                        writer = csv.writer(file)
                        x, y = geoToCart(self.origin, 500000, csv_datas[i][j])
                        writer.writerow((x, y))
                    number_of_waypoints += 1
        # return 1

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

    def generate_bezier_curve(self):
        for num in range(self.num_of_drones):
            waypoints = []
            with open(self._grid_csv(num + 1), "r") as file:
                csv_reader = csv.reader(file)
                for row in csv_reader:
                    self.waypoints.append([float(row[0]), float(row[1])])
                    waypoints.append([float(row[0]), float(row[1])])

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
                    sampled_indices = np.linspace(
                        0, len(path1) - 1, 10, dtype=int
                    )  # Select 20 key points
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
            self.write_to_csv(result, num + 1)
            self.write_kml(result, num + 1)
        return self.path

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
            lat, lon = cartToGeo(self.origin, 500000, [cmd[0], cmd[1]])
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

    def plot_curve(self):
        import matplotlib.pyplot as plt

        for num in range(self.num_of_drones):
            predict_path = np.array(self.path[num])
            # sampled_points = np.array(self.sample_points)
            plt.figure(figsize=(8, 6))
            plt.plot(
                predict_path[:, 0],
                predict_path[:, 1],
                "r-",
                label="Predicted Path",
                linewidth=2,
            )
            plt.scatter(
                *zip(*self.waypoints),
                color="blue",
                s=100,
                label="Waypoints",
                marker="X",
            )

            #     plt.scatter(sampled_points[:, 0], sampled_points[:, 1], color='black', marker='o', s=40, label="Bézier Points")

            # # Annotate the sampled Bézier points with t-values
            #     for i, (x, y) in enumerate(sampled_points):
            #         plt.text(x, y, f'w', fontsize=10, verticalalignment='top', horizontalalignment='left')

            plt.xlabel("X Position (m)")
            plt.ylabel("Y Position (m)")
            plt.title("Aircraft Path Prediction with 40° Roll Limit")
            plt.legend()
            plt.grid()
            plt.axis("equal")

            # --- Ensure Plot Opens Correctly ---
            plt.show(block=True)  # Ensures the window stays open
