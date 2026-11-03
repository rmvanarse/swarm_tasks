from swarm_tasks.simulation.simulation import Simulation
import swarm_tasks.envs as envs

import matplotlib.patches as patches
from matplotlib import pyplot as plt
from matplotlib import animation
from matplotlib.lines import Line2D
import numpy as np
from shapely.geometry import Point

# Both trails (sim bot path, real drone track) show only the most
# recent stretch of travel, not the whole mission's history -- an
# unbounded trail would grow these lists (and the line drawn from
# them) forever over a long-running mission, both a memory/perf
# concern and visual clutter right next to the current position.
TRAIL_MAX_LENGTH_M = 100.0


class Gui:
	def __init__(self, sim,isGui=False):
		self.sim = sim
		self.size = sim.size
		self.env_name = sim.env_name
		#self.ax = plt.axes()
		#self.fig = plt.gcf()
		self.fig, self.ax = plt.subplots(figsize=(10,10))
		self.ax.set_ylim([0, sim.size[1]])
		self.ax.set_xlim([0, sim.size[0]])

		#self.state_colors = ['blue','green','red','orange', 'purple']
		self.state_colors = ['b','g','r','c', 'm','y','purple','orange','k','brown','lime','pink','teal','gold','gray','violet']
		self.grid_scatter = None

		#Contents list (used in remove_artists())
		self.content_fills = []
		self.limits = ([0,0,self.size[0], self.size[0]], [0,self.size[1], self.size[1],0])
		self.area_covered=0
		self.search_time=0
		self.coverage_text = None
		self.coverage_text1 = None
		self.uav_trajectories=[]
		self.uav_positions = {'uav_1': [], 'uav_2': [], 'uav_3': [], 'uav_4': [], 'uav_5': [], 'uav_6':[], 'uav_7':[], 'uav_8':[]}
		self.trail_lines = []  # sim BOT path (solid) -- drawn by show_bots()
		self.trail_x = []
		self.trail_y = []
		# Real DRONE track (dashed) -- drawn by show_gps_positions(), kept
		# separate from gps_artists below: gps_artists is the current-
		# position '+' marker/heading arrow, fully rebuilt every call
		# (see remove-and-redraw in show_gps_positions), while these
		# lines must persist and only grow, exactly like trail_lines
		# above -- that's what makes it a track instead of a dot.
		self.drone_trail_lines = []
		self.drone_trail_x = []
		self.drone_trail_y = []
		self.goal_artists = []
		# Planned route lines: list of (set of bot indices, line artist)
		self.route_artists = []
		self.gps_artists = []
		self.lookahead_artists = []
		self.bot_artists = []  # bot circles + heading arrows drawn by show_bots()
		self.isGui = isGui
		self._legend_drawn = False

	@staticmethod
	def _append_trail_point(xs, ys, x, y, max_length_m=TRAIL_MAX_LENGTH_M):
		"""Appends (x,y) to the trail lists xs/ys IN PLACE, then trims
		from the front so the trail never represents more than
		max_length_m of actual travel -- walks backward from the
		newest point accumulating segment lengths, and drops every
		older point once that running total passes max_length_m.
		"""
		xs.append(x)
		ys.append(y)
		total = 0.0
		keep_from = 0
		for i in range(len(xs) - 1, 0, -1):
			total += ((xs[i] - xs[i-1]) ** 2 + (ys[i] - ys[i-1]) ** 2) ** 0.5
			if total > max_length_m:
				keep_from = i
				break
		if keep_from > 0:
			del xs[:keep_from]
			del ys[:keep_from]

	def show_bots(self):
		if not self.isGui: return
		# bot.size is the real collision radius used by the simulation logic
		# (often as small as 0.4) -- on a world sized in real ground units
		# (e.g. a fence loaded from swarm_env can be thousands of units wide)
		# a circle that small is sub-pixel and invisible. vis_r is a
		# draw-only radius scaled to the current world so bots stay visible
		# regardless of environment size; it never feeds back into sim state.
		vis_r = max(self.size) * 0.0009

		if not self.trail_lines:
			for i in range(len(self.sim.swarm)):
				color = self.state_colors[i % len(self.state_colors)]
				line, = self.ax.plot([], [], '-', color=color, linewidth=1, alpha=0.6)
				self.trail_lines.append(line)
				self.trail_x.append([])
				self.trail_y.append([])

		#show bots
		for i,bot in enumerate(self.sim.swarm):
			#self.show_neighbourhood(bot,3)

			x,y,theta = bot.get_pose()
			bot_color = self.state_colors[i % len(self.state_colors)]

			self._append_trail_point(self.trail_x[i], self.trail_y[i], x, y)
			self.trail_lines[i].set_data(self.trail_x[i], self.trail_y[i])

			circle = plt.Circle((x,y), vis_r, color=bot_color, fill=True)
			self.fig.gca().add_artist(circle)
			self.bot_artists.append(circle)

			l=vis_r*0.4
			arrow = self.ax.arrow(x,y, \
				(vis_r-l)*np.cos(theta), (vis_r-l)*np.sin(theta), \
				head_width=l, head_length=l, \
				fc='k', ec='k', zorder=50)
			self.bot_artists.append(arrow)

	def show_goals(self, goals):
		"""goals: list of (x,y) target points, one per bot. Pass the same
		list across frames (only overwriting entries as new goals are
		computed) rather than rebuilding it each frame, so a bot with no
		goal yet (None) simply has no marker instead of flickering."""
		if not self.isGui: return
		for artist in self.goal_artists:
			artist.remove()
		self.goal_artists = []
		for i, goal in enumerate(goals):
			if goal is None:
				continue
			gx, gy = goal
			color = self.state_colors[i % len(self.state_colors)]
			marker, = self.ax.plot(gx, gy, marker='x', markersize=5, markeredgewidth=2, color=color)
			self.goal_artists.append(marker)

	def clear_routes(self, bot_indices=None):
		"""Removes the planned route lines of these bots (every bot if None).
		A route shared by several bots goes away when any of them is cleared."""
		if not self.isGui: return
		wanted = None if bot_indices is None else set(bot_indices)
		kept = []
		for owners, line in self.route_artists:
			if wanted is None or owners & wanted:
				try:
					line.remove()
				except (ValueError, NotImplementedError):
					pass
			else:
				kept.append((owners, line))
		self.route_artists = kept

	def show_route(self, bot_indices, xs, ys):
		"""Draws the planned waypoint route for these bots, replacing whatever
		route they had before, so only the newest route per bot stays on the
		plot. Routes of other bots are left alone."""
		if not self.isGui: return
		bot_indices = list(bot_indices)
		self.clear_routes(bot_indices)
		color = self.state_colors[bot_indices[0] % len(self.state_colors)]
		line, = self.ax.plot(xs, ys, 'x--', color=color, markersize=8, mew=2, alpha=0.6)
		self.route_artists.append((set(bot_indices), line))

	def show_gps_positions(self, points, headings=None):
		"""Plot each bot's real (live GPS-derived) position, distinct from
		the simulated bot circle drawn by show_bots(), so simulated vs real
		position can be compared visually during a search operation.

		points: list of (x, y) in the same local sim frame as s.swarm[i].x/y
		(already converted from lat/lon by the caller), one entry per bot,
		or None where no live GPS fix is available yet.

		The '+' marker and heading arrow show the CURRENT position only --
		they're removed and redrawn fresh every call (gps_artists, below).
		The dashed line is the real drone's TRACK: every call appends this
		point and extends it, the same way show_bots()'s solid trail_lines
		grow for the sim bot -- so the two can be compared on the plot
		(dashed drone track vs. solid bot path) to see whether they're
		actually flying the same route.

		headings: optional list of heading angles (radians, math convention:
		0 = +x axis, counter-clockwise -- see compass_to_math_rad()) matched
		to `points`, one per bot. When given, draws an arrow from the live
		GPS marker showing the real drone's current heading. Use
		matplotlib's annotate() rather than ax.arrow() here: ax.arrow()
		returns a FancyArrow, the exact class update()/remove_artists()
		sweeps every frame to redraw the sim-bot heading arrows -- since
		this is called before update() in gui_tick(), a FancyArrow drawn
		here would be erased before it ever got painted. annotate()'s arrow
		is a different artist class, so it survives that sweep and is
		cleaned up by this method's own gps_artists tracking instead.
		"""
		if not self.isGui: return

		if not self.drone_trail_lines:
			for i in range(len(points)):
				color = self.state_colors[i % len(self.state_colors)]
				line, = self.ax.plot([], [], '--', color=color, linewidth=1.2, alpha=0.6, zorder=59)
				self.drone_trail_lines.append(line)
				self.drone_trail_x.append([])
				self.drone_trail_y.append([])

		for artist in self.gps_artists:
			artist.remove()
		self.gps_artists = []
		arrow_len = max(self.size) * 0.02
		for i, point in enumerate(points):
			if point is None:
				continue
			gx, gy = point
			color = self.state_colors[i % len(self.state_colors)]

			if i < len(self.drone_trail_lines):
				self._append_trail_point(self.drone_trail_x[i], self.drone_trail_y[i], gx, gy)
				self.drone_trail_lines[i].set_data(self.drone_trail_x[i], self.drone_trail_y[i])

			marker, = self.ax.plot(
				gx, gy, marker='+', markersize=10, markeredgewidth=2,
				color=color, zorder=60,
			)
			self.gps_artists.append(marker)

			heading = headings[i] if headings else None
			if heading is not None:
				arrow = self.ax.annotate(
					'', xy=(gx + arrow_len * np.cos(heading), gy + arrow_len * np.sin(heading)),
					xytext=(gx, gy),
					arrowprops=dict(arrowstyle='-|>', color=color, lw=2),
					zorder=61,
				)
				self.gps_artists.append(arrow)

	def show_lookahead_targets(self, points):
		"""Plot the tx,ty target compute_lookahead_target() hands to
		send_reposition() each tick, one per bot, so what the real vehicle
		is actually being commanded to fly toward is visible alongside its
		live GPS position (show_gps_positions) and the bot's own goal
		(show_goals).

		points: list of (x, y) in the same local sim frame as
		s.swarm[i].x/y, one entry per bot, or None where no target was
		computed this tick. Current target only -- no trail, redrawn fresh
		every call, same pattern as show_gps_positions.
		"""
		if not self.isGui: return
		for artist in self.lookahead_artists:
			artist.remove()
		self.lookahead_artists = []
		for i, point in enumerate(points):
			# swarm.py's _guide_and_reposition() returns (None, None) --
			# not a bare None -- for a bot whose tick was skipped, so
			# check the components too rather than just `point is None`.
			if point is None or point[0] is None or point[1] is None:
				continue
			tx, ty = point
			color = self.state_colors[i % len(self.state_colors)]
			marker, = self.ax.plot(
				tx, ty, marker='*', markersize=12, markeredgewidth=1,
				markeredgecolor='k', color=color, zorder=62,
			)
			self.lookahead_artists.append(marker)

	def show_env(self):
		if not self.isGui: return
		for obs in self.sim.env.obstacles:
			x, y = obs.exterior.xy
			self.ax.fill(x, y, fc='gray', alpha=0.9)
			if(self.env_name=="rectangles1"):
				x_coords = [int(i) for i in x]
				y_coords = [int(i) for i in y]

				# Set the grid cells inside the obstacle to True
				for i in range(max(0, min(x_coords)), min(self.sim.size[0], max(x_coords))):
					for j in range(max(0, min(y_coords)), min(self.sim.size[1], max(y_coords))):
						if 0 <= i < self.sim.size[1] and 0 <= j < self.sim.size[0]:
							if obs.contains(Point(i, j)):  # Check if the point is inside the polygon
								self.sim.grid[j, i] = True

	'''
	def show_env(self):
		for obs in self.sim.env.obstacles:
			x,y = obs.exterior.xy
			self.ax.fill(x,y, fc='gray', alpha=0.9)
			
			x_coords = [int(i) for i in x]
			y_coords = [int(i) for i in y]

			# Set the grid cells inside the obstacle to True
			for i in range(max(0, min(x_coords)), min(self.sim.size[0], max(x_coords) )):
		    		for j in range(max(0, min(y_coords)), min(self.sim.size[1], max(y_coords) )):
		    			if obs.contains(Point(i, j)):  # Check if the point is inside the polygon
		    				self.sim.grid[j, i] = True
			    				
			
			x_coords = [int(i) for i in x]
			y_coords = [int(i) for i in y]

			# Set the grid cells inside the obstacle to True
			for i in range(self.sim.size[0]):
			    for j in range(self.sim.size[1]):
			    	if (
				    min(x_coords) >= i >= max(x_coords)
				    and min(y_coords) >= j >= max(y_coords)
				    and obs.contains(Point(i, j))):
				    	self.sim.grid[j, i] = True
	'''

	def show_contents(self):
		if not self.isGui: return
		for item in self.sim.contents.items:
			x,y = item.polygon.exterior.xy
			if item.subtype == 'contamination':
				self.ax.fill(x,y, fc='red', alpha=0.5)
				continue
			elif item.subtype == 'nest':
				self.ax.fill(x,y, fc='purple', alpha =0.3)
				continue
			self.ax.fill(x,y, fc='orange', alpha=0.9)
			#self.content_fills.append((x,y))
	

	def update(self):
		"""

		"""
		if not self.isGui: return
		self.remove_artists()
		#self.show_bots()
		if self.sim.has_item_moved:
			#for ext in self.content_fills:
				#self.ax.fill(*ext, fc='w')
			#self.content_fills=[]
			#self.ax.fill(*self.limits, 'w')
			self.show_contents()
			self.show_env()
			self.sim.has_item_moved = False
		self.show_bots()
		if not self._legend_drawn:
			self._draw_legend()
			self._legend_drawn = True
		if(self.env_name=="rectangles1"):
			self.show_coverage(self.area_covered,self.search_time)
		# Draw THIS Gui's own figure/canvas directly, instead of
		# plt.pause() -- which redraws whatever pyplot's GLOBAL
		# "current figure" happens to be. _reset_gui() closes and
		# recreates a brand new Gui (and figure) every time a mission
		# starts; once that global pointer drifts, plt.pause() can end
		# up pumping a figure that isn't the one actually on screen,
		# leaving the real window frozen white even though the tick
		# loop underneath is still running fine. draw_idle() +
		# flush_events() target this canvas specifically, so they
		# can't go stale that way.
		self.fig.canvas.draw_idle()
		self.fig.canvas.flush_events()

	def _draw_legend(self):
		"""One-time legend naming what each marker/line SHAPE means.
		Color already encodes WHICH bot (state_colors, shared across
		that bot's goal/bot-circle/drone-marker/lookahead-star), so
		these proxy handles are neutral-colored and only distinguish
		shape -- drawn once; matplotlib keeps the legend on the axes
		on its own, it isn't part of remove_artists()'s per-frame sweep.
		"""
		handles = [
			Line2D([], [], color='k', marker='o', linestyle='-', markersize=6,
				label='Bot (sim) + path'),
			Line2D([], [], color='k', marker='+', linestyle='--', markersize=8,
				markeredgewidth=2, label='Drone (real GPS) + track'),
			Line2D([], [], color='k', marker='x', linestyle='None', markersize=6,
				markeredgewidth=2, label='Goal'),
			Line2D([], [], color='k', marker='*', linestyle='None', markersize=9,
				label='Lookahead target'),
		]
		self.ax.legend(handles=handles, loc='upper left', fontsize=8, framealpha=0.8)

	def remove_artists(self):
		"""
		Previous figures need to be removed before updating positions on GUI

		Bot circles/arrows are removed via self.bot_artists (populated by
		show_bots() every frame) rather than ax.findobj(Circle)/(FancyArrow),
		which walks and isinstance-checks every artist on the axes each
		call -- direct removal from the list we already built is O(num
		bots) instead of O(everything on the axes), every single frame.
		"""
		if not self.isGui: return
		for obj in self.bot_artists:
			obj.remove()
		self.bot_artists = []

		if self.sim.has_item_moved:
			for obj in self.ax.findobj(patches.Polygon):
				obj.remove()


	def show_neighbourhood(self, bot, r=None):
		#Shows the neighbourhood of a robot
		if not self.isGui: return
		x,y = bot.get_position()
		if r == None:
			r = bot.neighbourhood_radius
		circle2 = plt.Circle((x,y), r, color='red', fill=True, alpha=0.1)
		circle1 = plt.Circle((x,y), r/2+bot.size, color='red', fill=True, alpha=0.1)
		self.fig.gca().add_artist(circle1)
		self.fig.gca().add_artist(circle2)


	def show_grid(self):
		"""
		Args:
			grid: A 2D array of values from 0-1 
		
		TODO:
		Plotting the grid as an image takes high computation
		Plot in another format
		"""
		if not self.isGui: return
		if self.grid_scatter != None:
			self.grid_scatter.remove()

		x_size, y_size = self.sim.size
		x_labels = [i +0.5 for i in range(x_size)]*y_size
		y_labels = [int(i/x_size)+0.5 for i in range(y_size*x_size)]
		
		grid_1d = self.sim.grid.ravel()
		#grid_1d[20:40] = True	#debug

		col = ['green' if i else 'yellow' for i in grid_1d]


		if len(col) != len(x_labels) or len(x_labels) != len(y_labels):
			print("ERROR: Grid shape mismatch")
			return False

		self.grid_scatter = self.ax.scatter(x_labels, y_labels, c=col)
		#plt.draw()

	def show_coverage(self, area_covered,search_time):
		if not self.isGui: return
		self.area_covered=area_covered
		self.search_time=search_time
		#self.elapsed_time_value = elapsed_time
		timer_text = f"Time: {int(self.search_time // 60):02d}:{int(self.search_time % 60):02d}"
		if self.coverage_text is not None:
			self.coverage_text.remove()			
			#if hasattr(self, 'elapsed_time_text'):
			 #   self.elapsed_time.remove()
		if self.coverage_text1 is not None:
			self.coverage_text1.remove()	
		# Adjust the coordinates as needed for the position of the text box
		text_box_props = dict(boxstyle='round', facecolor='wheat', alpha=0.5)
		label_text = "Area Covered ="
		self.coverage_text = self.ax.text(1, 1.05, f"{label_text} {area_covered}", transform=self.ax.transAxes,
		                  fontsize=10, verticalalignment='top', horizontalalignment='right', bbox=text_box_props)
            
            
		text_box_props1 = dict(boxstyle='round', facecolor='wheat', alpha=0.5)
		label_text1 = "Time ="
		self.coverage_text1 = self.ax.text(1, 1.09, f"{timer_text} ", transform=self.ax.transAxes,
		                  fontsize=10, verticalalignment='top', horizontalalignment='right', bbox=text_box_props1)
        
		return area_covered
	def run(self):
		if not self.isGui: return
		plt.show(block=False)
		# Force the first real paint right away -- without this the
		# window can sit blank/white until some later update() call
		# happens to land on it correctly.
		self.fig.canvas.draw()
		self.fig.canvas.flush_events()

	def close(self):
		if not self.isGui: return
		#Close THIS Gui's figure specifically: bare plt.close() closes
		#whichever figure happens to be current, which may belong to
		#something else entirely once more than one has been created.
		plt.close(self.fig)
		
