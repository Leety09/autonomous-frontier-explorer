# OccupancyGrid Class Definition
# File: occupancy_grid.py

import sys
import math
import pioneer_simpleproxsensors as psps
import pose

class OccupancyGrid:
    """
    A custom class to model a Bayesian Occupancy Grid Map.
    It handles map storage (log-odds), probability updates (sensor model),
    visualization (paint), and autonomous exploration logic (frontier detection).
    """

    # --------------------------------
    # Color Definitions (Hex values for Webots Display)
    # --------------------------------
    COLOR_UNKNOWN = 0x808080  # Gray: Areas not yet explored
    COLOR_FREE    = 0xFFFFFF  # White: Known empty space
    COLOR_WALL    = 0x000000  # Black: Known obstacles
    COLOR_ROBOT   = 0xFF0000  # Red: Robot position marker
    COLOR_GRID    = 0x666666  # Dark Gray: Grid lines for visualization

    # --------------------------------
    # Probability Parameters (Log Odds)
    # --------------------------------
    # We use Log-Odds to make probability updates additive (more stable/faster).
    # lprior = 0.0 corresponds to Probability = 0.5 (Unknown).
    lprior = 0.0

    # Log-odds when a cell is detected as Occupied (P approx 0.75)
    # A lower value (0.75 vs 0.99) reduces noise sensitivity.
    log_odds_occ = math.log(0.75/(1-0.75))

    # Log-odds when a cell is detected as Free (P approx 0.3-0.4)
    # Controls how fast the robot "erases" unknown areas.
    log_odds_free = math.log(0.4/(1-0.4))

    # --------------------------------
    # Sensor Model Parameters
    # --------------------------------
    # HALFALPHA: Half the thickness of a wall estimate (0.06m = 6cm).
    # Represents measurement uncertainty in distance.
    HALFALPHA = 0.06

    # HALFBETA: Half the opening angle of the sensor cone (15 degrees).
    # Represents measurement uncertainty in angle.
    HALFBETA = math.pi/12.0

    # Standard Pioneer P3-DX Sensor Angles (8 front-facing sonars)
    SENSOR_ANGLES = [
        math.radians(90),  math.radians(50),  math.radians(30),  math.radians(10),
        math.radians(-10), math.radians(-30), math.radians(-50), math.radians(-90)
    ]

    def __init__(self, robot, grid_scale, display_name, robot_pose, prox_sensors):
        """
        Initialize the grid map.
        :param robot: Webots Robot instance
        :param grid_scale: Resolution (cells per meter), e.g., 20
        :param display_name: Name of the Display device in Webots
        :param robot_pose: Initial pose object
        :param prox_sensors: Sensor wrapper instance
        """
        self.robot = robot
        self.robot_pose = robot_pose
        self.prox_sensors = prox_sensors
        self.radius = self.prox_sensors.get_radius()

        # Get Arena dimensions from the World Info (DEF ARENA)
        self.arena = robot.getFromDef("ARENA")
        if self.arena is None:
            print("Please define the RectangleArena DEF field as ARENA.", file=sys.stderr)
            return

        floorSize_field = self.arena.getField("floorSize")
        floorSize = floorSize_field.getSFVec2f()
        self.arena_width = floorSize[0]
        self.arena_height = floorSize[1]

        # Initialize the grid data structure
        # We use a 1D list to represent a 2D grid for performance.
        self.num_row_cells = int(grid_scale * self.arena_width)
        self.num_col_cells = int(grid_scale * self.arena_height)
        self.grid = [self.lprior]*(self.num_row_cells * self.num_col_cells)

        # Setup the Display device for visualization
        self.display = robot.getDevice(display_name)
        if self.display is not None:
            self.device_width = self.display.getWidth()
            self.device_height = self.display.getHeight()

            # Calculate scaling factors to fit the arena onto the display screen
            wsf = self.device_width / self.arena_width
            hsf = self.device_height / self.arena_height
            self.scalefactor = min(wsf, hsf)

            # Calculate pixel size of a single grid cell
            self.cell_width = int(self.device_width / self.num_row_cells)
            self.cell_height = int(self.device_height / self.num_col_cells)

            # Initialize screen with "Unknown" color
            self.display.setColor(self.COLOR_UNKNOWN)
            self.display.fillRectangle(0, 0, self.device_width, self.device_height)
        else:
            self.scalefactor = 0.0

    # --- Getters ---
    def get_num_row_cells(self):
        return self.num_row_cells
    def get_num_col_cells(self):
        return self.num_col_cells
    def get_grid_size(self):
        return len(self.grid)

    # --- Coordinate Transformation Helpers ---
    def scale(self, l):
        """ Scales physical meters to screen pixels """
        return int(l * self.scalefactor)

    def mapx(self, x):
        """ Maps world X coordinate to screen X coordinate (Center origin) """
        return int((self.device_width / 2.0) + self.scale(x))

    def mapy(self, y):
        """ Maps world Y coordinate to screen Y coordinate (Inverts Y axis) """
        return int((self.device_height / 2.0) - self.scale(y))

    def set_pose(self, p):
        """ Updates the internal robot pose """
        self.robot_pose.set_pose_position(p)

    def cell_probability(self, lodds):
        """ Converts Log-Odds value back to Probability (0.0 to 1.0) """
        try:
            exp = math.exp(lodds)
        except:
            exp = math.inf # Handle overflow
        return 1 - (1 / (1 + exp))

    # ---------------------------------------------------------------------------
    # Inverse Sensor Model
    # Calculates the update value for a specific cell (x, y) based on sensor data.
    # ---------------------------------------------------------------------------
    def inv_sensor_model(self, p, x, y):

        # 1. Calculate the position of the cell relative to the robot
        dx = x - p.x
        dy = y - p.y
        r = math.sqrt(dx**2 + dy**2) # Distance to cell

        # Calculate the angle (bearing) of the cell relative to the robot's heading
        cell_bearing = math.atan2(dy, dx) - p.theta
        # Normalize angle to [-pi, pi]
        while cell_bearing > math.pi: cell_bearing -= 2*math.pi
        while cell_bearing < -math.pi: cell_bearing += 2*math.pi

        # 2. Find which sensor covers this cell (Nearest Neighbor)
        best_sensor_id = -1
        min_angle_diff = float('inf')

        for i in range(8):
            sensor_angle = self.SENSOR_ANGLES[i]
            diff = abs(cell_bearing - sensor_angle)
            if diff < min_angle_diff:
                min_angle_diff = diff
                best_sensor_id = i

        # If a valid sensor is found
        if best_sensor_id != -1:
            z_k = self.prox_sensors.get_value(best_sensor_id) # Sensor reading
            z_max = self.prox_sensors.get_maxRange()          # Max range
            r_adjusted = r - self.radius                      # Adjust for robot body size

            # Check if the cell is within the sensor's Field of View (Cone)
            if min_angle_diff < self.HALFBETA:

                # A. Safety Filter: Ignore unreliable readings
                # Ignore readings that are too close (noise/self-reflection) or max range (no echo)
                if z_k > (z_max - 0.5) or z_k < 0.25:
                    return 0

                # B. Case 1: Occupied (The cell is at the obstacle distance)
                # If the cell distance (r) matches the sensor reading (z_k) within tolerance
                if abs(r_adjusted - z_k) <= self.HALFALPHA:
                    # Range Gating: Only draw walls that are close (< 2.2m).
                    # Distant measurements are too noisy and cause "ghost" obstacles.
                    if z_k < 2.2:
                        return self.log_odds_occ

                # C. Case 2: Free (The cell is closer than the obstacle)
                # If the cell is in front of the detected obstacle, it must be empty.
                # [CRITICAL FIX]: Added a 15cm buffer (z_k - 0.15).
                # This prevents the "Free" update from accidentally erasing a wall
                # if the sensor reading fluctuates slightly due to noise.
                elif r_adjusted < (z_k - 0.15):

                    # Range Gating: Only clear free space within 1.8m.
                    # Prevents clearing distant unexplored areas erroneously.
                    if r_adjusted < 1.8:
                        return self.log_odds_free

        # Default: No update (Unknown)
        return 0

    def map(self, p):
        """
        Main mapping function. Iterates through the grid and updates probabilities.
        Optimized to only check cells near the robot.
        """
        if self.arena is None: return

        # Pre-calculate coordinate conversion constants
        x_orig_offset = self.arena_width / 2
        y_orig_offset = self.arena_height / 2
        x_inc = self.arena_width / self.num_row_cells
        y_inc = self.arena_height / self.num_col_cells
        x_cell_offset = x_inc / 2
        y_cell_offset = y_inc / 2

        self.robot_pose.set_pose_position(p)

        # Iterate through all grid cells (In a real implementation, use a bounding box)
        for i in range(len(self.grid)):
            # Calculate world coordinates of the cell
            x = x_inc * int(i % self.num_row_cells) - x_orig_offset + x_cell_offset
            y = -(y_inc * int(i / self.num_row_cells) - y_orig_offset + y_cell_offset)

            # Optimization: Only update cells within 3.0 meters of the robot
            if abs(x - p.x) > 3.0 or abs(y - p.y) > 3.0: continue

            # Calculate update value using Inverse Sensor Model
            val = self.inv_sensor_model(self.robot_pose, x, y)

            # Apply update with saturation (Clamping)
            # Prevents probabilities from becoming 0 or 1 (infinite log-odds)
            if val != 0:
                self.grid[i] += val
                if self.grid[i] > 20: self.grid[i] = 20
                if self.grid[i] < -20: self.grid[i] = -20

    # ---------------------------------------------------------------------------
    # Frontier Exploration Logic
    # Returns the coordinates of the best "Unknown" area to explore next.
    # ---------------------------------------------------------------------------
    def get_frontier_target(self, current_pose):
        best_target = None

        # Geometry calculations
        x_orig_offset = self.arena_width / 2
        y_orig_offset = self.arena_height / 2
        x_inc = self.arena_width / self.num_row_cells
        y_inc = self.arena_height / self.num_col_cells

        # [Parameter] Boundary Margin
        # Ignore unknown cells within 0.2m of the physical map border.
        # This prevents the robot from trying to explore inside the arena walls.
        boundary_margin = 0.2

        candidates = []

        # 1. Identify Candidate Frontiers
        for i in range(len(self.grid)):
            # Check if cell is "Unknown" (Probability approx 0.5)
            prob = self.cell_probability(self.grid[i])
            if 0.45 < prob < 0.55:

                # Calculate world coordinates
                r = int(i % self.num_row_cells)
                c = int(i / self.num_row_cells)
                x = x_inc * r - x_orig_offset + (x_inc/2)
                y = -(y_inc * c - y_orig_offset + (y_inc/2))

                # Filter: Ignore if too close to the map boundary
                if abs(x) > (self.arena_width/2 - boundary_margin): continue
                if abs(y) > (self.arena_height/2 - boundary_margin): continue

                # Check Neighbors: A valid frontier must be adjacent to "Known Free" space.
                # This ensures the robot can actually reach the target.
                has_free_neighbor = False
                neighbors = [i-1, i+1, i-self.num_row_cells, i+self.num_row_cells]
                for n in neighbors:
                    if 0 <= n < len(self.grid):
                        # Check if neighbor is Free (Prob < 0.35)
                        if self.cell_probability(self.grid[n]) < 0.35:
                            has_free_neighbor = True
                            break

                if has_free_neighbor:
                    dist = math.sqrt((x - current_pose.x)**2 + (y - current_pose.y)**2)
                    candidates.append((dist, x, y))

        # 2. Select the Best Candidate
        # Strategy: Prioritize medium-range targets to encourage cross-map movement.

        best_target = None
        min_dist_score = float('inf')
        found_far_target = False

        for (dist, x, y) in candidates:
            # Filter extremely close noise (< 0.8m)
            if dist < 0.8: continue

            # Priority Range: 1.5m to 5.0m
            # This encourages the robot to drive to the other side of the room.
            if 1.5 < dist < 5.0:
                if dist < min_dist_score:
                    min_dist_score = dist
                    best_target = pose.Pose(x, y, 0)
                    found_far_target = True

        # Fallback Strategy:
        # If no distant targets exist (e.g., room is mostly explored or robot is cornered),
        # accept ANY valid target that is not directly under the robot (> 0.25m).
        if not found_far_target:
            min_dist_score = float('inf')

            for (dist, x, y) in candidates:
                if dist > 0.25 and dist < min_dist_score:
                    min_dist_score = dist
                    best_target = pose.Pose(x, y, 0)

        return best_target

    def save_map(self, filename):
        """ Placeholder to save the map (Implementation depends on library) """
        print(f"Map saved to {filename}")

    # ---------------------------------------------------------------------------
    # Paint Method (Visualization)
    # Uses a layered approach: Background -> Map Content -> Grid Lines -> Robot -> Text
    # ---------------------------------------------------------------------------
    def paint(self):
        if self.arena is None or self.display is None: return

        # 1. Clear background
        # Fills the screen with light gray (representing "Unknown" areas)
        self.display.setColor(0xF0F0F0)
        self.display.fillRectangle(0, 0, self.device_width, self.device_height)

        self.coverage = 0.0

        # Calculate scaling/dimensions
        x_orig_offset = self.arena_width / 2
        y_orig_offset = self.arena_height / 2
        x_inc = self.arena_width / self.num_row_cells
        y_inc = self.arena_height / self.num_col_cells

        # Calculate block size for drawing cells
        # +1 ensures there are no gaps between blocks due to rounding
        block_w = int(x_inc * self.scalefactor) + 1
        block_h = int(y_inc * self.scalefactor) + 1

        # ---------------------------------------------------------
        # Layer 1: Draw Map Content (Walls and Free Space)
        # ---------------------------------------------------------
        for i in range(len(self.grid)):
            p = self.cell_probability(self.grid[i])

            # Optimization: Skip drawing "Unknown" cells (let background show)
            if 0.45 < p < 0.55: continue

            # Calculate screen coordinates for the center of the cell
            # Uses precise world-to-screen mapping to ensure alignment with the robot
            r = int(i % self.num_row_cells)
            c = int(i / self.num_row_cells)
            wx = x_inc * r - x_orig_offset + (x_inc/2)
            wy = -(y_inc * c - y_orig_offset + (y_inc/2))
            sx = self.mapx(wx)
            sy = self.mapy(wy)

            # Adjust to top-left corner for rectangle drawing
            draw_x = sx - (block_w // 2)
            draw_y = sy - (block_h // 2)

            # Determine Color based on Probability
            if p > 0.8:
                self.display.setColor(self.COLOR_WALL) # High prob = Black Wall
                self.coverage += 1.0
            elif p > 0.6:
                self.display.setColor(0x555555)        # Med prob = Dark Gray
            elif p < 0.2:
                self.display.setColor(self.COLOR_FREE) # Low prob = White Space
                self.coverage += 1.0
            elif p < 0.4:
                self.display.setColor(0xDDDDDD)        # Med-Low prob = Light Gray
            else:
                continue

            self.display.fillRectangle(draw_x, draw_y, block_w, block_h)

        # ---------------------------------------------------------
        # Layer 2: Draw Grid Lines
        # ---------------------------------------------------------
        # We use drawLine on top of the map for sharp, crisp lines.
        # Color is set to a medium gray to be visible on both black and white.
        self.display.setColor(0xAAAAAA)

        # Draw Vertical lines
        for col in range(self.num_row_cells + 1):
            x = int(col * self.cell_width)
            self.display.drawLine(x, 0, x, self.device_height)

        # Draw Horizontal lines
        for row in range(self.num_col_cells + 1):
            y = int(row * self.cell_height)
            self.display.drawLine(0, y, self.device_width, y)

        # ---------------------------------------------------------
        # Layer 3: Draw Robot
        # ---------------------------------------------------------
        self.display.setColor(self.COLOR_ROBOT)
        rx, ry = self.mapx(self.robot_pose.x), self.mapy(self.robot_pose.y)
        self.display.fillOval(rx, ry, self.scale(self.radius), self.scale(self.radius))

        # Draw heading vector (Green line indicating where robot is facing)
        head_x = rx + math.cos(self.robot_pose.theta) * 6
        head_y = ry - math.sin(self.robot_pose.theta) * 6
        self.display.setColor(0x00FF00)
        self.display.drawLine(rx, ry, int(head_x), int(head_y))

        # ---------------------------------------------------------
        # Layer 4: Text Overlay (Coverage Info)
        # ---------------------------------------------------------
        self.coverage = self.coverage / len(self.grid)

        # Draw text background for readability
        self.display.setColor(self.COLOR_WALL)
        self.display.setFont("Arial", 10, True)
        self.display.fillRectangle(0, self.device_height-18, 100, 18)

        # Draw text
        self.display.setColor(0xFFFFFF)
        self.display.drawText(f"Cov: {self.coverage * 100:.1f}%", 5, self.device_height-14)
