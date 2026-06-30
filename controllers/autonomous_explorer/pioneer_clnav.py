# pioneer_clnav.py
# Updated: Robust Loop Detection & Compact Map Navigation
# This class implements a Tangent Bug algorithm with PID-controlled wall following.

import math
import pose
import pioneer_simpleproxsensors as psps

class PioneerCLNav:
    # -------------------------------------------------------------------------
    # State Constants
    # -------------------------------------------------------------------------
    GOALSEEKING = 0     # Robot is driving straight towards the goal
    WALLFOLLOWING = 1   # Robot is navigating around an obstacle
    STUCK_RECOVERY = 99 # Robot is physically stuck and attempting to free itself

    # Sub-states for Wall Following (Tangent Bug Logic)
    WF_HIT = 0      # Just hit a wall, initialize wall following
    WF_SEARCH = 1   # Moving along the wall searching for a clear path
    WF_LEAVE = 2    # (Not used in this simplified version, kept for compatibility)
    WF_ROTATE = 3   # Rotating towards the goal after clearing the wall

    # -------------------------------------------------------------------------
    # Tuning Parameters
    # -------------------------------------------------------------------------
    WF_VEL = 1.5         # Standard velocity for wall following

    # [Tuning] Wall Following Distance (Meters)
    # Set to 0.2m (20cm) for compact maps. This allows the robot to squeeze
    # through narrow gaps, though it requires precise turning.
    WF_DIST = 0.2

    GOAL_RADIUS = 0.2    # Distance to target to consider it "reached"
    GOAL_VEL = 1.5       # Default velocity
    DECEL_DIST = 0.8     # Distance to start slowing down when approaching goal
    ACCEL_ANGLE = 0.5    # Angle threshold for turning speed adjustment

    def __init__(self, robot, prox_sensors):
        """
        Initialize the Navigation Controller.
        :param robot: The Webots Robot instance.
        :param prox_sensors: The wrapper class for proximity sensors.
        """
        self.robot = robot
        self.robot_node = self.robot.getSelf()
        self.pps = prox_sensors

        # Navigation Goals
        self.goal = self.get_real_pose()
        self.start = self.get_real_pose()
        self.state = self.GOALSEEKING

        # PID Controller Variables (Proportional-Integral-Derivative)
        self.prev_error = 0
        self.total_error = 0
        self.pid_counter = 0
        self.pid_diff = 0.0

        # Tangent Bug State Variables
        self.wf_state = self.WF_HIT
        self.hit_point = pose.Pose(0,0,0) # Where we first hit the wall

        # [New] Loop Detection Variable
        # Tracks the maximum distance the robot has traveled away from the hit point.
        # This prevents false positives where the robot thinks it looped immediately after starting.
        self.max_hit_dist = 0.0

        # Safety & Recovery Counters
        self.reverse_counter = 0  # Timer for reversing
        self.stuck_counter = 0    # Timer for detecting if stuck

        # Motor Initialization
        self.left_motor = self.robot.getDevice('left wheel')
        self.right_motor = self.robot.getDevice('right wheel')
        # Set to velocity control mode
        self.left_motor.setPosition(float('inf'))
        self.right_motor.setPosition(float('inf'))
        self.max_vel = self.left_motor.getMaxVelocity()

        # Bumper / Touch Sensor (Optional)
        self.bumper = robot.getDevice("touch sensor")
        if self.bumper:
            self.bumper.enable(int(robot.getBasicTimeStep()))
        else:
            self.bumper = None

        self.stop()

    # --- Hardware Abstraction Methods ---

    def set_velocity(self, lv, rv):
        """ Sets the velocity of left and right motors. """
        self.left_motor.setVelocity(lv)
        self.right_motor.setVelocity(rv)

    def range_to_frontobstacle(self):
        """ Returns the minimum distance detected by front sensors. """
        return min(self.pps.get_maxRange(),
                   self.pps.get_value(1), self.pps.get_value(2),
                   self.pps.get_value(3), self.pps.get_value(4),
                   self.pps.get_value(5), self.pps.get_value(6))

    def range_to_leftobstacle(self):
        """ Returns distance from the left-side sensor (used for wall following). """
        return self.pps.get_value(0)

    def get_real_pose(self):
        """ Gets the robot's ground truth position (x, y, theta) from Webots. """
        if self.robot_node is None: return pose.Pose(0,0,0)
        real_pos = self.robot_node.getPosition()
        rot = self.robot_node.getOrientation()
        theta = math.atan2(-rot[0], rot[3])

        # Convert coordinate systems if necessary
        halfpi = math.pi / 2
        theta2 = theta + halfpi
        if (theta > halfpi): theta2 = -(3 * halfpi) + theta

        return pose.Pose(real_pos[0], real_pos[1], theta2)

    # --- Math Helpers ---

    def distance_to_m_line(self, p):
        """ Calculates perpendicular distance from current position to the 'M-Line' (Start->Goal line). """
        x0, y0 = p.x, p.y
        x1, y1 = self.start.x, self.start.y
        x2, y2 = self.goal.x, self.goal.y
        numerator = abs((y1 - y2) * x0 + (x2 - x1) * y0 + x1 * y2 - x2 * y1)
        denominator = math.sqrt((y1 - y2)**2 + (x2 - x1)**2)
        if denominator == 0: return 0
        return numerator / denominator

    def pid(self, error):
        """
        PID Controller for Wall Following.
        Adjusts steering based on the error between current distance and desired wall distance.
        """
        kp = 8.0   # Proportional gain (reacts to current error)
        kd = 40.0  # Derivative gain (reacts to rate of change, dampens oscillation)
        ki = 0.0   # Integral gain (not used here)

        prop = error
        self.total_error += error

        # Calculate derivative every 3 steps to smooth out noise
        if (self.pid_counter % 3) == 0:
            self.pid_diff = error - self.prev_error
            self.prev_error = error
        self.pid_counter += 1

        return (kp * prop) + (ki * self.total_error) + (kd * self.pid_diff)

    def rotate(self, bearing, velocity):
        """ Rotates the robot in place to face a specific bearing. """
        if abs(bearing) < 0.1: return True # Target angle reached
        rotate_vel = velocity
        if bearing < 0: rotate_vel = -rotate_vel
        self.set_velocity(-rotate_vel, rotate_vel)
        return False

    def adjust_velocity(self, bearing, velocity):
        """ Adjusts wheel velocities to drive towards a bearing while maintaining forward momentum. """
        rotate_vel = min(self.max_vel, abs(bearing * self.max_vel / self.ACCEL_ANGLE))
        if bearing < 0: rotate_vel = -rotate_vel
        lv = min(self.max_vel, velocity - rotate_vel)
        rv = min(self.max_vel, velocity + rotate_vel)
        self.set_velocity(lv, rv)

    # -------------------------------------------------------------------------
    # Core Logic: Wall Following
    # -------------------------------------------------------------------------
    def wall_following(self, p):
        front_dist = self.range_to_frontobstacle()

        # 1. Emergency Obstacle Avoidance
        # If too close to a wall in front (< 0.35m), reverse/spin to unstick.
        if front_dist < 0.35:
            self.set_velocity(2.0, -2.0)
            self.stuck_counter += 1
            if self.stuck_counter > 30:
                 self.reverse_counter = 15
            return

        # Threshold to detect an inner corner (wall directly ahead)
        CORNER_THRESHOLD = self.WF_DIST + 0.15

        # 2. Inner Corner Logic (Turning Right)
        if front_dist < CORNER_THRESHOLD:
            # We are approaching a wall head-on.
            # In compact maps, an in-place rotation is safest to avoid hitting the wall while turning.
            self.set_velocity(1.0, -1.0)
            self.stuck_counter = 0
        else:
            # 3. Straight Wall / Outer Corner Logic
            left_dist = self.range_to_leftobstacle()
            max_range = self.pps.get_maxRange()

            # Case A: Wall is detected on the left
            if left_dist < (max_range - 0.1):
                # Use PID to maintain WF_DIST
                error = left_dist - self.WF_DIST
                control = self.pid(error)
                control = max(min(control, self.WF_VEL), -self.WF_VEL) # Clamp control value
                self.set_velocity(self.WF_VEL, self.WF_VEL + control)
            else:
                # Case B: Wall on the left disappeared (Outer Corner)
                # ---------------------------------------------------------
                # [Optimization for Compact Maps]
                # ---------------------------------------------------------
                # We use a "Medium Radius Turn" (0.5x Left, 1.0x Right).
                # - Logic: Drive forward while turning left.
                # - Why? A sharp turn (0.4x) hits the inner corner. A wide turn (0.9x) hits the opposite wall.
                # - 0.5x is the "Golden Mean" for 0.22m wall distance.
                self.set_velocity(self.WF_VEL * 0.5, self.WF_VEL * 1.0)

            self.stuck_counter = 0

    # -------------------------------------------------------------------------
    # Core Logic: Tangent Bug (High Level Decisions)
    # -------------------------------------------------------------------------
    def tangent_bug(self, p):
        dist_to_goal = p.get_range(self.goal)

        # Update Loop Detection Metric
        dist_from_hit = p.get_range(self.hit_point)
        if dist_from_hit > self.max_hit_dist:
            self.max_hit_dist = dist_from_hit

        # State: Just hit the wall, initialize
        if self.wf_state == self.WF_HIT:
            if dist_from_hit > 0.5:
                self.wf_state = self.WF_SEARCH
            self.wall_following(p)
            return False

        # State: Searching along the wall
        if self.wf_state == self.WF_SEARCH:

            # ---------------------------------------------------------
            # [Loop Detection] - Detecting Unreachable Goals
            # ---------------------------------------------------------
            # We assume a loop (island) if:
            # 1. We are back near the start point (< 0.6m)
            # 2. BUT we have traveled far enough away (> 1.5m) to ensure it's not just initial jitter.
            if dist_from_hit < 0.6 and self.max_hit_dist > 1.5:
                print("Loop detected (Confirmed). Target unreachable.")
                self.state = self.GOALSEEKING
                self.stop()
                return True # Return True to signal controller to pick a new target

            bearing = p.get_bearing(self.goal)
            front_dist = self.range_to_frontobstacle()

            # Check if the path to the goal is clear (Leave Condition 1)
            # - Facing roughly towards goal
            # - No obstacle in front (limited by goal distance)
            path_clear = abs(bearing) < 0.4 and front_dist > min(dist_to_goal, 1.2)

            if path_clear:
                self.wf_state = self.WF_ROTATE
                return False

            # Check M-Line (Leave Condition 2)
            # - We crossed the M-Line (line from start to goal)
            # - And we are closer to the goal than when we started wall following
            dist_to_line = self.distance_to_m_line(p)
            hit_dist_to_goal = self.hit_point.get_range(self.goal)

            if dist_to_line < 0.2 and dist_to_goal < (hit_dist_to_goal - 0.2):
                self.wf_state = self.WF_ROTATE
                return False

            self.wall_following(p)
            return False

        # State: Aligning to goal before moving straight
        if self.wf_state == self.WF_ROTATE:
            bearing = p.get_bearing(self.goal)
            if self.rotate(bearing, self.WF_VEL):
                self.state = self.GOALSEEKING
                self.start = self.get_real_pose()
                self.stuck_counter = 0
                return False

        return False

    # -------------------------------------------------------------------------
    # Core Logic: Goal Seeking
    # -------------------------------------------------------------------------
    def goal_seeking(self, p):
        dist = p.get_range(self.goal)
        bearing = p.get_bearing(self.goal)

        # [Optimization] Use Fixed Cruise Speed
        # Dynamic acceleration causes instability in compact maps.
        final_vel = 2.0

        # Slow down when reaching the target
        if dist < self.DECEL_DIST:
             final_vel = final_vel * (dist / self.DECEL_DIST) + 0.5

        obstacle_dist = self.range_to_frontobstacle()

        # If obstacle detected ahead, switch to Wall Following
        if obstacle_dist <= 0.6:
            self.state = self.WALLFOLLOWING
            self.wf_state = self.WF_HIT
            self.hit_point = p
            self.max_hit_dist = 0.0 # Reset loop detection
            self.prev_error = 0
            return

        self.adjust_velocity(bearing, final_vel)

    def stuck_recovery(self):
        """ Simple routine to back up and turn if physically stuck. """
        self.set_velocity(-3.0, -1.0)
        self.reverse_counter -= 1
        if self.reverse_counter <= 0:
            self.state = self.GOALSEEKING
            self.stuck_counter = 0

    # -------------------------------------------------------------------------
    # Main Update Loop
    # -------------------------------------------------------------------------
    def update(self):
        """ Called every time step to update robot behavior. """
        p = self.get_real_pose()

        # 1. Handle Reversing (Recovery)
        if self.reverse_counter > 0:
            if self.state == self.STUCK_RECOVERY:
                self.stuck_recovery()
            else:
                self.reverse_counter -= 1
                self.set_velocity(-0.5, -3.0)
            return False

        # 2. Handle Physical Collision
        if self.bumper is not None and self.bumper.getValue() > 0.0:
            self.reverse_counter = 15
            return False

        # 3. Check if Goal Reached
        if p.get_range(self.goal) < self.GOAL_RADIUS:
            self.stop()
            return True

        # 4. Check if Stuck (Time based)
        if self.stuck_counter > 60:
            self.state = self.STUCK_RECOVERY
            self.reverse_counter = 40
            self.stuck_counter = 0

        # 5. Execute State Machine
        if self.state == self.STUCK_RECOVERY:
            self.stuck_recovery()
            return False

        elif self.state == self.GOALSEEKING:
            self.goal_seeking(p)
            return False

        else:
            # Execute Tangent Bug (returns True if loop detected/target unreachable)
            return self.tangent_bug(p)

    def set_goal(self, p):
        """ Updates the navigation target. """
        new_goal = pose.Pose(p.x, p.y, p.theta)

        # If currently navigating around a wall, just update coordinates without resetting state
        if self.state in [self.WALLFOLLOWING, self.STUCK_RECOVERY]:
            self.goal = new_goal
            return

        self.goal = new_goal
        self.start = self.get_real_pose()
        self.state = self.GOALSEEKING
        self.wf_state = self.WF_HIT
        self.hit_point = self.get_real_pose()
        self.max_hit_dist = 0.0 # Reset loop detection logic
        self.reverse_counter = 0
        self.stuck_counter = 0
        self.update()

    def stop(self):
        self.set_velocity(0,0)