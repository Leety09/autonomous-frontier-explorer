# autonomous_explorer.py
# Strategy: Fully Dynamic Frontier Exploration (No fixed waypoints)

from controller import Supervisor
import pioneer_clnav as pn
import pioneer_simpleproxsensors as psps
import occupancy_grid as ogrid
import pose
import math

def run_robot(robot):
    timestep = int(robot.getBasicTimeStep())
    pps = psps.PioneerSimpleProxSensors(robot)
    nav = pn.PioneerCLNav(robot, pps)

    # Keep resolution at 20
    occupancy_grid = ogrid.OccupancyGrid(robot, 15, "display", nav.get_real_pose(), pps)

    # State definitions
    STATE_INIT_SCAN = 0  # Rotate in place to map
    STATE_EXPLORE = 1    # Move to unknown areas

    current_state = STATE_INIT_SCAN
    scan_counter = 0

    # Failure counter (prevent getting stuck at a point)
    fail_count = 0
    current_goal = None

    print("Strategy: Dynamic Frontier Exploration Started.")

    steps = 0

    while robot.step(timestep) != -1:
        steps += 1
        current_pose = nav.get_real_pose()

        # 1. Mapping (frequency optimization)
        if steps % 3 == 0:
            occupancy_grid.map(current_pose)
        if steps % 15 == 0:
            occupancy_grid.paint()

            # --- New: Coverage check ---
            # occupancy_grid.coverage is a value between 0.0 and 1.0 calculated in paint()
            if occupancy_grid.coverage > 0.96:
                print(f"Map Coverage Reached: {occupancy_grid.coverage * 100:.1f}%! Stopping.")
                nav.stop()
                break # Exit loop, end program
            # ---------------------

        # --- State machine ---

        # State 0: Initial in-place rotation scan (establish first batch of known areas)
        if current_state == STATE_INIT_SCAN:
            nav.set_velocity(1.0, -1.0) # Rotate in place
            scan_counter += 1
            if scan_counter > 80: # Time to rotate approximately one circle (adjust based on simulation step size)
                print("Init Scan Complete. Switching to Auto Explore.")
                nav.stop()
                current_state = STATE_EXPLORE
                steps = 0 # Reset step count for timeout calculation
            continue # Skip subsequent logic

        # State 1: Automatically find unknown areas
        if current_state == STATE_EXPLORE:

            # If no target, or target reached, or timeout -> find new target
            reached = nav.update()
            is_timeout = (steps > 4000) # Give it enough time to run

            needs_new_goal = (current_goal is None) or reached or is_timeout

            if needs_new_goal:
                if reached: print("Target Reached.")
                if is_timeout: print("Target Timeout (Stuck?). Finding new target.")

                nav.stop()

                # Get next best exploration point
                target = occupancy_grid.get_frontier_target(current_pose)

                if target:
                    print(f"New Frontier Found at: ({target.x:.2f}, {target.y:.2f})")
                    nav.set_goal(target)
                    current_goal = target
                    steps = 0 # Reset timeout count
                    fail_count = 0
                else:
                    fail_count += 1
                    print(f"No frontiers found... Scanning ({fail_count}/5)")
                    # If no target found several times, might just be bad angle, rotate in place and check again
                    nav.set_velocity(-1.0, 1.0)

                    if fail_count > 10:
                        print("Mission Complete! Map is fully explored.")
                        nav.stop()
                        occupancy_grid.paint()
                        break

if __name__ == "__main__":
    my_robot = Supervisor()
    run_robot(my_robot)
