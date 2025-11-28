"""waypoint_navigation package initializer.

This file makes the `waypoint_navigation` directory a proper Python package
so ROS/catkin-installed scripts can import it reliably.

Keep this file minimal to avoid side effects on import.
"""

__all__ = [
    "graph_data",
    "path_planner",
    "pure_pursuit_follower",
    "route_manager",
]
