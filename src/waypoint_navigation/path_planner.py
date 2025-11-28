#!/usr/bin/env python
# -- coding: utf-8 --

import math
import rospy
import networkx as nx
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from waypoint_navigation.graph_data import data  # Waypoints and links


class PathPlanner:
    """Path planner that computes shortest paths on a graph of waypoints
    and publishes the resulting path (with orientations) to /planned_path.

    Usage:
      planner = PathPlanner()
      planner.plan(start_id, goal_id)
    """

    def __init__(self):
        # Nothing that requires rospy.init_node is done at construction time.
        # Graph is built on demand in plan() so updates to graph_data are
        # picked up without restarting this object.
        pass

    def build_graph(self):
        """Build a directed graph from `data` where edge weights are Euclidean
        distances between connected waypoints."""
        G = nx.DiGraph()
        for node in data:
            from_id = node["id"]
            x1, y1 = node["x"], node["y"]
            for to_id in node.get("links", []):
                target = next((d for d in data if d["id"] == to_id), None)
                if target:
                    x2, y2 = target["x"], target["y"]
                    dist = math.hypot(x2 - x1, y2 - y1)
                    G.add_edge(from_id, to_id, weight=dist)
        return G

    def get_path_with_orientation(self, G, start_id, goal_id):
        """Compute the shortest path (by weight) and return both the list of
        waypoint ids and a path table [(x, y, theta), ...] representing pose
        targets with orientation toward the next waypoint."""

        path = nx.shortest_path(G, source=start_id, target=goal_id, weight='weight')
        path_table = []

        for i in range(len(path)):
            current_node = next(d for d in data if d["id"] == path[i])
            x, y = current_node["x"], current_node["y"]

            if i < len(path) - 1:
                next_node = next(d for d in data if d["id"] == path[i + 1])
                x2, y2 = next_node["x"], next_node["y"]
                dx = x2 - x
                dy = y2 - y
                theta = math.atan2(dy, dx)
            else:
                # Reuse last angle if this is the final point
                theta = path_table[-1][2]

            path_table.append((x, y, theta))

        return path, path_table

    def publish_path(self, path_table):
        """Publish a nav_msgs/Path to /planned_path using the (x,y,theta)
        tuples in path_table."""
        pub = rospy.Publisher("/planned_path", Path, queue_size=1, latch=True)
        path_msg = Path()
        path_msg.header.frame_id = "map"

        for x, y, theta in path_table:
            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.orientation.x = 0.0
            pose.pose.orientation.y = 0.0
            pose.pose.orientation.z = math.sin(theta / 2.0)
            pose.pose.orientation.w = math.cos(theta / 2.0)
            path_msg.poses.append(pose)

        # Give ROS time to establish connections before publishing
        rospy.sleep(0.5)
        pub.publish(path_msg)
        rospy.loginfo("Path published to /planned_path.")

    def plan(self, start, goal):
        """Compute a path from start to goal and publish it.

        Assumes rospy has been initialized by the caller. Returns True on
        success, False on failure.
        """
        try:
            G = self.build_graph()

            try:
                path_ids, path_table = self.get_path_with_orientation(G, start, goal)
            except nx.NetworkXNoPath:
                rospy.logerr("No path found between %d and %d.", start, goal)
                return False

            rospy.loginfo("Shortest path from %d to %d: %s", start, goal, str(path_ids))
            for i, (x, y, theta) in enumerate(path_table):
                rospy.loginfo("%02d: x=%.4f y=%.4f theta=%.2f deg", i + 1, x, y, math.degrees(theta))

            self.publish_path(path_table)
            rospy.loginfo("Path calculation complete.")
            return True
        except Exception as e:
            rospy.logerr("Unexpected error in PathPlanner.plan: %s", str(e))
            return False
