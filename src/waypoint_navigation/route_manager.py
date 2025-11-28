import rospy
import math
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import Int32
from waypoint_navigation.graph_data import data
from waypoint_navigation.path_planner import PathPlanner


class RouteManager:
    def __init__(self):
        """Route manager handles incoming destination requests and
        orchestrates planning and following.

        Do not call rospy.init_node() or rospy.spin() here so the class can be
        instantiated inside another process. Call start() after rospy.init_node
        has been called.
        """

        self.robot_pose = None
        self.path_planner = None
        self.started = False

    def start(self):
        """Initialize ROS publishers/subscribers. Assumes rospy.init_node() has
        already been called by the caller."""

        if self.started:
            return

        # Create planner instance now that rospy is expected to be initialized
        self.path_planner = PathPlanner()

        rospy.Subscriber("/destination", Int32, self.destination_callback)
        rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, self.pose_callback)

        rospy.loginfo("Destination Manager started.")
        self.started = True

    def pose_callback(self, msg):
        """Update the stored robot pose from AMCL."""
        self.robot_pose = msg.pose.pose

    def find_closest_waypoint(self):
        """Return the ID of the closest waypoint to the robot's current position.

        Returns None if the robot pose is not yet known.
        """

        if not self.robot_pose:
            rospy.logwarn("Couldn't find closest waypoint: robot position unknown")
            return None

        closest_id = -1
        min_dist = float('inf')

        for waypoint in data:
            dist = math.hypot(waypoint["x"] - self.robot_pose.position.x,
                              waypoint["y"] - self.robot_pose.position.y)

            if dist < min_dist:
                min_dist = dist
                closest_id = waypoint["id"]

        rospy.loginfo("Closest departure waypoint found: ID %d", closest_id)
        return closest_id

    def destination_callback(self, msg):
        """Handle a new destination request: plan a path and start the follower."""

        destination_id = msg.data
        rospy.loginfo("New destination received: ID %d", destination_id)

        # Find the closest waypoint to use as start
        start_id = self.find_closest_waypoint()
        if start_id is None or start_id == -1:
            return

        if start_id == destination_id:
            rospy.loginfo("Robot is already at the destination.")
            return

        # Compute path using the planner
        try:
            rospy.loginfo("Starting path planner from ID %d to ID %d...", start_id, destination_id)
            success = self.path_planner.plan(start_id, destination_id)
            if not success:
                rospy.logerr("Path planner failed to generate a path.")
                return
        except Exception as e:
            rospy.logerr("Critical error calling PathPlanner.plan: %s", str(e))
            return


