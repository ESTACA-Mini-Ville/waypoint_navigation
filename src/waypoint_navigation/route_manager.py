import rospy
import math
import socket
import threading
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import Int32
from waypoint_navigation.graph_data import data
from waypoint_navigation.path_planner import PathPlanner

# --- CONFIGURATION DES FEUX ---
# Remplace x, y par les coordonnées réelles des feux
TRAFFIC_LIGHTS = {
    "FEU_1": {"x": 1.2, "y": 3.4},
    "FEU_2": {"x": -0.5, "y": 1.1},
    "FEU_3": {"x": 2.8, "y": -2.0},
    "FEU_4": {"x": 0.0, "y": 0.0},
}
ZONE_THRESHOLD = 0.5 # Rayon de la zone qui sera ajoutée et retiré a x et y pour définir la zone d'arrêt
UDP_PORT = 5005      # Port UDP à adapter

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
        self.light_states = {} # Stocke l'état (ex: {"FEU_1": "RED"})
        self.lock = threading.Lock()

    def start(self):
        """Initialize ROS publishers/subscribers. Assumes rospy.init_node() has
        already been called by the caller."""

        if self.started:
            return

        # Create planner instance now that rospy is expected to be initialized
        self.path_planner = PathPlanner()
        # Topic pour dire au follower de s'arrêter
        self.stop_pub = rospy.Publisher("/traffic_stop", Bool, queue_size=1)
        
        rospy.Subscriber("/destination", Int32, self.destination_callback)
        rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, self.pose_callback)
        
        # Lancement de l'écoute UDP dans un thread séparé
        threading.Thread(target=self.udp_listener, daemon=True).start()
        
        rospy.loginfo("Destination Manager started.")
        self.started = True
        
        def udp_listener(self):
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            sock.bind(("0.0.0.0", UDP_PORT))
            while not rospy.is_shutdown():
                try:
                    data_raw, _ = sock.recvfrom(1024)
                    # Format attendu: "FEU_1:RED"
                    msg = data_raw.decode().strip().split(":")
                    if len(msg) == 2:
                        with self.lock:
                            self.light_states[msg[0]] = msg[1]
                except: pass
                    
    def pose_callback(self, msg):
        """Update the stored robot pose from AMCL."""
        self.robot_pose = msg.pose.pose
        self.check_traffic_logic() # On vérifie à chaque update de pose
        
    def check_traffic_logic(self):
            if not self.robot_pose: return
            
            rx = self.robot_pose.position.x
            ry = self.robot_pose.position.y
            must_stop = False
    
            for name, pos in TRAFFIC_LIGHTS.items():
                # Vérification de la zone (+-X, +-Y)
                if abs(rx - pos["x"]) < ZONE_THRESHOLD and abs(ry - pos["y"]) < ZONE_THRESHOLD:
                    with self.lock:
                        state = self.light_states.get(name, "GREEN")
                        if state == "RED":
                            must_stop = True
                            rospy.logwarn_throttle(2, f"ARRÊT: {name} est ROUGE")
                            break
        
        self.stop_pub.publish(Bool(must_stop))


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


