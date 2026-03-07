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
    "ZONE_1_ROUTE_A": {"x": 1.55, "y": 0.37, "route": "A"},
    "ZONE_2_ROUTE_A": {"x": 5.25, "y": 5.22, "route": "A"},
    "ZONE_3_ROUTE_B": {"x": 0.37, "y": 4.05, "route": "B"},
    "ZONE_4_ROUTE_B": {"x": 6.42, "y": 1.55, "route": "B"},
}
ZONE_THRESHOLD = 0.5 # Rayon de la zone +-X, +-Y
UDP_PORT = 5005      # Port UDP 

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
        self.current_traffic_state = 3  # Par défaut : ALL_RED (3)
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
        """Écoute les états envoyés par le TrafficLightManager."""
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.bind(("0.0.0.0", UDP_PORT))
        while not rospy.is_shutdown():
            try:
                data_raw, _ = sock.recvfrom(1024)
                # On attend l'entier correspondant à l'IntEnum (1 à 5)
                state = int(data_raw.decode().strip())
                with self.lock:
                    self.current_traffic_state = state
            except Exception as e:
                rospy.logerr("UDP Error: %s", str(e))
                    
        def pose_callback(self, msg):
            """Met à jour la pose et vérifie immédiatement les feux."""
            self.robot_pose = msg.pose.pose
            self.check_traffic_conditions()
    
        def check_traffic_conditions(self):
            """Logique de décision d'arrêt."""
            if not self.robot_pose:
                return
    
            rx = self.robot_pose.position.x
            ry = self.robot_pose.position.y
            must_stop = False
    
            with self.lock:
                state = self.current_traffic_state
    
            for name, zone in TRAFFIC_ZONES.items():
                # Vérification de la zone de proximité
                if abs(rx - zone["x"]) < ZONE_THRESHOLD and abs(ry - zone["y"]) < ZONE_THRESHOLD:
                    # Logique basée sur ton TrafficLightState :
                    # Route A : S'arrête si l'état n'est pas A_GREEN (1)
                    if zone["route"] == "A" and state != 1:
                        must_stop = True
                    # Route B : S'arrête si l'état n'est pas B_GREEN (4)
                    elif zone["route"] == "B" and state != 4:
                        must_stop = True
                    
                    if must_stop:
                        rospy.logwarn_throttle(2, "FEU ROUGE détecté dans %s (Etat: %d)", name, state)
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
            destination_id = msg.data
            start_id = self.find_closest_waypoint()
            if start_id is not None and start_id != -1 and start_id != destination_id:
                self.path_planner.plan(start_id, destination_id)
