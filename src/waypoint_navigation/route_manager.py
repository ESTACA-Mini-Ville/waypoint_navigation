import rospy
import math
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import Int32, Bool
from rospy.msg import AnyMsg
from waypoint_navigation.graph_data import data
from waypoint_navigation.path_planner import PathPlanner
import threading
import struct

# --- CONFIGURATION DES FEUX ---
# Remplace x, y par les coordonnées réelles des feux
TRAFFIC_LIGHTS = {
    "ZONE_1_ROUTE_A": {"x": 1.55, "y": 0.37, "route": "A"},
    "ZONE_2_ROUTE_A": {"x": 5.25, "y": 5.22, "route": "A"},
    "ZONE_3_ROUTE_B": {"x": 0.37, "y": 4.05, "route": "B"},
    "ZONE_4_ROUTE_B": {"x": 6.42, "y": 1.55, "route": "B"},
}
ZONE_THRESHOLD = 0.5 # Rayon de la zone +-X, +-Y
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
        self.current_schedule = []
        self.stop_pub = None
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
        
        # Ecoute du statut des feux
        rospy.Subscriber("/traffic_lights_status", AnyMsg, self.traffic_callback)
        
        rospy.loginfo("Destination Manager started.")
        self.started = True
        
    def traffic_callback(self, msg):
        """Récupère l'état actuel et le schedule depuis le topic ROS."""
        try:
            state = 3
            schedule = []
            
            if hasattr(msg, '_connection_header') and hasattr(msg, '_buff'):
                type_str = msg._connection_header.get('type')
                import roslib.message
                msg_class = roslib.message.get_message_class(type_str) if type_str else None
                
                if msg_class:
                    parsed_msg = msg_class().deserialize(msg._buff)
                    state = getattr(parsed_msg, 'current_state', 3)
                    schedule = getattr(parsed_msg, 'schedule', [])
                else:
                    # Fallback struct parsing if Python message not generated
                    buff = msg._buff
                    if len(buff) >= 16:
                        state, timestamp, sched_len = struct.unpack('<idI', buff[:16])
                        offset = 16
                        for i in range(sched_len):
                            if offset + 20 > len(buff): break
                            st, start_time, duration = struct.unpack('<idd', buff[offset:offset+20])
                            schedule.append({'state': st, 'start_time': start_time, 'duration': duration})
                            offset += 20
            else:
                state = getattr(msg, 'current_state', 3)
                schedule = getattr(msg, 'schedule', [])
                
            with self.lock:
                self.current_traffic_state = state
                self.current_schedule = schedule
                
        except Exception as e:
            rospy.logwarn_throttle(2, "Erreur de parsing traffic status: %s", str(e))
                    
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
            schedule = self.current_schedule

        for name, zone in TRAFFIC_LIGHTS.items():
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
