# -*- coding: utf-8 -*-
import rospy
import math
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import Int32, Bool
from rospy.msg import AnyMsg
from waypoint_navigation.graph_data import data
from waypoint_navigation.path_planner import PathPlanner
try:
    from ros_dds_bridge.msg import TrafficLightsStatus
except ImportError:
    # Fallback if the package is not found during static analysis
    TrafficLightsStatus = AnyMsg
import threading
import struct

# --- CONFIGURATION DES FEUX ---
# Remplace x, y par les coordonness reelles des feux
TRAFFIC_LIGHTS = {
    "ZONE_1_ROUTE_A": {"x": -0.82, "y": -0.9, "route": "A"}, # Haut => bas
    "ZONE_2_ROUTE_A": {"x": -0.7, "y": -2.43, "route": "A"}, # Bas => haut
    "ZONE_3_ROUTE_B": {"x": -1.37, "y": -1.39, "route": "B"}, # gauche => droite
    "ZONE_4_ROUTE_B": {"x": 0.26, "y": -1.28, "route": "B"}, # Droite => gauche 
}
ZONE_THRESHOLD = 0.2 # Rayon de la zone +-X, +-Y
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
        
        rospy.Subscriber("/traffic_lights_status", TrafficLightsStatus, self.traffic_callback)
        rospy.Subscriber("/destination_reached", Bool, self.reached_callback)
        rospy.Subscriber("/cancel_goal", Bool, self.cancel_callback)
        
        rospy.loginfo("Destination Manager started.")
        self.started = True
        
    def traffic_callback(self, msg):
        """Récupère l'état actuel et le schedule depuis le topic ROS."""
        try:
            # Si on utilise AnyMsg (fallback), on ne peut pas accéder aux attributs directement
            if hasattr(msg, '_buff'):
                # On réutilise une partie de l'ancienne logique de parsing si nécessaire, 
                # mais le but est d'utiliser le type TrafficLightsStatus
                state = 3
                if len(msg._buff) >= 4:
                    state = struct.unpack('<i', msg._buff[:4])[0]
            else:
                state = getattr(msg, 'current_state', 3)
                
            with self.lock:
                self.current_traffic_state = state
                # On peut aussi récupérer le schedule si besoin
                self.current_schedule = getattr(msg, 'schedule', [])
                
            # Re-vérifier les conditions dès que le statut change
            self.check_traffic_conditions()
                
        except Exception as e:
            rospy.logwarn_throttle(2, "Erreur de parsing traffic status: %s", str(e))

    def reached_callback(self, msg):
        """Called when the follower notifies that the goal is reached."""
        if msg.data:
            rospy.loginfo("Goal reached reached signal received. Cancelling/Clearing current path.")
            if self.path_planner:
                self.path_planner.clear()

    def cancel_callback(self, msg):
        """Allows external systems to cancel the current navigation goal."""
        if msg.data:
            rospy.loginfo("Navigation cancel requested.")
            if self.path_planner:
                self.path_planner.clear()
                    
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
