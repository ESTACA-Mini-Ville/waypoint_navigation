#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import math
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
from nav_msgs.msg import Path
from std_msgs.msg import Bool
from tf.transformations import euler_from_quaternion

class PurePursuitFollower:
    def __init__(self):
        # Do not call rospy.init_node or rospy.spin here so the class can be
        # instantiated inside another node/process. Call start() to create
        # ROS publishers/subscribers after rospy.init_node has been called.
        self.cmd_pub = None
        self.path_sub = None
        self.pose_sub = None
        self.stop_sub = None
        
        self.path = []
        self.pose = None
        self.current_index = 0
        self.is_stopped_by_traffic = False
        
        self.lookahead_distance = 0.3
        self.linear_speed = 0.3
        self.angular_gain = 0.7
        self.max_angular_speed = 1.0

        self._started = False

    def start(self):
        """Initialize ROS publishers/subscribers. Assumes rospy.init_node was
        already called by the caller (e.g. navigation_manager)."""
        if self._started:
            return
        self.cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        self.path_sub = rospy.Subscriber("/planned_path", Path, self.path_callback)
        self.pose_sub = rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, self.pose_callback)
        self.stop_sub = rospy.Subscriber("/traffic_stop", Bool, self.traffic_callback)
        
        self._started = True
        rospy.loginfo("🚗 Pure Pursuit path follower started (in-process).")

    def traffic_callback(self, msg):
        self.is_stopped_by_traffic = msg.data

    def pose_callback(self, msg):
        self.pose = msg.pose.pose
        self.follow_path()

    def follow_path(self):
        if not self.path or not self.pose:
            return

        # --- SÉCURITÉ FEU DE CIRCULATION ---
        if self.is_stopped_by_traffic:
            self.cmd_pub.publish(Twist()) # Force l'arrêt
            return
        # ------------------------------------

        x = self.pose.position.x
        y = self.pose.position.y
        (_, _, yaw) = euler_from_quaternion([
            self.pose.orientation.x, self.pose.orientation.y,
            self.pose.orientation.z, self.pose.orientation.w
        ])

        target = None
        for i in range(self.current_index, len(self.path)):
            px, py = self.path[i]
            if math.hypot(px - x, py - y) >= self.lookahead_distance:
                target = (px, py)
                self.current_index = i
                break

        if not target:
            final_x, final_y = self.path[-1]
            if math.hypot(final_x - x, final_y - y) < 0.1:
                self.cmd_pub.publish(Twist())
            return

        tx, ty = target
        angle_diff = self.normalize_angle(math.atan2(ty - y, tx - x) - yaw)
        
        cmd = Twist()
        cmd.linear.x = self.linear_speed
        cmd.angular.z = max(-self.max_angular_speed, min(self.angular_gain * angle_diff, self.max_angular_speed))
        self.cmd_pub.publish(cmd)

    def normalize_angle(self, angle):
        while angle > math.pi: angle -= 2 * math.pi
        while angle < -math.pi: angle += 2 * math.pi
        return angle

    def path_callback(self, msg):
        self.path = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
        self.current_index = 0

    def stop(self):
        if self.stop_sub: self.stop_sub.unregister()
        if self.cmd_pub: self.cmd_pub.publish(Twist())

