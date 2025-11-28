#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import math
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
from nav_msgs.msg import Path
from tf.transformations import euler_from_quaternion

class PurePursuitFollower:
    def __init__(self):
        # Do not call rospy.init_node or rospy.spin here so the class can be
        # instantiated inside another node/process. Call start() to create
        # ROS publishers/subscribers after rospy.init_node has been called.
        self.cmd_pub = None
        self.path_sub = None
        self.pose_sub = None

        self.path = []
        self.pose = None
        self.current_index = 0

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
        self._started = True
        rospy.loginfo("🚗 Pure Pursuit path follower started (in-process).")

    def stop(self):
        """Stop the follower: unregister subscribers and publish zero velocity."""
        try:
            if self.path_sub is not None:
                self.path_sub.unregister()
                self.path_sub = None
            if self.pose_sub is not None:
                self.pose_sub.unregister()
                self.pose_sub = None
        except Exception:
            pass

        if self.cmd_pub is not None:
            try:
                self.cmd_pub.publish(Twist())
            except Exception:
                pass
        rospy.loginfo("� Pure Pursuit follower stopped.")

    def path_callback(self, msg):
        self.path = [(pose.pose.position.x, pose.pose.position.y) for pose in msg.poses]
        self.current_index = 0
        rospy.loginfo("📥 Received path with %d points", len(self.path))

    def pose_callback(self, msg):
        self.pose = msg.pose.pose
        self.follow_path()

    def follow_path(self):
        if not self.path or not self.pose:
            rospy.logwarn("❌ Path or pose not ready.")
            return

        x = self.pose.position.x
        y = self.pose.position.y
        (_, _, yaw) = euler_from_quaternion([
            self.pose.orientation.x,
            self.pose.orientation.y,
            self.pose.orientation.z,
            self.pose.orientation.w
        ])

        target = None
        for i in range(self.current_index, len(self.path)):
            px, py = self.path[i]
            dist = math.hypot(px - x, py - y)
            if dist >= self.lookahead_distance:
                target = (px, py)
                self.current_index = i
                break

        if not target:
            final_x, final_y = self.path[-1]
            final_dist = math.hypot(final_x - x, final_y - y)
            if final_dist < 0.1:
                rospy.loginfo("✅ Close enough to final goal. Stopping.")
                self.cmd_pub.publish(Twist())
            else:
                rospy.logwarn("⚠️ No valid lookahead point, but not at goal. Holding.")
            return

        tx, ty = target
        angle_to_target = math.atan2(ty - y, tx - x)
        angle_diff = self.normalize_angle(angle_to_target - yaw)

        cmd = Twist()
        cmd.linear.x = self.linear_speed
        cmd.angular.z = self.angular_gain * angle_diff
        cmd.angular.z = max(-self.max_angular_speed, min(cmd.angular.z, self.max_angular_speed))

        rospy.loginfo("🔄 Robot at: x=%.2f y=%.2f yaw=%.2f", x, y, yaw)
        rospy.loginfo("🎯 Target: x=%.2f y=%.2f | Δθ=%.2f", tx, ty, angle_diff)
        rospy.loginfo("🚀 cmd_vel: linear=%.2f angular=%.2f", cmd.linear.x, cmd.angular.z)
        self.cmd_pub.publish(cmd)

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

