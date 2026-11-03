#!/usr/bin/env python3
"""
field_navigator.py

Drives the TurtleBot3 through a zigzag aisle route visiting each plant
column exactly once, in visit order matching plant_map.yaml IDs 1-16:

  Pass 1: Aisle 1 (x=1.4) northbound -> scans left  (col 0, plants #1-4)
  Pass 2: Aisle 2 (x=3.0) southbound -> scans right (col 1, plants #5-8)
  Pass 3: Aisle 2 (x=3.0) northbound -> scans right (col 2, plants #9-12)
  Pass 4: Aisle 3 (x=4.6) southbound -> scans left  (col 3, plants #13-16)
  Return home.

Publishes:
  /scan_side        (String, latched): 'left' / 'right' relative to robot heading
  /scan_active      (Bool, latched): True strictly during straight FORWARD aisle sweeps
  /target_column    (Int32, latched): 0, 1, 2, 3 corresponding to active pass

Uses a closed-loop proportional (P) heading-hold controller during all
straight-line driving to prevent lateral drift from compounding over the
length of an aisle. Reactive LIDAR obstacle avoidance kept as safety net.
"""

import math
import rospy
import rospkg
import yaml
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String, Bool, Int32
from tf.transformations import euler_from_quaternion


def normalize_angle(angle):
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def safe_min(values):
    valid = [v for v in values if math.isfinite(v) and v > 0.0]
    return min(valid) if valid else float('inf')


def clamp(value, lo, hi):
    return max(lo, min(hi, value))


# Full route definition: each entry is one pass or one transition.
# type: 'pass'   -> drive along y between y_start and y_end in given direction
#       'shift'  -> drive along x to target_x
# scan_side: published to /scan_side ('left' / 'right' relative to robot forward)
# target_col: index of plant column (0, 1, 2, 3)
# direction: +1 = northbound (toward y_end), -1 = southbound (toward y_start)
ROUTE = [
    {'type': 'pass',  'aisle_idx': 0, 'direction': +1, 'target_col': 0, 'scan_side': 'left'},   # Pass 1: Col 0 (#1-4)
    {'type': 'shift', 'to_aisle':  1},                                                          # Shift to Aisle 2
    {'type': 'pass',  'aisle_idx': 1, 'direction': -1, 'target_col': 1, 'scan_side': 'right'},  # Pass 2: Col 1 (#5-8)
    {'type': 'shift', 'to_aisle':  1},                                                          # Turn around in Aisle 2
    {'type': 'pass',  'aisle_idx': 1, 'direction': +1, 'target_col': 2, 'scan_side': 'right'},  # Pass 3: Col 2 (#9-12)
    {'type': 'shift', 'to_aisle':  2},                                                          # Shift to Aisle 3
    {'type': 'pass',  'aisle_idx': 2, 'direction': -1, 'target_col': 3, 'scan_side': 'left'},   # Pass 4: Col 3 (#13-16)
]


class FieldNavigator:
    def __init__(self):
        rospy.init_node('field_navigator')

        pkg_path = rospkg.RosPack().get_path('farm_field_scout')
        with open(pkg_path + '/config/plant_map.yaml') as f:
            field_data = yaml.safe_load(f)

        nav = field_data['navigation']
        self.aisles  = nav['aisles']    # [1.4, 3.0, 4.6]
        self.y_start = nav['y_start']   # -0.3
        self.y_end   = nav['y_end']     # 6.3
        self.home_x  = self.aisles[0]   # 1.4

        rospy.loginfo("field_navigator: aisles=%s  y=%.1f..%.1f",
                      self.aisles, self.y_start, self.y_end)

        self.linear_speed  = rospy.get_param('~linear_speed',  0.20)
        self.angular_speed = rospy.get_param('~angular_speed', 0.50)
        self.pos_tolerance = rospy.get_param('~pos_tolerance', 0.05)
        self.yaw_tolerance = rospy.get_param('~yaw_tolerance', 0.05)
        self.heading_kp    = rospy.get_param('~heading_kp',    1.8)

        self.obstacle_threshold    = rospy.get_param('~obstacle_threshold',    0.30)
        self.obstacle_clear_dist   = rospy.get_param('~obstacle_clear_distance', 0.50)
        self.avoid_pass_distance   = rospy.get_param('~avoid_pass_distance',   0.30)
        self.min_avoid_turn_angle  = rospy.get_param('~min_avoid_turn_angle',  0.35)

        self.cmd_pub         = rospy.Publisher('/cmd_vel',        Twist,  queue_size=10)
        self.scan_side_pub   = rospy.Publisher('/scan_side',      String, queue_size=10, latch=True)
        self.scan_active_pub = rospy.Publisher('/scan_active',    Bool,   queue_size=10, latch=True)
        self.target_col_pub  = rospy.Publisher('/target_column',  Int32,  queue_size=10, latch=True)

        rospy.Subscriber('/odom',  Odometry,  self.odom_callback)
        rospy.Subscriber('/scan',  LaserScan, self.scan_callback)

        self.x = 0.0; self.y = 0.0; self.yaw = 0.0
        self.odom_received   = False
        self.front_distance  = float('inf')
        self.left_distance   = float('inf')
        self.right_distance  = float('inf')
        self.scan_received   = False

        # Route sequencing
        self.route_step      = 0        # index into ROUTE list
        self.state           = 'ALIGN_START'
        self.heading_target  = 0.0
        self.patrol_complete = False

        # Avoidance
        self.resume_state         = None
        self.resume_yaw           = 0.0
        self.avoid_turn_dir       = 1
        self.avoid_turn_start_yaw = 0.0
        self.avoid_pass_start_x   = 0.0
        self.avoid_pass_start_y   = 0.0

        # Initially inactive
        self.scan_active_pub.publish(Bool(False))

        rospy.loginfo("field_navigator: waiting for /odom and /scan...")
        rate = rospy.Rate(10)
        while not rospy.is_shutdown() and not (self.odom_received and self.scan_received):
            rate.sleep()

        rospy.loginfo("field_navigator: sensors ready. Starting zigzag patrol.")

        while not rospy.is_shutdown():
            if self.patrol_complete:
                self.stop()
            else:
                self.control_loop()
            rate.sleep()

    # ------------------------------------------------------------------ #
    #  Sensor callbacks                                                    #
    # ------------------------------------------------------------------ #

    def odom_callback(self, msg):
        self.odom_received = True
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        _, _, self.yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])

    def scan_callback(self, msg):
        self.scan_received = True
        n = len(msg.ranges)
        if n == 0:
            return
        w = 10
        self.front_distance = safe_min(
            [msg.ranges[i] for i in list(range(0, w)) + list(range(n - w, n))])
        lc = n // 4
        self.left_distance  = safe_min(msg.ranges[max(0, lc-w):min(n, lc+w)])
        rc = (3 * n) // 4
        self.right_distance = safe_min(msg.ranges[max(0, rc-w):min(n, rc+w)])

    def stop(self):
        self.cmd_pub.publish(Twist())

    def heading_correction(self):
        err = normalize_angle(self.heading_target - self.yaw)
        return clamp(self.heading_kp * err, -self.angular_speed, self.angular_speed)

    # ------------------------------------------------------------------ #
    #  Main control loop                                                   #
    # ------------------------------------------------------------------ #

    def control_loop(self):
        moving = ('FORWARD', 'SHIFT', 'RETURN_HOME_DRIVE')

        if self.state in moving and self.front_distance < self.obstacle_threshold:
            rospy.logwarn("field_navigator: obstacle %.2fm ahead - avoiding.", self.front_distance)
            self.scan_active_pub.publish(Bool(False))
            self.resume_state         = self.state
            self.resume_yaw           = self.yaw
            self.avoid_turn_dir       = 1 if self.left_distance > self.right_distance else -1
            self.avoid_turn_start_yaw = self.yaw
            self.state = 'AVOID_TURN'

        if   self.state == 'ALIGN_START':       self.do_align()
        elif self.state == 'FORWARD':           self.do_forward()
        elif self.state == 'TURN_TO_NEXT':      self.do_turn_to_next()
        elif self.state == 'SHIFT':             self.do_shift()
        elif self.state == 'TURN_TO_SWEEP':     self.do_turn_to_sweep()
        elif self.state == 'RETURN_HOME_TURN':  self.do_return_home_turn()
        elif self.state == 'RETURN_HOME_DRIVE': self.do_return_home_drive()
        elif self.state == 'AVOID_TURN':        self.do_avoid_turn()
        elif self.state == 'AVOID_PASS':        self.do_avoid_pass()
        elif self.state == 'AVOID_RETURN':
            self.do_turn(self.resume_yaw, 'AVOID_RETURN_DONE')
        elif self.state == 'AVOID_RETURN_DONE':
            self.heading_target = self.resume_yaw
            self.state = self.resume_state
            if self.state == 'FORWARD':
                self.scan_active_pub.publish(Bool(True))

    # ------------------------------------------------------------------ #
    #  Route sequencing helpers                                            #
    # ------------------------------------------------------------------ #

    def current_step(self):
        if self.route_step < len(ROUTE):
            return ROUTE[self.route_step]
        return None

    def advance_route(self):
        """Move to the next route step and set up the relevant state."""
        self.scan_active_pub.publish(Bool(False))
        self.route_step += 1
        step = self.current_step()
        if step is None:
            # All route steps done - go home
            self.state = 'RETURN_HOME_TURN'
            rospy.loginfo("field_navigator: all passes complete, returning home.")
            return

        if step['type'] == 'pass':
            if step.get('scan_side'):
                self.scan_side_pub.publish(String(step['scan_side']))
            if 'target_col' in step:
                self.target_col_pub.publish(Int32(step['target_col']))

            rospy.loginfo("field_navigator: pass %d/%d  side=%s  dir=%s  col=%s",
                          self.route_step + 1, len(ROUTE),
                          step.get('scan_side'),
                          'north' if step['direction'] > 0 else 'south',
                          step.get('target_col'))
            yaw = math.pi / 2.0 if step['direction'] > 0 else -math.pi / 2.0
            self.state = 'TURN_TO_SWEEP'
            self._next_sweep_yaw = yaw
        else:  # shift
            self.state = 'TURN_TO_NEXT'

    # ------------------------------------------------------------------ #
    #  Motion states                                                       #
    # ------------------------------------------------------------------ #

    def do_align(self):
        """Initial alignment: face the direction of the first pass (north)."""
        step = self.current_step()
        target_yaw = math.pi / 2.0 if step['direction'] > 0 else -math.pi / 2.0
        if abs(normalize_angle(target_yaw - self.yaw)) < self.yaw_tolerance:
            self.stop()
            self.heading_target = target_yaw
            if step.get('scan_side'):
                self.scan_side_pub.publish(String(step['scan_side']))
            if 'target_col' in step:
                self.target_col_pub.publish(Int32(step['target_col']))
            rospy.loginfo("field_navigator: aligned. Starting pass 1 (north, left side, col 0).")
            self.state = 'FORWARD'
            self.scan_active_pub.publish(Bool(True))
            return
        twist = Twist()
        twist.angular.z = self.angular_speed if normalize_angle(target_yaw - self.yaw) > 0 \
                          else -self.angular_speed
        self.cmd_pub.publish(twist)

    def do_forward(self):
        step = self.current_step()
        target_y = self.y_end if step['direction'] > 0 else self.y_start

        if abs(self.y - target_y) < self.pos_tolerance:
            self.stop()
            self.scan_active_pub.publish(Bool(False))
            rospy.loginfo("field_navigator: reached y=%.2f, pass done.", self.y)
            self.advance_route()
            return

        twist = Twist()
        twist.linear.x  = self.linear_speed
        twist.angular.z = self.heading_correction()
        self.cmd_pub.publish(twist)

    def do_turn_to_next(self):
        """Turn to face +x before shifting to the next aisle."""
        self.scan_active_pub.publish(Bool(False))
        step = self.current_step()
        next_aisle_x = self.aisles[step['to_aisle']]
        target_yaw   = 0.0 if next_aisle_x >= self.x else math.pi

        if abs(normalize_angle(target_yaw - self.yaw)) < self.yaw_tolerance:
            self.stop()
            self.heading_target = target_yaw
            self.state = 'SHIFT'
            return

        twist = Twist()
        twist.angular.z = self.angular_speed \
                          if normalize_angle(target_yaw - self.yaw) > 0 \
                          else -self.angular_speed
        self.cmd_pub.publish(twist)

    def do_shift(self):
        self.scan_active_pub.publish(Bool(False))
        step = self.current_step()
        target_x = self.aisles[step['to_aisle']]

        if abs(self.x - target_x) < self.pos_tolerance:
            self.stop()
            rospy.loginfo("field_navigator: shifted to aisle x=%.2f.", target_x)
            self.advance_route()
            return

        twist = Twist()
        twist.linear.x  = self.linear_speed
        twist.angular.z = self.heading_correction()
        self.cmd_pub.publish(twist)

    def do_turn_to_sweep(self):
        self.scan_active_pub.publish(Bool(False))
        target_yaw = self._next_sweep_yaw
        if abs(normalize_angle(target_yaw - self.yaw)) < self.yaw_tolerance:
            self.stop()
            self.heading_target = target_yaw
            rospy.loginfo("field_navigator: turn done, starting FORWARD sweep.")
            self.state = 'FORWARD'
            self.scan_active_pub.publish(Bool(True))
            return
        twist = Twist()
        twist.angular.z = self.angular_speed \
                          if normalize_angle(target_yaw - self.yaw) > 0 \
                          else -self.angular_speed
        self.cmd_pub.publish(twist)

    def do_return_home_turn(self):
        self.scan_active_pub.publish(Bool(False))
        target_yaw = 0.0 if self.home_x > self.x else math.pi
        if abs(normalize_angle(target_yaw - self.yaw)) < self.yaw_tolerance:
            self.stop()
            self.heading_target = target_yaw
            self.state = 'RETURN_HOME_DRIVE'
            return
        twist = Twist()
        twist.angular.z = self.angular_speed \
                          if normalize_angle(target_yaw - self.yaw) > 0 \
                          else -self.angular_speed
        self.cmd_pub.publish(twist)

    def do_return_home_drive(self):
        self.scan_active_pub.publish(Bool(False))
        if abs(self.x - self.home_x) < self.pos_tolerance:
            self.stop()
            rospy.loginfo("field_navigator: PATROL COMPLETE - returned home.")
            self.patrol_complete = True
            return
        twist = Twist()
        twist.linear.x  = self.linear_speed
        twist.angular.z = self.heading_correction()
        self.cmd_pub.publish(twist)

    def do_turn(self, target_yaw, next_state):
        self.scan_active_pub.publish(Bool(False))
        if abs(normalize_angle(target_yaw - self.yaw)) < self.yaw_tolerance:
            self.stop()
            self.state = next_state
            return
        twist = Twist()
        twist.angular.z = self.angular_speed \
                          if normalize_angle(target_yaw - self.yaw) > 0 \
                          else -self.angular_speed
        self.cmd_pub.publish(twist)

    # ------------------------------------------------------------------ #
    #  Obstacle avoidance (safety net)                                     #
    # ------------------------------------------------------------------ #

    def do_avoid_turn(self):
        self.scan_active_pub.publish(Bool(False))
        turned = abs(normalize_angle(self.yaw - self.avoid_turn_start_yaw))
        if self.front_distance > self.obstacle_clear_dist and \
                turned >= self.min_avoid_turn_angle:
            self.stop()
            self.avoid_pass_start_x = self.x
            self.avoid_pass_start_y = self.y
            self.state = 'AVOID_PASS'
            return
        twist = Twist()
        twist.angular.z = self.angular_speed * self.avoid_turn_dir
        self.cmd_pub.publish(twist)

    def do_avoid_pass(self):
        self.scan_active_pub.publish(Bool(False))
        dx = self.x - self.avoid_pass_start_x
        dy = self.y - self.avoid_pass_start_y
        if math.sqrt(dx*dx + dy*dy) >= self.avoid_pass_distance:
            self.stop()
            self.state = 'AVOID_RETURN'
            return
        if self.front_distance < self.obstacle_threshold:
            self.avoid_turn_dir       = 1 if self.left_distance > self.right_distance else -1
            self.avoid_turn_start_yaw = self.yaw
            self.state = 'AVOID_TURN'
            return
        twist = Twist()
        twist.linear.x = self.linear_speed
        self.cmd_pub.publish(twist)


if __name__ == '__main__':
    try:
        FieldNavigator()
    except rospy.ROSInterruptException:
        pass
