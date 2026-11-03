#!/usr/bin/env python3
"""
plant_inspector.py

Detects plants using a calibrated LIDAR diagonal band inside the camera's FOV.
Listens to /scan_active and /target_column to ensure inspection only occurs
during straight aisle passes and matches only plants in the active column.

Feature extraction uses saturation masking and hue normalization:
  H_adj = H if H < 90.0 else H - 180.0
Produces highly accurate classification between Red, Yellow, and Green.
Publishes inspection reports to /plant_inspection_report.
"""

import math
import pickle
import cv2
import numpy as np
import rospy
import rospkg
import yaml
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan, Image
from std_msgs.msg import String, Bool, Int32
from cv_bridge import CvBridge


def safe_min(values):
    valid = [v for v in values if math.isfinite(v) and v > 0.0]
    return min(valid) if valid else float('inf')


class PlantInspector:
    def __init__(self):
        rospy.init_node('plant_inspector')

        pkg_path = rospkg.RosPack().get_path('farm_field_scout')
        with open(pkg_path + '/config/plant_map.yaml') as f:
            self.plants = yaml.safe_load(f)['plants']
        with open(pkg_path + '/config/plant_classifier.pkl', 'rb') as f:
            saved = pickle.load(f)
        self.model  = saved['model']
        self.scaler = saved['scaler']

        rospy.loginfo("plant_inspector: loaded %d plants + classifier.", len(self.plants))

        # Band angles (degrees CCW from forward):
        # Left band: 25-42 deg (front-left diagonal)
        # Right band: 318-335 deg (front-right diagonal)
        self.left_band_min_deg  = rospy.get_param('~left_band_min_deg',  25.0)
        self.left_band_max_deg  = rospy.get_param('~left_band_max_deg',  42.0)
        self.right_band_min_deg = rospy.get_param('~right_band_min_deg', 318.0)
        self.right_band_max_deg = rospy.get_param('~right_band_max_deg', 335.0)

        self.detection_threshold = rospy.get_param('~detection_threshold', 1.6)
        self.match_radius        = rospy.get_param('~match_radius', 2.0)
        self.sat_min             = rospy.get_param('~saturation_mask_min', 40)

        self.bridge        = CvBridge()
        self.latest_image  = None
        self.active_side   = None
        self.scan_active   = False
        self.target_column = None
        self.x = 0.0
        self.y = 0.0
        self.currently_near  = False
        self.inspected_ids   = set()

        self.report_pub = rospy.Publisher('/plant_inspection_report', String, queue_size=10)

        rospy.Subscriber('/scan_side',       String,    self.side_callback)
        rospy.Subscriber('/scan_active',     Bool,      self.active_callback)
        rospy.Subscriber('/target_column',   Int32,     self.column_callback)
        rospy.Subscriber('/odom',            Odometry,  self.odom_callback)
        rospy.Subscriber('/scan',            LaserScan, self.scan_callback)
        rospy.Subscriber('/camera/rgb/image_raw', Image, self.image_callback)

        rospy.loginfo("plant_inspector: ready.")
        rospy.spin()

    def side_callback(self, msg):
        if self.active_side != msg.data:
            self.active_side    = msg.data
            self.currently_near = False
            rospy.loginfo("plant_inspector: active side -> %s", self.active_side)

    def active_callback(self, msg):
        self.scan_active = msg.data
        if not self.scan_active:
            self.currently_near = False

    def column_callback(self, msg):
        self.target_column = msg.data
        rospy.loginfo("plant_inspector: target column -> %d", self.target_column)

    def odom_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

    def image_callback(self, msg):
        try:
            self.latest_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            rospy.logwarn("plant_inspector: image convert error: %s", e)

    def band_indices(self, n):
        if self.active_side == 'left':
            lo = int(round(self.left_band_min_deg  / 360.0 * n))
            hi = int(round(self.left_band_max_deg  / 360.0 * n))
        else:  # right
            lo = int(round(self.right_band_min_deg / 360.0 * n))
            hi = int(round(self.right_band_max_deg / 360.0 * n))
        return range(max(0, lo), min(n, hi + 1))

    def scan_callback(self, msg):
        # Only detect when sweep is actively forward in an aisle
        if not self.scan_active or self.active_side is None or self.target_column is None:
            return

        n = len(msg.ranges)
        if n == 0:
            return

        indices = self.band_indices(n)
        dist = safe_min([msg.ranges[i] for i in indices if 0 <= i < n])

        if dist < self.detection_threshold and not self.currently_near:
            self.currently_near = True
            self.trigger_inspection(dist)
        elif dist >= self.detection_threshold:
            self.currently_near = False

    def find_nearest_plant(self):
        # Strictly search uninspected plants within the target column
        col_candidates = [
            p for p in self.plants
            if p['col'] == self.target_column and p['id'] not in self.inspected_ids
        ]
        if not col_candidates:
            # Fallback to all plants in column if needed
            col_candidates = [p for p in self.plants if p['col'] == self.target_column]
        if not col_candidates:
            return None, float('inf')

        best, best_dist = None, float('inf')
        for p in col_candidates:
            d = math.hypot(p['x'] - self.x, p['y'] - self.y)
            if d < best_dist:
                best_dist = d
                best = p

        return (best, best_dist) if best_dist <= self.match_radius else (None, best_dist)

    def extract_hsv_features(self, image):
        h_img, w_img = image.shape[:2]
        if self.active_side == 'left':
            roi = image[:, :int(w_img * 0.70)]
        elif self.active_side == 'right':
            roi = image[:, int(w_img * 0.30):]
        else:
            roi = image

        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        sat_mask = (hsv[:, :, 1] >= self.sat_min).astype(np.uint8)

        contours, _ = cv2.findContours(sat_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return None

        largest_c = max(contours, key=cv2.contourArea)
        if cv2.contourArea(largest_c) < 50:
            return None

        plant_mask = np.zeros_like(sat_mask)
        cv2.drawContours(plant_mask, [largest_c], -1, 1, thickness=-1)
        mask_bool = (plant_mask == 1)

        h_raw = hsv[:, :, 0][mask_bool].astype(np.float32)
        s_raw = hsv[:, :, 1][mask_bool].astype(np.float32)
        v_raw = hsv[:, :, 2][mask_bool].astype(np.float32)

        # Per-pixel hue unwrapping:
        # Red wraps around 0 and 180. Pixels >= 90 wrap to negative (h - 180)
        h_unwrapped = np.where(h_raw >= 90.0, h_raw - 180.0, h_raw)
        h_adj = float(np.mean(h_unwrapped))
        s_mean = float(np.mean(s_raw))
        v_mean = float(np.mean(v_raw))
        h_raw_mean = float(np.mean(h_raw))

        return (h_adj, s_mean, v_mean, h_raw_mean)

    def trigger_inspection(self, distance):
        if self.latest_image is None:
            rospy.logwarn("plant_inspector: no image yet, skipping.")
            return

        plant, match_dist = self.find_nearest_plant()
        if plant is None:
            rospy.logwarn("plant_inspector: no plant found in col %s within match_radius (closest %.2fm).",
                          str(self.target_column), match_dist)
            return

        if plant['id'] in self.inspected_ids:
            return

        features = self.extract_hsv_features(self.latest_image)
        if features is None:
            rospy.logwarn("plant_inspector: not enough colorful pixels, skipping.")
            return

        h_adj, s, v, h_raw = features
        features_scaled = self.scaler.transform([[h_adj, s, v]])
        predicted = self.model.predict(features_scaled)[0]

        self.inspected_ids.add(plant['id'])
        correct = (predicted == plant['category'])

        def describe(cat):
            if cat == 'healthy_ripe':    return "HEALTHY",   "RIPE"
            elif cat == 'healthy_unripe': return "HEALTHY",   "NOT RIPE"
            else:                         return "UNHEALTHY", "N/A"

        ph, pr = describe(predicted)
        th, tr = describe(plant['category'])
        verdict = "CORRECT" if correct else "WRONG"

        report = (f"Plant #{plant['id']}  ->  "
                  f"Health: {ph}, Ripeness: {pr}  "
                  f"[Actual: {th}, {tr}]  ({verdict})  "
                  f"[CALIBRATION hsv=({h_raw:.1f},{s:.1f},{v:.1f}) "
                  f"true={plant['category']}]")

        rospy.loginfo("plant_inspector: %s", report)
        self.report_pub.publish(String(report))


if __name__ == '__main__':
    try:
        PlantInspector()
    except rospy.ROSInterruptException:
        pass
