#!/usr/bin/env python3
"""
scout_logger.py
----------------
ROS node: subscribes to /plant_inspection_report (std_msgs/String) and
writes every inspection result to a CSV log file. Prints a running
accuracy summary to the terminal and a final summary table on shutdown.
"""

import csv
import os
import re
from datetime import datetime

import rospkg
import rospy
from std_msgs.msg import String

REPORT_PATTERN = re.compile(
    r"Plant\s*#(?P<id>\d+)\s*->\s*"
    r"Health:\s*(?P<pred_health>\w+),\s*Ripeness:\s*(?P<pred_ripeness>[^\[]+?)\s*"
    r"\[Actual:\s*(?P<actual_health>\w+),\s*(?P<actual_ripeness>[^\]]+?)\]\s*"
    r"\((?P<verdict>\w+)\)"
    r"(?:\s*\[CALIBRATION\s+hsv=\((?P<hsv>[^)]+)\)\s+true_category=(?P<true_category>\w+)\])?"
)

CSV_HEADER = [
    "timestamp",
    "plant_id",
    "predicted_health",
    "predicted_ripeness",
    "actual_health",
    "actual_ripeness",
    "correct",
]


class ScoutLogger(object):
    def __init__(self):
        rospy.init_node("scout_logger", anonymous=False)

        log_filename = rospy.get_param("~log_filename", "scout_log.csv")
        default_log_dir = os.path.join(
            rospkg.RosPack().get_path("farm_field_scout"), "data"
        )
        log_dir = rospy.get_param("~log_dir", default_log_dir)
        self.print_summary = rospy.get_param("~print_summary", True)

        if not os.path.isdir(log_dir):
            os.makedirs(log_dir)
        self.log_path = os.path.join(log_dir, log_filename)

        self.csv_file = open(self.log_path, "w")
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(CSV_HEADER)
        self.csv_file.flush()

        self.rows = []
        self.seen_ids = set()

        rospy.Subscriber(
            "/plant_inspection_report", String, self.report_callback, queue_size=10
        )
        rospy.on_shutdown(self.print_final_summary)

        rospy.loginfo("[scout_logger] Logging inspections to: %s", self.log_path)

    def report_callback(self, msg):
        match = REPORT_PATTERN.search(msg.data)
        if not match:
            rospy.logwarn(
                "[scout_logger] Could not parse report, skipping: %s", msg.data
            )
            return

        plant_id = match.group("id")

        if plant_id in self.seen_ids:
            rospy.logwarn(
                "[scout_logger] Duplicate report for Plant #%s, skipping.", plant_id
            )
            return
        self.seen_ids.add(plant_id)

        correct = match.group("verdict").upper() == "CORRECT"
        row = [
            datetime.now().isoformat(timespec="seconds"),
            plant_id,
            match.group("pred_health"),
            match.group("pred_ripeness"),
            match.group("actual_health"),
            match.group("actual_ripeness"),
            correct,
        ]
        self.rows.append(row)
        self.csv_writer.writerow(row)
        self.csv_file.flush()

        if self.print_summary:
            correct_count = sum(1 for r in self.rows if r[-1])
            rospy.loginfo(
                "[scout_logger] Logged Plant #%s (%d/%d correct so far)",
                plant_id,
                correct_count,
                len(self.rows),
            )

    def print_final_summary(self):
        if not self.rows:
            rospy.loginfo("[scout_logger] No inspections were logged this run.")
            self.csv_file.close()
            return

        total = len(self.rows)
        correct = sum(1 for r in self.rows if r[-1])
        accuracy = 100.0 * correct / total

        rospy.loginfo("=" * 50)
        rospy.loginfo("[scout_logger] PATROL SUMMARY")
        rospy.loginfo("  Plants logged : %d", total)
        rospy.loginfo("  Correct       : %d", correct)
        rospy.loginfo("  Accuracy      : %.1f%%", accuracy)
        rospy.loginfo("  CSV saved to  : %s", self.log_path)
        rospy.loginfo("=" * 50)

        self.csv_file.close()


if __name__ == "__main__":
    try:
        ScoutLogger()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
