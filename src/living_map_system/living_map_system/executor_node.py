"""Executor Robot (ground rover): briefed by the Outside Network Area before
entry, then navigates purely by inherited beacon coordinates (no GPS inside).
Fresh beacons are trusted outright; stale ones are verified against the
robot's own sensing (ground-truth proximity check in this simulation) before
acting.
"""
import json
import math
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from gazebo_msgs.msg import ModelStates

ENTRANCE_X, ENTRANCE_Y = -8.5, -8.0
ARRIVE_TOL = 0.5


class ExecutorNode(Node):
    def __init__(self):
        super().__init__('executor_node')
        self.cmd_pub = self.create_publisher(Twist, '/executor/cmd_vel', 10)
        self.report_pub = self.create_publisher(String, '/executor/to_outside_network', 10)
        self.create_subscription(String, '/outside_network/executor_brief', self.on_brief, 10)
        self.create_subscription(Odometry, '/executor/odom', self.on_odom, 10)
        self.create_subscription(LaserScan, '/executor/scan', self.on_scan, 10)
        self.create_subscription(ModelStates, '/gazebo/model_states', self.on_models, 10)

        self.pose = (ENTRANCE_X, ENTRANCE_Y)
        self.scan_ranges = []
        self.model_positions = {}
        self.mission = None
        self.targets = []
        self.current_idx = 0
        self.verify_until = 0.0
        self.completed = []

        self.timer = self.create_timer(0.2, self.control_loop)
        self.get_logger().info('Executor standing by at entrance - awaiting mission briefing.')

    def on_brief(self, msg: String):
        if self.mission is not None:
            return
        self.mission = json.loads(msg.data)
        self.targets = sorted(self.mission['targets'],
                               key=lambda t: math.hypot(t['x'] - ENTRANCE_X, t['y'] - ENTRANCE_Y))
        self.get_logger().info(
            f"Mission {self.mission['mission_id']} received: {len(self.targets)} target(s). Entering space.")

    def on_odom(self, msg: Odometry):
        p = msg.pose.pose.position
        self.pose = (p.x, p.y)

    def on_scan(self, msg: LaserScan):
        self.scan_ranges = msg.ranges

    def on_models(self, msg: ModelStates):
        for name, pose in zip(msg.name, msg.pose):
            if name.startswith('event_'):
                self.model_positions[name] = (pose.position.x, pose.position.y)

    def _freshness(self, target) -> str:
        age = time.time() - target['when']
        if age < 300:
            return 'fresh'
        if age < 900:
            return 'aging'
        return 'stale'

    def _verified_by_sensor(self, target) -> bool:
        kind = 'hazard' if target['what'] == 'hazard' else 'victim'
        for name, (x, y) in self.model_positions.items():
            if kind in name and math.hypot(x - target['x'], y - target['y']) < 1.0:
                return True
        return False

    def control_loop(self):
        if self.mission is None:
            return
        if self.current_idx >= len(self.targets):
            if self.completed and len(self.completed) == len(self.targets) and not getattr(self, '_reported', False):
                self._reported = True
                self.report_pub.publish(String(data=json.dumps({
                    'mission_id': self.mission['mission_id'], 'status': 'complete',
                    'completed': self.completed, 'reported_at': time.time(),
                })))
                self.get_logger().info('All targets actioned -> report handed to Outside Network Area.')
            self.cmd_pub.publish(Twist())
            return

        target = self.targets[self.current_idx]
        fresh = self._freshness(target)

        if fresh == 'stale' and time.monotonic() < self.verify_until:
            self.cmd_pub.publish(Twist())
            return
        if fresh == 'stale' and self.verify_until == 0.0:
            self.get_logger().warn(f"Beacon {target['beacon_id']} is STALE - verifying with own sensors first.")
            self.verify_until = time.monotonic() + 2.0
            if not self._verified_by_sensor(target):
                self.get_logger().warn(f"Could not verify {target['beacon_id']} - skipping, trusting own senses.")
                self.current_idx += 1
                self.verify_until = 0.0
            return

        dx = target['x'] - self.pose[0]
        dy = target['y'] - self.pose[1]
        dist = math.hypot(dx, dy)
        front = min(self.scan_ranges[170:190], default=10.0) if self.scan_ranges else 10.0

        if dist < ARRIVE_TOL:
            self.cmd_pub.publish(Twist())
            self.get_logger().info(
                f"Arrived at {target['beacon_id']} ({target['what']}) -> executing '{target['action']}'.")
            self.completed.append(target['beacon_id'])
            self.current_idx += 1
            self.verify_until = 0.0
            return

        tw = Twist()
        yaw_err = math.atan2(dy, dx)
        tw.linear.x = min(0.5, dist)
        tw.angular.z = max(-1.0, min(1.0, yaw_err))
        if front < 0.9:
            tw.linear.x = 0.0
            tw.angular.z = 0.7
        self.cmd_pub.publish(tw)


def main():
    rclpy.init()
    node = ExecutorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
