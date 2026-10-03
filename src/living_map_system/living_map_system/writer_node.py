"""Writer Robot (drone): explores autonomously, senses events, drops beacons,
and - only once back at the entrance boundary - hands its beacon log to the
Outside Network Area over a short-range link. It never talks to the command
post directly.
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
from gazebo_msgs.srv import SpawnEntity

from .beacon_message import Beacon

ENTRANCE_X, ENTRANCE_Y = -8.5, -8.5
SENSE_RADIUS = 1.3
EXPLORE_SECONDS = 90.0

# Deterministic boustrophedon sweep over the interior - guarantees coverage of
# all event markers regardless of random-wander luck. The drone cruises at
# z=1.5m, above the 1.2m wall height, so straight-line legs are unobstructed.
SWEEP_WAYPOINTS = [
    (-7, -8), (-7, 8), (-3, 8), (-3, -8),
    (1, -8), (1, 8), (5, 8), (5, -8),
    (8, -8), (8, 8),
]
WAYPOINT_TOL = 0.5

BEACON_SDF = """<?xml version="1.0"?>
<sdf version="1.6"><model name="{name}"><static>true</static>
<link name="link"><visual name="v"><geometry><cylinder><radius>0.12</radius><length>0.15</length></cylinder></geometry>
<material><ambient>1 0.85 0 1</ambient><diffuse>1 0.85 0 1</diffuse></material></visual></link>
</model></sdf>"""


class WriterNode(Node):
    def __init__(self):
        super().__init__('writer_node')
        self.cmd_pub = self.create_publisher(Twist, '/writer/cmd_vel', 10)
        self.tx_pub = self.create_publisher(String, '/writer/to_outside_network', 10)
        self.create_subscription(LaserScan, '/writer/scan', self.on_scan, 10)
        self.create_subscription(Odometry, '/writer/odom', self.on_odom, 10)
        self.create_subscription(ModelStates, '/gazebo/model_states', self.on_models, 10)
        self.spawn_cli = self.create_client(SpawnEntity, '/spawn_entity')

        self.pose = (ENTRANCE_X, ENTRANCE_Y)
        self.scan_ranges = []
        self.detected_ids = set()
        self.beacon_log = []
        self.start_t = time.monotonic()
        self.state = 'EXPLORE'
        self.transmitted = False
        self.wp_idx = 0

        self.timer = self.create_timer(0.2, self.control_loop)
        self.get_logger().info('Writer drone airborne - exploring GPS-denied zone.')

    def on_scan(self, msg: LaserScan):
        self.scan_ranges = msg.ranges

    def on_odom(self, msg: Odometry):
        p = msg.pose.pose.position
        self.pose = (p.x, p.y)

    def on_models(self, msg: ModelStates):
        if self.state != 'EXPLORE':
            return
        for name, pose in zip(msg.name, msg.pose):
            if not name.startswith('event_') or name in self.detected_ids:
                continue
            dx = pose.position.x - self.pose[0]
            dy = pose.position.y - self.pose[1]
            if math.hypot(dx, dy) <= SENSE_RADIUS:
                self.detected_ids.add(name)
                what = 'hazard' if 'hazard' in name else 'victim'
                b = Beacon(
                    id=f'B{len(self.beacon_log)+1}',
                    what=what,
                    x=pose.position.x, y=pose.position.y,
                    distance=math.hypot(pose.position.x - ENTRANCE_X, pose.position.y - ENTRANCE_Y),
                    when=time.time(),
                )
                self.beacon_log.append(b)
                self.get_logger().info(f'EVENT DETECTED [{what}] at ({b.x:.1f},{b.y:.1f}) -> dropping beacon {b.id}')
                self.drop_beacon(b)

    def drop_beacon(self, b: Beacon):
        if not self.spawn_cli.service_is_ready():
            return
        req = SpawnEntity.Request()
        req.name = f'beacon_{b.id}'
        req.xml = BEACON_SDF.format(name=req.name)
        req.initial_pose.position.x = b.x
        req.initial_pose.position.y = b.y
        req.initial_pose.position.z = 0.1
        self.spawn_cli.call_async(req)

    def control_loop(self):
        elapsed = time.monotonic() - self.start_t

        if self.state == 'EXPLORE':
            if elapsed > EXPLORE_SECONDS or len(self.detected_ids) >= 3 or self.wp_idx >= len(SWEEP_WAYPOINTS):
                self.state = 'RETURN'
                self.get_logger().info('Exploration sweep complete -> returning to entrance.')
                return
            wx, wy = SWEEP_WAYPOINTS[self.wp_idx]
            dx, dy = wx - self.pose[0], wy - self.pose[1]
            dist = math.hypot(dx, dy)
            if dist < WAYPOINT_TOL:
                self.wp_idx += 1
                return
            tw = Twist()
            yaw_err = math.atan2(dy, dx)
            tw.linear.x = min(0.9, dist)
            tw.angular.z = max(-1.2, min(1.2, 2.0 * yaw_err))
            self.cmd_pub.publish(tw)

        elif self.state == 'RETURN':
            dx = ENTRANCE_X - self.pose[0]
            dy = ENTRANCE_Y - self.pose[1]
            dist = math.hypot(dx, dy)
            if dist < 0.6:
                self.cmd_pub.publish(Twist())
                self.state = 'TRANSMIT'
                return
            tw = Twist()
            yaw_err = math.atan2(dy, dx)
            tw.linear.x = min(0.9, dist)
            tw.angular.z = max(-1.2, min(1.2, 2.0 * yaw_err))
            self.cmd_pub.publish(tw)

        elif self.state == 'TRANSMIT' and not self.transmitted:
            payload = {
                'robot': 'writer',
                'entrance_origin_local': [ENTRANCE_X, ENTRANCE_Y],
                'beacons': [b.__dict__ for b in self.beacon_log],
            }
            self.tx_pub.publish(String(data=json.dumps(payload)))
            self.transmitted = True
            self.get_logger().info(
                f'Docked at entrance -> handed {len(self.beacon_log)} beacon(s) to Outside Network Area.')
            self.state = 'DONE'


def main():
    rclpy.init()
    node = WriterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
