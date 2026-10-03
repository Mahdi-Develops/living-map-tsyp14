"""Outside Network Area: the ONLY bridge between the disconnected zone and the
outside world. Four roles, exactly as specified:
  1. RECEIVE  - takes in what the Writer produced at the entrance boundary.
  2. TRANSLATE - private robot (x,y) -> real-world GPS lat/lon.
  3. CARRY    - forwards translated data to a distant Command Post (wireless/
                satellite link; simulated here as a ROS topic / rosbridge-
                reachable channel, per the challenge Q&A).
  4. BRIEF    - prepares and delivers the Executor's mission before entry.

Neither the Writer nor the Executor ever publishes to the Command Post
directly, and the Command Post never publishes to the robots directly -
everything funnels through this single node.
"""
import json
import math
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from .beacon_message import Beacon

ORIGIN_LAT = 36.8065
ORIGIN_LON = 10.1815
M_PER_DEG_LAT = 111_320.0


def local_to_gps(x: float, y: float):
    lat = ORIGIN_LAT + (y / M_PER_DEG_LAT)
    m_per_deg_lon = M_PER_DEG_LAT * math.cos(math.radians(ORIGIN_LAT))
    lon = ORIGIN_LON + (x / m_per_deg_lon)
    return lat, lon


MISSION_FOR = {'hazard': 'seal', 'victim': 'extract'}


class OutsideNetworkNode(Node):
    def __init__(self):
        super().__init__('outside_network_node')
        self.create_subscription(String, '/writer/to_outside_network', self.on_receive, 10)
        self.create_subscription(String, '/executor/to_outside_network', self.on_executor_report, 10)
        self.cp_pub = self.create_publisher(String, '/command_post/beacons', 10)
        self.cp_report_pub = self.create_publisher(String, '/command_post/mission_report', 10)
        self.brief_pub = self.create_publisher(String, '/outside_network/executor_brief', 10)
        self.get_logger().info('Outside Network Area online - bridging inside <-> outside.')

    def on_receive(self, msg: String):
        payload = json.loads(msg.data)
        beacons = [Beacon.from_dict(b) for b in payload['beacons']]
        self.get_logger().info(f'[RECEIVE] {len(beacons)} beacon(s) handed off by Writer at entrance.')

        translated = []
        for b in beacons:
            lat, lon = local_to_gps(b.x, b.y)
            translated.append({
                **b.__dict__,
                'lat': lat, 'lon': lon,
                'freshness': b.freshness(),
            })
        self.get_logger().info(f'[TRANSLATE] {len(translated)} beacon(s) -> real-world GPS.')

        cp_msg = {
            'source': 'outside_network',
            'sent_at': time.time(),
            'origin_local': payload['entrance_origin_local'],
            'origin_gps': {'lat': ORIGIN_LAT, 'lon': ORIGIN_LON},
            'beacons': translated,
        }
        self.cp_pub.publish(String(data=json.dumps(cp_msg)))
        self.get_logger().info('[CARRY] beacon data forwarded to distant Command Post.')

        brief = {
            'mission_id': f'M-{int(time.time())}',
            'issued_at': time.time(),
            'targets': [
                {
                    'beacon_id': b.id, 'what': b.what, 'action': MISSION_FOR.get(b.what, 'inspect'),
                    'x': b.x, 'y': b.y, 'when': b.when, 'version': b.version,
                }
                for b in beacons
            ],
        }
        self.brief_pub.publish(String(data=json.dumps(brief)))
        self.get_logger().info(
            f"[BRIEF] mission {brief['mission_id']} ({len(brief['targets'])} target(s)) sent to Executor.")

    def on_executor_report(self, msg: String):
        report = json.loads(msg.data)
        self.get_logger().info(f"[RECEIVE] Executor mission report: {report.get('status')}")
        self.cp_report_pub.publish(String(data=json.dumps({
            'source': 'outside_network', 'relayed_at': time.time(), 'report': report,
        })))
        self.get_logger().info('[CARRY] mission report forwarded to Command Post.')


def main():
    rclpy.init()
    node = OutsideNetworkNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
