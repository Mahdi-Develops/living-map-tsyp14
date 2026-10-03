"""Command Post: receives translated beacon data and mission reports ONLY from
the Outside Network Area (never talks to the robots directly), and renders a
self-contained, auto-refreshing live map to disk.
"""
import json
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

MAP_PATH = '/root/living_map_ws/live_map.html'

COLOR = {'hazard': '#d1392b', 'victim': '#2b5fd1', 'exit': '#2bb673', 'junction': '#9b59b6'}
FRESH_OPACITY = {'fresh': 1.0, 'aging': 0.6, 'stale': 0.3}

PAGE_TEMPLATE = """<!doctype html><html><head><meta charset="utf-8">
<meta http-equiv="refresh" content="2">
<title>Living Map - Command Post</title>
<style>
 body{{font-family:system-ui,sans-serif;background:#0b1626;color:#e8edf5;margin:0;padding:20px}}
 h1{{font-size:20px}}
 .meta{{color:#8fa3bf;font-size:13px;margin-bottom:16px}}
 svg{{background:#101b2d;border:1px solid #233350;border-radius:8px}}
 .legend span{{display:inline-block;width:10px;height:10px;border-radius:50%;margin-right:6px}}
 table{{border-collapse:collapse;margin-top:16px;font-size:13px}}
 td,th{{border:1px solid #233350;padding:4px 8px;text-align:left}}
</style></head><body>
<h1>Living Map &mdash; Command Post</h1>
<div class="meta">Last update: {ts} &middot; origin GPS {origin_lat:.5f}, {origin_lon:.5f} &middot; {n} beacon(s) on record</div>
<svg width="640" height="420" viewBox="-11 -11 22 22">
 <rect x="-11" y="-11" width="22" height="22" fill="#0e1a2c"/>
 <line x1="-11" y1="0" x2="11" y2="0" stroke="#233350"/>
 <line x1="0" y1="-11" x2="0" y2="11" stroke="#233350"/>
 <circle cx="0" cy="0" r="0.3" fill="#e8edf5"/>
 <text x="0.4" y="0.4" fill="#8fa3bf" font-size="0.6">entrance</text>
 {markers}
</svg>
<div class="legend">
 <span style="background:#d1392b"></span>hazard&nbsp;&nbsp;
 <span style="background:#2b5fd1"></span>victim&nbsp;&nbsp;
 &mdash; opacity = freshness (solid=fresh, faint=stale)
</div>
<table><tr><th>id</th><th>what</th><th>lat</th><th>lon</th><th>freshness</th><th>age (s)</th></tr>
{rows}
</table>
</body></html>"""


class CommandPostNode(Node):
    def __init__(self):
        super().__init__('command_post_node')
        self.create_subscription(String, '/command_post/beacons', self.on_beacons, 10)
        self.create_subscription(String, '/command_post/mission_report', self.on_report, 10)
        self.beacons = {}
        self.origin = {'lat': 0.0, 'lon': 0.0}
        self.render()
        self.get_logger().info(f'Command Post online - live map at {MAP_PATH}')

    def on_beacons(self, msg: String):
        payload = json.loads(msg.data)
        self.origin = payload['origin_gps']
        for b in payload['beacons']:
            self.beacons[b['id']] = b
        self.get_logger().info(f"Live map updated: {len(self.beacons)} beacon(s) known.")
        self.render()

    def on_report(self, msg: String):
        payload = json.loads(msg.data)
        self.get_logger().info(f"Mission report received via Outside Network: {payload}")

    def render(self):
        now = time.time()
        markers, rows = [], []
        for b in self.beacons.values():
            x, y = b['x'], b['y']
            age = now - b['when']
            fresh = 'fresh' if age < 300 else ('aging' if age < 900 else 'stale')
            color = COLOR.get(b['what'], '#999')
            op = FRESH_OPACITY[fresh]
            markers.append(
                f'<circle cx="{x:.2f}" cy="{-y:.2f}" r="0.35" fill="{color}" opacity="{op:.2f}"/>'
                f'<text x="{x+0.4:.2f}" y="{-y:.2f}" fill="#e8edf5" font-size="0.5">{b["id"]}</text>')
            rows.append(
                f"<tr><td>{b['id']}</td><td>{b['what']}</td><td>{b['lat']:.5f}</td>"
                f"<td>{b['lon']:.5f}</td><td>{fresh}</td><td>{age:.0f}</td></tr>")
        html = PAGE_TEMPLATE.format(
            ts=time.strftime('%H:%M:%S'), origin_lat=self.origin['lat'], origin_lon=self.origin['lon'],
            n=len(self.beacons), markers=''.join(markers), rows=''.join(rows) or '<tr><td colspan=6>none yet</td></tr>')
        try:
            with open(MAP_PATH, 'w') as f:
                f.write(html)
        except OSError as e:
            self.get_logger().warn(f'could not write live map: {e}')


def main():
    rclpy.init()
    node = CommandPostNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
