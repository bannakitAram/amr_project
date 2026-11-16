#!/usr/bin/env python3
"""Map zones for the goal_picker page, as small JSON instead of the masks.

rosboard sends OccupancyGrids to the browser as a JPEG render, which loses
the exact mask values (speed 30 % vs 50 %). This node reads the same mask
files the Nav2 costmap filters use (launched by navigation.launch.py only
when the map has them) and publishes the zones as run-length rectangles:

  /amr_zones  std_msgs/String JSON, every 2 s:
    {"zones": [[x0, y0, x1, y1, kind], ...]}   map frame, metres
    one rectangle per horizontal run of equal cells, sampled every CELL_M;
    kind = "keepout" or the speed % (e.g. 30, 50).

Display only - the costmap filters are what act on the robot.
"""
import json
import os

import numpy as np
import rclpy
import rclpy.executors
import yaml
from PIL import Image
from rclpy.node import Node
from std_msgs.msg import String

CELL_M = 0.1   # display resolution (masks are 0.05 m: 4x fewer cells to send)
KEEPOUT = -1


def load_mask(yaml_path):
    with open(yaml_path) as f:
        meta = yaml.safe_load(f)
    img = os.path.join(os.path.dirname(os.path.abspath(yaml_path)), meta['image'])
    return meta, np.array(Image.open(img).convert('L'))


def zone_runs(kinds, meta):
    """kinds: int array, 0 = no zone, KEEPOUT or speed %. Returns rectangles."""
    res = float(meta['resolution'])
    ox, oy = float(meta['origin'][0]), float(meta['origin'][1])
    step = max(1, int(round(CELL_M / res)))
    h = kinds.shape[0]
    k = kinds[::step, ::step]
    cell = res * step
    out = []
    for r in range(k.shape[0]):
        row = k[r]
        y1 = oy + (h - r * step) * res        # image row 0 = top = max y
        c = 0
        while c < row.shape[0]:
            v = int(row[c])
            if v == 0:
                c += 1
                continue
            c0 = c
            while c < row.shape[0] and row[c] == v:
                c += 1
            out.append([round(ox + c0 * cell, 2), round(y1 - cell, 2),
                        round(ox + c * cell, 2), round(y1, 2),
                        'keepout' if v == KEEPOUT else v])
    return out


class ZoneInfo(Node):
    def __init__(self):
        super().__init__('zone_info')
        keepout = self.declare_parameter('keepout_yaml', '').value
        speed = self.declare_parameter('speed_yaml', '').value
        zones = []
        try:
            if keepout:
                meta, a = load_mask(keepout)
                zones += zone_runs(np.where(a < 128, KEEPOUT, 0), meta)
            if speed:
                meta, a = load_mask(speed)
                zones += zone_runs(np.where((a >= 1) & (a <= 100), a, 0).astype(np.int16), meta)
        except (OSError, KeyError, ValueError, yaml.YAMLError) as e:
            self.get_logger().error('zone masks: %s' % e)
        self.msg = String(data=json.dumps({'zones': zones}))
        self.get_logger().info('zones: %d rectangles' % len(zones))
        self.pub = self.create_publisher(String, '/amr_zones', 10)
        self.create_timer(2.0, lambda: self.pub.publish(self.msg))


def main():
    rclpy.init()
    node = ZoneInfo()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
