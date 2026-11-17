#!/usr/bin/env python3
"""Map zones (keep-out / speed %) for Nav2's costmap filters, editable live.

Owns the two zone masks of the map Navigation runs on and serves them to the
KeepoutFilter / SpeedFilter that navigation.launch.py always loads:

  /keepout_filter_mask, /speed_filter_mask   nav_msgs/OccupancyGrid,
      transient_local, re-published after every edit (the filters pick the
      new mask up at once - no Navigation restart)
  /keepout_costmap_filter_info, /speed_costmap_filter_info
      nav2_msgs/CostmapFilterInfo, transient_local (type 0 keep-out,
      type 1 speed in %, base 0, multiplier 1)

Files (same as `map_edit.py zones-png / zones-build`, next to the map yaml):
  <map>_keepout.pgm/.yaml  trinary, black = keep-out
  <map>_speed.pgm/.yaml    raw, pixel = speed %, 255 = no limit
A map without the files simply has no zones (empty masks).

Edits from the goal_picker page (via rosboard.py, validated there and here):
  /rosboard_zone_cmd  std_msgs/String JSON
      {"action": "add"|"erase", "kind": "keepout"|30|50 (add only),
       "shape": "poly",  "points": [[x, y], ...]}          filled polygon (3+)
       "shape": "brush", "points": [[x, y], ...], "width": m   painted stroke
  map frame, metres (the page sends a rectangle as a 4-point polygon). "add"
  sets the cells to the kind (replacing any other kind there), "erase"
  clears both kinds. Every edit is written to the mask files at once (no
  save step); the first edit of a session copies the old files to .bak.

Out for the page:
  /amr_zones  std_msgs/String JSON every 2 s and after each edit:
      {"map": name, "zones": [[x0, y0, x1, y1, kind], ...], "msg": text}
      rectangles = horizontal runs of equal cells at CELL_M; msg reports the
      last edit, incl. stations / route points that ended up in keep-out.

Not a safety function (ISO 3691-4): zones depend on localization.
"""
import json
import os
import shutil

import numpy as np
import rclpy
import rclpy.executors
import yaml
from nav2_msgs.msg import CostmapFilterInfo
from nav_msgs.msg import OccupancyGrid
from PIL import Image, ImageDraw
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String

CELL_M = 0.1          # display resolution for /amr_zones
SPEED_KINDS = (30, 50)
SPEED_NONE = 255      # speed mask pixel: no limit
MAX_SIDE_M = 60.0     # largest shape extent accepted from the page
MAX_POINTS = 400
BRUSH_MIN_M, BRUSH_MAX_M = 0.1, 3.0
MAX_RECTS = 4000      # /amr_zones size guard


def read_yaml(path):
    with open(path) as f:
        return yaml.safe_load(f)


def read_image(yaml_path, meta):
    img = os.path.join(os.path.dirname(os.path.abspath(yaml_path)), meta['image'])
    return np.array(Image.open(img).convert('L'))


class ZoneManager(Node):
    def __init__(self):
        super().__init__('zone_manager')
        self.map_yaml = os.path.abspath(self.declare_parameter('map_yaml', '').value)
        self.stem = os.path.splitext(self.map_yaml)[0]
        self.map_name = os.path.basename(self.stem)
        meta = read_yaml(self.map_yaml)
        self.res = float(meta['resolution'])
        self.ox, self.oy = float(meta['origin'][0]), float(meta['origin'][1])
        self.h, self.w = read_image(self.map_yaml, meta).shape
        self.meta = meta

        # zone grid in image layout (row 0 = top): 0 none, -1 keep-out, else speed %
        self.zone = np.zeros((self.h, self.w), np.int16)
        self.load()
        self.backed_up = False
        self.msg = ''

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.keep_pub = self.create_publisher(OccupancyGrid, '/keepout_filter_mask', latched)
        self.speed_pub = self.create_publisher(OccupancyGrid, '/speed_filter_mask', latched)
        info_k = self.create_publisher(CostmapFilterInfo, '/keepout_costmap_filter_info', latched)
        info_s = self.create_publisher(CostmapFilterInfo, '/speed_costmap_filter_info', latched)
        for pub, ftype, topic in ((info_k, 0, '/keepout_filter_mask'),
                                  (info_s, 1, '/speed_filter_mask')):
            m = CostmapFilterInfo()
            m.header.frame_id = 'map'
            m.header.stamp = self.get_clock().now().to_msg()
            m.type, m.filter_mask_topic, m.base, m.multiplier = ftype, topic, 0.0, 1.0
            pub.publish(m)
        self.zones_pub = self.create_publisher(String, '/amr_zones', 10)
        self.create_subscription(String, '/rosboard_zone_cmd', self.on_cmd, 10)
        self.publish_masks()
        self.create_timer(2.0, self.publish_zones)
        self.get_logger().info('map %s: %d keep-out, %d speed cells' % (
            self.map_name, int((self.zone == -1).sum()), int((self.zone > 0).sum())))

    # ---- files ------------------------------------------------------------
    def paths(self, kind):
        return self.stem + '_%s.yaml' % kind, self.stem + '_%s.pgm' % kind

    def load(self):
        for kind in ('speed', 'keepout'):   # keep-out last: it wins
            y_path, _ = self.paths(kind)
            if not os.path.exists(y_path):
                continue
            try:
                meta = read_yaml(y_path)
                a = read_image(y_path, meta)
            except (OSError, KeyError, ValueError, yaml.YAMLError) as e:
                self.get_logger().error('%s: %s' % (y_path, e))
                continue
            if a.shape != self.zone.shape:
                self.get_logger().error('%s: size differs from the map - ignored' % y_path)
                continue
            if kind == 'keepout':
                self.zone[a < 128] = -1
            else:
                ok = (a >= 1) & (a <= 100)
                self.zone[ok] = a[ok]

    def save(self):
        if not self.backed_up:
            for kind in ('keepout', 'speed'):
                for p in self.paths(kind):
                    if os.path.exists(p):
                        shutil.copy(p, p + '.bak')
            self.backed_up = True
        imgs = {'keepout': (np.where(self.zone == -1, 0, 254).astype(np.uint8), 'trinary',
                            (self.zone == -1).any()),
                'speed': (np.where(self.zone > 0, self.zone, SPEED_NONE).astype(np.uint8), 'raw',
                          (self.zone > 0).any())}
        for kind, (img, mode, has) in imgs.items():
            y_path, p_path = self.paths(kind)
            if not has:
                for p in (y_path, p_path):
                    if os.path.exists(p):
                        os.remove(p)
                continue
            Image.fromarray(img).save(p_path + '.tmp.pgm')
            os.replace(p_path + '.tmp.pgm', p_path)
            meta = {'image': os.path.basename(p_path), 'mode': mode,
                    'resolution': self.res, 'origin': self.meta['origin'], 'negate': 0,
                    'occupied_thresh': 0.65, 'free_thresh': 0.25}
            with open(y_path + '.tmp', 'w') as f:
                yaml.safe_dump(meta, f, default_flow_style=None, sort_keys=False)
            os.replace(y_path + '.tmp', y_path)

    # ---- edits ------------------------------------------------------------
    def to_px(self, x, y):
        """map (m) -> image pixel (col, row), row 0 = top."""
        return (x - self.ox) / self.res, self.h - (y - self.oy) / self.res

    def shape_cells(self, shape, pts, width):
        """Boolean mask of the cells covered by the polygon / brush stroke."""
        img = Image.new('L', (self.w, self.h), 0)
        d = ImageDraw.Draw(img)
        px = [self.to_px(x, y) for x, y in pts]
        if shape == 'poly':
            d.polygon(px, fill=1, outline=1)
        else:
            r = max(0.5, width / self.res / 2.0)
            if len(px) > 1:
                d.line(px, fill=1, width=int(round(2 * r)), joint='curve')
            for cx, cy in px:          # round ends and joints
                d.ellipse([cx - r, cy - r, cx + r, cy + r], fill=1)
        return np.array(img, bool)

    def on_cmd(self, msg):
        try:
            c = json.loads(msg.data)
            action, kind, shape = c.get('action'), c.get('kind'), c.get('shape')
            pts = [(float(p[0]), float(p[1])) for p in c.get('points', [])]
            width = float(c.get('width', 0.5))
        except (ValueError, TypeError, KeyError, IndexError, AttributeError):
            return
        if action not in ('add', 'erase') or shape not in ('poly', 'brush'):
            return
        if action == 'add' and kind not in ('keepout',) + SPEED_KINDS:
            return
        if not pts or len(pts) > MAX_POINTS or not np.all(np.isfinite(pts)) or \
                (shape == 'poly' and len(pts) < 3) or not np.isfinite(width):
            return
        width = min(BRUSH_MAX_M, max(BRUSH_MIN_M, width))
        xs, ys = [p[0] for p in pts], [p[1] for p in pts]
        if max(xs) - min(xs) > MAX_SIDE_M or max(ys) - min(ys) > MAX_SIDE_M:
            self.report('โซนใหญ่เกิน %.0f ม. - ไม่บันทึก' % MAX_SIDE_M)
            return
        sel = self.shape_cells(shape, pts, width)
        if not sel.any():
            self.report('โซนอยู่นอกแผนที่ - ไม่บันทึก')
            return
        self.zone[sel] = 0 if action == 'erase' else (-1 if kind == 'keepout' else int(kind))
        try:
            self.save()
        except OSError as e:
            self.report('บันทึกไฟล์โซนไม่สำเร็จ: %s' % e)
            return
        self.publish_masks()
        what = 'ลบโซน' if action == 'erase' else (
            'เพิ่มโซนห้ามเข้า' if kind == 'keepout' else 'เพิ่มโซนความเร็ว %d%%' % kind)
        area = sel.sum() * self.res * self.res
        warn = self.keepout_warnings()
        self.get_logger().info('%s (%s, %.2f m^2)' % (what, shape, area))
        self.report('%s %.1f ม.² แล้ว (บันทึกไฟล์แล้ว)' % (what, area) +
                    (' - ⚠ ' + '; '.join(warn) if warn else ''))

    def keepout_warnings(self):
        keep = self.zone == -1

        def inside(x, y):
            c = int((x - self.ox) / self.res)
            r = self.h - 1 - int((y - self.oy) / self.res)
            return 0 <= r < self.h and 0 <= c < self.w and keep[r, c]
        out = []
        try:
            for st in (read_yaml(self.stem + '_stations.yaml') or {}).get('stations', []):
                if inside(float(st['x']), float(st['y'])):
                    out.append('Station %s อยู่ในโซนห้ามเข้า' % st['name'])
        except (OSError, KeyError, TypeError, ValueError, yaml.YAMLError):
            pass
        try:
            for rt in (read_yaml(self.stem + '_routes.yaml') or {}).get('routes', []):
                if any(inside(p[0], p[1]) for p in rt.get('poses', [])):
                    out.append('เส้นทาง %s→%s ผ่านโซนห้ามเข้า' % (rt.get('from'), rt.get('to')))
        except (OSError, KeyError, TypeError, ValueError, IndexError, yaml.YAMLError):
            pass
        return out

    # ---- outputs ----------------------------------------------------------
    def grid(self, data):
        g = OccupancyGrid()
        g.header.frame_id = 'map'
        g.header.stamp = self.get_clock().now().to_msg()
        g.info.resolution = self.res
        g.info.width, g.info.height = self.w, self.h
        g.info.origin.position.x, g.info.origin.position.y = self.ox, self.oy
        g.info.origin.orientation.w = 1.0
        g.data = data[::-1].astype(np.int8).ravel().tolist()   # grid row 0 = bottom
        return g

    def publish_masks(self):
        self.keep_pub.publish(self.grid(np.where(self.zone == -1, 100, 0)))
        # Humble SpeedFilter: 0 = no limit; -1 (unknown) is logged as an error every cycle
        self.speed_pub.publish(self.grid(np.where(self.zone > 0, self.zone, 0)))
        self.publish_zones()

    def report(self, text):
        self.msg = text
        self.publish_zones()

    def publish_zones(self):
        step = max(1, int(round(CELL_M / self.res)))
        k = self.zone[::step, ::step]
        cell = self.res * step
        rects = []
        for r in np.nonzero(k.any(axis=1))[0]:
            row = k[r]
            y1 = self.oy + (self.h - r * step) * self.res
            c = 0
            while c < row.shape[0] and len(rects) < MAX_RECTS:
                v = int(row[c])
                if v == 0:
                    c += 1
                    continue
                c0 = c
                while c < row.shape[0] and row[c] == v:
                    c += 1
                rects.append([round(self.ox + c0 * cell, 2), round(y1 - cell, 2),
                              round(self.ox + c * cell, 2), round(y1, 2),
                              'keepout' if v == -1 else v])
        self.zones_pub.publish(String(data=json.dumps(
            {'map': self.map_name, 'zones': rects, 'msg': self.msg}, ensure_ascii=False)))


def main():
    rclpy.init()
    node = ZoneManager()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
