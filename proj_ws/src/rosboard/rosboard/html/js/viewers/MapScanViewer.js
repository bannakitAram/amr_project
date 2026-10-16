"use strict";

// Combined map + live LiDAR scan + robot pose, like Foxglove's Map panel.
// The card subscribes to nav_msgs/msg/OccupancyGrid (/map) as usual, and
// internally taps /scan, /tf and /tf_static (via subscribeExtra/
// unsubscribeExtra in index.js) to place the scan and the robot correctly
// in the map frame instead of just showing scan in the sensor's own frame.
//
// TF handling here is a minimal from-scratch 2D (x, y, yaw-only) transform
// tree - not a full tf2 port - which is fine for a ground AMR where EKF
// runs in two_d_mode. It walks whatever /tf and /tf_static edges have been
// seen so far to compose a path between two frames; if no path exists yet
// (e.g. right after connecting, before the first /tf message) it just
// skips drawing that layer until one arrives.
class MapScanViewer extends Space2DViewer {
  onCreate() {
    super.onCreate();

    this.mapImage = null;
    this.mapInfo = null;
    this.scanPoints = null;
    this.scanFrameId = null;
    this.tf = {}; // "parentFrame>childFrame" -> {x, y, theta}
    this._haveSetDefaultView = false;

    this._onTF = (msg) => this._handleTF(msg);
    this._onScan = (msg) => this._handleScan(msg);
    subscribeExtra('/tf', 'tf2_msgs/msg/TFMessage', this._onTF);
    subscribeExtra('/tf_static', 'tf2_msgs/msg/TFMessage', this._onTF);
    subscribeExtra('/scan', 'sensor_msgs/msg/LaserScan', this._onScan);
  }

  destroy() {
    unsubscribeExtra('/tf', this._onTF);
    unsubscribeExtra('/tf_static', this._onTF);
    unsubscribeExtra('/scan', this._onScan);
    super.destroy();
  }

  _quatYaw(q) {
    return Math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z));
  }

  _compose(a, b) {
    const cos = Math.cos(a.theta), sin = Math.sin(a.theta);
    return {
      x: a.x + cos * b.x - sin * b.y,
      y: a.y + sin * b.x + cos * b.y,
      theta: a.theta + b.theta,
    };
  }

  _inverse(t) {
    const cos = Math.cos(t.theta), sin = Math.sin(t.theta);
    return {
      x: -(cos * t.x + sin * t.y),
      y: -(-sin * t.x + cos * t.y),
      theta: -t.theta,
    };
  }

  // Returns the pose of sourceFrame expressed in targetFrame, or null if no
  // chain of known transforms connects them yet.
  _lookup(targetFrame, sourceFrame) {
    if (targetFrame === sourceFrame) return {x: 0, y: 0, theta: 0};

    const adjacency = {};
    for (const key in this.tf) {
      const sep = key.indexOf('>');
      const parent = key.slice(0, sep), child = key.slice(sep + 1);
      const t = this.tf[key];
      (adjacency[parent] = adjacency[parent] || []).push({to: child, t: t});
      (adjacency[child] = adjacency[child] || []).push({to: parent, t: this._inverse(t)});
    }

    const visited = {[targetFrame]: {x: 0, y: 0, theta: 0}};
    const queue = [targetFrame];
    while (queue.length) {
      const cur = queue.shift();
      if (cur === sourceFrame) return visited[cur];
      for (const edge of (adjacency[cur] || [])) {
        if (edge.to in visited) continue;
        visited[edge.to] = this._compose(visited[cur], edge.t);
        queue.push(edge.to);
      }
    }
    return visited[sourceFrame] || null;
  }

  _handleTF(msg) {
    for (const t of msg.transforms) {
      this.tf[t.header.frame_id + '>' + t.child_frame_id] = {
        x: t.transform.translation.x,
        y: t.transform.translation.y,
        theta: this._quatYaw(t.transform.rotation),
      };
    }
    this._redraw();
  }

  _base64decode(base64) {
    const binary = window.atob(base64);
    const bytes = new Uint8Array(binary.length);
    for (let i = 0; i < binary.length; i++) bytes[i] = binary.charCodeAt(i);
    return bytes.buffer;
  }

  _decodeScan(msg) {
    if (msg.__comp) {
      const bounds = msg._ranges_uint16.bounds;
      const view = new DataView(this._base64decode(msg._ranges_uint16.points));
      const n = view.byteLength / 2;
      const points = new Float32Array(n * 2);
      const angleIncr = (msg.angle_max - msg.angle_min) / n;
      const range = bounds[1] - bounds[0], rmin = bounds[0];
      for (let i = 0; i < n; i++) {
        const r16 = view.getUint16(i * 2, true);
        if (r16 === 65535) { points[2*i] = NaN; points[2*i+1] = NaN; continue; }
        const r = r16 / 65534 * range + rmin;
        const angle = msg.angle_min + i * angleIncr;
        points[2*i] = r * Math.cos(angle);
        points[2*i+1] = r * Math.sin(angle);
      }
      return points;
    }
    const n = msg.ranges.length;
    const points = new Float32Array(n * 2);
    const angleIncr = (msg.angle_max - msg.angle_min) / n;
    for (let i = 0; i < n; i++) {
      const angle = msg.angle_min + i * angleIncr;
      if (-10000.0 < msg.ranges[i] && msg.ranges[i] < 10000.0) {
        points[2*i] = msg.ranges[i] * Math.cos(angle);
        points[2*i+1] = msg.ranges[i] * Math.sin(angle);
      } else {
        points[2*i] = NaN; points[2*i+1] = NaN;
      }
    }
    return points;
  }

  _handleScan(msg) {
    this.scanFrameId = msg.header.frame_id;
    this.scanPoints = this._decodeScan(msg);
    console.log("DEBUG scan received, frame=" + this.scanFrameId + " n=" + this.scanPoints.length + " tfKeys=" + JSON.stringify(Object.keys(this.tf)));
    this._redraw();
  }

  onData(msg) {
    this.card.title.text(msg._topic_name);

    this.mapInfo = {
      resolution: msg.info.resolution,
      widthCells: msg.info.width,
      heightCells: msg.info.height,
      originX: msg.info.origin.position.x,
      originY: msg.info.origin.position.y,
      originTheta: this._quatYaw(msg.info.origin.orientation),
      frameId: (msg.header && msg.header.frame_id) || 'map',
    };

    if (!this._haveSetDefaultView) {
      this.setDefaultView({
        xcenter: this.mapInfo.originX + this.mapInfo.widthCells * this.mapInfo.resolution / 2,
        ycenter: this.mapInfo.originY + this.mapInfo.heightCells * this.mapInfo.resolution / 2,
        scale: Math.max(this.mapInfo.widthCells, this.mapInfo.heightCells) * this.mapInfo.resolution * 1.1,
      });
      this._haveSetDefaultView = true;
    }

    if (msg._data_jpeg) {
      const img = new Image();
      img.onload = () => { this.mapImage = img; this._redraw(); };
      img.src = "data:image/jpeg;base64," + msg._data_jpeg;
    }

    this._redraw();
  }

  _redraw() {
    if (!this.mapInfo) return;
    const drawObjects = [];
    const mapFrame = this.mapInfo.frameId;

    if (this.mapImage) {
      drawObjects.push({
        type: 'image',
        img: this.mapImage,
        x: this.mapInfo.originX,
        y: this.mapInfo.originY,
        theta: this.mapInfo.originTheta,
        width: this.mapInfo.widthCells * this.mapInfo.resolution,
        height: this.mapInfo.heightCells * this.mapInfo.resolution,
      });
    }

    if (this.scanPoints && this.scanFrameId) {
      const scanPose = this._lookup(mapFrame, this.scanFrameId);
      console.log("DEBUG redraw mapFrame=" + mapFrame + " scanFrameId=" + this.scanFrameId + " scanPose=" + JSON.stringify(scanPose));
      if (scanPose) {
        const cos = Math.cos(scanPose.theta), sin = Math.sin(scanPose.theta);
        const n = this.scanPoints.length;
        const transformed = new Float32Array(n);
        for (let i = 0; i < n / 2; i++) {
          const lx = this.scanPoints[2*i], ly = this.scanPoints[2*i+1];
          transformed[2*i] = scanPose.x + cos * lx - sin * ly;
          transformed[2*i+1] = scanPose.y + sin * lx + cos * ly;
        }
        drawObjects.push({type: 'points', data: transformed, color: '#ffeb3b'});
      }
    }

    const robotPose = this._lookup(mapFrame, 'base_footprint');
    if (robotPose) {
      drawObjects.push({type: 'robot', x: robotPose.x, y: robotPose.y, theta: robotPose.theta});
    }

    this.draw(drawObjects);
  }
}

MapScanViewer.friendlyName = "Map + Scan";

MapScanViewer.supportedTypes = [
    "nav_msgs/msg/OccupancyGrid",
];

MapScanViewer.maxUpdateRate = 6.0;

Viewer.registerViewer(MapScanViewer);
