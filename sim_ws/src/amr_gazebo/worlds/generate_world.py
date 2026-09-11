#!/usr/bin/env python3
"""
Generates a 'complex' multi-room indoor SDF world for SLAM testing
(Ignition Gazebo Fortress). Layout: an ~20m x 14m building with a
central corridor, 5 rooms, doorways, and scattered furniture-like
obstacles (boxes, cylinders/pillars) so that 2D-lidar SLAM has
plenty of features to map.

Run:  python3 generate_world.py > complex_world.sdf
"""

WALL_HEIGHT = 2.5
WALL_THICK = 0.15

walls = []   # list of dict: x, y, length, yaw(0 or 1.5708), name
obstacles = []  # list of dict: type, x, y, z, size/radius/length, name

def add_wall(name, x, y, length, yaw_deg, height=WALL_HEIGHT, thick=WALL_THICK):
    walls.append(dict(name=name, x=x, y=y, length=length, yaw=yaw_deg, height=height, thick=thick))

def add_box(name, x, y, sx, sy, sz, z=None, yaw_deg=0):
    if z is None:
        z = sz / 2.0
    obstacles.append(dict(kind='box', name=name, x=x, y=y, z=z, sx=sx, sy=sy, sz=sz, yaw=yaw_deg))

def add_cylinder(name, x, y, r, h, z=None):
    if z is None:
        z = h / 2.0
    obstacles.append(dict(kind='cylinder', name=name, x=x, y=y, z=z, r=r, h=h))

# ----------------------------------------------------------------------
# Building envelope: 20m (X) x 14m (Y), centred at origin.
# Central east-west corridor 2m wide at y in [-1, 1].
# Rooms: NW, NE, N-middle (offices) above corridor; SW, SE below.
# ----------------------------------------------------------------------
HALF_X = 10.0
HALF_Y = 7.0

# ----------------------------------------------------------------------
# Clearance budget, sized to the real robot:
#   footprint: 0.951m (L) x 0.496m (W), centred on base_link
#   local/global costmap inflation_radius: 0.35m (nav2_params.yaml)
#   MPPI CostCritic.collision_margin_distance: 0.05m hard floor
# Minimum clear opening so a straight corridor/doorway isn't squeezed to
# nothing once inflation eats in from both sides:
#   0.496 (width) + 2*0.35 (inflation) + 2*0.05 (margin) = 1.296m
# Every doorway/aisle below is sized to at least 1.4m clear for headroom.
# ----------------------------------------------------------------------
CORRIDOR_HALF_WIDTH = 1.15   # was 1.0 -> clear width ~2.15m (unchanged concern, just more buffer)
DOOR_HALF = 0.7               # was 0.5 -> doorway gap 1.4m clear (was 1.0m)

# --- Outer perimeter walls ---
add_wall('wall_north',  0.0,  HALF_Y, HALF_X * 2, 0)
add_wall('wall_south',  0.0, -HALF_Y, HALF_X * 2, 0)
add_wall('wall_east',   HALF_X, 0.0, HALF_Y * 2, 90)
add_wall('wall_west',  -HALF_X, 0.0, HALF_Y * 2, 90)

# --- Central corridor walls (y = +/-CORRIDOR_HALF_WIDTH), broken by doorways ---
# Doorway centres kept where they were; only the gap half-width changed
# (0.5 -> 0.7) so each doorway is now 1.4m clear instead of 1.0m.
door_centers = [-5.7, -0.7, 4.3]
def segments_with_doors(centers, half_gap, x_min, x_max):
    bounds = sorted([x_min] + [c - half_gap for c in centers] + [c + half_gap for c in centers] + [x_max])
    return [(bounds[i], bounds[i + 1]) for i in range(0, len(bounds), 2)]

corridor_segments_top = segments_with_doors(door_centers, DOOR_HALF, -HALF_X, HALF_X)
for i, (xs, xe) in enumerate(corridor_segments_top):
    length = xe - xs
    cx = xs + length / 2.0
    add_wall(f'corridor_wall_n_{i}', cx, CORRIDOR_HALF_WIDTH, length, 0)

corridor_segments_bot = segments_with_doors(door_centers, DOOR_HALF, -HALF_X, HALF_X)
for i, (xs, xe) in enumerate(corridor_segments_bot):
    length = xe - xs
    cx = xs + length / 2.0
    add_wall(f'corridor_wall_s_{i}', cx, -CORRIDOR_HALF_WIDTH, length, 0)

# --- Room divider walls (perpendicular, splitting north side into 3 rooms, south into 2) ---
# Spans recomputed from CORRIDOR_HALF_WIDTH so they still meet the corridor
# wall exactly (no gap, no overlap).
n_div_len = HALF_Y - CORRIDOR_HALF_WIDTH
n_div_cy = CORRIDOR_HALF_WIDTH + n_div_len / 2.0
add_wall('divider_n1', -6.7, n_div_cy, n_div_len, 90)
add_wall('divider_n2',  0.3, n_div_cy, n_div_len, 90)

s_div_len = HALF_Y - CORRIDOR_HALF_WIDTH
s_div_cy = -(CORRIDOR_HALF_WIDTH + s_div_len / 2.0)
add_wall('divider_s1', -1.5, s_div_cy, s_div_len, 90)

# A small closet / storage room in the SE corner
add_wall('closet_wall_1', 6.0, -4.5, 3.0, 0)
add_wall('closet_wall_2', 7.4, -3.2, 2.6, 90)

# ----------------------------------------------------------------------
# Furniture / obstacles for rich SLAM features
# ----------------------------------------------------------------------
# NW room (office): desk (box) + chair pillar + bookshelf
add_box('nw_desk', -8.5, 4.5, 1.4, 0.7, 0.75)
add_box('nw_bookshelf', -8.7, 6.3, 1.2, 0.35, 1.8)
add_cylinder('nw_chair', -7.6, 4.3, 0.22, 0.85)

# N-middle room: meeting table (big box) + 4 chair pillars
add_box('mid_table', -3.2, 4.2, 1.8, 0.9, 0.75)
add_cylinder('mid_chair_1', -4.0, 3.6, 0.2, 0.85)
add_cylinder('mid_chair_2', -2.4, 3.6, 0.2, 0.85)
add_cylinder('mid_chair_3', -4.0, 4.8, 0.2, 0.85)
add_cylinder('mid_chair_4', -2.4, 4.8, 0.2, 0.85)

# NE room: server rack (tall box) + two pillars
add_box('ne_rack', 5.5, 4.8, 0.8, 0.8, 2.0)
add_cylinder('ne_pillar_1', 3.0, 6.0, 0.25, 2.5)
add_cylinder('ne_pillar_2', 8.5, 6.0, 0.25, 2.5)

# SW room: pallet racking - only 3 units now, spaced 2.0m apart (was 4 @
# 1.6m) and slimmed from 0.6m to 0.5m deep, so the aisle between adjacent
# shelf faces is 1.5m clear (was 1.0m) instead of collapsing to ~0.3m
# after costmap inflation.
for i in range(3):
    add_box(f'sw_shelf_{i}', -8.0 + i * 2.0, -3.0, 0.5, 1.6, 1.6)
add_box('sw_pallet', -6.0, -5.8, 1.2, 1.0, 0.15, z=0.075)

# SE room (larger open area): scattered crates + a round table
add_box('se_crate_1', 3.5, -5.5, 0.6, 0.6, 0.6)
add_box('se_crate_2', 4.6, -5.2, 0.6, 0.6, 0.6)
add_cylinder('se_round_table', 6.5, -6.2, 0.5, 0.75)
add_box('se_long_shelf', 8.7, -5.0, 0.4, 3.0, 1.8)

# Corridor clutter: pillars are now pushed flush against the corridor
# walls (offset to alternating sides) instead of sitting mid-channel, so
# they no longer split the corridor into two too-narrow side channels -
# the full opposite width stays clear at every point along the corridor.
pillar_1_y = CORRIDOR_HALF_WIDTH - 0.35   # hugs north corridor wall
pillar_2_y = -(CORRIDOR_HALF_WIDTH - 0.35)  # hugs south corridor wall
add_cylinder('corridor_pillar_1', -3.5, pillar_1_y, 0.2, 2.5)
add_cylinder('corridor_pillar_2', 2.5, pillar_2_y, 0.2, 2.5)
add_cylinder('corridor_bin', -8.5, -(CORRIDOR_HALF_WIDTH - 0.35), 0.18, 0.6)

# Closet obstacle
add_box('closet_box', 6.7, -3.8, 0.5, 0.5, 1.0)


# ----------------------------------------------------------------------
# SDF generation
# ----------------------------------------------------------------------
def wall_sdf(w):
    yaw_rad = w['yaw'] * 3.14159265358979 / 180.0
    return f"""
    <model name="{w['name']}">
      <static>true</static>
      <pose>{w['x']} {w['y']} {w['height']/2:.3f} 0 0 {yaw_rad:.6f}</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box><size>{w['length']:.3f} {w['thick']:.3f} {w['height']:.3f}</size></box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box><size>{w['length']:.3f} {w['thick']:.3f} {w['height']:.3f}</size></box>
          </geometry>
          <material>
            <ambient>0.82 0.80 0.75 1</ambient>
            <diffuse>0.82 0.80 0.75 1</diffuse>
          </material>
        </visual>
      </link>
    </model>"""

def obstacle_sdf(o):
    if o['kind'] == 'box':
        yaw_rad = o.get('yaw', 0) * 3.14159265358979 / 180.0
        geom = f"<box><size>{o['sx']:.3f} {o['sy']:.3f} {o['sz']:.3f}</size></box>"
        pose = f"{o['x']} {o['y']} {o['z']:.3f} 0 0 {yaw_rad:.6f}"
    else:
        geom = f"<cylinder><radius>{o['r']:.3f}</radius><length>{o['h']:.3f}</length></cylinder>"
        pose = f"{o['x']} {o['y']} {o['z']:.3f} 0 0 0"
    return f"""
    <model name="{o['name']}">
      <static>true</static>
      <pose>{pose}</pose>
      <link name="link">
        <collision name="collision">
          <geometry>{geom}</geometry>
        </collision>
        <visual name="visual">
          <geometry>{geom}</geometry>
          <material>
            <ambient>0.55 0.35 0.2 1</ambient>
            <diffuse>0.55 0.35 0.2 1</diffuse>
          </material>
        </visual>
      </link>
    </model>"""

header = """<?xml version="1.0" ?>
<sdf version="1.9">
  <world name="amr_complex_world">

    <physics name="1ms" type="ignored">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>

    <plugin filename="ignition-gazebo-physics-system" name="ignition::gazebo::systems::Physics"/>
    <plugin filename="ignition-gazebo-user-commands-system" name="ignition::gazebo::systems::UserCommands"/>
    <plugin filename="ignition-gazebo-scene-broadcaster-system" name="ignition::gazebo::systems::SceneBroadcaster"/>
    <plugin filename="ignition-gazebo-sensors-system" name="ignition::gazebo::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
    <plugin filename="ignition-gazebo-contact-system" name="ignition::gazebo::systems::Contact"/>
    <plugin filename="ignition-gazebo-imu-system" name="ignition::gazebo::systems::Imu"/>

    <scene>
      <ambient>0.6 0.6 0.6 1</ambient>
      <background>0.6 0.7 0.8 1</background>
      <shadows>true</shadows>
      <grid>false</grid>
    </scene>

    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.9 0.9 0.9 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <attenuation>
        <range>1000</range>
        <constant>0.9</constant>
        <linear>0.01</linear>
        <quadratic>0.001</quadratic>
      </attenuation>
      <direction>-0.4 0.3 -0.9</direction>
    </light>

    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry><plane><normal>0 0 1</normal><size>60 60</size></plane></geometry>
          <surface>
            <friction><ode><mu>1.0</mu><mu2>1.0</mu2></ode></friction>
          </surface>
        </collision>
        <visual name="visual">
          <geometry><plane><normal>0 0 1</normal><size>60 60</size></plane></geometry>
          <material>
            <ambient>0.35 0.35 0.35 1</ambient>
            <diffuse>0.35 0.35 0.35 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
"""

footer = """
  </world>
</sdf>
"""

def main():
    parts = [header]
    for w in walls:
        parts.append(wall_sdf(w))
    for o in obstacles:
        parts.append(obstacle_sdf(o))
    parts.append(footer)
    print(''.join(parts))

if __name__ == '__main__':
    main()
