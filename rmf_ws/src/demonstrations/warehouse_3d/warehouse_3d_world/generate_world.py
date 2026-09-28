#!/usr/bin/env python3
"""
Warehouse 3D World Generator
Generates a complete, presentation-ready SDF simulation world for Gazebo Sim (ROS 2 Jazzy).

Visual Architecture Layers:
1. Warehouse Architecture:
   - High-ceiling industrial enclosure (32m x 26m x 5.2m)
   - Dual-tone walls (dark concrete wainscot + light corrugated cladding + dark coping)
   - Translucent clerestory daylight windows
   - Heavy structural steel H-beam columns along perimeters with OSHA safety-yellow base footings
   - Overhead open-web steel roof trusses and transverse I-beams
   - Commercial loading dock roll-up shutter doors with weather hoods, bumpers, and hazard headers
   - Emergency personnel exit doors with illuminated signs
2. Industrial Floor & Markings:
   - Polished industrial concrete slab with expansion joint grid
   - Arterial 2-way AMR highway with solid yellow borders, dashed white centerline, and directional arrows
   - Designated Aisle entry guidance markings (Aisles 01 - 05)
   - High-contrast AMR Charging Bay parking stalls (Bays 01 - 05) with bolt symbols and stall numbers
   - Diagonal yellow/black hazard hatching around Inbound Dock & Packing aprons
   - Dedicated industrial blue pedestrian safety walkway with white zebra crossing stripes
3. Heavy-Duty Storage Racks (ASRS):
   - 12 Racking rows (6 West, 6 East) in industrial safety blue uprights and OSHA safety orange beams
   - Heavy-duty diagonal X-bracing on outer end frames
   - Galvanized wire mesh / steel pallet decks
   - Curved steel safety bollards protecting aisle corners
4. Detailed Pallets & Cardboard Packaging:
   - Authentic GMA/Euro wooden pallets with realistic deck slats, runners, and stringers
   - 96+ Cardboard Package boxes in 3 varied kraft brown shades
   - Sealing tape strips, white shipping barcode labels, and red handling warning marks
5. Dedicated Workstations & Equipment:
   - Packing Area (Zone C): Heavy-duty steel packing workbenches, equipment gantries, and gravity roller conveyor
   - Inbound Receiving Dock (Zone A): Hydraulic dock leveler plate, rubber dock bumpers, staging pallet stacks
   - AMR Fleet Depot (Zone D): 5 Industrial charging terminals with LED status displays and emergency cut-off
6. Signage & High-Bay Lighting:
   - Hanging Zone Signs: Zone A (Inbound Dock), Zone B (ASRS Storage), Zone C (Packing/Shipping), Zone D (AMR Depot)
   - Suspended Aisle Number plaques (Aisles 01 to 05)
   - Wall-mounted fire extinguisher stations
   - 9 High-bay industrial reflector luminaires with warm-neutral balanced lighting
7. Direct AMR Fleet Embedding:
   - Instant frame-1 spawning of Alpha, Beta, Gamma, Delta, Epsilon
"""

import os
import sys
import math
import subprocess
import re

def box_link(name, x, y, z, sx, sy, sz, r, g, b, a=1.0, collide=True, roll=0.0, pitch=0.0, yaw=0.0, emissive=(0,0,0)):
    """Helper to generate an SDF link with a box geometry."""
    col_str = ""
    if collide:
        col_str = f"""
        <collision name="{name}_col">
          <geometry><box><size>{sx:.4f} {sy:.4f} {sz:.4f}</size></box></geometry>
        </collision>"""
    emis_str = ""
    if emissive[0] > 0 or emissive[1] > 0 or emissive[2] > 0:
        emis_str = f"\n            <emissive>{emissive[0]:.3f} {emissive[1]:.3f} {emissive[2]:.3f} 1.0</emissive>"

    return f"""      <link name="{name}">
        <pose>{x:.4f} {y:.4f} {z:.4f} {roll:.4f} {pitch:.4f} {yaw:.4f}</pose>{col_str}
        <visual name="{name}_vis">
          <geometry><box><size>{sx:.4f} {sy:.4f} {sz:.4f}</size></box></geometry>
          <material>
            <ambient>{r*0.65:.3f} {g*0.65:.3f} {b*0.65:.3f} {a:.2f}</ambient>
            <diffuse>{r:.3f} {g:.3f} {b:.3f} {a:.2f}</diffuse>
            <specular>0.15 0.15 0.15 1.0</specular>{emis_str}
          </material>
        </visual>
      </link>
"""

def cylinder_link(name, x, y, z, radius, length, r, g, b, a=1.0, collide=True, roll=0.0, pitch=0.0, yaw=0.0, emissive=(0,0,0)):
    """Helper to generate an SDF link with a cylinder geometry."""
    col_str = ""
    if collide:
        col_str = f"""
        <collision name="{name}_col">
          <geometry><cylinder><radius>{radius:.4f}</radius><length>{length:.4f}</length></cylinder></geometry>
        </collision>"""
    emis_str = ""
    if emissive[0] > 0 or emissive[1] > 0 or emissive[2] > 0:
        emis_str = f"\n            <emissive>{emissive[0]:.3f} {emissive[1]:.3f} {emissive[2]:.3f} 1.0</emissive>"

    return f"""      <link name="{name}">
        <pose>{x:.4f} {y:.4f} {z:.4f} {roll:.4f} {pitch:.4f} {yaw:.4f}</pose>{col_str}
        <visual name="{name}_vis">
          <geometry><cylinder><radius>{radius:.4f}</radius><length>{length:.4f}</length></cylinder></geometry>
          <material>
            <ambient>{r*0.65:.3f} {g*0.65:.3f} {b*0.65:.3f} {a:.2f}</ambient>
            <diffuse>{r:.3f} {g:.3f} {b:.3f} {a:.2f}</diffuse>
            <specular>0.2 0.2 0.2 1.0</specular>{emis_str}
          </material>
        </visual>
      </link>
"""

def generate_warehouse_sdf(output_path):
    sdf = []
    sdf.append("""<?xml version="1.0" ?>
<sdf version="1.8">
  <world name="warehouse_3d">
    <!-- Physics Configuration (Optimized 0.008s step size for smooth RTF) -->
    <physics name="8ms" type="ignored">
      <max_step_size>0.008</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>

    <!-- Essential Gazebo Plugins -->
    <plugin
      filename="gz-sim-physics-system"
      name="gz::sim::systems::Physics">
    </plugin>
    <plugin
      filename="gz-sim-user-commands-system"
      name="gz::sim::systems::UserCommands">
    </plugin>
    <plugin
      filename="gz-sim-scene-broadcaster-system"
      name="gz::sim::systems::SceneBroadcaster">
    </plugin>
    <plugin
      filename="gz-sim-sensors-system"
      name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
    <plugin
      filename="gz-sim-imu-system"
      name="gz::sim::systems::Imu">
    </plugin>

    <!-- Gazebo Sim GUI Configuration with Cinematic Overview Camera -->
    <gui fullscreen="0">
      <plugin filename="MinimalScene" name="3D View">
        <gz-gui>
          <title>3D View</title>
          <property type="bool" key="showTitleBar">false</property>
          <property type="string" key="state">docked</property>
        </gz-gui>
        <engine>ogre2</engine>
        <scene>scene</scene>
        <ambient_light>0.65 0.68 0.72</ambient_light>
        <background_color>0.14 0.16 0.19</background_color>
        <!-- Elevated Southwest Isometric Overview directly framing all 5 AMR robots in docks and racks -->
        <camera_pose>-15.5 -10.5 8.2 0 0.52 0.62</camera_pose>
      </plugin>
      <plugin filename="EntityContextMenuPlugin" name="Entity context menu">
        <gz-gui>
          <property key="state" type="string">floating</property>
          <property key="width" type="double">5</property>
          <property key="height" type="double">5</property>
          <property key="showTitleBar" type="bool">false</property>
        </gz-gui>
      </plugin>
      <plugin filename="GzSceneManager" name="Scene Manager">
        <gz-gui>
          <property key="resizable" type="bool">false</property>
          <property key="width" type="double">5</property>
          <property key="height" type="double">5</property>
          <property key="state" type="string">floating</property>
          <property key="showTitleBar" type="bool">false</property>
        </gz-gui>
      </plugin>
      <plugin filename="InteractiveViewControl" name="Interactive view control">
        <gz-gui>
          <property key="resizable" type="bool">false</property>
          <property key="width" type="double">5</property>
          <property key="height" type="double">5</property>
          <property key="state" type="string">floating</property>
          <property key="showTitleBar" type="bool">false</property>
        </gz-gui>
      </plugin>
      <plugin filename="CameraTracking" name="Camera Tracking">
        <gz-gui>
          <property key="resizable" type="bool">false</property>
          <property key="width" type="double">5</property>
          <property key="height" type="double">5</property>
          <property key="state" type="string">floating</property>
          <property key="showTitleBar" type="bool">false</property>
        </gz-gui>
      </plugin>
      <plugin filename="WorldControl" name="World control">
        <gz-gui>
          <title>World control</title>
          <property type="bool" key="showTitleBar">false</property>
          <property type="bool" key="resizable">false</property>
          <property type="double" key="height">72</property>
          <property type="double" key="z">1</property>
          <property type="string" key="state">floating</property>
          <anchors target="3D View">
            <line own="left" target="left"/>
            <line own="bottom" target="bottom"/>
          </anchors>
        </gz-gui>
        <play_pause>true</play_pause>
        <step>true</step>
        <start_paused>false</start_paused>
      </plugin>
      <plugin filename="WorldStats" name="World stats">
        <gz-gui>
          <title>World stats</title>
          <property type="bool" key="showTitleBar">false</property>
          <property type="bool" key="resizable">false</property>
          <property type="double" key="height">110</property>
          <property type="double" key="width">290</property>
          <property type="double" key="z">1</property>
          <property type="string" key="state">floating</property>
          <anchors target="3D View">
            <line own="right" target="right"/>
            <line own="bottom" target="bottom"/>
          </anchors>
        </gz-gui>
        <sim_time>true</sim_time>
        <real_time>true</real_time>
        <real_time_factor>true</real_time_factor>
        <iterations>true</iterations>
      </plugin>
    </gui>

    <!-- Ambient Lighting & Atmospheric Scene -->
    <scene>
      <ambient>0.65 0.68 0.72 1.0</ambient>
      <background>0.14 0.16 0.19 1.0</background>
      <shadows>true</shadows>
      <grid>false</grid>
    </scene>

    <!-- Main Directional Overhead Sky/Sun Light -->
    <light type="directional" name="sun_main">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 16 0 0 0</pose>
      <diffuse>0.85 0.88 0.92 1.0</diffuse>
      <specular>0.3 0.3 0.35 1.0</specular>
      <attenuation>
        <range>1000</range>
        <constant>0.9</constant>
        <linear>0.01</linear>
        <quadratic>0.001</quadratic>
      </attenuation>
      <direction>-0.3 0.2 -0.9</direction>
    </light>
""")

    # 9 High-Bay Warehouse Lights (Balanced 4500K Industrial Luminaires)
    light_positions = [
        (-10.0, -6.0, 5.0), (-10.0, 0.0, 5.0), (-10.0, 6.0, 5.0),
        (-2.0, -6.0, 5.0),  (-2.0, 0.0, 5.0),  (-2.0, 6.0, 5.0),
        (6.0, -6.0, 5.0),   (6.0, 0.0, 5.0),   (6.0, 6.0, 5.0)
    ]
    for i, (lx, ly, lz) in enumerate(light_positions):
        sdf.append(f"""
    <light type="point" name="bay_light_{i}">
      <pose>{lx:.1f} {ly:.1f} {lz:.1f} 0 0 0</pose>
      <diffuse>0.72 0.75 0.80 1.0</diffuse>
      <specular>0.25 0.25 0.30 1.0</specular>
      <attenuation>
        <range>22</range>
        <constant>0.20</constant>
        <linear>0.04</linear>
        <quadratic>0.008</quadratic>
      </attenuation>
      <cast_shadows>false</cast_shadows>
    </light>
""")

    # =========================================================================
    # 1. GROUND FLOOR
    # =========================================================================
    sdf.append("""
    <!-- Warehouse Ground Floor -->
    <model name="warehouse_floor">
      <static>true</static>
      <link name="floor_link">
        <collision name="floor_collision">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>36 28</size>
            </plane>
          </geometry>
          <surface>
            <friction>
              <ode>
                <mu>1.0</mu>
                <mu2>1.0</mu2>
              </ode>
            </friction>
          </surface>
        </collision>
        <visual name="floor_visual">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>36 28</size>
            </plane>
          </geometry>
          <material>
            <ambient>0.32 0.34 0.36 1.0</ambient>
            <diffuse>0.40 0.42 0.45 1.0</diffuse>
            <specular>0.15 0.15 0.15 1.0</specular>
          </material>
        </visual>
      </link>
    </model>
""")

    # =========================================================================
    # 2. WAREHOUSE STRUCTURAL WALLS & DOORS
    # =========================================================================
    # Warehouse perimeter: X in [-16, 16], Y in [-13, 13], Total Height = 5.2m
    # Dual-tone wall: Base skirting (0.8m high, dark concrete) + Upper wall (4.4m high, light corrugated)
    sdf.append("""
    <!-- Warehouse Structural Architecture & Perimeter Walls -->
    <model name="warehouse_architecture">
      <static>true</static>
""")

    # North Wall (Y = 13.0, length 32.4m)
    # Lower skirting
    sdf.append(box_link("wall_north_base", 0.0, 13.0, 0.4, 32.4, 0.4, 0.8, 0.25, 0.28, 0.32))
    # Upper panel
    sdf.append(box_link("wall_north_upper", 0.0, 13.0, 3.0, 32.4, 0.4, 4.4, 0.72, 0.74, 0.77))
    # Top wall coping trim
    sdf.append(box_link("wall_north_coping", 0.0, 13.0, 5.25, 32.6, 0.5, 0.15, 0.18, 0.20, 0.22))
    # Clerestory high industrial window band (translucent sky-blue)
    sdf.append(box_link("window_north", 0.0, 12.85, 4.2, 28.0, 0.05, 1.0, 0.45, 0.65, 0.85, a=0.7, collide=False))

    # South Wall (Y = -13.0, length 32.4m)
    sdf.append(box_link("wall_south_base", 0.0, -13.0, 0.4, 32.4, 0.4, 0.8, 0.25, 0.28, 0.32))
    sdf.append(box_link("wall_south_upper", 0.0, -13.0, 3.0, 32.4, 0.4, 4.4, 0.72, 0.74, 0.77))
    sdf.append(box_link("wall_south_coping", 0.0, -13.0, 5.25, 32.6, 0.5, 0.15, 0.18, 0.20, 0.22))

    # East Wall (X = 16.0, length 25.6m)
    sdf.append(box_link("wall_east_base", 16.0, 0.0, 0.4, 0.4, 25.6, 0.8, 0.25, 0.28, 0.32))
    sdf.append(box_link("wall_east_upper", 16.0, 0.0, 3.0, 0.4, 25.6, 4.4, 0.72, 0.74, 0.77))
    sdf.append(box_link("wall_east_coping", 16.0, 0.0, 5.25, 0.5, 25.8, 0.15, 0.18, 0.20, 0.22))
    sdf.append(box_link("window_east", 15.85, 0.0, 4.2, 0.05, 20.0, 1.0, 0.45, 0.65, 0.85, a=0.7, collide=False))

    # West Wall (X = -16.0, partitioned with 2 Commercial Roll-up Shutter Doors at Y=8.5 and Y=-8.5)
    # Segments:
    # 1. North section: Y = 11.25 to 12.8 (length 3.1m, center Y = 11.45)
    sdf.append(box_link("wall_west_n_base", -16.0, 11.45, 0.4, 0.4, 3.1, 0.8, 0.25, 0.28, 0.32))
    sdf.append(box_link("wall_west_n_upper", -16.0, 11.45, 3.0, 0.4, 3.1, 4.4, 0.72, 0.74, 0.77))

    # 2. Central section: Y = -6.0 to 6.0 (length 12.0m, center Y = 0.0)
    sdf.append(box_link("wall_west_mid_base", -16.0, 0.0, 0.4, 0.4, 12.0, 0.8, 0.25, 0.28, 0.32))
    sdf.append(box_link("wall_west_mid_upper", -16.0, 0.0, 3.0, 0.4, 12.0, 4.4, 0.72, 0.74, 0.77))

    # 3. South section: Y = -12.8 to -11.0 (length 3.6m, center Y = -11.9)
    sdf.append(box_link("wall_west_s_base", -16.0, -11.9, 0.4, 0.4, 3.6, 0.8, 0.25, 0.28, 0.32))
    sdf.append(box_link("wall_west_s_upper", -16.0, -11.9, 3.0, 0.4, 3.6, 4.4, 0.72, 0.74, 0.77))

    # Top header above Inbound Dock Shutter (Y = 8.5, Z = 4.0 to 5.2m)
    sdf.append(box_link("dock_shutter_n_hdr", -16.0, 8.5, 4.4, 0.4, 4.0, 1.6, 0.72, 0.74, 0.77))
    # Top header above Outbound Dispatch Shutter (Y = -8.5, Z = 4.0 to 5.2m)
    sdf.append(box_link("dock_shutter_s_hdr", -16.0, -8.5, 4.4, 0.4, 4.0, 1.6, 0.72, 0.74, 0.77))
    sdf.append(box_link("wall_west_coping", -16.0, 0.0, 5.25, 0.5, 25.8, 0.15, 0.18, 0.20, 0.22))

    # =========================================================================
    # ROLL-UP DOCK DOORS & DETAILS (Zone A and Zone C)
    # =========================================================================
    # Roll-up Door Inbound (Y = 8.5): Corrugated steel blue-grey shutter
    sdf.append(box_link("rollup_door_inbound", -16.05, 8.5, 1.8, 0.1, 3.8, 3.6, 0.28, 0.35, 0.42))
    # Hazard frame yellow/black chevron header
    sdf.append(box_link("rollup_hazard_hdr_in", -15.9, 8.5, 3.65, 0.15, 4.0, 0.2, 0.95, 0.80, 0.05))
    # Exterior loading canopy hood
    sdf.append(box_link("dock_canopy_in", -16.8, 8.5, 3.9, 1.6, 4.4, 0.15, 0.22, 0.25, 0.30))
    # Heavy rubber dock bumpers flanking door
    sdf.append(box_link("dock_bumper_in_1", -15.85, 6.4, 0.5, 0.2, 0.25, 0.9, 0.10, 0.10, 0.10))
    sdf.append(box_link("dock_bumper_in_2", -15.85, 10.6, 0.5, 0.2, 0.25, 0.9, 0.10, 0.10, 0.10))

    # Roll-up Door Outbound (Y = -8.5)
    sdf.append(box_link("rollup_door_outbound", -16.05, -8.5, 1.8, 0.1, 3.8, 3.6, 0.28, 0.35, 0.42))
    sdf.append(box_link("rollup_hazard_hdr_out", -15.9, -8.5, 3.65, 0.15, 4.0, 0.2, 0.95, 0.80, 0.05))
    sdf.append(box_link("dock_canopy_out", -16.8, -8.5, 3.9, 1.6, 4.4, 0.15, 0.22, 0.25, 0.30))
    sdf.append(box_link("dock_bumper_out_1", -15.85, -10.6, 0.5, 0.2, 0.25, 0.9, 0.10, 0.10, 0.10))
    sdf.append(box_link("dock_bumper_out_2", -15.85, -6.4, 0.5, 0.2, 0.25, 0.9, 0.10, 0.10, 0.10))

    # Emergency Personnel Exit Doors (North and South walls)
    # North Exit Door (X=0.0, Y=12.9)
    sdf.append(box_link("exit_door_north", 0.0, 12.85, 1.1, 1.1, 0.05, 2.2, 0.30, 0.32, 0.35))
    # Emergency Push Bar (Silver)
    sdf.append(box_link("exit_pushbar_north", 0.0, 12.80, 1.0, 0.85, 0.06, 0.06, 0.80, 0.80, 0.85))
    # Illuminated Green EXIT Sign
    sdf.append(box_link("exit_sign_north", 0.0, 12.80, 2.35, 0.55, 0.08, 0.25, 0.10, 0.85, 0.25, collide=False, emissive=(0.1, 0.8, 0.2)))

    # South Exit Door (X=0.0, Y=-12.9)
    sdf.append(box_link("exit_door_south", 0.0, -12.85, 1.1, 1.1, 0.05, 2.2, 0.30, 0.32, 0.35))
    sdf.append(box_link("exit_pushbar_south", 0.0, -12.80, 1.0, 0.85, 0.06, 0.06, 0.80, 0.80, 0.85))
    sdf.append(box_link("exit_sign_south", 0.0, -12.80, 2.35, 0.55, 0.08, 0.25, 0.10, 0.85, 0.25, collide=False, emissive=(0.1, 0.8, 0.2)))

    # =========================================================================
    # HEAVY INDUSTRIAL H-BEAM STRUCTURAL COLUMNS (Perimeter Safe Placement)
    # =========================================================================
    # Placed along perimeter walls (X = ±15.8 and Y = ±12.8)
    column_coords = [
        (-15.75, -12.0), (-15.75, -4.0), (-15.75, 4.0), (-15.75, 12.0),
        (15.75, -12.0),  (15.75, -6.0),  (15.75, 0.0),  (15.75, 6.0),  (15.75, 12.0),
        (-8.0, 12.75),   (0.0, 12.75),   (8.0, 12.75),
        (-8.0, -12.75),  (0.0, -12.75),  (8.0, -12.75)
    ]
    for ci, (cx, cy) in enumerate(column_coords):
        # Base footing in OSHA safety yellow (0 to 1.0m)
        sdf.append(box_link(f"col_{ci}_base", cx, cy, 0.5, 0.45, 0.45, 1.0, 0.95, 0.75, 0.05))
        # Upper column in industrial structural steel (1.0m to 5.2m)
        sdf.append(box_link(f"col_{ci}_stem", cx, cy, 3.1, 0.35, 0.35, 4.2, 0.28, 0.30, 0.34))

    # =========================================================================
    # ROOF TRUSSES & CEILING BEAMS (Z = 4.8m to 5.2m)
    # =========================================================================
    # Primary Transverse Steel I-Beams (spanning East-West from X = -16 to +16 at Y = -9, -3, 3, 9)
    for bi, by in enumerate([-9.0, -3.0, 3.0, 9.0]):
        # Main bottom chord
        sdf.append(box_link(f"truss_beam_{bi}", 0.0, by, 4.9, 32.0, 0.25, 0.35, 0.22, 0.24, 0.28, collide=False))
        # Upper roof joist
        sdf.append(box_link(f"truss_top_{bi}", 0.0, by, 5.2, 32.0, 0.20, 0.15, 0.22, 0.24, 0.28, collide=False))
        # Web diagonal truss braces (4 per beam)
        for wj, wx in enumerate([-12.0, -6.0, 0.0, 6.0, 12.0]):
            sdf.append(box_link(f"truss_web_{bi}_{wj}", wx, by, 5.05, 0.12, 0.12, 0.30, 0.28, 0.30, 0.34, collide=False))

    # Longitudinal tie-girders (North-South spanning along X = -9.0, 0.0, 9.0)
    for gi, gx in enumerate([-9.0, 0.0, 9.0]):
        sdf.append(box_link(f"girder_{gi}", gx, 0.0, 5.15, 0.20, 25.6, 0.20, 0.20, 0.22, 0.26, collide=False))

    # High-Bay Fixture Housings (Industrial Bell Reflector Hoods at light locations)
    for fi, (lx, ly, lz) in enumerate(light_positions):
        # Bell reflector housing (cylinder)
        sdf.append(cylinder_link(f"fixture_bell_{fi}", lx, ly, lz + 0.15, 0.35, 0.25, 0.35, 0.38, 0.42, collide=False))
        # Top mounting stem
        sdf.append(cylinder_link(f"fixture_stem_{fi}", lx, ly, lz + 0.35, 0.04, 0.20, 0.15, 0.15, 0.18, collide=False))
        # Glowing LED lens disc
        sdf.append(cylinder_link(f"fixture_lens_{fi}", lx, ly, lz + 0.02, 0.32, 0.04, 0.95, 0.98, 1.0, collide=False, emissive=(0.85, 0.88, 0.95)))

    sdf.append("    </model>\n")

    # =========================================================================
    # 3. INDUSTRIAL FLOOR & LANE MARKINGS (Visual Only, Zero Physics Overhead)
    # =========================================================================
    sdf.append("""
    <!-- High-Definition Industrial Floor Markings & Safety Lanes -->
    <model name="industrial_floor_markings">
      <static>true</static>
""")

    # Concrete Expansion Joint Grid Lines (scratched into concrete every 6m)
    # North-South joint lines
    for jx in [-12.0, -6.0, 0.0, 6.0, 12.0]:
        sdf.append(box_link(f"joint_ns_{jx}", jx, 0.0, 0.002, 0.03, 25.6, 0.002, 0.22, 0.24, 0.26, collide=False))
    # East-West joint lines
    for jy in [-9.0, -3.0, 3.0, 9.0]:
        sdf.append(box_link(f"joint_ew_{jy}", 0.0, jy, 0.002, 32.0, 0.03, 0.002, 0.22, 0.24, 0.26, collide=False))

    # A. ARTERIAL AMR HIGHWAY (Two-way travel corridor at X = -9.0, Y from -10.0 to 9.5)
    # West Boundary (Solid Safety Yellow)
    sdf.append(box_link("arterial_border_w", -10.1, 0.0, 0.003, 0.10, 20.0, 0.002, 0.95, 0.78, 0.05, collide=False))
    # East Boundary (Solid Safety Yellow)
    sdf.append(box_link("arterial_border_e", -7.9, 0.0, 0.003, 0.10, 20.0, 0.002, 0.95, 0.78, 0.05, collide=False))
    # Dashed White Centerline (Dashes: 1.0m mark, 1.0m gap)
    for di, dy in enumerate(range(-9, 10, 2)):
        sdf.append(box_link(f"arterial_dash_{di}", -9.0, float(dy), 0.003, 0.08, 1.0, 0.002, 0.92, 0.92, 0.95, collide=False))

    # Traffic Flow Direction Arrows on Arterial Highway (Chevron Arrow graphics)
    # Northbound lane (X = -8.5)
    for ay in [-6.0, 2.0]:
        sdf.append(box_link(f"arrow_nb_stem_{ay}", -8.5, ay, 0.003, 0.10, 0.8, 0.002, 0.92, 0.92, 0.95, collide=False))
        sdf.append(box_link(f"arrow_nb_head1_{ay}", -8.6, ay + 0.35, 0.003, 0.08, 0.3, 0.002, 0.92, 0.92, 0.95, collide=False, yaw=0.5))
        sdf.append(box_link(f"arrow_nb_head2_{ay}", -8.4, ay + 0.35, 0.003, 0.08, 0.3, 0.002, 0.92, 0.92, 0.95, collide=False, yaw=-0.5))
    # Southbound lane (X = -9.5)
    for ay in [-2.0, 6.0]:
        sdf.append(box_link(f"arrow_sb_stem_{ay}", -9.5, ay, 0.003, 0.10, 0.8, 0.002, 0.92, 0.92, 0.95, collide=False))
        sdf.append(box_link(f"arrow_sb_head1_{ay}", -9.6, ay - 0.35, 0.003, 0.08, 0.3, 0.002, 0.92, 0.92, 0.95, collide=False, yaw=-0.5))
        sdf.append(box_link(f"arrow_sb_head2_{ay}", -9.4, ay - 0.35, 0.003, 0.08, 0.3, 0.002, 0.92, 0.92, 0.95, collide=False, yaw=0.5))

    # B. AISLE ENTRY GUIDANCE MARKINGS (Aisles 01 to 05 at Y = -6.0, -3.0, 0.0, 3.0, 6.0)
    aisle_ys = [-6.0, -3.0, 0.0, 3.0, 6.0]
    for ai, ay in enumerate(aisle_ys):
        # Aisle threshold stop/entry bar
        sdf.append(box_link(f"aisle_entry_bar_{ai}", -7.7, ay, 0.003, 0.12, 1.8, 0.002, 0.95, 0.78, 0.05, collide=False))
        # Aisle center guide line into the racks (X from -7.5 to -2.5)
        sdf.append(box_link(f"aisle_center_w_{ai}", -5.0, ay, 0.003, 5.0, 0.06, 0.002, 0.85, 0.85, 0.90, collide=False))
        # East Aisle center guide line (X from 3.5 to 8.5)
        sdf.append(box_link(f"aisle_center_e_{ai}", 6.0, ay, 0.003, 5.0, 0.06, 0.002, 0.85, 0.85, 0.90, collide=False))

    # C. PEDESTRIAN SAFETY WALKWAY (OSHA Safety Blue path along West Wall: X = -14.6 to -13.4)
    # Blue floor path background
    sdf.append(box_link("walkway_blue_base", -14.0, 0.0, 0.002, 1.2, 22.0, 0.002, 0.10, 0.40, 0.78, collide=False))
    # Pedestrian Walkway Yellow Edge Borders
    sdf.append(box_link("walkway_edge_w", -14.6, 0.0, 0.003, 0.08, 22.0, 0.002, 0.95, 0.78, 0.05, collide=False))
    sdf.append(box_link("walkway_edge_e", -13.4, 0.0, 0.003, 0.08, 22.0, 0.002, 0.95, 0.78, 0.05, collide=False))
    # White Zebra Crossing Stripes at Cross-Traffic Intersections (Y = 4.0 and Y = -4.0)
    for ci, cy_cross in enumerate([4.0, -4.0]):
        for si in range(6):
            sy_off = cy_cross - 0.75 + si * 0.3
            sdf.append(box_link(f"zebra_{ci}_{si}", -14.0, sy_off, 0.004, 1.1, 0.16, 0.002, 0.95, 0.95, 0.98, collide=False))

    # D. HAZARD HATCHING AROUND INBOUND DOCK (Zone A: X in [-13.2, -10.2], Y in [6.5, 11.5])
    # Dock Apron Base Pad (Subtle concrete seal tint)
    sdf.append(box_link("dock_apron_pad", -12.0, 9.0, 0.002, 4.4, 6.0, 0.002, 0.28, 0.35, 0.30, collide=False))
    # Diagonal yellow/black hazard borders around dock staging
    # Outer dock yellow boundary
    sdf.append(box_link("dock_border_e", -10.0, 9.0, 0.003, 0.12, 6.0, 0.002, 0.95, 0.78, 0.05, collide=False))
    sdf.append(box_link("dock_border_s", -12.0, 6.0, 0.003, 4.0, 0.12, 0.002, 0.95, 0.78, 0.05, collide=False))
    # Diagonal hazard stripes (45 degree angled stripes)
    for hi, hy in enumerate(range(7, 12)):
        sdf.append(box_link(f"dock_hazard_{hi}", -10.2, float(hy), 0.004, 0.3, 0.6, 0.002, 0.10, 0.10, 0.10, collide=False, yaw=0.785))

    # E. HAZARD HATCHING AROUND OUTBOUND PACKING (Zone C: X in [-13.2, -10.2], Y in [-11.5, -6.5])
    sdf.append(box_link("packing_apron_pad", -12.0, -9.0, 0.002, 4.4, 6.0, 0.002, 0.38, 0.32, 0.24, collide=False))
    sdf.append(box_link("packing_border_e", -10.0, -9.0, 0.003, 0.12, 6.0, 0.002, 0.95, 0.78, 0.05, collide=False))
    sdf.append(box_link("packing_border_n", -12.0, -6.0, 0.003, 4.0, 0.12, 0.002, 0.95, 0.78, 0.05, collide=False))
    for hi, hy in enumerate(range(-11, -6)):
        sdf.append(box_link(f"packing_hazard_{hi}", -10.2, float(hy), 0.004, 0.3, 0.6, 0.002, 0.10, 0.10, 0.10, collide=False, yaw=-0.785))

    # F. AMR CHARGING BAY PARKING STALLS (Zone D: Bays 01 to 05 at X = -12.5, Y = -2.0 to 2.0)
    charging_stalls = [
        ("01_Alpha",   -2.0, 0.12, 0.45, 0.95),
        ("02_Beta",    -1.0, 0.95, 0.15, 0.15),
        ("03_Gamma",    0.0, 0.15, 0.85, 0.25),
        ("04_Delta",    1.0, 0.95, 0.85, 0.10),
        ("05_Epsilon",  2.0, 0.75, 0.18, 0.92),
    ]
    # Blue charging depot zone background pad
    sdf.append(box_link("charging_depot_pad", -12.4, 0.0, 0.002, 2.4, 5.2, 0.002, 0.18, 0.28, 0.40, collide=False))
    for sname, sy, cr, cg, cb in charging_stalls:
        # Yellow stall outline
        sdf.append(box_link(f"stall_box_{sname}", -12.5, sy, 0.003, 1.3, 0.85, 0.002, 0.95, 0.78, 0.05, collide=False))
        # Fleet-color identification inner pad
        sdf.append(box_link(f"stall_pad_{sname}", -12.5, sy, 0.004, 1.15, 0.70, 0.002, cr*0.4, cg*0.4, cb*0.4, collide=False))
        # High-visibility charging lightning bolt graphic (center disc)
        sdf.append(cylinder_link(f"stall_bolt_{sname}", -12.5, sy, 0.005, 0.15, 0.002, 0.98, 0.98, 0.20, collide=False))
        # Wheel stop position guides
        sdf.append(box_link(f"wheel_stop_l_{sname}", -12.9, sy - 0.25, 0.015, 0.25, 0.08, 0.03, 0.95, 0.78, 0.05))
        sdf.append(box_link(f"wheel_stop_r_{sname}", -12.9, sy + 0.25, 0.015, 0.25, 0.08, 0.03, 0.95, 0.78, 0.05))

    sdf.append("    </model>\n")

    # =========================================================================
    # 4. STORAGE RACKS (ASRS MODULES) WITH DIAGONAL X-BRACES & DECKS
    # =========================================================================
    # 12 Racking modules (6 West, 6 East)
    rack_defs = [
        ("Rack_A", -6.0, -7.5),
        ("Rack_B", -6.0, -4.5),
        ("Rack_C", -6.0, -1.5),
        ("Rack_D", -6.0, 1.5),
        ("Rack_E", -6.0, 4.5),
        ("Rack_F", -6.0, 7.5),
        ("Rack_A_East", 5.5, -7.5),
        ("Rack_B_East", 5.5, -4.5),
        ("Rack_C_East", 5.5, -1.5),
        ("Rack_D_East", 5.5, 1.5),
        ("Rack_E_East", 5.5, 4.5),
        ("Rack_F_East", 5.5, 7.5),
    ]

    for rack_name, rx, ry in rack_defs:
        sdf.append(f"""
    <!-- Industrial Storage {rack_name} -->
    <model name="{rack_name}">
      <static>true</static>
      <pose>{rx:.2f} {ry:.2f} 0 0 0 0</pose>
""")
        # 5 Pairs of Heavy Upright Columns (Industrial Blue: 0.08 0.32 0.75)
        col_x = [-4.0, -2.0, 0.0, 2.0, 4.0]
        col_y = [-0.5, 0.5]
        for ci, cx in enumerate(col_x):
            for cj, cy in enumerate(col_y):
                sdf.append(box_link(f"col_{ci}_{cj}", cx, cy, 1.8, 0.10, 0.10, 3.6, 0.08, 0.32, 0.75))
                # Base foot plate
                sdf.append(box_link(f"foot_{ci}_{cj}", cx, cy, 0.02, 0.18, 0.18, 0.04, 0.95, 0.75, 0.05))

        # Structural Diagonal X-Bracing on Rack Outer End Frames (X = -4.0 and X = +4.0)
        for end_x, end_suffix in [(-4.0, "w"), (4.0, "e")]:
            # Lower X-brace
            sdf.append(box_link(f"x_brace_low_1_{end_suffix}", end_x, 0.0, 0.9, 0.04, 1.05, 0.04, 0.08, 0.32, 0.75, collide=False, roll=0.55))
            sdf.append(box_link(f"x_brace_low_2_{end_suffix}", end_x, 0.0, 0.9, 0.04, 1.05, 0.04, 0.08, 0.32, 0.75, collide=False, roll=-0.55))
            # Upper X-brace
            sdf.append(box_link(f"x_brace_hi_1_{end_suffix}", end_x, 0.0, 2.3, 0.04, 1.05, 0.04, 0.08, 0.32, 0.75, collide=False, roll=0.55))
            sdf.append(box_link(f"x_brace_hi_2_{end_suffix}", end_x, 0.0, 2.3, 0.04, 1.05, 0.04, 0.08, 0.32, 0.75, collide=False, roll=-0.55))

        # Shelves (3 Tiers at Z = 0.6m, 1.6m, 2.6m)
        shelf_heights = [0.6, 1.6, 2.6]
        for si, sz in enumerate(shelf_heights):
            # Heavy Load Beams (OSHA Safety Orange: 0.92 0.38 0.06)
            # Front Beam
            sdf.append(box_link(f"beam_front_{si}", 0.0, 0.5, sz, 8.2, 0.08, 0.12, 0.92, 0.38, 0.06))
            # Rear Beam
            sdf.append(box_link(f"beam_rear_{si}", 0.0, -0.5, sz, 8.2, 0.08, 0.12, 0.92, 0.38, 0.06))
            # Galvanized Steel / Wire Deck Surface
            sdf.append(box_link(f"deck_{si}", 0.0, 0.0, sz + 0.04, 8.0, 0.94, 0.04, 0.58, 0.60, 0.62))
            # Pallet Safety Backstop Rail (running along rack center at Z = sz + 0.15m)
            sdf.append(box_link(f"backstop_{si}", 0.0, 0.0, sz + 0.15, 8.1, 0.04, 0.10, 0.95, 0.78, 0.05, collide=False))

        # Aisle Corner Heavy-Duty Curved Safety Bollards (protecting rack ends from AMR turn clipping)
        # Positioned slightly off rack outer corners
        for bi, (bx_off, by_off) in enumerate([(-4.35, -0.65), (-4.35, 0.65), (4.35, -0.65), (4.35, 0.65)]):
            sdf.append(cylinder_link(f"bollard_{bi}", bx_off, by_off, 0.4, 0.06, 0.8, 0.95, 0.75, 0.05))
            # Black top cap
            sdf.append(cylinder_link(f"bollard_cap_{bi}", bx_off, by_off, 0.82, 0.065, 0.04, 0.15, 0.15, 0.15))

        sdf.append("    </model>\n")

    # =========================================================================
    # 5. REALISTIC WOODEN PALLETS (Top Slats, Stringers, Bottom Boards)
    # =========================================================================
    # Standard GMA/Euro Pallets: 1.2m x 0.8m x 0.144m
    pallet_positions = [
        ("pallet_dock_1", -12.0, 9.5, 0.0),
        ("pallet_dock_2", -12.0, 7.5, 0.0),
        ("pallet_pack_1", -12.0, -9.5, 0.0),
        ("pallet_pack_2", -12.0, -7.5, 0.0),
        ("pallet_aisle_end1", 12.0, -6.0, 0.0),
        ("pallet_aisle_end2", 12.0, 6.0, 0.0),
        ("pallet_stage_dock", -14.0, 9.5, 0.0),
        ("pallet_stage_pack", -14.0, -9.5, 0.0)
    ]
    for pname, px, py, pz in pallet_positions:
        sdf.append(f"""
    <!-- Realistic Wooden Pallet {pname} -->
    <model name="{pname}">
      <static>true</static>
      <pose>{px:.2f} {py:.2f} {pz:.2f} 0 0 0</pose>
""")
        # 3 Solid Wood Stringer Runners (along X axis, length 1.2m, height 0.09m, width 0.05m)
        for ri, ry in enumerate([-0.36, 0.0, 0.36]):
            sdf.append(box_link(f"stringer_{ri}", 0.0, ry, 0.065, 1.2, 0.05, 0.09, 0.58, 0.44, 0.28))
        # 5 Top Deck Slat Boards (across Y axis, length 0.8m, width 0.12m, thickness 0.022m)
        for si, sx in enumerate([-0.50, -0.25, 0.0, 0.25, 0.50]):
            sdf.append(box_link(f"top_slat_{si}", sx, 0.0, 0.12, 0.12, 0.80, 0.022, 0.65, 0.50, 0.32))
        # 3 Bottom Base Boards
        for bi, bx in enumerate([-0.50, 0.0, 0.50]):
            sdf.append(box_link(f"bot_slat_{bi}", bx, 0.0, 0.01, 0.12, 0.80, 0.02, 0.55, 0.42, 0.26))
        sdf.append("    </model>\n")

    # =========================================================================
    # 6. CARDBOARD PACKAGE BOXES (Biscuit / Light Cardboard Brown Variations)
    # =========================================================================
    box_placements = []
    pid_counter = 1

    # West Racks (Rack_A to Rack_F)
    west_rack_ys = [-7.5, -4.5, -1.5, 1.5, 4.5, 7.5]
    west_rack_xs = [-9.0, -7.5, -6.0, -4.5, -3.0]
    for y_coord in west_rack_ys:
        # Tier 1 (lower shelf level at z=0.82)
        for x_coord in west_rack_xs:
            sx = 0.40 + ((pid_counter * 7) % 9) * 0.01
            sy = 0.38 + ((pid_counter * 13) % 7) * 0.01
            sz = 0.34 + ((pid_counter * 3) % 5) * 0.01
            box_placements.append((f"P{pid_counter:03d}", x_coord, y_coord, 0.82, round(sx, 2), round(sy, 2), round(sz, 2)))
            pid_counter += 1
        # Tier 2 (upper shelf level at z=1.82)
        for x_coord in [west_rack_xs[0], west_rack_xs[2], west_rack_xs[4]]:
            sx = 0.42 + ((pid_counter * 5) % 8) * 0.01
            sy = 0.40 + ((pid_counter * 11) % 6) * 0.01
            sz = 0.35 + ((pid_counter * 2) % 5) * 0.01
            box_placements.append((f"P{pid_counter:03d}", x_coord, y_coord, 1.82, round(sx, 2), round(sy, 2), round(sz, 2)))
            pid_counter += 1

    # East Racks (Rack_A_East to Rack_F_East)
    east_rack_ys = [-7.5, -4.5, -1.5, 1.5, 4.5, 7.5]
    east_rack_xs = [3.0, 4.5, 6.0, 7.5, 9.0]
    for y_coord in east_rack_ys:
        for x_coord in east_rack_xs:
            sx = 0.41 + ((pid_counter * 7) % 8) * 0.01
            sy = 0.39 + ((pid_counter * 13) % 6) * 0.01
            sz = 0.34 + ((pid_counter * 3) % 5) * 0.01
            box_placements.append((f"P{pid_counter:03d}", x_coord, y_coord, 0.82, round(sx, 2), round(sy, 2), round(sz, 2)))
            pid_counter += 1
        for x_coord in [east_rack_xs[1], east_rack_xs[3]]:
            sx = 0.43 + ((pid_counter * 5) % 7) * 0.01
            sy = 0.40 + ((pid_counter * 11) % 5) * 0.01
            sz = 0.35 + ((pid_counter * 2) % 5) * 0.01
            box_placements.append((f"P{pid_counter:03d}", x_coord, y_coord, 1.82, round(sx, 2), round(sy, 2), round(sz, 2)))
            pid_counter += 1

    # Staging Pallets & Docks
    for px, py in [(-12.0, 9.5), (-12.0, 7.5), (-12.0, -9.5), (-12.0, -7.5), (12.0, -6.0), (12.0, 6.0)]:
        box_placements.append((f"P{pid_counter:03d}", px, py, 0.32, 0.48, 0.42, 0.36))
        pid_counter += 1

    # Cardboard color nuances (3 subtle shades of biscuit / kraft brown)
    cardboard_shades = [
        (0.82, 0.69, 0.51), # Classic biscuit / light cardboard brown
        (0.76, 0.61, 0.43), # Medium natural kraft
        (0.86, 0.73, 0.56), # Pale corrugated boxboard
    ]

    for idx, (pid, bx, by, bz, bsx, bsy, bsz) in enumerate(box_placements):
        cr, cg, cb = cardboard_shades[idx % len(cardboard_shades)]
        sdf.append(f"""
    <!-- Cardboard Package {pid} (Biscuit / Light Cardboard Brown) -->
    <model name="package_{pid}">
      <static>true</static>
      <pose>{bx:.2f} {by:.2f} {bz:.2f} 0 0 0</pose>
      <link name="box_link">
        <collision name="box_col">
          <geometry><box><size>{bsx:.2f} {bsy:.2f} {bsz:.2f}</size></box></geometry>
        </collision>
        <visual name="box_vis">
          <geometry><box><size>{bsx:.2f} {bsy:.2f} {bsz:.2f}</size></box></geometry>
          <material>
            <ambient>{cr*0.7:.2f} {cg*0.7:.2f} {cb*0.7:.2f} 1.0</ambient>
            <diffuse>{cr:.2f} {cg:.2f} {cb:.2f} 1.0</diffuse>
            <specular>0.08 0.08 0.08 1.0</specular>
          </material>
        </visual>
        <!-- Dark Brown Sealing Tape running along top seam -->
        <visual name="top_tape">
          <pose>0 0 {bsz/2.0 + 0.001:.4f} 0 0 0</pose>
          <geometry><box><size>{bsx + 0.002:.3f} 0.05 0.002</size></box></geometry>
          <material>
            <ambient>0.48 0.35 0.20 1.0</ambient>
            <diffuse>0.55 0.40 0.24 1.0</diffuse>
          </material>
        </visual>
        <!-- White Barcode Shipping Label -->
        <visual name="shipping_label">
          <pose>{bsx*0.15:.3f} {bsy*0.2:.3f} {bsz/2.0 + 0.002:.4f} 0 0 0</pose>
          <geometry><box><size>0.14 0.10 0.002</size></box></geometry>
          <material>
            <ambient>0.92 0.92 0.92 1.0</ambient>
            <diffuse>0.98 0.98 0.98 1.0</diffuse>
          </material>
        </visual>
      </link>
    </model>
""")

    # =========================================================================
    # 7. DEDICATED WORKSTATIONS & INDUSTRIAL EQUIPMENT
    # =========================================================================
    # Packing Workbenches & Gravity Roller Conveyor (Zone C, placed against West wall at X = -14.5)
    sdf.append("""
    <!-- Packing Workstations & Equipment (Zone C) -->
    <model name="packing_equipment">
      <static>true</static>
""")
    # 3 Heavy-Duty Steel Packing Workbenches (Y = -6.0, -8.0, -10.0)
    for bi, by in enumerate([-6.0, -8.0, -10.0]):
        # Maple butcher-block tabletop
        sdf.append(box_link(f"bench_top_{bi}", -14.6, by, 0.90, 0.85, 1.5, 0.08, 0.78, 0.60, 0.42))
        # Steel tubular legs (4 legs)
        for li, (lx, ly) in enumerate([(-14.9, by - 0.65), (-14.9, by + 0.65), (-14.3, by - 0.65), (-14.3, by + 0.65)]):
            sdf.append(box_link(f"bench_leg_{bi}_{li}", lx, ly, 0.43, 0.06, 0.06, 0.86, 0.20, 0.22, 0.25))
        # Overhead tool/scanner gantry post
        sdf.append(box_link(f"bench_gantry_{bi}", -14.95, by, 1.45, 0.05, 1.4, 1.0, 0.20, 0.22, 0.25, collide=False))
        # Packing terminal monitor display screen
        sdf.append(box_link(f"bench_screen_{bi}", -14.8, by, 1.25, 0.06, 0.40, 0.28, 0.10, 0.10, 0.10, collide=False))
        sdf.append(box_link(f"bench_display_{bi}", -14.76, by, 1.25, 0.01, 0.36, 0.24, 0.15, 0.55, 0.85, collide=False, emissive=(0.1, 0.4, 0.7)))

    # Gravity Roller Conveyor Section (Length 3.2m along Y = -7.5 to -10.5, X = -13.8)
    # Side steel channel rails
    sdf.append(box_link("conveyor_rail_l", -14.15, -9.0, 0.80, 0.05, 3.2, 0.10, 0.12, 0.35, 0.75))
    sdf.append(box_link("conveyor_rail_r", -13.65, -9.0, 0.80, 0.05, 3.2, 0.10, 0.12, 0.35, 0.75))
    # Support stands (3 leg pairs)
    for si, sy in enumerate([-10.4, -9.0, -7.6]):
        sdf.append(box_link(f"conveyor_stand_{si}", -13.9, sy, 0.38, 0.50, 0.06, 0.76, 0.20, 0.22, 0.25))
    # 12 Galvanized Steel Roller Cylinders
    for ri, ry in enumerate([y * 0.24 for y in range(-42, -30)]):
        sdf.append(cylinder_link(f"roller_{ri}", -13.9, ry, 0.82, 0.035, 0.45, 0.75, 0.78, 0.82, roll=1.5708, collide=False))

    # Hydraulic Loading Dock Leveler Plate (Zone A: X = -15.2, Y = 8.5)
    # Textured steel dock plate
    sdf.append(box_link("dock_leveler_plate", -15.2, 8.5, 0.02, 1.8, 2.4, 0.04, 0.24, 0.28, 0.32))
    # Yellow safety side curbs
    sdf.append(box_link("dock_curb_n", -15.2, 9.7, 0.06, 1.8, 0.08, 0.12, 0.95, 0.78, 0.05))
    sdf.append(box_link("dock_curb_s", -15.2, 7.3, 0.06, 1.8, 0.08, 0.12, 0.95, 0.78, 0.05))

    sdf.append("    </model>\n")

    # =========================================================================
    # 8. AMR FLEET CHARGING TERMINALS (Zone D)
    # =========================================================================
    charging_bays = [
        ("charging_bay_alpha",   -13.8, -2.0, "Alpha (Blue)",   0.12, 0.45, 0.95),
        ("charging_bay_beta",    -13.8, -1.0, "Beta (Red)",     0.95, 0.15, 0.15),
        ("charging_bay_gamma",   -13.8,  0.0, "Gamma (Green)",  0.15, 0.85, 0.25),
        ("charging_bay_delta",   -13.8,  1.0, "Delta (Yellow)", 0.95, 0.85, 0.10),
        ("charging_bay_epsilon", -13.8,  2.0, "Epsilon (Purple)", 0.75, 0.18, 0.92),
    ]
    for bay_name, bx, by, blabel, cr, cg, cb in charging_bays:
        sdf.append(f"""
    <!-- Charging Station {blabel} -->
    <model name="{bay_name}">
      <static>true</static>
      <pose>{bx:.2f} {by:.2f} 0.0 0 0 0</pose>
      <link name="terminal_housing">
        <!-- Vertical Column Pylon -->
        <pose>0 0 0.55 0 0 0</pose>
        <collision name="pylon_col">
          <geometry><box><size>0.25 0.35 1.1</size></box></geometry>
        </collision>
        <visual name="pylon_vis">
          <geometry><box><size>0.25 0.35 1.1</size></box></geometry>
          <material>
            <ambient>0.22 0.25 0.28 1.0</ambient>
            <diffuse>0.30 0.34 0.38 1.0</diffuse>
          </material>
        </visual>
        <!-- Fleet Identity Color Stripe -->
        <visual name="color_accent">
          <pose>0.13 0 0.25 0 0 0</pose>
          <geometry><box><size>0.01 0.33 0.15</size></box></geometry>
          <material>
            <ambient>{cr*0.7:.2f} {cg*0.7:.2f} {cb*0.7:.2f} 1.0</ambient>
            <diffuse>{cr:.2f} {cg:.2f} {cb:.2f} 1.0</diffuse>
          </material>
        </visual>
        <!-- Illuminated LED Status Display -->
        <visual name="status_led">
          <pose>0.13 0 0.42 0 0 0</pose>
          <geometry><box><size>0.01 0.22 0.10</size></box></geometry>
          <material>
            <ambient>0.1 0.8 0.3 1.0</ambient>
            <diffuse>0.2 0.95 0.4 1.0</diffuse>
            <emissive>0.2 0.9 0.3 1.0</emissive>
          </material>
        </visual>
        <!-- Copper Charging Contact Spring Plates -->
        <visual name="charge_contact">
          <pose>0.135 0 -0.35 0 0 0</pose>
          <geometry><box><size>0.02 0.24 0.08</size></box></geometry>
          <material>
            <ambient>0.75 0.45 0.15 1.0</ambient>
            <diffuse>0.85 0.55 0.20 1.0</diffuse>
          </material>
        </visual>
      </link>
    </model>
""")

    # =========================================================================
    # 9. WAREHOUSE INDUSTRIAL SIGNAGE & SAFETY EQUIPMENT
    # =========================================================================
    sdf.append("""
    <!-- Warehouse Overhead Industrial Signage & Safety Equipment -->
    <model name="warehouse_signage">
      <static>true</static>
""")
    # A. Overhead Hanging Zone Signs (Suspended at Z = 3.8m from ceiling trusses)
    # Zone A: Inbound Receiving
    sdf.append(box_link("sign_zone_a_plate", -11.5, 8.5, 3.8, 0.08, 3.2, 0.75, 0.95, 0.78, 0.05, collide=False))
    sdf.append(box_link("sign_zone_a_inner", -11.45, 8.5, 3.8, 0.01, 3.0, 0.65, 0.12, 0.14, 0.16, collide=False))
    # Suspension rods
    sdf.append(cylinder_link("sign_zone_a_rod1", -11.5, 7.5, 4.35, 0.015, 1.1, 0.2, 0.2, 0.2, collide=False))
    sdf.append(cylinder_link("sign_zone_a_rod2", -11.5, 9.5, 4.35, 0.015, 1.1, 0.2, 0.2, 0.2, collide=False))

    # Zone B: ASRS Storage Racks
    sdf.append(box_link("sign_zone_b_plate", -0.5, 0.0, 3.8, 0.08, 3.4, 0.75, 0.95, 0.78, 0.05, collide=False))
    sdf.append(box_link("sign_zone_b_inner", -0.45, 0.0, 3.8, 0.01, 3.2, 0.65, 0.12, 0.14, 0.16, collide=False))
    sdf.append(cylinder_link("sign_zone_b_rod1", -0.5, -1.0, 4.35, 0.015, 1.1, 0.2, 0.2, 0.2, collide=False))
    sdf.append(cylinder_link("sign_zone_b_rod2", -0.5, 1.0, 4.35, 0.015, 1.1, 0.2, 0.2, 0.2, collide=False))

    # Zone C: Outbound Packing & Shipping
    sdf.append(box_link("sign_zone_c_plate", -11.5, -8.5, 3.8, 0.08, 3.2, 0.75, 0.95, 0.78, 0.05, collide=False))
    sdf.append(box_link("sign_zone_c_inner", -11.45, -8.5, 3.8, 0.01, 3.0, 0.65, 0.12, 0.14, 0.16, collide=False))
    sdf.append(cylinder_link("sign_zone_c_rod1", -11.5, -9.5, 4.35, 0.015, 1.1, 0.2, 0.2, 0.2, collide=False))
    sdf.append(cylinder_link("sign_zone_c_rod2", -11.5, -7.5, 4.35, 0.015, 1.1, 0.2, 0.2, 0.2, collide=False))

    # Zone D: AMR Charging Depot
    sdf.append(box_link("sign_zone_d_plate", -13.0, 0.0, 3.6, 0.08, 2.8, 0.65, 0.12, 0.45, 0.95, collide=False))
    sdf.append(cylinder_link("sign_zone_d_rod1", -13.0, -0.8, 4.25, 0.015, 1.3, 0.2, 0.2, 0.2, collide=False))
    sdf.append(cylinder_link("sign_zone_d_rod2", -13.0, 0.8, 4.25, 0.015, 1.3, 0.2, 0.2, 0.2, collide=False))

    # B. Aisle Hanging Signs at Rack Entrances (AISLE 01 - AISLE 05 at West rack ends)
    aisle_sign_ys = [-6.0, -3.0, 0.0, 3.0, 6.0]
    for ai, ay in enumerate(aisle_sign_ys):
        # Aisle banner plaque (suspended at entrance at X = -7.8, Z = 2.9m)
        sdf.append(box_link(f"aisle_sign_plate_{ai}", -7.8, ay, 2.9, 0.05, 0.90, 0.35, 0.95, 0.78, 0.05, collide=False))
        sdf.append(box_link(f"aisle_sign_inner_{ai}", -7.78, ay, 2.9, 0.01, 0.82, 0.28, 0.15, 0.15, 0.18, collide=False))
        # Hanger bracket
        sdf.append(cylinder_link(f"aisle_hanger_{ai}", -7.8, ay, 3.25, 0.012, 0.5, 0.3, 0.3, 0.3, collide=False))

    # C. Wall-Mounted Industrial Fire Extinguisher Cabinets (Red Enclosures)
    for fi, (fx, fy) in enumerate([(-15.8, -3.0), (-15.8, 3.0), (15.8, -3.0), (15.8, 3.0)]):
        # Red cabinet box
        sdf.append(box_link(f"extinguisher_box_{fi}", fx, fy, 1.2, 0.25, 0.35, 0.75, 0.85, 0.12, 0.12))
        # Inspection glass window
        sdf.append(box_link(f"extinguisher_glass_{fi}", fx + (0.13 if fx < 0 else -0.13), fy, 1.2, 0.01, 0.22, 0.50, 0.9, 0.95, 1.0, collide=False, a=0.7))
        # Location sign board above
        sdf.append(box_link(f"extinguisher_sign_{fi}", fx + (0.13 if fx < 0 else -0.13), fy, 1.8, 0.01, 0.25, 0.25, 0.85, 0.12, 0.12, collide=False))

    # D. Safety Yellow Traffic Cones near Inbound Dock & Storage
    for ci, (cx, cy) in enumerate([(-10.5, 6.2), (-10.5, -5.8), (11.0, -1.0)]):
        sdf.append(cylinder_link(f"cone_base_{ci}", cx, cy, 0.015, 0.20, 0.03, 0.95, 0.45, 0.05))
        sdf.append(cylinder_link(f"cone_stem_{ci}", cx, cy, 0.30, 0.12, 0.55, 0.95, 0.45, 0.05))
        # White reflective collar
        sdf.append(cylinder_link(f"cone_collar_{ci}", cx, cy, 0.35, 0.125, 0.12, 0.95, 0.95, 0.95, collide=False))

    sdf.append("    </model>\n")

    # =========================================================================
    # 10. DIRECT AMR FLEET EMBEDDING (Zero Spawner Timeout, Instant Frame-1 Init)
    # =========================================================================
    desc_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "warehouse_3d_description"))
    xacro_path = os.path.join(desc_dir, "urdf", "warehouse_amr.urdf.xacro")
    if not os.path.exists(xacro_path):
        try:
            from ament_index_python.packages import get_package_share_directory
            xacro_path = os.path.join(get_package_share_directory('warehouse_3d_description'), 'urdf', 'warehouse_amr.urdf.xacro')
        except Exception:
            pass

    robot_fleet = [
        {"name": "Alpha",   "color": "Blue",   "ns": "robot_alpha",   "id": 1, "x": -12.5, "y": -2.0, "z": 0.05, "yaw": 0.0},
        {"name": "Beta",    "color": "Red",    "ns": "robot_beta",    "id": 2, "x": -12.5, "y": -1.0, "z": 0.05, "yaw": 0.0},
        {"name": "Gamma",   "color": "Green",  "ns": "robot_gamma",   "id": 3, "x": -12.5, "y":  0.0, "z": 0.05, "yaw": 0.0},
        {"name": "Delta",   "color": "Yellow", "ns": "robot_delta",   "id": 4, "x": -12.5, "y":  1.0, "z": 0.05, "yaw": 0.0},
        {"name": "Epsilon", "color": "Purple", "ns": "robot_epsilon", "id": 5, "x": -12.5, "y":  2.0, "z": 0.05, "yaw": 0.0},
    ]

    for rob in robot_fleet:
        try:
            res_xacro = subprocess.run([
                'xacro', xacro_path,
                f"robot_name:={rob['name']}",
                f"robot_color:={rob['color']}",
                f"robot_namespace:={rob['ns']}",
                f"robot_id:={rob['id']}"
            ], capture_output=True, text=True, check=True)

            tmp_urdf = f"/tmp/{rob['ns']}_gen.urdf"
            with open(tmp_urdf, "w") as f:
                f.write(res_xacro.stdout)

            res_sdf = subprocess.run(['gz', 'sdf', '-p', tmp_urdf], capture_output=True, text=True, check=True)
            m = re.search(r"<model name='warehouse_amr'>(.*?)</model>", res_sdf.stdout, re.DOTALL)
            if m:
                inner = m.group(1)
                sdf.append(f"""
    <!-- Autonomous Mobile Robot: {rob['name']} ({rob['color']}) -->
    <model name="amr_{rob['ns']}">
      <pose>{rob['x']} {rob['y']} {rob['z']} 0 0 {rob['yaw']}</pose>
{inner}
    </model>
""")
        except Exception as e:
            print(f"Warning: Could not embed robot {rob['ns']}: {e}")

    sdf.append("""
  </world>
</sdf>
""")

    os.makedirs(os.path.dirname(output_path), exist_ok=True)
    with open(output_path, "w") as f:
        f.write("".join(sdf))
    print(f"Successfully generated presentation-ready warehouse world SDF at: {output_path}")

if __name__ == "__main__":
    out_file = os.path.abspath(os.path.join(os.path.dirname(__file__), "worlds", "warehouse.sdf"))
    if len(sys.argv) > 1:
        out_file = sys.argv[1]
    generate_warehouse_sdf(out_file)
