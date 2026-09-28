#!/usr/bin/env python3
"""
High-Fidelity 4x Expanded Dynamic Logistics Warehouse 3D World Generator for Open-RMF.
Expands the warehouse floor to 4x size (64m x 52m building, 76m x 62m slab).
Provides wide open 4.8m-5.8m aisles, 4.0m arterial highways, high 9.0m ceiling,
unobstructed camera sightlines, and full AWS RoboMaker dynamic warehouse assets
(ShelfE, ShelfF, ShelfD, PalletJack, Desk, Cluttering, TrashCan, Lamp).
"""

import os
import sys

def box_link(name, x, y, z, sx, sy, sz, r, g, b, a=1.0, collide=False, roll=0.0, pitch=0.0, yaw=0.0, emissive=(0,0,0)):
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

def cylinder_link(name, x, y, z, radius, length, r, g, b, a=1.0, collide=False, roll=0.0, pitch=0.0, yaw=0.0, emissive=(0,0,0)):
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

def generate_expanded_warehouse_world(output_path):
    sdf = []
    # Header & World Definition
    sdf.append("""<sdf version="1.7">
  <world name="sim_world">
    <!-- Physics Configuration -->
    <physics name="10ms" type="ode">
      <max_step_size>0.01</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>

    <!-- Essential Gazebo Plugins -->
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics" />
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands" />
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster" />
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
    <plugin filename="gz-sim-imu-system" name="gz::sim::systems::Imu" />

    <!-- Scene & Ambient Environment -->
    <scene>
      <ambient>0.74 0.76 0.80 1.0</ambient>
      <background>0.16 0.18 0.22 1.0</background>
      <shadows>false</shadows>
      <grid>false</grid>
    </scene>

    <!-- 3D GUI Camera Viewport Overview -->
    <gui fullscreen="0">
      <camera name="user_camera">
        <pose>40.0 -85.0 52.0 0 0.85 1.5708</pose>
        <view_controller>orbit</view_controller>
      </camera>
    </gui>

    <!-- Primary Directional Sunlight -->
    <light type="directional" name="sun_primary">
      <cast_shadows>false</cast_shadows>
      <pose>40.0 -32.5 25.0 0 0 0</pose>
      <diffuse>0.88 0.88 0.92 1.0</diffuse>
      <specular>0.25 0.25 0.25 1.0</specular>
      <attenuation>
        <range>150</range>
        <constant>0.9</constant>
        <linear>0.01</linear>
        <quadratic>0.001</quadratic>
      </attenuation>
      <direction>-0.3 0.2 -0.9</direction>
    </light>
""")

    # 15 High-Bay Point Lights across the 64m x 52m floor
    light_positions = [
        (16.0, -14.0, 9.0), (16.0, -32.5, 9.0), (16.0, -50.0, 9.0),
        (30.0, -14.0, 9.0), (30.0, -32.5, 9.0), (30.0, -50.0, 9.0),
        (42.0, -14.0, 9.0), (42.0, -32.5, 9.0), (42.0, -50.0, 9.0),
        (54.0, -14.0, 9.0), (54.0, -32.5, 9.0), (54.0, -50.0, 9.0),
        (66.0, -14.0, 9.0), (66.0, -32.5, 9.0), (66.0, -50.0, 9.0),
    ]
    for i, (lx, ly, lz) in enumerate(light_positions):
        sdf.append(f"""
    <light type="point" name="bay_light_{i}">
      <pose>{lx:.1f} {ly:.1f} {lz:.1f} 0 0 0</pose>
      <diffuse>0.78 0.82 0.86 1.0</diffuse>
      <specular>0.20 0.20 0.25 1.0</specular>
      <attenuation>
        <range>28</range>
        <constant>0.22</constant>
        <linear>0.04</linear>
        <quadratic>0.005</quadratic>
      </attenuation>
      <cast_shadows>false</cast_shadows>
    </light>
""")

    # High-Bay Hanging Lamps (AWS RoboMaker) at Z = 8.8m
    for i, (lx, ly, lz) in enumerate(light_positions):
        sdf.append(f"""
    <include>
      <name>warehouse_lamp_{i}</name>
      <uri>model://aws_robomaker_warehouse_Lamp_01</uri>
      <pose>{lx:.1f} {ly:.1f} 8.8 0 0 0</pose>
      <static>true</static>
    </include>
""")

    # =========================================================================
    # 1. 4X GROUND FLOOR SLAB (Centered at X=40.0, Y=-32.5)
    # =========================================================================
    sdf.append("""
    <!-- 4x Expanded Warehouse Ground Floor Slab (76m x 62m) -->
    <model name="warehouse_floor">
      <static>true</static>
      <pose>40.0 -32.5 0.0 0 0 0</pose>
      <link name="floor_link">
        <collision name="floor_collision">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>76 62</size>
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
              <size>76 62</size>
            </plane>
          </geometry>
          <material>
            <ambient>0.34 0.36 0.38 1.0</ambient>
            <diffuse>0.42 0.44 0.47 1.0</diffuse>
            <specular>0.15 0.15 0.15 1.0</specular>
          </material>
        </visual>
      </link>
    </model>
""")

    # =========================================================================
    # 2. PERIMETER ARCHITECTURE & WALLS (High 9.0m Ceiling, Unobstructed Center)
    # =========================================================================
    # Center: (40.0, -32.5)
    # North wall: Y = -6.5 -> offset Y = +26.0
    # South wall: Y = -58.5 -> offset Y = -26.0
    # East wall:  X = 72.0 -> offset X = +32.0
    # West wall:  X = 8.0  -> offset X = -32.0
    sdf.append("""
    <!-- Warehouse Perimeter Architecture (9.0m High Clear-Span Architecture) -->
    <model name="warehouse_architecture">
      <static>true</static>
      <pose>40.0 -32.5 0.0 0 0 0</pose>
""")
    # North Wall (Y = 26.0, length 64.8m)
    sdf.append(box_link("wall_north_base", 0.0, 26.0, 0.6, 64.8, 0.5, 1.2, 0.24, 0.27, 0.31, collide=True))
    sdf.append(box_link("wall_north_upper", 0.0, 26.0, 5.0, 64.8, 0.5, 7.8, 0.70, 0.72, 0.75, collide=True))
    sdf.append(box_link("wall_north_coping", 0.0, 26.0, 9.05, 65.2, 0.6, 0.3, 0.18, 0.20, 0.22))
    sdf.append(box_link("window_north", 0.0, 25.8, 7.0, 56.0, 0.05, 1.6, 0.45, 0.65, 0.85, a=0.7))

    # South Wall (Low Retaining Perimeter Wall - Architectural Cutaway for Unobstructed Sightlines)
    sdf.append(box_link("wall_south_base", 0.0, -26.0, 0.6, 64.8, 0.5, 1.2, 0.24, 0.27, 0.31, collide=True))
    sdf.append(box_link("wall_south_coping", 0.0, -26.0, 1.25, 65.2, 0.6, 0.1, 0.95, 0.75, 0.05))

    # East Wall (X = 32.0, width 52.0m)
    sdf.append(box_link("wall_east_base", 32.0, 0.0, 0.6, 0.5, 52.0, 1.2, 0.24, 0.27, 0.31, collide=True))
    sdf.append(box_link("wall_east_upper", 32.0, 0.0, 5.0, 0.5, 52.0, 7.8, 0.70, 0.72, 0.75, collide=True))
    sdf.append(box_link("wall_east_coping", 32.0, 0.0, 9.05, 0.6, 52.4, 0.3, 0.18, 0.20, 0.22))
    sdf.append(box_link("window_east", 31.8, 0.0, 7.0, 0.05, 44.0, 1.6, 0.45, 0.65, 0.85, a=0.7))

    # West Wall (X = -32.0, width 52.0m) with Inbound & Outbound Bay Openings
    # Inbound Dock at Y = -12.0 -> relative Y = +20.5
    # Outbound Dock at Y = -48.0 -> relative Y = -15.5
    sdf.append(box_link("wall_west_n", -32.0, 24.5, 4.5, 0.5, 3.0, 9.0, 0.70, 0.72, 0.75, collide=True))
    sdf.append(box_link("lintel_inbound", -32.0, 20.5, 7.0, 0.5, 5.0, 4.0, 0.24, 0.27, 0.31, collide=True))
    sdf.append(box_link("shutter_inbound", -31.8, 20.5, 2.5, 0.1, 4.8, 5.0, 0.35, 0.42, 0.52))

    sdf.append(box_link("wall_west_c", -32.0, 2.5, 4.5, 0.5, 31.0, 9.0, 0.70, 0.72, 0.75, collide=True))

    sdf.append(box_link("lintel_outbound", -32.0, -15.5, 7.0, 0.5, 5.0, 4.0, 0.24, 0.27, 0.31, collide=True))
    sdf.append(box_link("shutter_outbound", -31.8, -15.5, 2.5, 0.1, 4.8, 5.0, 0.35, 0.42, 0.52))

    sdf.append(box_link("wall_west_s", -32.0, -21.5, 4.5, 0.5, 7.0, 9.0, 0.70, 0.72, 0.75, collide=True))

    # Perimeter Steel Columns (Placed strictly along the walls to preserve wide open center)
    for col_x in [-31.6, 31.6]:
        for col_y in [-25.5, -13.0, 0.0, 13.0, 25.5]:
            sdf.append(box_link(f"pcol_{col_x}_{col_y}", col_x, col_y, 4.5, 0.6, 0.6, 9.0, 0.20, 0.23, 0.28, collide=True))
            sdf.append(box_link(f"pcol_base_{col_x}_{col_y}", col_x, col_y, 0.6, 0.8, 0.8, 1.2, 0.95, 0.75, 0.05))

    sdf.append("    </model>\\n")

    # =========================================================================
    # 3. HIGHWAY FLOOR MARKINGS & ZONE STRIPING (4X WIDE OPEN ROADS)
    # =========================================================================
    sdf.append("""
    <!-- 4x Wide Open Highway Safety Floor Markings -->
    <model name="industrial_floor_markings">
      <static>true</static>
      <pose>40.0 -32.5 0.0 0 0 0</pose>
""")
    # North-South AMR Main Arterial Highway at X = 20.0 (relative X = -20.0)
    # 4.0m Wide Roadway (Borders at relative X = -22.0 and -18.0)
    sdf.append(box_link("highway_line_w", -22.0, 0.0, 0.002, 0.15, 48.0, 0.001, 0.96, 0.78, 0.05))
    sdf.append(box_link("highway_line_e", -18.0, 0.0, 0.002, 0.15, 48.0, 0.001, 0.96, 0.78, 0.05))
    for di, dy in enumerate([y * 4.0 for y in range(-6, 7)]):
        sdf.append(box_link(f"highway_dash_{di}", -20.0, dy, 0.003, 0.12, 2.0, 0.001, 0.96, 0.96, 0.96))

    # East-West Central Cross-Highway at Y = -32.5 (relative Y = 0.0)
    # 4.0m Wide Roadway (Borders at relative Y = -2.0 and +2.0)
    sdf.append(box_link("cross_line_s", 0.0, -2.0, 0.002, 58.0, 0.15, 0.001, 0.96, 0.78, 0.05))
    sdf.append(box_link("cross_line_n", 0.0,  2.0, 0.002, 58.0, 0.15, 0.001, 0.96, 0.78, 0.05))
    for cdi, cdx in enumerate([x * 4.0 for x in range(-7, 8)]):
        sdf.append(box_link(f"cross_dash_{cdi}", cdx, 0.0, 0.003, 2.0, 0.12, 0.001, 0.96, 0.96, 0.96))

    # Crosswalks
    # Inbound Dock Crosswalk (Y = -12.0 -> relative Y = +20.5)
    for zi, zx in enumerate([-25.0 + i*0.8 for i in range(7)]):
        sdf.append(box_link(f"zebra_in_{zi}", zx, 20.5, 0.003, 0.45, 3.0, 0.001, 0.95, 0.95, 0.95))

    # Packing Crosswalk (Y = -51.0 -> relative Y = -18.5)
    for zi, zx in enumerate([-25.0 + i*0.8 for i in range(7)]):
        sdf.append(box_link(f"zebra_pack_{zi}", zx, -18.5, 0.003, 0.45, 3.0, 0.001, 0.95, 0.95, 0.95))

    # Floor Zones
    # Zone A (Inbound Dock): relative X = -25.5, Y = +18.5, size 10m x 10m
    sdf.append(box_link("zone_a_floor", -25.5, 18.5, 0.001, 10.0, 10.0, 0.001, 0.18, 0.35, 0.55, a=0.35))
    # Zone C (Packing): relative X = -25.5, Y = -18.5, size 10m x 12m
    sdf.append(box_link("zone_c_floor", -25.5, -18.5, 0.001, 10.0, 12.0, 0.001, 0.20, 0.50, 0.35, a=0.35))
    # Zone D (AMR Fleet Depot): relative X = -25.5, Y = 0.0, size 9m x 15m
    sdf.append(box_link("zone_d_floor", -25.5, 0.0, 0.001, 9.0, 15.0, 0.001, 0.25, 0.40, 0.55, a=0.25))
    # Zone B (Bulk Storage): relative X = +26.0, Y = 0.0, size 10m x 15m
    sdf.append(box_link("zone_b_floor",  26.0, 0.0, 0.001, 10.0, 15.0, 0.001, 0.45, 0.25, 0.55, a=0.30))

    # 5 Fleet Charger Bays (X = 13.5 -> relative X = -26.5)
    # Y coords: -28.5 (+4.0), -30.5 (+2.0), -32.5 (0.0), -34.5 (-2.0), -36.5 (-4.0)
    chargers_rel_y = [4.0, 2.0, 0.0, -2.0, -4.0]
    charger_colors = [(0.12, 0.45, 0.95), (0.95, 0.15, 0.15), (0.15, 0.85, 0.25), (0.95, 0.85, 0.10), (0.75, 0.18, 0.92)]
    for si, (sry, scol) in enumerate(zip(chargers_rel_y, charger_colors)):
        sname = f"bay_{si}"
        sdf.append(box_link(f"stall_box_{sname}",   -26.5, sry, 0.002, 2.4, 1.5, 0.001, scol[0], scol[1], scol[2], a=0.55))
        sdf.append(box_link(f"stall_border_{sname}",-26.5, sry, 0.003, 2.5, 1.6, 0.001, 0.96, 0.78, 0.05))
        sdf.append(cylinder_link(f"stall_bolt_{sname}", -26.5, sry, 0.005, 0.20, 0.002, 0.98, 0.98, 0.20))

    sdf.append("    </model>\\n")

    # =========================================================================
    # 4. HIGH-FIDELITY AWS ROBOMAKER STORAGE RACKING (48 Units, Spacious Aisles)
    # =========================================================================
    # 12 Racks, each composed of 4 connected AWS RoboMaker shelving units (10.2m length)
    # Exactly aligned 1:1 with RViz floorplan bounding boxes
    rack_defs = [
        # West Racks (Center X = 31.50)
        ("Rack_F",      31.50, -16.70, ["ShelfE_01", "ShelfF_01", "ShelfD_01", "ShelfE_01"]),
        ("Rack_E",      31.50, -21.70, ["ShelfF_01", "ShelfE_01", "ShelfE_01", "ShelfF_01"]),
        ("Rack_D",      31.50, -27.70, ["ShelfE_01", "ShelfD_01", "ShelfF_01", "ShelfE_01"]),
        ("Rack_C",      31.50, -39.70, ["ShelfD_01", "ShelfE_01", "ShelfF_01", "ShelfE_01"]),
        ("Rack_B",      31.50, -45.70, ["ShelfE_01", "ShelfF_01", "ShelfE_01", "ShelfD_01"]),
        ("Rack_A",      31.50, -50.70, ["ShelfF_01", "ShelfE_01", "ShelfD_01", "ShelfE_01"]),
        # East Racks (Center X = 52.50)
        ("Rack_F_East", 52.50, -16.70, ["ShelfF_01", "ShelfE_01", "ShelfF_01", "ShelfE_01"]),
        ("Rack_E_East", 52.50, -21.70, ["ShelfE_01", "ShelfD_01", "ShelfE_01", "ShelfF_01"]),
        ("Rack_D_East", 52.50, -27.70, ["ShelfF_01", "ShelfE_01", "ShelfD_01", "ShelfE_01"]),
        ("Rack_C_East", 52.50, -39.70, ["ShelfE_01", "ShelfF_01", "ShelfE_01", "ShelfD_01"]),
        ("Rack_B_East", 52.50, -45.70, ["ShelfD_01", "ShelfE_01", "ShelfF_01", "ShelfE_01"]),
        ("Rack_A_East", 52.50, -50.70, ["ShelfE_01", "ShelfF_01", "ShelfE_01", "ShelfF_01"]),
    ]

    unit_offsets = [-3.825, -1.275, 1.275, 3.825]  # 4 units side-by-side span 10.20m along X

    for r_idx, (rname, rx, ry, models) in enumerate(rack_defs):
        for u_idx, (dx, mtype) in enumerate(zip(unit_offsets, models)):
            ux = rx + dx
            sdf.append(f"""
    <!-- {rname} Unit {u_idx+1} ({mtype}) -->
    <include>
      <name>{rname}_unit_{u_idx+1}</name>
      <uri>model://aws_robomaker_warehouse_{mtype}</uri>
      <pose>{ux:.2f} {ry:.2f} 0.0 0 0 1.570796</pose>
      <static>true</static>
    </include>
""")

        # Safety Bollards at rack row corners (Visual cues, non-colliding to prevent robot wheel snags)
        sdf.append(f"""
    <model name="{rname}_bollards">
      <static>true</static>
      <pose>{rx:.2f} {ry:.2f} 0.0 0 0 0</pose>
""")
        sdf.append(cylinder_link("bollard_w1", -5.20, -0.55, 0.40, 0.08, 0.80, 0.95, 0.78, 0.05, collide=False))
        sdf.append(cylinder_link("bollard_w2", -5.20,  0.55, 0.40, 0.08, 0.80, 0.95, 0.78, 0.05, collide=False))
        sdf.append(cylinder_link("bollard_e1",  5.20, -0.55, 0.40, 0.08, 0.80, 0.95, 0.78, 0.05, collide=False))
        sdf.append(cylinder_link("bollard_e2",  5.20,  0.55, 0.40, 0.08, 0.80, 0.95, 0.78, 0.05, collide=False))
        sdf.append("    </model>\\n")

    # =========================================================================
    # 5. DYNAMIC LOGISTICS EQUIPMENT: PALLET JACKS (AWS RoboMaker)
    # =========================================================================
    pallet_jacks = [
        ("pallet_jack_dock_1",  10.50, -10.00,  0.0),
        ("pallet_jack_dock_2",  10.50, -14.00,  1.57),
        ("pallet_jack_pack_1",  10.50, -53.00,  0.0),
        ("pallet_jack_staging", 65.00, -29.00, -1.57),
        ("pallet_jack_corridor",65.00, -37.00, -1.57),
    ]
    for pj_name, px, py, pyaw in pallet_jacks:
        sdf.append(f"""
    <!-- Industrial Pallet Jack ({pj_name}) -->
    <include>
      <name>{pj_name}</name>
      <uri>model://aws_robomaker_warehouse_PalletJackB_01</uri>
      <pose>{px:.2f} {py:.2f} 0.0 0 0 {pyaw:.2f}</pose>
      <static>true</static>
    </include>
""")

    # =========================================================================
    # 6. DISPATCH & SUPERVISOR DESKS (AWS RoboMaker)
    # =========================================================================
    desks = [
        ("packing_workstation_1", 10.50, -45.00, 1.57),
        ("packing_workstation_2", 10.50, -49.00, 1.57),
        ("inbound_supervisor_desk", 10.50, -17.00, 1.57),
    ]
    for dname, dx, dy, dyaw in desks:
        sdf.append(f"""
    <!-- Workstation Desk ({dname}) -->
    <include>
      <name>{dname}</name>
      <uri>model://aws_robomaker_warehouse_DeskC_01</uri>
      <pose>{dx:.2f} {dy:.2f} 0.0 0 0 {dyaw:.2f}</pose>
      <static>true</static>
    </include>
""")

    # =========================================================================
    # 7. LOGISTICS FREIGHT CLUTTER & STAGING TOTES (AWS RoboMaker)
    # =========================================================================
    clutter_items = [
        ("inbound_freight_stack_1", "ClutteringA_01", 10.00, -15.50, 0.0),
        ("inbound_freight_totes",   "ClutteringC_01", 15.00, -10.00, 0.0),
        ("packing_shipping_crates", "ClutteringD_01", 10.00, -42.00, 0.0),
        ("storage_staging_pallets", "ClutteringA_01", 66.00, -43.00, 0.0),
        ("east_overflow_cartons",   "ClutteringD_01", 66.00, -22.00, 0.0),
    ]
    for cname, ctype, cx, cy, cyaw in clutter_items:
        sdf.append(f"""
    <!-- Logistics Clutter ({cname}) -->
    <include>
      <name>{cname}</name>
      <uri>model://aws_robomaker_warehouse_{ctype}</uri>
      <pose>{cx:.2f} {cy:.2f} 0.0 0 0 {cyaw:.2f}</pose>
      <static>true</static>
    </include>
""")

    # =========================================================================
    # 8. INDUSTRIAL SAFETY TRASH CANS (AWS RoboMaker)
    # =========================================================================
    trash_cans = [
        ("trash_can_dock",     9.00, -12.00),
        ("trash_can_packing",  9.00, -51.00),
        ("trash_can_central", 40.00, -31.00),
    ]
    for tcname, tcx, tcy in trash_cans:
        sdf.append(f"""
    <!-- Industrial Trash Can ({tcname}) -->
    <include>
      <name>{tcname}</name>
      <uri>model://aws_robomaker_warehouse_TrashCanC_01</uri>
      <pose>{tcx:.2f} {tcy:.2f} 0.0 0 0 0</pose>
      <static>true</static>
    </include>
""")

    # =========================================================================
    # 9. CHARGING TERMINALS (X = 11.80)
    # =========================================================================
    charging_terminals = [
        ("charging_bay_alpha",   11.80, -28.50, "Alpha (Edge-AI TinyRobot1)", 0.12, 0.45, 0.95),
        ("charging_bay_beta",    11.80, -30.50, "Beta (Traditional TinyRobot2)", 0.95, 0.15, 0.15),
        ("charging_bay_gamma",   11.80, -32.50, "Gamma", 0.15, 0.85, 0.25),
        ("charging_bay_delta",   11.80, -34.50, "Delta", 0.95, 0.85, 0.10),
        ("charging_bay_epsilon", 11.80, -36.50, "Epsilon", 0.75, 0.18, 0.92),
    ]
    for bay_name, bx, by, blabel, cr, cg, cb in charging_terminals:
        sdf.append(f"""
    <!-- Charging Station {blabel} -->
    <model name="{bay_name}">
      <static>true</static>
      <pose>{bx:.2f} {by:.2f} 0.0 0 0 0</pose>
      <link name="terminal_housing">
        <pose>0 0 0.65 0 0 0</pose>
        <visual name="pylon_vis">
          <geometry><box><size>0.35 0.50 1.3</size></box></geometry>
          <material><ambient>0.22 0.25 0.28 1.0</ambient><diffuse>0.30 0.34 0.38 1.0</diffuse></material>
        </visual>
      </link>
      <link name="terminal_cap">
        <pose>0 0 1.33 0 0 0</pose>
        <visual name="cap_vis">
          <geometry><box><size>0.38 0.53 0.08</size></box></geometry>
          <material><ambient>{cr*0.8:.2f} {cg*0.8:.2f} {cb*0.8:.2f} 1.0</ambient><diffuse>{cr:.2f} {cg:.2f} {cb:.2f} 1.0</diffuse></material>
        </visual>
      </link>
      <link name="charging_beacon">
        <pose>0.15 0 1.10 0 0 0</pose>
        <visual name="beacon_vis">
          <geometry><cylinder><radius>0.06</radius><length>0.12</length></cylinder></geometry>
          <material>
            <ambient>{cr:.2f} {cg:.2f} {cb:.2f} 1.0</ambient>
            <diffuse>{cr:.2f} {cg:.2f} {cb:.2f} 1.0</diffuse>
            <emissive>{cr:.2f} {cg:.2f} {cb:.2f} 1.0</emissive>
          </material>
        </visual>
      </link>
    </model>
""")

    # =========================================================================
    # 10. INDUSTRIAL SIGNAGE (High Visibility Overhead Signs at Z = 5.0m)
    # =========================================================================
    sign_posts = [
        ("sign_zone_a", 16.0,  -9.0, 5.0, "ZONE A: INBOUND RECEIVING", 0.15, 0.45, 0.85),
        ("sign_zone_b", 66.0, -28.0, 5.0, "ZONE B: BULK STORAGE",     0.55, 0.20, 0.75),
        ("sign_zone_c", 16.0, -53.0, 5.0, "ZONE C: PACKING & SHIP",   0.20, 0.65, 0.35),
        ("sign_zone_d", 11.0, -25.0, 5.0, "ZONE D: AMR FLEET DEPOT",  0.85, 0.55, 0.10),
    ]
    for sname, sx, sy, sz, stitle, sr, sg, sb in sign_posts:
        sdf.append(f"""
    <!-- Signboard {stitle} -->
    <model name="{sname}">
      <static>true</static>
      <pose>{sx:.2f} {sy:.2f} {sz:.2f} 0 0 0</pose>
      <link name="sign_link">
        <visual name="panel_vis">
          <geometry><box><size>3.6 0.10 0.90</size></box></geometry>
          <material><ambient>0.15 0.18 0.22 1.0</ambient><diffuse>0.20 0.24 0.28 1.0</diffuse></material>
        </visual>
        <visual name="banner_vis">
          <pose>0 -0.052 0 0 0 0</pose>
          <geometry><box><size>3.4 0.01 0.75</size></box></geometry>
          <material>
            <ambient>{sr*0.8:.2f} {sg*0.8:.2f} {sb*0.8:.2f} 1.0</ambient>
            <diffuse>{sr:.2f} {sg:.2f} {sb:.2f} 1.0</diffuse>
            <emissive>{sr*0.5:.2f} {sg*0.5:.2f} {sb*0.5:.2f} 1.0</emissive>
          </material>
        </visual>
      </link>
    </model>
""")

    # =========================================================================
    # 11. OPEN-RMF TINYROBOT FLEET
    # =========================================================================
    sdf.append("""
    <!-- Open-RMF TinyRobot Fleet: 5 AMRs across all 5 charging bays -->
    <include>
      <name>tinyRobot1</name>
      <uri>model://TinyRobot</uri>
      <pose>13.50 -28.50 0.0 0 0 0</pose>
    </include>
    <include>
      <name>tinyRobot2</name>
      <uri>model://TinyRobot</uri>
      <pose>13.50 -30.50 0.0 0 0 0</pose>
    </include>
    <include>
      <name>tinyRobot3</name>
      <uri>model://TinyRobot</uri>
      <pose>13.50 -32.50 0.0 0 0 0</pose>
    </include>
    <include>
      <name>tinyRobot4</name>
      <uri>model://TinyRobot</uri>
      <pose>13.50 -34.50 0.0 0 0 0</pose>
    </include>
    <include>
      <name>tinyRobot5</name>
      <uri>model://TinyRobot</uri>
      <pose>13.50 -36.50 0.0 0 0 0</pose>
    </include>

    <!-- Open-RMF Charger Component Registry Plugin -->
    <plugin name="register_component" filename="libregister_component.so">
      <component name="Chargers">
        <rmf_charger name="tinyRobot1_charger" x="13.50" y="-28.50" z="0.0" />
        <rmf_charger name="tinyRobot2_charger" x="13.50" y="-30.50" z="0.0" />
        <rmf_charger name="charger_gamma" x="13.50" y="-32.50" z="0.0" />
        <rmf_charger name="charger_delta" x="13.50" y="-34.50" z="0.0" />
        <rmf_charger name="charger_epsilon" x="13.50" y="-36.50" z="0.0" />
      </component>
    </plugin>
  </world>
</sdf>
""")

    os.makedirs(os.path.dirname(os.path.abspath(output_path)), exist_ok=True)
    with open(output_path, "w") as f:
        f.write("".join(sdf))
    print(f"[generate_warehouse_world] Successfully generated 4x expanded warehouse world at: {output_path}")

if __name__ == "__main__":
    target = sys.argv[1] if len(sys.argv) > 1 else "/home/manoj/SIH/rmf_ws/src/demonstrations/rmf_demos/rmf_demos_maps/maps/warehouse/warehouse.world"
    generate_expanded_warehouse_world(target)
