#!/usr/bin/env python3
"""
Procedural 3D Subsea Pipeline and Scenario Generator for POSIM Gazebo.

Generates:
1. Procedural 3D pipeline SDF model (connected cylindrical segments, flanges, and concrete saddles).
2. Randomized obstacle placements along the pipeline corridor.
3. Synchronized YAML inspection mission waypoints matching the pipe centerline at inspection standoff altitude.
"""

import argparse
import math
import os
import random
import sys
from typing import Dict, List, Tuple

DEFAULT_NUM_SEGMENTS = 4
DEFAULT_NUM_OBSTACLES = 1
DEFAULT_PIPE_RADIUS_M = 0.25
DEFAULT_PIPE_START_M = (2.0, 0.0, -2.8)
DEFAULT_MIN_SEGMENT_LENGTH_M = 4.0
DEFAULT_MAX_SEGMENT_LENGTH_M = 7.0
DEFAULT_MAX_BEND_DEG = 25.0
DEFAULT_STANDOFF_HEIGHT_M = 0.8
DEFAULT_ACCEPTANCE_RADIUS_M = 0.40
DEFAULT_WAYPOINT_SPACING_M = 2.0
DEFAULT_MISSION_SPEED_MPS = 0.5
DEFAULT_WAYPOINT_TIMEOUT_SEC = 45.0
ENDPOINT_DWELL_SEC = 2.0


def generate_pipeline_geometry(
    num_segments: int = DEFAULT_NUM_SEGMENTS,
    min_length: float = DEFAULT_MIN_SEGMENT_LENGTH_M,
    max_length: float = DEFAULT_MAX_SEGMENT_LENGTH_M,
    max_bend_deg: float = DEFAULT_MAX_BEND_DEG,
    start_pos: Tuple[float, float, float] = DEFAULT_PIPE_START_M,
    rng: random.Random = None
) -> List[Tuple[float, float, float]]:
    """
    Computes 3D vertices for a continuous subsea pipeline with randomized bends and gentle bathymetry slopes.

    :param num_segments: Number of linear pipe segments.
    :param min_length: Minimum segment length in meters.
    :param max_length: Maximum segment length in meters.
    :param max_bend_deg: Maximum horizontal bend angle between segments in degrees.
    :param start_pos: Initial pipeline start coordinate (X, Y, Z) in meters.
    :param rng: Seeded random.Random instance.
    :return: List of 3D vertex tuples [(x0, y0, z0), (x1, y1, z1), ...].
    """
    if rng is None:
        rng = random.Random()

    vertices = [start_pos]
    current_x, current_y, current_z = start_pos
    current_heading = 0.0  # Initial direction straight along +X

    max_bend_rad = math.radians(max_bend_deg)

    for i in range(num_segments):
        # Apply slight heading bend for segments after the first
        if i > 0:
            bend = rng.uniform(-max_bend_rad, max_bend_rad)
            current_heading += bend

        length = rng.uniform(min_length, max_length)
        # Gentle bathymetric slope between -2.5m and -3.2m
        delta_z = rng.uniform(-0.15, 0.05)
        new_z = max(-3.2, min(-2.5, current_z + delta_z))

        current_x += length * math.cos(current_heading)
        current_y += length * math.sin(current_heading)
        current_z = new_z

        vertices.append((round(current_x, 3), round(current_y, 3), round(current_z, 3)))

    return vertices


def generate_random_obstacles(
    vertices: List[Tuple[float, float, float]],
    num_obstacles: int = DEFAULT_NUM_OBSTACLES,
    pipe_radius: float = DEFAULT_PIPE_RADIUS_M,
    rng: random.Random = None
) -> List[Dict[str, float]]:
    """
    Places obstacles randomly along the pipeline corridor.

    :param vertices: 3D pipeline vertices.
    :param num_obstacles: Number of obstacles to generate (0 to 3).
    :param pipe_radius: Pipe outer radius in meters.
    :param rng: Seeded random.Random instance.
    :return: List of obstacle dictionaries with coordinates and dimensions.
    """
    if rng is None:
        rng = random.Random()

    if num_obstacles <= 0 or len(vertices) < 2:
        return []

    num_segments = len(vertices) - 1
    # Distribute obstacles across intermediate segments (avoiding very start or very end)
    eligible_segments = list(range(num_segments))
    chosen_segments = rng.sample(eligible_segments, min(num_obstacles, len(eligible_segments)))

    obstacles = []
    for idx, seg_idx in enumerate(chosen_segments):
        p1 = vertices[seg_idx]
        p2 = vertices[seg_idx + 1]

        # Fraction along the segment
        t = rng.uniform(0.3, 0.7)
        obs_x = p1[0] + t * (p2[0] - p1[0])
        obs_y = p1[1] + t * (p2[1] - p1[1])
        base_z = p1[2] + t * (p2[2] - p1[2])

        # Randomize obstacle type/dimensions: cinder block or shipping debris crate
        size_x = round(rng.uniform(0.45, 0.60), 2)
        size_y = round(rng.uniform(0.45, 0.60), 2)
        size_z = round(rng.uniform(0.45, 0.65), 2)

        # Place directly resting on top of the pipe crown
        obs_z = base_z + pipe_radius + (size_z / 2.0)

        # Small lateral perturbation (on pipe crown or slightly resting on side)
        side_offset = rng.uniform(-0.15, 0.15)
        heading = math.atan2(p2[1] - p1[1], p2[0] - p1[0])
        obs_x += -math.sin(heading) * side_offset
        obs_y += math.cos(heading) * side_offset

        obstacles.append({
            "name": f"pipeline_obstacle_{idx + 1}",
            "x": round(obs_x, 3),
            "y": round(obs_y, 3),
            "z": round(obs_z, 3),
            "size_x": size_x,
            "size_y": size_y,
            "size_z": size_z,
        })

    return obstacles


def generate_pipeline_sdf(
    vertices: List[Tuple[float, float, float]],
    pipe_radius: float = DEFAULT_PIPE_RADIUS_M,
    obstacles: List[Dict[str, float]] = None
) -> str:
    """
    Generates Gazebo Simulation Description Format (SDF v1.8) string for the 3D pipeline model.

    :param vertices: Ordered list of 3D pipeline vertices.
    :param pipe_radius: Pipe outer radius in meters.
    :param obstacles: Optional list of obstacles to embed in the model.
    :return: Formatted SDF XML string.
    """
    sdf_lines = [
        '<?xml version="1.0"?>',
        '<sdf version="1.8">',
        '  <model name="subsea_pipeline">',
        '    <static>true</static>',
    ]

    # 1. Pipe Segments (Cylinders)
    for i in range(len(vertices) - 1):
        p1 = vertices[i]
        p2 = vertices[i + 1]

        dx = p2[0] - p1[0]
        dy = p2[1] - p1[1]
        dz = p2[2] - p1[2]

        length = math.sqrt(dx * dx + dy * dy + dz * dz)
        mid_x = (p1[0] + p2[0]) / 2.0
        mid_y = (p1[1] + p2[1]) / 2.0
        mid_z = (p1[2] + p2[2]) / 2.0

        # Euler angles to align local +Z cylinder with 3D segment vector
        d_xy = math.sqrt(dx * dx + dy * dy)
        pitch = math.atan2(d_xy, dz)
        yaw = math.atan2(dy, dx)
        roll = 0.0

        sdf_lines.extend([
            f'    <!-- Pipeline Segment {i + 1} -->',
            f'    <link name="segment_{i + 1}">',
            f'      <pose>{mid_x:.3f} {mid_y:.3f} {mid_z:.3f} {roll:.4f} {pitch:.4f} {yaw:.4f}</pose>',
            '      <collision name="col">',
            '        <geometry>',
            f'          <cylinder><radius>{pipe_radius:.3f}</radius><length>{length:.3f}</length></cylinder>',
            '        </geometry>',
            '      </collision>',
            '      <visual name="vis">',
            '        <geometry>',
            f'          <cylinder><radius>{pipe_radius:.3f}</radius><length>{length:.3f}</length></cylinder>',
            '        </geometry>',
            '        <material>',
            '          <ambient>0.85 0.65 0.10 1.0</ambient>',
            '          <diffuse>0.85 0.65 0.10 1.0</diffuse>',
            '          <specular>0.3 0.3 0.3 1.0</specular>',
            '        </material>',
            '      </visual>',
            '    </link>',
        ])

    # 2. Joint Flange Collars and Concrete Support Saddles at each vertex
    for i, pt in enumerate(vertices):
        # Joint Flange
        sdf_lines.extend([
            f'    <!-- Joint Flange {i + 1} -->',
            f'    <link name="flange_{i + 1}">',
            f'      <pose>{pt[0]:.3f} {pt[1]:.3f} {pt[2]:.3f} 0 0 0</pose>',
            '      <collision name="col">',
            '        <geometry>',
            f'          <sphere><radius>{pipe_radius * 1.15:.3f}</radius></sphere>',
            '        </geometry>',
            '      </collision>',
            '      <visual name="vis">',
            '        <geometry>',
            f'          <sphere><radius>{pipe_radius * 1.15:.3f}</radius></sphere>',
            '        </geometry>',
            '        <material>',
            '          <ambient>0.25 0.28 0.32 1.0</ambient>',
            '          <diffuse>0.30 0.35 0.40 1.0</diffuse>',
            '        </material>',
            '      </visual>',
            '    </link>',
            f'    <!-- Concrete Support Saddle {i + 1} -->',
            f'    <link name="support_{i + 1}">',
            f'      <pose>{pt[0]:.3f} {pt[1]:.3f} {pt[2] - pipe_radius - 0.10:.3f} 0 0 0</pose>',
            '      <collision name="col">',
            '        <geometry><box><size>0.6 0.6 0.2</size></box></geometry>',
            '      </collision>',
            '      <visual name="vis">',
            '        <geometry><box><size>0.6 0.6 0.2</size></box></geometry>',
            '        <material>',
            '          <ambient>0.45 0.45 0.45 1.0</ambient>',
            '          <diffuse>0.50 0.50 0.50 1.0</diffuse>',
            '        </material>',
            '      </visual>',
            '    </link>',
        ])

    # 3. Embedded Obstacles (if any)
    if obstacles:
        for obs in obstacles:
            sdf_lines.extend([
                f'    <!-- Obstacle: {obs["name"]} -->',
                f'    <link name="{obs["name"]}">',
                f'      <pose>{obs["x"]:.3f} {obs["y"]:.3f} {obs["z"]:.3f} 0 0 0</pose>',
                '      <collision name="col">',
                f'        <geometry><box><size>{obs["size_x"]} {obs["size_y"]} {obs["size_z"]}</size></box></geometry>',
                '      </collision>',
                '      <visual name="vis">',
                f'        <geometry><box><size>{obs["size_x"]} {obs["size_y"]} {obs["size_z"]}</size></box></geometry>',
                '        <material>',
                '          <ambient>0.90 0.15 0.15 1.0</ambient>',
                '          <diffuse>0.90 0.15 0.15 1.0</diffuse>',
                '        </material>',
                '      </visual>',
                '    </link>',
            ])

    sdf_lines.extend(['  </model>', '</sdf>', ''])
    return '\n'.join(sdf_lines)


def generate_mission_yaml(
    vertices: List[Tuple[float, float, float]],
    standoff_height: float = DEFAULT_STANDOFF_HEIGHT_M,
    acceptance_radius: float = DEFAULT_ACCEPTANCE_RADIUS_M,
    waypoint_spacing: float = DEFAULT_WAYPOINT_SPACING_M,
    seed_used: int = None
) -> str:
    """
    Generates mission YAML string with dense waypoints tracking directly above the pipeline centerline.

    :param vertices: 3D pipeline vertices (segment endpoints).
    :param standoff_height: Elevation above the pipe crown for inspection transit (meters).
    :param acceptance_radius: Arrival radius tolerance (meters).
    :param waypoint_spacing: Distance between consecutive tracking waypoints along the pipe (meters).
    :param seed_used: Optional seed integer logged into mission header.
    :return: Formatted YAML string.
    """
    # Sample dense tracking points along each segment
    dense_waypoints = []
    for i in range(len(vertices) - 1):
        p1 = vertices[i]
        p2 = vertices[i + 1]
        dx = p2[0] - p1[0]
        dy = p2[1] - p1[1]
        dz = p2[2] - p1[2]
        seg_len = math.sqrt(dx * dx + dy * dy + dz * dz)
        num_subdivisions = max(1, round(seg_len / waypoint_spacing))

        for step in range(num_subdivisions):
            t = step / float(num_subdivisions)
            wx = p1[0] + t * dx
            wy = p1[1] + t * dy
            wz = p1[2] + t * dz + standoff_height
            dense_waypoints.append((wx, wy, wz))

    # Add final terminus
    p_last = vertices[-1]
    dense_waypoints.append((p_last[0], p_last[1], p_last[2] + standoff_height))

    yaml_lines = [
        'mission:',
        f'  name: "procedural_pipeline_inspection_{seed_used if seed_used is not None else "rand"}"',
        f'  default_acceptance_radius: {acceptance_radius:.2f}',
        f'  default_speed: {DEFAULT_MISSION_SPEED_MPS}',
        f'  timeout_per_waypoint_sec: {DEFAULT_WAYPOINT_TIMEOUT_SEC}',
        '  waypoints:',
    ]

    for idx, (wx, wy, wz) in enumerate(dense_waypoints):
        wp_name = "pipe_entry" if idx == 0 else ("survey_terminus" if idx == len(dense_waypoints) - 1 else f"track_{idx}")
        dwell = ENDPOINT_DWELL_SEC if (idx == 0 or idx == len(dense_waypoints) - 1) else 0.0

        yaml_lines.extend([
            f'    - name: "{wp_name}"',
            f'      x: {wx:.2f}',
            f'      y: {wy:.2f}',
            f'      z: {wz:.2f}',
            f'      dwell_sec: {dwell:.1f}',
            '',
        ])

    return '\n'.join(yaml_lines)


def main():
    parser = argparse.ArgumentParser(description="Procedural Subsea Pipeline Scenario Generator")
    parser.add_argument("--seed", type=int, default=None, help="Random seed for reproducibility (omit for true random)")
    parser.add_argument("--num-segments", type=int, default=DEFAULT_NUM_SEGMENTS, help="Number of pipe segments (3-6)")
    parser.add_argument("--num-obstacles", type=int, default=DEFAULT_NUM_OBSTACLES, help="Number of obstacles on pipeline path (0-3)")
    parser.add_argument("--standoff-height", type=float, default=DEFAULT_STANDOFF_HEIGHT_M, help="Inspection standoff altitude above pipe (m)")
    parser.add_argument("--acceptance-radius", type=float, default=DEFAULT_ACCEPTANCE_RADIUS_M, help="Arrival tolerance radius (m)")
    parser.add_argument("--waypoint-spacing", type=float, default=DEFAULT_WAYPOINT_SPACING_M, help="Spacing between intermediate tracking waypoints (m)")
    parser.add_argument("--output-sdf", type=str, default=None, help="Path to write pipeline model.sdf")
    parser.add_argument("--output-yaml", type=str, default=None, help="Path to write pipeline_mission.yaml")
    args = parser.parse_args()

    # Determine seed
    if args.seed is None:
        seed = random.SystemRandom().randint(1, 999999)
    else:
        seed = args.seed

    rng = random.Random(seed)
    print(f"[Scenario Generator] Initializing procedural pipeline (Seed: {seed})")

    # Resolve paths: prefer writing to source directory and syncing to install share
    try:
        from ament_index_python.packages import get_package_share_directory
        share_dir = get_package_share_directory("subsea_inspection")
    except Exception:
        share_dir = None

    src_dir = os.path.expanduser("~/posim_ws/src/subsea_inspection")
    if not os.path.exists(src_dir):
        src_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

    output_sdf = args.output_sdf or os.path.join(src_dir, "models", "pipeline", "model.sdf")
    output_yaml = args.output_yaml or os.path.join(src_dir, "config", "pipeline_mission.yaml")

    # Generate geometry and obstacles
    vertices = generate_pipeline_geometry(
        num_segments=args.num_segments,
        start_pos=DEFAULT_PIPE_START_M,
        rng=rng
    )

    obstacles = generate_random_obstacles(
        vertices=vertices,
        num_obstacles=args.num_obstacles,
        pipe_radius=DEFAULT_PIPE_RADIUS_M,
        rng=rng
    )

    # Generate SDF and YAML contents
    sdf_content = generate_pipeline_sdf(vertices, pipe_radius=DEFAULT_PIPE_RADIUS_M, obstacles=obstacles)
    yaml_content = generate_mission_yaml(
        vertices,
        standoff_height=args.standoff_height,
        acceptance_radius=args.acceptance_radius,
        waypoint_spacing=args.waypoint_spacing,
        seed_used=seed
    )

    # Write files to target paths
    target_sdfs = [output_sdf]
    target_yamls = [output_yaml]

    if args.output_sdf is None and share_dir and os.path.exists(share_dir):
        target_sdfs.append(os.path.join(share_dir, "models", "pipeline", "model.sdf"))

    if args.output_yaml is None and share_dir and os.path.exists(share_dir):
        target_yamls.append(os.path.join(share_dir, "config", "pipeline_mission.yaml"))

    for p in target_sdfs:
        os.makedirs(os.path.dirname(p), exist_ok=True)
        with open(p, "w") as f:
            f.write(sdf_content)
        print(f"[Scenario Generator] Wrote 3D pipeline SDF ({len(vertices) - 1} segments) -> {p}")

    num_waypoints = yaml_content.count('- name:')
    for p in target_yamls:
        os.makedirs(os.path.dirname(p), exist_ok=True)
        with open(p, "w") as f:
            f.write(yaml_content)
        print(f"[Scenario Generator] Wrote mission YAML ({num_waypoints} waypoints) -> {p}")

    if obstacles:
        print(f"[Scenario Generator] Placed {len(obstacles)} randomized obstacle(s):")
        for obs in obstacles:
            print(f"  - {obs['name']}: Pos [{obs['x']}, {obs['y']}, {obs['z']}] Size [{obs['size_x']}x{obs['size_y']}x{obs['size_z']}]m")


if __name__ == "__main__":
    main()
