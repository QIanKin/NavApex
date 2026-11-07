#!/usr/bin/env python3
"""Generate the known grid and a simulation-only robot SDF. No SLAM is involved."""
import argparse
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET


def compose(parent, element):
    pose = [float(v) for v in element.findtext('pose', '0 0 0 0 0 0').split()]
    x, y, yaw = parent
    c, s = math.cos(yaw), math.sin(yaw)
    return x + c * pose[0] - s * pose[1], y + s * pose[0] + c * pose[1], yaw + pose[5]


def generate(world, output, config):
    # Bounds match the supplied world; box footprints are rasterized in world XY.
    resolution = config["grid"]["resolution"]
    width, height = round(10 / resolution), round(8 / resolution)
    origin_x, origin_y = -4.0, -4.0
    boxes = []
    for model in ET.parse(world).findall('./world/model'):
        if model.findtext('static', 'false') != 'true':
            continue
        model_pose = compose((0, 0, 0), model)
        for link in model.findall('link'):
            link_pose = compose(model_pose, link)
            for collision in link.findall('collision'):
                size = collision.find('geometry/box/size')
                if size is None:  # The world's ground plane is traversable.
                    continue
                sx, sy, _ = [float(v) for v in size.text.split()]
                boxes.append((*compose(link_pose, collision), sx / 2, sy / 2))
    costs = []
    for row in range(height):
        y = origin_y + (row + 0.5) * resolution
        for col in range(width):
            x = origin_x + (col + 0.5) * resolution
            occupied = False
            for cx, cy, yaw, hx, hy in boxes:
                dx, dy = x - cx, y - cy
                u = math.cos(yaw) * dx + math.sin(yaw) * dy
                v = -math.sin(yaw) * dx + math.cos(yaw) * dy
                if abs(u) <= hx and abs(v) <= hy:
                    occupied = True
                    break
            costs.append(254 if occupied else 0)
    # Match the lidar grid's distance convention. At radius=resolution this
    # marks exactly the eight neighboring cells, without cascading dilation.
    forbidden_radius = config["inflation"]["forbidden_radius"]
    cell_padding = math.sqrt(0.5) * resolution
    extent = math.ceil((forbidden_radius + cell_padding) / resolution)
    inflated = costs.copy()
    if forbidden_radius > 0:
        offsets = [(dx, dy) for dy in range(-extent, extent + 1)
                   for dx in range(-extent, extent + 1)
                   if math.hypot(dx, dy) * resolution - cell_padding <= forbidden_radius]
        for row in range(height):
            for col in range(width):
                if costs[row * width + col] != 254:
                    continue
                for dx, dy in offsets:
                    x, y = col + dx, row + dy
                    if 0 <= x < width and 0 <= y < height and costs[y * width + x] == 0:
                        inflated[y * width + x] = 253
    destination = Path(output)
    destination.parent.mkdir(parents=True, exist_ok=True)
    destination.write_text(json.dumps(dict(resolution=resolution, width=width, height=height,
                                          origin_x=origin_x, origin_y=origin_y, costs=inflated)))


def prepare_robot(source, output):
    tree = ET.parse(source)
    model = tree.find('model')
    drive = model.find("plugin[@filename='libgazebo_ros_diff_drive.so']")
    # WORLD=1 uses Gazebo truth, rather than wheel-integrated localization.
    ET.SubElement(drive, 'odometry_source').text = '1'
    drive.find('update_rate').text = '200'
    laser = model.find(".//sensor[@type='ray']")
    laser.find('update_rate').text = '20'
    laser.find('ray/range/max').text = '8.0'
    laser.find('ray/noise/stddev').text = '0'
    tree.write(output, encoding='unicode')


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--world', required=True)
    parser.add_argument('--output', required=True)
    parser.add_argument('--perception-config', required=True)
    parser.add_argument('--robot')
    parser.add_argument('--robot-output')
    args = parser.parse_args()
    config = json.loads(Path(args.perception_config).read_text())
    generate(args.world, args.output, config)
    if args.robot:
        prepare_robot(args.robot, args.robot_output)
