#!/usr/bin/env python3
"""Rasterize the simulation world's static box collisions; no perception dependency."""
import argparse
import math
from pathlib import Path
import xml.etree.ElementTree as ET


def generate(world, output):
    resolution, width, height = 0.05, 200, 160
    origin_x, origin_y = -4.0, -4.0
    boxes = []
    for model in ET.parse(world).findall('./world/model'):
        if model.findtext('static', 'false') != 'true':
            continue
        pose = [float(v) for v in model.findtext('pose', '0 0 0 0 0 0').split()]
        for collision in model.findall('./link/collision'):
            box = collision.find('geometry/box/size')
            if box is None:
                continue
            local = [float(v) for v in collision.findtext('pose', '0 0 0 0 0 0').split()]
            sx, sy, _ = [float(v) for v in box.text.split()]
            boxes.append((pose[0] + local[0], pose[1] + local[1], pose[5] + local[5], sx / 2, sy / 2))
    pixels = bytearray()
    for row in range(height):
        y = origin_y + (height - row - 0.5) * resolution
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
            pixels.append(0 if occupied else 254)
    destination = Path(output)
    destination.parent.mkdir(parents=True, exist_ok=True)
    destination.write_bytes(f'P5\n{width} {height}\n255\n'.encode() + pixels)


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--world', required=True)
    parser.add_argument('--output', required=True)
    args = parser.parse_args()
    generate(args.world, args.output)
