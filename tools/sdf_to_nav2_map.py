#!/usr/bin/env python3
import argparse
import math
import os
import xml.etree.ElementTree as ET
from PIL import Image, ImageDraw


def parse_pose(pose_elem):
    """Parses SDF <pose> tag string 'x y z roll pitch yaw' into floats."""
    if pose_elem is None or not pose_elem.text:
        return 0.0, 0.0, 0.0, 0.0, 0.0, 0.0
    vals = [float(v) for v in pose_elem.text.strip().split()]
    while len(vals) < 6:
        vals.append(0.0)
    return tuple(vals[:6])


def combine_poses(p1, p2):
    """Combines two poses hierarchically (2D translation/rotation + Z addition)."""
    x1, y1, z1, r1, p1_pitch, yaw1 = p1
    x2, y2, z2, r2, p2_pitch, yaw2 = p2

    x_comb = x1 + x2 * math.cos(yaw1) - y2 * math.sin(yaw1)
    y_comb = y1 + x2 * math.sin(yaw1) + y2 * math.cos(yaw1)
    z_comb = z1 + z2
    yaw_comb = yaw1 + yaw2

    return (x_comb, y_comb, z_comb, r1 + r2, p1_pitch + p2_pitch, yaw_comb)


def extract_boxes(sdf_path, min_z=0.2, max_z=0.5):
    """
    Extracts 2D box collision geometries and their world transforms from SDF,
    filtering for objects that intersect the vertical height window [min_z, max_z].
    """
    tree = ET.parse(sdf_path)
    root = tree.getroot()
    boxes = []

    world = root.find("world")
    search_root = world if world is not None else root

    for model in search_root.findall("model"):
        model_pose = parse_pose(model.find("pose"))

        for link in model.findall("link"):
            link_pose = combine_poses(model_pose, parse_pose(link.find("pose")))

            # Check collision tags first; fall back to visual tags if no collision defined
            targets = link.findall("collision") or link.findall("visual")
            for elem in targets:
                elem_pose = combine_poses(link_pose, parse_pose(elem.find("pose")))
                geom = elem.find("geometry")
                if geom is None:
                    continue

                box = geom.find("box")
                if box is not None and box.find("size") is not None:
                    size_vals = [float(v) for v in box.find("size").text.strip().split()]
                    sx, sy = size_vals[0], size_vals[1]
                    sz = size_vals[2] if len(size_vals) > 2 else 0.0

                    x, y, z, _, _, yaw = elem_pose

                    # Calculate vertical range of the box
                    box_z_min = z - (sz / 2.0)
                    box_z_max = z + (sz / 2.0)

                    # Check if box overlaps with the LiDAR height slice [min_z, max_z]
                    if box_z_max >= min_z and box_z_min <= max_z:
                        boxes.append({
                            "x": x,
                            "y": y,
                            "sx": sx,
                            "sy": sy,
                            "yaw": yaw
                        })

    return boxes


def generate_map(boxes, resolution=0.05, padding=1.0, output_prefix="map"):
    """Renders 2D Occupancy Grid map (.pgm) and ROS map server metadata (.yaml)."""
    if not boxes:
        raise ValueError("No box geometries found in the specified height range.")

    # Compute bounding box for full map area
    all_x, all_y = [], []
    for b in boxes:
        hx, hy = b["sx"] / 2.0, b["sy"] / 2.0
        corners_local = [(hx, hy), (-hx, hy), (-hx, -hy), (hx, -hy)]
        cos_y, sin_y = math.cos(b["yaw"]), math.sin(b["yaw"])
        for cx, cy in corners_local:
            wx = b["x"] + cx * cos_y - cy * sin_y
            wy = b["y"] + cx * sin_y + cy * cos_y
            all_x.append(wx)
            all_y.append(wy)

    min_x = math.floor(min(all_x) - padding)
    max_x = math.ceil(max(all_x) + padding)
    min_y = math.floor(min(all_y) - padding)
    max_y = math.ceil(max(all_y) + padding)

    width_m = max_x - min_x
    height_m = max_y - min_y

    img_w = int(round(width_m / resolution))
    img_h = int(round(height_m / resolution))

    # Create grayscale canvas: 255 = Free space (white)
    img = Image.new("L", (img_w, img_h), 255)
    draw = ImageDraw.Draw(img)

    def world_to_pixel(wx, wy):
        px = (wx - min_x) / resolution
        py = (max_y - wy) / resolution  # Invert Y for image coordinate system
        return px, py

    # Draw occupied spaces: 0 = Obstacle (black)
    for b in boxes:
        hx, hy = b["sx"] / 2.0, b["sy"] / 2.0
        corners_local = [(hx, hy), (-hx, hy), (-hx, -hy), (hx, -hy)]
        cos_y, sin_y = math.cos(b["yaw"]), math.sin(b["yaw"])

        poly_pixels = []
        for cx, cy in corners_local:
            wx = b["x"] + cx * cos_y - cy * sin_y
            wy = b["y"] + cx * sin_y + cy * cos_y
            poly_pixels.append(world_to_pixel(wx, wy))

        draw.polygon(poly_pixels, fill=0)

    pgm_filename = f"{output_prefix}.pgm"
    yaml_filename = f"{output_prefix}.yaml"

    # Save PGM image
    img.save(pgm_filename)

    # Save ROS/ROS2 Nav2 Map Server YAML file
    yaml_content = f"""image: {os.path.basename(pgm_filename)}
mode: trinary
resolution: {resolution}
origin: [{min_x:.2f}, {min_y:.2f}, 0.0]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.196
"""
    with open(yaml_filename, "w") as f:
        f.write(yaml_content)

    print(f"Map successfully generated:")
    print(f"  - Image file:    {pgm_filename} ({img_w}x{img_h} px)")
    print(f"  - Metadata file: {yaml_filename}")
    print(f"  - World Origin:  [{min_x:.2f}, {min_y:.2f}, 0.0]")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Generate ROS 2D Occupancy Grid Map from Gazebo SDF World")
    parser.add_argument("-i", "--input", default="warehouse_world.sdf", help="Path to input .sdf file")
    parser.add_argument("-r", "--resolution", type=float, default=0.05, help="Resolution in meters/pixel (default: 0.05)")
    parser.add_argument("-p", "--padding", type=float, default=1.0, help="Padding around world boundary in meters (default: 1.0)")
    parser.add_argument("-o", "--output", default="warehouse_map", help="Output file prefix (default: warehouse_map)")
    parser.add_argument("--min-z", type=float, default=0.2, help="Minimum obstacle height slice in meters (default: 0.2)")
    parser.add_argument("--max-z", type=float, default=0.5, help="Maximum obstacle height slice in meters (default: 0.5)")

    args = parser.parse_args()

    boxes = extract_boxes(
        args.input,
        min_z=args.min_z,
        max_z=args.max_z
    )
    generate_map(
        boxes,
        resolution=args.resolution,
        padding=args.padding,
        output_prefix=args.output
    )