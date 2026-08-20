import math
import random

random.seed(42)
points = []

# Ground plane: y = 0, x in [-5, 5], z in [0, 10]
step = 0.3
x = -5.0
while x <= 5.0:
    z = 0.0
    while z <= 10.0:
        noise = random.uniform(-0.02, 0.02)
        points.append((x, noise, z))
        z += step
    x += step

# Box obstacle: center at (2, 0.5, 5), size 1x1x1
box_cx, box_cy, box_cz = 2.0, 0.5, 5.0
bw, bh, bd = 0.5, 0.5, 0.5
for face in ["front", "back", "top", "left", "right"]:
    u = -bw
    while u <= bw:
        v = -bh
        while v <= bh:
            if face == "front":
                points.append((box_cx + u, box_cy + v, box_cz + bd))
            elif face == "back":
                points.append((box_cx + u, box_cy + v, box_cz - bd))
            elif face == "top":
                points.append((box_cx + u, box_cy + bh, box_cz + v))
            elif face == "left":
                points.append((box_cx - bw, box_cy + v, box_cz + u))
            elif face == "right":
                points.append((box_cx + bw, box_cy + v, box_cz + u))
            v += 0.1
        u += 0.1

# Cylinder pillar: center at (-1.5, 0, 7), radius 0.3, height 1.5
cx_p, cz_p, r_p, h_p = -1.5, 7.0, 0.3, 1.5
theta = 0.0
while theta < 2 * math.pi:
    y = 0.0
    while y <= h_p:
        px = cx_p + r_p * math.cos(theta)
        pz = cz_p + r_p * math.sin(theta)
        points.append((px, y, pz))
        y += 0.1
    theta += 0.15

lines = [
    "VERSION 0.7",
    "FIELDS x y z",
    "SIZE 4 4 4",
    "TYPE F F F",
    "COUNT 1 1 1",
    f"WIDTH {len(points)}",
    "HEIGHT 1",
    "VIEWPOINT 0 0 0 1 0 0 0",
    f"POINTS {len(points)}",
    "DATA ascii",
]
for p in points:
    lines.append(f"{p[0]:.6f} {p[1]:.6f} {p[2]:.6f}")

with open("sample.pcd", "w") as f:
    f.write("\n".join(lines) + "\n")

print(f"Generated {len(points)} points -> sample.pcd")
