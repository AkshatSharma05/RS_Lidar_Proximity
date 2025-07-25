import open3d as o3d
import numpy as np
import time
import os
import pygame
import math

file = './pcd/latest.pcd'

pcd = o3d.geometry.PointCloud()
added = False

# Visualization parameters
SCREEN_SIZE = 500
CENTER = SCREEN_SIZE // 2
FOV = 60
SLICES = 72
MAX_DIST = 4.0
OUT_OF_RANGE_VAL = 65535

# Initialize pygame
pygame.init()
screen = pygame.display.set_mode((SCREEN_SIZE, SCREEN_SIZE))
pygame.display.set_caption("LIDAR Radar View")
clock = pygame.time.Clock()

def simple_distance_array(points, fov_deg, slices, max_dist, out_of_range_val=65535):
    points = np.asarray(points)
    angles = np.degrees(np.arctan2(points[:,1], points[:,0]))  # y/x
    dists = np.linalg.norm(points[:, :2], axis=1)

    half_fov = fov_deg / 2
    valid = dists <= max_dist
    dists = dists[valid]
    angles = angles[valid]

    bin_size = fov_deg / slices
    bins = ((angles + half_fov) / bin_size).astype(int)
    bins = np.clip(bins, 0, slices - 1)

    distances = np.full(slices, out_of_range_val, dtype=float)
    for b, dist in zip(bins, dists):
        if dist < distances[b]:
            distances[b] = dist
    return distances

def draw_radar(distances, fov_deg, slices, max_dist):
    screen.fill((0, 0, 0))  # Clear screen

    bin_angle = fov_deg / slices
    for i, dist in enumerate(distances):
        angle_deg = -fov_deg / 2 + i * bin_angle
        angle_rad = math.radians(angle_deg)
        if dist != OUT_OF_RANGE_VAL:
            length = (dist / max_dist) * (SCREEN_SIZE // 2)
            end_x = int(CENTER + length * math.cos(angle_rad))
            end_y = int(CENTER - length * math.sin(angle_rad))
            pygame.draw.line(screen, (0, 255, 0), (CENTER, CENTER), (end_x, end_y), 2)
        else:
            # Draw faint gray for out-of-range
            end_x = int(CENTER + (SCREEN_SIZE//2) * math.cos(angle_rad))
            end_y = int(CENTER - (SCREEN_SIZE//2) * math.sin(angle_rad))
            pygame.draw.line(screen, (50, 50, 50), (CENTER, CENTER), (end_x, end_y), 1)

    pygame.draw.circle(screen, (255, 255, 255), (CENTER, CENTER), SCREEN_SIZE // 2, 1)
    pygame.display.flip()

try:
    while True:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                raise KeyboardInterrupt
        
        if os.path.exists(file):
            new_pcd = o3d.io.read_point_cloud(file)
            if not new_pcd.is_empty():
                if not added:
                    pcd.points = new_pcd.points
                    added = True
                else:
                    pcd.points = new_pcd.points

                points = np.asarray(pcd.points)
                distances = simple_distance_array(points, fov_deg=FOV, slices=SLICES, max_dist=MAX_DIST)
                print("Distances:", distances)
                draw_radar(distances, fov_deg=FOV, slices=SLICES, max_dist=MAX_DIST)

        time.sleep(0.02)
        clock.tick(30)

except KeyboardInterrupt:
    print("Exiting.")
    pygame.quit()
