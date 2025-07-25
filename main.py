import open3d as o3d
import numpy as np
import time
import os

file = './pcd/latest.pcd'

pcd = o3d.geometry.PointCloud()
added = False

def simple_distance_array(points, fov_deg, slices, max_dist, out_of_range_val=65535):
    points = np.asarray(points)
    angles = np.degrees(np.arctan2(points[:,1], points[:,0])) #y/x
    dists = np.linalg.norm(points[:, :2], axis=1) #axis 0 for across rows 1 for across col

    half_fov = fov_deg / 2
    dists = dists[dists <= max_dist]

    bin_size = fov_deg / slices
    bins = ((angles + half_fov) / bin_size).astype(int) #adding +30 to map from 0 index of mat 
    bins = np.clip(bins, 0, slices - 1)

    distances = np.full(slices, out_of_range_val, dtype=float)
    
    #min dist of each bin
    # print(list(zip(bins, dists)))

    for b, dist in zip(bins, dists):
        if dist < distances[b]:
            distances[b] = dist

    return distances


try:
    while True:
        if os.path.exists(file):
            new_pcd = o3d.io.read_point_cloud(file)
            if not new_pcd.is_empty():
                if not added:
                    pcd.points = new_pcd.points
                    
                    # print(points)
                    added = True
                else:
                    pcd.points = new_pcd.points
                    
                points = np.asarray(pcd.points)
                distances = simple_distance_array(points, fov_deg=60, slices=72, max_dist=2)
                print("Distances:", distances)

        time.sleep(0.02)  

except KeyboardInterrupt:
    print("Exiting.")
