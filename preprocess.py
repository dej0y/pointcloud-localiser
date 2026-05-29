import numpy as np

VOXEL_SIZE = 0.001

def voxel_downsample(points, voxel_size=VOXEL_SIZE):
    voxel_map = {}
    for point in points:
        voxel_index = tuple(np.floor(point / voxel_size).astype(int))
        
        if voxel_index not in voxel_map:
            voxel_map[voxel_index] = [point.copy(), 1]
        else:
            voxel_map[voxel_index][0] += point
            voxel_map[voxel_index][1] += 1

    downsampled_points = []

    for pts_sum, count in voxel_map.values():
        downsampled_points.append(pts_sum/count)

    return np.array(downsampled_points)
