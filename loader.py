import open3d as o3d

def load_pcd(pcd_path):
    pcd = o3d.io.read_point_cloud(pcd_path)

    if pcd.is_empty():
        raise ValueError("Point cloud failed to load")

    return pcd