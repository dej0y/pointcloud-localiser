import open3d as o3d
import numpy as np


def create_transform(rx_deg, ry_deg, rz_deg, translation):

    rx = np.deg2rad(rx_deg)
    ry = np.deg2rad(ry_deg)
    rz = np.deg2rad(rz_deg)

    Rx = np.array([
        [1, 0, 0],
        [0, np.cos(rx), -np.sin(rx)],
        [0, np.sin(rx), np.cos(rx)]
    ])

    Ry = np.array([
        [np.cos(ry), 0, np.sin(ry)],
        [0, 1, 0],
        [-np.sin(ry), 0, np.cos(ry)]
    ])

    Rz = np.array([
        [np.cos(rz), -np.sin(rz), 0],
        [np.sin(rz), np.cos(rz), 0],
        [0, 0, 1]
    ])

    R = Rz @ Ry @ Rx

    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = translation

    return T


def generate_synthetic_pair(
    mesh_path,
    sample_points=10000,
    source_noise=0.0001,
    target_noise=0.002,
    rx=10,
    ry=-5,
    rz=20,
    translation=np.array([0.1, 0.02, -0.03]),
):

    mesh = o3d.io.read_triangle_mesh(mesh_path)

    if mesh.is_empty():
        raise ValueError("Mesh failed to load")

    mesh.compute_vertex_normals()

    source = mesh.sample_points_poisson_disk(
        number_of_points=sample_points
    )

    points = np.asarray(source.points)

    points += np.random.normal(
        loc=0.0,
        scale=source_noise,
        size=points.shape
    )

    source.points = o3d.utility.Vector3dVector(points)

    T_gt = create_transform(
        rx,
        ry,
        rz,
        translation
    )

    R = T_gt[:3, :3]
    t = T_gt[:3, 3]

    target_points = (
        R @ points.T
    ).T + t

    target_points += np.random.normal(
        loc=0.0,
        scale=target_noise,
        size=target_points.shape
    )

    target = o3d.geometry.PointCloud()

    target.points = (
        o3d.utility.Vector3dVector(
            target_points
        )
    )

    return source, target, T_gt


if __name__ == "__main__":

    source, target, T_gt = generate_synthetic_pair(
        mesh_path="data/bunny/data/stanford-bunny.obj",
        sample_points=10000,
        target_noise=0.002,
        rx=10,
        ry=-5,
        rz=20,
        translation=np.array([0.1, 0.02, -0.03])
    )

    source.paint_uniform_color([1, 0, 0])
    target.paint_uniform_color([0, 1, 0])

    output_dir = "data/bunny/synthetic"

    o3d.io.write_point_cloud(
        f"{output_dir}/source.ply",
        source
    )

    o3d.io.write_point_cloud(
        f"{output_dir}/target.ply",
        target
    )

    np.save(
        f"{output_dir}/T_gt.npy",
        T_gt
    )

    print("Saved:")
    print(f"{output_dir}/source.ply")
    print(f"{output_dir}/target.ply")
    print(f"{output_dir}/T_gt.npy")

    print("\nGround Truth Transform:")
    print(T_gt)

    o3d.visualization.draw_geometries(
        [source, target]
    )