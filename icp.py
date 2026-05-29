import numpy as np
from scipy.spatial import cKDTree


def apply_transform(points, T):
    ones = np.ones((points.shape[0], 1))
    homo = np.hstack((points, ones))
    transformed = (T @ homo.T).T
    return transformed[:, :3]


def nearest_neighbor(src, tgt):
    tree = cKDTree(tgt)
    distances, indices = tree.query(src)
    return distances, indices


def compute_rigid_transform(src, tgt):
    centroid_src = np.mean(src, axis=0)
    centroid_tgt = np.mean(tgt, axis=0)

    src_centered = src - centroid_src
    tgt_centered = tgt - centroid_tgt

    H = src_centered.T @ tgt_centered

    U, _, Vt = np.linalg.svd(H)

    R = Vt.T @ U.T

    if np.linalg.det(R) < 0:
        Vt[-1, :] *= -1
        R = Vt.T @ U.T

    t = centroid_tgt - R @ centroid_src

    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = t

    return T


def compute_rmse(src, tgt):
    errors = np.linalg.norm(src - tgt, axis=1)
    return np.sqrt(np.mean(errors ** 2))


def icp_registration(
    source_points,
    target_points,
    threshold=0.02,
    max_iterations=100,
    tolerance=1e-7,
    trans_init=np.eye(4),
):

    src_transformed = apply_transform(source_points, trans_init)

    final_transform = trans_init.copy()

    prev_rmse = np.inf
    rmse_history = []

    for i in range(max_iterations):

        distances, indices = nearest_neighbor(
            src_transformed,
            target_points
        )

        mask = distances < threshold

        src_corr = src_transformed[mask]
        tgt_corr = target_points[indices[mask]]

        if len(src_corr) < 3:
            raise RuntimeError(
                "Too few correspondences."
            )

        T = compute_rigid_transform(
            src_corr,
            tgt_corr
        )

        src_transformed = apply_transform(
            src_transformed,
            T
        )

        final_transform = T @ final_transform

        src_corr_aligned = apply_transform(
            src_corr,
            T
        )

        rmse = compute_rmse(
            src_corr_aligned,
            tgt_corr
        )

        rmse_history.append(rmse)

        print(
            f"Iteration {i+1}: RMSE={rmse:.6f}"
        )

        if abs(prev_rmse - rmse) < tolerance:
            break

        prev_rmse = rmse

    return {
        "transformation": final_transform,
        "rmse": rmse,
        "iterations": i + 1,
        "rmse_history": rmse_history,
    }