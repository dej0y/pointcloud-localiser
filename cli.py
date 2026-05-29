import argparse
import numpy as np
import open3d as o3d

from loader import load_pcd
from preprocess import voxel_downsample
from icp import (
    icp_registration,
    apply_transform,
)
from evaluate import (
    evaluate_transform,
    plot_rmse,
)


def main():

    parser = argparse.ArgumentParser()

    parser.add_argument(
        "--source",
        required=True,
    )

    parser.add_argument(
        "--target",
        required=True,
    )

    parser.add_argument(
        "--gt",
        default=None,
        help="Ground truth transform (.npy)"
    )

    args = parser.parse_args()

    source = load_pcd(args.source)
    target = load_pcd(args.target)

    source_pc = voxel_downsample(
        np.asarray(source.points)
    )

    target_pc = voxel_downsample(
        np.asarray(target.points)
    )

    result = icp_registration(
        source_pc,
        target_pc,
    )

    print("\n===== ICP RESULT =====")

    print(
        f"Iterations : {result['iterations']}"
    )

    print(
        f"Final RMSE : {result['rmse']:.6f}"
    )

    print("\nEstimated Transform:")

    print(
        result["transformation"]
    )

    plot_rmse(
        result["rmse_history"],
        "rmse_curve.png"
    )

    if args.gt:

        T_gt = np.load(args.gt)

        rot_err, trans_err = evaluate_transform(
            result["transformation"],
            T_gt,
        )

        print(
            f"\nRotation Error (deg): {rot_err:.4f}"
        )

        print(
            f"Translation Error (m): {trans_err:.6f}"
        )

    source_final = o3d.geometry.PointCloud()
    target_final = o3d.geometry.PointCloud()

    aligned_source = apply_transform(
        source_pc,
        result["transformation"],
    )

    source_final.points = (
        o3d.utility.Vector3dVector(
            aligned_source
        )
    )

    target_final.points = (
        o3d.utility.Vector3dVector(
            target_pc
        )
    )

    source_final.paint_uniform_color(
        [1, 0, 0]
    )

    target_final.paint_uniform_color(
        [0, 0, 1]
    )

    o3d.visualization.draw_geometries(
        [
            source_final,
            target_final,
        ],
        window_name="After Registration",
    )


if __name__ == "__main__":
    main()