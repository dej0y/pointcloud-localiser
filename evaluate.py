import numpy as np
import matplotlib.pyplot as plt


def translation_error(T_est, T_gt):
    return np.linalg.norm(
        T_est[:3, 3] - T_gt[:3, 3]
    )


def rotation_error(T_est, T_gt):
    R_est = T_est[:3, :3]
    R_gt = T_gt[:3, :3]

    R_err = R_gt.T @ R_est

    angle = np.arccos(
        np.clip(
            (np.trace(R_err) - 1) / 2,
            -1.0,
            1.0,
        )
    )

    return np.degrees(angle)


def evaluate_transform(
    T_est,
    T_gt,
):
    rot_err = rotation_error(
        T_est,
        T_gt,
    )

    trans_err = translation_error(
        T_est,
        T_gt,
    )

    return rot_err, trans_err


def plot_rmse(
    rmse_history,
    save_path=None,
):
    plt.figure(figsize=(6, 4))

    plt.plot(
        rmse_history,
        marker="o",
    )

    plt.xlabel("Iteration")
    plt.ylabel("RMSE")
    plt.title("ICP Convergence")
    plt.grid(True)

    if save_path:
        plt.savefig(
            save_path,
            bbox_inches="tight",
        )

    plt.show()