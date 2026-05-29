<h1 align="center">PointCloud Localizer</h1>

A custom implementation of Iterative Closest Point (ICP) for registration of overlapping point clouds.

---

## Environment Setup

### Create Virtual Environment

```bash
python3 -m venv venv
```

Activate:

#### Linux / macOS

```bash
source venv/bin/activate
```

#### Windows

```powershell
venv\Scripts\activate
```

---

### Install Dependencies

Install all dependencies from the provided requirements file:

```bash
pip install -r requirements.txt
```

Verify installation:

```bash
python -c "import open3d, numpy, scipy"
```

---

## Methodology

### Synthetic Dataset Generation

A synthetic target point cloud is generated from a source cloud using a known rigid transformation.

Steps:

1. Load Stanford Bunny mesh.
2. Sample points using Poisson-disk sampling.
3. Add Gaussian measurement noise.
4. Apply a known rigid transform.
5. Add additional noise to the transformed cloud.
6. Save the ground-truth transformation.

Ground-truth transform:

```text
Rotation:
Rx = 10°
Ry = -5°
Rz = 20°

Translation:
[0.10, 0.02, -0.03] m
```

Generated files:

```text
source.ply
target.ply
T_gt.npy
```

---

### Point Cloud Preprocessing

Voxel downsampling is implemented manually.

Procedure:

1. Divide 3D space into voxels.
2. Assign each point to a voxel.
3. Compute the centroid of all points within a voxel.
4. Replace all points in the voxel with the centroid.

Benefits:

- Reduces computational cost
- Reduces redundant points
- Improves ICP runtime

---

### ICP Registration

The implementation follows the standard ICP pipeline.

#### Step 1: Correspondence Search

Nearest-neighbour correspondences are found using:

```python
scipy.spatial.cKDTree
```

Each source point is matched with its nearest target point.

---

#### Step 2: Rigid Transform Estimation

For matched point sets:

```text
P = {p_i}
Q = {q_i}
```

The optimal rigid transform is computed using the Kabsch algorithm.

Minimizes:

```math
\sum_i ||Rp_i + t - q_i||^2
```

Steps:

1. Compute centroids
2. Remove centroids
3. Compute covariance matrix
4. Perform SVD
5. Recover rotation matrix
6. Recover translation vector

---

#### Step 3: Apply Transform

The estimated transform is applied to the source cloud.

The global transformation estimate is accumulated across iterations.

---

#### Step 4: Convergence Check

ICP terminates when:

```math
|RMSE_k - RMSE_{k-1}| < \epsilon
```

or when the maximum iteration limit is reached.

---

## Evaluation Metrics

### Translation Error

```math
e_t = ||t_{est} - t_{gt}||
```

Reported in metres.

---

### Rotation Error

Compute:

```math
R_{err}=R_{gt}^{T}R_{est}
```

Then:

```math
\theta=\cos^{-1}
\left(
\frac{\text{trace}(R_{err})-1}{2}
\right)
```

Reported in degrees.

---

### RMSE

```math
RMSE =
\sqrt{
\frac{1}{N}
\sum_i
||p_i-q_i||^2
}
```

Used to monitor convergence during registration.

---

## Running the Project

### Generate Synthetic Dataset

Noise-free:

```bash
python cli.py \
    --generate-synthetic \
    --mesh data/bunny/data/stanford-bunny.obj \
    --noise 0
```

Noise level 0.005 m:

```bash
python cli.py \
    --generate-synthetic \
    --mesh data/bunny/data/stanford-bunny.obj \
    --noise 0.005
```

Noise level 0.02 m:

```bash
python cli.py \
    --generate-synthetic \
    --mesh data/bunny/data/stanford-bunny.obj \
    --noise 0.02
```

---

### Run ICP Registration

```bash
python cli.py \
    --source output/sweep/noise0/source.ply \
    --target output/sweep/noise0/target.ply \
    --gt output/sweep/noise0/T_gt.npy \
    --init output/initial_small.npy
```

---

## Results

### Noise-Free Registration

ICP successfully recovered the exact ground-truth transformation for all tested initialization magnitudes.

| Initialization | Rotation Error | Translation Error |
|---------------|----------------|------------------|
| Small | 0° | 0 m |
| Medium | 0° | 0 m |
| Large | 0° | 0 m |

---

## Robustness Sweep

Three noise levels and three initialization magnitudes were evaluated.

### Results Table

| Noise σ (m) | Initial Misalignment | Final RMSE | Rotation Error (deg) | Translation Error (m) |
|------------|------|------------|----------------------|----------------------|
| 0.000 | Small | 0.0000 | 0.00 | 0.0000 |
| 0.000 | Medium | 0.0000 | 0.00 | 0.0000 |
| 0.000 | Large | 0.0000 | 0.00 | 0.0000 |
| 0.005 | Small | 0.00249 | 4.78 | 0.00875 |
| 0.005 | Medium | 0.00245 | 2.71 | 0.00633 |
| 0.005 | Large | 0.00243 | 1.28 | 0.00149 |
| 0.010 | Small | 0.00345 | 14.87 | 0.03417 |
| 0.010 | Medium | 0.00310 | 7.99 | 0.01760 |
| 0.010 | Large | 0.00311 | 5.39 | 0.00699 |
| 0.020 | Small | 0.00414 | 18.57 | 0.05822 |
| 0.020 | Medium | 0.00407 | 15.44 | 0.03779 |
| 0.020 | Large | 0.00450 | 22.74 | 0.01591 |

---

## Discussion

The Stanford Bunny point cloud has approximate dimensions:

```text
0.156 m × 0.154 m × 0.121 m
```

The noise levels therefore correspond to:

| Noise | Percentage of Bunny Width |
|---------|---------|
| 0.005 m | ~3.2% |
| 0.010 m | ~6.4% |
| 0.020 m | ~12.8% |

Observations:

- Noise-free data yields perfect recovery.
- Registration quality degrades as noise increases.
- RMSE increases monotonically with noise.
- Large noise produces inaccurate correspondences.
- Point-to-point ICP becomes increasingly sensitive to local minima.
- At σ = 0.02 m, the algorithm frequently converges to incorrect alignments despite continued RMSE reduction.

These results demonstrate a known limitation of ICP: its dependence on correspondence quality and initialization.

---

## Output Files

The output/ directory contains registration results, evaluation metrics, and visualization plots produced during experimentation.  
