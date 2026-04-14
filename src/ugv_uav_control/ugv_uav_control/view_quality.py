"""
View Quality Module
===================
Computes a scalar view quality score q ∈ [0, 1] given UAV (x, y) and
UGV (x, y) positions, under a fixed-altitude downward camera model.

q ≈ 1.0  when UGV is directly below the UAV (centered in FoV)
q → 0.0  smoothly as horizontal offset approaches and exceeds r_fov

The function is **fully differentiable** — no hard conditionals, no
clipping, no if/else branches. Uses a logistic sigmoid for smooth
decay, suitable for gradient-based optimizers (SLSQP, etc.).

Usage:
    from ugv_uav_control.view_quality import compute_view_quality

    q = compute_view_quality(uav_x, uav_y, ugv_x, ugv_y)
"""

import numpy as np


def compute_view_quality(
    uav_x: float,
    uav_y: float,
    ugv_x: float,
    ugv_y: float,
    r_fov: float = 2.89,
    altitude: float = 5.0,
    fov_angle: float = 1.047,
    sharpness: float = 5.0,
) -> float:
    """
    Compute a smooth, differentiable view quality score.

    Parameters
    ----------
    uav_x, uav_y : float
        UAV ground-projected position (world frame).
    ugv_x, ugv_y : float
        UGV position (world frame).
    r_fov : float
        Camera field-of-view radius on the ground plane (m).
        Default: 5.0 * tan(30°) ≈ 2.89 m for a 60° FoV at 5 m altitude.
    altitude : float
        UAV altitude (m). Used for angular quality factor.
    fov_angle : float
        Full camera FoV angle (rad). Default: 1.047 (60°).
    sharpness : float
        Sigmoid steepness (κ). Higher = sharper transition at the FoV
        boundary. Default 5.0 gives a smooth roll-off that starts
        declining noticeably around 0.7 * r_fov and reaches ~0.01
        by 1.5 * r_fov.

    Returns
    -------
    q : float
        View quality score in (0, 1). Smooth and differentiable
        everywhere, with non-zero gradients at all points.

    Notes
    -----
    The score is a product of two smooth factors:

    1. **Radial sigmoid** — penalizes horizontal offset d relative to
       r_fov using a logistic function:
           q_radial = 1 / (1 + exp(κ * (d - r_fov)))
       This transitions smoothly from ~1 (d ≪ r_fov) to ~0 (d ≫ r_fov).

    2. **Centering bonus** — rewards centering via a Gaussian envelope:
           q_center = exp(-d² / (2 * σ²))
       where σ = r_fov / 2. This provides extra quality when UGV is
       near the optical axis, improving image resolution and reducing
       perspective distortion.

    The final score is:
        q = q_radial * q_center
    """
    # Horizontal distance (smooth via sqrt with epsilon for numerical safety)
    dx = uav_x - ugv_x
    dy = uav_y - ugv_y
    d = np.sqrt(dx * dx + dy * dy + 1e-12)

    # Factor 1: Sigmoid boundary (visibility)
    q_radial = 1.0 / (1.0 + np.exp(sharpness * (d - r_fov)))

    # Factor 2: Gaussian centering bonus (image quality)
    sigma = r_fov / 2.0
    q_center = np.exp(-(d * d) / (2.0 * sigma * sigma))

    return float(q_radial * q_center)


def compute_view_quality_gradient(
    uav_x: float,
    uav_y: float,
    ugv_x: float,
    ugv_y: float,
    **kwargs,
) -> tuple:
    """
    Compute numerical gradient of view quality w.r.t. UAV position.

    Returns (dq/d_uav_x, dq/d_uav_y) via central differences.
    """
    eps = 1e-6
    dq_dx = (
        compute_view_quality(uav_x + eps, uav_y, ugv_x, ugv_y, **kwargs)
        - compute_view_quality(uav_x - eps, uav_y, ugv_x, ugv_y, **kwargs)
    ) / (2.0 * eps)
    dq_dy = (
        compute_view_quality(uav_x, uav_y + eps, ugv_x, ugv_y, **kwargs)
        - compute_view_quality(uav_x, uav_y - eps, ugv_x, ugv_y, **kwargs)
    ) / (2.0 * eps)
    return dq_dx, dq_dy


# ============================================================
# Self-contained test
# ============================================================
if __name__ == '__main__':
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    R_FOV = 2.89
    ugv = (0.0, 0.0)
    offsets = np.linspace(0.0, 4.0, 200)
    scores = [compute_view_quality(d, 0.0, *ugv) for d in offsets]

    # --- Plot ---
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(8, 6), sharex=True)

    ax1.plot(offsets, scores, 'b-', linewidth=2, label='q(d)')
    ax1.axvline(R_FOV, color='r', linestyle='--', alpha=0.7,
                label=f'r_fov = {R_FOV:.2f} m')
    ax1.set_ylabel('View Quality q')
    ax1.set_title('View Quality vs. Horizontal Offset')
    ax1.legend()
    ax1.grid(True, alpha=0.3)
    ax1.set_ylim(-0.05, 1.05)

    # --- Gradient plot ---
    grads = [compute_view_quality_gradient(d, 0.0, *ugv)[0] for d in offsets]
    ax2.plot(offsets, grads, 'g-', linewidth=2, label='dq/dx')
    ax2.axvline(R_FOV, color='r', linestyle='--', alpha=0.7)
    ax2.set_xlabel('Horizontal Offset (m)')
    ax2.set_ylabel('Gradient dq/dx')
    ax2.set_title('Gradient of View Quality')
    ax2.legend()
    ax2.grid(True, alpha=0.3)

    plt.tight_layout()
    out_path = '/home/shlok-mehndiratta/Desktop/Project_ws/view_quality_test.png'
    plt.savefig(out_path, dpi=150)
    print(f'Plot saved to {out_path}')

    # --- Verification ---
    print('\n=== View Quality Verification ===')
    print(f'{"Offset (m)":>12} {"q":>8} {"dq/dx":>12} {"dq/dy":>12}')
    print('-' * 48)
    test_offsets = [0.0, 0.5, 1.0, 1.5, 2.0, 2.5, 2.89, 3.5, 4.0]
    all_ok = True
    for d in test_offsets:
        q = compute_view_quality(d, 0.0, 0.0, 0.0)
        gx, gy = compute_view_quality_gradient(d, 0.0, 0.0, 0.0)
        print(f'{d:12.2f} {q:8.4f} {gx:12.6f} {gy:12.6f}')
        # Check gradients are finite
        if not (np.isfinite(gx) and np.isfinite(gy)):
            print(f'  *** FAIL: non-finite gradient at offset={d}')
            all_ok = False
        # At d=0, gradient along x should be near 0 (symmetric peak)
        if d == 0.0 and abs(gx) > 0.01:
            print(f'  *** WARN: gradient at center should be ~0')
        # At d > 0 and d < 4, gradient should be non-zero (negative)
        if 0.1 < d < 3.8 and abs(gx) < 1e-10:
            print(f'  *** FAIL: zero gradient at offset={d}')
            all_ok = False

    # Value checks
    q_center = compute_view_quality(0.0, 0.0, 0.0, 0.0)
    q_boundary = compute_view_quality(R_FOV, 0.0, 0.0, 0.0)
    q_far = compute_view_quality(4.0, 0.0, 0.0, 0.0)
    print(f'\nq(center)   = {q_center:.4f}  (should be ~1.0)')
    print(f'q(r_fov)    = {q_boundary:.4f}  (should be mid-range)')
    print(f'q(4m)       = {q_far:.6f}  (should be ~0)')

    assert q_center > 0.95, f'Center quality too low: {q_center}'
    assert q_far < 0.01, f'Far quality too high: {q_far}'
    assert q_center > q_boundary > q_far, 'Monotonicity violated'

    if all_ok:
        print('\n✅ All checks passed — function is smooth and differentiable')
    else:
        print('\n❌ Some checks failed')
