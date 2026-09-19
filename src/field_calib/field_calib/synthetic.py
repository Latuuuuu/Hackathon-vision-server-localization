"""Synthetic table scene + self test for the edge calibration."""
import cv2
import numpy as np

from . import core

K = np.array([[643.94, 0, 645.63], [0, 642.97, 368.15], [0, 0, 1.0]])
D = np.array([-0.05697, 0.06652, -0.000459, 0.001326, -0.02179])
T_LO = np.array([0.0, -0.059, 0.0])
Q_LO = np.array([-0.5, 0.5, -0.5, 0.5])
CAM_TF = (0.91, 1.18, 1.3, 0.0, 1.178, -1.5708)


def render_scene(field, p, lower_dx, w=1280, h=720, ss=2, seed=0):
    """Gray image of two white tables on a wood-like floor, with a robot at the origin
    corner and a dark occluder over the lower-right edge."""
    rng = np.random.default_rng(seed)
    R_wc, t_wc = core.world_cam_from_pose(p)
    uu, vv = np.meshgrid((np.arange(w * ss) + 0.5) / ss - 0.5, (np.arange(h * ss) + 0.5) / ss - 0.5)
    pix = np.column_stack([uu.ravel(), vv.ravel()])
    n = cv2.undistortPoints(pix.reshape(-1, 1, 2), K, D).reshape(-1, 2)
    rays = (R_wc @ np.column_stack([n, np.ones(len(n))]).T).T
    s = -t_wc[2] / np.minimum(rays[:, 2], -1e-6)
    X = t_wc[0] + s * rays[:, 0]
    Y = t_wc[1] + s * rays[:, 1]
    img = 105 + 18 * np.sin(X * 35 + 2 * np.sin(Y * 5)) + 8 * np.sin(Y * 90)

    def rounded_rect(x0, y0, x1, y1, rad):
        cx = np.clip(X, x0 + rad, x1 - rad)
        cy = np.clip(Y, y0 + rad, y1 - rad)
        return (X - cx) ** 2 + (Y - cy) ** 2 <= rad ** 2

    L, d = field.L, field.d
    upper = rounded_rect(0, 0, L, d, 0.04)
    lower = rounded_rect(lower_dx, d, L + lower_dx, 2 * d, 0.04)
    img[upper | lower] = 212
    img[(upper | lower) & (np.abs(Y - d) < 0.002)] = 85
    img[(X - 0.08) ** 2 + (Y - 0.02) ** 2 < 0.06 ** 2] = 240
    img = cv2.resize(img.reshape(h * ss, w * ss).astype(np.float32), (w, h), interpolation=cv2.INTER_AREA)
    uv, _ = cv2.projectPoints(np.array([[0.0, 0.95, 0.0]]).reshape(-1, 1, 3), p[:3], p[3:6], K, D)
    u0, v0 = uv.ravel().astype(int)
    cv2.rectangle(img, (u0 - 60, v0 - 70), (u0 + 150, v0 + 70), 30, -1)
    img += rng.normal(0, 3, img.shape)
    return np.clip(img, 0, 255).astype(np.uint8)


def selftest(field, bands, step, blur, grad_thresh, contrast_thresh, delta, trials=3,
             callback=None, log=print):
    """Converge from 3 deg / 10 cm off. Pass = < 0.2 deg and < 5 mm. Returns (ok, gray, last stage)."""
    p_true = core.pose_from_cam_tf(CAM_TF, core.quat_to_rot(Q_LO), T_LO)
    R_wc, t_wc = core.world_cam_from_pose(p_true)
    lower_dx = 0.012 if field.fit_dx else 0.0
    gray = render_scene(field, p_true, lower_dx)
    gray_f = cv2.GaussianBlur(gray.astype(np.float32), (0, 0), blur)
    rng = np.random.default_rng(1)
    ok_all = True
    stage = None
    for trial in range(trials):
        dt = rng.normal(size=3)
        p0 = core.pose_from_world_cam(core.axis_angle_rot(rng.normal(size=3), 3.0) @ R_wc,
                                      t_wc + 0.10 * dt / np.linalg.norm(dt))
        if field.fit_dx:
            p0 = np.append(p0, 0.0)
        log(f'trial {trial}: init off by 3.00 deg, 100.0 mm')
        p, stage = core.calibrate(gray_f, field, p0, K, D, bands, step, grad_thresh, contrast_thresh,
                                  delta, callback=callback if trial == trials - 1 else None, log=log)
        R_est, t_est = core.world_cam_from_pose(p)
        e_rot = core.rot_angle_deg(R_est.T @ R_wc)
        e_pos = np.linalg.norm(t_est - t_wc) * 1000
        msg = f'  result: rot err {e_rot:.3f} deg, pos err {e_pos:.2f} mm'
        if field.fit_dx:
            msg += f', lower_dx {p[6] * 1000:.2f} mm (true {lower_dx * 1000:.1f})'
        ok = e_rot < 0.2 and e_pos < 5.0
        log(msg + ('  PASS' if ok else '  FAIL'))
        ok_all &= ok
    return ok_all, gray, stage
