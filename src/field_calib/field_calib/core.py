"""Overhead camera extrinsic calibration from the table edges (algorithm core, no ROS).

Pipeline:
  1. initial pose = current map -> camera_color_optical_frame (hand-measured cam_tf)
  2. for every model segment: project with the current pose (with distortion), sample every
     `step` meters, search along the image normal within +-band px for
       - table edge: strongest white -> floor gradient (polarity checked)
       - seam      : darkest valley
     with parabolic subpixel refinement; every sample keeps a status (accepted / reject reason)
  3. 6-DoF (+ optional lower-table x offset) LM with Huber IRLS on the distance of the
     undistorted edge points to the projected (pinhole) model lines
  4. repeat 2-3 with a shrinking band; a callback receives every stage for debug views
"""
import math

import cv2
import numpy as np

# Sample status codes (see extract_edges / calibrate)
ACCEPTED = 0
OUT_OF_IMAGE = 1
WEAK_GRADIENT = 2
LOW_CONTRAST = 3
AT_BAND_EDGE = 4
OUTLIER = 5
STATUS_NAMES = {ACCEPTED: 'accepted', OUT_OF_IMAGE: 'out_of_image', WEAK_GRADIENT: 'weak_gradient',
                LOW_CONTRAST: 'low_contrast', AT_BAND_EDGE: 'at_band_edge', OUTLIER: 'huber_outlier'}

# Profile window (in 0.5 px steps) used for the contrast / valley checks on either side of the peak
_WIN_NEAR, _WIN_FAR = 4, 12


# ---------------------------------------------------------------------------
# Rotation helpers
# ---------------------------------------------------------------------------
def quat_to_rot(q_xyzw):
    x, y, z, w = np.asarray(q_xyzw, dtype=np.float64) / np.linalg.norm(q_xyzw)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def rpy_to_rot(roll, pitch, yaw):
    # tf2 setRPY convention: R = Rz(yaw) * Ry(pitch) * Rx(roll)
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    return rz @ ry @ rx


def rot_to_rpy(R):
    pitch = math.asin(max(-1.0, min(1.0, -R[2, 0])))
    roll = math.atan2(R[2, 1], R[2, 2])
    yaw = math.atan2(R[1, 0], R[0, 0])
    return roll, pitch, yaw


def rot_angle_deg(R):
    return math.degrees(math.acos(max(-1.0, min(1.0, (np.trace(R) - 1) / 2))))


def axis_angle_rot(axis, deg):
    axis = np.asarray(axis, dtype=np.float64)
    R, _ = cv2.Rodrigues(axis / np.linalg.norm(axis) * math.radians(deg))
    return R


def pose_from_world_cam(R_wc, t_wc):
    """world_from_cam -> p = (rvec_cw, tvec_cw) for cv2.projectPoints."""
    R_cw = R_wc.T
    rvec, _ = cv2.Rodrigues(R_cw)
    return np.concatenate([rvec.ravel(), -R_cw @ t_wc])


def world_cam_from_pose(p):
    R_cw, _ = cv2.Rodrigues(np.asarray(p[:3], dtype=np.float64).reshape(3, 1))
    R_wc = R_cw.T
    return R_wc, -R_wc @ p[3:6]


def pose_delta(p_a, p_b):
    """(camera center change in mm, rotation change in deg) between two poses."""
    Ra, ta = world_cam_from_pose(p_a)
    Rb, tb = world_cam_from_pose(p_b)
    return np.linalg.norm(tb - ta) * 1000, rot_angle_deg(Ra.T @ Rb)


def cam_tf_from_pose(p, R_lo, t_lo):
    """map -> camera_link (x, y, z, roll, pitch, yaw) from a map -> optical pose."""
    R_wc, t_wc = world_cam_from_pose(p)
    R_ml = R_wc @ R_lo.T
    t_ml = t_wc - R_ml @ t_lo
    return np.array([*t_ml, *rot_to_rpy(R_ml)])


def pose_from_cam_tf(cam_tf, R_lo, t_lo):
    """Inverse of cam_tf_from_pose."""
    R_ml = rpy_to_rot(*cam_tf[3:6])
    return pose_from_world_cam(R_ml @ R_lo, np.asarray(cam_tf[:3]) + R_ml @ t_lo)


# ---------------------------------------------------------------------------
# Field model
# ---------------------------------------------------------------------------
class Field:
    """Model segments on z = 0. Each segment: name, A(x, y), B(x, y), inward dir, kind, table."""

    def __init__(self, cfg):
        self.L = float(cfg['table_length'])
        self.d = float(cfg['table_depth'])
        self.ce = float(cfg['corner_exclusion'])
        self.fit_dx = bool(cfg.get('fit_lower_dx', False))
        L, d, ce = self.L, self.d, self.ce
        segs = [
            ('upper_top', (ce, 0.0), (L - ce, 0.0), (0, 1), 'edge', 'upper'),
            ('upper_right', (0.0, ce), (0.0, d - ce), (1, 0), 'edge', 'upper'),
            ('upper_left', (L, ce), (L, d - ce), (-1, 0), 'edge', 'upper'),
            ('lower_right', (0.0, d + ce), (0.0, 2 * d - ce), (1, 0), 'edge', 'lower'),
            ('lower_left', (L, d + ce), (L, 2 * d - ce), (-1, 0), 'edge', 'lower'),
            ('lower_bottom', (ce, 2 * d), (L - ce, 2 * d), (0, -1), 'edge', 'lower'),
        ]
        if cfg.get('use_seam', True):
            segs.append(('seam', (ce, d), (L - ce, d), (0, 1), 'seam', 'both'))
        disabled = set(cfg.get('disabled_segments') or [])
        unknown = disabled - {s[0] for s in segs}
        if unknown:
            raise ValueError(f'unknown segments in disabled_segments: {sorted(unknown)}')
        self.segs = [s for s in segs if s[0] not in disabled]
        self.names = [s[0] for s in segs]
        self.weights = np.array([float(cfg.get('seam_weight', 0.5)) if s[4] == 'seam' else 1.0
                                 for s in segs])

    def endpoints(self, dx):
        A = np.zeros((len(self.segs), 3))
        B = np.zeros((len(self.segs), 3))
        for i, (_, a, b, _, _, table) in enumerate(self.segs):
            off = dx if table == 'lower' else 0.0
            A[i, :2] = (a[0] + off, a[1])
            B[i, :2] = (b[0] + off, b[1])
        return A, B

    def inward(self, j):
        return np.array([self.segs[j][3][0], self.segs[j][3][1], 0.0])

    def outline(self, dx, n=60):
        """Full table outlines (incl. corners) and the seam as world polylines, for drawing."""
        L, d = self.L, self.d
        polys = []
        for y0, off in ((0.0, 0.0), (d, dx)):
            c = np.array([[off, y0], [L + off, y0], [L + off, y0 + d], [off, y0 + d], [off, y0]])
            pts = np.concatenate([np.linspace(c[i], c[i + 1], n) for i in range(4)])
            polys.append(np.column_stack([pts, np.zeros(len(pts))]))
        return polys

    def corners(self, dx):
        """Outer field corners (world, z = 0): upper tr, upper tl, lower bl, lower br."""
        L, d = self.L, self.d
        return np.array([[0, 0, 0], [L, 0, 0], [L + dx, 2 * d, 0], [dx, 2 * d, 0]], dtype=np.float64)

    def inside(self, X, Y, dx, margin):
        """Points on the table tops, shrunk by margin."""
        L, d = self.L, self.d
        up = (X > margin) & (X < L - margin) & (Y > margin) & (Y < d - margin)
        lo = (X > dx + margin) & (X < L + dx - margin) & (Y > d + margin) & (Y < 2 * d - margin)
        return up | lo


def get_dx(p):
    return p[6] if len(p) > 6 else 0.0


# ---------------------------------------------------------------------------
# Edge extraction
# ---------------------------------------------------------------------------
def extract_edges(gray, field, p, K, D, band, step, grad_thresh, contrast_thresh):
    """Search every model sample for its edge. Returns a diagnostics dict of per-sample arrays:
    seg, frac (0..1 along the segment), u_pred (distorted px), nrm (outward unit normal),
    status, offset (px along nrm, nan if none), q (detected distorted px), grad, contrast; and band.
    """
    A, B = field.endpoints(get_dx(p))
    rvec, tvec = p[:3], p[3:6]
    h, w = gray.shape
    offs = np.arange(-band, band + 0.25, 0.5)
    ds = offs[1] - offs[0]
    n_off = len(offs)
    cols = {k: [] for k in ('seg', 'frac', 'u_pred', 'nrm', 'status', 'offset', 'grad', 'contrast')}

    for j, seg in enumerate(field.segs):
        length = np.linalg.norm(B[j] - A[j])
        n_s = max(2, int(length / step) + 1)
        P = np.linspace(A[j], B[j], n_s)
        direction = (B[j] - A[j]) / length
        allp = np.concatenate([P, P + 1e-3 * direction, P + 1e-2 * field.inward(j)])
        uv, _ = cv2.projectPoints(allp.reshape(-1, 1, 3), rvec, tvec, K, D)
        uv = uv.reshape(3, n_s, 2)
        u, u_t, u_in = uv[0], uv[1], uv[2]
        t = u_t - u
        t /= np.linalg.norm(t, axis=1, keepdims=True)
        nrm = np.column_stack([-t[:, 1], t[:, 0]])
        # Orient the normal outward (table -> floor)
        nrm[np.sum(nrm * (u_in - u), axis=1) > 0] *= -1

        status = np.full(n_s, OUT_OF_IMAGE)
        offset = np.full(n_s, np.nan)
        grad = np.full(n_s, np.nan)
        contrast = np.full(n_s, np.nan)
        # The search line is clipped to the image; only the model point itself must be inside
        ok = (u[:, 0] > 1) & (u[:, 0] < w - 2) & (u[:, 1] > 1) & (u[:, 1] < h - 2)
        if ok.any():
            ki = np.where(ok)[0]
            mx = (u[ki, 0:1] + offs[None, :] * nrm[ki, 0:1]).astype(np.float32)
            my = (u[ki, 1:2] + offs[None, :] * nrm[ki, 1:2]).astype(np.float32)
            valid = (mx > 1) & (mx < w - 2) & (my > 1) & (my < h - 2)
            prof = cv2.remap(gray, mx, my, cv2.INTER_LINEAR, borderMode=cv2.BORDER_REPLICATE).astype(np.float64)
            # First / last in-image index of every search line (the line is convex, so valid is contiguous)
            lo = np.argmax(valid, axis=1)
            hi = n_off - 1 - np.argmax(valid[:, ::-1], axis=1)
            if seg[4] == 'edge':
                g = np.gradient(prof, ds, axis=1)
                g[~valid] = np.inf
                idx = np.argmin(g, axis=1)
                for row, k in enumerate(ki):
                    i = idx[row]
                    grad[k] = -g[row, i]
                    if i < lo[row] + _WIN_FAR or i > hi[row] - _WIN_FAR:
                        status[k] = AT_BAND_EDGE
                        continue
                    if grad[k] < grad_thresh:
                        status[k] = WEAK_GRADIENT
                        continue
                    contrast[k] = (prof[row, i - _WIN_FAR:i - _WIN_NEAR].mean()
                                   - prof[row, i + _WIN_NEAR:i + _WIN_FAR].mean())
                    if contrast[k] < contrast_thresh:
                        status[k] = LOW_CONTRAST
                        continue
                    offset[k] = offs[i] + _parabola(g[row, i - 1], g[row, i], g[row, i + 1]) * ds
                    status[k] = ACCEPTED
            else:
                idx = np.argmin(np.where(valid, prof, np.inf), axis=1)
                for row, k in enumerate(ki):
                    i = idx[row]
                    if i < lo[row] + _WIN_FAR or i > hi[row] - _WIN_FAR:
                        status[k] = AT_BAND_EDGE
                        continue
                    contrast[k] = (min(prof[row, i - _WIN_FAR:i].max(), prof[row, i + 1:i + _WIN_FAR + 1].max())
                                   - prof[row, i])
                    if contrast[k] < contrast_thresh:
                        status[k] = LOW_CONTRAST
                        continue
                    offset[k] = offs[i] + _parabola(prof[row, i - 1], prof[row, i], prof[row, i + 1]) * ds
                    status[k] = ACCEPTED
        cols['seg'].append(np.full(n_s, j))
        cols['frac'].append(np.linspace(0, 1, n_s))
        cols['u_pred'].append(u)
        cols['nrm'].append(nrm)
        cols['status'].append(status)
        cols['offset'].append(offset)
        cols['grad'].append(grad)
        cols['contrast'].append(contrast)

    diag = {k: np.concatenate(v) for k, v in cols.items()}
    diag['q'] = diag['u_pred'] + np.nan_to_num(diag['offset'])[:, None] * diag['nrm']
    diag['q'][np.isnan(diag['offset'])] = np.nan
    diag['band'] = band
    return diag


def _parabola(y0, y1, y2):
    den = y0 - 2 * y1 + y2
    return 0.5 * (y0 - y2) / den if abs(den) > 1e-9 else 0.0


# ---------------------------------------------------------------------------
# Robust pose optimization
# ---------------------------------------------------------------------------
def line_residuals(p, q_ideal, seg_idx, field, K):
    """Signed pixel distance of undistorted edge points to the projected model lines."""
    A, B = field.endpoints(get_dx(p))
    ab, _ = cv2.projectPoints(np.concatenate([A, B]).reshape(-1, 1, 3), p[:3], p[3:6], K, None)
    ab = ab.reshape(-1, 2)
    a, b = ab[:len(A)][seg_idx], ab[len(A):][seg_idx]
    d = b - a
    nrm = np.column_stack([-d[:, 1], d[:, 0]]) / np.linalg.norm(d, axis=1, keepdims=True)
    return np.sum((q_ideal - a) * nrm, axis=1)


def outward_signs(field, p, K):
    """Per segment: multiply line_residuals by this to get + = outside the model (floor side)."""
    A, B = field.endpoints(get_dx(p))
    signs = np.ones(len(field.segs))
    for j in range(len(field.segs)):
        inside = 0.5 * (A[j] + B[j]) + 0.05 * field.inward(j)
        pr, _ = cv2.projectPoints(np.array([A[j], B[j], inside]).reshape(-1, 1, 3), p[:3], p[3:6], K, None)
        a, b, c = pr.reshape(-1, 2)
        d = b - a
        signs[j] = -1.0 if np.dot(c - a, np.array([-d[1], d[0]])) > 0 else 1.0
    return signs


def huber_weights(r, delta):
    a = np.abs(r)
    w = np.ones_like(r)
    big = a > delta
    w[big] = delta / a[big]
    return w


def _jacobian(p, r, q_ideal, seg_idx, field, K, eps):
    J = np.empty((len(r), len(p)))
    for j in range(len(p)):
        pe = p.copy()
        pe[j] += eps[j]
        J[:, j] = (line_residuals(pe, q_ideal, seg_idx, field, K) - r) / eps[j]
    return J


def robust_lm(p0, q_ideal, seg_idx, field, K, delta=1.5, irls_iter=8, lm_iter=30):
    p = p0.copy()
    seg_w = field.weights[seg_idx]
    eps = np.array([1e-6] * 3 + [1e-5] * 3 + [1e-5] * (len(p) - 6))
    for _ in range(irls_iter):
        r = line_residuals(p, q_ideal, seg_idx, field, K)
        sw = np.sqrt(seg_w * huber_weights(r, delta))
        p_start = p.copy()
        cost = np.sum((sw * r) ** 2)
        lam = 1e-3
        for _ in range(lm_iter):
            J = _jacobian(p, r, q_ideal, seg_idx, field, K, eps)
            Jw = J * sw[:, None]
            A = Jw.T @ Jw
            g = Jw.T @ (sw * r)
            improved = False
            while lam < 1e8:
                step = np.linalg.solve(A + lam * np.diag(np.diag(A)), -g)
                r_try = line_residuals(p + step, q_ideal, seg_idx, field, K)
                c_try = np.sum((sw * r_try) ** 2)
                if c_try < cost:
                    p, r, cost = p + step, r_try, c_try
                    lam = max(lam / 10, 1e-9)
                    improved = True
                    break
                lam *= 10
            if not improved or np.linalg.norm(step) < 1e-10:
                break
        if np.linalg.norm(p - p_start) < 1e-9:
            break
    # Final weights, Jacobian and a (statistical only) covariance
    r = line_residuals(p, q_ideal, seg_idx, field, K)
    w = seg_w * huber_weights(r, delta)
    J = _jacobian(p, r, q_ideal, seg_idx, field, K, eps)
    dof = max(1, len(r) - len(p))
    sigma2 = np.sum(w * r ** 2) / dof
    cov = sigma2 * np.linalg.inv((J * w[:, None]).T @ J)
    return p, r, w, cov


def camera_center_cov(p, cov):
    c0 = world_cam_from_pose(p)[1]
    Jc = np.zeros((3, len(p)))
    for j in range(len(p)):
        pe = p.copy()
        pe[j] += 1e-6
        Jc[:, j] = (world_cam_from_pose(pe)[1] - c0) / 1e-6
    return Jc @ cov @ Jc.T


def _residuals_for(diag, p, field, K, D):
    """Residuals (line_residuals convention) of the accepted samples under pose p."""
    acc = np.where(diag['status'] == ACCEPTED)[0]
    q_ideal = cv2.undistortPoints(diag['q'][acc].reshape(-1, 1, 2), K, D, P=K).reshape(-1, 2)
    return acc, q_ideal


def evaluate(gray, field, p, K, D, band, step, grad_thresh, contrast_thresh, delta):
    """Edge search + residuals under a fixed pose (no optimization) - live view / monitoring."""
    diag = extract_edges(gray, field, p, K, D, band, step, grad_thresh, contrast_thresh)
    acc, q_ideal = _residuals_for(diag, p, field, K, D)
    r = line_residuals(p, q_ideal, diag['seg'][acc], field, K) if len(acc) else np.zeros(0)
    _finish_stage(diag, acc, r, field, p, K, delta)
    return dict(band=band, p_before=p, p_after=p, diag=diag)


def _finish_stage(diag, acc, r, field, p, K, delta):
    """Store signed (+ = outward) residuals per sample and flag Huber outliers."""
    signs = outward_signs(field, p, K)
    res = np.full(len(diag['status']), np.nan)
    res[acc] = r * signs[diag['seg'][acc]]
    diag['resid'] = res
    diag['status'][acc[np.abs(r) >= 3 * delta]] = OUTLIER


def calibrate(gray, field, p0, K, D, bands, step, grad_thresh, contrast_thresh, delta,
              callback=None, log=print):
    """Full calibration. callback(stage) is called after every band with
    stage = dict(index, band, p_before, p_after, diag, cov). Returns (p, last_stage)."""
    p = p0.copy()
    stage = None
    for index, band in enumerate(bands):
        diag = extract_edges(gray, field, p, K, D, band, step, grad_thresh, contrast_thresh)
        acc, q_ideal = _residuals_for(diag, p, field, K, D)
        if len(acc) < 20:
            raise RuntimeError(f'only {len(acc)} edge points found at band {band}px')
        p_before = p
        p, r, w, cov = robust_lm(p, q_ideal, diag['seg'][acc], field, K, delta=delta)
        _finish_stage(diag, acc, r, field, p, K, delta)
        inl = np.abs(r) < 3 * delta
        if log:
            dmm, ddeg = pose_delta(p_before, p)
            log(f'  band {band:3.0f}px: {len(acc):4d} pts, inlier {inl.mean() * 100:5.1f}%, '
                f'inlier RMS {np.sqrt(np.mean(r[inl] ** 2)):.3f} px, pose moved {dmm:.1f} mm / {ddeg:.3f} deg')
        stage = dict(index=index, band=band, p_before=p_before, p_after=p, diag=diag, cov=cov)
        if callback:
            callback(stage)
    return p, stage


# ---------------------------------------------------------------------------
# Reporting
# ---------------------------------------------------------------------------
def segment_stats(field, diag):
    """Per segment: samples, per-status counts, inlier RMS, inlier mean (+ = outward)."""
    rows = []
    for j, name in enumerate(field.names):
        m = diag['seg'] == j
        st = diag['status'][m]
        counts = {s: int(np.sum(st == s)) for s in STATUS_NAMES}
        r = diag['resid'][m & (diag['status'] == ACCEPTED)] if 'resid' in diag else np.zeros(0)
        rows.append(dict(name=name, n=int(m.sum()), counts=counts,
                         rms=float(np.sqrt(np.mean(r ** 2))) if len(r) else float('nan'),
                         mean=float(np.mean(r)) if len(r) else float('nan')))
    return rows


def format_segment_table(rows):
    lines = [f'  {"segment":13s} {"n":>4s} {"acc":>4s} {"weak":>4s} {"lowc":>4s} {"edge":>4s} '
             f'{"outl":>4s} {"oob":>4s} {"RMS px":>7s} {"mean px":>8s}']
    for r in rows:
        c = r['counts']
        lines.append(f'  {r["name"]:13s} {r["n"]:4d} {c[ACCEPTED]:4d} {c[WEAK_GRADIENT]:4d} '
                     f'{c[LOW_CONTRAST]:4d} {c[AT_BAND_EDGE]:4d} {c[OUTLIER]:4d} {c[OUT_OF_IMAGE]:4d} '
                     f'{r["rms"]:7.3f} {r["mean"]:+8.3f}')
    return lines


def ray_to_plane(p, K, D, pix, z):
    R_wc, t_wc = world_cam_from_pose(p)
    n = cv2.undistortPoints(np.asarray(pix, dtype=np.float64).reshape(-1, 1, 2), K, D).reshape(-1, 2)
    rays = (R_wc @ np.column_stack([n, np.ones(len(n))]).T).T
    s = (z - t_wc[2]) / rays[:, 2]
    return t_wc + s[:, None] * rays


def tag_check(gray_u8, poses, K, D, tag_id, size, h):
    """Plane-constrained (z = h) tag center / yaw for each named pose. Returns text lines."""
    if not hasattr(cv2, 'aruco'):
        return ['tag check skipped: cv2.aruco not available']
    dic = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_16h5)
    if hasattr(cv2.aruco, 'DetectorParameters_create'):
        params = cv2.aruco.DetectorParameters_create()
    else:
        params = cv2.aruco.DetectorParameters()
    params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
    corners, ids, _ = cv2.aruco.detectMarkers(gray_u8, dic, parameters=params)
    if ids is None or tag_id not in ids.ravel():
        return [f'tag check: id {tag_id} not found']
    c = corners[list(ids.ravel()).index(tag_id)].reshape(4, 2)
    hs = size / 2
    marker = np.array([[-hs, hs], [hs, hs], [hs, -hs], [-hs, -hs]])
    lines = [f'Tag {tag_id} (z fixed at {h} m), pixel center {c.mean(0).round(1).tolist()}:']
    for name, p in poses:
        P = ray_to_plane(p, K, D, c, h)[:, :2]
        ctr = P.mean(0)
        b = P - ctr
        yaw = math.atan2(np.sum(marker[:, 0] * b[:, 1] - marker[:, 1] * b[:, 0]), np.sum(marker * b))
        lines.append(f'  {name:10s}: x={ctr[0]:.4f} y={ctr[1]:.4f} yaw={math.degrees(yaw):+.2f} deg')
    lines.append('  (compare with the measured ground truth; confirm the robot has not moved)')
    return lines


# ---------------------------------------------------------------------------
# Depth plane cross-check
# ---------------------------------------------------------------------------
def depth_plane_check(depth_m, K_depth, R_cd, t_cd, field, p, margin=0.10, z_gate=0.05,
                      ransac_iter=300, inlier_m=0.005, seed=0):
    """Fit the table plane from depth and compare with the edge solution.

    depth_m: (H, W) depth in meters (0 = invalid), in the depth optical frame.
    R_cd, t_cd: depth -> color optical transform (p_color = R_cd p_depth + t_cd).
    Returns dict(n_points, inlier_ratio, tilt_deg, height_edge, height_depth) or None.
    """
    v, u = np.nonzero(depth_m > 0)
    if len(u) < 1000:
        return None
    step = max(1, len(u) // 60000)
    u, v = u[::step], v[::step]
    z = depth_m[v, u]
    Xd = np.column_stack([(u - K_depth[0, 2]) / K_depth[0, 0] * z, (v - K_depth[1, 2]) / K_depth[1, 1] * z, z])
    Xc = Xd @ R_cd.T + t_cd
    R_wc, t_wc = world_cam_from_pose(p)
    Xw = Xc @ R_wc.T + t_wc
    keep = field.inside(Xw[:, 0], Xw[:, 1], get_dx(p), margin) & (np.abs(Xw[:, 2]) < z_gate)
    Xc = Xc[keep]
    if len(Xc) < 500:
        return None
    rng = np.random.default_rng(seed)
    best, best_n = None, -1
    for _ in range(ransac_iter):
        s = Xc[rng.choice(len(Xc), 3, replace=False)]
        n = np.cross(s[1] - s[0], s[2] - s[0])
        if np.linalg.norm(n) < 1e-9:
            continue
        n /= np.linalg.norm(n)
        cnt = np.sum(np.abs((Xc - s[0]) @ n) < inlier_m)
        if cnt > best_n:
            best, best_n = (n, s[0]), cnt
    n, x0 = best
    inl = Xc[np.abs((Xc - x0) @ n) < inlier_m]
    # Least-squares refit on the inliers
    c = inl.mean(0)
    _, _, vt = np.linalg.svd(inl - c)
    n = vt[2]
    if n @ c > 0:  # orient the normal toward the camera
        n = -n
    # Edge solution: map +Z expressed in the camera frame, and the camera height
    z_cam = R_wc.T @ np.array([0, 0, 1.0])
    return dict(n_points=int(len(Xc)), inlier_ratio=float(len(inl) / len(Xc)),
                tilt_deg=float(math.degrees(math.acos(min(1.0, abs(n @ z_cam))))),
                height_depth=float(-n @ c), height_edge=float(t_wc[2]))
