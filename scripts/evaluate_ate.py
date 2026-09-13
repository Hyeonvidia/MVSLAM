#!/usr/bin/env python3
"""Absolute trajectory error against EuRoC or KITTI ground truth (numpy only).

usage: evaluate_ate.py --gt <mav0/state_groundtruth_estimate0/data.csv> --traj <tum.txt> [--sim3] [--max-dt 0.01]
       evaluate_ate.py --gt <poses/NN.txt> --times <sequences/NN/times.txt> --traj <tum.txt>   (KITTI)
       evaluate_ate.py --gt <groundtruth.txt> --gt-format tum --traj <tum.txt>                 (TUM RGB-D)
Trajectory format: TUM (t[s] tx ty tz qx qy qz qw). Alignment: Umeyama SE(3) (or Sim(3) with --sim3).
"""
import argparse, json, sys
import numpy as np


def load_euroc_gt(path):
    rows = []
    with open(path) as f:
        for line in f:
            if not line or line[0] == '#':
                continue
            v = line.strip().split(',')
            rows.append([float(v[0]) * 1e-9] + [float(x) for x in v[1:4]])
    a = np.array(rows)
    return a[:, 0], a[:, 1:4]


def load_kitti_gt(poses_path, times_path):
    P = np.loadtxt(poses_path).reshape(-1, 3, 4)
    t = np.loadtxt(times_path).reshape(-1)
    n = min(len(P), len(t))
    return t[:n], P[:n, :, 3]


def load_tum_gt(path):
    rows = []
    with open(path) as f:
        for line in f:
            if not line.strip() or line[0] == '#':
                continue
            v = line.split()
            rows.append([float(x) for x in v[:4]])
    a = np.array(rows)
    return a[:, 0], a[:, 1:4]


def load_tum(path):
    a = np.loadtxt(path)
    if a.ndim == 1:
        a = a.reshape(1, -1)
    return a[:, 0], a[:, 1:4]


def associate(t_est, t_gt, max_dt):
    j = np.searchsorted(t_gt, t_est)
    j = np.clip(j, 1, len(t_gt) - 1)
    left, right = t_gt[j - 1], t_gt[j]
    pick = np.where(np.abs(left - t_est) < np.abs(right - t_est), j - 1, j)
    ok = np.abs(t_gt[pick] - t_est) <= max_dt
    return np.nonzero(ok)[0], pick[ok]


def umeyama(src, dst, with_scale):
    mu_s, mu_d = src.mean(0), dst.mean(0)
    xs, xd = src - mu_s, dst - mu_d
    cov = xd.T @ xs / len(src)
    U, D, Vt = np.linalg.svd(cov)
    S = np.eye(3)
    if np.linalg.det(U) * np.linalg.det(Vt) < 0:
        S[2, 2] = -1
    R = U @ S @ Vt
    s = (np.trace(np.diag(D) @ S) / (xs ** 2).sum() * len(src)) if with_scale else 1.0
    t = mu_d - s * R @ mu_s
    return s, R, t


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--gt', required=True)
    ap.add_argument('--traj', required=True)
    ap.add_argument('--sim3', action='store_true')
    ap.add_argument('--max-dt', type=float, default=0.01)
    ap.add_argument('--times', help='KITTI times.txt (selects the KITTI pose format for --gt)')
    ap.add_argument('--gt-format', choices=['euroc', 'tum'], default='euroc')
    a = ap.parse_args()
    if a.times:
        t_gt, p_gt = load_kitti_gt(a.gt, a.times)
    elif a.gt_format == 'tum':
        t_gt, p_gt = load_tum_gt(a.gt)
    else:
        t_gt, p_gt = load_euroc_gt(a.gt)
    t_est, p_est = load_tum(a.traj)
    if len(t_est) == 0:
        print(json.dumps({'error': 'empty trajectory'}))
        sys.exit(1)
    ie, ig = associate(t_est, t_gt, a.max_dt)
    if len(ie) < 3:
        print(json.dumps({'error': 'too few associations', 'pairs': int(len(ie))}))
        sys.exit(1)
    s, R, t = umeyama(p_est[ie], p_gt[ig], a.sim3)
    aligned = (s * (R @ p_est[ie].T)).T + t
    err = np.linalg.norm(aligned - p_gt[ig], axis=1)
    out = {'pairs': int(len(ie)), 'est_poses': int(len(t_est)), 'gt_span_s': float(t_gt[-1] - t_gt[0]),
           'coverage': float((t_est[-1] - t_est[0]) / (t_gt[-1] - t_gt[0])), 'scale': float(s),
           'ate_rmse_m': float(np.sqrt((err ** 2).mean())), 'ate_mean_m': float(err.mean()),
           'ate_median_m': float(np.median(err)), 'ate_max_m': float(err.max())}
    print(json.dumps(out))


if __name__ == '__main__':
    main()
