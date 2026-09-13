#!/usr/bin/env python3
"""Relocalization success rate of `mvslam_dataset_runner --localize-every N --localize-out loc.txt`.

usage: evaluate_localization.py --ref <traj of the mapping run, TUM> --loc <loc.txt> [--pos-tol 0.05] [--rot-tol-deg 3]
                               [--gt <EuRoC gt csv>]
loc.txt lines: t ok tx ty tz qx qy qz qw. A localization counts as a success when ok == 1 and the pose is
within the tolerances of the mapping run's own pose at that time (same map frame, no alignment needed).
With --gt the successful poses are also aligned (Umeyama SE3) to ground truth for an ATE.
"""
import argparse, json, sys
import numpy as np


def quat_angle_deg(q1, q2):  # xyzw
    d = abs(float(np.dot(q1, q2)))
    return 2.0 * np.degrees(np.arccos(min(1.0, d)))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--ref', required=True)
    ap.add_argument('--loc', required=True)
    ap.add_argument('--pos-tol', type=float, default=0.05)
    ap.add_argument('--rot-tol-deg', type=float, default=3.0)
    ap.add_argument('--gt')
    a = ap.parse_args()
    ref = np.loadtxt(a.ref)
    loc = np.loadtxt(a.loc)
    if loc.ndim == 1:
        loc = loc.reshape(1, -1)
    t_ref = ref[:, 0]
    pos_err, rot_err, ok_rows = [], [], []
    for row in loc:
        t, ok = row[0], int(row[1])
        if not ok:
            continue
        j = int(np.argmin(np.abs(t_ref - t)))
        if abs(t_ref[j] - t) > 1e-3:
            continue
        pos_err.append(float(np.linalg.norm(row[2:5] - ref[j, 1:4])))
        rot_err.append(quat_angle_deg(row[5:9], ref[j, 4:8]))
        ok_rows.append(row)
    pos_err, rot_err = np.array(pos_err), np.array(rot_err)
    within = int(np.sum((pos_err <= a.pos_tol) & (rot_err <= a.rot_tol_deg))) if len(pos_err) else 0
    out = {'attempts': int(len(loc)), 'localized': int(np.sum(loc[:, 1] > 0)), 'within_tolerance': within,
           'success_rate': within / len(loc) if len(loc) else 0.0,
           'pos_err_median_m': float(np.median(pos_err)) if len(pos_err) else None,
           'pos_err_max_m': float(np.max(pos_err)) if len(pos_err) else None,
           'rot_err_median_deg': float(np.median(rot_err)) if len(rot_err) else None}
    if a.gt and ok_rows:
        sys.path.insert(0, __file__.rsplit('/', 1)[0])
        from evaluate_ate import load_euroc_gt, associate, umeyama
        t_gt, p_gt = load_euroc_gt(a.gt)
        rows = np.array(ok_rows)
        ie, ig = associate(rows[:, 0], t_gt, 0.01)
        if len(ie) >= 3:
            s, R, t = umeyama(rows[ie, 2:5], p_gt[ig], False)
            aligned = (R @ rows[ie, 2:5].T).T + t
            err = np.linalg.norm(aligned - p_gt[ig], axis=1)
            out['gt_ate_rmse_m'] = float(np.sqrt((err ** 2).mean()))
            out['gt_pairs'] = int(len(ie))
    print(json.dumps(out))


if __name__ == '__main__':
    main()
