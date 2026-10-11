#!/usr/bin/env python3
# Plots the traces written by test_trajectory_follower_closed_loop when run
# with MPP_FOLLOWER_TRACE_DIR=<dir>: one row per path, one column per
# simulated sensing/actuation condition (reference path vs ground truth).
#
# Usage: plot_follower_traces.py <trace_dir> [out.png]

import csv
import glob
import math
import os
import sys
from collections import defaultdict

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402

STACKS = ['ideal', 'typical', 'overloaded']


def load(fn):
    ref = []
    gt = []
    with open(fn) as f:
        for row in csv.DictReader(f):
            p = (float(row['x']), float(row['y']), float(row['phi']))
            (ref if row['kind'] == 'ref' else gt).append(p)
    return ref, gt


def arrow(ax, p, color):
    ax.annotate('', xy=(p[0] + 0.5 * math.cos(p[2]), p[1] + 0.5 * math.sin(p[2])),
                xytext=(p[0], p[1]), arrowprops=dict(arrowstyle='->', color=color, lw=1.5))


def main():
    trace_dir = sys.argv[1]
    out = sys.argv[2] if len(sys.argv) > 2 else os.path.join(trace_dir, 'follower_traces.png')
    cases = defaultdict(dict)
    for fn in glob.glob(os.path.join(trace_dir, '*.csv')):
        name = os.path.splitext(os.path.basename(fn))[0]
        case, stack = name.rsplit('_', 1)
        cases[case][stack] = fn
    names = sorted(cases)
    fig, axs = plt.subplots(len(names), len(STACKS), figsize=(5 * len(STACKS), 4 * len(names)),
                            squeeze=False)
    for i, case in enumerate(names):
        for j, stack in enumerate(STACKS):
            ax = axs[i][j]
            if stack not in cases[case]:
                ax.axis('off')
                continue
            ref, gt = load(cases[case][stack])
            ax.plot([p[0] for p in ref], [p[1] for p in ref], '-', color='tab:blue', lw=2,
                    label='reference')
            ax.plot([p[0] for p in gt], [p[1] for p in gt], '-', color='tab:green', lw=1.5,
                    label='ground truth')
            arrow(ax, ref[-1], 'tab:red')
            arrow(ax, gt[-1], 'tab:green')
            err = math.hypot(gt[-1][0] - ref[-1][0], gt[-1][1] - ref[-1][1])
            ax.set_title(f'{case} [{stack}]\nfinal pos err {err:.2f} m', fontsize=9)
            ax.set_aspect('equal')
            ax.grid(alpha=0.3)
            ax.legend(fontsize=7, loc='best')
    fig.tight_layout()
    fig.savefig(out, dpi=90)
    print('saved', out)


if __name__ == '__main__':
    main()
