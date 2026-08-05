#!/usr/bin/env python3
"""Offline sandbox for trying segmentation ideas against a recorded cloud.

    ros2 run lunabot_perception prototype_cluster.py cloud.npy --plot

NOT part of the runtime pipeline. The shipped detector is C++ because the real
thing runs at 10 Hz on an embedded board, and prototyping in Python then
rewriting in February is the trap this repo is trying to avoid.

What this IS for: deciding whether an idea is worth implementing at all.
Trying a RANSAC ground fit, a different clustering approach, or a shape
descriptor takes minutes here and an afternoon in C++.

Once an idea works, port it into cloud_segmentation.cpp and add a case to
test_cloud_segmentation.cpp -- that test file is the record of what the
pipeline is supposed to do.

Input is an (N, 3) float array saved with numpy.save. To capture one from a
running system:

    ros2 topic echo /oak_d/points --once  # then convert, or use ros2 bag
"""

import argparse
import sys

import numpy as np


def split_by_ground(points, ground_z, threshold):
    """Three-way split, matching cloud_segmentation.cpp.

    Three, not two: boulders are above, craters are below. Keeping the same
    shape here means a prototype ports across without restructuring.
    """
    height = points[:, 2] - ground_z
    return (
        points[np.abs(height) <= threshold],
        points[height > threshold],
        points[height < -threshold],
    )


def cluster(points, tolerance, min_points):
    """Single-link clustering via a KD-tree, standing in for PCL's Euclidean
    extraction. Same idea, far slower, fine for a few thousand points."""
    if len(points) == 0:
        return []

    try:
        from scipy.cluster.hierarchy import fcluster, linkage
        from scipy.spatial.distance import pdist
    except ImportError:
        print('needs scipy: pip install scipy', file=sys.stderr)
        raise

    if len(points) > 4000:
        # pdist is O(n^2) in memory. Subsample rather than exhaust RAM.
        idx = np.random.default_rng(0).choice(len(points), 4000, replace=False)
        points = points[idx]

    labels = fcluster(linkage(pdist(points), method='single'), tolerance, criterion='distance')

    clusters = []
    for label in np.unique(labels):
        member = points[labels == label]
        if len(member) < min_points:
            continue
        lo, hi = member.min(axis=0), member.max(axis=0)
        clusters.append(
            {
                'centroid': (lo + hi) / 2,  # box centre, not point centroid
                'dimensions': hi - lo,
                'points': len(member),
            }
        )
    return clusters


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('cloud', help='(N, 3) float array saved with numpy.save')
    parser.add_argument('--ground-z', type=float, default=-0.10)
    parser.add_argument('--ground-threshold', type=float, default=0.05)
    parser.add_argument('--tolerance', type=float, default=0.10)
    parser.add_argument('--min-points', type=int, default=20)
    parser.add_argument('--plot', action='store_true', help='needs matplotlib')
    args = parser.parse_args()

    points = np.load(args.cloud)
    points = points[np.isfinite(points).all(axis=1)]
    print(f'{len(points)} finite points')

    ground, above, below = split_by_ground(points, args.ground_z, args.ground_threshold)
    print(f'ground {len(ground)}  above {len(above)}  below {len(below)}')

    clusters = cluster(above, args.tolerance, args.min_points)
    print(f'\n{len(clusters)} clusters:')
    for i, c in enumerate(clusters):
        centre = np.round(c['centroid'], 3)
        size = np.round(c['dimensions'], 3)
        print(f'  {i}: centre={centre}  size={size}  points={c["points"]}')

    if args.plot:
        import matplotlib.pyplot as plt

        _, ax = plt.subplots(figsize=(8, 8))
        ax.scatter(ground[:, 0], ground[:, 1], s=1, c='0.7', label='ground')
        ax.scatter(above[:, 0], above[:, 1], s=2, c='tab:orange', label='above')
        if len(below):
            ax.scatter(below[:, 0], below[:, 1], s=2, c='tab:blue', label='below')
        for c in clusters:
            ax.add_patch(
                plt.Rectangle(
                    (c['centroid'][0] - c['dimensions'][0] / 2,
                     c['centroid'][1] - c['dimensions'][1] / 2),
                    c['dimensions'][0],
                    c['dimensions'][1],
                    fill=False,
                    edgecolor='red',
                )
            )
        ax.set_aspect('equal')
        ax.set_xlabel('x (m, forward)')
        ax.set_ylabel('y (m, left)')
        ax.legend()
        plt.show()

    return 0


if __name__ == '__main__':
    sys.exit(main())
