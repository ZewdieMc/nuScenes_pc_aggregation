#!/usr/bin/env python3
from aggregate import *

if __name__ == "__main__":
    print("Aggregating point clouds...")
    agg_pc, annotaions, pc_colors = aggregate_pc()
    print("Visualizing point cloud...")

    moving, static = detect_moving_objects(np.vstack(agg_pc), annotaions)
    visualize_static_and_moving_points(static, pc_colors, moving)
    draw_pc(np.vstack(agg_pc), np.vstack(pc_colors))
    
    # Filter colors corresponding to static points
    all_points = np.vstack(agg_pc)
    all_colors = np.vstack(pc_colors)
    static_mask = np.isin(all_points, static).all(axis=1)
    static_colors = all_colors[static_mask]

    draw_pc(static, static_colors)