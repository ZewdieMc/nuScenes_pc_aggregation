#!/usr/bin/env python3
from aggregate import *

if __name__ == "__main__":
    print("Aggregating point clouds...")
    agg_pc, annotaions, pc_colors = aggregate_pc()
    print("Visualizing point cloud...")
    # visualize_pc(agg_pc)

    # moving, static = detect_moving_objects(np.vstack(agg_pc), annotaions)
    # visualize_static_and_moving_points(static, pc_colors, moving)
    draw_pc(np.vstack(agg_pc), np.vstack(pc_colors))
    # show_scene_image()