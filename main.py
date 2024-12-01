#!/usr/bin/env python3
from aggregate import aggregate_pc, visualize_pc, draw_pc, visualize_moving_objects
import numpy as np

if __name__ == "__main__":
    print("Aggregating point clouds...")
    pc = aggregate_pc()
    print("Visualizing point cloud...")
    visualize_pc(pc)

    draw_pc(np.vstack(pc))

    visualize_moving_objects(np.vstack(pc))