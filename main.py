#!/usr/bin/env python3
from aggregate import aggregate_pc, visualize_pc

if __name__ == "__main__":
    print("Aggregating point clouds...")
    pc = aggregate_pc()
    print("Visualizing point cloud...")
    visualize_pc(pc)