# High-Performance 2D Spatial Partitioning Engine

A lightweight, real-time spatial indexing and collision detection engine built in **Modern C++17** and **Raylib**. Demonstrates spatial partitioning using a custom recursive **Quadtree** compared against an (N^2)$ brute-force baseline.

## Performance Benchmark (2,000 Entities @ 1200x800)

| Metric | Brute Force ((N^2)$) | Quadtree Partitioning ((N \log N)$) | Reduction |
| :--- | :---: | :---: | :---: |
| **Collision Narrowphase Checks** | **1,999,000** | **~300 - 800** | **> 99.9%** |
| **Engine Execution Time** | ~2.70 ms | ~1.68 ms | ~38% faster frame budget |
| **Frame Rate** | 60-66 FPS | 71-120 FPS | Stable target refresh |

## Key Architecture & Features
- **Hierarchical Spatial Subdivision:** Dynamic recursive Quadtree with depth limiting and point thresholding.
- **AABB Fast Pruning:** Axis-Aligned Bounding Box tests to eliminate non-overlapping candidates before narrow-phase Euclidean checks.
- **Memory Safety & RAII:** Uses smart pointers (std::unique_ptr) and bounded leaf nodes to prevent dynamic memory leaks in high-frequency frame loops.

## Build Instructions (Windows / MSYS2)
\\\ash
g++ -O3 -std=c++17 main.cpp -lraylib -lopengl32 -lgdi32 -lwinmm -o engine.exe
./engine.exe
\\\
- **SPACE**: Toggle between Brute Force and Quadtree mode.
- **T**: Toggle boundary grid visualization.
