\# TreeSeg-Manifold: Individual Tree Segmentation Integrating Topological Manifolds and Biological Priors

\*(融合拓扑流形与生物学先验的点云单木分割框架)\*



!\[C++](https://img.shields.io/badge/C++-14%2F17-blue.svg)

!\[PCL](https://img.shields.io/badge/PCL-1.10+-lightgrey.svg)

!\[License](https://img.shields.io/badge/License-MIT-green.svg)



\## 📖 Overview (项目简介)

This repository contains the official C++ implementation of the Individual Tree Segmentation (ITS) framework. 

针对复杂林分下地基激光雷达点云（TLS）单木分割面临的参数敏感、边界判定刚性与泛化能力不足等问题，本项目提出了一种融合拓扑流形与生物学先验的自动化分割框架。



本算法在冠层深度交错与点云残缺场景下具有极强的鲁棒性，平均 F-score 达 0.965，有效解决了冠层交织区的“归属漂移”难题。



\## ✨ Key Features (核心亮点)

\* \*\*Bottom-up Supervoxelization (自底向上的超体素构建)\*\*: 以多层局部超体素为基元，大幅降低对人工参数调优的依赖。

\* \*\*Adaptive Skeleton Reconstruction (骨架自适应重构)\*\*: 利用法向量连续性与几何显著性提取主干，结合异速生长（Metabolic Allometric）规律实现重构。

\* \*\*Competitive Region Growing (多源竞争性精细分割)\*\*: 在冠层分割阶段模拟\*\*最小水力阻抗原则\*\*，引入法线正交性约束，实现高精度的图生长（Graph Growing）分割。



\## 📂 Repository Structure (项目结构)

```text

TreeSeg-Manifold/

├── CMakeLists.txt            # CMake build configuration

├── README.md                 # Project documentation

├── LICENSE                   # MIT License

├── include/                  # Header files (treeseg dependencies)

│   ├── treeseg.h

│   └── treeseg\_pointtype.h

├── src/                      # Core algorithm implementation

│   ├── treeseg.cpp           # Core point cloud math and processing library

│   ├── downsample.cpp        # Voxel grid downsampling

│   ├── getdtmslice.cpp       # DTM and slice extraction

│   ├── findstems.cpp         # Stem detection \& cylinder fitting

│   ├── segmentstem.cpp       # Stem skeleton extraction

│   ├── step1\_supervoxelizer.cpp # \[Core] Multi-layer supervoxel generation

│   └── step2\_graph\_grower.cpp   # \[Core] Topological manifold competitive growing

├── scripts/

│   └── run\_pipeline.sh       # Automated pipeline execution script

└── data/                     # Sample data for testing

🛠️ Dependencies (依赖环境)

OS: Linux (Ubuntu 18.04 / 20.04 recommended) or macOS / Windows with WSL2.



C++ Standard: C++ 14 or 17



PCL (Point Cloud Library): >= 1.10



Armadillo: >= 9.8



CMake: >= 3.10



🚀 Build \& Run (编译与运行)

1\. Build (编译项目)

Bash

mkdir build \&\& cd build

cmake ..

make -j4

2\. Quick Test (快速运行测试)

We provide an automated shell script to run the entire pipeline from downsampling to colored point cloud rendering.

(假设您的测试数据为 data/sample\_plot.tile.0.pcd，坐标边界为 -4 17 -16 5)



Bash

\# Return to the project root directory

cd ..

\# Run the pipeline script: bash scripts/run\_pipeline.sh <Prefix> <Coords>

bash scripts/run\_pipeline.sh data/sample\_plot "-4 17 -16 5"

3\. Output (结果输出)

The final colored segmented point clouds will be exported as Tree\_\*.pcd files. You can drag them into CloudCompare to visualize the distinct colors representing individual trees.

