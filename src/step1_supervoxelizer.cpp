/**
 * @file step1_supervoxelizer.cpp
 * @brief Multi-layer supervoxel generation for individual tree segmentation.
 * @details Extracts supervoxels from input point clouds, computing centroids, 
 * normals, and adjacency graphs to support competitive region growing.
 * This is the preprocessing step for topological manifold extraction.
 */

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/supervoxel_clustering.h>

#include <fstream>
#include <iostream>
#include <map>
#include <string>
#include <vector>

// 现代 C++ 类型别名规范
using PointT = pcl::PointXYZRGBA;
using PointCloudT = pcl::PointCloud<PointT>;
using PointNT = pcl::PointNormal;
using PointCloudNT = pcl::PointCloud<PointNT>;
using SupervoxelT = pcl::Supervoxel<PointT>;

// 统一管理输出的中间文件名称
constexpr char OUT_ID_MAP[]      = "plot.sv_id.txt";
constexpr char OUT_CENTROIDS[]   = "plot.sv_centroids.pcd";
constexpr char OUT_NORMALS[]     = "plot.sv_normals.pcd";
constexpr char OUT_ADJACENCY[]   = "plot.adj.txt";
constexpr char OUT_LABELED[]     = "plot.labeled.pcd";

int main(int argc, char **argv) {
    if (argc < 4) {
        std::cerr << "Usage: " << argv[0] 
                  << " <voxel_res> <seed_res> <input_plot.pcd>\n"
                  << "Example: " << argv[0] << " 0.05 0.20 input.pcd\n";
        return EXIT_FAILURE;
    }

    const float voxel_res = std::atof(argv[1]);
    const float seed_res  = std::atof(argv[2]);
    const std::string input_file = argv[3];

    // 1. Load point cloud
    std::cout << "[INFO] Loading point cloud: " << input_file << " ...\n";
    PointCloudT::Ptr cloud(new PointCloudT);
    if (pcl::io::loadPCDFile<PointT>(input_file, *cloud) == -1) {
        std::cerr << "[ERROR] Failed to read PCD file: " << input_file << "\n";
        return EXIT_FAILURE;
    }

    // 2. Supervoxel Clustering (VCCS Algorithm)
    std::cout << "[INFO] Computing supervoxels (Voxel res: " << voxel_res 
              << "m, Seed res: " << seed_res << "m) ...\n";
              
    pcl::SupervoxelClustering<PointT> super(voxel_res, seed_res);
    super.setInputCloud(cloud);
    
    // Emphasize spatial and normal features for forest structures; ignore color.
    super.setColorImportance(0.0f);
    super.setSpatialImportance(0.4f);
    super.setNormalImportance(1.0f);

    std::map<uint32_t, SupervoxelT::Ptr> supervoxel_clusters;
    super.extract(supervoxel_clusters);
    std::cout << "[INFO] Extracted " << supervoxel_clusters.size() << " supervoxels.\n";

    // 3. Extract and export intermediate representations
    std::cout << "[INFO] Exporting centroids, normals, and mapping tables...\n";
    PointCloudT::Ptr sv_centroids(new PointCloudT);
    PointCloudNT::Ptr sv_normals(new PointCloudNT);
    
    std::ofstream id_map_file(OUT_ID_MAP);
    if (!id_map_file.is_open()) {
        std::cerr << "[ERROR] Failed to create output ID map file.\n";
        return EXIT_FAILURE;
    }
    sv_centroids->reserve(supervoxel_clusters.size());
    sv_normals->reserve(supervoxel_clusters.size());

    for (const auto& [sv_id, sv] : supervoxel_clusters) {
        // Centroid geometry
        PointT p;
        p.x = sv->centroid_.x;
        p.y = sv->centroid_.y;
        p.z = sv->centroid_.z;
        sv_centroids->push_back(p);
        
        // Centroid normals (Fix: 初始化法线点云的三维坐标)
        PointNT n;
        n.x = p.x; n.y = p.y; n.z = p.z; 
        n.normal_x = sv->normal_.normal_x;
        n.normal_y = sv->normal_.normal_y;
        n.normal_z = sv->normal_.normal_z;
        sv_normals->push_back(n);

        id_map_file << sv_id << "\n";
    }
    id_map_file.close();
    
    pcl::io::savePCDFileBinary(OUT_CENTROIDS, *sv_centroids);
    pcl::io::savePCDFileBinary(OUT_NORMALS, *sv_normals);

    // 4. Export Adjacency Graph
    std::cout << "[INFO] Exporting adjacency graph...\n";
    std::multimap<uint32_t, uint32_t> supervoxel_adjacency;
    super.getSupervoxelAdjacency(supervoxel_adjacency);

    std::ofstream adj_file(OUT_ADJACENCY);
    for (const auto& pair : supervoxel_adjacency) {
        adj_file << pair.first << " " << pair.second << "\n";
    }
    adj_file.close();

    // 5. Export Point-to-Supervoxel Mapping
    std::cout << "[INFO] Exporting labeled point cloud...\n";
    pcl::PointCloud<pcl::PointXYZL>::Ptr labeled_cloud = super.getLabeledCloud();
    pcl::io::savePCDFileBinary(OUT_LABELED, *labeled_cloud);

    std::cout << "[INFO] Step 1 completed successfully.\n";
    return EXIT_SUCCESS;
}