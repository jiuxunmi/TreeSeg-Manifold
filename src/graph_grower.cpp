#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <Eigen/Dense>
#include <vector>
#include <unordered_map>
#include <unordered_set>
#include <queue>
#include <fstream>
#include <cmath>
#include <iostream>
#include <string>

using PointT       = pcl::PointXYZRGBA;
using PointNT      = pcl::PointNormal;
using PointLT      = pcl::PointXYZL;
using PointCloudT  = pcl::PointCloud<PointT>;
using PointCloudNT = pcl::PointCloud<PointNT>;
using PointCloudLT = pcl::PointCloud<PointLT>;

constexpr float MAX_GROWTH_DIST   = 0.4f;
constexpr float ANGLE_PENALTY_FAC = 4.0f;
constexpr float COST_INFINITY     = 1e9f;
constexpr float MAX_FILL_DIST     = 100.0f;

struct PathNode {
    uint32_t sv_id;
    int tree_id;
    float cost;

    PathNode(uint32_t id = 0, int t_id = 0, float c = 0.0f) 
        : sv_id(id), tree_id(t_id), cost(c) {}

    bool operator>(const PathNode& other) const { 
        return cost > other.cost; 
    }
};

void HSVtoRGB(float h, float s, float v, uint8_t& r, uint8_t& g, uint8_t& b) {
    int i = static_cast<int>(h * 6);
    float f = h * 6 - i;
    float p = v * (1 - s);
    float q = v * (1 - f * s);
    float t = v * (1 - (1 - f) * s);
    
    float r_f = 0.0f, g_f = 0.0f, b_f = 0.0f;
    switch (i % 6) {
        case 0: r_f = v; g_f = t; b_f = p; break;
        case 1: r_f = q; g_f = v; b_f = p; break;
        case 2: r_f = p; g_f = v; b_f = t; break;
        case 3: r_f = p; g_f = q; b_f = v; break;
        case 4: r_f = t; g_f = p; b_f = v; break;
        case 5: r_f = v; g_f = p; b_f = q; break;
    }
    r = static_cast<uint8_t>(r_f * 255);
    g = static_cast<uint8_t>(g_f * 255);
    b = static_cast<uint8_t>(b_f * 255);
}

void getTreeColor(int tree_id, uint8_t& r, uint8_t& g, uint8_t& b) {
    const float golden_ratio_conjugate = 0.618033988749895f;
    float h = std::fmod((tree_id * golden_ratio_conjugate), 1.0f); 
    HSVtoRGB(h, 0.85f, 0.95f, r, g, b);
}

float computeGrowthCost(const PointT& parent_pt, const PointNT& parent_norm,
                        const PointT& child_pt, const PointNT& child_norm) {
    Eigen::Vector3f p_vec(parent_pt.x, parent_pt.y, parent_pt.z);
    Eigen::Vector3f c_vec(child_pt.x, child_pt.y, child_pt.z);
    Eigen::Vector3f link_vec = c_vec - p_vec;
    
    float dist = link_vec.norm();
    if (dist > MAX_GROWTH_DIST) {
        return COST_INFINITY;
    }

    link_vec.normalize();
    Eigen::Vector3f child_dir(child_norm.normal_x, child_norm.normal_y, child_norm.normal_z);
    
    float dot = std::abs(link_vec.dot(child_dir)); 
    float angle_penalty = ANGLE_PENALTY_FAC * (1.0f - dot);

    return dist * (1.0f + angle_penalty);
}

int main(int argc, char **argv) {
    if (argc < 2) {
        std::cerr << "Usage: " << argv[0] << " <stem_1.pcd> [stem_2.pcd ...]\n";
        return EXIT_FAILURE;
    }

    PointCloudT::Ptr sv_centroids(new PointCloudT);
    if (pcl::io::loadPCDFile<PointT>("plot.sv_centroids.pcd", *sv_centroids) == -1) {
        return EXIT_FAILURE;
    }
    
    PointCloudNT::Ptr sv_normals(new PointCloudNT);
    pcl::io::loadPCDFile<PointNT>("plot.sv_normals.pcd", *sv_normals);

    std::vector<uint32_t> index_to_id;
    std::unordered_map<uint32_t, int> id_to_index;
    
    std::ifstream id_file("plot.sv_id.txt");
    uint32_t tmp_id;
    int idx_counter = 0;
    while (id_file >> tmp_id) {
        index_to_id.push_back(tmp_id);
        id_to_index[tmp_id] = idx_counter++;
    }
    id_file.close();

    std::multimap<uint32_t, uint32_t> adj_graph;
    std::ifstream adj_file("plot.adj.txt");
    uint32_t u, v;
    while (adj_file >> u >> v) {
        adj_graph.emplace(u, v);
    }
    adj_file.close();

    std::unordered_map<uint32_t, int> sv_ownership;
    std::unordered_map<uint32_t, float> sv_min_cost; 
    
    for (const auto id : index_to_id) {
        sv_min_cost[id] = COST_INFINITY;
    }

    std::priority_queue<PathNode, std::vector<PathNode>, std::greater<PathNode>> pq;
    pcl::KdTreeFLANN<PointT> kdtree;
    kdtree.setInputCloud(sv_centroids);

    for (int i = 1; i < argc; ++i) {
        const int tree_id = i; 
        PointCloudT::Ptr stem(new PointCloudT);
        if (pcl::io::loadPCDFile<PointT>(argv[i], *stem) == -1) continue;

        for (size_t k = 0; k < stem->size(); k += 5) { 
            std::vector<int> pointIdx(1);
            std::vector<float> pointDist(1);
            if (kdtree.nearestKSearch(stem->points[k], 1, pointIdx, pointDist) > 0) {
                uint32_t sv_id = index_to_id[pointIdx[0]];
                if (sv_min_cost[sv_id] > 0.0f) {
                    sv_min_cost[sv_id] = 0.0f;
                    sv_ownership[sv_id] = tree_id;
                    pq.emplace(sv_id, tree_id, 0.0f);
                }
            }
        }
    }

    while (!pq.empty()) {
        PathNode curr = pq.top();
        pq.pop();

        if (curr.cost > sv_min_cost[curr.sv_id]) continue;

        auto range = adj_graph.equal_range(curr.sv_id);
        int curr_idx = id_to_index[curr.sv_id];

        for (auto it = range.first; it != range.second; ++it) {
            uint32_t neighbor_id = it->second;
            
            if (id_to_index.find(neighbor_id) == id_to_index.end()) continue;
            int neighbor_idx = id_to_index[neighbor_id];

            float step_cost = computeGrowthCost(
                sv_centroids->points[curr_idx], sv_normals->points[curr_idx],
                sv_centroids->points[neighbor_idx], sv_normals->points[neighbor_idx]
            );

            if (step_cost >= COST_INFINITY) continue;

            float new_cost = curr.cost + step_cost;
            if (new_cost < sv_min_cost[neighbor_id]) {
                sv_min_cost[neighbor_id] = new_cost;
                sv_ownership[neighbor_id] = curr.tree_id; 
                pq.emplace(neighbor_id, curr.tree_id, new_cost);
            }
        }
    }

    std::unordered_set<uint32_t> unassigned_sv_ids;
    for (const auto id : index_to_id) {
        if (sv_ownership.find(id) == sv_ownership.end()) {
            unassigned_sv_ids.insert(id);
        }
    }
    
    if (!unassigned_sv_ids.empty()) {
        PointCloudT::Ptr assigned_centroids(new PointCloudT);
        std::vector<int> assigned_tree_ids;
        assigned_centroids->reserve(sv_ownership.size());
        assigned_tree_ids.reserve(sv_ownership.size());
        
        for (const auto& [id, t_id] : sv_ownership) {
            assigned_centroids->push_back(sv_centroids->points[id_to_index[id]]);
            assigned_tree_ids.push_back(t_id);
        }
        
        if (!assigned_centroids->empty()) {
            pcl::KdTreeFLANN<PointT> assigned_kdtree;
            assigned_kdtree.setInputCloud(assigned_centroids);
            
            for (const auto sv_id : unassigned_sv_ids) {
                PointT query_pt = sv_centroids->points[id_to_index[sv_id]];
                std::vector<int> k_indices(1);
                std::vector<float> k_distances(1);
                
                if (assigned_kdtree.nearestKSearch(query_pt, 1, k_indices, k_distances) > 0) {
                    if (k_distances[0] < MAX_FILL_DIST) { 
                        sv_ownership[sv_id] = assigned_tree_ids[k_indices[0]];
                    }
                }
            }
        }
    }

    PointCloudLT::Ptr labeled_cloud(new PointCloudLT);
    if (pcl::io::loadPCDFile<PointLT>("plot.labeled.pcd", *labeled_cloud) == -1) {
        return EXIT_FAILURE;
    }

    std::unordered_map<int, PointCloudT::Ptr> final_trees;
    for (const auto& pt : labeled_cloud->points) {
        uint32_t sv_id = pt.label; 
        
        auto it = sv_ownership.find(sv_id);
        if (it != sv_ownership.end() && it->second > 0) {
            int tree_id = it->second;
            
            if (final_trees.find(tree_id) == final_trees.end()) {
                final_trees[tree_id] = pcl::make_shared<PointCloudT>();
            }
            
            uint8_t r, g, b;
            getTreeColor(tree_id, r, g, b);

            PointT p_out;
            p_out.x = pt.x; p_out.y = pt.y; p_out.z = pt.z;
            p_out.r = r; p_out.g = g; p_out.b = b; p_out.a = 255;
            
            final_trees[tree_id]->push_back(p_out);
        }
    }

    pcl::PCDWriter writer;
    for (const auto& [tree_id, cloud_ptr] : final_trees) {
        std::string filename = "Tree_" + std::to_string(tree_id) + ".pcd";
        writer.writeBinary(filename, *cloud_ptr);
    }

    return EXIT_SUCCESS;
}