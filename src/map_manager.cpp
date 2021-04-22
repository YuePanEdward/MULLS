//
// This file is for the map related operations for Lidar Odometry and Lidar SLAM
// Compulsory Dependent 3rd Libs: PCL (>1.7), glog, gflags
// Author: Yue Pan @ ETH Zurich
//

#include "map_manager.h"
#include "cfilter.hpp"
#include "utility.hpp"
#include "pca.hpp"

#include <pcl/kdtree/kdtree_flann.h>
#include <boost/make_shared.hpp>

namespace lo
{

bool MapManager::update_local_map(cloudblock_Ptr local_map, cloudblock_Ptr last_target_cblock,
                                  float local_map_radius, int max_num_pts, int kept_vertex_num, float last_frame_reliable_radius,
                                  bool map_based_dynamic_removal_on, std::string used_feature_type,
                                  float dynamic_removal_center_radius, float dynamic_dist_thre_min,
                                  float dynamic_dist_thre_max, float near_dist_thre, bool recalculate_feature_on)
{
    std::chrono::steady_clock::time_point tic = std::chrono::steady_clock::now();

    //float near_dist_thre = 0.03;
    CFilter<Point_T> cf;
    Eigen::Matrix4d tran_target_map; //from the last local_map to the target frame
    tran_target_map = last_target_cblock->pose_lo.inverse() * local_map->pose_lo;
   

    //to avoid building kd-tree twice , do the map based filtering at first before the transforming
    last_target_cblock->transform_feature(tran_target_map.inverse(), true, false);
    
    dynamic_dist_thre_max = max_(dynamic_dist_thre_max, dynamic_dist_thre_min + 0.1);
    LOG(INFO) << "Map based filtering range(m): (0, " << near_dist_thre << "] U [" << dynamic_dist_thre_min << "," << dynamic_dist_thre_max << "]";

    if (map_based_dynamic_removal_on && local_map->feature_point_num > max_num_pts / 5)
    {
        map_based_dynamic_close_removal(local_map, last_target_cblock, used_feature_type, dynamic_removal_center_radius,
                                        dynamic_dist_thre_min, dynamic_dist_thre_max, near_dist_thre);

        LOG(INFO) << "Feature point number of last frame after dynamic removal: Pillar: [" << last_target_cblock->pc_pillar->points.size() << " | "
                  << last_target_cblock->pc_pillar_down->points.size() << "] Beam: [" << last_target_cblock->pc_beam->points.size() << " | "
                  << last_target_cblock->pc_beam_down->points.size() << "] Facade: [" << last_target_cblock->pc_facade->points.size() << " | "
                  << last_target_cblock->pc_facade_down->points.size() << "] Roof: [" << last_target_cblock->pc_roof->points.size() << " | "
                  << last_target_cblock->pc_roof_down->points.size() << "] Vertex: [" << last_target_cblock->pc_vertex->points.size() << "].";
    }

    std::chrono::steady_clock::time_point toc_1 = std::chrono::steady_clock::now();

    // float keypoint_close_dist_thre = 8.0;
    // cf.dist_filter(last_target_cblock->pc_vertex, keypoint_close_dist_thre, false); // get more points that are far away from the center of the scanner
    local_map->append_feature(*last_target_cblock, true, used_feature_type);

    //we only use the undown (regarded as target) feature points for local map
    local_map->transform_feature(tran_target_map, false);
    local_map->pose_lo = last_target_cblock->pose_lo;
    local_map->pose_gt = last_target_cblock->pose_gt;

    //keep only the points in the sphere
    cf.dist_filter(local_map->pc_ground, local_map_radius);
    cf.dist_filter(local_map->pc_facade, local_map_radius);
    cf.dist_filter(local_map->pc_pillar, local_map_radius);
    cf.dist_filter(local_map->pc_beam, local_map_radius);
    cf.dist_filter(local_map->pc_roof, local_map_radius);
    cf.dist_filter(local_map->pc_vertex, local_map_radius);

    local_map->feature_point_num = local_map->pc_ground->points.size() + local_map->pc_facade->points.size() +
                                   local_map->pc_roof->points.size() + local_map->pc_pillar->points.size() +
                                   local_map->pc_beam->points.size();

    int rand_down_rate = (int)(local_map->feature_point_num / max_num_pts + 1);
    int current_pts_count = local_map->feature_point_num;
    int kept_ground_num = (int)(1.0 * max_num_pts / current_pts_count * local_map->pc_ground->points.size() + 1);
    int kept_facade_num = (int)(1.0 * max_num_pts / current_pts_count * local_map->pc_facade->points.size() + 1);
    int kept_roof_num = (int)(1.0 * max_num_pts / current_pts_count * local_map->pc_roof->points.size() + 1);
    int kept_pillar_num = (int)(1.0 * max_num_pts / current_pts_count * local_map->pc_pillar->points.size() + 1);
    int kept_beam_num = (int)(1.0 * max_num_pts / current_pts_count * local_map->pc_beam->points.size() + 1);

    cf.random_downsample_pcl(local_map->pc_ground, kept_ground_num);
    cf.random_downsample_pcl(local_map->pc_facade, kept_facade_num);
    cf.random_downsample_pcl(local_map->pc_roof, kept_roof_num);
    cf.random_downsample_pcl(local_map->pc_pillar, kept_pillar_num);
    cf.random_downsample_pcl(local_map->pc_beam, kept_beam_num);
    cf.random_downsample_pcl(local_map->pc_vertex, kept_vertex_num);

    //calculate bbx (local)
    local_map->merge_feature_points(local_map->pc_raw, false);
    cf.get_cloud_bbx(local_map->pc_raw, local_map->local_bound);

    //calculate bbx (global)
    pcl::transformPointCloud(*local_map->pc_raw, *local_map->pc_raw_w, local_map->pose_lo);
    cf.get_cloud_bbx(local_map->pc_raw_w, local_map->bound);

    std::chrono::steady_clock::time_point toc_2 = std::chrono::steady_clock::now();

    if (recalculate_feature_on) //Re-calculation primary vector and filter some of the points in the local map at the same time
    {
        std::chrono::steady_clock::time_point tic_0 = std::chrono::steady_clock::now();
        float pca_radius = 1.8;
        int pca_max_k = 20;
        int pca_min_k = 6;
        float sin_high_pillar = 0.80; //55 deg
        float sin_low_beam = 0.25;    //15 deg
        float min_linearity = 0.65;

        //pillar
        if (used_feature_type[1] == '1')
            update_cloud_vectors(local_map->pc_pillar, local_map->tree_pillar, pca_radius, pca_max_k, pca_min_k,
                                 0.0, sin_high_pillar, min_linearity);
        //beam
        if (used_feature_type[3] == '1')
            update_cloud_vectors(local_map->pc_beam, local_map->tree_beam, pca_radius, pca_max_k, pca_min_k,
                                 sin_low_beam, 1.0, min_linearity);
        std::chrono::steady_clock::time_point toc_0 = std::chrono::steady_clock::now();
        std::chrono::duration<double> time_update_local_map_linear_pts = std::chrono::duration_cast<std::chrono::duration<double>>(toc_0 - tic_0);
        LOG(INFO) << "Update map pillar and beam feature points done in [" << time_update_local_map_linear_pts.count() * 1000.0 << "] ms.\n";
    }

    std::chrono::steady_clock::time_point toc_3 = std::chrono::steady_clock::now();

    local_map->feature_point_num = local_map->pc_ground->points.size() + local_map->pc_facade->points.size() +
                                   local_map->pc_roof->points.size() + local_map->pc_pillar->points.size() +
                                   local_map->pc_beam->points.size();

    local_map->free_tree();
    local_map->free_raw_cloud();

    std::chrono::steady_clock::time_point toc = std::chrono::steady_clock::now();
    std::chrono::duration<double> time_update_local_map = std::chrono::duration_cast<std::chrono::duration<double>>(toc - tic);

    LOG(INFO) << "Feature point number of the local map: G: [" << local_map->pc_ground->points.size() << "] P: ["
              << local_map->pc_pillar->points.size() << "] B: [" << local_map->pc_beam->points.size() << "] F: ["
              << local_map->pc_facade->points.size() << "] R: [" << local_map->pc_roof->points.size() << "] V: ["
              << local_map->pc_vertex->points.size() << "].";

    LOG(INFO) << "Update local map ([" << local_map->feature_point_num << "] points at present) done in [" << time_update_local_map.count() * 1000.0 << "] ms.\n";
    return true;
}

//the map consturction and updateing strategy is now the most important part for our application
//and also for loop closure and pose graph optimization

//map based active object removal
//for each non-boundary unground points of current frame, check its nearest neighbor in the local map, if the distance
//to the nearest neighbor is larger than a threshold, then this point would be regarded as a part of an active object
//we will filter the points whose distance is (0, near_dist_thre] U [dynamic_dist_thre_min, dynamic_dist_thre_max] to its nearest neighbor in the local map
bool MapManager::map_based_dynamic_close_removal(cloudblock_Ptr local_map, cloudblock_Ptr last_target_cblock, std::string used_feature_type,
                                                 float center_radius, float dynamic_dist_thre_min, float dynamic_dist_thre_max, float near_dist_thre)
//feature_pts_down is used to append into the local map [dynamic_dist_thre used]
//feature_pts is used to do the next registration (act as target point cloud) [dynamic_dist_thre_down used]
{
#if 0       
        //CFilter<Point_T> cf;

        //kd-tree not built yet
        //build local map's unground kdtree
        //do not need to build the tree multiple time if it's available
        // pcTreePtr tree_pillar(new pcTree());
        // pcTreePtr tree_beam(new pcTree());
        // pcTreePtr tree_facade(new pcTree());
        // pcTreePtr tree_roof(new pcTree());

        // pcTPtr close_pillar(new pcT());
        // pcTPtr close_beam(new pcT());
        // pcTPtr close_facade(new pcT());
        // pcTPtr close_roof(new pcT());

        // double extended_dist = 4.0;

        //save the time for buidling the k-d tree
        // cf.dist_filter(local_map->pc_pillar, close_pillar, center_radius + extended_dist);
        // cf.dist_filter(local_map->pc_beam, close_beam, center_radius + extended_dist);
        // cf.dist_filter(local_map->pc_facade, close_facade, center_radius + extended_dist);
        // cf.dist_filter(local_map->pc_roof, close_roof, center_radius + extended_dist);

        // if (close_pillar->points.size() > 0)
        //     tree_pillar->setInputCloud(close_pillar);
        // if (close_beam->points.size() > 0)
        //     tree_beam->setInputCloud(close_beam);
        // if (close_facade->points.size() > 0)
        //     tree_facade->setInputCloud(close_facade);
        // if (close_roof->points.size() > 0)
        //     tree_roof->setInputCloud(close_roof);
#endif
    //kd-tree already built
    //suppose the relative velocity of self and the coming vehcile is 180 km/m (50 m/s), then the distance would be 5m /ms, you should set the threshold according to the velocity of the vehicle
    //dynamic_dist_thre_max = 3.0;

#pragma omp parallel sections
    {
#pragma omp section
        {
            //dynamic + close points
            if (used_feature_type[1] == '1')
                map_scan_feature_pts_distance_removal(last_target_cblock->pc_pillar_down, local_map->tree_pillar, center_radius, dynamic_dist_thre_min, dynamic_dist_thre_max, near_dist_thre);
            if (used_feature_type[3] == '1')
                map_scan_feature_pts_distance_removal(last_target_cblock->pc_beam_down, local_map->tree_beam, center_radius, dynamic_dist_thre_min, dynamic_dist_thre_max, near_dist_thre);
        }
#pragma omp section
        {
            //dynamic + close points
            if (used_feature_type[2] == '1')
                map_scan_feature_pts_distance_removal(last_target_cblock->pc_facade_down, local_map->tree_facade, center_radius, dynamic_dist_thre_min, dynamic_dist_thre_max, near_dist_thre);
        }
        // #pragma omp section
        //         {
        //             //close points
        //             if (used_feature_type[0] == '1' && near_dist_thre > 0.0)
        //                 map_scan_feature_pts_distance_removal(last_target_cblock->pc_ground_down, local_map->tree_ground, center_radius, FLT_MAX, FLT_MAX, near_dist_thre);
        //             if (used_feature_type[4] == '1' && near_dist_thre > 0.0)
        //                 map_scan_feature_pts_distance_removal(last_target_cblock->pc_roof_down, local_map->tree_roof, center_radius, FLT_MAX, FLT_MAX, near_dist_thre);
        //         }
    }

    return true;
}

// filter (0, near_dist_thre) U (dynamic_dist_thre_min, dynamic_dist_thre_max)
// keep (near_dist_thre, dynamic_dist_thre_min) U (dynamic_dist_thre_max, +inf)
bool MapManager::map_scan_feature_pts_distance_removal(pcTPtr feature_pts, const pcTreePtr map_kdtree, float center_radius,
                                                       float dynamic_dist_thre_min, float dynamic_dist_thre_max, float near_dist_thre)
{
    if (feature_pts->points.size() <= 10)
        return false;

    pcTPtr feature_pts_temp(new pcT);
    int i;

    //#pragma omp parallel for private(i) //Multi-thread

    std::vector<float> distances_square;
    std::vector<int> search_indices;
    for (i = 0; i < feature_pts->points.size(); i++)
    {
        if (feature_pts->points[i].x * feature_pts->points[i].x +
                feature_pts->points[i].y * feature_pts->points[i].y >
            center_radius * center_radius)
            feature_pts_temp->points.push_back(feature_pts->points[i]);
        else
        {
            map_kdtree->nearestKSearch(feature_pts->points[i], 1, search_indices, distances_square);                                                                                                                   //search nearest neighbor
            if ((distances_square[0] > near_dist_thre * near_dist_thre && distances_square[0] < dynamic_dist_thre_min * dynamic_dist_thre_min) || distances_square[0] > dynamic_dist_thre_max * dynamic_dist_thre_max) // the distance should not be too close to keep the map more uniform
                feature_pts_temp->points.push_back(feature_pts->points[i]);
            // else
            //     LOG(INFO) << "Filter out the point, dist [" << std::sqrt(distances_square[0]) << "].";

            std::vector<float>().swap(distances_square);
            std::vector<int>().swap(search_indices);
        }
    }
    feature_pts_temp->points.swap(feature_pts->points);

    return true;
}

bool MapManager::update_cloud_vectors(pcTPtr feature_pts, const pcTreePtr map_kdtree,
                                      float pca_radius, int pca_k,
                                      int k_min, float sin_low, float sin_high, float min_linearity)
{
    if (feature_pts->size() == 0)
        return false;

    int pt_num_before_process = feature_pts->points.size();

    PrincipleComponentAnalysis<Point_T> pca_estimator;

    std::vector<pca_feature_t> pca_features;

    pcTPtr temp_pts(new pcT());

    pca_estimator.get_pc_pca_feature(feature_pts, pca_features, pca_radius, pca_k, k_min);

    for (int i = 0; i < feature_pts->points.size(); i++)
    {
        if (pca_features[i].pt_num >= k_min && pca_features[i].linear_2 > min_linearity)
        {
            pca_estimator.assign_normal(feature_pts->points[i], pca_features[i], false);
            if (std::abs(pca_features[i].vectors.principalDirection.z()) > sin_high || std::abs(pca_features[i].vectors.principalDirection.z()) < sin_low)
            {
                feature_pts->points[i].curvature = pca_features[i].linear_2; //use the lineraity to overwrite the curvature property (originally timestamp) //TODO: fix
                temp_pts->points.push_back(feature_pts->points[i]);
            }
        }
    }
    feature_pts->points.swap(temp_pts->points);

    //LOG(INFO) << "# point before = " << pt_num_before_process << " , # point after = " << feature_pts->points.size();

    return true;
}

// Divide the strip into several submaps according to multiple rules (judge if a new submap would be created)
// Rules: consecutive frame number, accumulated translation (using), accumilated heading angle (using) ...
bool MapManager::judge_new_submap(float &accu_tran, float &accu_rot, int &accu_frame,
                                  float max_accu_tran, float max_accu_rot, int max_accu_frame)
{
    // LOG(INFO) << "Submap division criterion is: \n"
    //           << "1. Frame Number <= " << max_accu_frame
    //           << " , 2. Translation <= " << max_accu_tran
    //           << "m , 3. Rotation <= " << max_accu_rot << " degree.";

    if (accu_tran > max_accu_tran || accu_rot > max_accu_rot || accu_frame > max_accu_frame)
    {
        //recount from begining
        accu_tran = 0.0;
        accu_rot = 0.0;
        accu_frame = 0;
        return true;
    }
    else
        return false;
}

// Get the heading angle change within one frame (10 imu data) using Trapezoidal integral of imu heading angular velocity
// Input: vector of imu_datas within a frame and the sample time of imu in second
// Output: the heading changing
// float MapManager::get_heading(std::vector<imu_info_t> &imu_datas, float delta_time_second)
// {

//     float result = 0;

//     for (int i = 0; i < imu_datas.size() - 1; i++)
//     {
//         //result += (180.0 / 3.14159) * 0.5 * (imu_datas[i].wz + imu_datas[i + 1].wz) * delta_time_second;
//         result += 0.5 * (imu_datas[i].wz + imu_datas[i + 1].wz) * delta_time_second;
//     }
//     return fabs(result);
// }

 //only use the points that are more close to the lidar for target points since they are more accurate
    //(experiment proves that this is not ture --> so set the last_frame_reliable_radius as a large number is a better idea)
    //not used now
    // cf.dist_filter(last_target_cblock->pc_ground_down, last_frame_reliable_radius);
    // cf.dist_filter(last_target_cblock->pc_facade_down, last_frame_reliable_radius);
    // cf.dist_filter(last_target_cblock->pc_roof_down, last_frame_reliable_radius);
    // cf.dist_filter(last_target_cblock->pc_pillar_down, last_frame_reliable_radius);
    // cf.dist_filter(last_target_cblock->pc_beam_down, last_frame_reliable_radius);

} // namespace lo
