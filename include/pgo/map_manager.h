//
// This file is for the implements of the map management in Multi-Metrics Linear Least Square Lidar Odometry And Mapping (MULLS)
// By Yue Pan 
//

#ifndef _INCLUDE_MAP_MANAGER_H
#define _INCLUDE_MAP_MANAGER_H

#include <vector>
#include <queue>

#include "utility.hpp"

using namespace std;

namespace lo
{

class MapManager
{
public:
  bool update_local_map(cloudblock_Ptr local_map, cloudblock_Ptr last_target_cblock,
                        float local_map_radius = 80, int max_num_pts = 20000, int kept_vertex_num = 800,
                        float last_frame_reliable_radius = 60,
                        bool map_based_dynamic_removal_on = false,
                        std::string used_feature_type = "111110",
                        float dynamic_removal_center_radius = 30.0,
                        float dynamic_dist_thre_min = 0.3,
                        float dynamic_dist_thre_max = 3.0,
                        float near_dist_thre = 0.03,
                        bool recalculate_feature_on = false);

  bool map_based_dynamic_close_removal(cloudblock_Ptr local_map, cloudblock_Ptr last_target_cblock, std::string used_feature_type,
                                       float center_radius, float dynamic_dist_thre_min, float dynamic_dist_thre_max, float near_dist_thre);

  //keep the points meet dist ~ (near_dist_thre, dynamic_dist_thre_min) U (dynamic_dist_thre_max, +inf)
  //filter the points meet dist ~ (0, near_dist_thre] U [dynamic_dist_thre_min, dynamic_dist_thre_max]
  bool map_scan_feature_pts_distance_removal(pcTPtr feature_pts, const pcTreePtr map_kdtree, float center_radius,
                                             float dynamic_dist_thre_min = FLT_MAX, float dynamic_dist_thre_max = FLT_MAX, float near_dist_thre = 0.0);

  bool update_cloud_vectors(pcTPtr feature_pts, const pcTreePtr map_kdtree,
                            float pca_radius = 1.5, int pca_k = 20,
                            int k_min = 8, float sin_low = 0.5, float sin_high = 0.5, float min_linearity = 0.0);

  bool judge_new_submap(float &accu_tran, float &accu_rot, int &accu_frame,
                        float max_accu_tran = 30.0, float max_accu_rot = 90.0, int max_accu_frame = 150);

  //TODO add the codes related to global & local map management (for loop closure detection)

protected:
private:
};

} // namespace lo

#endif //_INCLUDE_MAP_MANAGER_H