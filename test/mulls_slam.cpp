//
// This file is for the general implements of Multi-Metrics Linear Least Square Lidar SLAM (MULLS)
// Compulsory Dependent 3rd Libs: PCL (>1.7), glog, gflags
// By Yue Pan

#include "dataio.hpp"
#include "cfilter.hpp"
#include "cregistration.hpp"
#include "utility.hpp"
#include "map_viewer.h"
#include "map_manager.h"
#include "odom_error_compute.h"
#include "common_nav.h"
#include "build_pose_graph.h"
#include "graph_optimizer.h"

#include <glog/logging.h>
#include <gflags/gflags.h>

using namespace lo;
//static void CheckCudaErrorAux(const char *, unsigned, const char *, cudaError_t);
//#define CUDA_CHECK(value) CheckCudaErrorAux(__FILE__, __LINE__, #value, value)
//Parameter Lists: //TODO: delete those deprecated parameters (most of the parameters listed below can be used as default in practice, you may just fix them)
//--------------------------------------------------------------------------------------------------------------------------------------------------------------
//GFLAG Template: DEFINE_TYPE(Flag_variable_name, default_value, "Comments")
//data path
DEFINE_string(point_cloud_folder, "", "folder containing the point cloud of each frame");
DEFINE_string(pc_format, ".pcd", "input point cloud format (select from .pcd, .ply, .txt, .las, .h5 ...");
DEFINE_string(output_adjacent_lo_pose_file_path, "", "the file for saving the adjacent transformation of lidar odometry");
DEFINE_string(gt_body_pose_file_path, "", "optional: the file containing the ground truth pose in body coor. sys. (as the format of transformation matrix)");
DEFINE_string(calib_file_path, "", "optional: the file containing the calibration matrix (as the format of transformation matrix)");
DEFINE_string(output_gt_lidar_pose_file_path, "", "optional: the file for saving the groundtruth pose in the lidar coor. sys.");
DEFINE_string(output_lo_body_pose_file_path, "", "optional: the file for saving the pose estimated by the lidar odometry in the body coor. sys.");
DEFINE_string(output_lo_lidar_pose_file_path, "", "optional: the file for saving the pose estimated by the lidar odometry in the lidar coor. sys.");
DEFINE_string(output_map_point_cloud_folder_path, "", "optional: the folder for saving the transformed point clouds in world coordinate system");
DEFINE_string(lo_lidar_pose_point_cloud, "", "optional: save the lidar odometry trajectory as point cloud");
DEFINE_string(gt_lidar_pose_point_cloud, "", "optional: save the ground truth trajectory as point cloud");
DEFINE_string(timing_report_file, "", "optional: the path of the file for recording the consuming time detail of the program");
DEFINE_bool(gt_in_lidar_frame, false, "whether the ground truth pose is provided in the lidar frame or the body/camera frame");
DEFINE_bool(gt_oxts_format, false, "is the ground truth pose in oxts (index ts tran quat) format or the transformation matrix format");
//used frame
DEFINE_int32(frame_num_begin, 0, "begin from this frame (file sequence in the folder)");
DEFINE_int32(frame_num_end, 99999, "end at this frame (file sequence in the folder)");
DEFINE_int32(frame_step, 1, "use one in ${frame_step} frames");
//map related
DEFINE_bool(write_out_map_on, false, "output map point clouds or not");
DEFINE_bool(write_out_gt_map_on, false, "output map point clouds generated from the gnssins pose or not");
DEFINE_bool(write_map_each_frame, false, "output each frame's point cloud in map coordinate system");
DEFINE_int32(map_downrate_output, 5, "downsampling rate for output map point cloud");
DEFINE_bool(map_filter_on, false, "clean the map point cloud before output");
DEFINE_bool(apply_dist_filter, false, "Use only the points inside a distance range or not");
DEFINE_double(min_dist_used, 1.0, "only the points whose distance to the scanner is larger than this value would be used for scan matching (m)");
DEFINE_double(max_dist_used, 120.0, "only the points whose distance to the scanner is smaller than this value would be used for scan matching (m)");
DEFINE_double(min_dist_mapping, 2.0, "only the points whose distance to the scanner is larger than this value would be used for map merging (m)");
DEFINE_double(max_dist_mapping, 60.0, "only the points whose distance to the scanner is smaller than this value would be used for map merging (m)");
//viusalization related
DEFINE_bool(real_time_viewer_on, false, "launch real time viewer or not");
DEFINE_int32(screen_width, 1920, "monitor horizontal resolution (pixel)");
DEFINE_int32(screen_height, 1080, "monitor vertical resolution (pixel)");
DEFINE_double(vis_intensity_scale, 256.0, "max intensity value of your data");
DEFINE_int32(vis_map_history_down_rate, 300, "downsampling rate of the map point cloud kept in the memory");
DEFINE_int32(vis_map_history_keep_frame_num, 150, "frame number of the dense map that would be kept in the memory");
DEFINE_int32(vis_initial_color_type, 0, "map viewer's rendering color style: (0: single color & semantic mask, 1: frame-wise, 2: height, 3: intensity)");
DEFINE_double(laser_vis_size, 0.5, "size of the laser on the map viewer");
DEFINE_bool(vis_pause_at_loop_closure, false, "the visualizer would pause when a new loop closure is cosntructed");
DEFINE_bool(show_range_image, false, "display the range image or not in realtime");
DEFINE_bool(show_bev_image, false, "display the bev image or not in realtime");
//lidar odometry related
DEFINE_bool(scan_to_scan_module_on, false, "apply scan-to-scan registration or just scan-to-localmap matching");
DEFINE_int32(initial_scan2scan_frame_num, 2, "only conduct scan to scan registration for the first　${initial_scan2scan_frame_num} frames");
DEFINE_int32(motion_compensation_method, 0, "method for motion compensation of lidar (0: disabled, 1: uniform motion model (from point-wise timestamp), 2: from azimuth, 3: from azimuth (rotation-only), 4: imu-assisted)");
DEFINE_bool(vertical_ang_calib_on, false, "apply vertical intrinsic angle correction for sepecific lidar");
DEFINE_double(vertical_ang_correction_deg, 0.0, "the intrinsic correction of the vertical angle of lidar");
DEFINE_bool(zupt_on_or_not, false, "enable zupt (zero velocity updating) or not");
DEFINE_bool(apply_scanner_filter, false, "enable scanner based distance filtering or not");
DEFINE_bool(semantic_assist_on, false, "apply semantic mask to assist the geometric feature points extraction");
DEFINE_double(cloud_down_res, 0.0, "voxel size(m) of downsample for target point cloud");
DEFINE_int32(dist_inverse_sampling_method, 2, "use distance inverse sampling or not (0: disabled, 1: linear weight, 2: quadratic weight)");
DEFINE_double(unit_dist, 15.0, "distance that correspoinding to unit weight in inverse distance downsampling");
DEFINE_bool(adaptive_parameters_on, false, "use self-adaptive parameters for different surroundings and road situation");
DEFINE_double(cloud_pca_neigh_r, 0.6, "pca neighborhood searching radius(m) for target point cloud");
DEFINE_int32(cloud_pca_neigh_k, 25, "use only the k nearest neighbor in the r-neighborhood to do PCA");
DEFINE_int32(cloud_pca_neigh_k_min, 8, "the min number of points in the neighborhood for doing PCA");
DEFINE_int32(pca_down_rate, 2, "Downsampling rate of the pca querying points and the points used to calculate pca and build the kd-tree");
DEFINE_bool(sharpen_with_nms_on, true, "using non-maximum supression to get the sharpen feature points from unsharpen points or not (use higher threshold)");
DEFINE_bool(fixed_num_downsampling_on, true, "enable/disable the fixed point number downsampling (processing time's standard deviation would br smaller)");
DEFINE_int32(ground_down_fixed_num, 300, "fixed number of the detected ground feature points (for source)");
DEFINE_int32(pillar_down_fixed_num, 100, "fixed number of the detected pillar feature points (for source)");
DEFINE_int32(facade_down_fixed_num, 400, "fixed number of the detected facade feature points (for source)");
DEFINE_int32(beam_down_fixed_num, 100, "fixed number of the detected beam feature points (for source)");
DEFINE_int32(roof_down_fixed_num, 0, "fixed number of the detected roof feature points (for source)");
DEFINE_int32(unground_down_fixed_num, 10000, "fixed number of the unground points used for PCA calculation");
DEFINE_double(gf_grid_size, 3.0, "grid size(m) of ground segmentation");
DEFINE_double(gf_in_grid_h_thre, 0.3, "height threshold(m) above the lowest point in a grid for ground segmentation");
DEFINE_double(gf_neigh_grid_h_thre, 1.5, "height threshold(m) among neighbor grids for ground segmentation");
DEFINE_double(gf_max_h, 5.0, "max height(m) allowed for ground point");
DEFINE_int32(ground_normal_method, 0, "method for estimating the ground points' normal vector ( 0: directly use (0,0,1), 1: estimate normal in fix radius neighborhood , 2: estimate normal in k nearest neighborhood, 3: use ransac to estimate plane coeffs in a grid)");
DEFINE_double(gf_normal_estimation_radius, 2.0, "neighborhood radius for local normal estimation of ground points (only enabled when ground_normal_method=1)");
DEFINE_int32(gf_ground_down_rate, 15, "downsampling decimation rate for target ground point cloud");
DEFINE_int32(gf_nonground_down_rate, 3, "downsampling decimation rate for non-ground point cloud");
DEFINE_double(intensity_thre_nonground, FLT_MAX, "Points whose intensity is larger than this value would be regarded as highly reflective object so that downsampling would not be applied.");
DEFINE_int32(gf_grid_min_pt_num, 10, "min number of points in a grid (if < this value, the grid would not be considered");
DEFINE_int32(gf_reliable_neighbor_grid_thre, 0, "min number of neighboring grid whose point number is larger than gf_grid_min_pt_num-1");
DEFINE_int32(gf_down_down_rate, 2, "downsampling rate based on the already downsampled ground point clouds used for source point cloud");
DEFINE_double(feature_pts_ratio_guess, 0.3, "A guess of the percent of the geometric feature points in the neighborhood");
DEFINE_double(linearity_thre, 0.65, "pca linearity threshold for target point cloud");
DEFINE_double(planarity_thre, 0.65, "pca planarity threshold for target point cloud");
DEFINE_double(linearity_thre_down, 0.75, "pca linearity threshold for source point cloud");
DEFINE_double(planarity_thre_down, 0.75, "pca planarity threshold for source point cloud");
DEFINE_double(curvature_thre, 0.12, "pca local curvature threshold");
DEFINE_int32(bsc_grid_num_per_side, 7, "numbder of grid per side in BSC feature");
DEFINE_double(beam_direction_ang, 25, "the verticle angle threshold for the direction vector of beam-type feature points");
DEFINE_double(pillar_direction_ang, 60, "the verticle angle threshold for the direction vector of pillar-type feature points");
DEFINE_double(facade_normal_ang, 30, "the verticle angle threshold for the normal vector of facade-type feature points");
DEFINE_double(roof_normal_ang, 75, "the verticle angle threshold for the normal vector roof-type feature points");
DEFINE_double(beam_max_height, FLT_MAX, "max bearable height for beam points");
DEFINE_int32(vertex_extraction_method, 2, "extraction method of vertex points (0: disabled, 1: maximum local curvature within stable points, 2: intersection points of pillar and beams)");
DEFINE_bool(detect_curb_or_not, false, "detect curb feature for urban scenarios or not");
DEFINE_bool(apply_roi_filter, false, "use the region of interest filter to remove dynamic objects or not");
DEFINE_double(roi_min_y, -FLT_MAX, "region of interest (delete part): min_y");
DEFINE_double(roi_max_y, FLT_MAX, "region of interest (delete part): max_y");
DEFINE_string(used_feature_type, "111100", "used_feature_type (1: on, 0: off, order: ground, pillar, beam, facade, roof, vetrex)");
DEFINE_bool(reg_intersection_filter_on, true, "filter the points outside the intersection aera of two point cloud during registration");
DEFINE_bool(normal_shooting_on, false, "using normal shooting instead of nearest neighbor searching when determing correspondences");
DEFINE_double(normal_bearing, 35.0, "the normal consistency checking angle threshold (unit: degree)");
DEFINE_double(corr_dis_thre_init, 1.5, "distance threshold between correspondence points at begining");
DEFINE_double(corr_dis_thre_min, 0.5, "minimum distance threshold between correspondence points at begining");
DEFINE_double(dis_thre_update_rate, 1.1, "update rate (divided by this value at each iteration) for distance threshold between correspondence points");
DEFINE_string(corr_weight_strategy, "1101", "weighting strategy for correspondences (1: on, 0: off, order: x,y,z balanced weight, residual weight, distance weight, intensity weight)");
DEFINE_double(z_xy_balance_ratio, 1.0, "the weight ratio of the error along z and x,y direction when balanced weight is enabled");
DEFINE_double(pt2pt_res_window, 0.1, "residual window size for the residual robust kernel function of point to point correspondence");
DEFINE_double(pt2pl_res_window, 0.1, "residual window size for the residual robust kernel function of point to plane correspondence");
DEFINE_double(pt2li_res_window, 0.1, "residual window size for the residual robust kernel function of point to line correspondence");
DEFINE_int32(reg_max_iter_num_s2s, 1, "max iteration number for icp-based registration (scan to scan)");
DEFINE_int32(reg_max_iter_num_s2m, 1, "max iteration number for icp-based registration (scan to map)");
DEFINE_int32(reg_max_iter_num_m2m, 3, "max iteration number for icp-based registration (map to map)");
DEFINE_double(converge_tran, 0.001, "convergence threshold for translation (in m)");
DEFINE_double(converge_rot_d, 0.01, "convergence threshold for rotation (in degree)");
DEFINE_double(post_sigma_thre, 0.35, "the maximum threshold for the posterior standard deviation of the least square adjustment during the registration.(unit:m)");
DEFINE_double(local_map_radius, 50.0, "the radius of the local map (regarded as a sphere aera)");
DEFINE_int32(local_map_max_pt_num, 8000, "max point number allowed for the local map");
DEFINE_int32(local_map_max_vertex_pt_num, 1000, "max vertex point number allowed for the local map");
DEFINE_double(append_frame_radius, 60.0, "the radius of the frame that used to append into the local map");
DEFINE_bool(apply_map_based_dynamic_removal, false, "use map based dynamic object removal or not");
DEFINE_double(map_min_dist_within_feature, 0.03, "if the expanded feature point is too close to already exsit map points, it would not be added to the map");
DEFINE_double(dynamic_removal_radius, 30.0, "the radius of the map based dynamic object removing");
DEFINE_double(dynamic_dist_thre_min, 0.3, "the distance threshold to judge if a point is dynamic or not");
DEFINE_int32(local_map_recalculation_frequency, 99999, "Recalculate the linear features in the local map each ${local_map_recalculation_frequency} frame");
DEFINE_int32(s2m_frequency, 1, "frequency of scan to map registration");
DEFINE_int32(initial_guess_mode, 2, "Use which kind of initial guess(0: no initial guess, 1: uniform motion(translation only), 2: uniform motion(translation+rotation), 3:imu based)");
//prior knowledge
DEFINE_double(approx_scanner_height, 1.5, "approximate height of the scanner (m)");
DEFINE_double(underground_height_thre, -6.0, "z-axis threshold for rejecting underground ghost points (lines)");
//loop closure and pose graph optimization related
DEFINE_bool(loop_closure_detection_on, false, "do loop closure detection and pose graph optimization or not");
DEFINE_double(submap_accu_tran, 15.0, "accumulated translation (m) for generating a new submap");
DEFINE_double(submap_accu_rot, 90.0, "accumulated rotation (deg) for generating a new submap");
DEFINE_int32(submap_accu_frame, 150, "accumulated frame number for generating a new submap");
DEFINE_double(map2map_reliable_sigma_thre, 0.04, "if the standard deviation of the map to map registration is smaller than this value, we would use it as odometry result");
DEFINE_bool(overall_loop_closure_searching_on, false, "searching loop clousre within a larger neighborhood");
DEFINE_double(min_iou_thre, 0.4, "min boundingbox iou for candidate registration edge");
DEFINE_double(min_iou_thre_global_reg, 0.5, "min boundingbox iou for global registration edge");
DEFINE_int32(min_submap_id_diff, 8, "min submap id difference between two submaps for a putable registration edge");
DEFINE_double(neighbor_search_dist, 50.0, "max distance for candidate registration edge");
DEFINE_double(map_to_map_min_cor_ratio, 0.15, "min feature point overlapping ratio for map to map registration");
DEFINE_int32(cooling_submap_num, 2, "waiting for several submaps (without loop closure detection) after applying a successful pgo");
DEFINE_double(adjacent_edge_weight_ratio, 1.0, "weight of adjacent edge compared with registration edge");
DEFINE_int32(num_frame_thre_large_drift, 1000, "the lidar odometry may have large drift after so many frames so we would use global registration instead of a local registration with lidar odom initial guess");
DEFINE_int32(max_used_reg_edge_per_optimization, 3, "only usee the first ${max_used_reg_edge_per_optimization} for pgo");
DEFINE_bool(equal_weight_on, false, "using equal weight for the information matrix in pose graph optimization");
DEFINE_bool(reciprocal_feature_match_on, false, "using reciprocal nn feature matching or not");
DEFINE_bool(best_n_feature_match_on, true, "select the n correspondence with min feature distance as the putatble matches");
DEFINE_int32(feature_corr_num, 1000, "number of the correspondence for global coarse registration");
DEFINE_bool(teaser_based_global_registration_on, true, "Using TEASER++ to do the global coarse registration or not");
DEFINE_int32(global_reg_min_inlier_count, 7, "min inlier correspondence for a successful feature based registration (for teaser or ransac)");
DEFINE_string(pose_graph_optimization_method, "ceres", "use which library to do pgo (select from g2o, ceres and gtsam)");
DEFINE_double(inter_submap_t_limit, 2.0, "the submap node's limit of translation variation, unit:m");
DEFINE_double(inter_submap_r_limit, 0.05, "the submap node's limit of rotation variation, unit quaternion");
DEFINE_double(inner_submap_t_limit, 0.1, "the inner submap frame node's limit of translation variation, unit:m");
DEFINE_double(inner_submap_r_limit, 0.01, "the inner submap frame node's limit of rotation variation, unit:m");
DEFINE_int32(max_iter_inter_submap, 100, "max iteration number for inter submap pgo");
DEFINE_int32(max_iter_inner_submap, 100, "max iteration number for inner submap pgo");
DEFINE_double(first_time_cov_update_ratio, 1.0, "edge covariance update (at first pgo)");
DEFINE_double(life_long_cov_update_ratio, 1.0, "edge covariance update (after first pgo)");
DEFINE_bool(diagonal_information_matrix_on, false, "use diagonal information matrix in pgo or not");
DEFINE_double(wrong_edge_tran_thre, 5.0, "translation threshold for judging if a edge is wrong or not");
DEFINE_double(wrong_edge_rot_thre_deg, 25.0, "rotation threshold for judging if a edge is wrong or not");
DEFINE_double(frame_estimated_error_tran, 1.0, "estimated max translation error of the lidar odometry per frame");
DEFINE_double(frame_estimated_error_rot_deg, 2.0, "estimated max rotation error of the lidar odometry per frame");
DEFINE_bool(robust_kernel_on, false, "turn on the robust kernel function in pgo");
DEFINE_bool(free_node_on, false, "enable the free node module or not");
DEFINE_bool(transfer_correct_reg_tran_on, true, "enable the registration tranformation transfer (only do global reg. once for each query submap)");
DEFINE_bool(framewise_pgo_on, false, "use frame-wise pgo or not");
//baseline method options
DEFINE_string(baseline_reg_method, "", "name of the baseline lidar odometery method (ndt, gicp, pclicp, etc.)");
DEFINE_double(reg_voxel_size, 1.0, "the grid size of ndt or vgicp");
DEFINE_bool(ndt_searching_method, true, "using direct searching or kdtree (0: kdtree, 1: direct7)");
DEFINE_bool(voxel_gicp_on, true, "using voxel based gicp (faster)");
//Note: Set these parameters in the config file , or the default values are used.
//--------------------------------------------------------------------------------------------------------------------------------------------------------------
int main(int argc, char **argv)
{
    google::ParseCommandLineFlags(&argc, &argv, true);
    google::InitGoogleLogging("Mylog_testlo");
    LOG(INFO) << "Launch the program!";
    LOG(INFO) << "Logging is written to " << FLAGS_log_dir;

    pcl::console::setVerbosityLevel(pcl::console::L_ALWAYS); //Ban pcl warnings
    CHECK(FLAGS_point_cloud_folder != "") << "Need to specify the point cloud's folder.";
    CHECK(FLAGS_output_lo_lidar_pose_file_path != "") << "Need to specify the output lidar odometery pose (in lidar coor. sys.) file's path.";
    //Import configuration
    //Data path (now only the *.pcd format is available)
    std::string pc_folder = FLAGS_point_cloud_folder;
    std::string pc_format = FLAGS_pc_format;
    std::string gt_body_pose_file = FLAGS_gt_body_pose_file_path;
    std::string calib_file = FLAGS_calib_file_path;
    std::string output_adjacent_lo_pose_file = FLAGS_output_adjacent_lo_pose_file_path;
    std::string output_lo_body_pose_file = FLAGS_output_lo_body_pose_file_path;
    std::string output_lo_lidar_pose_file = FLAGS_output_lo_lidar_pose_file_path;
    std::string output_gt_lidar_pose_file = FLAGS_output_gt_lidar_pose_file_path;
    std::string output_pc_folder = FLAGS_output_map_point_cloud_folder_path;
    std::string gt_lidar_pose_point_cloud_file = FLAGS_gt_lidar_pose_point_cloud;
    std::string lo_lidar_pose_point_cloud_file = FLAGS_lo_lidar_pose_point_cloud;
    //visualization settings
    int downsamping_rate_scan_vis = 5;
    int display_time_ms = 1;
    //parameters (mainly for experiment)
    float gf_grid_resolution = FLAGS_gf_grid_size;
    float gf_max_grid_height_diff = FLAGS_gf_in_grid_h_thre;
    float gf_neighbor_height_diff = FLAGS_gf_neigh_grid_h_thre;
    int ground_down_rate = FLAGS_gf_ground_down_rate;
    int nonground_down_rate = FLAGS_gf_nonground_down_rate;
    int gf_grid_min_pt_num = FLAGS_gf_grid_min_pt_num;
    int gf_reliable_neighbor_grid_thre = FLAGS_gf_reliable_neighbor_grid_thre;
    float pca_neigh_r = FLAGS_cloud_pca_neigh_r;
    int pca_neigh_k = FLAGS_cloud_pca_neigh_k;
    float feature_neighbor_radius = 2.0 * pca_neigh_r;
    float pca_linearity_thre = FLAGS_linearity_thre;
    float pca_planarity_thre = FLAGS_planarity_thre;
    float beam_direction_sin = std::sin(FLAGS_beam_direction_ang / 180.0 * M_PI);
    float pillar_direction_sin = std::sin(FLAGS_pillar_direction_ang / 180.0 * M_PI);
    float facade_normal_sin = std::sin(FLAGS_facade_normal_ang / 180.0 * M_PI);
    float roof_normal_sin = std::sin(FLAGS_roof_normal_ang / 180.0 * M_PI);
    float pca_curvature_thre = FLAGS_curvature_thre;
    float reg_corr_dis_thre_init = FLAGS_corr_dis_thre_init;
    float reg_corr_dis_thre_min = FLAGS_corr_dis_thre_min;
    float dis_thre_update_rate = FLAGS_dis_thre_update_rate;
    float z_xy_balance_ratio = FLAGS_z_xy_balance_ratio;
    float converge_tran = FLAGS_converge_tran;
    float converge_rot_d = FLAGS_converge_rot_d;
    float pt2pt_residual_window = FLAGS_pt2pt_res_window;
    float pt2pl_residual_window = FLAGS_pt2pl_res_window;
    float pt2li_residual_window = FLAGS_pt2li_res_window;
    int max_iteration_num_s2s = FLAGS_reg_max_iter_num_s2s;
    int max_iteration_num_s2m = FLAGS_reg_max_iter_num_s2m;
    int max_iteration_num_m2m = FLAGS_reg_max_iter_num_m2m;
    int initial_guess_mode = FLAGS_initial_guess_mode;
    float local_map_radius = FLAGS_local_map_radius;
    float append_frame_radius = FLAGS_append_frame_radius;
    int local_map_max_pt_num = FLAGS_local_map_max_pt_num;
    int vertex_keeping_num = FLAGS_local_map_max_vertex_pt_num;
    float dynamic_removal_radius = FLAGS_dynamic_removal_radius;
    float dynamic_dist_thre_min = FLAGS_dynamic_dist_thre_min;
    bool loop_closure_detection_on = FLAGS_loop_closure_detection_on;

    DataIo<Point_T> dataio;
    MapViewer<Point_T> mviewer(0.0, 1, 0, 1, 0, FLAGS_vis_intensity_scale, FLAGS_vis_initial_color_type, FLAGS_laser_vis_size); //downsampling ratio //N.B. default: map_viewer on, feature_viewer on, or the others off for the initialization
    CFilter<Point_T> cfilter;
    CRegistration<Point_T> creg;
    MapManager mmanager;
    Navigation nav;
    Constraint_Finder confinder;
    GlobalOptimize pgoptimizer;

    //set pgo options
    pgoptimizer.set_covariance_updating_ratio(FLAGS_first_time_cov_update_ratio, FLAGS_life_long_cov_update_ratio);
    pgoptimizer.set_wrong_edge_check_threshold(FLAGS_wrong_edge_tran_thre, FLAGS_wrong_edge_rot_thre_deg);
    //Launch visualization module
    boost::shared_ptr<pcl::visualization::PCLVisualizer> feature_viewer, reg_viewer, scan_viewer, map_viewer;
    if (FLAGS_real_time_viewer_on)
    {
        feature_viewer = boost::shared_ptr<pcl::visualization::PCLVisualizer>(new pcl::visualization::PCLVisualizer("Feature Viewer"));
        reg_viewer = boost::shared_ptr<pcl::visualization::PCLVisualizer>(new pcl::visualization::PCLVisualizer("Registration Viewer"));
        scan_viewer = boost::shared_ptr<pcl::visualization::PCLVisualizer>(new pcl::visualization::PCLVisualizer("Scan Viewer"));
        map_viewer = boost::shared_ptr<pcl::visualization::PCLVisualizer>(new pcl::visualization::PCLVisualizer("Map Viewer"));
        mviewer.set_interactive_events(feature_viewer, FLAGS_screen_width / 2, FLAGS_screen_height / 2);
        mviewer.set_interactive_events(reg_viewer, FLAGS_screen_width / 2, FLAGS_screen_height / 2);
        mviewer.set_interactive_events(scan_viewer, FLAGS_screen_width / 2, FLAGS_screen_height / 2);
        mviewer.set_interactive_events(map_viewer, FLAGS_screen_width / 2, FLAGS_screen_height);
    }
    double time_count = 0.0;
    std::vector<std::string> filenames;
    dataio.batch_read_filenames_in_folder(pc_folder, "_filelist.txt", pc_format, filenames, FLAGS_frame_num_begin, FLAGS_frame_num_end, FLAGS_frame_step);
    Matrix4ds poses_gt_body_cs;  //in vehicle body (gnssins) coordinate system
    Matrix4ds poses_lo_body_cs;  //in vehicle body (gnssins) coordinate system
    Matrix4ds poses_gt_lidar_cs; //in lidar coordinate system
    Matrix4ds poses_lo_lidar_cs; //in lidar coordinate system
    Matrix4ds poses_lo_adjacent;
    Eigen::Matrix4d calib_mat; //the calib_mat is the transformation from lidar frame to body/camera frame (Tb_l)
    Eigen::Matrix4d init_poses_gt_lidar_cs;
    if (FLAGS_gt_in_lidar_frame)
    {
        poses_gt_lidar_cs = dataio.load_poses_from_transform_matrix(gt_body_pose_file, FLAGS_frame_num_begin, FLAGS_frame_num_end, FLAGS_frame_step);
        init_poses_gt_lidar_cs = poses_gt_lidar_cs[0];
    }
    else
    {
        if (FLAGS_gt_oxts_format)
            poses_gt_body_cs = dataio.load_poses_from_pose_quat(gt_body_pose_file, FLAGS_frame_num_begin, FLAGS_frame_num_end, FLAGS_frame_step);
        else
            poses_gt_body_cs = dataio.load_poses_from_transform_matrix(gt_body_pose_file, FLAGS_frame_num_begin, FLAGS_frame_num_end, FLAGS_frame_step);
    }

    dataio.load_calib_mat(calib_file, calib_mat);
    int frame_num = filenames.size();
    std::vector<std::vector<float>> timing_array(frame_num); //unit: s

    cloudblock_Ptr cblock_target(new cloudblock_t());
    cloudblock_Ptr cblock_source(new cloudblock_t());
    cloudblock_Ptr cblock_history(new cloudblock_t());
    cloudblock_Ptr cblock_local_map(new cloudblock_t());
    cloudblock_Ptrs cblock_submaps;
    cloudblock_Ptrs cblock_frames;
    constraint_t scan2scan_reg_con;
    constraint_t scan2map_reg_con;
    constraint_t scan2ground_reg_con;
    constraint_t scan2history_reg_con;
    constraints pgo_edges;
    Eigen::Matrix4d initial_guess_tran = Eigen::Matrix4d::Identity();
    Eigen::Matrix4d adjacent_pose_out = Eigen::Matrix4d::Identity();
    Eigen::Matrix4d first_frame_body = Eigen::Matrix4d::Identity();
    LOG(WARNING) << "[" << omp_get_max_threads() << "] threads availiable in total";

    bool seg_new_submap = false, local_map_recalculate_feature_on = false, motion_com_while_reg_on = false, apply_roi_filter = false, lo_status_healthy =true;
    int submap_count = 0, cooling_index =0, accu_frame = 0, accu_frame_count_wo_opt =0;
    float accu_tran = 0.0, accu_rot_deg = 0.0, current_linear_velocity=0.0, current_angular_velocity =0.0, add_length=0.0, roi_min_y = 0.0, roi_max_y = 0.0;
    float non_max_suppresssion_radius = 0.25 * pca_neigh_r;

    if (FLAGS_motion_compensation_method > 0)
        motion_com_while_reg_on = true;
    cblock_target->filename = filenames[0];
    cblock_target->unique_id = FLAGS_frame_num_begin;
    cblock_target->is_single_scanline = false; //multi-scanline lidar
    cblock_target->pose_lo.setIdentity();
    cblock_target->pose_gt.setIdentity();
    dataio.check_overwrite_exsiting_file_or_not(output_adjacent_lo_pose_file);
    dataio.write_lo_pose_overwrite(cblock_target->pose_lo, output_lo_lidar_pose_file);
    dataio.write_lo_pose_overwrite(cblock_target->pose_gt, output_gt_lidar_pose_file);
    dataio.write_lo_pose_overwrite(first_frame_body, output_lo_body_pose_file);
    poses_lo_body_cs.push_back(first_frame_body);
    poses_lo_lidar_cs.push_back(cblock_target->pose_lo);
    if (FLAGS_gt_in_lidar_frame)
        poses_gt_lidar_cs[0] = cblock_target->pose_gt;
    else
        poses_gt_lidar_cs.push_back(cblock_target->pose_gt);
    dataio.read_pc_cloud_block(cblock_target);

    std::chrono::steady_clock::time_point tic_feature_extraction_init = std::chrono::steady_clock::now();
    if (FLAGS_apply_dist_filter)
        cfilter.dist_filter(cblock_target->pc_raw, FLAGS_min_dist_used, FLAGS_max_dist_used);
    if (FLAGS_vertical_ang_calib_on) //intrinsic angle correction
        cfilter.vertical_intrinsic_calibration(cblock_target->pc_raw, FLAGS_vertical_ang_correction_deg);
    cfilter.extract_semantic_pts(cblock_target, FLAGS_cloud_down_res, gf_grid_resolution, gf_max_grid_height_diff,
                                 gf_neighbor_height_diff, FLAGS_gf_max_h, ground_down_rate,
                                 nonground_down_rate, pca_neigh_r, pca_neigh_k,
                                 pca_linearity_thre, pca_planarity_thre, pca_curvature_thre,
                                 FLAGS_linearity_thre_down, FLAGS_planarity_thre_down, true,
                                 FLAGS_dist_inverse_sampling_method, FLAGS_unit_dist,
                                 FLAGS_ground_normal_method, FLAGS_gf_normal_estimation_radius,
                                 FLAGS_adaptive_parameters_on, FLAGS_apply_scanner_filter, FLAGS_detect_curb_or_not,
                                 FLAGS_vertex_extraction_method, gf_grid_min_pt_num, gf_reliable_neighbor_grid_thre,
                                 FLAGS_gf_down_down_rate, FLAGS_cloud_pca_neigh_k_min, FLAGS_pca_down_rate, FLAGS_intensity_thre_nonground,
                                 pillar_direction_sin, beam_direction_sin, roof_normal_sin, facade_normal_sin,
                                 FLAGS_sharpen_with_nms_on, FLAGS_fixed_num_downsampling_on, FLAGS_ground_down_fixed_num, FLAGS_pillar_down_fixed_num,
                                 FLAGS_facade_down_fixed_num, FLAGS_beam_down_fixed_num, FLAGS_roof_down_fixed_num, FLAGS_unground_down_fixed_num,
                                 FLAGS_beam_max_height, FLAGS_approx_scanner_height + 0.5, FLAGS_approx_scanner_height, FLAGS_underground_height_thre,
                                 FLAGS_feature_pts_ratio_guess, FLAGS_semantic_assist_on, apply_roi_filter, roi_min_y, roi_max_y);
                                 
    std::chrono::steady_clock::time_point toc_feature_extraction_init = std::chrono::steady_clock::now();
    std::chrono::duration<double> time_used_init_feature_extraction = std::chrono::duration_cast<std::chrono::duration<double>>(toc_feature_extraction_init - tic_feature_extraction_init);
    timing_array[0].push_back(time_used_init_feature_extraction.count());
    for (int k = 0; k < 3; k++) //for the first frame, we only extract its feature points
        timing_array[0].push_back(0.0);

    initial_guess_tran(0, 3) = 0.5;     //initialization
    for (int i = 1; i < frame_num; i++) //throughout all the used frames
    {
        std::chrono::steady_clock::time_point tic = std::chrono::steady_clock::now();
        accu_frame_count_wo_opt++;
        if (i == 1) // for the first scan matching (no initial guess avaliable --> larger corr distance)
            add_length = 1.0;
        cblock_source->filename = filenames[i];
        cblock_source->unique_id = i * FLAGS_frame_step + FLAGS_frame_num_begin;
        cblock_source->is_single_scanline = false;
        if (poses_gt_body_cs.size() > 0)
        {                                                                                                                     //check if gt pose is availiable
            cblock_source->pose_gt = calib_mat.inverse() * (poses_gt_body_cs[0].inverse() * poses_gt_body_cs[i]) * calib_mat; //set ground truth pose (from body to lidar coordinate system)
            if (FLAGS_gt_in_lidar_frame)
                cblock_source->pose_gt = init_poses_gt_lidar_cs.inverse() * poses_gt_lidar_cs[i];
            poses_gt_body_cs[i] = poses_gt_body_cs[0].inverse() * poses_gt_body_cs[i]; //according to the first frame
        }
        dataio.read_pc_cloud_block(cblock_source);
        std::chrono::steady_clock::time_point toc_import_pc = std::chrono::steady_clock::now();
        if (FLAGS_apply_dist_filter)
            cfilter.dist_filter(cblock_source->pc_raw, FLAGS_min_dist_used, FLAGS_max_dist_used);
        if (FLAGS_vertical_ang_calib_on) //intrinsic angle correction
            cfilter.vertical_intrinsic_calibration(cblock_source->pc_raw, FLAGS_vertical_ang_correction_deg);
        // motion compensation [first step: using the last frame's transformation]
        if (FLAGS_motion_compensation_method == 1) //calculate from time-stamp
            cfilter.get_pts_timestamp_ratio_in_frame(cblock_source->pc_raw, true);
        else if (FLAGS_motion_compensation_method == 2)                                   //calculate from azimuth
            cfilter.get_pts_timestamp_ratio_in_frame(cblock_source->pc_raw, false, 90.0); //HESAI Lidar: 90.0 (y+ axis, clockwise), Velodyne Lidar: 180.0
        if (!strcmp(FLAGS_baseline_reg_method.c_str(), "ndt"))                            //baseline_method
            cfilter.voxel_downsample(cblock_source->pc_raw, cblock_source->pc_down, FLAGS_cloud_down_res);
        else if (!strcmp(FLAGS_baseline_reg_method.c_str(), "gicp")) //baseline_method
            cfilter.voxel_downsample(cblock_source->pc_raw, cblock_source->pc_down, FLAGS_cloud_down_res);
        else
            cfilter.extract_semantic_pts(cblock_source, FLAGS_cloud_down_res, gf_grid_resolution, gf_max_grid_height_diff,
                                         gf_neighbor_height_diff, FLAGS_gf_max_h, ground_down_rate,
                                         nonground_down_rate, pca_neigh_r, pca_neigh_k, pca_linearity_thre, pca_planarity_thre, pca_curvature_thre,
                                         FLAGS_linearity_thre_down, FLAGS_planarity_thre_down, true, FLAGS_dist_inverse_sampling_method, FLAGS_unit_dist,
                                         FLAGS_ground_normal_method, FLAGS_gf_normal_estimation_radius, FLAGS_adaptive_parameters_on, FLAGS_apply_scanner_filter, FLAGS_detect_curb_or_not,
                                         FLAGS_vertex_extraction_method, gf_grid_min_pt_num, gf_reliable_neighbor_grid_thre,
                                         FLAGS_gf_down_down_rate, FLAGS_cloud_pca_neigh_k_min, FLAGS_pca_down_rate, FLAGS_intensity_thre_nonground,
                                         pillar_direction_sin, beam_direction_sin, roof_normal_sin, facade_normal_sin, FLAGS_sharpen_with_nms_on, FLAGS_fixed_num_downsampling_on, FLAGS_ground_down_fixed_num,
                                         FLAGS_pillar_down_fixed_num, FLAGS_facade_down_fixed_num, FLAGS_beam_down_fixed_num, FLAGS_roof_down_fixed_num, FLAGS_unground_down_fixed_num,
                                         FLAGS_beam_max_height, FLAGS_approx_scanner_height + 0.5, FLAGS_approx_scanner_height, FLAGS_underground_height_thre, FLAGS_feature_pts_ratio_guess, FLAGS_semantic_assist_on,
                                         apply_roi_filter, roi_min_y, roi_max_y);
        std::chrono::steady_clock::time_point toc_feature_extraction = std::chrono::steady_clock::now();

        //update local map
        if (i % FLAGS_local_map_recalculation_frequency == 0)
            local_map_recalculate_feature_on = true;
        else
            local_map_recalculate_feature_on = false;

        if (i > FLAGS_initial_scan2scan_frame_num + 1)
            mmanager.update_local_map(cblock_local_map, cblock_target, local_map_radius, local_map_max_pt_num, vertex_keeping_num, append_frame_radius,
                                      FLAGS_apply_map_based_dynamic_removal, FLAGS_used_feature_type, dynamic_removal_radius, dynamic_dist_thre_min, current_linear_velocity * 0.15,
                                      FLAGS_map_min_dist_within_feature, local_map_recalculate_feature_on); //1.5 * 0.1 * velocity (set as the max distance threshold for dynamic obejcts)
        else
            mmanager.update_local_map(cblock_local_map, cblock_target, local_map_radius, local_map_max_pt_num, vertex_keeping_num, append_frame_radius, false, FLAGS_used_feature_type);

        int temp_accu_frame = accu_frame;
        if (loop_closure_detection_on) //determine if we can add a new submap
            seg_new_submap = mmanager.judge_new_submap(accu_tran, accu_rot_deg, accu_frame, FLAGS_submap_accu_tran, FLAGS_submap_accu_rot, FLAGS_submap_accu_frame);
        else
            seg_new_submap = false;
        std::chrono::steady_clock::time_point toc_update_map = std::chrono::steady_clock::now();

        if (seg_new_submap) //add nodes and edges in pose graph for pose graph optimization (pgo)
        {
            LOG(INFO) << "Create new submap [" << submap_count << "]";
            cloudblock_Ptr current_cblock_local_map(new cloudblock_t(*cblock_local_map, true));
            current_cblock_local_map->strip_id = 0;
            current_cblock_local_map->id_in_strip = submap_count;
            current_cblock_local_map->last_frame_index = i - 1; //save the last frame index
            current_cblock_local_map->unique_id = cblock_target->unique_id;
            current_cblock_local_map->pose_init = current_cblock_local_map->pose_lo;                                            //init guess for pgo
            current_cblock_local_map->information_matrix_to_next = 1.0 / temp_accu_frame * scan2map_reg_con.information_matrix; //not used now //Use the correct function by deducing Jacbobian of transformation
            //encode features in the submap (this is already done in each frame)
            cfilter.non_max_suppress(current_cblock_local_map->pc_vertex, non_max_suppresssion_radius, false, current_cblock_local_map->tree_vertex);
            if (submap_count == 0)
                current_cblock_local_map->pose_fixed = true; //fixed the first submap
            LOG(INFO) << "Submap node index [" << submap_count << "]";
            cblock_submaps.push_back(current_cblock_local_map); //add new node
            submap_count++;
            cooling_index--;
            if (submap_count > 1)
            {
                //delete the past registration edges (which are not reliable)
                confinder.cancel_registration_constraint(pgo_edges);
                //add adjacent edge between current submap and the last submap
                confinder.add_adjacent_constraint(cblock_submaps, pgo_edges, submap_count);
                // do adjacent map to map registration
                int current_edge_index = pgo_edges.size() - 1;
                int registration_status_map2map = creg.mm_lls_icp(pgo_edges[current_edge_index], max_iteration_num_m2m, 1.5 * reg_corr_dis_thre_init,
                                                                  converge_tran, converge_rot_d, 1.5 * reg_corr_dis_thre_min, dis_thre_update_rate,
                                                                  FLAGS_used_feature_type, "1101", z_xy_balance_ratio,
                                                                  2 * pt2pt_residual_window, 2 * pt2pl_residual_window, 2 * pt2li_residual_window,
                                                                  pgo_edges[current_edge_index].Trans1_2, FLAGS_reg_intersection_filter_on, false,
                                                                  FLAGS_normal_shooting_on, 1.5 * FLAGS_normal_bearing, true, true, FLAGS_post_sigma_thre); //use its information matrix for pgo

                if (registration_status_map2map <= 0 && FLAGS_real_time_viewer_on) //candidate wrong registration
                    mviewer.keep_visualize(reg_viewer);
                else
                    LOG(INFO) << "map to map registration done\nsubmap [" << pgo_edges[current_edge_index].block1->id_in_strip << "] - [" << pgo_edges[current_edge_index].block2->id_in_strip << "]:\n"
                              << pgo_edges[current_edge_index].Trans1_2;
                if (pgo_edges[current_edge_index].sigma > FLAGS_map2map_reliable_sigma_thre)                                                                          // if the estimated posterior standard deviation of map to map registration is a bit large
                    pgo_edges[current_edge_index].Trans1_2 = pgo_edges[current_edge_index].block1->pose_lo.inverse() * pgo_edges[current_edge_index].block2->pose_lo; //still use the scan-to-map odometry's prediction (but we will keep the information matrix)
                else
                {
                    LOG(WARNING) << "We would trust the map to map registration, update current pose";                                  //the edge's transformation and information are already calculted via the last map-to-map registration
                    cblock_local_map->pose_lo = pgo_edges[current_edge_index].block1->pose_lo * pgo_edges[current_edge_index].Trans1_2; //update current local map's pose
                    cblock_target->pose_lo = cblock_local_map->pose_lo;                                                                 //update target frame
                    cblock_submaps[submap_count - 1]->pose_lo = cblock_local_map->pose_lo;                                              //update the pgo node (submap)
                    cblock_submaps[submap_count - 1]->pose_init = cblock_local_map->pose_lo;                                            //update the initial guess of pgo node (submap)
                }
                pgo_edges[current_edge_index].information_matrix = FLAGS_adjacent_edge_weight_ratio * pgo_edges[current_edge_index].information_matrix; //TODO: fix (change the weight of the weight of adjacent edges)
                constraints current_registration_edges;
                if (cooling_index < 0) //find registration edges and then do pgo
                {
                    bool overall_loop_searching_on = false;
                    int reg_edge_count = 0;
                    if (accu_frame_count_wo_opt > FLAGS_num_frame_thre_large_drift && FLAGS_overall_loop_closure_searching_on) //expand the loop closure searching area
                    {
                        overall_loop_searching_on = true;
                        reg_edge_count = confinder.find_overlap_registration_constraint(cblock_submaps, current_registration_edges, 1.5 * FLAGS_neighbor_search_dist, 0.0, FLAGS_min_submap_id_diff, true, 20);
                    }
                    else //standard loop closure searching
                        reg_edge_count = confinder.find_overlap_registration_constraint(cblock_submaps, current_registration_edges, FLAGS_neighbor_search_dist, FLAGS_min_iou_thre, FLAGS_min_submap_id_diff, true);
                    int reg_edge_successful_count = 0;
                    bool stable_reg_found = false;
                    //suppose node 3 is the current submap, node 1 and node 2 are two history submaps with loop constraints with node 3, suppose node 1 and node 3 's transformation is already known (a stable registration)
                    Eigen::Matrix4d reference_pose_mat;      //the pose of the reference node (node 1), Tw1
                    Eigen::Matrix4d reference_loop_tran_mat; //the loop transformation of the reference node , T13
                    for (int j = 0; j < reg_edge_count; j++)
                    {
                        if (reg_edge_successful_count >= FLAGS_max_used_reg_edge_per_optimization) // we do not need too many registration edges (for example, more than 3 reg edges)
                            break;
                        pcTPtr cur_map_origin(new pcT()), cur_map_guess(new pcT()), cur_map_tran(new pcT()), hist_map(new pcT()), kp_guess(new pcT());
                        pcTPtr target_cor(new pcT()), source_cor(new pcT());
                        current_registration_edges[j].block2->merge_feature_points(cur_map_origin, false);
                        current_registration_edges[j].block1->merge_feature_points(hist_map, false);
                        Eigen::Matrix4d init_mat = current_registration_edges[j].Trans1_2;
                        if (stable_reg_found)                                                                                                  //infer the init guess according to a already successfully registered loop edge for current submap
                            init_mat = current_registration_edges[j].block1->pose_lo.inverse() * reference_pose_mat * reference_loop_tran_mat; //T23 = T21 * T13 = T2w * Tw1 * T13
                        // global (coarse) registration by teaser or ransac (using ncc, bsc or fpfh as feature)
                        LOG(INFO) << "Transformation initial guess predicted by lidar odometry:\n"
                                  << init_mat;
                        bool global_reg_on = false;
                        if (!stable_reg_found && (current_registration_edges[j].overlapping_ratio > FLAGS_min_iou_thre_global_reg || overall_loop_searching_on)) //with higher overlapping ratio, try to TEASER
                        {
                            creg.find_feature_correspondence_ncc(current_registration_edges[j].block1->pc_vertex, current_registration_edges[j].block2->pc_vertex,
                                                                 target_cor, source_cor, FLAGS_best_n_feature_match_on, FLAGS_feature_corr_num, FLAGS_reciprocal_feature_match_on);
                            int global_reg_status = -1;
                            if (FLAGS_teaser_based_global_registration_on)
                                global_reg_status = creg.coarse_reg_teaser(target_cor, source_cor, init_mat, pca_neigh_r, FLAGS_global_reg_min_inlier_count);
                            else // using ransac, a bit slower than teaser
                                global_reg_status = creg.coarse_reg_ransac(target_cor, source_cor, init_mat, pca_neigh_r, FLAGS_global_reg_min_inlier_count);
                            if (FLAGS_real_time_viewer_on)
                            {
                                pcl::transformPointCloud(*source_cor, *kp_guess, init_mat);
                                pcl::transformPointCloud(*cur_map_origin, *cur_map_guess, init_mat);
                                mviewer.display_correspondences_compare(feature_viewer, source_cor, target_cor, kp_guess, cur_map_origin,
                                                                        hist_map, cur_map_guess, current_registration_edges[j].Trans1_2, 5);
                            }
                            if (global_reg_status == 0) //double check
                            {
                                if (overall_loop_searching_on)
                                    global_reg_on = confinder.double_check_tran(init_mat, current_registration_edges[j].Trans1_2, init_mat, 10.0 * FLAGS_wrong_edge_tran_thre, 6.0 * FLAGS_wrong_edge_rot_thre_deg); //the difference tolerance can be a bit larger
                                else
                                    global_reg_on = confinder.double_check_tran(init_mat, current_registration_edges[j].Trans1_2, init_mat, 3.0 * FLAGS_wrong_edge_tran_thre, 3.0 * FLAGS_wrong_edge_rot_thre_deg); //if the difference of the transformation of teaser and lo initial guess is too large, we will trust lidar odometry
                            }
                            else if (global_reg_status == 1) //reliable
                                global_reg_on = true;
                        }
                        if (!global_reg_on && !stable_reg_found && accu_frame_count_wo_opt > FLAGS_num_frame_thre_large_drift) //TODO: the tolerance should be determine using pose covariance (reference: overlapnet)
                            continue;                                                                                                        //without the global registration and since the lidar odometry may drift a lot, the inital guess may not be reliable, just skip
                        int registration_status_map2map = creg.mm_lls_icp(current_registration_edges[j], max_iteration_num_m2m, 3.5 * reg_corr_dis_thre_init,
                                                                          converge_tran, converge_rot_d, 2.0 * reg_corr_dis_thre_min, dis_thre_update_rate,
                                                                          FLAGS_used_feature_type, "1101", z_xy_balance_ratio,
                                                                          2 * pt2pt_residual_window, 2 * pt2pl_residual_window, 2 * pt2li_residual_window,
                                                                          init_mat, FLAGS_reg_intersection_filter_on, false,
                                                                          FLAGS_normal_shooting_on, 1.5 * FLAGS_normal_bearing,
                                                                          true, true, FLAGS_post_sigma_thre, FLAGS_map_to_map_min_cor_ratio);
                        if (registration_status_map2map > 0)
                        {
                            pgo_edges.push_back(current_registration_edges[j]);
                            reg_edge_successful_count++; //putable correctly registered registration edge
                            if (!stable_reg_found && FLAGS_transfer_correct_reg_tran_on)
                            {
                                reference_pose_mat = current_registration_edges[j].block1->pose_lo; //the correct registered loop closure history node's pose
                                reference_loop_tran_mat = current_registration_edges[j].Trans1_2;   //the correct registered loop closure's transformation
                                stable_reg_found = true;                                            //first time global registration, then local registration
                            }
                            if (FLAGS_real_time_viewer_on) //for visualization
                            {
                                mviewer.judge_pause();
                                mviewer.set_reg_viewer(true);
                                pcl::transformPointCloud(*cur_map_origin, *cur_map_guess, init_mat);
                                pcl::transformPointCloud(*cur_map_origin, *cur_map_tran, current_registration_edges[j].Trans1_2);
                                mviewer.display_2_pc_compare_realtime(cur_map_guess, hist_map, cur_map_tran, hist_map, reg_viewer, display_time_ms);
                                if (FLAGS_vis_pause_at_loop_closure)
                                    mviewer.keep_visualize(reg_viewer);
                                mviewer.set_reg_viewer(false);
                            }
                        }
                        LOG(INFO) << "map to map registration done\nsubmap [" << current_registration_edges[j].block1->id_in_strip << "] - [" << current_registration_edges[j].block2->id_in_strip << "]:\n"
                                  << current_registration_edges[j].Trans1_2;

                        pcT().swap(*cur_map_origin);
                        pcT().swap(*cur_map_guess);
                        pcT().swap(*cur_map_tran);
                        pcT().swap(*hist_map);
                    }
                    if (reg_edge_successful_count > 0) //apply pose graph optimization (pgo) only when there's correctly registered registration edge
                    {
                        pgoptimizer.set_robust_function(FLAGS_robust_kernel_on);
                        pgoptimizer.set_equal_weight(FLAGS_equal_weight_on);
                        pgoptimizer.set_max_iter_num(FLAGS_max_iter_inter_submap);
                        pgoptimizer.set_diagonal_information_matrix(FLAGS_diagonal_information_matrix_on);
                        pgoptimizer.set_free_node(FLAGS_free_node_on);
                        pgoptimizer.set_problem_size(false);
                        bool pgo_successful;
                        if (!strcmp(FLAGS_pose_graph_optimization_method.c_str(), "g2o"))
                            pgo_successful = pgoptimizer.optimize_pose_graph_g2o(cblock_submaps, pgo_edges);
                        else if (!strcmp(FLAGS_pose_graph_optimization_method.c_str(), "ceres"))
                            pgo_successful = pgoptimizer.optimize_pose_graph_ceres(cblock_submaps, pgo_edges, FLAGS_inter_submap_t_limit, FLAGS_inter_submap_r_limit);
                        else if (!strcmp(FLAGS_pose_graph_optimization_method.c_str(), "gtsam"))
                            pgo_successful = pgoptimizer.optimize_pose_graph_gtsam(cblock_submaps, pgo_edges); //TODO: you'd better use gtsam instead (just like lego-loam)
                        else                                                                                   //default: ceres
                            pgo_successful = pgoptimizer.optimize_pose_graph_ceres(cblock_submaps, pgo_edges, FLAGS_inter_submap_t_limit, FLAGS_inter_submap_r_limit);
                        if (pgo_successful)
                        {
                            pgoptimizer.update_optimized_nodes(cblock_submaps, true, true);        //let pose_lo = = pose_init = pose_optimized && update bbx at the same time
                            cblock_local_map->pose_lo = cblock_submaps[submap_count - 1]->pose_lo; //update current local map's pose
                            cblock_target->pose_lo = cblock_local_map->pose_lo;                    //update target frame
                            cooling_index = FLAGS_cooling_submap_num;                              //wait for several submap (without loop closure detection)
                            for (int k = 0; k < cblock_submaps.size(); k++)
                                cblock_submaps[k]->pose_stable = true;
                            accu_frame_count_wo_opt = 0; //clear
                        }
                    }
                }
                constraints().swap(current_registration_edges);
            }
        }
        std::chrono::steady_clock::time_point toc_loop_closure = std::chrono::steady_clock::now();
        //scan to scan registration
        if (FLAGS_scan_to_scan_module_on || i <= FLAGS_initial_scan2scan_frame_num)
        {
            creg.assign_source_target_cloud(cblock_target, cblock_source, scan2scan_reg_con);
            if (!strcmp(FLAGS_baseline_reg_method.c_str(), "ndt")) //baseline_method 
                creg.omp_ndt(scan2scan_reg_con, FLAGS_reg_voxel_size, FLAGS_ndt_searching_method,
                             initial_guess_tran, FLAGS_reg_intersection_filter_on);
            else if (!strcmp(FLAGS_baseline_reg_method.c_str(), "gicp")) //baseline_method
                creg.omp_gicp(scan2scan_reg_con, max_iteration_num_s2s, reg_corr_dis_thre_init + add_length, FLAGS_voxel_gicp_on,
                              FLAGS_reg_voxel_size, initial_guess_tran, FLAGS_reg_intersection_filter_on);
            else
            {
                int registration_status_scan2scan = creg.mm_lls_icp(scan2scan_reg_con, max_iteration_num_s2s, reg_corr_dis_thre_init + add_length,
                                                                    converge_tran, converge_rot_d, reg_corr_dis_thre_min, dis_thre_update_rate,
                                                                    FLAGS_used_feature_type, FLAGS_corr_weight_strategy, z_xy_balance_ratio,
                                                                    pt2pt_residual_window, pt2pl_residual_window, pt2li_residual_window,
                                                                    initial_guess_tran, FLAGS_reg_intersection_filter_on,
                                                                    motion_com_while_reg_on, FLAGS_normal_shooting_on, FLAGS_normal_bearing,
                                                                    false, false, FLAGS_post_sigma_thre);

                if (registration_status_scan2scan < 0 && FLAGS_real_time_viewer_on) //candidate wrong registration --> use baseline method instead to avoid the crash of the system
                {
                    add_length = 0.8;
                    lo_status_healthy = false;
                    mviewer.keep_visualize(map_viewer); //pause
                }
                else
                    add_length = 1.0;
            }
            if (FLAGS_zupt_on_or_not)
                nav.zupt_simple(scan2scan_reg_con.Trans1_2);
            cblock_source->pose_lo = cblock_target->pose_lo * scan2scan_reg_con.Trans1_2;
            LOG(INFO) << "scan to scan registration done\nframe [" << i - 1 << "] - [" << i << "]:\n"
                      << scan2scan_reg_con.Trans1_2;
            initial_guess_tran = scan2scan_reg_con.Trans1_2;
        }
        //scan to map registration
        if (i % FLAGS_s2m_frequency == 0 && i > FLAGS_initial_scan2scan_frame_num)
        {
            creg.assign_source_target_cloud(cblock_local_map, cblock_source, scan2map_reg_con);

            if (!strcmp(FLAGS_baseline_reg_method.c_str(), "ndt")) //baseline_method
                creg.omp_ndt(scan2map_reg_con, FLAGS_reg_voxel_size, FLAGS_ndt_searching_method,
                             initial_guess_tran, FLAGS_reg_intersection_filter_on);
            if (!strcmp(FLAGS_baseline_reg_method.c_str(), "gicp")) //baseline_method
                creg.omp_gicp(scan2map_reg_con, max_iteration_num_s2s, reg_corr_dis_thre_init + add_length,
                              FLAGS_voxel_gicp_on, FLAGS_reg_voxel_size, initial_guess_tran, FLAGS_reg_intersection_filter_on);
            else
            {
                int registration_status_scan2map = creg.mm_lls_icp(scan2map_reg_con, max_iteration_num_s2m, reg_corr_dis_thre_init + add_length,
                                                                   converge_tran, converge_rot_d, reg_corr_dis_thre_min, dis_thre_update_rate,
                                                                   FLAGS_used_feature_type, FLAGS_corr_weight_strategy, z_xy_balance_ratio,
                                                                   pt2pt_residual_window, pt2pl_residual_window, pt2li_residual_window,
                                                                   initial_guess_tran, FLAGS_reg_intersection_filter_on,
                                                                   motion_com_while_reg_on, FLAGS_normal_shooting_on, FLAGS_normal_bearing,
                                                                   false, false, FLAGS_post_sigma_thre);
                if (registration_status_scan2map < 0 && FLAGS_real_time_viewer_on) //candidate wrong registration
                {
                    add_length = 1.0;
                    lo_status_healthy = false;
                    mviewer.keep_visualize(map_viewer); //pause
                }
                else
                    add_length = 0.0;
            }
            if (FLAGS_zupt_on_or_not)
                nav.zupt_simple(scan2map_reg_con.Trans1_2);
            cblock_source->pose_lo = cblock_local_map->pose_lo * scan2map_reg_con.Trans1_2;
            LOG(INFO) << "scan to map registration done\nframe [" << i << "]:\n"
                      << scan2map_reg_con.Trans1_2;
        }
        adjacent_pose_out = cblock_source->pose_lo.inverse() * cblock_target->pose_lo; //adjacent_pose_out is the transformation from k to k+1 frame (T2_1)

        std::chrono::steady_clock::time_point toc_registration = std::chrono::steady_clock::now();
        if (motion_com_while_reg_on)
        {
            std::chrono::steady_clock::time_point tic_undistortion = std::chrono::steady_clock::now();
            cfilter.apply_motion_compensation(cblock_source->pc_raw, adjacent_pose_out);
            cfilter.batch_apply_motion_compensation(cblock_source->pc_ground, cblock_source->pc_pillar, cblock_source->pc_facade,
                                                    cblock_source->pc_beam, cblock_source->pc_roof, cblock_source->pc_vertex, adjacent_pose_out);
            cfilter.batch_apply_motion_compensation(cblock_source->pc_ground_down, cblock_source->pc_pillar_down, cblock_source->pc_facade_down,
                                                    cblock_source->pc_beam_down, cblock_source->pc_roof_down, cblock_source->pc_vertex, adjacent_pose_out); //TODO: we do not need vertex feature (the curvature of vertex stores the descriptor instead of the timestamp)
            std::chrono::steady_clock::time_point toc_undistortion = std::chrono::steady_clock::now();
            std::chrono::duration<double> time_used_undistortion = std::chrono::duration_cast<std::chrono::duration<double>>(toc_undistortion - tic_undistortion);
            LOG(INFO) << "map motion compensation done in [" << 1000.0 * time_used_undistortion.count() << "] ms.\n";
        }
        std::chrono::steady_clock::time_point toc = std::chrono::steady_clock::now();

        if (FLAGS_real_time_viewer_on) //visualization
        {
            std::chrono::steady_clock::time_point tic_vis = std::chrono::steady_clock::now();
#if OPENCV_ON
            if (FLAGS_show_range_image)
            {
                cv::Mat ri_temp;
                cfilter.pointcloud_to_rangeimage(cblock_target->pc_raw, ri_temp);
                mviewer.display_image(ri_temp, "Range Image", 1, 1);
            }
            if (FLAGS_show_bev_image)
            {
                cv::Mat bev_temp;
                bev_temp = cfilter.generate_2d_map(cblock_target->pc_raw, 4, 400, 400, 2, 50, -5.0, 15.0);
                mviewer.display_image(bev_temp, "BEV Image", 0, 1);
            }
#endif
            pcTPtr pointCloudS_reg(new pcT());
            pcl::transformPointCloud(*cblock_source->pc_raw, *pointCloudS_reg, adjacent_pose_out.inverse().cast<float>());
            pcl::transformPointCloud(*cblock_source->pc_raw, *cblock_source->pc_raw_w, cblock_source->pose_lo); //get raw point cloud in world coordinate system of current frame
            mviewer.display_scan_realtime(cblock_source->pc_raw, scan_viewer, display_time_ms);
            if (i % FLAGS_s2m_frequency == 0)
                mviewer.display_feature_pts_compare_realtime(cblock_local_map, cblock_source, feature_viewer, display_time_ms);
            else
                mviewer.display_feature_pts_compare_realtime(cblock_target, cblock_source, feature_viewer, display_time_ms);
            mviewer.display_2_pc_compare_realtime(cblock_source->pc_raw, cblock_target->pc_raw,
                                                  pointCloudS_reg, cblock_target->pc_raw, reg_viewer, display_time_ms);
            mviewer.display_lo_realtime(cblock_source, map_viewer, display_time_ms, downsamping_rate_scan_vis, FLAGS_vis_map_history_down_rate);
            mviewer.display_dense_map_realtime(cblock_source, map_viewer, FLAGS_vis_map_history_keep_frame_num, display_time_ms);
            mviewer.display_feature_map_realtime(cblock_local_map, map_viewer, display_time_ms);
            if (seg_new_submap)
            {
                mviewer.display_pg_realtime(pgo_edges, map_viewer, display_time_ms);
                mviewer.display_2d_bbx_realtime(cblock_submaps, map_viewer, display_time_ms);
            }
            pcT().swap(*pointCloudS_reg);
            std::chrono::steady_clock::time_point toc_vis = std::chrono::steady_clock::now();
            std::chrono::duration<double> vis_time_used_per_frame = std::chrono::duration_cast<std::chrono::duration<double>>(toc_vis - tic_vis);
            LOG(INFO) << "Render frame [" << i << "] in [" << 1000.0 * vis_time_used_per_frame.count() << "] ms.\n";
        }
        std::chrono::steady_clock::time_point tic_output = std::chrono::steady_clock::now();
        if (i == 1) //write out pose
        {
            dataio.write_lo_pose_overwrite(adjacent_pose_out, output_adjacent_lo_pose_file);
            if (FLAGS_real_time_viewer_on)
                mviewer.is_seed_origin_ = false;
        }
        else
            dataio.write_lo_pose_append(adjacent_pose_out, output_adjacent_lo_pose_file);
        Eigen::Matrix4d pose_lo_body_frame;
        pose_lo_body_frame = calib_mat * cblock_source->pose_lo * calib_mat.inverse();
        dataio.write_lo_pose_append(cblock_source->pose_lo, output_lo_lidar_pose_file); //lo pose in lidar frame
        dataio.write_lo_pose_append(cblock_source->pose_gt, output_gt_lidar_pose_file); //gt pose in lidar frame
        dataio.write_lo_pose_append(pose_lo_body_frame, output_lo_body_pose_file);      //lo pose in body frame (requried by KITTI)
        poses_lo_lidar_cs.push_back(cblock_source->pose_lo);
        if (FLAGS_gt_in_lidar_frame)
            poses_gt_lidar_cs[i] = cblock_source->pose_gt;
        else
            poses_gt_lidar_cs.push_back(cblock_source->pose_gt);
        poses_lo_body_cs.push_back(pose_lo_body_frame);
        poses_lo_adjacent.push_back(adjacent_pose_out); //poses_lo_adjacent is the container of adjacent_pose_out

        //update initial guess
        initial_guess_tran.setIdentity();
        if (initial_guess_mode == 1 && lo_status_healthy)
            initial_guess_tran.block<3, 1>(0, 3) = adjacent_pose_out.inverse().block<3, 1>(0, 3); //uniform motion model
        else if (initial_guess_mode == 2 && lo_status_healthy)
            initial_guess_tran = adjacent_pose_out.inverse();

        //save current frame (only metadata)
        cloudblock_Ptr current_cblock_frame(new cloudblock_t(*cblock_target));
        current_cblock_frame->pose_optimized = current_cblock_frame->pose_lo;
        if (i % FLAGS_s2m_frequency == 0 && i > FLAGS_initial_scan2scan_frame_num) //scan-to-map reg on
            current_cblock_frame->information_matrix_to_next = scan2map_reg_con.information_matrix;
        else //scan-to-map reg off (only scan-to-scan)
            current_cblock_frame->information_matrix_to_next = scan2scan_reg_con.information_matrix;
        cblock_frames.push_back(current_cblock_frame);
        //use this frame as the next iter's target frame
        cblock_target.swap(cblock_source);
        cblock_source->free_all();
        lo_status_healthy = true;
        //update accumulated information
        accu_tran += nav.cal_translation_from_tranmat(adjacent_pose_out);
        accu_rot_deg += nav.cal_rotation_deg_from_tranmat(adjacent_pose_out);
        accu_frame += FLAGS_frame_step;
        current_linear_velocity = nav.cal_velocity(poses_lo_adjacent);
        //report timing
        std::chrono::steady_clock::time_point toc_output = std::chrono::steady_clock::now();
        std::chrono::duration<double> time_used_per_frame_lo_1 = std::chrono::duration_cast<std::chrono::duration<double>>(toc_update_map - toc_import_pc);
        std::chrono::duration<double> time_used_per_frame_lo_2 = std::chrono::duration_cast<std::chrono::duration<double>>(toc_registration - toc_loop_closure);
        std::chrono::duration<double> time_used_per_frame_1 = std::chrono::duration_cast<std::chrono::duration<double>>(toc - tic);
        std::chrono::duration<double> time_used_per_frame_2 = std::chrono::duration_cast<std::chrono::duration<double>>(toc_output - tic_output);
        time_count += (time_used_per_frame_lo_1.count() + time_used_per_frame_lo_2.count());
        LOG(INFO) << "Consuming time of lidar odometry for current frame is [" << 1000.0 * (time_used_per_frame_lo_1.count() + time_used_per_frame_lo_2.count()) << "] ms.\n";
        LOG(INFO) << "Process frame (including data IO) [" << i << "] in [" << 1000.0 * (time_used_per_frame_1.count() + time_used_per_frame_2.count()) << "] ms.\n";
        //record timing
        std::chrono::duration<double> time_used_per_frame_feature_extraction = std::chrono::duration_cast<std::chrono::duration<double>>(toc_feature_extraction - toc_import_pc);
        std::chrono::duration<double> time_used_per_frame_map_updating = std::chrono::duration_cast<std::chrono::duration<double>>(toc_update_map - toc_feature_extraction);
        std::chrono::duration<double> time_used_per_frame_loop_closure = std::chrono::duration_cast<std::chrono::duration<double>>(toc_loop_closure - toc_update_map);
        std::chrono::duration<double> time_used_per_frame_registration = std::chrono::duration_cast<std::chrono::duration<double>>(toc_registration - toc_loop_closure);
        timing_array[i].push_back(time_used_per_frame_feature_extraction.count());
        timing_array[i].push_back(time_used_per_frame_map_updating.count());
        timing_array[i].push_back(time_used_per_frame_registration.count());
        timing_array[i].push_back(time_used_per_frame_loop_closure.count());
    }
    cloudblock_Ptr current_cblock_frame(new cloudblock_t(*cblock_target));
    current_cblock_frame->pose_optimized = current_cblock_frame->pose_lo;
    cblock_frames.push_back(current_cblock_frame);
    LOG(INFO) << "Lidar Odometry done. Average processing time per frame is ["
              << 1000.0 * time_count / frame_num << "] ms over [" << frame_num << "] frames\n";
    if (FLAGS_real_time_viewer_on) {
        mviewer.keep_visualize(map_viewer);
    }

    if (loop_closure_detection_on)
    {
        std::chrono::steady_clock::time_point tic_inner_submap_refine = std::chrono::steady_clock::now();
        if (FLAGS_framewise_pgo_on)//method 1: pgo of all the frame nodes
        {
            pgoptimizer.set_robust_function(FLAGS_robust_kernel_on);
            pgoptimizer.set_equal_weight(FLAGS_equal_weight_on);
            pgoptimizer.set_max_iter_num(FLAGS_max_iter_inner_submap);
            pgoptimizer.set_diagonal_information_matrix(FLAGS_diagonal_information_matrix_on);
            pgoptimizer.set_free_node(FLAGS_free_node_on);
            pgoptimizer.set_problem_size(false); //large size problem
            constraints framewise_pgo_edges;
            cblock_frames[0]->pose_fixed = true; //fix the first frame
            for (int i = 0; i < cblock_frames.size(); i++)
            {
                cblock_frames[i]->id_in_strip = i;
                cblock_frames[i]->pose_init = cblock_frames[i]->pose_lo;
                if (i < cblock_frames.size() - 1)
                {
                    Eigen::Matrix4d tran_mat_12 = poses_lo_adjacent[i].inverse();
                    confinder.add_adjacent_constraint(cblock_frames, framewise_pgo_edges, tran_mat_12, i + 2);
                }
            }
            for (int i = 0; i < pgo_edges.size(); i++)
            {
                if (pgo_edges[i].con_type == REGISTRATION)
                {
                    int frame_idx_1 = cblock_submaps[pgo_edges[i].block1->id_in_strip]->last_frame_index;
                    int frame_idx_2 = cblock_submaps[pgo_edges[i].block2->id_in_strip]->last_frame_index;
                    pgo_edges[i].block1 = cblock_frames[frame_idx_1];
                    pgo_edges[i].block2 = cblock_frames[frame_idx_2];
                    framewise_pgo_edges.push_back(pgo_edges[i]);
                }
            }
            bool inner_submap_optimization_status = false;
            inner_submap_optimization_status = pgoptimizer.optimize_pose_graph_ceres(cblock_frames, framewise_pgo_edges, FLAGS_inner_submap_t_limit, FLAGS_inner_submap_r_limit, false); //set the limit better
            //inner_submap_optimization_status = pgoptimizer.optimize_pose_graph_g2o(cblock_frames, framewise_pgo_edges, false);
            for (int i = 0; i < cblock_frames.size(); i++)
            {
                if (!inner_submap_optimization_status)
                    cblock_frames[i]->pose_optimized = cblock_frames[i]->pose_init;
            }
            constraints().swap(framewise_pgo_edges);
        }
        else
        {
            //method 2: inner-submap pgo (post processing : refine pose within the submap and output final map, update pose of each frame in each submap using pgo)
            pgoptimizer.set_robust_function(false);
            pgoptimizer.set_equal_weight(FLAGS_equal_weight_on);
            pgoptimizer.set_max_iter_num(FLAGS_max_iter_inner_submap);
            pgoptimizer.set_diagonal_information_matrix(FLAGS_diagonal_information_matrix_on);
            pgoptimizer.set_free_node(FLAGS_free_node_on);
            pgoptimizer.set_problem_size(true); //small size problem --> dense schur
            for (int i = 1; i < cblock_submaps.size(); i++)
            {
                cloudblock_Ptrs cblock_frames_in_submap;
                constraints inner_submap_edges;
                cblock_frames[cblock_submaps[i - 1]->last_frame_index]->pose_init = cblock_submaps[i - 1]->pose_lo;
                cblock_frames[cblock_submaps[i - 1]->last_frame_index]->strip_id = cblock_submaps[i]->id_in_strip;
                cblock_frames_in_submap.push_back(cblock_frames[cblock_submaps[i - 1]->last_frame_index]); //end frame of the last submap
                cblock_frames_in_submap[0]->id_in_strip = 0;
                cblock_frames_in_submap[0]->pose_fixed = true; //fix the first frame
                int node_count = 1;
                for (int j = cblock_submaps[i - 1]->last_frame_index; j < cblock_submaps[i]->last_frame_index; j++) //last submap's end frame to this submap's end frame (index j) [last_frame_index store the index of the last frame of the submap]
                {
                    Eigen::Matrix4d tran_mat_12 = poses_lo_adjacent[j].inverse();
                    cblock_frames[j + 1]->id_in_strip = node_count;
                    cblock_frames[j + 1]->strip_id = cblock_submaps[i]->id_in_strip;
                    cblock_frames[j + 1]->pose_init = cblock_frames[j]->pose_init * tran_mat_12;
                    cblock_frames_in_submap.push_back(cblock_frames[j + 1]);
                    node_count++;
                    confinder.add_adjacent_constraint(cblock_frames_in_submap, inner_submap_edges, tran_mat_12, node_count);
                }
                cblock_frames_in_submap[node_count - 1]->pose_fixed = true; //fix the last frame
                cblock_frames_in_submap[node_count - 1]->pose_init = cblock_submaps[i]->pose_lo;
                bool inner_submap_optimization_status = false;
                if (!strcmp(FLAGS_pose_graph_optimization_method.c_str(), "g2o"))
                    inner_submap_optimization_status = pgoptimizer.optimize_pose_graph_g2o(cblock_frames_in_submap, inner_submap_edges, false);
                else if (!strcmp(FLAGS_pose_graph_optimization_method.c_str(), "ceres"))
                    inner_submap_optimization_status = pgoptimizer.optimize_pose_graph_ceres(cblock_frames_in_submap, inner_submap_edges, FLAGS_inner_submap_t_limit, FLAGS_inner_submap_r_limit, false); //set the limit better
                else if (!strcmp(FLAGS_pose_graph_optimization_method.c_str(), "gtsam"))
                    inner_submap_optimization_status = pgoptimizer.optimize_pose_graph_gtsam(cblock_frames_in_submap, inner_submap_edges);
                else //default: ceres
                    inner_submap_optimization_status = pgoptimizer.optimize_pose_graph_ceres(cblock_frames_in_submap, inner_submap_edges, FLAGS_inner_submap_t_limit, FLAGS_inner_submap_r_limit, false);
                for (int j = cblock_submaps[i - 1]->last_frame_index + 1; j <= cblock_submaps[i]->last_frame_index; j++)
                {
                    if (inner_submap_optimization_status)
                        cblock_frames[j]->pose_optimized = cblock_frames_in_submap[cblock_frames[j]->id_in_strip]->pose_optimized;
                    else
                        cblock_frames[j]->pose_optimized = cblock_frames[j]->pose_init;
                }
                constraints().swap(inner_submap_edges);
                cloudblock_Ptrs().swap(cblock_frames_in_submap);
                LOG(INFO) << "Inner-submap pose refining done for submap [" << i << "]";
            }
        }
        std::chrono::steady_clock::time_point toc_inner_submap_refine = std::chrono::steady_clock::now();
        std::chrono::duration<double> time_used_inner_submap_refine = std::chrono::duration_cast<std::chrono::duration<double>>(toc_inner_submap_refine - tic_inner_submap_refine);
        LOG(INFO) << "Inner-submap pose refinement done in [" << 1000 * time_used_inner_submap_refine.count() << "] ms.";
    }
    if (loop_closure_detection_on)
    {
        for (int i = 0; i < frame_num; i++) //update poses
        {
            poses_lo_lidar_cs[i] = cblock_frames[i]->pose_optimized;
            if (i == 0) //write optimized pose out (overwrite)
            {
                dataio.write_lo_pose_overwrite(poses_lo_lidar_cs[0], output_lo_lidar_pose_file);
                dataio.write_lo_pose_overwrite(poses_lo_body_cs[0], output_lo_body_pose_file); //required by KITTI
            }
            else
            {
                poses_lo_body_cs[i] = calib_mat * poses_lo_lidar_cs[i] * calib_mat.inverse(); //what we want actually is the Tb0_bi
                dataio.write_lo_pose_append(poses_lo_lidar_cs[i], output_lo_lidar_pose_file);
                dataio.write_lo_pose_append(poses_lo_body_cs[i], output_lo_body_pose_file); //required by KITTI
            }
        }
        for (int i = 0; i < cblock_submaps.size(); i++) //free submaps' point cloud
            cblock_submaps[i]->free_all();
        constraints().swap(pgo_edges);
    }
    if (FLAGS_real_time_viewer_on) {
        map_viewer->removeAllPointClouds(); // refresh the map
        mviewer.update_lo_pose(poses_lo_lidar_cs, poses_gt_lidar_cs, map_viewer, display_time_ms);
        mviewer.update_submap_node(cblock_submaps, map_viewer, display_time_ms);
        mviewer.keep_visualize(map_viewer);
    }
    if (FLAGS_write_out_map_on || FLAGS_write_out_gt_map_on) //export map point cloud //TODO: be careful of the memory problem here!!! //FIX memory leakage while outputing map point cloud
    {
        LOG(WARNING) << "Begin to output the generated map";
        pcTPtr pc_map_merged(new pcT), pc_map_gt_merged(new pcT);
        for (int i = 0; i < frame_num; i++) //output merged map (with dist_filter, intrinsic correction and motion distortion
        {
            dataio.read_pc_cloud_block(cblock_frames[i]);
            if (FLAGS_vertical_ang_calib_on) //intrinsic angle correction
                cfilter.vertical_intrinsic_calibration(cblock_frames[i]->pc_raw, FLAGS_vertical_ang_correction_deg);
            if (FLAGS_apply_dist_filter)
                cfilter.dist_filter(cblock_frames[i]->pc_raw, FLAGS_min_dist_mapping, FLAGS_max_dist_mapping);
            cfilter.random_downsample(cblock_frames[i]->pc_raw, FLAGS_map_downrate_output);
            if (FLAGS_motion_compensation_method == 1) //calculate from time-stamp
                cfilter.get_pts_timestamp_ratio_in_frame(cblock_frames[i]->pc_raw, true);
            else if (FLAGS_motion_compensation_method == 2)                                      //calculate from azimuth
                cfilter.get_pts_timestamp_ratio_in_frame(cblock_frames[i]->pc_raw, false, 90.0); //HESAI Lidar: 90.0 (y+ axis, clockwise), Velodyne Lidar: 180.0
            if (FLAGS_write_out_map_on)
            {
                if (FLAGS_motion_compensation_method > 0 && i > 0)
                {
                    Eigen::Matrix4d adjacent_tran = cblock_frames[i]->pose_optimized.inverse() * cblock_frames[i - 1]->pose_optimized;
                    cfilter.apply_motion_compensation(cblock_frames[i]->pc_raw, adjacent_tran);
                }
                pcl::transformPointCloud(*cblock_frames[i]->pc_raw, *cblock_frames[i]->pc_raw_w, cblock_frames[i]->pose_optimized);
                if (FLAGS_write_map_each_frame)
                {
                    std::string filename_without_path = cblock_frames[i]->filename.substr(cblock_frames[i]->filename.rfind('/') + 1);
                    std::string filename_without_extension = filename_without_path.substr(0, filename_without_path.rfind('.'));
                    std::string output_pc_file = output_pc_folder + "/" + filename_without_extension + ".pcd";
                    dataio.write_cloud_file(output_pc_file, cblock_frames[i]->pc_raw_w);
                }
                pc_map_merged->points.insert(pc_map_merged->points.end(), cblock_frames[i]->pc_raw_w->points.begin(), cblock_frames[i]->pc_raw_w->points.end());
                cfilter.random_downsample(cblock_frames[i]->pc_raw_w, 2);
                if (FLAGS_real_time_viewer_on) {
                    mviewer.display_dense_map_realtime(cblock_frames[i], map_viewer, frame_num, display_time_ms);
                }
            }
            if (FLAGS_write_out_gt_map_on)
            {
                if (FLAGS_motion_compensation_method > 0 && i > 0)
                {
                    Eigen::Matrix4d adjacent_tran = cblock_frames[i]->pose_gt.inverse() * cblock_frames[i - 1]->pose_gt;
                    cfilter.apply_motion_compensation(cblock_frames[i]->pc_raw, adjacent_tran);
                }
                pcl::transformPointCloud(*cblock_frames[i]->pc_raw, *cblock_frames[i]->pc_sketch, cblock_frames[i]->pose_gt);
                pc_map_gt_merged->points.insert(pc_map_gt_merged->points.end(), cblock_frames[i]->pc_sketch->points.begin(), cblock_frames[i]->pc_sketch->points.end());
            }
            cblock_frames[i]->free_all();
        }
        if (FLAGS_map_filter_on)                        //TODO: add more map based operation //1.generate 2D geo-referenced image //2.intensity generalization
            cfilter.sor_filter(pc_map_merged, 20, 2.0); //sor filtering before output
        std::string output_merged_map_file = output_pc_folder + "/" + "merged_map.pcd";
        if (FLAGS_write_out_map_on)
            dataio.write_pcd_file(output_merged_map_file, pc_map_merged); //write merged map point cloud
        std::string output_merged_gt_map_file = output_pc_folder + "/" + "merged_gt_map.pcd";
        if (FLAGS_write_out_gt_map_on)
            dataio.write_pcd_file(output_merged_gt_map_file, pc_map_gt_merged); //write merged map point cloud
#if OPENCV_ON
        if (FLAGS_write_out_map_on)
        {
            cv::Mat map_2d;
            map_2d = cfilter.generate_2d_map(pc_map_merged, 1, FLAGS_screen_width, FLAGS_screen_height, 20, 1000, -FLT_MAX, FLT_MAX, true);
            if (FLAGS_show_bev_image && FLAGS_real_time_viewer_on)
                mviewer.display_image(map_2d, "2D Map");
            std::string output_merged_map_image = output_pc_folder + "/" + "merged_map_2d.png";
            cv::imwrite(output_merged_map_image, map_2d);
        }
#endif
        pcT().swap(*pc_map_merged);
        pcT().swap(*pc_map_gt_merged);
    }
    dataio.report_consuming_time(FLAGS_timing_report_file, timing_array);
    dataio.write_pose_point_cloud(lo_lidar_pose_point_cloud_file, poses_lo_lidar_cs);
    dataio.write_pose_point_cloud(gt_lidar_pose_point_cloud_file, poses_gt_lidar_cs);
    OdomErrorCompute ec;
    std::vector<odom_errors_t> odom_errs, slam_errors;
    if (poses_gt_body_cs.size() > 0) //gt pose is availiable
    {
        poses_gt_body_cs[0].setIdentity();
        if (FLAGS_gt_in_lidar_frame)
            odom_errs = ec.compute_error(poses_gt_lidar_cs, poses_lo_lidar_cs);
        else
            odom_errs = ec.compute_error(poses_gt_body_cs, poses_lo_body_cs);
        ec.print_error(odom_errs);
        if (loop_closure_detection_on)
        {
            if (FLAGS_gt_in_lidar_frame)
                slam_errors = ec.compute_error(poses_gt_lidar_cs, poses_lo_lidar_cs, true); //longer segments, better for the evaluation of global localization performance
            else
                slam_errors = ec.compute_error(poses_gt_body_cs, poses_lo_body_cs, true); //longer segments
            ec.print_error(slam_errors, true);
        }
    }
    if (FLAGS_real_time_viewer_on) {
        map_viewer->removeAllShapes();
        mviewer.keep_visualize(map_viewer);
    }
    return 0;
}

//TODO LIST (potential future work)
//TODO: add ros support
//TODO: use deep learning based methods to do global registration (for loop closure)
//TODO: add the on-fly calibration of intrinsic parameters (intrinsic angle displacement)
//TODO: code refactoring from scratch
