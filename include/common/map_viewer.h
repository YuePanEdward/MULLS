//
// This file is for the general implementation of visualization based on pcl viewer
// Dependent 3rd Libs: PCL (>1.7)
// by Yue Pan
//

#ifndef _INCLUDE_MAP_VIEWER_H
#define _INCLUDE_MAP_VIEWER_H

#include <string>
#include <fstream>
#include <vector>

//pcl
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/visualization/common/common.h>
#include <pcl/visualization/pcl_visualizer.h>

#if OPENCV_ON
//opencv
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#endif

#include <glog/logging.h>

#include <boost/thread/thread.hpp>

#include "utility.hpp"

namespace lo
{

// Visualization color scale
enum color_type
{
    SINGLE = 0,
    FRAME = 1,
    HEIGHT = 2,
    INTENSITY = 3,
    INTENSITY_2 = 4
};

template <typename PointT>
class MapViewer
{
  public:
    //Constructor

    MapViewer()
    {
        refreshing_time_ms_ = 1;
        sleep_micro_second_ = 100;
    };

    MapViewer(float initial_background_color, bool map_window_on = true,
              bool reg_window_on = false, bool feature_window_on = false,
              bool scan_window_on = false,
              float intensity_scale = 255.0, int used_color_style = 0,
              float laser_visualization_size = 0.5,
              int initial_downsample_ratio = 2)
    {
        is_frist_epoch_ = true;
        is_seed_origin_ = true;
        is_paused_ = false;
        show_gt_trajectory_ = true;
        enabled_jump_ = false;
        show_dense_map_ = false;
        show_pose_graph_edge_ = true;
        show_pose_graph_node_ = false;
        show_feature_map_ = false;
        show_point_normal_ = false;
        reset_camera_ = false;
        show_distance_circles_ = false;
        show_correspondences_ = true;
        show_vertex_keypoints_ = false;

        show_feature_ = feature_window_on;
        show_reg_ = reg_window_on;
        show_sparse_map_ = map_window_on;
        show_current_scan_ = scan_window_on;
        color_rendering_type_ = (color_type)(used_color_style);
        background_color_ = initial_background_color;
        laser_visualization_size_ = laser_visualization_size;

        downsample_ratio_vis_ = initial_downsample_ratio;

        intensity_scale_ = intensity_scale;
        color_scale_ = 1.5;
        sample_frame_rate_ = 2; //keep just one frame's downsampled point cloud in 2 frames
        refreshing_time_ms_ = 1;
        sleep_micro_second_ = 100;

        lo_pose_pc_ = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
        gt_pose_pc_ = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
    };

    ~MapViewer(){};

    void get_random_color(float &r, float &g, float &b, float range_max) //range_max example 1,255...
    {
        r = range_max * (rand() / (1.0 + RAND_MAX));
        g = range_max * (rand() / (1.0 + RAND_MAX));
        b = range_max * (rand() / (1.0 + RAND_MAX));
    }

    void judge_pause(boost::shared_ptr<pcl::visualization::PCLVisualizer> &viewer, int display_time_ms)
    {
        //LOG(ERROR) << is_paused_;
        if (is_paused_)
        {
            // pcl::visualization::Camera camera;
            // viewer->getCameraParameters(camera);
            // printf("%lf,%lf,%lf,", camera.pos[0], camera.pos[1], camera.pos[2]);
            // printf("%lf,%lf,%lf\n", camera.view[0], camera.view[1], camera.view[2]);
            while (!viewer->wasStopped() && is_paused_)
            {
                viewer->spinOnce(refreshing_time_ms_);
                boost::this_thread::sleep(boost::posix_time::microseconds(sleep_micro_second_));
            }
        }
        else
        {
            //LOG(ERROR)<<display_time_ms<<" ms";
            viewer->spinOnce(display_time_ms);
            boost::this_thread::sleep(boost::posix_time::microseconds(sleep_micro_second_));
        }
    }

    bool jump_to_frame(int &i)
    {
        //std::cout << enabled_jump_ << std::endl;

        if (enabled_jump_)
        {
            std::cout << "Please enter the frame number to which you'd like to jump:\n";
            int input_num;
            cin >> input_num;
            i = input_num;
            enabled_jump_ = false;
            is_paused_ = true;
            return true;
        }
        // else
        //std::cout << "You can't jump\n";
        return false;
    }

    void set_pause(bool is_pasue = 0) //default -> resume
    {
        is_paused_ = is_pasue;
    }

    void set_reg_viewer(bool show_reg_or_not)
    {
        show_reg_ = show_reg_or_not;
    }

    void set_feature_viewer(bool show_feature_or_not)
    {
        show_feature_ = show_feature_or_not;
    }

    void judge_pause()
    {
        if (pause_at_loop_)
            is_paused_ = true;
    }

    //Interactions
    static void mouse_event_occurred(const pcl::visualization::MouseEvent &event,
                                     void *viewer_void);

    static void keyboard_event_occurred(const pcl::visualization::KeyboardEvent &event,
                                        void *viewer_void);

    void set_interactive_events(boost::shared_ptr<pcl::visualization::PCLVisualizer> &viewer,
                                int window_length = 1920, int window_height = 1080, bool seg_viewport = false);

    void keep_visualize(boost::shared_ptr<pcl::visualization::PCLVisualizer> &viewer);

    void assign_color_by_class(int category, int &r, int &g, int &b);

    void apply_label_color_mapping(int semantic_kitti_label, int &r, int &g, int &b);

    void display_2d_bbx_realtime(cloudblock_Ptrs &blocks,
                                 boost::shared_ptr<pcl::visualization::PCLVisualizer> &viewer,
                                 int display_time_ms = 100);

    void display_pg_realtime(const constraints &cons,
                             boost::shared_ptr<pcl::visualization::PCLVisualizer> &viewer,
                             int display_time_ms = 100);

    void display_scan_realtime(const typename pcl::PointCloud<PointT>::Ptr &pc, boost::shared_ptr<pcl::visualization::PCLVisualizer> &viewer, int display_time_ms = 50);

    void display_2_pc(const typename pcl::PointCloud<PointT>::Ptr &Cloud1,
                      const typename pcl::PointCloud<PointT>::Ptr &Cloud2,
                      std::string displayname, int display_downsample_ratio);

    void display_2_pc_compare(const typename pcl::PointCloud<PointT>::Ptr &Cloud1_left,
                              const typename pcl::PointCloud<PointT>::Ptr &Cloud2_left,
                              const typename pcl::PointCloud<PointT>::Ptr &Cloud1_right,
                              const typename pcl::PointCloud<PointT>::Ptr &Cloud2_right,
                              std::string displayname, int display_downsample_ratio = 1);

    void display_feature_pts_compare_realtime(cloudblock_Ptr &in_block_tc, cloudblock_Ptr &in_block_sc,
                                              boost::shared_ptr<pcl::visualization::PCLVisualizer> &viewer,
                                              int display_time_ms = 50);

    void display_2_pc_compare_realtime(const typename pcl::PointCloud<PointT>::Ptr &Cloud1_left, const typename pcl::PointCloud<PointT>::Ptr &Cloud2_left,
                                       const typename pcl::PointCloud<PointT>::Ptr &Cloud1_right, const typename pcl::PointCloud<PointT>::Ptr &Cloud2_right,
                                       boost::shared_ptr<pcl::visualization::PCLVisualizer> &viewer, int display_time_ms = 50);

    void display_lo_realtime(cloudblock_Ptr &current_frame,
                             boost::shared_ptr<pcl::visualization::PCLVisualizer> &viewer,
                             int display_time_ms = 50, int display_downsample_ratio_current = 2, int display_downsample_ratio_history = 400);

    void update_lo_pose(Matrix4ds &poses_lo, Matrix4ds &poses_gt,
                        boost::shared_ptr<pcl::visualization::PCLVisualizer> &viewer,
                        int display_time_ms = 50);

    void update_submap_node(cloudblock_Ptrs &submaps, boost::shared_ptr<pcl::visualization::PCLVisualizer> &viewer,
                            int display_time_ms = 50);

    void display_dense_map_realtime(cloudblock_Ptr &current_frame,
                                    boost::shared_ptr<pcl::visualization::PCLVisualizer> &viewer,
                                    int keep_frame_num = 100, int display_time_ms = 50);

    void display_feature_map_realtime(cloudblock_Ptr &local_map,
                                      boost::shared_ptr<pcl::visualization::PCLVisualizer> &viewer,
                                      int display_time_ms = 50);

    void display_distance_circle(boost::shared_ptr<pcl::visualization::PCLVisualizer> &viewer);

    void add_lines_to_viewer(boost::shared_ptr<pcl::visualization::PCLVisualizer> &viewer,
                             const typename pcl::PointCloud<PointT>::Ptr pts_1, const typename pcl::PointCloud<PointT>::Ptr pts_2,
                             const std::string lines_name,
                             const std::vector<unsigned int> &red, const std::vector<unsigned int> &green, const std::vector<unsigned int> &blue,
                             const int view_port = 0, int down_rate = 1);

    void add_triangles_to_viewer(boost::shared_ptr<pcl::visualization::PCLVisualizer> &viewer,
                                 const typename pcl::PointCloud<PointT>::Ptr pts_1, const typename pcl::PointCloud<PointT>::Ptr pts_2, const typename pcl::PointCloud<PointT>::Ptr pts_3,
                                 const std::string lines_name,
                                 const std::vector<unsigned int> &red, const std::vector<unsigned int> &green, const std::vector<unsigned int> &blue,
                                 const int view_port = 0);

    void display_pc_with_bbx_realtime(boost::shared_ptr<pcl::visualization::PCLVisualizer> &viewer,
                                      const typename pcl::PointCloud<PointT>::Ptr &pc_0,
                                      const typename pcl::PointCloud<PointT>::Ptr &pc_1,
                                      const typename pcl::PointCloud<PointT>::Ptr &bbx_vertex,
                                      const std::vector<int> &bbx_class,
                                      int display_time_ms = 50);

    void display_cross_section_compare_realtime(std::vector<typename pcl::PointCloud<PointT>::Ptr> &cross_cloud_S,
                                                std::vector<typename pcl::PointCloud<PointT>::Ptr> &cross_cloud_T,
                                                std::vector<typename pcl::PointCloud<PointT>::Ptr> &cross_cloud_SR,
                                                boost::shared_ptr<pcl::visualization::PCLVisualizer> &viewer,
                                                int display_time_ms = 50, int display_downsample_ratio = 1);

    void display_feature_pts_compare(cloudblock_t &in_block_tc, cloudblock_t &in_block_sc, std::string displayname);

    void highlight_problematic_frame(boost::shared_ptr<pcl::visualization::PCLVisualizer> &viewer,
                                     bool horizontal_flag = false, bool vertical_flag = false, bool heading_flag = false, bool pause_or_not = false);

    void display_map_plane(boost::shared_ptr<pcl::visualization::PCLVisualizer> &viewer, const std::string &plane_name, float z_position);

    void display_correspondences(boost::shared_ptr<pcl::visualization::PCLVisualizer> &viewer,
                                 const typename pcl::PointCloud<PointT>::Ptr &kpc_source, const typename pcl::PointCloud<PointT>::Ptr &kpc_target,
                                 const typename pcl::PointCloud<PointT>::Ptr &pc_source, const typename pcl::PointCloud<PointT>::Ptr &pc_target,
                                 Eigen::Matrix4d &pre_tran, int display_time_ms = 50);

    void display_correspondences_compare(boost::shared_ptr<pcl::visualization::PCLVisualizer> &viewer,
                                         const typename pcl::PointCloud<PointT>::Ptr &kpc_source,
                                         const typename pcl::PointCloud<PointT>::Ptr &kpc_target,
                                         const typename pcl::PointCloud<PointT>::Ptr &kpc_source_tran,
                                         const typename pcl::PointCloud<PointT>::Ptr &pc_source,
                                         const typename pcl::PointCloud<PointT>::Ptr &pc_target,
                                         const typename pcl::PointCloud<PointT>::Ptr &pc_source_tran,
                                         Eigen::Matrix4d &pre_tran, int down_rate = 1, int display_time_ms = 50);

    // mapping from [0,1] to [0,1]
    float simple_look_up_table(float x, float x_a, float x_b, float y_a, float y_b) //TODO: speed up
    {
        float y;
        float k1 = y_a / x_a;
        float k2 = (y_b - y_a) / (x_b - x_a);
        float k3 = (1.0 - y_b) / (1.0 - x_b);

        if (x <= x_a)
            y = k1 * x;
        else if (x <= x_b && x > x_a)
            y = y_a + k2 * (x - x_a);
        else if (x <= 1.0 && x > x_b)
            y = y_b + k3 * (x - x_b);
        else
            y = 0.;

        //LOG(INFO) << x << " - " << y;

        return y;
    }

#if OPENCV_ON
    void display_image(cv::Mat &image, const std::string image_viewer_name, int color_scale = 0, int time_delay_ms = 0);
#endif

    void show_help()
    {
        std::cout << "| Help:\n-------\n"
                  << "          Space  : pause/resume the playing visualization\n\n"
                  << "          F1     : switch to single color (golden)\n"
                  << "          F2     : switch to frame-wise random color\n"
                  << "          F3     : switch to color map with regard to elevation (z value)\n"
                  << "          F4     : switch to color map with regard to intensity (i value)\n\n"
                  << "          F7     : turn on/off the visualization of the pose graph (nodes and edges)\n"
                  << "          F8     : turn on/off the visualization of local feature map\n"
                  << "          F9     : turn on/off the visualization of dense point cloud map\n"
                  << "          F10    : turn on/off the visualization of feature points used for registration\n"
                  << "          F11    : turn on/off the visualization of pairwise registration \n"
                  << "          F12    : turn on/off the visualization of pose graph\n\n"
                  << "          T      : turn on/off the visualization of ground truth trajectory\n"
                  << "          G      : turn on/off the plotting scale\n\n"
                  << "          Up     : decrease the downsampling ratio (display denser points)\n"
                  << "          Down   : increase the downsampling ratio (display sparser points)\n"
                  << "          Left   : decrease the brightness of the background\n"
                  << "          Right  : increase the brightness of the background\n";
        is_paused_ = true;
    }

    bool is_seed_origin_;

  private:
    static bool is_paused_;
    static bool enabled_jump_;

    static bool show_feature_;
    static bool show_reg_;
    static bool show_sparse_map_;
    static bool show_dense_map_;
    static bool show_feature_map_;
    static bool show_point_normal_;
    static bool show_pose_graph_edge_;
    static bool show_pose_graph_node_;
    static bool show_gt_trajectory_;
    static bool reset_camera_;
    static bool show_distance_circles_;
    static bool show_correspondences_;
    static bool pause_at_loop_;
    static bool show_current_scan_;
    static bool show_vertex_keypoints_;

    static color_type color_rendering_type_;
    static double background_color_;
    static int downsample_ratio_vis_;

    float intensity_scale_;
    bool is_frist_epoch_;
    int refreshing_time_ms_;
    int sleep_micro_second_;
    int sample_frame_rate_;
    float color_scale_;
    float laser_visualization_size_;

    float intensity_lut_x_a_ = 0.65;
    float intensity_lut_x_b_ = 0.97;
    float intensity_lut_y_a_ = 0.05;
    float intensity_lut_y_b_ = 0.6;

    //for lidar odometry visualization
    typename pcl::PointCloud<pcl::PointXYZ>::Ptr lo_pose_pc_;
    typename pcl::PointCloud<pcl::PointXYZ>::Ptr gt_pose_pc_;
};
} // namespace lo

#include "map_viewer.hpp"

#endif //_INCLUDE_MAP_VIEWER_H
