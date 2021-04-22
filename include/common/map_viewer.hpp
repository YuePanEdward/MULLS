//
// This file is for the general implementation of visualization based on pcl viewer
// Dependent 3rd Libs: PCL (>1.7)
// By Yue Pan 
//

#ifndef _INCLUDE_MAP_VIEWER_HPP
#define _INCLUDE_MAP_VIEWER_HPP

//pcl
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/common/common.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/point_cloud_color_handlers.h>
#include <pcl/console/parse.h>
#include <pcl/common/common_headers.h>

//VTK include needed for drawing graph lines
#include <vtkPolyLine.h>
#include <vtkLine.h>

#if OPENCV_ON
//opencv
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#endif

using namespace lo;

template <typename PointT>
bool MapViewer<PointT>::is_paused_;

template <typename PointT>
bool MapViewer<PointT>::enabled_jump_;

template <typename PointT>
bool MapViewer<PointT>::show_feature_;

template <typename PointT>
bool MapViewer<PointT>::show_reg_;

template <typename PointT>
bool MapViewer<PointT>::show_sparse_map_;

template <typename PointT>
bool MapViewer<PointT>::show_dense_map_;

template <typename PointT>
bool MapViewer<PointT>::show_feature_map_;

template <typename PointT>
bool MapViewer<PointT>::show_point_normal_;

template <typename PointT>
bool MapViewer<PointT>::show_pose_graph_edge_;

template <typename PointT>
bool MapViewer<PointT>::show_pose_graph_node_;

template <typename PointT>
bool MapViewer<PointT>::show_gt_trajectory_;

template <typename PointT>
bool MapViewer<PointT>::show_correspondences_;

template <typename PointT>
bool MapViewer<PointT>::reset_camera_;

template <typename PointT>
bool MapViewer<PointT>::show_distance_circles_;

template <typename PointT>
bool MapViewer<PointT>::pause_at_loop_;

template <typename PointT>
bool MapViewer<PointT>::show_current_scan_;

template <typename PointT>
bool MapViewer<PointT>::show_vertex_keypoints_;

template <typename PointT>
color_type MapViewer<PointT>::color_rendering_type_;

template <typename PointT>
double MapViewer<PointT>::background_color_;

template <typename PointT>
int MapViewer<PointT>::downsample_ratio_vis_;

template <typename PointT>
void MapViewer<PointT>::keyboard_event_occurred(const pcl::visualization::KeyboardEvent &event,
                                                void *viewer_void)
{
    pcl::visualization::PCLVisualizer *viewer = static_cast<pcl::visualization::PCLVisualizer *>(viewer_void);

    if (event.keyDown())
    {
        if (event.getKeySym() == "space")
        {
            if (!is_paused_)
            {
                std::cout << "[space] was pressed => pause" << std::endl;
                is_paused_ = true;
            }
            else
            {
                std::cout << "[space] was pressed => resume" << std::endl;
                is_paused_ = false;
            }
        }
        else if (event.getKeySym() == "F1")
        {
            std::cout << "[F1] was pressed => use single color mode" << std::endl;
            color_rendering_type_ = SINGLE;
        }
        else if (event.getKeySym() == "F2")
        {
            std::cout << "[F2] was pressed => use frame color mode" << std::endl;
            color_rendering_type_ = FRAME;
        }
        else if (event.getKeySym() == "F3")
        {
            std::cout << "[F3] was pressed => use height color mode" << std::endl;
            color_rendering_type_ = HEIGHT;
        }
        else if (event.getKeySym() == "F4")
        {
            std::cout << "[F4] was pressed => use intensity color mode" << std::endl;
            color_rendering_type_ = INTENSITY;
        }
        else if (event.getKeySym() == "F5")
        {
            std::cout << "[F5] was pressed => use intensity_2 color mode" << std::endl;
            color_rendering_type_ = INTENSITY_2;
        }
        else if (event.getKeySym() == "F6")
        {
            if (!show_current_scan_)
            {
                std::cout << "[F6] was pressed => current scan point cloud visualization on" << std::endl;
                show_current_scan_ = true;
            }
            else
            {
                std::cout << "[F7] was pressed => current scan point cloud visualization off" << std::endl;
                show_current_scan_ = false;
            }
        }
        else if (event.getKeySym() == "F7")
        {
            if (!show_pose_graph_node_)
            {
                std::cout << "[F7] was pressed => global pose graph visualization on" << std::endl;
                show_pose_graph_node_ = true;
            }
            else
            {
                std::cout << "[F7] was pressed => global pose graph visualization off" << std::endl;
                show_pose_graph_node_ = false;
            }
        }
        else if (event.getKeySym() == "F8")
        {
            if (!show_feature_map_)
            {
                std::cout << "[F8] was pressed => local feature map visualization on" << std::endl;
                show_feature_map_ = true;
            }
            else
            {
                std::cout << "[F8] was pressed => local feature map visualization off" << std::endl;
                show_feature_map_ = false;
            }
        }
        else if (event.getKeySym() == "F9")
        {
            if (!show_dense_map_)
            {
                std::cout << "[F9] was pressed => local dense map visualization on" << std::endl;
                show_dense_map_ = true;
            }
            else
            {
                std::cout << "[F9] was pressed => local dense map visualization off" << std::endl;
                show_dense_map_ = false;
            }
        }
        else if (event.getKeySym() == "F10")
        {
            if (!show_feature_)
            {
                std::cout << "[F10] was pressed => feature points visualization on" << std::endl;
                show_feature_ = true;
            }
            else
            {
                std::cout << "[F10] was pressed => feature points visualization off" << std::endl;
                show_feature_ = false;
            }
        }
        else if (event.getKeySym() == "F11")
        {
            if (!show_reg_)
            {
                std::cout << "[F11] was pressed => registration visualization on" << std::endl;
                show_reg_ = true;
            }
            else
            {
                std::cout << "[F11] was pressed => registration visualization off" << std::endl;
                show_reg_ = false;
            }
        }
        else if (event.getKeySym() == "F12")
        {
            if (!show_pose_graph_edge_)
            {
                std::cout << "[F12] was pressed => pose graph visualization on" << std::endl;
                show_pose_graph_edge_ = true;
            }
            else
            {
                std::cout << "[F12] was pressed => pose graph visualization off" << std::endl;
                show_pose_graph_edge_ = false;
            }
        }
        else if (event.getKeySym() == "z")
        {
            if (!show_sparse_map_)
            {
                std::cout << "[z] was pressed => show the sparse map" << std::endl;
                show_sparse_map_ = true;
            }
            else
            {
                std::cout << "[z] was pressed => do not show the sparse map" << std::endl;
                show_sparse_map_ = false;
            }
        }
        else if (event.getKeySym() == "Up")
        {
            downsample_ratio_vis_ = max_(downsample_ratio_vis_ - 1, 1);
            std::cout << "[Up] was pressed => render more denser point cloud" << std::endl;
        }
        else if (event.getKeySym() == "Down")
        {
            downsample_ratio_vis_ += 1;
            std::cout << "[Down] was pressed => render more sparse point cloud" << std::endl;
        }
        else if (event.getKeySym() == "Left")
        {
            background_color_ = max_(background_color_ - 0.1, 0);
            std::cout << "[Left] was precessed => darker background color " << std::endl;
        }
        else if (event.getKeySym() == "Right")
        {
            background_color_ = min_(background_color_ + 0.1, 1.0);
            std::cout << "[Right] was precessed => brighter background color " << std::endl;
        }
        else if (event.getKeySym() == "l")
        {
            if (!pause_at_loop_)
            {
                std::cout << "[l] was pressed => pause at the loop" << std::endl;
                pause_at_loop_ = true;
            }
            else
            {
                std::cout << "[l] was pressed => ban the pause at the loop" << std::endl;
                pause_at_loop_ = false;
            }
        }
        else if (event.getKeySym() == "k")
        {
            if (!enabled_jump_)
            {
                std::cout << "[k] was pressed => enable the jump to x frame function" << std::endl;
                enabled_jump_ = true;
            }
            else
            {
                std::cout << "[k] was pressed => ban the jump to x frame function" << std::endl;
                enabled_jump_ = false;
            }
        }
        else if (event.getKeySym() == "n")
        {
            if (!show_point_normal_)
            {
                std::cout << "[n] was pressed => point normal visualization on" << std::endl;
                show_point_normal_ = true;
            }
            else
            {
                std::cout << "[n] was pressed => point normal visualization off" << std::endl;
                show_point_normal_ = false;
            }
        }
        else if (event.getKeySym() == "y")
        {
            if (!reset_camera_)
            {
                std::cout << "[y] was pressed => reset camera" << std::endl;
                reset_camera_ = true;
            }
        }
        else if (event.getKeySym() == "s")
        {
            if (!show_distance_circles_)
            {
                std::cout << "[s] was pressed => circle ring visualization on" << std::endl;
                show_distance_circles_ = true;
            }
            else
            {
                std::cout << "[s] was pressed => circle ring visualization off" << std::endl;
                show_distance_circles_ = false;
            }
        }
        else if (event.getKeySym() == "t")
        {
            if (!show_gt_trajectory_)
            {
                std::cout << "[t] was pressed => ground truth trajectory (if availiable) visualization on" << std::endl;
                show_gt_trajectory_ = true;
            }
            else
            {
                std::cout << "[t] was pressed => ground truth trajectory (if availiable) visualization off" << std::endl;
                show_gt_trajectory_ = false;
            }
        }
        else if (event.getKeySym() == "v")
        {
            if (!show_vertex_keypoints_)
            {
                std::cout << "[v] was pressed => vertex keypoints visualization on" << std::endl;
                show_vertex_keypoints_ = true;
            }
            else
            {
                std::cout << "[v] was pressed => vertex keypoints visualization off" << std::endl;
                show_vertex_keypoints_ = false;
            }
        }
        else if (event.getKeySym() == "h")
        {
            //reference (some more pcl visualizer built-in keyboard interactions): https://blog.csdn.net/joker_hapy/article/details/86165623
            std::cout << "| Help:\n-------\n"
                      << "          Space  : pause/resume the playing visualization\n\n"
                      << "          F1     : switch to single color (golden)\n"
                      << "          F2     : switch to frame-wise random color\n"
                      << "          F3     : switch to color map with regard to elevation (z value)\n"
                      << "          F4     : switch to color map with regard to intensity (i value)\n\n"
                      << "          F5     : switch to color map with regard to intensity (i value) [mode 2]\n\n"
                      << "          F6     : turn on/off the visualization of current scan \n"
                      << "          F7     : turn on/off the visualization of the pose graph (nodes)\n"
                      << "          F8     : turn on/off the visualization of local feature map\n"
                      << "          F9     : turn on/off the visualization of dense point cloud map\n"
                      << "          F10    : turn on/off the visualization of feature points used for registration\n"
                      << "          F11    : turn on/off the visualization of pairwise registration \n"
                      << "          F12    : turn on/off the visualization of the pose graph (edges)\n\n"
                      << "          T      : turn on/off the visualization of ground truth trajectory\n"
                      << "          N      : turn on/off the visualization of normal/primary vector\n"
                      << "          S      : turn on/off the visualization of distance (30,60,90,120m) circles\n"
                      << "          U      : turn on/off the visualization of color bar\n"
                      << "          J      : printscreen\n"
                      << "          H      : help (instructions)\n"
                      << "          R      : reset window scale\n"
                      << "          Y      : reset window persepective\n"
                      << "          K      : enable jump to X frame mode\n"
                      << "          L      : enable pausing at the loop\n"
                      << "          F      : fly to point\n"
                      << "          G      : turn on/off the plotting scale\n"
                      << "          V      : turn on/off the vertex keypoints\n"
                      << "          O      : switch between persepective/parallel projection\n"
                      << "          M      : updating (to make the visualization more fluent)\n\n"
                      << "          Up     : decrease the downsampling ratio (display denser points)\n"
                      << "          Down   : increase the downsampling ratio (display sparser points)\n"
                      << "          Left   : decrease the brightness of the background\n"
                      << "          Right  : increase the brightness of the background\n\n"
                      << "          +      : increase point size\n"
                      << "          -      : decrease point size\n";

            is_paused_ = true;
        }
        else if (event.getKeySym() == "m")
        {
            ; //a dummy function for updating
        }
    }
}

template <typename PointT>
void MapViewer<PointT>::mouse_event_occurred(const pcl::visualization::MouseEvent &event,
                                             void *viewer_void)
{
    pcl::visualization::PCLVisualizer *viewer = static_cast<pcl::visualization::PCLVisualizer *>(viewer_void);
    if (event.getButton() == pcl::visualization::MouseEvent::RightButton &&
        event.getType() == pcl::visualization::MouseEvent::MouseButtonRelease)
    {
        std::cout << "Right mouse button released at position (" << event.getX() << ", " << event.getY() << ")" << std::endl;
    }
}

template <typename PointT>
void MapViewer<PointT>::keep_visualize(boost::shared_ptr<pcl::visualization::PCLVisualizer> &viewer)
{
    is_paused_ = true;
    std::cout << "Press [space] to continue\n";
    while (viewer && !viewer->wasStopped() && is_paused_)
    {
        viewer->spinOnce(refreshing_time_ms_);
        boost::this_thread::sleep(boost::posix_time::microseconds(sleep_micro_second_));
    }
}

template <typename PointT>
void MapViewer<PointT>::set_interactive_events(boost::shared_ptr<pcl::visualization::PCLVisualizer> &viewer, int window_length, int window_width, bool seg_viewport)
{
    viewer->setBackgroundColor(background_color_, background_color_, background_color_);
    viewer->addCoordinateSystem(2.0);

    viewer->registerKeyboardCallback(keyboard_event_occurred, (void *)&viewer);
    viewer->registerMouseCallback(mouse_event_occurred, (void *)&viewer);

    viewer->setSize(window_length, window_width); // Visualiser window size

    if (seg_viewport)
    {
        int v1(0);
        int v2(0);
        viewer->createViewPort(0.0, 0.0, 0.5, 1.0, v1);
        viewer->createViewPort(0.5, 0.0, 1.0, 1.0, v2);
    }
}
#if OPENCV_ON
template <typename PointT>
void MapViewer<PointT>::display_image(cv::Mat &image, const std::string image_viewer_name, int color_scale, int time_delay_ms)
{
    cv::Mat ri_temp_cm;
    if (color_scale == 0) //gray image
        ri_temp_cm = image;
    else if (color_scale == 1) // COLORMAP_JET
        cv::applyColorMap(image, ri_temp_cm, cv::COLORMAP_JET);
    else if (color_scale == 2) //COLORMAP_AUTUMN
        cv::applyColorMap(image, ri_temp_cm, cv::COLORMAP_AUTUMN);
    else if (color_scale == 3) //COLORMAP_HOT
        cv::applyColorMap(image, ri_temp_cm, cv::COLORMAP_HOT);
    else //default: gray
        ri_temp_cm = image;
    cv::imshow(image_viewer_name, ri_temp_cm);
    cv::waitKey(time_delay_ms);
}
#endif

template <typename PointT>
void MapViewer<PointT>::display_scan_realtime(const typename pcl::PointCloud<PointT>::Ptr &pc,
                                              boost::shared_ptr<pcl::visualization::PCLVisualizer> &viewer, int display_time_ms)
{
    viewer->removeAllPointClouds();
    viewer->removeAllShapes();
    viewer->setBackgroundColor(background_color_, background_color_, background_color_);

    if (show_current_scan_)
    {
        typename pcl::PointCloud<PointT>::Ptr pc_clone(new pcl::PointCloud<PointT>());
        *pc_clone = *pc;

        std::string frame_name = "current_scan";
        switch (color_rendering_type_)
        {
        case SINGLE:
        {
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc_frame_visual_rgb(new pcl::PointCloud<pcl::PointXYZRGB>);
            float intensity_color;
            int r_c,g_c,b_c;
            for (int i = 0; i < pc->points.size(); ++i)
            {
                pcl::PointXYZRGB pt;
                pt.x = pc->points[i].x;
                pt.y = pc->points[i].y;
                pt.z = pc->points[i].z;
                apply_label_color_mapping((int)(pc->points[i].curvature), r_c, g_c, b_c); //the curvature here indicates the point semantic label (mask)
                float intensity_color = 1.0 / intensity_scale_ * pc->points[i].intensity;
                intensity_color = 0.2 + min_(0.8, intensity_color);
                pt.r = r_c * intensity_color;
                pt.g = g_c * intensity_color;
                pt.b = b_c * intensity_color;
                pc_frame_visual_rgb->points.push_back(pt);
            }
            viewer->addPointCloud(pc_frame_visual_rgb, frame_name);
            break;

        }
        case HEIGHT: //Height ramp color scalar
        {
            typename pcl::visualization::PointCloudColorHandlerGenericField<PointT>::PointCloudColorHandlerGenericField rgb_z(pc_clone, "z");
            viewer->addPointCloud<PointT>(pc, rgb_z, frame_name);
            break;
        }
        case INTENSITY:
        {
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc_frame_visual_rgb(new pcl::PointCloud<pcl::PointXYZRGB>);
            float intensity_color;
            for (int i = 0; i < pc->points.size(); ++i)
            {
                pcl::PointXYZRGB pt;
                pt.x = pc->points[i].x;
                pt.y = pc->points[i].y;
                pt.z = pc->points[i].z;
                float intensity_color = 1.0 / intensity_scale_ * pc->points[i].intensity;
                intensity_color = 0.2 + min_(0.8, intensity_color);
                pt.r = 255 * intensity_color;
                pt.g = 255 * intensity_color;
                pt.b = 255 * intensity_color;
                pc_frame_visual_rgb->points.push_back(pt);
            }
            viewer->addPointCloud(pc_frame_visual_rgb, frame_name);
            break;
        }
        case INTENSITY_2:
        {
            for (int i = 0; i < pc_clone->points.size(); i++)
                pc_clone->points[i].intensity = intensity_scale_ * simple_look_up_table((intensity_scale_ - pc_clone->points[i].intensity) / intensity_scale_,
                                                                                        intensity_lut_x_a_, intensity_lut_x_b_, intensity_lut_y_a_, intensity_lut_y_b_);

            typename pcl::visualization::PointCloudColorHandlerGenericField<PointT>::PointCloudColorHandlerGenericField rgb_i(pc_clone, "intensity");
            viewer->addPointCloud<PointT>(pc_clone, rgb_i, frame_name);
            break;
        }
        }
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_IMMEDIATE_RENDERING, 1.0f, frame_name);

        if (show_distance_circles_)
            display_distance_circle(viewer);

        judge_pause(viewer, display_time_ms);

        pc_clone.reset(new pcl::PointCloud<PointT>());
    }
    else
        judge_pause(viewer, 1);
}

template <typename PointT>
void MapViewer<PointT>::display_2d_bbx_realtime(cloudblock_Ptrs &blocks,
                                                boost::shared_ptr<pcl::visualization::PCLVisualizer> &viewer,
                                                int display_time_ms)
{
    viewer->removeShape("submap_bbx");

    float line_width = 0.5;

    if (show_pose_graph_node_)
    {
        int edge_count = 4 * blocks.size();
        std::vector<unsigned int> edge_r(edge_count), edge_g(edge_count), edge_b(edge_count);

        typename pcl::PointCloud<PointT>::Ptr pc_nodes_1(new pcl::PointCloud<PointT>());
        typename pcl::PointCloud<PointT>::Ptr pc_nodes_2(new pcl::PointCloud<PointT>());

        for (int i = 0; i < blocks.size(); i++)
        {
            PointT ptc1;
            ptc1.x = blocks[i]->bound.min_x;
            ptc1.y = blocks[i]->bound.min_y;
            ptc1.z = blocks[i]->pose_lo(2, 3);

            PointT ptc2;
            ptc2.x = blocks[i]->bound.max_x;
            ptc2.y = blocks[i]->bound.min_y;
            ptc2.z = blocks[i]->pose_lo(2, 3);

            PointT ptc3;
            ptc3.x = blocks[i]->bound.max_x;
            ptc3.y = blocks[i]->bound.max_y;
            ptc3.z = blocks[i]->pose_lo(2, 3);

            PointT ptc4;
            ptc4.x = blocks[i]->bound.min_x;
            ptc4.y = blocks[i]->bound.max_y;
            ptc4.z = blocks[i]->pose_lo(2, 3);

            //1----2
            pc_nodes_1->points.push_back(ptc1);
            pc_nodes_2->points.push_back(ptc2);
            //2----3
            pc_nodes_1->points.push_back(ptc2);
            pc_nodes_2->points.push_back(ptc3);
            //3----4
            pc_nodes_1->points.push_back(ptc3);
            pc_nodes_2->points.push_back(ptc4);
            //4----1
            pc_nodes_1->points.push_back(ptc4);
            pc_nodes_2->points.push_back(ptc1);

            //orange
            edge_r[4 * i] = 235;
            edge_g[4 * i] = 97;
            edge_b[4 * i] = 35;
            edge_r[4 * i + 1] = 235;
            edge_g[4 * i + 1] = 97;
            edge_b[4 * i + 1] = 35;
            edge_r[4 * i + 2] = 235;
            edge_g[4 * i + 2] = 97;
            edge_b[4 * i + 2] = 35;
            edge_r[4 * i + 3] = 235;
            edge_g[4 * i + 3] = 97;
            edge_b[4 * i + 3] = 35;
        }

        add_lines_to_viewer(viewer, pc_nodes_1, pc_nodes_2, "submap_bbx", edge_r, edge_g, edge_b, 0);

        viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, line_width, "submap_bbx");

        judge_pause(viewer, display_time_ms);
    }
    else
        judge_pause(viewer, 1);
}

template <typename PointT>
void MapViewer<PointT>::display_pg_realtime(const constraints &cons,
                                            boost::shared_ptr<pcl::visualization::PCLVisualizer> &viewer, int display_time_ms)
{
    viewer->removeShape("edges");
    viewer->removePointCloud("nodes");

    float line_width = 2.0;
    float node_size = 7.0;

    if (show_pose_graph_edge_)
    {
        int edge_count = cons.size();
        std::vector<unsigned int> edge_r(edge_count), edge_g(edge_count), edge_b(edge_count);

        typename pcl::PointCloud<PointT>::Ptr pc_nodes_1(new pcl::PointCloud<PointT>());
        typename pcl::PointCloud<PointT>::Ptr pc_nodes_2(new pcl::PointCloud<PointT>());
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr nodes(new pcl::PointCloud<pcl::PointXYZRGB>());

        for (int i = 0; i < edge_count; i++)
        {
            float r, g, b;

            PointT ptc1;
            ptc1.x = cons[i].block1->pose_lo(0, 3);
            ptc1.y = cons[i].block1->pose_lo(1, 3);
            ptc1.z = cons[i].block1->pose_lo(2, 3);

            pc_nodes_1->points.push_back(ptc1);

            pcl::PointXYZRGB ptc1_rgb;
            ptc1_rgb.x = ptc1.x;
            ptc1_rgb.y = ptc1.y;
            ptc1_rgb.z = ptc1.z;
            get_random_color(r, g, b, 255);
            ptc1_rgb.r = r;
            ptc1_rgb.g = g;
            ptc1_rgb.b = b;
            nodes->points.push_back(ptc1_rgb);

            if (show_gt_trajectory_)
            {
                pcl::PointXYZRGB ptc1_gt_rgb;
                ptc1_gt_rgb.x = cons[i].block1->pose_gt(0, 3);
                ptc1_gt_rgb.y = cons[i].block1->pose_gt(1, 3);
                ptc1_gt_rgb.z = cons[i].block1->pose_gt(2, 3);
                ptc1_gt_rgb.r = r;
                ptc1_gt_rgb.g = g;
                ptc1_gt_rgb.b = b;
                nodes->points.push_back(ptc1_gt_rgb);
            }

            PointT ptc2;
            ptc2.x = cons[i].block2->pose_lo(0, 3);
            ptc2.y = cons[i].block2->pose_lo(1, 3);
            ptc2.z = cons[i].block2->pose_lo(2, 3);

            pc_nodes_2->points.push_back(ptc2);

            pcl::PointXYZRGB ptc2_rgb;
            ptc2_rgb.x = ptc2.x;
            ptc2_rgb.y = ptc2.y;
            ptc2_rgb.z = ptc2.z;
            get_random_color(r, g, b, 255);
            ptc2_rgb.r = r;
            ptc2_rgb.g = g;
            ptc2_rgb.b = b;
            nodes->points.push_back(ptc2_rgb);

            if (show_gt_trajectory_)
            {
                pcl::PointXYZRGB ptc2_gt_rgb;
                ptc2_gt_rgb.x = cons[i].block2->pose_gt(0, 3);
                ptc2_gt_rgb.y = cons[i].block2->pose_gt(1, 3);
                ptc2_gt_rgb.z = cons[i].block2->pose_gt(2, 3);
                ptc2_gt_rgb.r = r;
                ptc2_gt_rgb.g = g;
                ptc2_gt_rgb.b = b;
                nodes->points.push_back(ptc2_gt_rgb);
            }

            switch (cons[i].con_type)
            {
            case ADJACENT:
                edge_r[i] = 0;
                edge_g[i] = 255;
                edge_b[i] = 255;
                break;
            case REGISTRATION:
                edge_r[i] = 255;
                edge_g[i] = 0;
                edge_b[i] = 255;
                break;
            case HISTORY: //edges in the long-term memory (history)
                edge_r[i] = 255;
                edge_g[i] = 255;
                edge_b[i] = 0;
                break;
            case NONE: //failed/wrong edge
                edge_r[i] = 255;
                edge_g[i] = 0;
                edge_b[i] = 0;
                break;
            default:
                break;
            }
        }
        add_lines_to_viewer(viewer, pc_nodes_1, pc_nodes_2, "edges", edge_r, edge_g, edge_b, 0);

        viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, line_width, "edges");

        viewer->addPointCloud(nodes, "nodes");

        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, node_size, "nodes");

        judge_pause(viewer, display_time_ms);
    }
    else
        judge_pause(viewer, 1);
}

template <typename PointT>
void MapViewer<PointT>::display_2_pc(const typename pcl::PointCloud<PointT>::Ptr &Cloud1, const typename pcl::PointCloud<PointT>::Ptr &Cloud2,
                                     std::string displayname, int display_downsample_ratio)
{

    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer(displayname));

    viewer->setBackgroundColor(255, 255, 255);
    char t[256];
    std::string s;
    int n = 0;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud1(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud2(new pcl::PointCloud<pcl::PointXYZRGB>);

    for (size_t i = 0; i < Cloud1->points.size(); ++i)
    {
        if (i % display_downsample_ratio == 0)
        {
            pcl::PointXYZRGB pt;
            pt.x = Cloud1->points[i].x;
            pt.y = Cloud1->points[i].y;
            pt.z = Cloud1->points[i].z;
            pt.r = 255;
            pt.g = 215;
            pt.b = 0;
            pointcloud1->points.push_back(pt);
        }
    } // Golden

    viewer->addPointCloud(pointcloud1, "pointcloudT");

    for (size_t i = 0; i < Cloud2->points.size(); ++i)
    {
        if (i % display_downsample_ratio == 0)
        {
            pcl::PointXYZRGB pt;
            pt.x = Cloud2->points[i].x;
            pt.y = Cloud2->points[i].y;
            pt.z = Cloud2->points[i].z;
            pt.r = 233;
            pt.g = 233;
            pt.b = 216;
            pointcloud2->points.push_back(pt);
        }
    } // Silver

    viewer->addPointCloud(pointcloud2, "pointcloudS");

    cout << "Click X(close) to continue..." << endl;
    while (!viewer->wasStopped())
    {
        viewer->spinOnce(100);
        boost::this_thread::sleep(boost::posix_time::microseconds(100000));
    }
}

template <typename PointT>
void MapViewer<PointT>::display_2_pc_compare(const typename pcl::PointCloud<PointT>::Ptr &Cloud1_left, const typename pcl::PointCloud<PointT>::Ptr &Cloud2_left,
                                             const typename pcl::PointCloud<PointT>::Ptr &Cloud1_right, const typename pcl::PointCloud<PointT>::Ptr &Cloud2_right,
                                             std::string displayname, int display_downsample_ratio)
{

    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer(displayname));
    viewer->setBackgroundColor(255, 255, 255);
    char t[256];
    std::string s;
    int n = 0;

    //Create two vertically separated viewports
    int v1(0);
    int v2(1);
    viewer->createViewPort(0.0, 0.0, 0.5, 1.0, v1);
    viewer->createViewPort(0.5, 0.0, 1.0, 1.0, v2);

    viewer->addCoordinateSystem(4.0);

    // Set camera position and orientation
    float x_position = Cloud1_left->points[0].x;
    float y_position = Cloud1_left->points[0].y;
    float z_position = Cloud1_left->points[0].z;

    viewer->setCameraPosition(x_position, y_position, z_position, 1, 0, 0, 0, 0, 1, 0);
    viewer->setCameraPosition(x_position, y_position, z_position, 1, 0, 0, 0, 0, 1, 1);

    viewer->setSize(1920, 1080); // Visualiser window size

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud1l(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud2l(new pcl::PointCloud<pcl::PointXYZRGB>);

    for (size_t i = 0; i < Cloud1_left->points.size(); ++i)
    {
        if (i % display_downsample_ratio == 0)
        {
            pcl::PointXYZRGB pt;
            pt.x = Cloud1_left->points[i].x;
            pt.y = Cloud1_left->points[i].y;
            pt.z = Cloud1_left->points[i].z;
            pt.r = 255;
            pt.g = 215;
            pt.b = 0;
            pointcloud1l->points.push_back(pt);
        }
    } // Golden

    viewer->addPointCloud(pointcloud1l, "pointcloudT_left", v1);

    for (size_t i = 0; i < Cloud2_left->points.size(); ++i)
    {
        if (i % display_downsample_ratio == 0)
        {
            pcl::PointXYZRGB pt;
            pt.x = Cloud2_left->points[i].x;
            pt.y = Cloud2_left->points[i].y;
            pt.z = Cloud2_left->points[i].z;
            pt.r = 233;
            pt.g = 233;
            pt.b = 216;
            pointcloud2l->points.push_back(pt);
        }
    } // Silver

    viewer->addPointCloud(pointcloud2l, "pointcloudS_left", v1);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud1r(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud2r(new pcl::PointCloud<pcl::PointXYZRGB>);

    for (size_t i = 0; i < Cloud1_right->points.size(); ++i)
    {
        if (i % display_downsample_ratio == 0)
        {
            pcl::PointXYZRGB pt;
            pt.x = Cloud1_right->points[i].x;
            pt.y = Cloud1_right->points[i].y;
            pt.z = Cloud1_right->points[i].z;
            pt.r = 255;
            pt.g = 215;
            pt.b = 0;
            pointcloud1r->points.push_back(pt);
        }
    } // Golden

    viewer->addPointCloud(pointcloud1r, "pointcloudT_right", v2);

    for (size_t i = 0; i < Cloud2_right->points.size(); ++i)
    {
        if (i % display_downsample_ratio == 0)
        {
            pcl::PointXYZRGB pt;
            pt.x = Cloud2_right->points[i].x;
            pt.y = Cloud2_right->points[i].y;
            pt.z = Cloud2_right->points[i].z;
            pt.r = 233;
            pt.g = 233;
            pt.b = 216;
            pointcloud2r->points.push_back(pt);
        }
    } // Silver

    viewer->addPointCloud(pointcloud2r, "pointcloudS_right", v2);

    cout << "Enter F to switch the persepective, Click X(close) to continue..." << endl;
    while (!viewer->wasStopped())
    {
        viewer->spinOnce(100);
        boost::this_thread::sleep(boost::posix_time::microseconds(100000));
    }
}

template <typename PointT>
void MapViewer<PointT>::display_pc_with_bbx_realtime(boost::shared_ptr<pcl::visualization::PCLVisualizer> &viewer,
                                                     const typename pcl::PointCloud<PointT>::Ptr &pc_0,
                                                     const typename pcl::PointCloud<PointT>::Ptr &pc_1,
                                                     const typename pcl::PointCloud<PointT>::Ptr &bbx_vertex,
                                                     const std::vector<int> &bbx_class,
                                                     int display_time_ms)
{
    std::string pc_name = "current_pc";

    viewer->removeAllPointClouds();
    viewer->removeAllShapes();
    viewer->removeCoordinateSystem();

    viewer->setBackgroundColor(background_color_, background_color_, background_color_);

    typename pcl::PointCloud<PointT>::Ptr vertex_0(new pcl::PointCloud<PointT>);
    typename pcl::PointCloud<PointT>::Ptr vertex_1(new pcl::PointCloud<PointT>);
    std::vector<unsigned int> red, green, blue;
    //std::vector<float> linewidth;

    int r, g, b;

    //[0, 1], [0, 3], [1, 2], [2, 3], [4, 5], [5, 6], [4, 7], [6, 7], [0, 4], [1, 5], [2, 6], [3, 7]

    for (size_t i = 0; i < bbx_class.size(); ++i)
    {
        vertex_0->push_back(bbx_vertex->points[8 * i]);
        vertex_1->push_back(bbx_vertex->points[8 * i + 1]);
        vertex_0->push_back(bbx_vertex->points[8 * i]);
        vertex_1->push_back(bbx_vertex->points[8 * i + 3]);
        vertex_0->push_back(bbx_vertex->points[8 * i + 1]);
        vertex_1->push_back(bbx_vertex->points[8 * i + 2]);
        vertex_0->push_back(bbx_vertex->points[8 * i + 2]);
        vertex_1->push_back(bbx_vertex->points[8 * i + 3]);
        vertex_0->push_back(bbx_vertex->points[8 * i + 4]);
        vertex_1->push_back(bbx_vertex->points[8 * i + 5]);
        vertex_0->push_back(bbx_vertex->points[8 * i + 5]);
        vertex_1->push_back(bbx_vertex->points[8 * i + 6]);
        vertex_0->push_back(bbx_vertex->points[8 * i + 4]);
        vertex_1->push_back(bbx_vertex->points[8 * i + 7]);
        vertex_0->push_back(bbx_vertex->points[8 * i + 6]);
        vertex_1->push_back(bbx_vertex->points[8 * i + 7]);
        vertex_0->push_back(bbx_vertex->points[8 * i + 0]);
        vertex_1->push_back(bbx_vertex->points[8 * i + 4]);
        vertex_0->push_back(bbx_vertex->points[8 * i + 1]);
        vertex_1->push_back(bbx_vertex->points[8 * i + 5]);
        vertex_0->push_back(bbx_vertex->points[8 * i + 2]);
        vertex_1->push_back(bbx_vertex->points[8 * i + 6]);
        vertex_0->push_back(bbx_vertex->points[8 * i + 3]);
        vertex_1->push_back(bbx_vertex->points[8 * i + 7]);

        assign_color_by_class(bbx_class[i], r, g, b);

        for (int j = 0; j < 12; j++)
        {
            red.push_back(r);
            green.push_back(g);
            blue.push_back(b);
        }
    }

    add_lines_to_viewer(viewer, vertex_0, vertex_1, "bbxs", red, green, blue);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc_rgb(new pcl::PointCloud<pcl::PointXYZRGB>);
    float intensity_color;

    for (size_t i = 0; i < pc_0->points.size(); ++i)
    {
        if (i % downsample_ratio_vis_ == 0)
        {
            pcl::PointXYZRGB pt;
            pt.x = pc_0->points[i].x;
            pt.y = pc_0->points[i].y;
            pt.z = pc_0->points[i].z;
            intensity_color = 0.3 + min_(0.7, 1.0 / 55 * pc_0->points[i].intensity);
            pt.r = 233 * intensity_color;
            pt.g = 233 * intensity_color;
            pt.b = 216 * intensity_color;
            pc_rgb->points.push_back(pt);
        }
    } // Silver

    for (size_t i = 0; i < pc_1->points.size(); ++i)
    {
        if (i % downsample_ratio_vis_ == 0)
        {
            pcl::PointXYZRGB pt;
            pt.x = pc_1->points[i].x;
            pt.y = pc_1->points[i].y;
            pt.z = pc_1->points[i].z;
            intensity_color = 0.2 + min_(0.8, 1.0 / 55 * pc_1->points[i].intensity);
            pt.r = 233 * intensity_color;
            pt.g = 233 * intensity_color;
            pt.b = 216 * intensity_color;
            pc_rgb->points.push_back(pt);
        }
    } // Silver

    viewer->addPointCloud(pc_rgb, pc_name);

    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2.5, pc_name);

    //viewer->addCoordinateSystem(3.0);

    pcl::PointCloud<pcl::PointXYZRGB>().swap(*pc_rgb);
    pcl::PointCloud<PointT>().swap(*vertex_0);
    pcl::PointCloud<PointT>().swap(*vertex_1);

    judge_pause(viewer, display_time_ms);
}

template <typename PointT>
void MapViewer<PointT>::display_2_pc_compare_realtime(const typename pcl::PointCloud<PointT>::Ptr &Cloud1_left, const typename pcl::PointCloud<PointT>::Ptr &Cloud2_left,
                                                      const typename pcl::PointCloud<PointT>::Ptr &Cloud1_right, const typename pcl::PointCloud<PointT>::Ptr &Cloud2_right,
                                                      boost::shared_ptr<pcl::visualization::PCLVisualizer> &viewer, int display_time_ms)
{
    int v1(0);
    int v2(1);

    viewer->removeAllPointClouds(v1);
    viewer->removeAllPointClouds(v2);
    viewer->removeCoordinateSystem();
    viewer->removeAllShapes();

    if (show_reg_)
    {

        //Create two vertically separated viewports
        viewer->createViewPort(0.0, 0.0, 0.5, 1.0, v1);
        viewer->createViewPort(0.5, 0.0, 1.0, 1.0, v2);

        viewer->setBackgroundColor(background_color_, background_color_, background_color_);

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud1l(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud2l(new pcl::PointCloud<pcl::PointXYZRGB>);

        float intensity_color;
        bool intensity_availiable = false;

        //judge if point cloud intensity is availiable
        for (int i = 0; i < Cloud1_left->points.size(); i += 50)
        {
            if (Cloud1_left->points[i].intensity > 1e-12)
            {
                intensity_availiable = true;
                break;
            }
        }

        for (size_t i = 0; i < Cloud1_left->points.size(); ++i)
        {
            if (i % downsample_ratio_vis_ == 0)
            {
                pcl::PointXYZRGB pt;
                pt.x = Cloud1_left->points[i].x;
                pt.y = Cloud1_left->points[i].y;
                pt.z = Cloud1_left->points[i].z;
                intensity_color = 0.2 + min_(0.8, 1.0 / intensity_scale_ * Cloud1_left->points[i].intensity);
                if (!intensity_availiable)
                    intensity_color = 1.0;
                
                //LOG(INFO)<< intensity_color;
                pt.r = 255 * intensity_color;
                pt.g = 215 * intensity_color;
                pt.b = 0 * intensity_color;
                pointcloud1l->points.push_back(pt);
            }
        } // Golden

        viewer->addPointCloud(pointcloud1l, "pointcloudS_left", v1);

        for (size_t i = 0; i < Cloud2_left->points.size(); ++i)
        {
            if (i % downsample_ratio_vis_ == 0)
            {
                pcl::PointXYZRGB pt;
                pt.x = Cloud2_left->points[i].x;
                pt.y = Cloud2_left->points[i].y;
                pt.z = Cloud2_left->points[i].z;
                intensity_color = 0.2 + min_(0.8, 1.0 / intensity_scale_ * Cloud2_left->points[i].intensity);
                if (!intensity_availiable)
                    intensity_color = 1.0;
                pt.r = 233 * intensity_color;
                pt.g = 233 * intensity_color;
                pt.b = 216 * intensity_color;
                pointcloud2l->points.push_back(pt);
            }
        } // Silver

        viewer->addPointCloud(pointcloud2l, "pointcloudT_left", v1);

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud1r(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud2r(new pcl::PointCloud<pcl::PointXYZRGB>);

        for (size_t i = 0; i < Cloud1_right->points.size(); ++i)
        {
            if (i % downsample_ratio_vis_ == 0)
            {
                pcl::PointXYZRGB pt;
                pt.x = Cloud1_right->points[i].x;
                pt.y = Cloud1_right->points[i].y;
                pt.z = Cloud1_right->points[i].z;
                intensity_color = 0.2 + min_(0.8, 1.0 / intensity_scale_ * Cloud1_right->points[i].intensity);
                if (!intensity_availiable)
                    intensity_color = 1.0;
                pt.r = 255 * intensity_color;
                pt.g = 215 * intensity_color;
                pt.b = 0 * intensity_color;
                pointcloud1r->points.push_back(pt);
            }
        } // Golden

        viewer->addPointCloud(pointcloud1r, "pointcloudS_right", v2);

        // for (size_t i = 0; i < Cloud2_right->points.size(); ++i)
        // {
        //     if (i % downsample_ratio_vis_ == 0)
        //     {
        //         pcl::PointXYZRGB pt;
        //         pt.x = Cloud2_right->points[i].x;
        //         pt.y = Cloud2_right->points[i].y;
        //         pt.z = Cloud2_right->points[i].z;
        //         intensity_color = max_(0.2, intensity_scale_ * Cloud2_left->points[i].intensity);
        //         if (intensity_color < 1e-10) //no intensity availiable
        //             intensity_color = 1;
        //         pt.r = 233 * intensity_color;
        //         pt.g = 233 * intensity_color;
        //         pt.b = 216 * intensity_color;
        //         pointcloud2r->points.push_back(pt);
        //     }
        // } // Silver

        viewer->addPointCloud(pointcloud2l, "pointcloudT_right", v2);

        viewer->addCoordinateSystem(3.0);

        if (reset_camera_)
        {
            viewer->setCameraPosition(3.403921, 1.755630, 120.086624, 0.000000, 0.999979, -0.006464);
            //viewer->setCameraPosition(-50.852133,-142.740895,1251.049052,0.000000,0.999977,0.006791);
            reset_camera_ = false;
        }

        if (show_distance_circles_)
            display_distance_circle(viewer);

        pcl::PointCloud<pcl::PointXYZRGB>().swap(*pointcloud2r);
        pcl::PointCloud<pcl::PointXYZRGB>().swap(*pointcloud1r);
        pcl::PointCloud<pcl::PointXYZRGB>().swap(*pointcloud2l);
        pcl::PointCloud<pcl::PointXYZRGB>().swap(*pointcloud1l);

        judge_pause(viewer, display_time_ms);
    }
    else
        judge_pause(viewer, 1);
}

template <typename PointT>
void MapViewer<PointT>::display_feature_pts_compare(cloudblock_t &in_block_tc, cloudblock_t &in_block_sc, std::string displayname)
{
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer(displayname));

    viewer->setSize(1920, 1080); // Visualiser window size
    viewer->setBackgroundColor(0, 0, 0);

    //Create two vertically separated viewports
    int v1(0);
    int v2(1);
    viewer->createViewPort(0.0, 0.0, 0.5, 1.0, v1);
    viewer->createViewPort(0.5, 0.0, 1.0, 1.0, v2);

    viewer->addCoordinateSystem(4.0);

    float x_position, y_position, z_position;

    // Set camera position and orientation
    if (!in_block_tc.station_position_available)
    {
        x_position = in_block_sc.local_cp.x;
        y_position = in_block_sc.local_cp.y;
        z_position = in_block_sc.local_cp.z;
    }
    else
    {
        x_position = in_block_sc.local_station.x;
        y_position = in_block_sc.local_station.y;
        z_position = in_block_sc.local_station.z;
    }

    LOG(INFO) << "camera position: (" << x_position << "," << y_position << "," << z_position << ")";

    viewer->setCameraPosition(x_position, y_position, z_position, 1, 0, 0, 0, 0, 1, 0);
    viewer->setCameraPosition(x_position, y_position, z_position, 1, 0, 0, 0, 0, 1, 1);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr feature_frame_tc(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr feature_frame_sc(new pcl::PointCloud<pcl::PointXYZRGB>);

    for (int i = 0; i < in_block_sc.pc_ground_down->points.size(); ++i)
    {
        pcl::PointXYZRGB pt;

        pt.x = in_block_sc.pc_ground_down->points[i].x;
        pt.y = in_block_sc.pc_ground_down->points[i].y;
        pt.z = in_block_sc.pc_ground_down->points[i].z;

        pt.r = 128;
        pt.g = 128;
        pt.b = 128;
        //Ground - Gray
        feature_frame_sc->points.push_back(pt);
    }

    for (int i = 0; i < in_block_sc.pc_pillar_down->points.size(); ++i)
    {
        pcl::PointXYZRGB pt;

        pt.x = in_block_sc.pc_pillar_down->points[i].x;
        pt.y = in_block_sc.pc_pillar_down->points[i].y;
        pt.z = in_block_sc.pc_pillar_down->points[i].z;

        pt.r = 0;
        pt.g = 255;
        pt.b = 0;
        //Pillar - Green
        feature_frame_sc->points.push_back(pt);
    }

    for (int i = 0; i < in_block_sc.pc_beam_down->points.size(); ++i)
    {
        pcl::PointXYZRGB pt;

        pt.x = in_block_sc.pc_beam_down->points[i].x;
        pt.y = in_block_sc.pc_beam_down->points[i].y;
        pt.z = in_block_sc.pc_beam_down->points[i].z;

        pt.r = 255;
        pt.g = 255;
        pt.b = 0;
        //Beam - Yellow
        feature_frame_sc->points.push_back(pt);
    }

    for (int i = 0; i < in_block_sc.pc_facade_down->points.size(); ++i)
    {
        pcl::PointXYZRGB pt;

        pt.x = in_block_sc.pc_facade_down->points[i].x;
        pt.y = in_block_sc.pc_facade_down->points[i].y;
        pt.z = in_block_sc.pc_facade_down->points[i].z;

        pt.r = 0;
        pt.g = 0;
        pt.b = 255;
        //Facade - Blue
        feature_frame_sc->points.push_back(pt);
    }

    for (int i = 0; i < in_block_sc.pc_roof_down->points.size(); ++i)
    {
        pcl::PointXYZRGB pt;

        pt.x = in_block_sc.pc_roof_down->points[i].x;
        pt.y = in_block_sc.pc_roof_down->points[i].y;
        pt.z = in_block_sc.pc_roof_down->points[i].z;

        pt.r = 255;
        pt.g = 0;
        pt.b = 0;
        //Roof - Red
        feature_frame_sc->points.push_back(pt);
    }

    for (int i = 0; i < in_block_sc.pc_vertex->points.size(); ++i)
    {
        pcl::PointXYZRGB pt;

        pt.x = in_block_sc.pc_vertex->points[i].x;
        pt.y = in_block_sc.pc_vertex->points[i].y;
        pt.z = in_block_sc.pc_vertex->points[i].z;

        pt.r = 0;
        pt.g = 255;
        pt.b = 255;
        //Vertex - Cyan
        feature_frame_sc->points.push_back(pt);
    }

    for (int i = 0; i < in_block_tc.pc_ground->points.size(); ++i)
    {
        pcl::PointXYZRGB pt;

        pt.x = in_block_tc.pc_ground->points[i].x;
        pt.y = in_block_tc.pc_ground->points[i].y;
        pt.z = in_block_tc.pc_ground->points[i].z;

        pt.r = 128;
        pt.g = 128;
        pt.b = 128;
        //Ground - Silver
        feature_frame_tc->points.push_back(pt);
    }

    for (int i = 0; i < in_block_tc.pc_pillar->points.size(); ++i)
    {
        pcl::PointXYZRGB pt;

        pt.x = in_block_tc.pc_pillar->points[i].x;
        pt.y = in_block_tc.pc_pillar->points[i].y;
        pt.z = in_block_tc.pc_pillar->points[i].z;

        pt.r = 0;
        pt.g = 255;
        pt.b = 0;
        //Pillar - Green
        feature_frame_tc->points.push_back(pt);
    }

    for (int i = 0; i < in_block_tc.pc_beam->points.size(); ++i)
    {
        pcl::PointXYZRGB pt;

        pt.x = in_block_tc.pc_beam->points[i].x;
        pt.y = in_block_tc.pc_beam->points[i].y;
        pt.z = in_block_tc.pc_beam->points[i].z;

        pt.r = 255;
        pt.g = 255;
        pt.b = 0;
        //Beam - Yellow
        feature_frame_tc->points.push_back(pt);
    }

    for (int i = 0; i < in_block_tc.pc_facade->points.size(); ++i)
    {
        pcl::PointXYZRGB pt;

        pt.x = in_block_tc.pc_facade->points[i].x;
        pt.y = in_block_tc.pc_facade->points[i].y;
        pt.z = in_block_tc.pc_facade->points[i].z;

        pt.r = 0;
        pt.g = 0;
        pt.b = 255;
        //Facade - Blue
        feature_frame_tc->points.push_back(pt);
    }

    for (int i = 0; i < in_block_tc.pc_roof->points.size(); ++i)
    {
        pcl::PointXYZRGB pt;

        pt.x = in_block_tc.pc_roof->points[i].x;
        pt.y = in_block_tc.pc_roof->points[i].y;
        pt.z = in_block_tc.pc_roof->points[i].z;

        pt.r = 255;
        pt.g = 0;
        pt.b = 0;
        //Roof - Red
        feature_frame_tc->points.push_back(pt);
    }

    for (int i = 0; i < in_block_tc.pc_vertex->points.size(); ++i)
    {
        pcl::PointXYZRGB pt;

        pt.x = in_block_tc.pc_vertex->points[i].x;
        pt.y = in_block_tc.pc_vertex->points[i].y;
        pt.z = in_block_tc.pc_vertex->points[i].z;

        pt.r = 0;
        pt.g = 255;
        pt.b = 255;
        //Vertex - Cyan
        feature_frame_tc->points.push_back(pt);
    }

    viewer->addPointCloud(feature_frame_tc, "feature_point_tc", v1);

    viewer->addPointCloud(feature_frame_sc, "feature_point_sc", v2);

    cout << "Enter F to switch the persepective, Click X(close) to continue..." << endl;
    while (!viewer->wasStopped())
    {
        viewer->spinOnce(100);
        boost::this_thread::sleep(boost::posix_time::microseconds(1000));
    }
}

template <typename PointT>
void MapViewer<PointT>::display_feature_pts_compare_realtime(cloudblock_Ptr &in_block_tc, cloudblock_Ptr &in_block_sc,
                                                             boost::shared_ptr<pcl::visualization::PCLVisualizer> &viewer,
                                                             int display_time_ms)

{
    int v1(0);
    int v2(1);

    viewer->removeAllPointClouds(v1);
    viewer->removeAllPointClouds(v2);
    viewer->removeCoordinateSystem();
    viewer->removeAllShapes();

    if (show_feature_)
    {
        //Create two vertically separated viewports
        viewer->createViewPort(0.0, 0.0, 0.5, 1.0, v1);
        viewer->createViewPort(0.5, 0.0, 1.0, 1.0, v2);

        viewer->setBackgroundColor(background_color_, background_color_, background_color_);

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr feature_frame_tc(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr feature_frame_sc(new pcl::PointCloud<pcl::PointXYZRGB>);

        for (int i = 0; i < in_block_sc->pc_ground_down->points.size(); ++i)
        {
            pcl::PointXYZRGB pt;

            pt.x = in_block_sc->pc_ground_down->points[i].x;
            pt.y = in_block_sc->pc_ground_down->points[i].y;
            pt.z = in_block_sc->pc_ground_down->points[i].z;

            pt.r = 128;
            pt.g = 128;
            pt.b = 128;
            //Ground - Gray
            feature_frame_sc->points.push_back(pt);
        }

        for (int i = 0; i < in_block_sc->pc_pillar_down->points.size(); ++i)
        {
            pcl::PointXYZRGB pt;

            pt.x = in_block_sc->pc_pillar_down->points[i].x;
            pt.y = in_block_sc->pc_pillar_down->points[i].y;
            pt.z = in_block_sc->pc_pillar_down->points[i].z;

            pt.r = 0;
            pt.g = 255;
            pt.b = 0;
            //Pillar - Green
            feature_frame_sc->points.push_back(pt);
        }

        for (int i = 0; i < in_block_sc->pc_beam_down->points.size(); ++i)
        {
            pcl::PointXYZRGB pt;

            pt.x = in_block_sc->pc_beam_down->points[i].x;
            pt.y = in_block_sc->pc_beam_down->points[i].y;
            pt.z = in_block_sc->pc_beam_down->points[i].z;

            pt.r = 255;
            pt.g = 255;
            pt.b = 0;
            //Beam - Yellow
            feature_frame_sc->points.push_back(pt);
        }

        for (int i = 0; i < in_block_sc->pc_facade_down->points.size(); ++i)
        {
            pcl::PointXYZRGB pt;

            pt.x = in_block_sc->pc_facade_down->points[i].x;
            pt.y = in_block_sc->pc_facade_down->points[i].y;
            pt.z = in_block_sc->pc_facade_down->points[i].z;

            pt.r = 0;
            pt.g = 0;
            pt.b = 255;
            //Facade - Blue
            feature_frame_sc->points.push_back(pt);
        }

        for (int i = 0; i < in_block_sc->pc_roof_down->points.size(); ++i)
        {
            pcl::PointXYZRGB pt;

            pt.x = in_block_sc->pc_roof_down->points[i].x;
            pt.y = in_block_sc->pc_roof_down->points[i].y;
            pt.z = in_block_sc->pc_roof_down->points[i].z;

            pt.r = 255;
            pt.g = 0;
            pt.b = 0;
            //Roof - Red
            feature_frame_sc->points.push_back(pt);
        }

        if (show_vertex_keypoints_)
        {
            for (int i = 0; i < in_block_sc->pc_vertex->points.size(); ++i)
            {
                pcl::PointXYZRGB pt;

                pt.x = in_block_sc->pc_vertex->points[i].x;
                pt.y = in_block_sc->pc_vertex->points[i].y;
                pt.z = in_block_sc->pc_vertex->points[i].z;

                pt.r = 255;
                pt.g = 0;
                pt.b = 255;
                //Vertex - Purple
                feature_frame_sc->points.push_back(pt);
            }
        }

        for (int i = 0; i < in_block_tc->pc_ground->points.size(); ++i)
        {
            pcl::PointXYZRGB pt;

            pt.x = in_block_tc->pc_ground->points[i].x;
            pt.y = in_block_tc->pc_ground->points[i].y;
            pt.z = in_block_tc->pc_ground->points[i].z;

            pt.r = 128;
            pt.g = 128;
            pt.b = 128;
            //Ground - Gray
            feature_frame_tc->points.push_back(pt);
        }

        for (int i = 0; i < in_block_tc->pc_pillar->points.size(); ++i)
        {
            pcl::PointXYZRGB pt;

            pt.x = in_block_tc->pc_pillar->points[i].x;
            pt.y = in_block_tc->pc_pillar->points[i].y;
            pt.z = in_block_tc->pc_pillar->points[i].z;

            pt.r = 0;
            pt.g = 255;
            pt.b = 0;
            //Pillar - Green
            feature_frame_tc->points.push_back(pt);
        }

        for (int i = 0; i < in_block_tc->pc_beam->points.size(); ++i)
        {
            pcl::PointXYZRGB pt;

            pt.x = in_block_tc->pc_beam->points[i].x;
            pt.y = in_block_tc->pc_beam->points[i].y;
            pt.z = in_block_tc->pc_beam->points[i].z;

            pt.r = 255;
            pt.g = 255;
            pt.b = 0;
            //Beam - Yellow
            feature_frame_tc->points.push_back(pt);
        }

        for (int i = 0; i < in_block_tc->pc_facade->points.size(); ++i)
        {
            pcl::PointXYZRGB pt;

            pt.x = in_block_tc->pc_facade->points[i].x;
            pt.y = in_block_tc->pc_facade->points[i].y;
            pt.z = in_block_tc->pc_facade->points[i].z;

            pt.r = 0;
            pt.g = 0;
            pt.b = 255;
            //Facade - Blue
            feature_frame_tc->points.push_back(pt);
        }

        for (int i = 0; i < in_block_tc->pc_roof->points.size(); ++i)
        {
            pcl::PointXYZRGB pt;

            pt.x = in_block_tc->pc_roof->points[i].x;
            pt.y = in_block_tc->pc_roof->points[i].y;
            pt.z = in_block_tc->pc_roof->points[i].z;

            pt.r = 255;
            pt.g = 0;
            pt.b = 0;
            //Roof - Red
            feature_frame_tc->points.push_back(pt);
        }

        if (show_vertex_keypoints_)
        {
            for (int i = 0; i < in_block_tc->pc_vertex->points.size(); ++i)
            {
                pcl::PointXYZRGB pt;

                pt.x = in_block_tc->pc_vertex->points[i].x;
                pt.y = in_block_tc->pc_vertex->points[i].y;
                pt.z = in_block_tc->pc_vertex->points[i].z;

                pt.r = 255;
                pt.g = 0;
                pt.b = 255;
                //Vertex - Purple
                feature_frame_tc->points.push_back(pt);
            }
        }

        LOG(WARNING) << "Ground - Silver, Pillar - Green, Beam - Yellow, Facade - Blue, Roof - Red, Vertex - Purple";

        viewer->addPointCloud(feature_frame_tc, "feature_points_tc", v1);
        viewer->addPointCloud(feature_frame_sc, "feature_points_sc", v2);

        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "feature_points_tc");
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "feature_points_sc");

        int normal_vis_down_rate = 2;
        float normal_arrow_size = 0.6;

        if (show_point_normal_)
        {
            if (in_block_sc->pc_ground_down->size() > 0)
            {
                viewer->addPointCloudNormals<PointT>(in_block_sc->pc_ground_down, normal_vis_down_rate, normal_arrow_size, "ground_down_normal", v2); //template function
                viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 1.0, 1.0, "ground_down_normal", v2);
            }
            if (in_block_sc->pc_facade_down->size() > 0)
            {
                viewer->addPointCloudNormals<PointT>(in_block_sc->pc_facade_down, normal_vis_down_rate, normal_arrow_size, "facade_down_normal", v2);
                viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 1.0, 1.0, "facade_down_normal", v2);
            }
            if (in_block_sc->pc_pillar_down->size() > 0)
            {
                viewer->addPointCloudNormals<PointT>(in_block_sc->pc_pillar_down, normal_vis_down_rate, normal_arrow_size, "pillar_down_normal", v2);
                viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 1.0, "pillar_down_normal", v2);
            }
            if (in_block_sc->pc_beam_down->size() > 0)
            {
                viewer->addPointCloudNormals<PointT>(in_block_sc->pc_beam_down, normal_vis_down_rate, normal_arrow_size, "beam_down_normal", v2);
                viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 1.0, "beam_down_normal", v2);
            }
            if (in_block_sc->pc_roof_down->size() > 0)
            {
                viewer->addPointCloudNormals<PointT>(in_block_sc->pc_roof_down, normal_vis_down_rate, normal_arrow_size, "roof_down_normal", v2);
                viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 1.0, 1.0, "roof_down_normal", v2);
            }
            if (in_block_tc->pc_ground->size() > 0)
            {
                viewer->addPointCloudNormals<PointT>(in_block_tc->pc_ground, normal_vis_down_rate, normal_arrow_size, "ground_normal", v1);
                viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 1.0, 1.0, "ground_normal", v1);
            }
            if (in_block_tc->pc_facade->size() > 0)
            {
                viewer->addPointCloudNormals<PointT>(in_block_tc->pc_facade, normal_vis_down_rate, normal_arrow_size, "facade_normal", v1);
                viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 1.0, 1.0, "facade_normal", v1);
            }
            if (in_block_tc->pc_pillar->size() > 0)
            {
                viewer->addPointCloudNormals<PointT>(in_block_tc->pc_pillar, normal_vis_down_rate, normal_arrow_size, "pillar_normal", v1);
                viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 1.0, "pillar_normal", v1);
            }
            if (in_block_tc->pc_beam->size() > 0)
            {
                viewer->addPointCloudNormals<PointT>(in_block_tc->pc_beam, normal_vis_down_rate, normal_arrow_size, "beam_normal", v1);
                viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 1.0, "beam_normal", v1);
            }
            if (in_block_tc->pc_roof->size() > 0)
            {
                viewer->addPointCloudNormals<PointT>(in_block_tc->pc_roof, normal_vis_down_rate, normal_arrow_size, "roof_normal", v1);
                viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 1.0, 1.0, "roof_normal", v1);
            }
        }
        if (show_distance_circles_)
            display_distance_circle(viewer);

        viewer->addCoordinateSystem(3.0);

        if (reset_camera_)
        {
            viewer->setCameraPosition(3.403921, 1.755630, 120.086624, 0.000000, 0.999979, -0.006464);
            //viewer->setCameraPosition(-50.852133,-142.740895,1251.049052,0.000000,0.999977,0.006791);
            reset_camera_ = false;
        }

        pcl::PointCloud<pcl::PointXYZRGB>().swap(*feature_frame_tc);
        pcl::PointCloud<pcl::PointXYZRGB>().swap(*feature_frame_sc);

        judge_pause(viewer, display_time_ms);
    }
    else
        judge_pause(viewer, 1);
}

template <typename PointT>
void MapViewer<PointT>::display_distance_circle(boost::shared_ptr<pcl::visualization::PCLVisualizer> &viewer)
{
    //four circles (radius 30, 60, 90, 120)
    pcl::ModelCoefficients circle_coeff_1;
    circle_coeff_1.values.resize(3);
    circle_coeff_1.values[0] = 0;
    circle_coeff_1.values[1] = 0;
    circle_coeff_1.values[2] = 30.0;
    viewer->addCircle(circle_coeff_1, "circle_1");
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.9, 0, 0 /*R,G,B*/, "circle_1");

    pcl::ModelCoefficients circle_coeff_2;
    circle_coeff_2.values.resize(3);
    circle_coeff_2.values[0] = 0;
    circle_coeff_2.values[1] = 0;
    circle_coeff_2.values[2] = 60.0;
    viewer->addCircle(circle_coeff_2, "circle_2");
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.7, 0, 0 /*R,G,B*/, "circle_2");

    pcl::ModelCoefficients circle_coeff_3;
    circle_coeff_3.values.resize(3);
    circle_coeff_3.values[0] = 0;
    circle_coeff_3.values[1] = 0;
    circle_coeff_3.values[2] = 90.0;
    viewer->addCircle(circle_coeff_3, "circle_3");
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.5, 0, 0 /*R,G,B*/, "circle_3");

    pcl::ModelCoefficients circle_coeff_4;
    circle_coeff_4.values.resize(3);
    circle_coeff_4.values[0] = 0;
    circle_coeff_4.values[1] = 0;
    circle_coeff_4.values[2] = 120.0;
    viewer->addCircle(circle_coeff_4, "circle_4");
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.3, 0, 0 /*R,G,B*/, "circle_4");
}

template <typename PointT>
void MapViewer<PointT>::display_map_plane(boost::shared_ptr<pcl::visualization::PCLVisualizer> &viewer, const std::string &plane_name, float z_position)
{
    viewer->removeShape(plane_name);
    pcl::ModelCoefficients plane_coeff;
    //plane: z=-2.0
    plane_coeff.values.resize(4); // We need 4 values
    plane_coeff.values[0] = 0;
    plane_coeff.values[1] = 0;
    plane_coeff.values[2] = 1;
    plane_coeff.values[3] = -z_position;
    viewer->addPlane(plane_coeff, plane_name);
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.1, 0.1, 0.1 /*R,G,B*/, plane_name);
}

template <typename PointT>
void MapViewer<PointT>::update_submap_node(cloudblock_Ptrs &submaps, boost::shared_ptr<pcl::visualization::PCLVisualizer> &viewer,
                                           int display_time_ms)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr nodes(new pcl::PointCloud<pcl::PointXYZRGB>());
    for (int i = 0; i < submaps.size(); i++)
    {
        float r, g, b;
        pcl::PointXYZRGB ptc1_rgb;
        ptc1_rgb.x = submaps[i]->pose_lo(0, 3);
        ptc1_rgb.y = submaps[i]->pose_lo(1, 3);
        ptc1_rgb.z = submaps[i]->pose_lo(2, 3);
        get_random_color(r, g, b, 255);
        ptc1_rgb.r = r;
        ptc1_rgb.g = g;
        ptc1_rgb.b = b;
        nodes->points.push_back(ptc1_rgb);

        pcl::PointXYZRGB ptc2_rgb;
        ptc2_rgb.x = submaps[i]->pose_gt(0, 3);
        ptc2_rgb.y = submaps[i]->pose_gt(1, 3);
        ptc2_rgb.z = submaps[i]->pose_gt(2, 3);
        ptc2_rgb.r = r;
        ptc2_rgb.g = g;
        ptc2_rgb.b = b;
        nodes->points.push_back(ptc2_rgb);
    }
    viewer->addPointCloud(nodes, "nodes");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 8.0, "nodes");

    judge_pause(viewer, display_time_ms);
}

template <typename PointT>
void MapViewer<PointT>::update_lo_pose(Matrix4ds &poses_lo, Matrix4ds &poses_gt,
                                       boost::shared_ptr<pcl::visualization::PCLVisualizer> &viewer,
                                       int display_time_ms)
{
    std::string lo_pose_pc_name = "lo_pose";
    viewer->removePointCloud(lo_pose_pc_name);
    double font_color = 1.0 - background_color_;
    pcl::PointCloud<pcl::PointXYZ>().swap(*lo_pose_pc_);
    for (int i = 0; i < poses_lo.size(); i++)
    {
        pcl::PointXYZ pt_temp;
        pt_temp.x = poses_lo[i](0, 3);
        pt_temp.y = poses_lo[i](1, 3);
        pt_temp.z = poses_lo[i](2, 3);
        lo_pose_pc_->points.push_back(pt_temp);
    }

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> lo_pose_color(lo_pose_pc_, font_color * 255, font_color * 255, font_color * 255);
    viewer->addPointCloud(lo_pose_pc_, lo_pose_color, lo_pose_pc_name);
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, lo_pose_pc_name);

    std::string gt_pose_pc_name = "gt_pose";
    viewer->removePointCloud(gt_pose_pc_name);

    pcl::PointCloud<pcl::PointXYZ>().swap(*gt_pose_pc_);
    for (int i = 0; i < poses_gt.size(); i++)
    {
        pcl::PointXYZ pt_temp;
        pt_temp.x = poses_gt[i](0, 3);
        pt_temp.y = poses_gt[i](1, 3);
        pt_temp.z = poses_gt[i](2, 3);
        gt_pose_pc_->points.push_back(pt_temp);
    }

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> gt_pose_color(gt_pose_pc_, 231, 84, 128); //pink
    viewer->addPointCloud(gt_pose_pc_, gt_pose_color, gt_pose_pc_name);
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, gt_pose_pc_name);

    judge_pause(viewer, display_time_ms);
}

template <typename PointT>
void MapViewer<PointT>::display_lo_realtime(cloudblock_Ptr &current_frame,
                                            boost::shared_ptr<pcl::visualization::PCLVisualizer> &viewer,
                                            int display_time_ms, int display_downsample_ratio_current, int display_downsample_ratio_history)
{
    char profile_name[256];
    std::string str_origin_frame = "current_pose";
    std::string lo_pose_pc_name = "lo_pose";
    std::string gt_pose_pc_name = "gt_pose";
    std::string frame_name = "frame_id: " + std::to_string(current_frame->unique_id);
    std::string instruction = "[Space]: pause(resume)\t  [T]: show(hide) gt trajectory\t  [G]: show(hide) scale bar\t [L]: pause at loop closure\t [N]: show(hide) point normal\t [S]: show(hide) range ring\n[H]: show help and pause\t [V]: show(hide) vertex keypoints\t [F1 - 5]: switch color map(single, framewise, elevation, intensity, high contrast intensity)\n[F6]: show(hide) current scan\t [F7]: show(hide) pose graph nodes\t [F8]: show(hide) local feature map\t [F9]: show(hide) dense map\n [F10]: show(hide) feature points in feature viewer\t [F11]: show(hide) adjacent registration result in registration viewer\t [F12]: show(hide) pose graph edges\n[< >]: configure the background color\t [+ -]: configure the point size\n";
    //std::string map_plane_name = "map_plane";
    float sphere_size = laser_visualization_size_;
    int font_size_frame = 20;
    int font_size_instruction = 11;
    double font_color = 1.0 - background_color_;

    //Draw pose
    pcl::PointXYZ pt_c(current_frame->pose_lo(0, 3), current_frame->pose_lo(1, 3), current_frame->pose_lo(2, 3));
    pcl::PointXYZ pt_c_gt(current_frame->pose_gt(0, 3), current_frame->pose_gt(1, 3), current_frame->pose_gt(2, 3));

    viewer->removePointCloud(gt_pose_pc_name);
    viewer->removePointCloud(lo_pose_pc_name);

    viewer->setBackgroundColor(background_color_, background_color_, background_color_);

    if (is_seed_origin_)
    {
        viewer->addSphere(pt_c, sphere_size, font_color, font_color, font_color, str_origin_frame);
        viewer->addText(frame_name, 10, 80, font_size_frame, font_color, font_color, font_color, "frame_name");
        viewer->addText(instruction, 60, 2, font_size_instruction, font_color, font_color, font_color, "instruction");
    }
    else
    {
        viewer->updateSphere(pt_c, sphere_size, font_color, font_color, font_color, str_origin_frame);
        viewer->updateText(frame_name, 10, 80, font_size_frame, font_color, font_color, font_color, "frame_name");
        // viewer->updateText(instruction, 60, 2, font_size_instruction, font_color, font_color, font_color, "instruction");
    }
    //display_map_plane(viewer, map_plane_name, -2.0);

    lo_pose_pc_->points.push_back(pt_c);
    gt_pose_pc_->points.push_back(pt_c_gt);

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> lo_pose_color(lo_pose_pc_, font_color * 255, font_color * 255, font_color * 255);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> gt_pose_color(gt_pose_pc_, 231, 84, 128); //pink

    viewer->addPointCloud(lo_pose_pc_, lo_pose_color, lo_pose_pc_name);
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, lo_pose_pc_name);
    if (show_gt_trajectory_)
    {
        viewer->addPointCloud(gt_pose_pc_, gt_pose_color, gt_pose_pc_name);
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, gt_pose_pc_name);
    }

    if (show_sparse_map_)
    {
        std::string frame_name = "current_scan";
        sprintf(profile_name, "profile_%d", current_frame->unique_id);

        //Draw point cloud
        typename pcl::PointCloud<PointT>::Ptr pc_frame_visual(new pcl::PointCloud<PointT>);
        typename pcl::PointCloud<PointT>::Ptr pc_frame_visual_profile(new pcl::PointCloud<PointT>);

        for (size_t i = 0; i < current_frame->pc_raw_w->points.size(); ++i)
        {
            if (i % display_downsample_ratio_current == 0)
            {
                PointT pt;
                pt.x = current_frame->pc_raw_w->points[i].x;
                pt.y = current_frame->pc_raw_w->points[i].y;
                pt.z = current_frame->pc_raw_w->points[i].z;
                pt.intensity = current_frame->pc_raw_w->points[i].intensity;
                pt.curvature = current_frame->pc_raw_w->points[i].curvature;
                pc_frame_visual->points.push_back(pt);

                if (i % display_downsample_ratio_history == 0)
                    pc_frame_visual_profile->points.push_back(pt);
            }
        }
        switch (color_rendering_type_)
        {
        case SINGLE: // Golden or semantic mask
        {
            float intensity_color;
            int r_c, g_c, b_c;
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc_frame_visual_rgb(new pcl::PointCloud<pcl::PointXYZRGB>);
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc_frame_visual_rgb_profile(new pcl::PointCloud<pcl::PointXYZRGB>);
            for (size_t i = 0; i < pc_frame_visual->points.size(); ++i)
            {
                pcl::PointXYZRGB pt;
                pt.x = pc_frame_visual->points[i].x;
                pt.y = pc_frame_visual->points[i].y;
                pt.z = pc_frame_visual->points[i].z;
                pt.r = 255;
                pt.g = 215;
                pt.b = 0;
                pc_frame_visual_rgb->points.push_back(pt);
            }
            for (size_t i = 0; i < pc_frame_visual_profile->points.size(); ++i) //rendered with semantic label
            {
                pcl::PointXYZRGB pt;
                pt.x = pc_frame_visual_profile->points[i].x;
                pt.y = pc_frame_visual_profile->points[i].y;
                pt.z = pc_frame_visual_profile->points[i].z;
                apply_label_color_mapping((int)(pc_frame_visual_profile->points[i].curvature), r_c, g_c, b_c); //the curvature here indicates the point semantic label (mask)
                float intensity_color = 1.0 / intensity_scale_ * pc_frame_visual_profile->points[i].intensity;
                intensity_color = 0.2 + min_(0.8, intensity_color);
                pt.r = r_c * intensity_color;
                pt.g = g_c * intensity_color;
                pt.b = b_c * intensity_color;
                pc_frame_visual_rgb_profile->points.push_back(pt);
            }
            if (is_seed_origin_)
                viewer->addPointCloud(pc_frame_visual_rgb, frame_name);
            else
                viewer->updatePointCloud(pc_frame_visual_rgb, frame_name);
            if (current_frame->unique_id % sample_frame_rate_ == 0)
                viewer->addPointCloud(pc_frame_visual_rgb_profile, profile_name);
            break;
        }
        case FRAME: // rand
        {
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc_frame_visual_rgb(new pcl::PointCloud<pcl::PointXYZRGB>);
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc_frame_visual_rgb_profile(new pcl::PointCloud<pcl::PointXYZRGB>);
            float color_r, color_g, color_b;
            get_random_color(color_r, color_g, color_b, 255);
            for (size_t i = 0; i < pc_frame_visual->points.size(); ++i)
            {
                pcl::PointXYZRGB pt;
                pt.x = pc_frame_visual->points[i].x;
                pt.y = pc_frame_visual->points[i].y;
                pt.z = pc_frame_visual->points[i].z;
                pt.r = color_r;
                pt.g = color_g;
                pt.b = color_b;
                pc_frame_visual_rgb->points.push_back(pt);
            }
            for (size_t i = 0; i < pc_frame_visual_profile->points.size(); ++i)
            {
                pcl::PointXYZRGB pt;
                pt.x = pc_frame_visual_profile->points[i].x;
                pt.y = pc_frame_visual_profile->points[i].y;
                pt.z = pc_frame_visual_profile->points[i].z;
                pt.r = color_r;
                pt.g = color_g;
                pt.b = color_b;
                pc_frame_visual_rgb_profile->points.push_back(pt);
            }
            if (is_seed_origin_)
                viewer->addPointCloud(pc_frame_visual_rgb, frame_name);
            else
                viewer->updatePointCloud(pc_frame_visual_rgb, frame_name);
            if (current_frame->unique_id % sample_frame_rate_ == 0)
                viewer->addPointCloud(pc_frame_visual_rgb_profile, profile_name);
            break;
        }

        case HEIGHT: //Height ramp color scalar
        {
            typename pcl::visualization::PointCloudColorHandlerGenericField<PointT>::PointCloudColorHandlerGenericField rgb_z(pc_frame_visual, "z");
            typename pcl::visualization::PointCloudColorHandlerGenericField<PointT>::PointCloudColorHandlerGenericField rgb_z_p(pc_frame_visual_profile, "z");
            if (is_seed_origin_)
                viewer->addPointCloud<PointT>(pc_frame_visual, rgb_z, frame_name);
            else
                viewer->updatePointCloud<PointT>(pc_frame_visual, rgb_z, frame_name);
            if (current_frame->unique_id % sample_frame_rate_ == 0)
                viewer->addPointCloud<PointT>(pc_frame_visual_profile, rgb_z_p, profile_name);
            break;
        }

        case INTENSITY: //intensity color scalar
        {
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc_frame_visual_rgb(new pcl::PointCloud<pcl::PointXYZRGB>);
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc_frame_visual_rgb_profile(new pcl::PointCloud<pcl::PointXYZRGB>);
            float intensity_color;
            for (size_t i = 0; i < pc_frame_visual->points.size(); ++i)
            {
                pcl::PointXYZRGB pt;
                pt.x = pc_frame_visual->points[i].x;
                pt.y = pc_frame_visual->points[i].y;
                pt.z = pc_frame_visual->points[i].z;
                float intensity_color = 1.0 / intensity_scale_ * pc_frame_visual->points[i].intensity;
                intensity_color = 0.2 + min_(0.8, intensity_color);
                pt.r = 255 * intensity_color; //golden with intensity
                pt.g = 215 * intensity_color;
                pt.b = 0 * intensity_color;
                //LOG(INFO) << pc_frame_visual->points[i].intensity << "," << intensity_color << "," << pt.r;
                pc_frame_visual_rgb->points.push_back(pt);
            }
            for (size_t i = 0; i < pc_frame_visual_profile->points.size(); ++i)
            {
                pcl::PointXYZRGB pt;
                pt.x = pc_frame_visual_profile->points[i].x; //white with intensity
                pt.y = pc_frame_visual_profile->points[i].y;
                pt.z = pc_frame_visual_profile->points[i].z;
                float intensity_color = 1.0 / intensity_scale_ * pc_frame_visual_profile->points[i].intensity;
                intensity_color = 0.2 + min_(0.8, intensity_color);
                pt.r = 255 * intensity_color;
                pt.g = 255 * intensity_color;
                pt.b = 255 * intensity_color;
                pc_frame_visual_rgb_profile->points.push_back(pt);
            }
            if (is_seed_origin_)
                viewer->addPointCloud(pc_frame_visual_rgb, frame_name);
            else
                viewer->updatePointCloud(pc_frame_visual_rgb, frame_name);
            if (current_frame->unique_id % sample_frame_rate_ == 0)
                viewer->addPointCloud(pc_frame_visual_rgb_profile, profile_name);
            break;
        }

        case INTENSITY_2: //intensity color scalar  //TODO: using the look up table (LUT), although not availiable for pcl 1.7
        {
            for (int i = 0; i < pc_frame_visual->points.size(); i++)
                pc_frame_visual->points[i].intensity = intensity_scale_ * simple_look_up_table((intensity_scale_ - pc_frame_visual->points[i].intensity) / intensity_scale_,
                                                                                               intensity_lut_x_a_, intensity_lut_x_b_, intensity_lut_y_a_, intensity_lut_y_b_);
            for (int i = 0; i < pc_frame_visual_profile->points.size(); i++)
                pc_frame_visual_profile->points[i].intensity = intensity_scale_ * simple_look_up_table((intensity_scale_ - pc_frame_visual_profile->points[i].intensity) / intensity_scale_,
                                                                                                       intensity_lut_x_a_, intensity_lut_x_b_, intensity_lut_y_a_, intensity_lut_y_b_);

            typename pcl::visualization::PointCloudColorHandlerGenericField<PointT>::PointCloudColorHandlerGenericField rgb_i(pc_frame_visual, "intensity");
            typename pcl::visualization::PointCloudColorHandlerGenericField<PointT>::PointCloudColorHandlerGenericField rgb_i_p(pc_frame_visual_profile, "intensity");
            if (is_seed_origin_)
                viewer->addPointCloud<PointT>(pc_frame_visual, rgb_i, frame_name);
            else
                viewer->updatePointCloud<PointT>(pc_frame_visual, rgb_i, frame_name);
            if (current_frame->unique_id % sample_frame_rate_ == 0)
                viewer->addPointCloud<PointT>(pc_frame_visual_profile, rgb_i_p, profile_name);
            break;
        }

        default:
            break;
        }

        pc_frame_visual.reset(new pcl::PointCloud<PointT>());
        pc_frame_visual_profile.reset(new pcl::PointCloud<PointT>());

        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1.0, frame_name);
        judge_pause(viewer, display_time_ms);
    }
    else
        judge_pause(viewer, 1);
}

template <typename PointT>
void MapViewer<PointT>::display_dense_map_realtime(cloudblock_Ptr &current_frame,
                                                   boost::shared_ptr<pcl::visualization::PCLVisualizer> &viewer,
                                                   int keep_frame_num, int display_time_ms)
{
    char dense_frame_name[256];
    char history_dense_frame_name[256];
    sprintf(dense_frame_name, "dense_%d", current_frame->unique_id);
    sprintf(history_dense_frame_name, "dense_%d", current_frame->unique_id - keep_frame_num);
    viewer->removePointCloud(history_dense_frame_name);

    if (show_dense_map_)
    {
        viewer->setBackgroundColor(background_color_, background_color_, background_color_);

        switch (color_rendering_type_)
        {
        case HEIGHT: //Height ramp color scalar
        {
            typename pcl::visualization::PointCloudColorHandlerGenericField<PointT>::PointCloudColorHandlerGenericField rgb_z(current_frame->pc_raw_w, "z");
            viewer->addPointCloud<PointT>(current_frame->pc_raw_w, rgb_z, dense_frame_name);
            break;
        }
        case INTENSITY: //intensity color scalar
        {
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc_frame_dense_rgb(new pcl::PointCloud<pcl::PointXYZRGB>);
            float intensity_color;
            for (size_t i = 0; i < current_frame->pc_raw_w->points.size(); ++i)
            {
                pcl::PointXYZRGB pt;
                pt.x = current_frame->pc_raw_w->points[i].x;
                pt.y = current_frame->pc_raw_w->points[i].y;
                pt.z = current_frame->pc_raw_w->points[i].z;
                intensity_color = 0.2 + min_(0.8, 1.0 / intensity_scale_ * current_frame->pc_raw_w->points[i].intensity);
                pt.r = 255 * intensity_color;
                pt.g = 255 * intensity_color;
                pt.b = 255 * intensity_color;
                pc_frame_dense_rgb->points.push_back(pt);
            }
            viewer->addPointCloud(pc_frame_dense_rgb, dense_frame_name);
            break;
        }
        case SINGLE:
        {
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc_frame_dense_rgb(new pcl::PointCloud<pcl::PointXYZRGB>);
            float intensity_color;
            int r, g, b;
            for (size_t i = 0; i < current_frame->pc_raw_w->points.size(); ++i)
            {
                pcl::PointXYZRGB pt;
                pt.x = current_frame->pc_raw_w->points[i].x;
                pt.y = current_frame->pc_raw_w->points[i].y;
                pt.z = current_frame->pc_raw_w->points[i].z;
                apply_label_color_mapping((int)(current_frame->pc_raw_w->points[i].curvature), r, g, b);
                intensity_color = 0.2 + min_(0.8, 1.0 / intensity_scale_ * current_frame->pc_raw_w->points[i].intensity);
                pt.r = r * intensity_color;
                pt.g = g * intensity_color;
                pt.b = b * intensity_color;
                pc_frame_dense_rgb->points.push_back(pt);
            }
            viewer->addPointCloud(pc_frame_dense_rgb, dense_frame_name);
            break;
        }
        case FRAME: 
        {
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc_frame_dense_rgb(new pcl::PointCloud<pcl::PointXYZRGB>);
            float color_r, color_g, color_b;
            get_random_color(color_r, color_g, color_b, 255);
            for (size_t i = 0; i < current_frame->pc_raw_w->points.size(); ++i)
            {
                pcl::PointXYZRGB pt;
                pt.x = current_frame->pc_raw_w->points[i].x;
                pt.y = current_frame->pc_raw_w->points[i].y;
                pt.z = current_frame->pc_raw_w->points[i].z;
                pt.r = color_r;
                pt.g = color_g;
                pt.b = color_b;
                pc_frame_dense_rgb->points.push_back(pt);
            }
            viewer->addPointCloud(pc_frame_dense_rgb, dense_frame_name);
            break;
        }
        case INTENSITY_2: //TODO: check (use other color scale)
        {
            //!!! the JET LUT in pcl is from red (small) to blue (large), however, we'd like to use the LUT from blue (small) to red (large)
            typename pcl::PointCloud<PointT>::Ptr pc_clone(new pcl::PointCloud<PointT>());
            *pc_clone = *current_frame->pc_raw_w;
            for (int i = 0; i < pc_clone->points.size(); i++)
                pc_clone->points[i].intensity = intensity_scale_ * simple_look_up_table((intensity_scale_ - pc_clone->points[i].intensity) / intensity_scale_, intensity_lut_x_a_, intensity_lut_x_b_, intensity_lut_y_a_, intensity_lut_y_b_);
            typename pcl::visualization::PointCloudColorHandlerGenericField<PointT>::PointCloudColorHandlerGenericField rgb_i(pc_clone, "intensity");
            viewer->addPointCloud<PointT>(pc_clone, rgb_i, dense_frame_name);
            pc_clone.reset(new pcl::PointCloud<PointT>());

            break;
        }
        default:
            break;
        }
        judge_pause(viewer, display_time_ms);
    }
    else
        judge_pause(viewer, 1);
}

template <typename PointT>
void MapViewer<PointT>::display_feature_map_realtime(cloudblock_Ptr &local_map,
                                                     boost::shared_ptr<pcl::visualization::PCLVisualizer> &viewer,
                                                     int display_time_ms)
{
    std::string feature_map_name = "feature_map";

    viewer->removePointCloud(feature_map_name);

    if (show_feature_map_)
    {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr feature_map(new pcl::PointCloud<pcl::PointXYZRGB>);
        typename pcl::PointCloud<PointT>::Ptr ground_pts(new pcl::PointCloud<PointT>);
        typename pcl::PointCloud<PointT>::Ptr facade_pts(new pcl::PointCloud<PointT>);
        typename pcl::PointCloud<PointT>::Ptr roof_pts(new pcl::PointCloud<PointT>);
        typename pcl::PointCloud<PointT>::Ptr pillar_pts(new pcl::PointCloud<PointT>);
        typename pcl::PointCloud<PointT>::Ptr beam_pts(new pcl::PointCloud<PointT>);
        typename pcl::PointCloud<PointT>::Ptr vertex_pts(new pcl::PointCloud<PointT>);
        // transform to world coordinate system
        pcl::transformPointCloudWithNormals(*local_map->pc_ground, *ground_pts, local_map->pose_lo);
        pcl::transformPointCloudWithNormals(*local_map->pc_facade, *facade_pts, local_map->pose_lo);
        pcl::transformPointCloudWithNormals(*local_map->pc_roof, *roof_pts, local_map->pose_lo);
        pcl::transformPointCloudWithNormals(*local_map->pc_pillar, *pillar_pts, local_map->pose_lo);
        pcl::transformPointCloudWithNormals(*local_map->pc_beam, *beam_pts, local_map->pose_lo);
        pcl::transformPointCloudWithNormals(*local_map->pc_vertex, *vertex_pts, local_map->pose_lo);

        //change color map to semantic kitti format
        for (int i = 0; i < ground_pts->points.size(); ++i)
        {
            pcl::PointXYZRGB pt;
            pt.x = ground_pts->points[i].x;
            pt.y = ground_pts->points[i].y;
            pt.z = ground_pts->points[i].z;
            pt.r = 128;
            pt.g = 128;
            pt.b = 128;
            //Ground - Gray
            feature_map->points.push_back(pt);
        }
        for (int i = 0; i < pillar_pts->points.size(); ++i)
        {
            pcl::PointXYZRGB pt;
            pt.x = pillar_pts->points[i].x;
            pt.y = pillar_pts->points[i].y;
            pt.z = pillar_pts->points[i].z;
            pt.r = 0;
            pt.g = 255;
            pt.b = 0;
            //Pillar - Green
            feature_map->points.push_back(pt);
        }
        for (int i = 0; i < beam_pts->points.size(); ++i)
        {
            pcl::PointXYZRGB pt;
            pt.x = beam_pts->points[i].x;
            pt.y = beam_pts->points[i].y;
            pt.z = beam_pts->points[i].z;
            pt.r = 255;
            pt.g = 255;
            pt.b = 0;
            //Beam - Yellow
            feature_map->points.push_back(pt);
        }
        for (int i = 0; i < facade_pts->points.size(); ++i)
        {
            pcl::PointXYZRGB pt;
            pt.x = facade_pts->points[i].x;
            pt.y = facade_pts->points[i].y;
            pt.z = facade_pts->points[i].z;
            pt.r = 0;
            pt.g = 0;
            pt.b = 255;
            //Facade - Blue
            feature_map->points.push_back(pt);
        }
        for (int i = 0; i < roof_pts->points.size(); ++i)
        {
            pcl::PointXYZRGB pt;
            pt.x = roof_pts->points[i].x;
            pt.y = roof_pts->points[i].y;
            pt.z = roof_pts->points[i].z;
            pt.r = 255;
            pt.g = 0;
            pt.b = 0;
            //Roof - Red
            feature_map->points.push_back(pt);
        }

        if (show_vertex_keypoints_)
        {
            for (int i = 0; i < vertex_pts->points.size(); ++i)
            {
                pcl::PointXYZRGB pt;
                pt.x = vertex_pts->points[i].x;
                pt.y = vertex_pts->points[i].y;
                pt.z = vertex_pts->points[i].z;
                pt.r = 255;
                pt.g = 0;
                pt.b = 255;
                //Vertex - Purple
                feature_map->points.push_back(pt);
            }
        }

        viewer->addPointCloud(feature_map, feature_map_name);

        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4.0, feature_map_name);

        pcl::PointCloud<pcl::PointXYZRGB>().swap(*feature_map);
        pcl::PointCloud<PointT>().swap(*facade_pts);
        pcl::PointCloud<PointT>().swap(*pillar_pts);
        pcl::PointCloud<PointT>().swap(*ground_pts);
        pcl::PointCloud<PointT>().swap(*vertex_pts);
        pcl::PointCloud<PointT>().swap(*roof_pts);
        pcl::PointCloud<PointT>().swap(*beam_pts);

        judge_pause(viewer, display_time_ms);
    }
    else
        judge_pause(viewer, 1);
}

template <typename PointT>
void MapViewer<PointT>::highlight_problematic_frame(boost::shared_ptr<pcl::visualization::PCLVisualizer> &viewer,
                                                    bool horizontal_flag, bool vertical_flag, bool heading_flag, bool pause_or_not)
{
    if (is_seed_origin_)
    {
        if (horizontal_flag)
            viewer->addText("t-xy", 200, 70, 18, 1.0, 0.0, 0.0, "hor_log");
        else
            viewer->addText(" ", 200, 70, 18, 1.0, 0.0, 0.0, "hor_log");
        if (vertical_flag)
            viewer->addText("t-z", 300, 70, 18, 0.0, 1.0, 0.0, "ver_log");
        else
            viewer->addText(" ", 300, 70, 18, 0.0, 1.0, 0.0, "ver_log");
        if (heading_flag)
            viewer->addText("r-y", 400, 70, 18, 0.0, 0.0, 1.0, "yaw_log");
        else
            viewer->addText(" ", 400, 70, 18, 0.0, 0.0, 1.0, "yaw_log");
    }
    else
    {
        if (horizontal_flag)
            viewer->updateText("t-xy", 200, 70, 18, 1.0, 0.0, 0.0, "hor_log");
        else
            viewer->updateText(" ", 200, 70, 18, 1.0, 0.0, 0.0, "hor_log");
        if (vertical_flag)
            viewer->updateText("t-z", 300, 70, 18, 0.0, 1.0, 0.0, "ver_log");
        else
            viewer->updateText(" ", 300, 70, 18, 0.0, 1.0, 0.0, "ver_log");
        if (heading_flag)
            viewer->updateText("r-y", 400, 70, 18, 0.0, 0.0, 1.0, "yaw_log");
        else
            viewer->updateText(" ", 400, 70, 18, 0.0, 0.0, 1.0, "yaw_log");
    }

    bool problematic = horizontal_flag || vertical_flag || heading_flag;
    if (pause_or_not && problematic)
    {
        set_pause(true);
        show_reg_ = true;
    }
}

template <typename PointT>
void MapViewer<PointT>::display_correspondences(boost::shared_ptr<pcl::visualization::PCLVisualizer> &viewer,
                                                const typename pcl::PointCloud<PointT>::Ptr &kpc_0, const typename pcl::PointCloud<PointT>::Ptr &kpc_1,
                                                const typename pcl::PointCloud<PointT>::Ptr &pc_0, const typename pcl::PointCloud<PointT>::Ptr &pc_1,
                                                Eigen::Matrix4d &pre_tran, int display_time_ms)

{
    viewer->removeAllPointClouds();
    viewer->removeAllShapes();

    if (show_correspondences_)
    {
        float line_width = 1.5;

        Eigen::Matrix4d pre_tranmat = pre_tran;
        pre_tranmat.setIdentity();
        pre_tranmat(0, 3) += 0.0;
        pre_tranmat(1, 3) += 30.0;
        pre_tranmat(2, 3) += 20.0;

        bool intensity_availiable = false;
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_0(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_1(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr keypoints_0(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr keypoints_1(new pcl::PointCloud<pcl::PointXYZRGB>);

        typename pcl::PointCloud<PointT>::Ptr kpc_0_copy(new pcl::PointCloud<PointT>());
        *kpc_0_copy = *kpc_0;

        //judge if point cloud intensity is availiable
        for (int i = 0; i < pc_0->points.size(); i += 50)
        {
            if (pc_0->points[i].intensity > 1e-12)
            {
                intensity_availiable = true;
                break;
            }
        }
        float intensity_color;

        for (int i = 0; i < pc_0->points.size(); ++i)
        {
            pcl::PointXYZRGB pt;
            pt.x = pc_0->points[i].x;
            pt.y = pc_0->points[i].y;
            pt.z = pc_0->points[i].z;
            intensity_color = 0.2 + min_(0.8, 1.0 / intensity_scale_ * pc_0->points[i].intensity);
            if (!intensity_availiable)
                intensity_color = 1.0;
            pt.r = 255 * intensity_color;
            pt.g = 215 * intensity_color;
            pt.b = 0 * intensity_color;
            cloud_0->points.push_back(pt);
        } // Golden

        pcl::transformPointCloud(*cloud_0, *cloud_0, pre_tranmat);
        viewer->addPointCloud(cloud_0, "pointcloudS");

        for (int i = 0; i < pc_1->points.size(); ++i)
        {
            pcl::PointXYZRGB pt;
            pt.x = pc_1->points[i].x;
            pt.y = pc_1->points[i].y;
            pt.z = pc_1->points[i].z;
            intensity_color = 0.2 + min_(0.8, 1.0 / intensity_scale_ * pc_1->points[i].intensity);
            if (!intensity_availiable)
                intensity_color = 1.0;
            pt.r = 233 * intensity_color;
            pt.g = 233 * intensity_color;
            pt.b = 216 * intensity_color;
            cloud_1->points.push_back(pt);
        } // Silver
        viewer->addPointCloud(cloud_1, "pointcloudT");

        for (int i = 0; i < kpc_0->points.size(); ++i)
        {
            pcl::PointXYZRGB pt;
            pt.x = kpc_0->points[i].x;
            pt.y = kpc_0->points[i].y;
            pt.z = kpc_0->points[i].z;
            pt.r = 255;
            pt.g = 0;
            pt.b = 0;
            keypoints_0->points.push_back(pt);
        } // Red

        pcl::transformPointCloud(*keypoints_0, *keypoints_0, pre_tranmat);
        viewer->addPointCloud(keypoints_0, "keypointS");

        for (int i = 0; i < kpc_1->points.size(); ++i)
        {
            pcl::PointXYZRGB pt;
            pt.x = kpc_1->points[i].x;
            pt.y = kpc_1->points[i].y;
            pt.z = kpc_1->points[i].z;
            pt.r = 0;
            pt.g = 0;
            pt.b = 255;
            keypoints_1->points.push_back(pt);
        } // Blue
        viewer->addPointCloud(keypoints_1, "keypointT");

        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 0.5, "pointcloudS");
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 0.5, "pointcloudT");
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4.0, "keypointS");
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4.0, "keypointT");

        int corr_num = kpc_0->points.size();
        std::vector<unsigned int> red(corr_num, 0), green(corr_num, 255), blue(corr_num, 0);

        pcl::transformPointCloud(*kpc_0_copy, *kpc_0_copy, pre_tranmat);

        add_lines_to_viewer(viewer, kpc_0_copy, kpc_1, "corres", red, green, blue);

        viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, line_width, "corres");

        kpc_0_copy.reset(new pcl::PointCloud<PointT>());

        judge_pause(viewer, display_time_ms);
    }
    else
        judge_pause(viewer, 1);
}

//show correspondences before and after the regsitration
template <typename PointT>
void MapViewer<PointT>::display_correspondences_compare(boost::shared_ptr<pcl::visualization::PCLVisualizer> &viewer,
                                                        const typename pcl::PointCloud<PointT>::Ptr &kpc_source,
                                                        const typename pcl::PointCloud<PointT>::Ptr &kpc_target,
                                                        const typename pcl::PointCloud<PointT>::Ptr &kpc_source_tran,
                                                        const typename pcl::PointCloud<PointT>::Ptr &pc_source,
                                                        const typename pcl::PointCloud<PointT>::Ptr &pc_target,
                                                        const typename pcl::PointCloud<PointT>::Ptr &pc_source_tran,
                                                        Eigen::Matrix4d &pre_tran, int down_rate, int display_time_ms)

{
    int v1(0);
    int v2(1);

    viewer->removeAllPointClouds(v1);
    viewer->removeAllPointClouds(v2);
    viewer->removeCoordinateSystem();
    viewer->removeAllShapes();

    if (show_correspondences_)
    {
        float line_width = 2.0;

        //Create two vertically separated viewports
        viewer->createViewPort(0.0, 0.0, 0.5, 1.0, v1);
        viewer->createViewPort(0.5, 0.0, 1.0, 1.0, v2);

        viewer->setBackgroundColor(background_color_, background_color_, background_color_);

        Eigen::Matrix4d pre_tranmat = pre_tran;
        //pre_tranmat.setIdentity();
        pre_tranmat(0, 3) += 0.0;
        pre_tranmat(1, 3) += 0.0;
        pre_tranmat(2, 3) += 30.0;

        bool intensity_availiable = false;
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_source(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_target(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_source_tran(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr keypoints_source(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr keypoints_target(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr keypoints_source_tran(new pcl::PointCloud<pcl::PointXYZRGB>);
        typename pcl::PointCloud<PointT>::Ptr keypoints_source_copy(new pcl::PointCloud<PointT>());
        typename pcl::PointCloud<PointT>::Ptr kpc_source_tran_correct(new pcl::PointCloud<PointT>());
        typename pcl::PointCloud<PointT>::Ptr kpc_target_correct(new pcl::PointCloud<PointT>());

        *keypoints_source_copy = *kpc_source;
        // *keypoints_source_tran_copy = *kpc_source_tran;

        //judge if point cloud intensity is availiable
        for (int i = 0; i < pc_source->points.size(); i += 50)
        {
            if (kpc_source->points[i].intensity > 1e-12)
            {
                intensity_availiable = true;
                break;
            }
        }
        float intensity_color;

        for (int i = 0; i < pc_source->points.size(); ++i)
        {
            pcl::PointXYZRGB pt;
            pt.x = pc_source->points[i].x;
            pt.y = pc_source->points[i].y;
            pt.z = pc_source->points[i].z;
            intensity_color = 0.2 + min_(0.8, 1.0 / intensity_scale_ * pc_source->points[i].intensity);
            if (!intensity_availiable)
                intensity_color = 1.0;
            //LOG(INFO)<< intensity_color;
            pt.r = 255 * intensity_color;
            pt.g = 215 * intensity_color;
            pt.b = 0 * intensity_color;
            cloud_source->points.push_back(pt);
        } // Golden

        pcl::transformPointCloud(*cloud_source, *cloud_source, pre_tranmat);
        viewer->addPointCloud(cloud_source, "pointcloudS", v1);

        for (int i = 0; i < pc_target->points.size(); ++i)
        {
            pcl::PointXYZRGB pt;
            pt.x = pc_target->points[i].x;
            pt.y = pc_target->points[i].y;
            pt.z = pc_target->points[i].z;
            intensity_color = 0.2 + min_(0.8, 1.0 / intensity_scale_ * pc_target->points[i].intensity);
            if (!intensity_availiable)
                intensity_color = 1.0;
            pt.r = 233 * intensity_color;
            pt.g = 233 * intensity_color;
            pt.b = 216 * intensity_color;
            cloud_target->points.push_back(pt);
        } // Silver
        viewer->addPointCloud(cloud_target, "pointcloudT", v1);
        viewer->addPointCloud(cloud_target, "pointcloudT_right", v2);

        for (int i = 0; i < pc_source_tran->points.size(); ++i)
        {
            pcl::PointXYZRGB pt;
            pt.x = pc_source_tran->points[i].x;
            pt.y = pc_source_tran->points[i].y;
            pt.z = pc_source_tran->points[i].z;
            intensity_color = 0.2 + min_(0.8, 1.0 / intensity_scale_ * pc_source_tran->points[i].intensity);
            if (!intensity_availiable)
                intensity_color = 1.0;
            pt.r = 255 * intensity_color;
            pt.g = 215 * intensity_color;
            pt.b = 0 * intensity_color;
            cloud_source_tran->points.push_back(pt);
        } // Golden
        viewer->addPointCloud(cloud_source_tran, "pointcloudS_tran", v2);

        for (int i = 0; i < kpc_source->points.size(); ++i)
        {
            pcl::PointXYZRGB pt;
            pt.x = kpc_source->points[i].x;
            pt.y = kpc_source->points[i].y;
            pt.z = kpc_source->points[i].z;
            pt.r = 255;
            pt.g = 0;
            pt.b = 0;
            keypoints_source->points.push_back(pt);
        } // Red

        pcl::transformPointCloud(*keypoints_source, *keypoints_source, pre_tranmat);
        viewer->addPointCloud(keypoints_source, "keypointS", v1);

        for (int i = 0; i < kpc_target->points.size(); ++i)
        {
            pcl::PointXYZRGB pt;
            pt.x = kpc_target->points[i].x;
            pt.y = kpc_target->points[i].y;
            pt.z = kpc_target->points[i].z;
            pt.r = 0;
            pt.g = 0;
            pt.b = 255;
            keypoints_target->points.push_back(pt);
        } // Blue
        viewer->addPointCloud(keypoints_target, "keypointT", v1);
        viewer->addPointCloud(keypoints_target, "keypointT_right", v2);

        for (int i = 0; i < kpc_source_tran->points.size(); ++i)
        {
            pcl::PointXYZRGB pt;
            pt.x = kpc_source_tran->points[i].x;
            pt.y = kpc_source_tran->points[i].y;
            pt.z = kpc_source_tran->points[i].z;
            pt.r = 255;
            pt.g = 0;
            pt.b = 0;
            keypoints_source_tran->points.push_back(pt);
        } // Red
        viewer->addPointCloud(keypoints_source_tran, "keypointS_tran", v2);

        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 0.5, "pointcloudS");
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 0.5, "pointcloudT");
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 0.5, "pointcloudT_right");
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 0.5, "pointcloudS_tran");
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4.0, "keypointS");
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4.0, "keypointT");
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4.0, "keypointT_right");
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4.0, "keypointS_tran");

        int corr_num = kpc_source->points.size();
        std::vector<unsigned int> red(corr_num, 0), green(corr_num, 255), blue(corr_num, 0);

        pcl::transformPointCloud(*keypoints_source_copy, *keypoints_source_copy, pre_tranmat);

        add_lines_to_viewer(viewer, keypoints_source_copy, kpc_target, "corres", red, green, blue, v1, down_rate);

        std::vector<unsigned int>().swap(red);
        std::vector<unsigned int>().swap(green);
        std::vector<unsigned int>().swap(blue);

        float dist_thre = 2.5;
        for (int i = 0; i < kpc_source_tran->points.size(); i++)
        {
            float dist = std::sqrt((kpc_source_tran->points[i].x - kpc_target->points[i].x) * (kpc_source_tran->points[i].x - kpc_target->points[i].x) + (kpc_source_tran->points[i].y - kpc_target->points[i].y) * (kpc_source_tran->points[i].y - kpc_target->points[i].y) + (kpc_source_tran->points[i].z - kpc_target->points[i].z) * (kpc_source_tran->points[i].z - kpc_target->points[i].z));
            if (dist < dist_thre)
            {
                kpc_source_tran_correct->points.push_back(kpc_source_tran->points[i]);
                kpc_target_correct->points.push_back(kpc_target->points[i]);
                red.push_back(0);
                green.push_back(255);
                blue.push_back(0);
            }
        }
        add_lines_to_viewer(viewer, kpc_source_tran_correct, kpc_target_correct, "corres_tran", red, green, blue, v2, 1);

        viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, line_width, "corres");
        viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, line_width, "corres_tran");

        keypoints_source_copy.reset(new pcl::PointCloud<PointT>());
        kpc_source_tran_correct.reset(new pcl::PointCloud<PointT>());
        kpc_target_correct.reset(new pcl::PointCloud<PointT>());

        if (show_distance_circles_)
            display_distance_circle(viewer);

        judge_pause(viewer, display_time_ms);
    }
    else
        judge_pause(viewer, 1);
}

template <typename PointT>
void MapViewer<PointT>::add_lines_to_viewer(boost::shared_ptr<pcl::visualization::PCLVisualizer> &viewer,
                                            const typename pcl::PointCloud<PointT>::Ptr pts_1, const typename pcl::PointCloud<PointT>::Ptr pts_2,
                                            const std::string lines_name,
                                            const std::vector<unsigned int> &red, const std::vector<unsigned int> &green, const std::vector<unsigned int> &blue,
                                            const int view_port, int down_rate)

{
    //reference: https://vtk.org/Wiki/VTK/Examples/Cxx/GeometricObjects/ColoredLines

    vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
    vtkSmartPointer<vtkCellArray> cells = vtkSmartPointer<vtkCellArray>::New();
    vtkSmartPointer<vtkLine> line;
    vtkSmartPointer<vtkPolyData> polyData = vtkSmartPointer<vtkPolyData>::New();
    vtkSmartPointer<vtkUnsignedCharArray> colors = vtkSmartPointer<vtkUnsignedCharArray>::New();

    int line_num = min_(pts_1->points.size(), pts_2->points.size());
    for (int i = 0; i < line_num; i++)
    {
        points->InsertNextPoint(pts_1->points[i].data);
        points->InsertNextPoint(pts_2->points[i].data);
    }

    // Add the points to the dataset
    polyData->SetPoints(points);

    colors->SetNumberOfComponents(3);

    unsigned char rgb[3];

    for (int i = 0; i < line_num; i++)
    {
        if (i % down_rate == 0)
        {
            line = vtkSmartPointer<vtkLine>::New();
            line->GetPointIds()->SetNumberOfIds(2);
            line->GetPointIds()->SetId(0, 2 * i);
            line->GetPointIds()->SetId(1, 2 * i + 1);
            cells->InsertNextCell(line);
            rgb[0] = red[i];
            rgb[1] = green[i];
            rgb[2] = blue[i];
#if VTK_MAJOR_VERSION < 7
            colors->InsertNextTupleValue(rgb);
#else
            colors->InsertNextTypedTuple(rgb);
#endif
        }
    }

    // Add the lines to the dataset
    polyData->SetLines(cells);
    // Add the color
    polyData->GetCellData()->SetScalars(colors);

    viewer->addModelFromPolyData(polyData, lines_name, view_port);
}

template <typename PointT>
void MapViewer<PointT>::add_triangles_to_viewer(boost::shared_ptr<pcl::visualization::PCLVisualizer> &viewer,
                                                const typename pcl::PointCloud<PointT>::Ptr pts_1, const typename pcl::PointCloud<PointT>::Ptr pts_2, const typename pcl::PointCloud<PointT>::Ptr pts_3,
                                                const std::string lines_name,
                                                const std::vector<unsigned int> &red, const std::vector<unsigned int> &green, const std::vector<unsigned int> &blue,
                                                const int view_port)
{
    //reference: https://vtk.org/Wiki/VTK/Examples/Cxx/GeometricObjects/ColoredLines

    vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
    vtkSmartPointer<vtkCellArray> cells = vtkSmartPointer<vtkCellArray>::New();
    vtkSmartPointer<vtkPolyLine> polyline;
    vtkSmartPointer<vtkPolyData> polyData = vtkSmartPointer<vtkPolyData>::New();
    vtkSmartPointer<vtkUnsignedCharArray> colors = vtkSmartPointer<vtkUnsignedCharArray>::New();

    int line_num = min_(min_(pts_1->points.size(), pts_2->points.size()), min_(pts_1->points.size(), pts_3->points.size()));

    for (int i = 0; i < line_num; i++)
    {
        points->InsertNextPoint(pts_1->points[i].data);
        points->InsertNextPoint(pts_2->points[i].data);
        points->InsertNextPoint(pts_3->points[i].data);
        points->InsertNextPoint(pts_1->points[i].data);
    }

    // Add the points to the dataset
    polyData->SetPoints(points);

    colors->SetNumberOfComponents(3);

    unsigned char rgb[3];

    for (int i = 0; i < line_num; i++)
    {
        polyline = vtkSmartPointer<vtkPolyLine>::New();
        polyline->GetPointIds()->SetNumberOfIds(4);
        polyline->GetPointIds()->SetId(0, 4 * i);
        polyline->GetPointIds()->SetId(1, 4 * i + 1);
        polyline->GetPointIds()->SetId(2, 4 * i + 2);
        polyline->GetPointIds()->SetId(3, 4 * i + 3);
        cells->InsertNextCell(polyline);
        rgb[0] = red[i];
        rgb[1] = green[i];
        rgb[2] = blue[i];
#if VTK_MAJOR_VERSION < 7
        colors->InsertNextTupleValue(rgb);
#else
        colors->InsertNextTypedTuple(rgb);
#endif
    }

    // Add the lines to the dataset
    polyData->SetLines(cells);
    // Add the color
    polyData->GetCellData()->SetScalars(colors);

    viewer->addModelFromPolyData(polyData, lines_name, view_port);
}

template <typename PointT>
void MapViewer<PointT>::display_cross_section_compare_realtime(std::vector<typename pcl::PointCloud<PointT>::Ptr> &cross_cloud_S,
                                                               std::vector<typename pcl::PointCloud<PointT>::Ptr> &cross_cloud_T,
                                                               std::vector<typename pcl::PointCloud<PointT>::Ptr> &cross_cloud_SR,
                                                               boost::shared_ptr<pcl::visualization::PCLVisualizer> &viewer,
                                                               int display_time_ms, int display_downsample_ratio)
{

    //Create 4 viewports
    int v1(0);
    int v2(1);
    int v3(2);
    int v4(3);

    if (is_seed_origin_)
    {
        viewer->setSize(1920, 1080); // Visualiser window size
        viewer->setBackgroundColor(0, 0, 0);
        viewer->addCoordinateSystem(1.0);
        display_time_ms = 3000;
    }
    else
    {
        viewer->removeAllPointClouds(v1);
        viewer->removeAllPointClouds(v2);
        viewer->removeAllPointClouds(v3);
        viewer->removeAllPointClouds(v4);
    }

    viewer->createViewPort(0.0, 0.0, 0.5, 0.5, v1);
    viewer->createViewPort(0.5, 0.0, 1.0, 0.5, v2);
    viewer->createViewPort(0.0, 0.5, 0.5, 1.0, v3);
    viewer->createViewPort(0.5, 0.5, 1.0, 1.0, v4);

    // Set camera position and orientation
    // float x_position = cross_cloud_S[0]->points[0].x;
    // float y_position = cross_cloud_S[0]->points[0].y;
    // float z_position = cross_cloud_S[0]->points[0].z;

    // viewer->setCameraPosition(x_position, y_position, z_position, 1, 0, 0, 0, 0, 1, 0);
    // viewer->setCameraPosition(x_position, y_position, z_position, 1, 0, 0, 0, 0, 1, 1);
    // viewer->setCameraPosition(x_position, y_position, z_position, 1, 0, 0, 0, 0, 1, 2);
    // viewer->setCameraPosition(x_position, y_position, z_position, 1, 0, 0, 0, 0, 1, 3);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud1s(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud1t(new pcl::PointCloud<pcl::PointXYZRGB>);

    for (size_t i = 0; i < cross_cloud_S[0]->points.size(); ++i)
    {
        if (i % display_downsample_ratio == 0)
        {
            pcl::PointXYZRGB pt;
            pt.x = cross_cloud_S[0]->points[i].x;
            pt.y = cross_cloud_S[0]->points[i].y;
            pt.z = cross_cloud_S[0]->points[i].z;
            pt.r = 255;
            pt.g = 0;
            pt.b = 0;
            pointcloud1s->points.push_back(pt);
        }
    } //Red

    viewer->addPointCloud(pointcloud1s, "pointcloudS_1", v1);

    for (size_t i = 0; i < cross_cloud_T[0]->points.size(); ++i)
    {
        if (i % display_downsample_ratio == 0)
        {
            pcl::PointXYZRGB pt;
            pt.x = cross_cloud_T[0]->points[i].x;
            pt.y = cross_cloud_T[0]->points[i].y;
            pt.z = cross_cloud_T[0]->points[i].z;
            pt.r = 0;
            pt.g = 0;
            pt.b = 255;
            pointcloud1t->points.push_back(pt);
        }
    } //Blue

    viewer->addPointCloud(pointcloud1t, "pointcloudT_1", v1);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud2s(new pcl::PointCloud<pcl::PointXYZRGB>);

    for (size_t i = 0; i < cross_cloud_SR[0]->points.size(); ++i)
    {
        if (i % display_downsample_ratio == 0)
        {
            pcl::PointXYZRGB pt;
            pt.x = cross_cloud_SR[0]->points[i].x;
            pt.y = cross_cloud_SR[0]->points[i].y;
            pt.z = cross_cloud_SR[0]->points[i].z;
            pt.r = 0;
            pt.g = 255;
            pt.b = 0;
            pointcloud2s->points.push_back(pt);
        }
    } //Green

    viewer->addPointCloud(pointcloud2s, "pointcloudS_2", v2);
    viewer->addPointCloud(pointcloud1t, "pointcloudT_2", v2);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud3s(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud3t(new pcl::PointCloud<pcl::PointXYZRGB>);

    for (size_t i = 0; i < cross_cloud_S[1]->points.size(); ++i)
    {
        if (i % display_downsample_ratio == 0)
        {
            pcl::PointXYZRGB pt;
            pt.x = cross_cloud_S[1]->points[i].x;
            pt.y = cross_cloud_S[1]->points[i].y;
            pt.z = cross_cloud_S[1]->points[i].z;
            pt.r = 255;
            pt.g = 0;
            pt.b = 0;
            pointcloud3s->points.push_back(pt);
        }
    } //Red

    viewer->addPointCloud(pointcloud3s, "pointcloudS_3", v3);

    for (size_t i = 0; i < cross_cloud_T[1]->points.size(); ++i)
    {
        if (i % display_downsample_ratio == 0)
        {
            pcl::PointXYZRGB pt;
            pt.x = cross_cloud_T[1]->points[i].x;
            pt.y = cross_cloud_T[1]->points[i].y;
            pt.z = cross_cloud_T[1]->points[i].z;
            pt.r = 0;
            pt.g = 0;
            pt.b = 255;
            pointcloud3t->points.push_back(pt);
        }
    } //Blue

    viewer->addPointCloud(pointcloud3t, "pointcloudT_3", v3);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud4s(new pcl::PointCloud<pcl::PointXYZRGB>);

    for (size_t i = 0; i < cross_cloud_SR[1]->points.size(); ++i)
    {
        if (i % display_downsample_ratio == 0)
        {
            pcl::PointXYZRGB pt;
            pt.x = cross_cloud_SR[1]->points[i].x;
            pt.y = cross_cloud_SR[1]->points[i].y;
            pt.z = cross_cloud_SR[1]->points[i].z;
            pt.r = 0;
            pt.g = 255;
            pt.b = 0;
            pointcloud4s->points.push_back(pt);
        }
    } //Green

    viewer->addPointCloud(pointcloud4s, "pointcloudS_4", v4);
    viewer->addPointCloud(pointcloud3t, "pointcloudT_4", v4);

    judge_pause(viewer, display_time_ms);
}

//this is a temp function for the color mapping of object detection
template <typename PointT>
void MapViewer<PointT>::assign_color_by_class(int category, int &r, int &g, int &b)
{
    switch (category)
    {
    case 1:
    {
        r = 255;
        g = 0;
        b = 0;
        break;
    }
    case 2:
    {
        r = 0;
        g = 255;
        b = 0;
        break;
    }
    case 3:
    {
        r = 0;
        g = 0;
        b = 255;
        break;
    }
    case 4:
    {
        r = 0;
        g = 255;
        b = 255;
        break;
    }
    case 5:
    {
        r = 255;
        g = 0;
        b = 255;
        break;
    }
    case 6:
    {
        r = 255;
        g = 255;
        b = 0;
        break;
    }
    case 7:
    {
        r = 0;
        g = 0;
        b = 0;
        break;
    }
    default:
    {
        r = 255;
        g = 255;
        b = 255;
    }
    }
}

template <typename PointT>
void MapViewer<PointT>::apply_label_color_mapping(int semantic_kitti_label, int &r, int &g, int &b)
{
    switch (semantic_kitti_label)
    {
    case 0: //unlabeled
    {
        r = 0;
        g = 0;
        b = 0;
        break;
    }
    case 1: //outlier
    {
        r = 255;
        g = 0;
        b = 0;
        break;
    }
    case 10: //car
    {
        r = 100;
        g = 150;
        b = 245;
        break;
    }
    case 11: //bicycle
    {
        r = 100;
        g = 230;
        b = 245;
        break;
    }
    case 13: //bus
    {
        r = 100;
        g = 80;
        b = 250;
        break;
    }
    case 15: //motorcycle
    {
        r = 30;
        g = 60;
        b = 150;
        break;
    }
    case 16: //on-rails
    {
        r = 0;
        g = 0;
        b = 255;
        break;
    }
    case 18: //truck
    {
        r = 80;
        g = 30;
        b = 180;
        break;
    }
    case 20: //other-vehicle
    {
        r = 0;
        g = 0;
        b = 255;
        break;
    }
    case 30: //person
    {
        r = 255;
        g = 30;
        b = 30;
        break;
    }
    case 31: //bicyclist
    {
        r = 255;
        g = 40;
        b = 200;
        break;
    }
    case 32: //motorcyclist
    {
        r = 150;
        g = 30;
        b = 90;
        break;
    }
    case 40: //road
    {
        r = 255;
        g = 0;
        b = 255;
        break;
    }
    case 44: //parking
    {
        r = 255;
        g = 150;
        b = 255;
        break;
    }
    case 48: //sidewalk
    {
        r = 75;
        g = 0;
        b = 75;
        break;
    }
    case 49: //other-ground
    {
        r = 175;
        g = 0;
        b = 75;
        break;
    }
    case 50: //building
    {
        r = 255;
        g = 200;
        b = 0;
        break;
    }
    case 51: //fence
    {
        r = 255;
        g = 120;
        b = 50;
        break;
    }
    case 52: //other-structure
    {
        r = 255;
        g = 150;
        b = 0;
        break;
    }
    case 60: //lane-marking
    {
        r = 150;
        g = 255;
        b = 170;
        break;
    }
    case 70: //vegetation
    {
        r = 0;
        g = 175;
        b = 0;
        break;
    }
    case 71: //trunk
    {
        r = 135;
        g = 60;
        b = 0;
        break;
    }
    case 72: //terrain
    {
        r = 150;
        g = 240;
        b = 80;
        break;
    }
    case 80: //pole
    {
        r = 255;
        g = 240;
        b = 150;
        break;
    }
    case 81: //traffic-sign
    {
        r = 255;
        g = 0;
        b = 0;
        break;
    }
    case 99: //other-object
    {
        r = 50;
        g = 255;
        b = 255;
        break;
    }
    default: //moving objects
    {
        r = 0;
        g = 0;
        b = 0;
    }
    }
}

#endif //_INCLUDE_MAP_VIEWER_HPP