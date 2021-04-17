//
// This file is for the implements of Lidar Odometry Replay and Checking
// Compulsory Dependent 3rd Libs: PCL (>1.7)
// By Yue Pan  
//

#include <iostream>

#include "utility.hpp"
#include "dataio.hpp"
#include "map_viewer.h"

#define launch_reg_viewer true
#define screen_width 1920
#define screen_height 1080

using namespace lo;

void check_frame(std::vector<std::vector<unsigned int>> &problematic_frame_ids, int frame_id,
                 bool &horizontal_flag, bool &vertical_flag, bool &heading_flag)
{
    horizontal_flag = false;
    vertical_flag = false;
    heading_flag = false;
    for (int i = 0; i < problematic_frame_ids[0].size(); i++)
    {
        if (problematic_frame_ids[0][i] == frame_id)
        {
            horizontal_flag = true;
            break;
        }
    }
    for (int i = 0; i < problematic_frame_ids[1].size(); i++)
    {
        if (problematic_frame_ids[1][i] == frame_id)
        {
            vertical_flag = true;
            break;
        }
    }
    for (int i = 0; i < problematic_frame_ids[2].size(); i++)
    {
        if (problematic_frame_ids[2][i] == frame_id)
        {
            heading_flag = true;
            break;
        }
    }
}

//TODO: add motion compensation !!!

int main(int argc, char **argv)
{
    std::string pointcloud_file_folder = argv[1];
    std::string lo_pose_lidar_cs_file = argv[2];
    std::string gt_pose_lidar_cs_file = argv[3];
    std::string evaluation_file = argv[4];
    std::string pc_format = argv[5];
    int frame_begin = atoi(argv[6]);
    int frame_end = atoi(argv[7]);
    int frame_step = atoi(argv[8]);
    int downsamping_rate_map_vis = atoi(argv[9]);

    pcl::console::setVerbosityLevel(pcl::console::L_ALWAYS); //Ban pcl warnings

    bool run_or_not = false;
    std::cout << "Do you want to replay the slam procedure ?  0. No  1. Yes [default 0]\n";
    std::cin >> run_or_not;
    if (!run_or_not)
    {
        std::cout << "Do not replay, Done\n";
        return 1;
    }

    //Data IO
    DataIo<Point_T> dataio;
    MapViewer<Point_T> mviewer(0.0, 1, 0, 0, 0, 255.0, 3); //downsampling ratio
    boost::shared_ptr<pcl::visualization::PCLVisualizer> reg_viewer = boost::shared_ptr<pcl::visualization::PCLVisualizer>(new pcl::visualization::PCLVisualizer("Registration Viewer"));
    boost::shared_ptr<pcl::visualization::PCLVisualizer> map_viewer = boost::shared_ptr<pcl::visualization::PCLVisualizer>(new pcl::visualization::PCLVisualizer("Map Viewer"));
    mviewer.set_interactive_events(reg_viewer, screen_width, screen_height / 2);
    mviewer.set_interactive_events(map_viewer, screen_width, screen_height / 2);

    std::vector<std::string> filenames;
    dataio.batch_read_filenames_in_folder(pointcloud_file_folder, "_filelist.txt", pc_format, filenames, frame_begin, frame_end, frame_step);
    Matrix4ds poses_gt_lidar_cs; //gt pose in lidar coordinate system
    Matrix4ds poses_lo_lidar_cs; //lo pose in lidar coordinate system

    poses_gt_lidar_cs = dataio.load_poses_from_transform_matrix(gt_pose_lidar_cs_file);
    poses_lo_lidar_cs = dataio.load_poses_from_transform_matrix(lo_pose_lidar_cs_file);

    std::vector<std::vector<unsigned int>> problematic_frame_ids;
    dataio.read_problematic_frames(evaluation_file, problematic_frame_ids);

    int frame_num = filenames.size();
    std::cout << "Import [" << frame_num << "] frames\n";

    cloudblock_Ptrs frames_used;
    frames_used.resize(frame_num);
    for (int i = 0; i < frame_num; i++)
    {
        frames_used[i] = cloudblock_Ptr(new cloudblock_t());
        frames_used[i]->filename = filenames[i];
        frames_used[i]->pose_lo = poses_lo_lidar_cs[i];
        frames_used[i]->pose_gt = poses_gt_lidar_cs[i];
        frames_used[i]->unique_id = i * frame_step + frame_begin;
    }

    //display settings
    int dense_map_keep_frame_num = 150;
    int downsamping_rate_scan_vis = 5;
    int display_refresh_time_ms = 25;

    //begin from first frame
    dataio.read_pc_cloud_block(frames_used[0]);
    mviewer.highlight_problematic_frame(map_viewer);
    mviewer.display_lo_realtime(frames_used[0], map_viewer, display_refresh_time_ms, downsamping_rate_scan_vis, downsamping_rate_map_vis);
    mviewer.is_seed_origin_ = false;

    bool horizontal_flag, vertical_flag, heading_flag;

    for (unsigned i = 1; i < frame_num; i++)
    {
        std::chrono::steady_clock::time_point tic = std::chrono::steady_clock::now();

        dataio.read_pc_cloud_block(frames_used[i]);

        check_frame(problematic_frame_ids, i, horizontal_flag, vertical_flag, heading_flag);
        mviewer.highlight_problematic_frame(map_viewer, horizontal_flag, vertical_flag, heading_flag, true);

        int frame_id_to_jump = -1;
        if (mviewer.jump_to_frame(frame_id_to_jump)) //judge whether to jump to a specific frame
        {
            if (frame_id_to_jump < frame_num && frame_id_to_jump >= 0)
            {
                frames_used[i - 1]->free_all(); //free memory
                frames_used[i]->free_all();     //free memory
                i = frame_id_to_jump;
                dataio.read_pc_cloud_block(frames_used[i]);
                continue;
            }
            else
            {
                std::cout << "Not an appropriate frame id\n";
                mviewer.set_pause();
            }
        }

        pcTPtr pointCloudS_reg(new pcT());
        Eigen::Matrix4d Tran_12;
        Tran_12 = frames_used[i - 1]->pose_lo.inverse() * frames_used[i]->pose_lo;
        pcl::transformPointCloud(*frames_used[i]->pc_raw, *pointCloudS_reg, Tran_12);
        pcl::transformPointCloud(*frames_used[i]->pc_raw, *frames_used[i]->pc_raw_w, frames_used[i]->pose_lo);

        mviewer.display_2_pc_compare_realtime(frames_used[i]->pc_raw, frames_used[i - 1]->pc_raw,
                                              pointCloudS_reg, frames_used[i - 1]->pc_raw, reg_viewer, display_refresh_time_ms);

        mviewer.display_lo_realtime(frames_used[i], map_viewer, display_refresh_time_ms, downsamping_rate_scan_vis, downsamping_rate_map_vis);
        mviewer.display_dense_map_realtime(frames_used[i], map_viewer, dense_map_keep_frame_num, display_refresh_time_ms);

        pcT().swap(*pointCloudS_reg);

        frames_used[i - 1]->free_all();

        std::chrono::steady_clock::time_point toc = std::chrono::steady_clock::now();
        std::chrono::duration<double> time_used_per_frame = std::chrono::duration_cast<std::chrono::duration<double>>(toc - tic);
        std::cout << "Render frame [" << i << "] in " << 1000.0 * time_used_per_frame.count() << " ms.\n";
    }

    mviewer.keep_visualize(map_viewer);

    return 1;
}