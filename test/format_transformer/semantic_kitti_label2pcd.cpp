
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <math.h>
#include <iostream>
#include <vector>

//pcl
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/segmentation/extract_clusters.h>
// workaround for PCL 1.7 https://github.com/PointCloudLibrary/pcl/issues/2406
#include <pcl/search/impl/search.hpp>

#include "semantic_kitti_api.h"

typedef pcl::PointXYZINormal Point_T;
typedef pcl::PointCloud<Point_T>::Ptr pcTPtr;
typedef pcl::PointCloud<Point_T> pcT;

bool write_pcd_file(const std::string &fileName, pcTPtr &pointCloud, bool as_binary = true)
{
    if (as_binary)
    {
        if (pcl::io::savePCDFileBinary(fileName, *pointCloud) == -1)
        {
            PCL_ERROR("Couldn't write file\n");
            return false;
        }
    }
    else
    {
        if (pcl::io::savePCDFile(fileName, *pointCloud) == -1)
        {
            PCL_ERROR("Couldn't write file\n");
            return false;
        }
    }
    std::cout << "Write to [" << fileName << "] done\n";
    return true;
}

void check_normal(pcTPtr &normals)
{
    for (int i = 0; i < normals->points.size(); i++)
    {
        if (!pcl::isFinite<Point_T>(normals->points[i]))
        {
            normals->points[i].normal_x = 0.577; // 1/ sqrt(3)
            normals->points[i].normal_y = 0.577;
            normals->points[i].normal_z = 0.577;
        }
    }
}

bool get_pc_semantic_normal(pcTPtr &cloud,
                            int K)
{
    // Create the normal estimation class, and pass the input dataset to it;
    pcl::NormalEstimationOMP<Point_T, Point_T> ne;
    ne.setNumberOfThreads(omp_get_max_threads()); //More threads sometimes would not speed up the procedure
    ne.setInputCloud(cloud);
    // Create an empty kd-tree representation, and pass it to the normal estimation object;
    typename pcl::search::KdTree<Point_T>::Ptr tree(new pcl::search::KdTree<Point_T>());
    ne.setSearchMethod(tree);
    // Use all neighbors in a sphere of radius;
    ne.setKSearch(K);
    // Compute the normal
    ne.compute(*cloud);
    check_normal(cloud);
    std::cout << "normal estimation done.\n";
    //std::cout << "example: point 10 (" << cloud->points[10].normal_x << "," << cloud->points[10].normal_y << "," << cloud->points[10].normal_z << ").\n";
    return true;
}

int main(int argc, char **argv)
{
    if (argv[1] == NULL)
    {
        fprintf(stderr, "argv[1] must be path to dataset sequence -> /path/to/sequence/00/\n");
        exit(1);
    }
    if (argv[2] == NULL)
    {
        fprintf(stderr, "argv[2] must be output directory -> /path/to/output/directory/\n");
        exit(1);
    }

    std::string datasetDirName = argv[1];
    std::string outputDirName = argv[2];
    skd::SKDAPI skdapi(datasetDirName);

    int timeIdx = 0;

    pcTPtr frame_pc(new pcT());
    std::vector<skd::VelodynePoint> velodynePoints;
    std::string frame_filename;

    for (;;)
    {
        velodynePoints = skdapi.getVelodynePoints(timeIdx);
        if ((int)velodynePoints.size() == 0)
            break;

        for (int i = 0; i < (int)velodynePoints.size(); i++)
        {
            Point_T pt;
            pt.x = velodynePoints[i].x_;
            pt.y = velodynePoints[i].y_;
            pt.z = velodynePoints[i].z_;
            pt.intensity = velodynePoints[i].intensity_ * 255;
            frame_pc->points.push_back(pt);
        }
        get_pc_semantic_normal(frame_pc, 30);

        for (int i = 0; i < frame_pc->points.size(); i++)
            frame_pc->points[i].curvature = velodynePoints[i].label_; //use curvature to store the label since curvature would not be used

        std::ostringstream oss;
        oss.setf(std::ios::right);
        oss.fill('0');
        oss.width(6);
        oss << timeIdx;

        frame_filename = outputDirName + oss.str() + ".pcd";
        write_pcd_file(frame_filename, frame_pc);

        std::vector<skd::VelodynePoint>().swap(velodynePoints);
        pcT().swap(*frame_pc);

        timeIdx++;
    }

    return 1;
}