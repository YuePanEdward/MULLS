//Copyright 2020 The Hesai Technology Authors. All Rights Reserved.

#ifndef _INCLUDE_H5_IO_HPP_
#define _INCLUDE_H5_IO_HPP_

#include "math.h"
#include <time.h>
#include <algorithm>
#include <numeric>
#include <queue>
#include <string>
#include <iostream>

#include <H5Cpp.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <boost/shared_ptr.hpp>

//Used for HESAI LiDAR's *.h5 format point cloud
#define HS_PIC_ROWS_LENGTH_P64 (64)
#define HS_PIC_COLS_LENGTH_P64 (1800)
#define HS_PIC_LENGTH_SINGLE_COL (1000000)

typedef struct
{
    int raw_mask[HS_PIC_ROWS_LENGTH_P64][HS_PIC_COLS_LENGTH_P64];
    int label_mask[HS_PIC_ROWS_LENGTH_P64][HS_PIC_COLS_LENGTH_P64] = {{0}};
    int ground_mask[HS_PIC_ROWS_LENGTH_P64][HS_PIC_COLS_LENGTH_P64] = {{0}};
    float x[HS_PIC_ROWS_LENGTH_P64][HS_PIC_COLS_LENGTH_P64];
    float y[HS_PIC_ROWS_LENGTH_P64][HS_PIC_COLS_LENGTH_P64];
    float z[HS_PIC_ROWS_LENGTH_P64][HS_PIC_COLS_LENGTH_P64];
    float intensity[HS_PIC_ROWS_LENGTH_P64][HS_PIC_COLS_LENGTH_P64];
    double timestamp[HS_PIC_ROWS_LENGTH_P64][HS_PIC_COLS_LENGTH_P64];
    float dist[HS_PIC_ROWS_LENGTH_P64][HS_PIC_COLS_LENGTH_P64];
    int num_cols[HS_PIC_COLS_LENGTH_P64] = {0};
    int num_rows[HS_PIC_ROWS_LENGTH_P64] = {0};
    int num_feature_points = 0;
} P64Pic;

typedef struct
{
    int raw_mask[HS_PIC_LENGTH_SINGLE_COL];
    int label_mask[HS_PIC_LENGTH_SINGLE_COL] = {0};
    int ground_mask[HS_PIC_LENGTH_SINGLE_COL] = {0};
    float x[HS_PIC_LENGTH_SINGLE_COL];
    float y[HS_PIC_LENGTH_SINGLE_COL];
    float z[HS_PIC_LENGTH_SINGLE_COL];
    float intensity[HS_PIC_LENGTH_SINGLE_COL];
    double timestamp[HS_PIC_LENGTH_SINGLE_COL];
    float dist[HS_PIC_LENGTH_SINGLE_COL];
    int num_feature_points = 0;
} SINGLECOLPic;

template <typename PointT>
class ReadFromH5
{

public:
    bool loadH5File_P64(std::string file_path, typename pcl::PointCloud<PointT>::Ptr &cloud)
    {
        boost::shared_ptr<P64Pic> pic_ptr(new P64Pic());
        H5::H5File *file = new H5::H5File(file_path, H5F_ACC_RDONLY);
        loadAttrsinH5_P64(file, *(pic_ptr->x), "x");
        loadAttrsinH5_P64(file, *(pic_ptr->y), "y");
        loadAttrsinH5_P64(file, *(pic_ptr->z), "z");
        // loadAttrsinH5_P64(file, *(pic_ptr->intensity), "i");
        // loadAttrsinH5_P64(file, *(pic_ptr->timestamp), "ts");

        file->close();
        fillinPointCloud_P64(cloud, pic_ptr);

        pic_ptr.reset(new P64Pic());

        return true;
    }

    bool loadH5File_SINGLE_COL(std::string file_path, typename pcl::PointCloud<PointT>::Ptr &cloud)
    {
        boost::shared_ptr<SINGLECOLPic> pic_ptr(new SINGLECOLPic());
        H5::H5File *file = new H5::H5File(file_path, H5F_ACC_RDONLY);
        //one-dimension array, its own name can also represents the first element's pointer
        //two-dimension array, its own name can also represents the first array's pointer, then you need another * to get such array, which is also the pointer to the first element of such array
        int point_number = loadAttrsinH5_SINGLE_COL_FLT(file, pic_ptr->x, "x");
        loadAttrsinH5_SINGLE_COL_FLT(file, pic_ptr->y, "y");
        loadAttrsinH5_SINGLE_COL_FLT(file, pic_ptr->z, "z");
        loadAttrsinH5_SINGLE_COL_FLT(file, pic_ptr->intensity, "intensity");
        loadAttrsinH5_SINGLE_COL_DBL(file, pic_ptr->timestamp, "ts");

        file->close();
        fillinPointCloud_SINGLE_COL(cloud, pic_ptr, point_number);

        pic_ptr.reset(new SINGLECOLPic());

        return true;
    }

private:
    void fillinPointCloud_P64(typename pcl::PointCloud<PointT>::Ptr cloud, boost::shared_ptr<P64Pic> pic_ptr)
    {
        for (int i = 0; i < HS_PIC_ROWS_LENGTH_P64; i++)
        {
            for (int j = 0; j < HS_PIC_COLS_LENGTH_P64; j++)
            {
                PointT point;
                point.x = pic_ptr->x[i][j];
                point.y = pic_ptr->y[i][j];
                point.z = pic_ptr->z[i][j];
                // point.intensity = pic_ptr->intensity[i][j];
                // point.curvature = pic_ptr->timestamp[i][j];
                cloud->points.push_back(point);
            }
        }
    }

    void fillinPointCloud_SINGLE_COL(typename pcl::PointCloud<PointT>::Ptr cloud, boost::shared_ptr<SINGLECOLPic> pic_ptr, int point_num,
                                 float range_min = 0.3, float range_max = 200.0)
    {
        for (int i = 0; i < point_num; i++)
        {
            PointT point;

            point.x = pic_ptr->x[i];
            point.y = pic_ptr->y[i];
            point.z = pic_ptr->z[i];
            point.intensity = pic_ptr->intensity[i];
            point.curvature = std::fmod(1000.0 * pic_ptr->timestamp[i], 1000000.0); //in ms
            // if (check_nan(point))
            //     continue;

            if ((point.x * point.x + point.y * point.y + point.z * point.z) < (range_max * range_max) &&
                (point.x * point.x + point.y * point.y + point.z * point.z) > (range_min * range_min))
                cloud->points.push_back(point);
        }
    }

    void loadAttrsinH5_P64(H5::H5File *file, float *data, std::string attr_name)
    {
        H5::DataSet attr_data = file->openDataSet(attr_name);
        hsize_t offset[2] = {0, 0};
        hsize_t dims[2] = {HS_PIC_ROWS_LENGTH_P64, HS_PIC_COLS_LENGTH_P64};
        hsize_t block[2] = {1, 1};
        hsize_t stride[2] = {1, 1};

        H5::DataSpace mspace(2, dims);
        H5::DataSpace dataSpace = attr_data.getSpace();
        dataSpace.selectHyperslab(H5S_SELECT_SET, dims, offset, stride, block);
        attr_data.read(data, H5::PredType::NATIVE_FLOAT, mspace, dataSpace);
    }

    int loadAttrsinH5_SINGLE_COL_FLT(H5::H5File *file, float *data, std::string attr_name)
    {
        H5::DataSet attr_data = file->openDataSet(attr_name);
        H5::DataSpace dataSpace = attr_data.getSpace();

        hsize_t dims[1];
        int ndims = dataSpace.getSimpleExtentDims(dims, NULL);

        hsize_t offset[1] = {0};
        hsize_t block[1] = {1};
        hsize_t stride[1] = {1};

        H5::DataSpace mspace(1, dims);
        dataSpace.selectHyperslab(H5S_SELECT_SET, dims, offset, stride, block);
        attr_data.read(data, H5::PredType::NATIVE_FLOAT, mspace, dataSpace);

        return dims[0]; //return the row number
    }

    int loadAttrsinH5_SINGLE_COL_DBL(H5::H5File *file, double *data, std::string attr_name)
    {
        H5::DataSet attr_data = file->openDataSet(attr_name);
        H5::DataSpace dataSpace = attr_data.getSpace();

        hsize_t dims[1];
        int ndims = dataSpace.getSimpleExtentDims(dims, NULL);

        hsize_t offset[1] = {0};
        hsize_t block[1] = {1};
        hsize_t stride[1] = {1};

        H5::DataSpace mspace(1, dims);
        dataSpace.selectHyperslab(H5S_SELECT_SET, dims, offset, stride, block);
        attr_data.read(data, H5::PredType::NATIVE_DOUBLE, mspace, dataSpace);

        return dims[0]; //return the row number
    }

    //return true if the point contains a nan
    bool check_nan(PointT point)
    {
        if (std::isnan(point.x) || std::isnan(point.y) || std::isnan(point.z) || std::isnan(point.intensity) || std::isnan(point.curvature))
            return true;
        else
            return false;
    }

private:
};

#endif //_INCLUDE_H5_IO_HPP_
