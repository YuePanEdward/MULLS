//
// This file is for all kinds of data I/O
// Dependent 3rd Libs: PCL (>1.7), LibLas (optional for *LAS IO)
// By Yue Pan
//

#ifndef _INCLUDE_DATA_IO_HPP
#define _INCLUDE_DATA_IO_HPP

//pcl
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/visualization/common/common.h>
#include <pcl/visualization/pcl_visualizer.h>

//boost
#include <boost/filesystem.hpp>
#include <boost/function.hpp>

#if LIBLAS_ON
//liblas (optional for *.las io)
#include <liblas/liblas.hpp>
#include <liblas/version.hpp>
#include <liblas/point.hpp>
#endif

// (optional for *.h5 io)
#if HDF5_ON
#include "h5_io.hpp"
#endif

#include <glog/logging.h>
#include <string>
#include <fstream>
#include <vector>

#include "utility.hpp"

using namespace boost::filesystem;
using namespace std;

namespace lo
{
inline std::ostream &operator<<(std::ostream &output, const Eigen::Matrix4d &mat)
{
    output << setprecision(8);

    for (int i = 0; i < 4; i++)
    {
        output << mat(i, 0) << "\t" << mat(i, 1) << "\t" << mat(i, 2) << "\t" << mat(i, 3) << "\n";
    }
    return output;
}

inline std::ostream &operator<<(std::ostream &output, const Matrix6d &mat)
{
    output << setprecision(8);

    for (int i = 0; i < 6; i++)
    {
        output << mat(i, 0) << "\t" << mat(i, 1) << "\t" << mat(i, 2) << "\t"
               << mat(i, 3) << "\t" << mat(i, 4) << "\t" << mat(i, 5) << "\n";
    }
    return output;
}

inline std::istream &operator>>(std::istream &input, pose_qua_t &pose)
{
    input >> pose.trans.x() >> pose.trans.y() >> pose.trans.z() >> pose.quat.x() >>
        pose.quat.y() >> pose.quat.z() >> pose.quat.w();
    // Normalize the quaternion to account for precision loss due to serialization.
    pose.quat.normalize();
    return input;
}

inline std::ostream &operator<<(std::ostream &output, const pose_qua_t &pose)
{
    output << pose.trans.x() << "\t" << pose.trans.y() << "\t" << pose.trans.z() << "\t"
           << pose.quat.x() << "\t" << pose.quat.y() << "\t" << pose.quat.z() << "\t" << pose.quat.w() << "\t";
    return output;
}

inline std::ostream &operator<<(std::ostream &output, const bounds_t &bbx)
{
    output << setprecision(8);
    output << bbx.min_x << "\t" << bbx.min_y << "\t" << bbx.min_z << "\t" << bbx.max_x << "\t" << bbx.max_y << "\t" << bbx.max_z << "\n";
    return output;
}

inline std::ostream &operator<<(std::ostream &output, const centerpoint_t &cp)
{
    output << setprecision(8);
    output << cp.x << "\t" << cp.y << "\t" << cp.z << "\n";
    return output;
}

inline std::ostream &operator<<(std::ostream &output, const constraint_t &con)
{
    output << con.unique_id << "\t" << con.con_type << "\t"
           << con.block1->unique_id << "\t" << con.block1->data_type << "\t"
           << con.block2->unique_id << "\t" << con.block2->data_type << "\n";

    output << con.Trans1_2;

    output << con.information_matrix;

    return output;
}

template <typename PointT>
class DataIo : public CloudUtility<PointT>
{
  public:
    bool check_dir(const std::string &dir)
    {
        if (!boost::filesystem::exists(dir.c_str()))
        {
            if (boost::filesystem::create_directory(dir.c_str()))
                return true;
            else
                return false;
        }
        return true;
    }
#if 0
    inline bool exists_file(const std::string &filename)
    {
        struct stat buffer;
        return (stat(filename.c_str(), &buffer) == 0);
    }
#endif
    inline bool exists_file(const std::string &filename)
    {
        std::ifstream f(filename.c_str());
        return f.good();
    }

    // Delete invalid characters such as spaces, tabs, etc. in strings
    std::string trim_str(std::string &str)
    {
        str.erase(0, str.find_first_not_of(" \t\r\n"));
        str.erase(str.find_last_not_of(" \t\r\n") + 1);
        return str;
    }

    //Brief: Read the point cloud data of various format
    bool read_cloud_file(const std::string &fileName, typename pcl::PointCloud<PointT>::Ptr &pointCloud)
    {
        std::chrono::steady_clock::time_point tic = std::chrono::steady_clock::now();

        std::string extension;
        extension = fileName.substr(fileName.find_last_of('.') + 1); //Get the suffix of the file;

        if (!strcmp(extension.c_str(), "pcd"))
        {
            read_pcd_file(fileName, pointCloud);
            LOG(INFO) << "A pcd file has been imported.";
            //std::cout << "A pcd file has been imported" << std::endl;
        }
#if LIBLAS_ON
        else if (!strcmp(extension.c_str(), "las"))
        {
            bool global_shift_or_not = 1;
            std::cout << "Would you like to do a global shift ?  0. No  1. Yes [default 1]" << std::endl;
            std::cin >> global_shift_or_not;
            if (!global_shift_or_not)
            {
                read_las_file(fileName, pointCloud);
            }
            else
            {
                bool use_automatic_drift = 0;
                std::cout << "Using the automatic shift or enter the global shift yourself ? " << std::endl
                          << "0. Read a global shift file  1.Use the automatic shift [default 0]" << std::endl;
                std::cin >> use_automatic_drift;
                read_las_file(fileName, pointCloud, use_automatic_drift);
            }
            LOG(INFO) << "A las file has been imported";
        }
#endif
        else if (!strcmp(extension.c_str(), "ply"))
        {
            read_ply_file(fileName, pointCloud);
            LOG(INFO) << "A ply file has been imported.";
        }
        else if (!strcmp(extension.c_str(), "txt"))
        {
            read_txt_file(fileName, pointCloud);
            LOG(INFO) << "A txt file has been imported.";
        }
#if HDF5_ON
        else if (!strcmp(extension.c_str(), "h5"))
        {
            read_h5_file(fileName, pointCloud);
            LOG(INFO) << "A h5 file has been imported.";
        }
#endif
        else if (!strcmp(extension.c_str(), "csv"))
        {
            read_csv_file(fileName, pointCloud);
            LOG(INFO) << "A csv file has been imported.";
        }
        else
        {
            LOG(INFO) << "Undefined Point Cloud Format.";
            return 0;
        }

        std::chrono::steady_clock::time_point toc = std::chrono::steady_clock::now();
        std::chrono::duration<double> time_used = std::chrono::duration_cast<std::chrono::duration<double>>(toc - tic);

        LOG(INFO) << "[" << pointCloud->points.size() << "] points loaded in [" << time_used.count() * 1000 << "] ms";
        //std::cout << "[" << pointCloud->points.size() << "] points loaded in [" << time_used.count() << "] s" << std::endl;

        return 1;
    }

    bool write_cloud_file(const std::string &fileName, const typename pcl::PointCloud<PointT>::Ptr &pointCloud)
    {
        std::chrono::steady_clock::time_point tic = std::chrono::steady_clock::now();

        std::string extension;
        extension = fileName.substr(fileName.find_last_of('.') + 1); //Get the suffix of the file;

        if (!strcmp(extension.c_str(), "pcd"))
        {
            write_pcd_file(fileName, pointCloud); //write out ascii format so that CC can load the file
            LOG(INFO) << "A pcd file has been exported" << std::endl;
        }
#if LIBLAS_ON
        else if (!strcmp(extension.c_str(), "las"))
        {
            bool global_shift_or_not = 0;
            std::cout << "Would you like to do a global shift ?  0. No  1. Yes [default 0]" << std::endl;
            std::cin >> global_shift_or_not;
            if (!global_shift_or_not)
            {
                write_las_file(fileName, pointCloud);
            }
            else
            {
                bool use_automatic_drift = 0;
                std::cout << "Using the automatic shift or enter the global shift yourself ? " << std::endl
                          << "0. Read a global shift file  1.Use the automatic shift [default 0]" << std::endl;
                std::cin >> use_automatic_drift;
                write_las_file(fileName, pointCloud, use_automatic_drift);
            }
            std::cout << "A las file has been exported" << std::endl;
        }
#endif
        else if (!strcmp(extension.c_str(), "ply"))
        {
            write_ply_file(fileName, pointCloud);
            LOG(INFO) << "A ply file has been exported" << std::endl;
        }
        else if (!strcmp(extension.c_str(), "txt"))
        {
            write_txt_file(fileName, pointCloud);
            LOG(INFO) << "A txt file has been exported" << std::endl;
        }
        else
        {
            LOG(INFO) << "Undefined Point Cloud Format." << std::endl;
            return 0;
        }

        std::chrono::steady_clock::time_point toc = std::chrono::steady_clock::now();
        std::chrono::duration<double> time_used = std::chrono::duration_cast<std::chrono::duration<double>>(toc - tic);

        LOG(INFO) << "[" << pointCloud->points.size() << "] points exported in [" << time_used.count() * 1000 << "] ms";
    }

    bool read_pcd_file(const std::string &fileName, typename pcl::PointCloud<PointT>::Ptr &pointCloud)
    {
        if (pcl::io::loadPCDFile<PointT>(fileName, *pointCloud) == -1)
        {
            PCL_ERROR("Couldn't read file\n");
            return false;
        }
        return true;
    }

    bool write_pcd_file(const std::string &fileName, const typename pcl::PointCloud<PointT>::Ptr &pointCloud, bool as_binary = true)
    {
        //do the reshaping
        pointCloud->width = 1;
        pointCloud->height = pointCloud->points.size();

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
        return true;
    }

    bool read_csv_file(const std::string &file_name, typename pcl::PointCloud<PointT>::Ptr &cloud)
    {
        std::ifstream fin(file_name); // Open the file

        std::string line;

        getline(fin, line); //head line

        double x_p = 0, y_p = 0, z_p = 0, i_p = 0, t_p = 0;
        int i = 0;

        while (getline(fin, line)) // The entire line is read, the newline character "\n" is distinguished
        {
            std::istringstream sline(line); // Read the entire line string into the string stream
            std::vector<std::string> point_datastrings;
            std::string point_datastring;

            while (getline(sline, point_datastring, ',')) // Use the ',' as the dividing marker, get
            {
                point_datastrings.push_back(point_datastring); // Add those isolated string into the vector
            }
            //order in mimap dataset:
            //Points:x","Points:y","Points:z","intensity","laser_id","azimuth","distance_m","timestamp"
            x_p = std::atof(trim_str(point_datastrings[0]).c_str()); //x
            y_p = std::atof(trim_str(point_datastrings[1]).c_str()); //y
            z_p = std::atof(trim_str(point_datastrings[2]).c_str()); //z
            i_p = std::atof(trim_str(point_datastrings[3]).c_str()); //intensity
            t_p = std::atof(trim_str(point_datastrings[7]).c_str()); //timestamp (pointwise)

            PointT pt;
            pt.x = x_p;
            pt.y = y_p;
            pt.z = z_p;
            pt.intensity = i_p;
            pt.curvature = t_p * 0.001; //curvature as timestamp (unit: ms)
            cloud->points.push_back(pt);

            //LOG(INFO)<<"Point："<< i << "\t" << x_p << "\t" << y_p <<  "\t" << z_p << "\t" << i_p<< std::endl;
            i++;
        }
        fin.close();
    }

#if HDF5_ON
    bool read_h5_file(const std::string &file_name, typename pcl::PointCloud<PointT>::Ptr &cloud)
    {
        ReadFromH5<PointT> h5_reader;
        if (!h5_reader.loadH5File_SINGLE_COL(file_name, cloud))
        {
            std::cout << "Read H5 file failed\n";
            return false;
        }
        return true;
    }
#endif

#if LIBLAS_ON
    bool read_las_file_head(const std::string &fileName, liblas::Header &header)
    {
        if (fileName.substr(fileName.rfind('.')).compare(".las"))
        {
            return 0;
        }
        else
        {
            std::ifstream ifs;
            ifs.open(fileName.c_str(), std::ios::in | std::ios::binary);
            if (ifs.bad())
            {
                return 0;
            }

            liblas::ReaderFactory f;
            liblas::Reader reader = f.CreateWithStream(ifs);

            header = reader.GetHeader();
        }
        return 1;
    }

    bool read_las_file(const std::string &fileName, typename pcl::PointCloud<PointT>::Ptr &pointCloud) //Without translation
    {
        //cout << "A global translation or gravitization should be done to keep the precision of point cloud when adopting pcl to do las file point cloud processing" << endl;

        if (fileName.substr(fileName.rfind('.')).compare(".las"))
        {
            return 0;
        }

        std::ifstream ifs;
        ifs.open(fileName.c_str(), std::ios::in | std::ios::binary);
        if (ifs.bad())
        {
            std::cout << "Matched Terms are not found." << std::endl;
        }
        liblas::ReaderFactory f;
        liblas::Reader reader = f.CreateWithStream(ifs);
        liblas::Header const &header = reader.GetHeader();

        //Bounding box Information
        double Xmin, Ymin, Zmin, Xmax, Ymax, Zmax;
        Xmin = header.GetMinX();
        Ymin = header.GetMinY();
        Zmin = header.GetMinZ();
        Xmax = header.GetMaxX();
        Ymax = header.GetMaxY();
        Zmax = header.GetMaxZ();

        while (reader.ReadNextPoint())
        {
            const liblas::Point &p = reader.GetPoint();
            PointT pt;
            pt.x = p.GetX();
            pt.y = p.GetY();
            pt.z = p.GetZ();

            //------------------------------------------------Assign Intensity--------------------------------------------------//
            //If the Point template PointT has intensity, you can assign the intensity with any feature of the point cloud in las.
            // bool intensity_available = pcl::traits::has_field<PointT, pcl::fields::intensity>::value;
            // if (intensity_available)
            // {
            //     pt.intensity = p.GetIntensity();
            // }

            //pt.intensity = p.GetIntensity();
            //pt.intensity = p.GetTime();
            //pt.intensity = p.GetScanAngleRank();
            //pt.intensity = p.GetNumberOfReturns();
            //pt.intensity = p.GetScanDirection();

            //---------------------------------------------------Assign Color--------------------------------------------------//
            //If the Point template PointT has RGB, you can assign the Color according to the point cloud in las.
            //If the Point template PointT is without RGB, you should comment the line.
            //liblas::Color lasColor;
            //lasColor= p.GetColor();
            //pt.r = lasColor.GetRed();
            //pt.g = lasColor.GetGreen();
            //pt.b = lasColor.GetBlue();

            pointCloud->points.push_back(pt);
        }
        ifs.close();
        return 1;
    }

    bool write_las_file(const std::string &fileName, const typename pcl::PointCloud<PointT>::Ptr &pointCloud) //Without translation
    {

        bounds_t bound;
        this->get_cloud_bbx(pointCloud, bound);

        std::ofstream ofs;
        ofs.open(fileName.c_str(), std::ios::out | std::ios::binary);
        if (ofs.is_open())
        {
            liblas::Header header;
            header.SetDataFormatId(liblas::ePointFormat2);
            header.SetVersionMajor(1);
            header.SetVersionMinor(2);
            header.SetMin(bound.min_x, bound.min_y, bound.min_z);
            header.SetMax(bound.max_x, bound.max_y, bound.max_z);
            header.SetOffset((bound.min_x + bound.max_x) / 2.0, (bound.min_y + bound.max_y) / 2.0, (bound.min_z + bound.max_z) / 2.0);
            header.SetScale(0.01, 0.01, 0.01);
            header.SetPointRecordsCount(pointCloud->points.size());

            liblas::Writer writer(ofs, header);
            liblas::Point pt(&header);

            for (int i = 0; i < pointCloud->points.size(); i++)
            {
                pt.SetCoordinates(double(pointCloud->points[i].x), double(pointCloud->points[i].y), double(pointCloud->points[i].z));

                // bool intensity_available = pcl::traits::has_field<PointT, pcl::fields::intensity>::value;
                // if (intensity_available)
                // {
                //     pt.SetIntensity(pointCloud->points[i].intensity);
                // }

                //If the Point template PointT is without RGB, you should comment the line.
                //liblas::Color lasColor;
                //lasColor.SetRed(pointCloud->points[i].r);
                //lasColor.SetGreen(pointCloud->points[i].g);
                //lasColor.SetBlue(pointCloud->points[i].b);
                //pt.SetColor(lasColor);

                writer.WritePoint(pt);
            }
            ofs.flush();
            ofs.close();
        }
        return 1;
    }

    bool read_las_file(const std::string &fileName, typename pcl::PointCloud<PointT>::Ptr &pointCloud, bool automatic_shift_or_not) //With translation
    {
        global_shift.resize(3);
        //std::cout << "A global translation or gravitization should be done to keep the precision of point cloud when adopting pcl to do las file point cloud processing" << endl;

        if (fileName.substr(fileName.rfind('.')).compare(".las"))
        {
            return 0;
        }

        std::ifstream ifs;
        ifs.open(fileName.c_str(), std::ios::in | std::ios::binary);
        if (ifs.bad())
        {
            std::cout << "Matched Terms are not found." << std::endl;
        }
        liblas::ReaderFactory f;
        liblas::Reader reader = f.CreateWithStream(ifs);
        liblas::Header const &header = reader.GetHeader();

        //Bounding box Information
        double Xmin, Ymin, Zmin, Xmax, Ymax, Zmax;
        Xmin = header.GetMinX();
        Ymin = header.GetMinY();
        Zmin = header.GetMinZ();
        Xmax = header.GetMaxX();
        Ymax = header.GetMaxY();
        Zmax = header.GetMaxZ();

        if (automatic_shift_or_not)
        {
            // Automatic Gloabl Shift Value;
            global_shift[0] = -0.5 * (Xmin + Xmax);
            global_shift[1] = -0.5 * (Ymin + Ymax);
            global_shift[2] = -0.5 * (Zmin + Zmax);

            std::ofstream out("GlobalShift.txt", std::ios::out);
            out << setiosflags(std::ios::fixed) << std::setprecision(8) << global_shift[0] << std::endl;
            out << setiosflags(std::ios::fixed) << std::setprecision(8) << global_shift[1] << std::endl;
            out << setiosflags(std::ios::fixed) << std::setprecision(8) << global_shift[2] << std::endl;
            out.close();

            std::cout << "A txt File named GlobalShift.txt is saved in current Folder" << std::endl;
        }
        else
        {
            std::string fileGlobalShift;
            std::cout << "Enter 1 to read the GlobalShift.txt file in project folder." << std::endl
                      << "Or please enter or drag in the Global Shift File" << std::endl
                      << "Example [GlobalShift.txt] :" << std::endl
                      << "-366370.90" << std::endl
                      << "-3451297.82" << std::endl
                      << "-14.29" << std::endl;

            std::cin >> fileGlobalShift;
            if (strcmp(fileGlobalShift.c_str(), "1"))
                fileGlobalShift = "./GlobalShift.txt";

            std::ifstream in(fileGlobalShift.c_str(), std::ios::in);
            in >> global_shift[0];
            in >> global_shift[1];
            in >> global_shift[2];
            in.close();
        }

        while (reader.ReadNextPoint())
        {
            const liblas::Point &p = reader.GetPoint();
            PointT pt;

            //A translation to keep the precision
            pt.x = p.GetX() + global_shift[0];
            pt.y = p.GetY() + global_shift[1];
            pt.z = p.GetZ() + global_shift[2];

            //------------------------------------------------Assign Intensity--------------------------------------------------//
            //If the Point template PointT has intensity, you can assign the intensity with any feature of the point cloud in las.
            //If the Point template PointT is without intensity, you should comment the line.
            // bool intensity_available = pcl::traits::has_field<PointT, pcl::fields::intensity>::value;
            // if (intensity_available)
            // {
            //     pt.intensity = p.GetIntensity();
            // }

            //pt.intensity = p.GetTime();
            //pt.intensity = p.GetScanAngleRank();
            //pt.intensity = p.GetNumberOfReturns();
            //pt.intensity = p.GetScanDirection();

            //---------------------------------------------------Assign Color--------------------------------------------------//
            //If the Point template PointT has RGB, you can assign the Color according to the point cloud in las.
            //If the Point template PointT is without RGB, you should comment the line.
            //liblas::Color lasColor;
            //lasColor= p.GetColor();
            //pt.r = lasColor.GetRed();
            //pt.g = lasColor.GetGreen();
            //pt.b = lasColor.GetBlue();

            pointCloud->points.push_back(pt);
        }

        ifs.close();

        return 1;
    }

    bool write_las_file(const std::string &fileName, const typename pcl::PointCloud<PointT>::Ptr &pointCloud, bool automatic_shift_or_not) //With translation
    {
        global_shift.resize(3);

        bounds_t bound;
        this->get_cloud_bbx(pointCloud, bound);

        if (!automatic_shift_or_not)
        {
            std::cout << "Use default (last input) global shift or not" << std::endl
                      << "1. Yes  0. No" << std::endl;
            bool use_last_shift;
            std::cin >> use_last_shift;
            if (!use_last_shift)
            {
                std::string fileGlobalShift;
                std::cout << "Please enter or drag in the Global Shift File" << std::endl
                          << "Example [GlobalShift.txt] :" << std::endl
                          << "-366370.90" << std::endl
                          << "-3451297.82" << std::endl
                          << "-14.29" << std::endl;

                std::cin >> fileGlobalShift;

                std::ifstream in(fileGlobalShift, std::ios::in);
                in >> global_shift[0];
                in >> global_shift[1];
                in >> global_shift[2];
                in.close();
            }
        }
        else //use the already existed global shift value
        {
            ;
        }
        std::ofstream ofs;
        ofs.open(fileName.c_str(), std::ios::out | std::ios::binary);
        if (ofs.is_open())
        {
            liblas::Header header;
            header.SetDataFormatId(liblas::ePointFormat2);
            header.SetVersionMajor(1);
            header.SetVersionMinor(2);
            header.SetMin(bound.min_x - global_shift[0], bound.min_y - global_shift[1], bound.min_z - global_shift[2]);
            header.SetMax(bound.max_x - global_shift[0], bound.max_y - global_shift[1], bound.max_z - global_shift[2]);
            header.SetOffset((bound.min_x + bound.max_x) / 2.0 - global_shift[0], (bound.min_y + bound.max_y) / 2.0 - global_shift[1], (bound.min_z + bound.max_z) / 2.0 - global_shift[2]);
            header.SetScale(0.01, 0.01, 0.01);
            header.SetPointRecordsCount(pointCloud->points.size());

            liblas::Writer writer(ofs, header);
            liblas::Point pt(&header);

            for (size_t i = 0; i < pointCloud->points.size(); i++)
            {
                pt.SetCoordinates(double(pointCloud->points[i].x) - global_shift[0], double(pointCloud->points[i].y) - global_shift[1], double(pointCloud->points[i].z) - global_shift[2]);

                bool intensity_available = pcl::traits::has_field<PointT, pcl::fields::intensity>::value;

                // figure out why this cannot be used properly.
                // if (intensity_available)
                // {
                //     pt.SetIntensity(pointCloud->points[i].intensity);
                // }

                //If the Point template PointT is without RGB, you should comment the line.
                //liblas::Color lasColor;
                //lasColor.SetRed(pointCloud->points[i].r);
                //lasColor.SetGreen(pointCloud->points[i].g);
                //lasColor.SetBlue(pointCloud->points[i].b);
                //pt.SetColor(lasColor);

                writer.WritePoint(pt);
            }
            ofs.flush();
            ofs.close();
        }
        return 1;
    }

    bool read_las_file_last(const std::string &fileName, const typename pcl::PointCloud<PointT>::Ptr &pointCloud)
    {
        if (fileName.substr(fileName.rfind('.')).compare(".las"))
        {
            return 0;
        }

        std::ifstream ifs;
        ifs.open(fileName.c_str(), std::ios::in | std::ios::binary);
        if (ifs.bad())
        {
            std::cout << "Matched Terms are not found." << std::endl;
        }
        liblas::ReaderFactory f;
        liblas::Reader reader = f.CreateWithStream(ifs);
        liblas::Header const &header = reader.GetHeader();

        while (reader.ReadNextPoint())
        {
            const liblas::Point &p = reader.GetPoint();
            PointT pt;

            //A translation to keep the precision
            pt.x = p.GetX() + global_shift[0];
            pt.y = p.GetY() + global_shift[1];
            pt.z = p.GetZ() + global_shift[2];

            //------------------------------------------------Assign Intensity--------------------------------------------------//
            //If the Point template PointT has intensity, you can assign the intensity with any feature of the point cloud in las.
            // bool intensity_available = pcl::traits::has_field<PointT, pcl::fields::intensity>::value;
            // if (intensity_available)
            // {
            //     pt.intensity = p.GetIntensity();
            // }
            //pt.intensity = p.GetTime();
            //pt.intensity = p.GetScanAngleRank();
            //pt.intensity = p.GetNumberOfReturns();
            //pt.intensity = p.GetScanDirection();

            //---------------------------------------------------Assign Color--------------------------------------------------//
            //If the Point template PointT has RGB, you can assign the Color according to the point cloud in las.
            //If the Point template PointT is without RGB, you should comment the line.
            //liblas::Color lasColor;
            //lasColor= p.GetColor();
            //pt.r = lasColor.GetRed();
            //pt.g = lasColor.GetGreen();
            //pt.b = lasColor.GetBlue();

            pointCloud->points.push_back(pt);
        }

        ifs.close();
        return 1;
    }
#endif
    bool read_ply_file(const std::string &fileName, const typename pcl::PointCloud<PointT>::Ptr &pointCloud)
    {
        if (pcl::io::loadPLYFile<PointT>(fileName, *pointCloud) == -1) //* load the file
        {
            PCL_ERROR("Couldn't read file \n");
            return (-1);
        }
    }

    bool write_ply_file(const std::string &fileName, const typename pcl::PointCloud<PointT>::Ptr &pointCloud)
    {
        //do the reshaping
        pointCloud->width = 1;
        pointCloud->height = pointCloud->points.size();

        if (pcl::io::savePLYFile<PointT>(fileName, *pointCloud) == -1) //* load the file
        {
            PCL_ERROR("Couldn't write file \n");
            return (-1);
        }
    }

    bool read_txt_file(const std::string &fileName, const typename pcl::PointCloud<PointT>::Ptr &pointCloud)
    {
        std::ifstream in(fileName.c_str(), std::ios::in);
        if (!in)
        {
            return 0;
        }
        double x_ = 0, y_ = 0, z_ = 0, intensity_ = 0;
        int i = 0;
        while (!in.eof())
        {
            in >> x_ >> y_ >> z_ >> intensity_;
            if (in.fail())
            {
                break;
            }
            PointT Pt;
            Pt.x = x_;
            Pt.y = y_;
            Pt.z = z_;
            Pt.intensity = intensity_;
            pointCloud->points.push_back(Pt);
            ++i;
        }
        in.close();
        //std::cout << "Import finished ... ..." << std::endl;
        return 1;
    }

    bool write_txt_file(const std::string &fileName, const typename pcl::PointCloud<PointT>::Ptr &pointCloud)
    {
        std::ofstream ofs;
        ofs.open(fileName.c_str());
        if (ofs.is_open())
        {
            for (size_t i = 0; i < pointCloud->size(); ++i)
            {
                ofs << setiosflags(std::ios::fixed) << std::setprecision(6) << pointCloud->points[i].x << "  "
                    << setiosflags(std::ios::fixed) << std::setprecision(6) << pointCloud->points[i].y << "  "
                    << setiosflags(std::ios::fixed) << std::setprecision(6) << pointCloud->points[i].z
                    //<<"  "<< setiosflags(std::ios::fixed) << std::setprecision(6) << pointCloud->points[i].intensity
                    << std::endl;
            }
            ofs.close();
        }
        else
        {
            return 0;
        }
        //std::cout << "Output finished ... ..." << std::endl;
        return 1;
    }

    bool write_txt_file(const std::string &fileName, const typename pcl::PointCloud<PointT>::Ptr &pointCloud, int subsample_ratio)
    {
        std::ofstream ofs;
        ofs.open(fileName.c_str());
        if (ofs.is_open())
        {
            for (size_t i = 0; i < pointCloud->size(); ++i)
            {
                if (i % subsample_ratio == 0) //Subsampling;
                {
                    ofs << setiosflags(std::ios::fixed) << std::setprecision(6) << pointCloud->points[i].x << "  "
                        << setiosflags(std::ios::fixed) << std::setprecision(6) << pointCloud->points[i].y << "  "
                        << setiosflags(std::ios::fixed) << std::setprecision(6) << pointCloud->points[i].z
                        //<<"  "<< setiosflags(std::ios::fixed) << std::setprecision(6) << pointCloud->points[i].intensity
                        << std::endl;
                }
            }
            ofs.close();
        }
        else
        {
            return 0;
        }
        //std::cout << "Output finished ... ..." << std::endl;
        return 1;
    }

#if defined(WIN32) || defined(_WIN32) || defined(__WIN32) && !defined(__CYGWIN__)
    //read filename in sequence (suitable for windows)
    bool batch_read_filenames_in_folder(const std::string &folderName, const std::string &extension,
                                        std::vector<std::string> &fileNames,
                                        int frame_begin = 0, int frame_end = 99999)
    {
        if (!exists(folderName))
        {
            return 0;
        }
        else
        {
            directory_iterator end_iter;
            for (directory_iterator iter(folderName); iter != end_iter; ++iter)
            {
                if (is_regular_file(iter->status()))
                {
                    std::string fileName;
                    fileName = iter->path().string();

                    path dir(fileName);

                    if (!dir.extension().string().empty())
                    {
                        if (!fileName.substr(fileName.rfind('.')).compare(extension))
                        {
                            fileNames.push_back(fileName);
                            LOG(INFO) << "Record the file: [" << fileName << "].";
                        }
                    }
                }
            }
        }

        return 1;
    }
#else
    // The files are disorderly in Linux, you need to use another filename list file to read the file names in order.
    bool batch_read_filenames_in_folder(const std::string &folderName,
                                        const std::string &file_list_extenstion,
                                        const std::string &extension,
                                        std::vector<std::string> &fileNames,
                                        int frame_begin = 0, int frame_end = 99999, int frame_step = 1)
    {
        std::string filename_list = folderName + file_list_extenstion;

        //read image filename
        std::ifstream name_list_file(filename_list.c_str(), std::ios::in);
        if (!name_list_file.is_open())
        {
            LOG(WARNING) << "open filename_list failed, file is: " << filename_list;
            return 0;
        }

        int frame_count = 0;

        while (name_list_file.peek() != EOF)
        {
            std::string cur_file;
            name_list_file >> cur_file;

            if (!cur_file.empty() && !cur_file.substr(cur_file.rfind('.')).compare(extension))
            {
                if (frame_count >= frame_begin && frame_count <= frame_end && ((frame_count - frame_begin) % frame_step == 0))
                {
                    cur_file = folderName + "/" + cur_file;
                    fileNames.push_back(cur_file);
                    LOG(INFO) << "Record the file: [" << cur_file << "].";
                }
                frame_count++;
            }
        }
        name_list_file.close();

        return 1;
    }

#endif
    bool batch_read_filenames_in_folder_subfolder(const std::string &folderName,
                                                  const std::string &extension, std::vector<std::string> &fileNames)
    {
        boost::filesystem::path fullpath(folderName);
        if (!exists(fullpath))
        {
            return false;
        }
        recursive_directory_iterator end_iter;
        for (recursive_directory_iterator iter(fullpath); iter != end_iter; iter++)
        {
            try
            {
                if (is_directory(*iter))
                {
                }
                else
                {
                    std::string sFileName = iter->path().string();
                    path dir(sFileName);

                    if (!dir.extension().string().empty())
                    {
                        if (!sFileName.substr(sFileName.rfind('.')).compare(extension))
                        {
                            fileNames.push_back(sFileName);
                        }
                    }
                }
            }
            catch (const std::exception &ex)
            {
                std::cerr << ex.what() << std::endl;
                continue;
            }
        }
        return true;
    }

    bool batch_read_filenames_subfolder(const std::string &folderName, const std::string &extension, std::vector<std::vector<std::string>> &fileNames)
    {
        int subfolder_num = 0;

        if (!exists(folderName))
        {
            return 0;
        }
        else
        {
            directory_iterator end_iter;
            for (directory_iterator iter(folderName); iter != end_iter; ++iter)
            {
                if (is_directory(iter->status()))
                {
                    string subfoldername;
                    subfoldername = iter->path().string();

                    std::vector<std::string> fileNames_in_subfolder;
                    batch_read_filenames_in_folder(subfoldername, extension, fileNames_in_subfolder);
                    fileNames.push_back(fileNames_in_subfolder);
                    subfolder_num++;
                }
            }
        }
        LOG(INFO) << subfolder_num << " Sub-folders in the folder have been processed";
        return 1;
    }

    // The files are disorderly in Linux, you need to use another filename list file to read the file names in order.
    // The config parameters are also readed
    bool batch_read_filenames_subfolder(const std::string &folderName, const std::string &filelist_extension, const std::string &extension,
                                        std::vector<std::vector<std::string>> &fileNames, std::vector<bool> &station_position_available_s,
                                        std::vector<bool> &station_pose_available_s, std::vector<bool> &is_single_scanline_s)
    {
        int subfolder_num = 0;

        if (!exists(folderName))
        {
            return 0;
        }
        else
        {
            directory_iterator end_iter;
            for (directory_iterator iter(folderName); iter != end_iter; ++iter)
            {
                if (is_directory(iter->status()))
                {
                    string subfoldername;
                    subfoldername = iter->path().string();

                    std::vector<std::string> fileNames_in_subfolder;
                    bool station_position_available, station_pose_available, is_single_scanline;
                    batch_read_filenames_in_folder(subfoldername, filelist_extension, extension, fileNames_in_subfolder,
                                                   station_position_available, station_pose_available, is_single_scanline);
                    fileNames.push_back(fileNames_in_subfolder);
                    station_position_available_s.push_back(station_position_available);
                    station_pose_available_s.push_back(station_pose_available);
                    is_single_scanline_s.push_back(is_single_scanline);
                    subfolder_num++;
                }
            }
        }
        LOG(INFO) << subfolder_num << " Sub-folders in the folder have been processed";
        return 1;
    }

    // The files are disorderly in Linux, you need to use another filename list file to read the file names in order.
    bool batch_read_filenames_subfolder(const std::string &folderName, const std::string &filelist_extension, const std::string &extension, std::vector<std::vector<std::string>> &fileNames)
    {
        int subfolder_num = 0;

        if (!exists(folderName))
        {
            return 0;
        }
        else
        {
            directory_iterator end_iter;
            for (directory_iterator iter(folderName); iter != end_iter; ++iter)
            {
                if (is_directory(iter->status()))
                {
                    string subfoldername;
                    subfoldername = iter->path().string();

                    std::vector<std::string> fileNames_in_subfolder;
                    batch_read_filenames_in_folder(subfoldername, filelist_extension, extension, fileNames_in_subfolder);
                    fileNames.push_back(fileNames_in_subfolder);
                    subfolder_num++;
                }
            }
        }
        LOG(INFO) << subfolder_num << " Sub-folders in the folder have been processed";
        return 1;
    }

    bool output_keypoints(const std::string &filename, const pcl::PointIndicesPtr &indices, const typename pcl::PointCloud<PointT>::Ptr &pointCloud)
    {
        std::ofstream ofs;
        ofs.open(filename);

        if (ofs.is_open())
        {
            for (int i = 0; i < indices->indices.size(); ++i)
            {
                ofs << setiosflags(std::ios::fixed) << std::setprecision(6) << pointCloud->points[indices->indices[i]].x << "\t"
                    << setiosflags(std::ios::fixed) << std::setprecision(6) << pointCloud->points[indices->indices[i]].y << "\t"
                    << setiosflags(std::ios::fixed) << std::setprecision(6) << pointCloud->points[indices->indices[i]].z << std::endl;
            }
            ofs.close();
        }
        else
        {
            return 0;
        }
        return 1;
    }

    bool save_coordinates(const typename pcl::PointCloud<PointT>::Ptr &Source_FPC, const typename pcl::PointCloud<PointT>::Ptr &Target_FPC,
                          pcl::PointIndicesPtr &Source_KPI, pcl::PointIndicesPtr &Target_KPI,
                          Eigen::MatrixX3d &SXYZ, Eigen::MatrixX3d &TXYZ)
    {

        SXYZ.resize(Source_KPI->indices.size(), 3);
        TXYZ.resize(Target_KPI->indices.size(), 3);
        for (int i = 0; i < Source_KPI->indices.size(); ++i)
        {
            SXYZ.row(i) << Source_FPC->points[Source_KPI->indices[i]].x, Source_FPC->points[Source_KPI->indices[i]].y, Source_FPC->points[Source_KPI->indices[i]].z;
        }
        for (int i = 0; i < Target_KPI->indices.size(); ++i)
        {
            TXYZ.row(i) << Target_FPC->points[Target_KPI->indices[i]].x, Target_FPC->points[Target_KPI->indices[i]].y, Target_FPC->points[Target_KPI->indices[i]].z;
        }
        std::cout << "Key points saved." << std::endl;
        return 1;
    }

    bool read_indices_list(std::vector<int> &indicesA, std::vector<int> &indicesB)
    {
        std::string indiceslistFile;

        std::cout << "Please enter or drag in the Correspondence Tie Point Indices List File" << std::endl
                  << "Example [IndicesListFile.txt] :" << std::endl
                  << "107562 934051 " << std::endl
                  << "275003 18204" << std::endl
                  << "872055 462058" << std::endl
                  << "...  ..." << std::endl;

        std::cin >> indiceslistFile;

        std::ifstream in(indiceslistFile, ios::in);
        if (!in)
        {
            return 0;
        }

        int i = 0;
        while (!in.eof())
        {
            int p1, p2;
            in >> p1 >> p2;
            if (in.fail())
            {
                break;
            }
            indicesA.push_back(p1);
            indicesB.push_back(p2);
            ++i;
        }
        in.close();
    }

    bool read_indices_list(const typename pcl::PointCloud<PointT>::Ptr &CloudA, const typename pcl::PointCloud<PointT>::Ptr &CloudB, std::vector<std::vector<double>> &coordinatesA, std::vector<std::vector<double>> &coordinatesB)
    {
        std::string indiceslistFile;

        std::cout << "Please enter or drag in the Correspondence Tie Point Indices List File" << std::endl
                  << "Example [IndicesListFile.txt] :" << std::endl
                  << "107562 934051 " << std::endl
                  << "275003 18204" << std::endl
                  << "872055 462058" << std::endl
                  << "...  ..." << std::endl;

        std::cin >> indiceslistFile;

        std::ifstream in(indiceslistFile, ios::in);
        if (!in)
        {
            return 0;
        }

        std::vector<int> pointlistA;
        std::vector<int> pointlistB;

        int i = 0;
        while (!in.eof())
        {
            int p1, p2;
            in >> p1 >> p2;
            if (in.fail())
            {
                break;
            }
            pointlistA.push_back(p1);
            pointlistB.push_back(p2);
            ++i;
        }
        in.close();

        for (int j = 0; j < pointlistA.size(); j++)
        {
            std::vector<double> pointA(3);
            pointA[0] = CloudA->points[pointlistA[j]].x;
            pointA[1] = CloudA->points[pointlistA[j]].y;
            pointA[2] = CloudA->points[pointlistA[j]].z;
            coordinatesA.push_back(pointA);
        }

        for (int j = 0; j < pointlistB.size(); j++)
        {
            std::vector<double> pointB(3);
            pointB[0] = CloudB->points[pointlistB[j]].x;
            pointB[1] = CloudB->points[pointlistB[j]].y;
            pointB[2] = CloudB->points[pointlistB[j]].z;
            coordinatesB.push_back(pointB);
        }

        std::cout << "Procession Done ..." << std::endl;
    }

    bool check_overwrite_exsiting_file_or_not(std::string &output_file, std::string new_file_tail = "_new.txt")
    {
        bool to_overwrite = true;

        srand(time(NULL)); //Generate the random seed according to current time
        int rand0_9999 = rand() % 10000;

        new_file_tail = "_" + std::to_string(rand0_9999) + ".txt"; //use a random code (from 0-9999)

        while (exists_file(output_file))
        {
            if (to_overwrite)
            {
                std::cout << "The file has already existed, are you sure to overwrite it?  0. No  1. Yes [default 1]" << std::endl;
                std::cin >> to_overwrite;
            }

            if (!to_overwrite)
                output_file = output_file.substr(0, output_file.rfind(".")) + new_file_tail;
            else
                break;
        }
        LOG(INFO) << "A new file [" << output_file << "] would be generated and used later";
    }

    bool write_constraint_file(const std::string &con_output_file, constraint_t &con)
    {
        std::ofstream out(con_output_file, std::ios::app); // add after the file

        if (!out)
        {
            return 0;
        }
        out << con; //already overloaded

        out.close();
        return 1;
    }

    bool read_constraint_file(const std::string &con_file, constraints &cons)
    {
        std::ifstream in(con_file, std::ios::in);
        if (!in)
        {
            return 0;
        }

        LOG(INFO) << "Begin to read the constraint file";
        //read three row of head
        std::string strline;
        int head_row_count = 7;
        for (int i = 0; i < head_row_count; i++)
            getline(in, strline);

        //read the global shift from world to map system
        global_shift.resize(3);
        in >> global_shift[0] >> global_shift[1] >> global_shift[2];

        LOG(INFO) << "The global shift is: (" << global_shift[0] << "\t" << global_shift[1] << "\t" << global_shift[2] << ")";

        //read another line ----------------
        in >> strline;
        LOG(INFO) << strline;
        LOG(INFO) << "Read the constraint file head done";

        //begin reading the main file

        if (in.eof())
            return 0;

        //read constraint data
        while (1) //deal with the empty line problem
        {
            constraint_t con;
            int temp_con_type, temp_block1_type, temp_block2_type;

            in >> con.unique_id >> temp_con_type >> con.block1->unique_id >>
                temp_block1_type >> con.block2->unique_id >> temp_block2_type;

            if (in.eof())
                break;

            con.con_type = (ConstraintType)temp_con_type;
            con.block1->data_type = (DataType)temp_block1_type;
            con.block2->data_type = (DataType)temp_block2_type;

            for (int i = 0; i < 4; i++)
            {
                in >> con.Trans1_2(i, 0) >> con.Trans1_2(i, 1) >> con.Trans1_2(i, 2) >> con.Trans1_2(i, 3);
            }
            for (int i = 0; i < 6; i++)
            {
                in >> con.information_matrix(i, 0) >> con.information_matrix(i, 1) >> con.information_matrix(i, 2) >> con.information_matrix(i, 3) >> con.information_matrix(i, 4) >> con.information_matrix(i, 5);
            }

            if (con.con_type != NONE) //NONE
                cons.push_back(con);

            LOG(INFO) << "import " << con.unique_id;
        }
        in.close();

        LOG(WARNING) << "Import [" << cons.size() << "] constraints from file [" << con_file << "].";

        return 1;
    }

    // bool read_station_pose(Eigen::Matrix4d &station_pose, std::string &infile)
    // {
    //     ;
    // }
    

    //read the coordinate of the line begining with target_station_name
    //format:
    //station_name x y z
    bool read_station_position(const std::string &infile, const std::string &target_station_name,
                               centerpoint_t &station_position, bool automatic_shift_or_not = false)
    {
        std::ifstream in(infile, ios::in);
        if (!in)
        {
            return 0;
        }

        std::string station_name;
        float sx, sy, sz;
        while (!in.eof())
        {
            in >> station_name >> sx >> sy >> sz;
            if (!station_name.compare(target_station_name)) //same string
            {
                station_position.x = sx;
                station_position.y = sy;
                station_position.z = sz;
                break;
            }
        }

        in.close();

        if (automatic_shift_or_not)
        {
            station_position.x += global_shift[0];
            station_position.y += global_shift[1];
            station_position.z += global_shift[2];
        }

        //std::cout<<"Import done"<<std::endl;
        return 1;
    }
    
    // Used for coordinate system convertation
    bool read_coord_list_pair(std::vector<std::vector<double>> &coordinatesA, std::vector<std::vector<double>> &coordinatesB)
    {
        std::string XYZListFileA, XYZListFileB;

        std::cout << "Please enter or drag in the Tie Points' XYZ List File of Station A" << std::endl
                  << "Example [pickinglist_XYZ_A.txt] :" << std::endl
                  << "11.92,23.07,0.82" << std::endl
                  << "15.34,18.02,1.25" << std::endl
                  << "27.01,-7.94,1.37" << std::endl
                  << "...  ... ..." << std::endl;

        std::cin >> XYZListFileA;

        std::ifstream inA(XYZListFileA, ios::in);
        if (!inA)
        {
            return 0;
        }

        int i = 0;
        while (!inA.eof())
        {
            std::vector<double> Pt(3);
            char comma;
            inA >> Pt[0] >> comma >> Pt[1] >> comma >> Pt[2];
            if (inA.fail())
            {
                break;
            }
            coordinatesA.push_back(Pt);
            ++i;
        }
        inA.close();

        std::cout << "Please enter or drag in the Tie Points' XYZ List File of Station B" << std::endl;

        std::cin >> XYZListFileB;

        std::ifstream inB(XYZListFileB, ios::in);
        if (!inB)
        {
            return 0;
        }

        int j = 0;
        while (!inB.eof())
        {
            std::vector<double> Pt(3);
            char comma;
            inB >> Pt[0] >> comma >> Pt[1] >> comma >> Pt[2];
            if (inB.fail())
            {
                break;
            }
            coordinatesB.push_back(Pt);
            ++j;
        }
        inB.close();

        std::cout << "Procession Done ..." << std::endl;
    }

    bool output_xyz_4dof_coord_syst_tran(std::vector<double> &transpara)
    {
        std::string XYZListFileA, XYZListFileB;
        std::vector<std::vector<double>> coordinatesA;
        std::cout << "Please enter or drag in the XYZ List File of Coordinate System A" << std::endl
                  << "Example [pointlist_XYZ_A.txt] :" << std::endl
                  << "11.92,23.07,0.82" << std::endl
                  << "15.34,18.02,1.25" << std::endl
                  << "27.01,-7.94,1.37" << std::endl
                  << "...  ... ..." << std::endl;

        std::cin >> XYZListFileA;

        std::ifstream inA(XYZListFileA, ios::in);
        if (!inA)
        {
            return 0;
        }

        int i = 0;
        while (!inA.eof())
        {
            std::vector<double> Pt(3);
            char comma;
            inA >> Pt[0] >> comma >> Pt[1] >> comma >> Pt[2];
            if (inA.fail())
            {
                break;
            }
            coordinatesA.push_back(Pt);
            ++i;
        }
        inA.close();

        std::cout << "Output the transformed result" << std::endl;
        XYZListFileB = XYZListFileA.substr(0, XYZListFileA.rfind(".")) + "_utm.txt";
        std::ofstream ofs;
        ofs.open(XYZListFileB);
        if (ofs.is_open())
        {
            for (int j = 0; j < i; j++)
            {
                double X_tran = transpara[2] * transpara[4] * coordinatesA[j][0] - transpara[2] * transpara[3] * coordinatesA[j][1] + transpara[0];
                double Y_tran = transpara[2] * transpara[3] * coordinatesA[j][0] + transpara[2] * transpara[4] * coordinatesA[j][1] + transpara[1];
                double Z_tran = coordinatesA[j][2];
                ofs << setiosflags(ios::fixed) << setprecision(8) << X_tran << ","
                    << setiosflags(ios::fixed) << setprecision(8) << Y_tran << ","
                    << setiosflags(ios::fixed) << setprecision(8) << Z_tran << std::endl;
            }
            ofs.close();
        }
        else
        {
            return 0;
        }

        std::cout << "Procession Done ..." << std::endl;
        return 1;
    }

    bool output_xyz_7dof_coord_syst_tran(std::vector<double> &transpara)
    {
        std::string XYZListFileA, XYZListFileB;
        std::vector<std::vector<double>> coordinatesA;
        std::cout << "Please enter or drag in the XYZ List File of Coordinate System A" << std::endl
                  << "Example [pointlist_XYZ_A.txt] :" << std::endl
                  << "11.92,23.07,0.82" << std::endl
                  << "15.34,18.02,1.25" << std::endl
                  << "27.01,-7.94,1.37" << std::endl
                  << "...  ... ..." << std::endl;

        std::cin >> XYZListFileA;

        std::ifstream inA(XYZListFileA, ios::in);
        if (!inA)
        {
            return 0;
        }

        int i = 0;
        while (!inA.eof())
        {
            std::vector<double> Pt(3);
            char comma;
            inA >> Pt[0] >> comma >> Pt[1] >> comma >> Pt[2];
            if (inA.fail())
            {
                break;
            }
            coordinatesA.push_back(Pt);
            ++i;
        }
        inA.close();

        std::cout << "Output the transformed result" << std::endl;
        XYZListFileB = XYZListFileA.substr(0, XYZListFileA.rfind(".")) + "_utm.txt";
        std::ofstream ofs;
        ofs.open(XYZListFileB);
        if (ofs.is_open())
        {
            for (int j = 0; j < i; j++)
            {
                double X_tran = transpara[0] + transpara[6] * coordinatesA[j][0] + transpara[5] * coordinatesA[j][1] - transpara[4] * coordinatesA[j][2];
                double Y_tran = transpara[1] + transpara[6] * coordinatesA[j][1] - transpara[5] * coordinatesA[j][0] + transpara[3] * coordinatesA[j][2];
                double Z_tran = transpara[2] + transpara[6] * coordinatesA[j][2] + transpara[4] * coordinatesA[j][0] - transpara[3] * coordinatesA[j][1];
                ofs << setiosflags(ios::fixed) << setprecision(8) << X_tran << ","
                    << setiosflags(ios::fixed) << setprecision(8) << Y_tran << ","
                    << setiosflags(ios::fixed) << setprecision(8) << Z_tran << std::endl;
            }
            ofs.close();
        }
        else
        {
            return 0;
        }

        std::cout << "Procession Done ..." << std::endl;
        return 1;
    }

#if 0
    bool tran_eng2utm(float centerlong_eng_proj)
    {
        string XYZENGListFile;

        cout << "Please enter or drag in the Points' XYZ List File for Engineering Coordinate System" << endl
             << "Example [Pointlist_XYZ_ENGCS.txt] :" << endl
             << "485026.778,3409071.864,474.672" << endl
             << "485182.217,3409201.304,474.314" << endl
             << "487070.108,3411533.570,471.484" << endl
             << "... ... ..." << endl;

        cin >> XYZENGListFile;

        ifstream inlist(XYZENGListFile, ios::in);
        if (!inlist)
        {
            return 0;
        }

        GeoTransform gt;

        int j = 0;
        while (!inlist.eof())
        {
            std::vector<double> PtENGXYZ(3);
            std::vector<double> PtBLH(3);
            std::vector<double> PtUTMXYZ(3);

            char comma;
            inlist >> PtENGXYZ[0] >> comma >> PtENGXYZ[1] >> comma >> PtENGXYZ[2];
            if (inlist.fail())
            {
                break;
            }

            cout.setf(ios::showpoint);
            cout.precision(12);

            gt.XYZ2BLH_ENG(PtENGXYZ, centerlong_eng_proj, PtBLH);
            cout << PtBLH[0] << " , " << PtBLH[1] << " , " << PtBLH[2] << endl;

            gt.BLH2XYZ_WGS84(PtBLH, PtUTMXYZ);
            cout << PtUTMXYZ[0] << " , " << PtUTMXYZ[1] << " , " << PtUTMXYZ[2] << endl;
            //coordinatesUTM_XYZ.push_back(PtUTM);
            ++j;
        }
        inlist.close();

        cout << "Procession Done ..." << endl;
        return 1;
    }

    bool tran_wgs2eng(float centerlong_eng_proj, float proj_surface_h_eng)
    {
        string XYZWGSListFile;

        cout << "Please enter or drag in the Points' BLH List File for WGS84/CGCS2000" << endl
             << "Example [Pointlist_BLH_WGS84.txt] :" << endl;

        cin >> XYZWGSListFile;

        ifstream inlist(XYZWGSListFile, ios::in);
        if (!inlist)
        {
            return 0;
        }

        GeoTransform gt;

        int j = 0;
        while (!inlist.eof())
        {
            std::vector<double> PtENGXYZ(3);
            std::vector<double> PtBLH(3);
            //std::vector<double> PtUTMXYZ(3);

            char comma;
            inlist >> PtBLH[0] >> comma >> PtBLH[1] >> comma >> PtBLH[2];
            if (inlist.fail())
            {
                break;
            }

            cout.setf(ios::showpoint);
            cout.precision(12);

            gt.BLH2XYZ_CGCS(PtBLH, centerlong_eng_proj, proj_surface_h_eng, PtENGXYZ);
            cout << PtENGXYZ[0] << " , " << PtENGXYZ[1] << " , " << PtENGXYZ[2] << endl;

            ++j;
        }
        inlist.close();

        cout << "Procession Done ..." << endl;
        return 1;
    }

    bool read_xyz_blh_list(std::vector<std::vector<double>> &coordinatesSC_XYZ,
                          std::vector<std::vector<double>> &coordinatesUTM_XYZ)
    {
        string XYZListFileA, BLHListFileB;

        cout << "Please enter or drag in the Tie Points' XYZ List File of Station A" << endl
             << "Example [pickinglist_XYZ_A.txt] :" << endl
             << "11.92,23.07,0.82" << endl
             << "15.34,18.02,1.25" << endl
             << "27.01,-7.94,1.37" << endl
             << "... ... ..." << endl;

        cin >> XYZListFileA;

        ifstream inA(XYZListFileA, ios::in);
        if (!inA)
        {
            return 0;
        }

        int i = 0;
        while (!inA.eof())
        {
            std::vector<double> Pt(3);
            char comma;
            inA >> Pt[0] >> comma >> Pt[1] >> comma >> Pt[2];
            if (inA.fail())
            {
                break;
            }
            coordinatesSC_XYZ.push_back(Pt);
            ++i;
        }
        inA.close();

        GeoTransform gt;
        int utmzone;

        cout << "Please enter or drag in the Tie Points' WGS84 BLH Coordinates List" << endl
             << "Example [pickinglist_BLH_WGS84.txt] :" << endl
             << "30.71418,115.71602,202.1275" << endl
             << "30.71803,115.71870,208.2477" << endl
             << "... ... ..." << endl;
        cin >> BLHListFileB;

        ifstream inB(BLHListFileB, ios::in);
        if (!inB)
        {
            return 0;
        }

        int j = 0;
        while (!inB.eof())
        {
            std::vector<double> Pt(3);
            std::vector<double> PtUTM(3);
            char comma;
            inB >> Pt[0] >> comma >> Pt[1] >> comma >> Pt[2];
            if (inB.fail())
            {
                break;
            }
            utmzone = gt.BLH2XYZ_WGS84(Pt, PtUTM);

            cout.setf(ios::showpoint);
            cout.precision(12);

            cout << PtUTM[0] << " , " << PtUTM[1] << " , " << PtUTM[2] << endl;
            coordinatesUTM_XYZ.push_back(PtUTM);
            ++j;
        }
        inB.close();

        cout << "Procession Done ..." << endl;
    }
#endif

    bool read_pc_cloud_block(cloudblock_Ptr &in_block, bool normalize_intensity_or_not = false)
    {
        if (read_cloud_file(in_block->filename, in_block->pc_raw))
        {
            this->get_cloud_bbx_cpt(in_block->pc_raw, in_block->local_bound, in_block->local_cp);

            if (normalize_intensity_or_not)
            {
                float min_intensity = FLT_MAX;
                float max_intensity = -FLT_MAX;
                for (int i = 0; i < in_block->pc_raw->points.size(); i++)
                {
                    min_intensity = min_(min_intensity, in_block->pc_raw->points[i].intensity);
                    max_intensity = max_(max_intensity, in_block->pc_raw->points[i].intensity);
                }
                float intesnity_scale = 255.0 / (max_intensity - min_intensity); //rescale to 0-255
                for (int i = 0; i < in_block->pc_raw->points.size(); i++)
                    in_block->pc_raw->points[i].intensity = (in_block->pc_raw->points[i].intensity - min_intensity) * intesnity_scale;
            }

            return 1;
        }
        else
            return 0;
    }

    bool read_problematic_frames(const std::string &eval_file, std::vector<std::vector<unsigned int>> &problematic_frames)
    {
        std::ifstream in(eval_file, std::ios::in);
        if (!in)
        {
            return 0;
        }

        problematic_frames.resize(3);
        unsigned int frame_id;

        //read eight row of head
        std::string strline;
        int head_row_count = 8;
        for (int i = 0; i < head_row_count; i++)
            getline(in, strline);

        getline(in, strline);
        std::stringstream ss_1(strline);

        while (1)
        {
            if (ss_1.peek() == EOF) //don't use ss_1.eof() --> it will read one more word/line
                break;
            ss_1 >> frame_id;
            problematic_frames[0].push_back(frame_id);
        }

        //jump 2 lines
        getline(in, strline);
        getline(in, strline);

        getline(in, strline);
        std::stringstream ss_2(strline);

        while (1)
        {
            if (ss_2.peek() == EOF)
                break;
            ss_2 >> frame_id;
            problematic_frames[1].push_back(frame_id);
        }

        //jump 2 lines
        getline(in, strline);
        getline(in, strline);

        getline(in, strline);
        std::stringstream ss_3(strline);

        while (1)
        {
            if (ss_3.peek() == EOF)
                break;
            ss_3 >> frame_id;
            problematic_frames[2].push_back(frame_id);
        }

        in.close();

        LOG(WARNING) << "Find [" << problematic_frames[0].size() << "] horizontal problematic frames";
        LOG(WARNING) << "Find [" << problematic_frames[1].size() << "] vertical problematic frames";
        LOG(WARNING) << "Find [" << problematic_frames[2].size() << "] heading angle problematic frames";

        return 1;
    }

    bool batch_write_optimized_cloud(strip &all_blocks)
    {
        std::string filename_in, filename_out, folder_out;

        global_shift.resize(3);
        for (auto iter = all_blocks.begin(); iter != all_blocks.end(); iter++)
        {
            filename_in = iter->filename;

            //example filename: /media/edward/Seagate/Data/Highway_dataset/ALS/blocks/L001-1-V02-S1-C1_r/1.las
            folder_out = filename_in.substr(0, filename_in.rfind("/")) + "/output"; // "/" for Linux and "\\" for windows

            check_dir(folder_out);

            filename_out = folder_out + filename_in.substr(filename_in.rfind("/"), filename_in.rfind(".") - filename_in.rfind("/")) + "_o.las";

            typename pcl::PointCloud<PointT>::Ptr cloudin(new pcl::PointCloud<PointT>());
            typename pcl::PointCloud<PointT>::Ptr cloudout(new pcl::PointCloud<PointT>());

            read_las_file_last(filename_in, cloudin); //use the recorded global shift

            //Eigen::Matrix4d corrected_pose;
            //regx.get_inv_trans((*iter).pose_optimized, corrected_pose);

            pcl::transformPointCloud(*cloudin, *cloudout, iter->pose_optimized);
            write_block_in_color(filename_out, cloudout, 1); //use the recorded global shift

            pcl::PointCloud<PointT>().swap(*cloudin);
            pcl::PointCloud<PointT>().swap(*cloudout);

            LOG(INFO) << "Output Done for cloud with index " << iter->unique_id;
            LOG(INFO) << "The output path is: " << filename_out;
        }
        LOG(INFO) << "Batch output optimized cloud blocks done.";

        return 1;
    }

    bool batch_write_block_metadata(strip &all_blocks, std::string &block_output_file)
    {
        check_overwrite_exsiting_file_or_not(block_output_file);

        //LOG(INFO) << "A new file [" << block_output_file << "] would be generated and used later";

        std::ofstream out(block_output_file); //overwrite
        if (!out)
        {
            return 0;
        }

        //Write the head of the file
        out << "#Format: Block_ID  Block_Type\n";
        out << "bbx (minx,miny,minz,maxx,maxy,maxz)\n";
        out << "pose_optimizedformation_Mat (4*4)\n";

        for (int i = 0; i < all_blocks.size(); i++)
        {
            out << all_blocks[i].unique_id << "\t" << all_blocks[i].data_type << "\n";
            out << all_blocks[i].filename << "\n";

            out << all_blocks[i].bound;
            out << all_blocks[i].pose_optimized;
        }

        out.close();

        LOG(INFO) << "Write [" << block_output_file << "] done";

        return 1;
    }

    bool write_lo_pose_overwrite(Eigen::Matrix4d &Trans1_2, std::string &output_file)
    {
        std::ofstream out(output_file);

        if (!out)
            return 0;

        std::string delimiter_str = " "; //or "\t"
        out << setprecision(8);
        out << Trans1_2(0, 0) << delimiter_str << Trans1_2(0, 1) << delimiter_str << Trans1_2(0, 2) << delimiter_str << Trans1_2(0, 3) << delimiter_str
            << Trans1_2(1, 0) << delimiter_str << Trans1_2(1, 1) << delimiter_str << Trans1_2(1, 2) << delimiter_str << Trans1_2(1, 3) << delimiter_str
            << Trans1_2(2, 0) << delimiter_str << Trans1_2(2, 1) << delimiter_str << Trans1_2(2, 2) << delimiter_str << Trans1_2(2, 3) << "\n";
        out.close();
        return 1;
    }

    bool write_lo_pose_append(Eigen::Matrix4d &Trans1_2, std::string &output_file)
    {
        std::ofstream out(output_file, std::ios::app); // add after the file

        if (!out)
            return 0;

        std::string delimiter_str = " "; //or "\t"
        out << setprecision(8);
        out << Trans1_2(0, 0) << delimiter_str << Trans1_2(0, 1) << delimiter_str << Trans1_2(0, 2) << delimiter_str << Trans1_2(0, 3) << delimiter_str
            << Trans1_2(1, 0) << delimiter_str << Trans1_2(1, 1) << delimiter_str << Trans1_2(1, 2) << delimiter_str << Trans1_2(1, 3) << delimiter_str
            << Trans1_2(2, 0) << delimiter_str << Trans1_2(2, 1) << delimiter_str << Trans1_2(2, 2) << delimiter_str << Trans1_2(2, 3) << "\n";
        out.close();
        return 1;
    }

    bool load_calib_mat(const std::string calib_file, Eigen::Matrix4d &calib_mat)
    {

        calib_mat.setIdentity();

        std::ifstream calibreader(calib_file);
        //cout<<filepath<<"\n";
        if (calibreader.is_open())
        {
            while (calibreader.peek() != EOF)
            {
                std::string line;
                getline(calibreader, line);

                std::stringstream ss(line);

                std::string flag;

                ss >> flag;

                if (flag.compare("Tr:") == 0)
                {
                    ss >> calib_mat(0, 0) >> calib_mat(0, 1) >> calib_mat(0, 2) >> calib_mat(0, 3) >> calib_mat(1, 0) >> calib_mat(1, 1) >> calib_mat(1, 2) >> calib_mat(1, 3) >> calib_mat(2, 0) >> calib_mat(2, 1) >> calib_mat(2, 2) >> calib_mat(2, 3);
                    break;
                }
            }
            calibreader.close();
        }
        else
            return 0;

        LOG(INFO) << "Calib matrix loaded\n";

        std::cout << setprecision(16) << calib_mat << std::endl;

        return 1;
    }

    //KITTI ground truth pose format
    //mat(0-11) split with space
    //example:
    //1.000000e+00 9.043680e-12 2.326809e-11 5.551115e-17 9.043683e-12 1.000000e+00 2.392370e-10 3.330669e-16 2.326810e-11 2.392370e-10 9.999999e-01 -4.440892e-16
    Matrix4ds load_poses_from_transform_matrix(std::string filepath, int frame_begin = 0, int frame_end = 99999, int frame_step = 1)
    {
        double tmp[12];
        Matrix4ds pose_vec;
        Eigen::Matrix4d temp_pose = Eigen::Matrix4d::Identity();
        std::ifstream posereader(filepath);

        int count = 0;
        while (posereader >> tmp[0])
        {
            for (int i = 1; i < 12; ++i)
            {
                posereader >> tmp[i];
            }
            for (int j = 0; j < 3; ++j)
            {
                for (int k = 0; k < 4; ++k)
                {
                    temp_pose(j, k) = tmp[4 * j + k];
                }
            }
            if ((count >= frame_begin) && (count <= frame_end) && ((count - frame_begin) % frame_step == 0))
                pose_vec.push_back(temp_pose);

            count++;
            //LOG(WARNING) << temp_pose;
        }
        return pose_vec;
    }

    // For OXTS Ground Truth pose
    // format:
    // index timestamp x y z q1 q2 q3 q4
    Matrix4ds load_poses_from_pose_quat(std::string filepath, int frame_begin = 0, int frame_end = 99999, int frame_step = 1)
    {
        std::ifstream tmp_file(filepath);

        Matrix4ds pose_vec;

        int count = 0;

        std::string line;
        if (tmp_file.is_open())
        {
            while (getline(tmp_file, line))
            {
                int temp_index;
                double temp_time;
                pose_qua_t temp_pose_quat;
                Eigen::Matrix4d temp_pose;

                std::istringstream in(line);

                in >> temp_index >> temp_time >> temp_pose_quat;

                temp_pose = temp_pose_quat.GetMatrix();

                if ((count >= frame_begin) && (count <= frame_end) && ((count - frame_begin) % frame_step == 0))
                    pose_vec.push_back(temp_pose);
                count++;
            }
        }
        else
        {
            LOG(ERROR) << "Unable to open input file [" << filepath << "].";
        }
        tmp_file.close();

        return pose_vec;
    }

    bool report_consuming_time(const std::string output_file, std::vector<std::vector<float>> &consuming_time) //TODO: output consuming time for each step
    {
        std::ofstream out(output_file); // add after the file

        if (!out)
            return false;

        std::string delimiter_str = " "; //or "\t"
        out << setprecision(8);
        float timing_factor = 1000; //unit: ms

        for (int i = 0; i < consuming_time.size(); i++)
        {
            out << timing_factor * consuming_time[i][0] << delimiter_str << timing_factor * consuming_time[i][1] << delimiter_str
                << timing_factor * consuming_time[i][2] << delimiter_str << timing_factor * consuming_time[i][3] << "\n";
        }
        out.close();
        return true;
    }

    bool read_bbx_8pts_with_type(const std::string filename, typename pcl::PointCloud<PointT>::Ptr &bbx_vertex, std::vector<int> &bbx_classes)
    {
        std::ifstream in(filename.c_str(), std::ios::in);
        if (!in)
        {
            return 0;
        }

        int i = 0;
        while (!in.eof())
        {
            PointT pt0, pt1, pt2, pt3, pt4, pt5, pt6, pt7;
            float bbx_class = 0;

            in >> pt0.x >> pt0.y >> pt0.z >> pt1.x >> pt1.y >> pt1.z >> pt2.x >> pt2.y >> pt2.z >> pt3.x >> pt3.y >> pt3.z >> pt4.x >> pt4.y >> pt4.z >> pt5.x >> pt5.y >> pt5.z >> pt6.x >> pt6.y >> pt6.z >> pt7.x >> pt7.y >> pt7.z >> bbx_class;
            if (in.fail())
            {
                break;
            }

            bbx_classes.push_back((int)bbx_class);
            bbx_vertex->points.push_back(pt0);
            bbx_vertex->points.push_back(pt1);
            bbx_vertex->points.push_back(pt2);
            bbx_vertex->points.push_back(pt3);
            bbx_vertex->points.push_back(pt4);
            bbx_vertex->points.push_back(pt5);
            bbx_vertex->points.push_back(pt6);
            bbx_vertex->points.push_back(pt7);
            ++i;
        }
        in.close();
        //std::cout << "Import finished ... ..." << std::endl;
        return 1;
    }

    bool get_global_shift(Eigen::Vector3f &global_shift_geo2local)
    {
        global_shift_geo2local(0) = global_shift[0];
        global_shift_geo2local(1) = global_shift[1];
        global_shift_geo2local(2) = global_shift[2];
        return 1;
    }

    bool write_pose_point_cloud(std::string &file_name, Matrix4ds &poses)
    {
        typename pcl::PointCloud<PointT>::Ptr pc_traj(new pcl::PointCloud<PointT>());

        for (int i = 0; i < poses.size(); i++)
        {
            PointT pt_temp;
            pt_temp.x = poses[i](0, 3);
            pt_temp.y = poses[i](1, 3);
            pt_temp.z = poses[i](2, 3);
            pt_temp.intensity = i;
            pc_traj->points.push_back(pt_temp);
        }

        write_pcd_file(file_name, pc_traj, true);
        return true;
    }

  private:
    void xyz_2_xy(const typename pcl::PointCloud<PointT>::Ptr &pointcloudxyz, pcl::PointCloud<pcl::PointXY>::Ptr &pointcloudxy)
    {
        for (size_t i = 0; i < pointcloudxyz->size(); i++)
        {
            pcl::PointXY ptXY;
            ptXY.x = pointcloudxyz->points[i].x;
            ptXY.y = pointcloudxyz->points[i].y;
            pointcloudxy->push_back(ptXY);
        }
    }

    std::vector<double> global_shift;
};
} // namespace lo

#endif // _INCLUDE_DATA_IO_HPP