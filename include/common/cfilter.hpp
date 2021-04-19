//
// This file is used for the filtering and feature point extraction of Point Cloud.
// Dependent 3rd Libs: PCL (>1.7)
// By Yue Pan
//

#ifndef _INCLUDE_FILTER_HPP
#define _INCLUDE_FILTER_HPP

//pcl
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/normal_space.h>
#include <pcl/filters/random_sample.h>

#if OPENCV_ON
//opencv
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#endif

#include <vector>
#include <iostream>
#include <cfloat>

#include "utility.hpp"
#include "cprocessing.hpp"
#include "pca.hpp"

namespace lo
{
template <typename PointT>
class CFilter : public CloudUtility<PointT>
{
  public:
	struct idpair_t
	{
		idpair_t() : idx(0), voxel_idx(0) {}

		unsigned long long voxel_idx;
		int idx;

		bool operator<(const idpair_t &pair) { return voxel_idx < pair.voxel_idx; }
	};

	struct grid_t
	{
		std::vector<int> point_id;
		float min_z;
		float max_z;
		float delta_z;
		float min_z_x; //X of Lowest Point in the Voxel;
		float min_z_y; //Y of Lowest Point in the Voxel;
		float min_z_outlier_thre;
		float neighbor_min_z;
		int pts_count;
		int reliable_neighbor_grid_num;
		float mean_z;
		float dist2station;

		grid_t()
		{
			min_z = min_z_x = min_z_y = neighbor_min_z = mean_z = 0.f;
			pts_count = 0;
			reliable_neighbor_grid_num = 0;
			delta_z = 0.0;
			dist2station = 0.001;
			min_z_outlier_thre = -FLT_MAX;
		}
	};

	struct simplified_voxel_t
	{
		std::vector<int> point_id;
		float max_curvature;
		int max_curvature_point_id;
		bool has_keypoint;
		simplified_voxel_t()
		{
			has_keypoint = false;
		}
	};

	bool voxel_downsample(const typename pcl::PointCloud<PointT>::Ptr &cloud_in, typename pcl::PointCloud<PointT>::Ptr &cloud_out, float voxel_size)
	{
		std::chrono::steady_clock::time_point tic = std::chrono::steady_clock::now();

		//You can set the downsampling_radius as 0 to disable the downsampling in order to save time for
		//lidar odometry test
		if (voxel_size < 0.001)
		{
			LOG(INFO) << "too small voxel size, the downsampling would be disabled";
			cloud_out = cloud_in;

			// for (int i=0; i<cloud_out->points.size(); i++)
			//   LOG(INFO)<< i <<","<< (int)(cloud_out->points[i].curvature);
			return false;
		}

		float inverse_voxel_size = 1.0f / voxel_size;

		Eigen::Vector4f min_p, max_p;
		pcl::getMinMax3D(*cloud_in, min_p, max_p);

		Eigen::Vector4f gap_p; //boundingbox gap;
		gap_p = max_p - min_p;

		unsigned long long max_vx = ceil(gap_p.coeff(0) * inverse_voxel_size) + 1;
		unsigned long long max_vy = ceil(gap_p.coeff(1) * inverse_voxel_size) + 1;
		unsigned long long max_vz = ceil(gap_p.coeff(2) * inverse_voxel_size) + 1;

		if (max_vx * max_vy * max_vz >= std::numeric_limits<unsigned long long>::max())
		{
			std::cout << "Filtering Failed: The number of box exceed the limit." << std::endl;
			return 0;
		}

		unsigned long long mul_vx = max_vy * max_vz;
		unsigned long long mul_vy = max_vz;
		unsigned long long mul_vz = 1;

		std::vector<idpair_t> id_pairs(cloud_in->points.size());

		int i;
//unsigned int idx = 0;
#pragma omp parallel for private(i) //Multi-thread
		for (i = 0; i < cloud_in->points.size(); i++)
		{
			unsigned long long vx = floor((cloud_in->points[i].x - min_p.coeff(0)) * inverse_voxel_size);
			unsigned long long vy = floor((cloud_in->points[i].y - min_p.coeff(1)) * inverse_voxel_size);
			unsigned long long vz = floor((cloud_in->points[i].z - min_p.coeff(2)) * inverse_voxel_size);

			unsigned long long voxel_idx = vx * mul_vx + vy * mul_vy + vz * mul_vz;
			idpair_t pair;
			pair.idx = i;
			pair.voxel_idx = voxel_idx;
			//id_pairs.push_back(pair);
			id_pairs[i] = pair;
		}

		//Do sorting
		std::sort(id_pairs.begin(), id_pairs.end());

		int begin_id = 0;

		while (begin_id < id_pairs.size())
		{
			cloud_out->push_back(cloud_in->points[id_pairs[begin_id].idx]);

			int compare_id = begin_id + 1;
			while (compare_id < id_pairs.size() && id_pairs[begin_id].voxel_idx == id_pairs[compare_id].voxel_idx)
				compare_id++;
			begin_id = compare_id;
		}

		std::chrono::steady_clock::time_point toc = std::chrono::steady_clock::now();
		std::chrono::duration<double> time_used = std::chrono::duration_cast<std::chrono::duration<double>>(toc - tic);

		//free the memory
		std::vector<idpair_t>().swap(id_pairs);

		LOG(INFO) << "[" << cloud_out->points.size() << "] points remain after the downsampling in ["
				  << time_used.count() * 1000.0 << "] ms.";

		return 1;
	}

	//Normal Space Downsampling (with normal already estimated)
	bool normal_downsample(const typename pcl::PointCloud<PointT>::Ptr &cloud_in_out, std::vector<int> &out_indices,
						   int downsample_ratio, int bin_num = 3, int rand_seed = 0)

	{
		std::chrono::steady_clock::time_point tic = std::chrono::steady_clock::now();

		int original_pts_num = cloud_in_out->points.size();
		typename pcl::PointCloud<PointT>::Ptr cloud_temp(new pcl::PointCloud<PointT>);
		pcl::NormalSpaceSampling<PointT, PointT> nss;
		nss.setInputCloud(cloud_in_out);

		nss.setNormals(cloud_in_out);

		nss.setBins(bin_num, bin_num, bin_num);
		nss.setSeed(rand_seed);
		nss.setSample(static_cast<unsigned int>(original_pts_num / downsample_ratio));

		std::chrono::steady_clock::time_point tic_2 = std::chrono::steady_clock::now();

		nss.filter(out_indices);

		for (int i = 0; i < out_indices.size(); i++)
			cloud_temp->points.push_back(cloud_in_out->points[out_indices[i]]);

		cloud_temp->points.swap(cloud_in_out->points);

		std::chrono::steady_clock::time_point toc = std::chrono::steady_clock::now();
		std::chrono::duration<double> time_set_normal_used = std::chrono::duration_cast<std::chrono::duration<double>>(tic_2 - tic);
		std::chrono::duration<double> time_used = std::chrono::duration_cast<std::chrono::duration<double>>(toc - tic);

		LOG(INFO) << "Normal space downsampling [ " << original_pts_num << " --> "
				  << out_indices.size() << " ] done in [" << time_used.count() * 1000.0 << "] ms [" << time_set_normal_used.count() * 1000.0 << "] ms for setting the normal\n";
		return 1;
	}

	//SOR (Statisics Outliers Remover);
	bool sor_filter(typename pcl::PointCloud<PointT>::Ptr &cloud_in, typename pcl::PointCloud<PointT>::Ptr &cloud_out, int mean_k, double n_std)
	{
		std::chrono::steady_clock::time_point tic = std::chrono::steady_clock::now();

		// Create the filtering object
		pcl::StatisticalOutlierRemoval<PointT> sor;

		sor.setInputCloud(cloud_in);
		sor.setMeanK(mean_k);
		sor.setStddevMulThresh(n_std);
		sor.filter(*cloud_out);

		std::chrono::steady_clock::time_point toc = std::chrono::steady_clock::now();
		std::chrono::duration<double> time_used = std::chrono::duration_cast<std::chrono::duration<double>>(toc - tic);

		LOG(INFO) << "SOR filter done in [" << time_used.count() * 1000.0 << "] ms";

		return 1;
	}

	//SOR (Statisics Outliers Remover)
	bool sor_filter(typename pcl::PointCloud<PointT>::Ptr &cloud_in_out, int mean_k, double n_std)
	{
		std::chrono::steady_clock::time_point tic = std::chrono::steady_clock::now();

		typename pcl::PointCloud<PointT>::Ptr cloud_temp(new pcl::PointCloud<PointT>());

		// Create the filtering object
		pcl::StatisticalOutlierRemoval<PointT> sor;

		sor.setInputCloud(cloud_in_out);
		sor.setMeanK(mean_k);
		sor.setStddevMulThresh(n_std);
		sor.filter(*cloud_temp);

		cloud_temp->points.swap(cloud_in_out->points);

		std::chrono::steady_clock::time_point toc = std::chrono::steady_clock::now();
		std::chrono::duration<double> time_used = std::chrono::duration_cast<std::chrono::duration<double>>(toc - tic);

		LOG(INFO) << "SOR filter done in [" << time_used.count() * 1000.0 << "] ms";

		return 1;
	}

	//regenerate the calibrated point cloud (the intrinsic angle might be uncorrect)
	bool vertical_intrinsic_calibration(typename pcl::PointCloud<PointT>::Ptr &cloud_in_out, double var_vertical_ang_d = 0.0, bool inverse_z = false) //inverse_z is for PANDAR XT Lidar
	{
		if (var_vertical_ang_d == 0)
			return false;

		if (var_vertical_ang_d >= 180.0) //this is a indicator for inversing z
			inverse_z = true;

		if (inverse_z)
		{
			for (int i = 0; i < cloud_in_out->points.size(); i++)
				cloud_in_out->points[i].z *= (-1.0);
			return false;
		}

		//typename pcl::PointCloud<PointT>::Ptr cloud_temp(new pcl::PointCloud<PointT>);
		double var_vertical_ang = var_vertical_ang_d / 180.0 * M_PI;

		// 	int i;
		// #pragma omp parallel for private(i) //Multi-thread
		for (int i = 0; i < cloud_in_out->points.size(); i++)
		{
			//LOG(INFO)<<cloud_in_out->points[i].x << ","<< cloud_in_out->points[i].y << ","<< cloud_in_out->points[i].z;

			double dist = std::sqrt(cloud_in_out->points[i].x * cloud_in_out->points[i].x +
									cloud_in_out->points[i].y * cloud_in_out->points[i].y +
									cloud_in_out->points[i].z * cloud_in_out->points[i].z);

			double v_ang = std::asin(cloud_in_out->points[i].z / dist);
			double v_ang_c = v_ang + var_vertical_ang;
			double hor_scale = std::cos(v_ang_c) / std::cos(v_ang);

			cloud_in_out->points[i].x *= hor_scale;
			cloud_in_out->points[i].y *= hor_scale;
			cloud_in_out->points[i].z = dist * std::sin(v_ang_c);

			//LOG(WARNING)<<cloud_in_out->points[i].x << ","<< cloud_in_out->points[i].y << ","<< cloud_in_out->points[i].z;
		}
		//cloud_temp->points.swap(cloud_in_out->points);

		return true;
	}

	//using uniform motion model to do motion compensation
	bool simple_motion_compensation(typename pcl::PointCloud<PointT>::Ptr &cloud_in_out,
									const Eigen::Matrix4d &estimated_tran2_1, // estimated_tran2_1 (from k to k+1 frame, the inverse of motion)
									double scan_begin_ang_anticlock_x_positive_deg = 180.0, double ambigous_angle_thre_deg = 1.0, bool only_rotation = false)
	//for velodyne HDL 64, the scan begin from the negative direction of x axis (scan_begin_ang_anticlock_x_positive_deg = 180), spining anti-clockwise
	//for HESAI Pandar 64, the scan begin from the negative direction of y axis (scan_begin_ang_anticlock_x_positive_deg = 270), spining anti-clockwise
	{
		typename pcl::PointCloud<PointT>::Ptr cloud_temp(new pcl::PointCloud<PointT>);
		double ang;
		double s; //interpolation ratio
		double scan_begin_ang_anticlock_x_positive_rad = scan_begin_ang_anticlock_x_positive_deg / 180.0 * M_PI;
		double ambigous_angle_thre_rad = ambigous_angle_thre_deg / 180.0 * M_PI;
		Eigen::Vector3d estimated_translation_2_1 = estimated_tran2_1.block<3, 1>(0, 3);
		Eigen::Quaterniond d_quat, estimated_quat2_1;
		Eigen::Vector3d d_translation;
		estimated_quat2_1 = Eigen::Quaterniond(estimated_tran2_1.block<3, 3>(0, 0));

		for (int i = 0; i < cloud_in_out->points.size(); i++)
		{
			ang = std::atan2(cloud_in_out->points[i].y, cloud_in_out->points[i].x);
			//atan2 (y,x) = atan(y/x) with x and y's sign--> [-pi , pi]
			//I:   x+ y+ --> [0,   pi/2]
			//II:  x- y+ --> [pi/2,  pi]
			//III: x- y- --> [-pi,-pi/2]
			//IV:  x+ y- --> [-pi/2,  0]

			if (ang < 0)
				ang += 2 * M_PI; // --> transform to the anti-clockwise angle from +x axis

			ang += scan_begin_ang_anticlock_x_positive_rad;
			if (ang >= 2 * M_PI)
				ang -= 2 * M_PI;
			// --> transform to the scan begining direction, now ang is the anti-clockwise angle from this begining direction

			//delete ambigous points (close to the begining or ending of a scan) --> only add those non-ambigious points
			if (ang > ambigous_angle_thre_rad && ang < 2 * M_PI - ambigous_angle_thre_rad)
			{
				s = (2 * M_PI - ang) / (2 * M_PI);
				//the first point (with earliest timestamp) would change the most
				//while the last point (with latest timestamp) would change the least
				d_quat = Eigen::Quaterniond::Identity().slerp(s, estimated_quat2_1); //Spherical linear interpolation (slerp) for quaternion
				d_translation = s * estimated_translation_2_1;
				Eigen::Vector3d temp_point(cloud_in_out->points[i].x, cloud_in_out->points[i].y, cloud_in_out->points[i].z);
				Eigen::Vector3d undistort_point;

				if (only_rotation)
					undistort_point = d_quat * temp_point;
				else
					undistort_point = d_quat * temp_point + d_translation;

				PointT pt;
				pt.x = undistort_point(0);
				pt.y = undistort_point(1);
				pt.z = undistort_point(2);
				pt.intensity = cloud_in_out->points[i].intensity;
				cloud_temp->points.push_back(pt);
			}
		}
		cloud_temp->points.swap(cloud_in_out->points);

		return 1;
	}

	//using uniform motion model to do motion compensation (with time stamp (store timestamp as curvature))
	bool simple_motion_compensation_with_timestamp(typename pcl::PointCloud<PointT>::Ptr &cloud_in_out,
												   const Eigen::Matrix4d &estimated_tran2_1,
												   float scan_duration_min_ms = 50) // estimated_tran2_1 (from k to k+1 frame, the inverse of motion)
	{
		typename pcl::PointCloud<PointT>::Ptr cloud_temp(new pcl::PointCloud<PointT>);
		double s; //interpolation ratio

		double last_timestamp = -DBL_MAX;
		double first_timestamp = DBL_MAX;
		double scan_duration;

		// get the first and last time stamp (stored as curvature) of this scan (time stamp unit: ms)
		for (int i = 0; i < cloud_in_out->points.size(); i++)
		{
			//LOG(INFO)<<cloud_in_out->points[i].curvature;
			last_timestamp = max_(last_timestamp, cloud_in_out->points[i].curvature);
			first_timestamp = min_(first_timestamp, cloud_in_out->points[i].curvature);
		}
		scan_duration = last_timestamp - first_timestamp;

		if (scan_duration < scan_duration_min_ms)
			return false;

		Eigen::Vector3d estimated_translation_2_1 = estimated_tran2_1.block<3, 1>(0, 3);
		Eigen::Quaterniond d_quat, estimated_quat2_1;
		Eigen::Vector3d d_translation;
		estimated_quat2_1 = Eigen::Quaterniond(estimated_tran2_1.block<3, 3>(0, 0));

		for (int i = 0; i < cloud_in_out->points.size(); i++)
		{
			s = (last_timestamp - cloud_in_out->points[i].curvature) / scan_duration; //curvature as time stamp //TODO: fix
			//the first point (with earliest timestamp) would change the most
			//while the last point (with latest timestamp) would change the least
			// if (i%10==0)
			// LOG(INFO) << s;

			d_quat = Eigen::Quaterniond::Identity().slerp(s, estimated_quat2_1); //Spherical linear interpolation (slerp) for quaternion
			d_translation = s * estimated_translation_2_1;
			Eigen::Vector3d temp_point(cloud_in_out->points[i].x, cloud_in_out->points[i].y, cloud_in_out->points[i].z);
			Eigen::Vector3d undistort_point = d_quat * temp_point + d_translation;
			PointT pt;
			pt.x = undistort_point(0);
			pt.y = undistort_point(1);
			pt.z = undistort_point(2);
			pt.intensity = cloud_in_out->points[i].intensity;
			pt.curvature = cloud_in_out->points[i].curvature;
			cloud_temp->points.push_back(pt);
		}
		cloud_temp->points.swap(cloud_in_out->points);

		LOG(INFO) << "Motion compensation done.\n";

		return true;
	}

	bool get_pts_timestamp_ratio_in_frame(typename pcl::PointCloud<PointT>::Ptr &cloud_in_out,
										  bool timestamp_availiable = true,
										  double scan_begin_ang_anticlock_x_positive_deg = 180.0, float scan_duration_ms = 100)
	{
		double s; //interpolation ratio
		double ang;
		double scan_begin_ang_anticlock_x_positive_rad = scan_begin_ang_anticlock_x_positive_deg / 180.0 * M_PI;

		double last_timestamp = -DBL_MAX;
		double first_timestamp = DBL_MAX;
		double actual_scan_duration;

		if (timestamp_availiable)
		{
			for (int i = 0; i < cloud_in_out->points.size(); i++)
			{
				last_timestamp = max_(last_timestamp, cloud_in_out->points[i].curvature); //ms as the unit
				first_timestamp = min_(first_timestamp, cloud_in_out->points[i].curvature);
			}
			actual_scan_duration = last_timestamp - first_timestamp;
			//LOG(INFO) << "scan duration:" << scan_duration;
			if (actual_scan_duration < scan_duration_ms * 0.75)
				scan_duration_ms = actual_scan_duration;

			for (int i = 0; i < cloud_in_out->points.size(); i++)
			{
				s = (last_timestamp - cloud_in_out->points[i].curvature) / scan_duration_ms; //curvature as time stamp
				cloud_in_out->points[i].curvature = min_(1.0, max_(0.0, s));				 //curvature as the time ratio in each frame
			}
			return true;
		}
		else //using azimuth to calculate timestamp ratio
		{
			for (int i = 0; i < cloud_in_out->points.size(); i++)
			{
				ang = std::atan2(cloud_in_out->points[i].y, cloud_in_out->points[i].x);
				//atan2 (x,y)  --> [-pi , pi]
				//I:   x+ y+ --> [0,   pi/2]
				//II:  x- y+ --> [pi/2,  pi]
				//III: x- y- --> [-pi,-pi/2]
				//IV:  x+ y- --> [-pi/2,  0]

				if (ang < 0)
					ang += 2 * M_PI; // --> transform to the anti-clockwise angle from +x axis [for HESAI, clockwise]

				ang += scan_begin_ang_anticlock_x_positive_rad;
				if (ang >= 2 * M_PI)
					ang -= 2 * M_PI;
				// --> transform to the scan begining direction, now ang is the anti-clockwise angle from this begining direction

				s = (2 * M_PI - ang) / (2 * M_PI);
				cloud_in_out->points[i].curvature = s;
			}
			return true;
		}
	}

	//already get the timestamp ratio (stored as curvature) using get_pts_timestamp_ratio_in_frame function
	void apply_motion_compensation(typename pcl::PointCloud<PointT>::Ptr pc_in_out, Eigen::Matrix4d &Tran, float s_ambigous_thre = 0.000)
	{
		Eigen::Vector3d estimated_translation_2_1 = Tran.block<3, 1>(0, 3);
		Eigen::Quaterniond d_quat, estimated_quat2_1;
		Eigen::Vector3d d_translation;
		estimated_quat2_1 = Eigen::Quaterniond(Tran.block<3, 3>(0, 0));

		omp_set_num_threads(min_(6, omp_get_max_threads()));
#pragma omp parallel for //Multi-thread
		for (int i = 0; i < pc_in_out->points.size(); i++)
		{
			if (pc_in_out->points[i].curvature < s_ambigous_thre || pc_in_out->points[i].curvature > 1.0 - s_ambigous_thre) //curvature as the timestamp
				continue;
			d_quat = Eigen::Quaterniond::Identity().slerp(pc_in_out->points[i].curvature, estimated_quat2_1); //Spherical linear interpolation (slerp) for quaternion
			d_translation = pc_in_out->points[i].curvature * estimated_translation_2_1;
			Eigen::Vector3d temp_point(pc_in_out->points[i].x, pc_in_out->points[i].y, pc_in_out->points[i].z);
			Eigen::Vector3d undistort_point = d_quat * temp_point + d_translation;
			pc_in_out->points[i].x = undistort_point(0);
			pc_in_out->points[i].y = undistort_point(1);
			pc_in_out->points[i].z = undistort_point(2);
		}
	}

	void apply_motion_compensation(const typename pcl::PointCloud<PointT>::Ptr pc_in, typename pcl::PointCloud<PointT>::Ptr pc_out,
								   Eigen::Matrix4d &Tran, float s_ambigous_thre = 0.0)
	{
		*pc_out = *pc_in;
		Eigen::Vector3d estimated_translation_2_1 = Tran.block<3, 1>(0, 3);
		Eigen::Quaterniond d_quat, estimated_quat2_1;
		Eigen::Vector3d d_translation;
		estimated_quat2_1 = Eigen::Quaterniond(Tran.block<3, 3>(0, 0));

		omp_set_num_threads(min_(6, omp_get_max_threads()));
#pragma omp parallel for //Multi-thread
		for (int i = 0; i < pc_in->points.size(); i++)
		{
			if (pc_in->points[i].curvature < s_ambigous_thre || pc_in->points[i].curvature > 1.0 - s_ambigous_thre) //curvature as the timestamp
				continue;
			d_quat = Eigen::Quaterniond::Identity().slerp(pc_in->points[i].curvature, estimated_quat2_1); //Spherical linear interpolation (slerp) for quaternion
			d_translation = pc_in->points[i].curvature * estimated_translation_2_1;
			Eigen::Vector3d temp_point(pc_in->points[i].x, pc_in->points[i].y, pc_in->points[i].z);
			Eigen::Vector3d undistort_point = d_quat * temp_point + d_translation;
			pc_out->points[i].x = undistort_point(0);
			pc_out->points[i].y = undistort_point(1);
			pc_out->points[i].z = undistort_point(2);
		}
	}

	//already get the timestamp ratio (stored as curvature) using get_pts_timestamp_ratio_in_frame function
	void batch_apply_motion_compensation(typename pcl::PointCloud<PointT>::Ptr pc_ground, typename pcl::PointCloud<PointT>::Ptr pc_pillar,
										 typename pcl::PointCloud<PointT>::Ptr pc_beam, typename pcl::PointCloud<PointT>::Ptr pc_facade,
										 typename pcl::PointCloud<PointT>::Ptr pc_roof, typename pcl::PointCloud<PointT>::Ptr pc_vertex,
										 Eigen::Matrix4d &Tran, bool undistort_keypoints_or_not = false)
	{
		apply_motion_compensation(pc_ground, Tran);
		apply_motion_compensation(pc_pillar, Tran);
		apply_motion_compensation(pc_beam, Tran);
		apply_motion_compensation(pc_facade, Tran);
		apply_motion_compensation(pc_roof, Tran);
		if (undistort_keypoints_or_not)
			apply_motion_compensation(pc_vertex, Tran);
	}

	//already get the timestamp ratio (stored as curvature) using get_pts_timestamp_ratio_in_frame function
	void batch_apply_motion_compensation(const typename pcl::PointCloud<PointT>::Ptr pc_ground, const typename pcl::PointCloud<PointT>::Ptr pc_pillar,
										 const typename pcl::PointCloud<PointT>::Ptr pc_beam, const typename pcl::PointCloud<PointT>::Ptr pc_facade,
										 const typename pcl::PointCloud<PointT>::Ptr pc_roof, const typename pcl::PointCloud<PointT>::Ptr pc_vertex,
										 typename pcl::PointCloud<PointT>::Ptr pc_ground_undistort, typename pcl::PointCloud<PointT>::Ptr pc_pillar_undistort,
										 typename pcl::PointCloud<PointT>::Ptr pc_beam_undistort, typename pcl::PointCloud<PointT>::Ptr pc_facade_undistort,
										 typename pcl::PointCloud<PointT>::Ptr pc_roof_undistort, typename pcl::PointCloud<PointT>::Ptr pc_vertex_undistort,
										 Eigen::Matrix4d &Tran, bool undistort_keypoints_or_not = false)
	{
		apply_motion_compensation(pc_ground, pc_ground_undistort, Tran);
		apply_motion_compensation(pc_pillar, pc_pillar_undistort, Tran);
		apply_motion_compensation(pc_beam, pc_beam_undistort, Tran);
		apply_motion_compensation(pc_facade, pc_facade_undistort, Tran);
		apply_motion_compensation(pc_roof, pc_roof_undistort, Tran);
		if (undistort_keypoints_or_not)
			apply_motion_compensation(pc_vertex, pc_vertex_undistort, Tran);
	}

	bool xy_normal_balanced_downsample(typename pcl::PointCloud<PointT>::Ptr &cloud_in_out,
									   int keep_number_per_sector, int sector_num)
	{
		if (cloud_in_out->points.size() <= keep_number_per_sector)
			return false;

		typename pcl::PointCloud<PointT>::Ptr cloud_temp(new pcl::PointCloud<PointT>);

		//divide into ${sector_num} sectors according to normal_x and normal_y
		std::vector<typename pcl::PointCloud<PointT>::Ptr> sectors(sector_num);

		//initialization
		for (int j = 0; j < sector_num; j++)
			sectors[j].reset(new pcl::PointCloud<PointT>());

		//LOG(INFO) << "initialization done\n";

		double angle_per_sector = 360.0 / sector_num;

		for (int i = 0; i < cloud_in_out->points.size(); i++)
		{
			double ang = std::atan2(cloud_in_out->points[i].normal_y, cloud_in_out->points[i].normal_x);
			//atan2 (x,y)  --> [-pi , pi]
			//I:   x+ y+ --> [0,   pi/2]
			//II:  x- y+ --> [pi/2,  pi]
			//III: x- y- --> [-pi,-pi/2]
			//IV:  x+ y- --> [-pi/2,  0]

			if (ang < 0)
				ang += 2 * M_PI; // --> transform to the anti-clockwise angle from +x axis

			ang *= (180.0 / M_PI);

			int sector_id = (int)(ang / angle_per_sector);
			sectors[sector_id]->points.push_back(cloud_in_out->points[i]); //push_back may cause collision for multi-thread processing
		}
		//LOG(INFO) << "assign sector done\n";

		for (int j = 0; j < sector_num; j++)
		{
			random_downsample_pcl(sectors[j], keep_number_per_sector);
			cloud_temp->points.insert(cloud_temp->points.end(), sectors[j]->points.begin(), sectors[j]->points.end());

			//LOG(INFO) << "sector " << j << " ok.";
		}

		cloud_temp->points.swap(cloud_in_out->points);

		std::vector<typename pcl::PointCloud<PointT>::Ptr>().swap(sectors);

		return true;
	}

	//fixed number random downsampling
	//when keep_number == 0, the output point cloud would be empty (in other words, the input point cloud would be cleared)
	bool random_downsample_pcl(typename pcl::PointCloud<PointT>::Ptr &cloud_in_out, int keep_number)
	{
		if (cloud_in_out->points.size() <= keep_number)
			return false;
		else
		{
			if (keep_number == 0)
			{
				cloud_in_out.reset(new typename pcl::PointCloud<PointT>());
				return false;
			}
			else
			{
				typename pcl::PointCloud<PointT>::Ptr cloud_temp(new pcl::PointCloud<PointT>);
				pcl::RandomSample<PointT> ran_sample(true); // Extract removed indices
				ran_sample.setInputCloud(cloud_in_out);
				ran_sample.setSample(keep_number);
				ran_sample.filter(*cloud_temp);
				cloud_temp->points.swap(cloud_in_out->points);
				return true;
			}
		}
	}

	//fixed number random downsampling
	//when keep_number == 0, the output point cloud would be empty
	bool random_downsample_pcl_with_height(typename pcl::PointCloud<PointT>::Ptr &cloud_in_out,
										   int keep_number, float non_downsample_height_thre)

	{
		std::chrono::steady_clock::time_point tic = std::chrono::steady_clock::now();

		if (cloud_in_out->points.size() <= keep_number)
			return false;
		else
		{
			if (keep_number == 0)
			{
				cloud_in_out.reset(new typename pcl::PointCloud<PointT>());
				return false;
			}
			else
			{
				typename pcl::PointCloud<PointT>::Ptr cloud_temp(new pcl::PointCloud<PointT>);
				typename pcl::PointCloud<PointT>::Ptr cloud_high(new pcl::PointCloud<PointT>);
				typename pcl::PointCloud<PointT>::Ptr cloud_low(new pcl::PointCloud<PointT>);

				int high_point_count = 0;
				for (int i = 0; i < cloud_in_out->points.size(); i++)
				{
					if (cloud_in_out->points[i].z > non_downsample_height_thre)
					{
						high_point_count++;
						cloud_high->points.push_back(cloud_in_out->points[i]);
					}
					else
						cloud_low->points.push_back(cloud_in_out->points[i]);
				}

				pcl::RandomSample<PointT> ran_sample(true); // Extract removed indices
				ran_sample.setInputCloud(cloud_low);
				ran_sample.setSample(max_(keep_number - high_point_count, 1));
				ran_sample.filter(*cloud_temp);
				cloud_temp->points.insert(cloud_temp->points.end(), cloud_high->points.begin(), cloud_high->points.end());
				cloud_temp->points.swap(cloud_in_out->points);

				cloud_high.reset(new pcl::PointCloud<PointT>());
				cloud_low.reset(new pcl::PointCloud<PointT>());

				std::chrono::steady_clock::time_point toc = std::chrono::steady_clock::now();
				std::chrono::duration<double> time_used = std::chrono::duration_cast<std::chrono::duration<double>>(toc - tic);
				//LOG(INFO) << "Random downsampling done in [" << time_used.count() * 1000.0 << "] ms.";
				return true;
			}
		}
	}

	//fixed number random downsampling
	//when keep_number == 0, the output point cloud would be empty
	bool random_downsample_pcl(typename pcl::PointCloud<PointT>::Ptr &cloud_in,
							   typename pcl::PointCloud<PointT>::Ptr &cloud_out, int keep_number)
	{
		std::chrono::steady_clock::time_point tic = std::chrono::steady_clock::now();

		if (cloud_in->points.size() <= keep_number)
		{
			cloud_out = cloud_in;
			return false;
		}
		else
		{
			if (keep_number == 0)
				return false;
			else
			{
				pcl::RandomSample<PointT> ran_sample(true); // Extract removed indices
				ran_sample.setInputCloud(cloud_in);
				ran_sample.setSample(keep_number);
				ran_sample.filter(*cloud_out);
				std::chrono::steady_clock::time_point toc = std::chrono::steady_clock::now();
				std::chrono::duration<double> time_used = std::chrono::duration_cast<std::chrono::duration<double>>(toc - tic);
				//LOG(INFO) << "Random downsampling done in [" << time_used.count() * 1000.0 << "] ms.";
				return true;
			}
		}
	}

	bool random_downsample(const typename pcl::PointCloud<PointT>::Ptr &cloud_in,
						   typename pcl::PointCloud<PointT>::Ptr &cloud_out, int downsample_ratio)
	{
		if (downsample_ratio > 1)
		{
			cloud_out->points.clear();
			for (int i = 0; i < cloud_in->points.size(); i++)
			{
				if (i % downsample_ratio == 0)
					cloud_out->points.push_back(cloud_in->points[i]);
			}
			return 1;
		}
		else
			return 0;
	}

	bool random_downsample(typename pcl::PointCloud<PointT>::Ptr &cloud_in_out, int downsample_ratio)
	{
		typename pcl::PointCloud<PointT>::Ptr cloud_temp(new pcl::PointCloud<PointT>);

		if (downsample_ratio > 1)
		{
			for (int i = 0; i < cloud_in_out->points.size(); i++)
			{
				if (i % downsample_ratio == 0)
					cloud_temp->points.push_back(cloud_in_out->points[i]);
			}
			cloud_temp->points.swap(cloud_in_out->points);
			//LOG(INFO)<<"rand_filter : " << cloud_in_out->points.size();
			return 1;
		}
		else
			return 0;
	}

	bool farthest_point_sampling(typename pcl::PointCloud<PointT>::Ptr &cloud_in_out, int downsample_ratio)
	{
		return 1;
		//TODO
	}

	bool intensity_filter(typename pcl::PointCloud<PointT>::Ptr &cloud_in_out,
						  float min_i_thre = 0, float max_i_thre = FLT_MAX, float intensity_scale = 255.0) //min_i_thre, max_i_thre are in the range [0,1]
	{
		std::chrono::steady_clock::time_point tic = std::chrono::steady_clock::now();

		int original_pts_num = cloud_in_out->points.size();
		typename pcl::PointCloud<PointT>::Ptr cloud_temp(new pcl::PointCloud<PointT>);
		for (int i = 0; i < cloud_in_out->points.size(); i++)
		{
			if (cloud_in_out->points[i].intensity > min_i_thre * intensity_scale && cloud_in_out->points[i].intensity < min_i_thre * intensity_scale)
				cloud_temp->points.push_back(cloud_in_out->points[i]);
		}

		std::chrono::steady_clock::time_point toc = std::chrono::steady_clock::now();
		std::chrono::duration<double> time_used = std::chrono::duration_cast<std::chrono::duration<double>>(toc - tic);

		// LOG(INFO) << "Intensity filtering [ " << original_pts_num << " --> "
		// 		  << cloud_in_out->points.size() << " ] done in [" << time_used.count() << "] s";

		return 1;
	}

	bool incidence_angle_filter(typename pcl::PointCloud<PointT>::Ptr &cloud_in_out,
								float min_ang_thre = 0, float max_ang_thre = 90)
	{
		std::chrono::steady_clock::time_point tic = std::chrono::steady_clock::now();

		int original_pts_num = cloud_in_out->points.size();
		float incidence_angle, dot_product_result;
		typename pcl::PointCloud<PointT>::Ptr cloud_temp(new pcl::PointCloud<PointT>);
		for (int i = 0; i < cloud_in_out->points.size(); i++)
		{
			dot_product_result = std::abs(cloud_in_out->points[i].x * cloud_in_out->points[i].normal_x + cloud_in_out->points[i].y * cloud_in_out->points[i].normal_y + cloud_in_out->points[i].z * cloud_in_out->points[i].normal_z);
			incidence_angle = std::acos(dot_product_result / (std::sqrt(cloud_in_out->points[i].x * cloud_in_out->points[i].x + cloud_in_out->points[i].y * cloud_in_out->points[i].y + cloud_in_out->points[i].z * cloud_in_out->points[i].z)));

			if (incidence_angle > min_ang_thre && incidence_angle < max_ang_thre)
				cloud_temp->points.push_back(cloud_in_out->points[i]);
		}

		cloud_temp->points.swap(cloud_in_out->points);

		std::chrono::steady_clock::time_point toc = std::chrono::steady_clock::now();
		std::chrono::duration<double> time_used = std::chrono::duration_cast<std::chrono::duration<double>>(toc - tic);

		// LOG(INFO) << "Incidence angle filtering [ " << original_pts_num << " --> "
		// 		  << cloud_in_out->points.size() << " ] done in [" << time_used.count() << "] s";

		return 1;
	}

	//Filter the point cloud according to the horizontal distance to the scanner (define dist_min and dist_max)
	bool dist_filter(typename pcl::PointCloud<PointT>::Ptr &cloud_in_out,
					 double xy_dist_min, double xy_dist_max)
	{
		std::chrono::steady_clock::time_point tic = std::chrono::steady_clock::now();

		double dis_square;
		int original_pts_num = cloud_in_out->points.size();
		typename pcl::PointCloud<PointT>::Ptr cloud_temp(new pcl::PointCloud<PointT>);

		for (int i = 0; i < cloud_in_out->points.size(); i++)
		{
			dis_square = cloud_in_out->points[i].x * cloud_in_out->points[i].x + cloud_in_out->points[i].y * cloud_in_out->points[i].y;

			if (dis_square < xy_dist_max * xy_dist_max && dis_square > xy_dist_min * xy_dist_min)
				cloud_temp->points.push_back(cloud_in_out->points[i]);
		}

		cloud_temp->points.swap(cloud_in_out->points);

		std::chrono::steady_clock::time_point toc = std::chrono::steady_clock::now();
		std::chrono::duration<double> time_used = std::chrono::duration_cast<std::chrono::duration<double>>(toc - tic);

		// LOG(INFO) << "Distance filter [ " << original_pts_num << " --> "
		// 		  << cloud_in_out->points.size() << " ] done in [" << time_used.count() << "] s";
		return 1;
	}

	//Filter the point cloud according to the horizontal and vertical distance to the lidar center
	bool dist_filter(typename pcl::PointCloud<PointT>::Ptr &cloud_in_out,
					 double xy_dis_thre, bool keep_inside = true, double z_min = -DBL_MAX, double z_max = DBL_MAX)
	{
		std::chrono::steady_clock::time_point tic = std::chrono::steady_clock::now();

		double dis_square;
		int original_pts_num = cloud_in_out->points.size();
		typename pcl::PointCloud<PointT>::Ptr cloud_temp(new pcl::PointCloud<PointT>);

		for (int i = 0; i < cloud_in_out->points.size(); i++)
		{
			dis_square = cloud_in_out->points[i].x * cloud_in_out->points[i].x + cloud_in_out->points[i].y * cloud_in_out->points[i].y;
			if (keep_inside)
			{
				if (dis_square < xy_dis_thre * xy_dis_thre && cloud_in_out->points[i].z < z_max && cloud_in_out->points[i].z > z_min)
				{
					cloud_temp->points.push_back(cloud_in_out->points[i]);
				}
			}
			else
			{
				if (dis_square > xy_dis_thre * xy_dis_thre && cloud_in_out->points[i].z < z_max && cloud_in_out->points[i].z > z_min)
				{
					cloud_temp->points.push_back(cloud_in_out->points[i]);
				}
			}
		}

		cloud_temp->points.swap(cloud_in_out->points);

		std::chrono::steady_clock::time_point toc = std::chrono::steady_clock::now();
		std::chrono::duration<double> time_used = std::chrono::duration_cast<std::chrono::duration<double>>(toc - tic);

		// LOG(INFO) << "Distance filter [ " << original_pts_num << " --> "
		// 		  << cloud_in_out->points.size() << " ] done in [" << time_used.count() << "] s";

		return 1;
	}

	//Filter the point cloud according to the horizontal and vertical distance to the lidar center
	bool dist_filter(typename pcl::PointCloud<PointT>::Ptr &cloud_in, typename pcl::PointCloud<PointT>::Ptr &cloud_out,
					 double xy_dis_thre, bool keep_inside = true, double z_min = -DBL_MAX, double z_max = DBL_MAX)
	{
		std::chrono::steady_clock::time_point tic = std::chrono::steady_clock::now();

		double dis_square;
		int original_pts_num = cloud_in->points.size();
		typename pcl::PointCloud<PointT>::Ptr cloud_temp(new pcl::PointCloud<PointT>);

		for (int i = 0; i < cloud_in->points.size(); i++)
		{
			dis_square = cloud_in->points[i].x * cloud_in->points[i].x + cloud_in->points[i].y * cloud_in->points[i].y;
			if (keep_inside)
			{
				if (dis_square < xy_dis_thre * xy_dis_thre && cloud_in->points[i].z < z_max && cloud_in->points[i].z > z_min)
				{
					cloud_out->points.push_back(cloud_in->points[i]);
				}
			}
			else
			{
				if (dis_square > xy_dis_thre * xy_dis_thre && cloud_in->points[i].z < z_max && cloud_in->points[i].z > z_min)
				{
					cloud_out->points.push_back(cloud_in->points[i]);
				}
			}
		}

		std::chrono::steady_clock::time_point toc = std::chrono::steady_clock::now();
		std::chrono::duration<double> time_used = std::chrono::duration_cast<std::chrono::duration<double>>(toc - tic);

		// LOG(INFO) << "Distance filter [ " << original_pts_num << " --> "
		// 		  << cloud_out->points.size() << " ] done in [" << time_used.count() << "] s";

		return 1;
	}

	//filter the ghost points under ground and the points on the ego-vehicle
	//ghost radius should be a bit larger than self radius
	//we assume that most of the ghost points are underground outliers within a radius from the laser scanner
	bool scanner_filter(const typename pcl::PointCloud<PointT>::Ptr &cloud_in_out,
						float self_radius, float ghost_radius, float z_min_thre_ghost, float z_min_thre_global)
	{
		typename pcl::PointCloud<PointT>::Ptr cloud_temp(new pcl::PointCloud<PointT>);
		for (int i = 0; i < cloud_in_out->points.size(); i++)
		{
			float dis_square = cloud_in_out->points[i].x * cloud_in_out->points[i].x + cloud_in_out->points[i].y * cloud_in_out->points[i].y;
			if (dis_square > self_radius * self_radius && cloud_in_out->points[i].z > z_min_thre_global)
			{
				if (dis_square > ghost_radius * ghost_radius || cloud_in_out->points[i].z > z_min_thre_ghost)
					cloud_temp->points.push_back(cloud_in_out->points[i]);
			}
		}
		cloud_temp->points.swap(cloud_in_out->points);
		return true;
	}

	bool bbx_filter(const typename pcl::PointCloud<PointT>::Ptr &cloud_in,
					typename pcl::PointCloud<PointT>::Ptr &cloud_out, bounds_t &bbx)
	{
		for (int i = 0; i < cloud_in->points.size(); i++)
		{
			//In the bounding box
			if (cloud_in->points[i].x > bbx.min_x && cloud_in->points[i].x < bbx.max_x &&
				cloud_in->points[i].y > bbx.min_y && cloud_in->points[i].y < bbx.max_y &&
				cloud_in->points[i].z > bbx.min_z && cloud_in->points[i].z < bbx.max_z)
			{
				cloud_out->points.push_back(cloud_in->points[i]);
			}
		}

		//LOG(INFO) << "# Points [" << cloud_in->points.size() << " -> bbx Filter -> " << cloud_out->points.size() << " ]";

		return 1;
	}

	bool bbx_filter(const typename pcl::PointCloud<PointT>::Ptr &cloud_in_out, bounds_t &bbx, bool delete_box = false)
	{
		std::chrono::steady_clock::time_point tic = std::chrono::steady_clock::now();
		typename pcl::PointCloud<PointT>::Ptr cloud_temp(new pcl::PointCloud<PointT>);
		int original_pts_num = cloud_in_out->points.size();
		for (int i = 0; i < cloud_in_out->points.size(); i++)
		{

			//In the bounding box
			if (cloud_in_out->points[i].x > bbx.min_x && cloud_in_out->points[i].x < bbx.max_x &&
				cloud_in_out->points[i].y > bbx.min_y && cloud_in_out->points[i].y < bbx.max_y &&
				cloud_in_out->points[i].z > bbx.min_z && cloud_in_out->points[i].z < bbx.max_z)
			{
				if (!delete_box)
					cloud_temp->points.push_back(cloud_in_out->points[i]);
			}
			else
			{
				if (delete_box)
					cloud_temp->points.push_back(cloud_in_out->points[i]);
			}
		}
		cloud_temp->points.swap(cloud_in_out->points);

		std::chrono::steady_clock::time_point toc = std::chrono::steady_clock::now();
		std::chrono::duration<double> time_used = std::chrono::duration_cast<std::chrono::duration<double>>(toc - tic);

		// LOG(INFO) << "Box filter [ " << original_pts_num << " --> "
		// 		  << cloud_in_out->points.size() << " ] done in [" << time_used.count() << "] s";

		return 1;
	}

	bool
	active_object_filter_by_bbx(const typename pcl::PointCloud<PointT>::Ptr &cloud_in,
								typename pcl::PointCloud<PointT>::Ptr &cloud_out,
								std::vector<bounds_t> &active_bbxs)
	{
		std::vector<bool> is_static(cloud_in->points.size(), 1);
		for (int i = 0; i < cloud_in->points.size(); i++)
		{
			for (int j = 0; j < active_bbxs.size(); j++)
			{
				//In the bounding box
				if (cloud_in->points[i].x > active_bbxs[j].min_x && cloud_in->points[i].x < active_bbxs[j].max_x &&
					cloud_in->points[i].y > active_bbxs[j].min_y && cloud_in->points[i].y < active_bbxs[j].max_y &&
					cloud_in->points[i].z > active_bbxs[j].min_z && cloud_in->points[i].z < active_bbxs[j].max_z)
				{
					is_static[i] = 0;
					break;
				}
			}
			if (is_static[i])
				cloud_out->points.push_back(cloud_in->points[i]);
		}

		std::vector<bool>().swap(is_static);

		return 1;
	}

	//Brief: Detect some stable key points (vertex points) from the point cloud according to the local curvature and stability
	bool detect_key_pts(const typename pcl::PointCloud<PointT>::Ptr &cloud_in, std::vector<pca_feature_t> &features,
						const std::vector<int> &index_with_feature,
						typename pcl::PointCloud<PointT>::Ptr &cloud_keypoints,
						double stable_ratio_max, float curvature_non_max_radius, float min_curvature)
	{
		std::chrono::steady_clock::time_point tic = std::chrono::steady_clock::now();

		int keypointNum = 0;

		pcl::PointIndicesPtr candidateIndices(new pcl::PointIndices());
		prune_unstable_pts(features, index_with_feature, stable_ratio_max, min_curvature, candidateIndices);

		std::vector<pca_feature_t> stableFeatures;
		for (int i = 0; i < candidateIndices->indices.size(); ++i)
		{
			stableFeatures.push_back(features[candidateIndices->indices[i]]);
		}

		if (stableFeatures.size() < 5)
			return false;

		pcl::PointIndicesPtr nonMaximaIndices(new pcl::PointIndices());
		non_max_suppress(stableFeatures, nonMaximaIndices, curvature_non_max_radius);

		// Create the filtering object
		typename pcl::ExtractIndices<PointT> extract;
		extract.setInputCloud(cloud_in);
		extract.setIndices(nonMaximaIndices);
		extract.setNegative(false);
		extract.filter(*cloud_keypoints);

		std::chrono::steady_clock::time_point toc = std::chrono::steady_clock::now();
		std::chrono::duration<double> time_used = std::chrono::duration_cast<std::chrono::duration<double>>(toc - tic);

		std::vector<pca_feature_t>().swap(stableFeatures);

		LOG(INFO) << "Vertex keypoints detection done in [" << 1000.0 * time_used.count() << "] ms.";
		//LOG(INFO) << "Keypoint detection done (" << cloud_keypoints->points.size() << " keypoints)";
		return true;
	}

	//Brief: Get rid of some unstable points from the keypoints list
	bool prune_unstable_pts(const std::vector<pca_feature_t> &features, const std::vector<int> &index_with_feature, float ratioMax, float min_curvature,
							pcl::PointIndicesPtr &stable_indices, int min_point_num_neighborhood = 10)

	{
		for (int i = 0; i < features.size(); ++i)
		{
			float ratio1, ratio2;
			ratio1 = features[i].values.lamada2 / features[i].values.lamada1;
			ratio2 = features[i].values.lamada3 / features[i].values.lamada2;
			if (ratio1 < ratioMax && ratio2 < ratioMax && features[i].pt_num > min_point_num_neighborhood && features[i].curvature > min_curvature)
			{
				stable_indices->indices.push_back(i);
			}
		}
		return true;
	}

	bool encode_stable_points(const typename pcl::PointCloud<PointT>::Ptr &cloud_in,
							  typename pcl::PointCloud<PointT>::Ptr &cloud_out,
							  const std::vector<pca_feature_t> &features,
							  const std::vector<int> &index_with_feature,
							  float min_curvature = 0.0,
							  int min_feature_point_num_neighborhood = 4,
							  int min_point_num_neighborhood = 8)
	//extract stable points and then encode point cloud neighborhood feature descriptor (ncc: neighborhood category context) at the same time
	{
		for (int i = 0; i < features.size(); ++i)
		{
			// float ratio1, ratio2;
			// ratio1 = features[i].values.lamada2 / features[i].values.lamada1;
			// ratio2 = features[i].values.lamada3 / features[i].values.lamada2;
			//if (ratio1 < stable_ratio_max && ratio2 < stable_ratio_max &&
			if(features[i].pt_num > min_point_num_neighborhood && features[i].curvature > min_curvature)
			{
				float accu_intensity = 0.0;
				PointT pt;
				pt = cloud_in->points[i];
				pt.normal[3] = features[i].curvature; //save in normal[3]

				int neighbor_total_count = 0, pillar_count = 0, beam_count = 0, facade_count = 0, roof_count = 0;
				int pillar_close_count = 0, pillar_far_count = 0, beam_close_count = 0, beam_far_count = 0, facade_close_count = 0, facade_far_count = 0, roof_close_count = 0, roof_far_count = 0;

				neighbor_total_count = features[i].neighbor_indices.size();

				for (int j = 0; j < neighbor_total_count; j++)
				{
					int temp_neighbor_index = features[i].neighbor_indices[j];
					switch (index_with_feature[temp_neighbor_index])
					{
					case 1:
					{
						pillar_count++;
						if (features[i].close_to_query_point[j])
							pillar_close_count++;
						else
							pillar_far_count++;
						break;
					}
					case 2:
					{
						beam_count++;
						if (features[i].close_to_query_point[j])
							beam_close_count++;
						else
							beam_far_count++;
						break;
					}
					case 3:
					{
						facade_count++;
						if (features[i].close_to_query_point[j])
							facade_close_count++;
						else
							facade_far_count++;
						break;
					}
					case 4:
					{
						roof_count++;
						if (features[i].close_to_query_point[j])
							roof_close_count++;
						else
							roof_far_count++;
						break;
					}
					default:
						break;
					}
					accu_intensity += cloud_in->points[temp_neighbor_index].intensity;
				}
				if (pillar_count + beam_count + facade_count + roof_count < min_feature_point_num_neighborhood)
					continue;

                //TODO: it's a very stupid way to doing so, change the feature encoding in code refactoring
				pillar_count = 100 * pillar_count / neighbor_total_count;
				beam_count = 100 * beam_count / neighbor_total_count;
				facade_count = 100 * facade_count / neighbor_total_count;
				roof_count = 100 * roof_count / neighbor_total_count;
				pillar_close_count = 100 * pillar_close_count / neighbor_total_count;
				beam_close_count = 100 * beam_close_count / neighbor_total_count;
				facade_close_count = 100 * facade_close_count / neighbor_total_count;
				roof_close_count = 100 * roof_close_count / neighbor_total_count;
				pillar_far_count = 100 * pillar_far_count / neighbor_total_count;
				beam_far_count = 100 * beam_far_count / neighbor_total_count;
				facade_far_count = 100 * facade_far_count / neighbor_total_count;
				roof_far_count = 100 * roof_far_count / neighbor_total_count;
                
				int descriptor = pillar_count * 1000000 + beam_count * 10000 + facade_count * 100 + roof_count; //the neighborhood discriptor (8 numbers)
				int descriptor_1 = pillar_close_count * 1000000 + beam_close_count * 10000 + facade_close_count * 100 + roof_close_count;
				int descriptor_2 = pillar_far_count * 1000000 + beam_far_count * 10000 + facade_far_count * 100 + roof_far_count;

				//TODO: fix later, keypoints would not be used in fine registration, so we do not need the timestamp (stored in curvature) and normal vector
				pt.curvature = descriptor;
				pt.normal[0] = descriptor_1;
				pt.normal[1] = descriptor_2;

				pt.intensity = accu_intensity / neighbor_total_count; //mean intensity of the nrighborhood
																	  //pt.normal[3] store the point curvature
																	  //pt.data[3] store the height of the point above the ground

				//!!! TODO: fix, use customed point type, you need a lot of porperties for saving linearity, planarity, curvature, semantic label and timestamp
				//!!! However, within the template class, there might be a lot of problems (waiting for the code reproducing)

				cloud_out->points.push_back(pt);
			}
		}
		return true;
	}

	bool non_max_suppress(typename pcl::PointCloud<PointT>::Ptr &cloud_in_out, float non_max_radius,
						  bool kd_tree_already_built = false, const typename pcl::search::KdTree<PointT>::Ptr &built_tree = NULL) //according to curvature
	{
		std::chrono::steady_clock::time_point tic = std::chrono::steady_clock::now();
		typename pcl::PointCloud<PointT>::Ptr cloud_temp(new pcl::PointCloud<PointT>());

		int pt_count_before = cloud_in_out->points.size();
		if (pt_count_before < 10)
			return false;

		std::sort(cloud_in_out->points.begin(), cloud_in_out->points.end(), [](const PointT &a, const PointT &b) { return a.normal[3] > b.normal[3]; });

		std::set<int, std::less<int>> unVisitedPtId;
		std::set<int, std::less<int>>::iterator iterUnseg;
		for (int i = 0; i < pt_count_before; ++i)
			unVisitedPtId.insert(i);

		//typename pcl::KdTreeFLANN<PointT>::Ptr tree(new pcl::KdTreeFLANN<PointT>());

		typename pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>());
		if (kd_tree_already_built)
			tree = built_tree;
		else
			tree->setInputCloud(cloud_in_out);

		std::vector<int> search_indices;
		std::vector<float> distances;
		int keypointnum = 0;
		do
		{
			keypointnum++;
			std::vector<int>().swap(search_indices);
			std::vector<float>().swap(distances);

			int id;
			iterUnseg = unVisitedPtId.begin();
			id = *iterUnseg;
			cloud_temp->points.push_back(cloud_in_out->points[id]);
			unVisitedPtId.erase(id);

			tree->radiusSearch(cloud_in_out->points[id], non_max_radius, search_indices, distances);

			for (int i = 0; i < search_indices.size(); i++)
				unVisitedPtId.erase(search_indices[i]);

		} while (!unVisitedPtId.empty());

		cloud_in_out->points.swap(cloud_temp->points);

		int pt_count_after_nms = cloud_in_out->points.size();

		std::chrono::steady_clock::time_point toc = std::chrono::steady_clock::now();
		std::chrono::duration<double> time_used = std::chrono::duration_cast<std::chrono::duration<double>>(toc - tic);

		LOG(INFO) << "NMS done from [" << pt_count_before << "] to [" << pt_count_after_nms << "] points in [" << 1000.0 * time_used.count() << "] ms.";

		return true;
	}

	//Brief: Use NMS to select those key points having locally maximal curvature
	bool non_max_suppress(typename pcl::PointCloud<PointT>::Ptr &cloud_in,
						  typename pcl::PointCloud<PointT>::Ptr &cloud_out, float nms_radius,
						  bool distance_adaptive_on = false, float unit_dist = 35.0,
						  bool kd_tree_already_built = false, const typename pcl::search::KdTree<PointT>::Ptr &built_tree = NULL) //according to curvature
	{
		std::chrono::steady_clock::time_point tic = std::chrono::steady_clock::now();
		typename pcl::PointCloud<PointT>::Ptr cloud_temp(new pcl::PointCloud<PointT>());

		int pt_count_before = cloud_in->points.size();
		if (pt_count_before < 10)
			return false;

		std::sort(cloud_in->points.begin(), cloud_in->points.end(), [](const PointT &a, const PointT &b) { return a.normal[3] > b.normal[3]; }); //using the unused normal[3] to represent what we want

		std::set<int, std::less<int>> unVisitedPtId;
		std::set<int, std::less<int>>::iterator iterUnseg;
		for (int i = 0; i < pt_count_before; ++i)
			unVisitedPtId.insert(i);

		typename pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>());
		if (kd_tree_already_built)
			tree = built_tree;
		else
			tree->setInputCloud(cloud_in);

		std::vector<int> search_indices;
		std::vector<float> distances;
		int keypointnum = 0;
		do
		{
			keypointnum++;
			std::vector<int>().swap(search_indices);
			std::vector<float>().swap(distances);

			int id;
			iterUnseg = unVisitedPtId.begin();
			id = *iterUnseg;
			cloud_out->points.push_back(cloud_in->points[id]);
			unVisitedPtId.erase(id);

			float non_max_radius = nms_radius;

			if (distance_adaptive_on)
			{
				double dist = std::sqrt(cloud_in->points[id].x * cloud_in->points[id].x +
										cloud_in->points[id].y * cloud_in->points[id].y +
										cloud_in->points[id].z * cloud_in->points[id].z);
				if (dist > unit_dist)
				{
					non_max_radius = std::sqrt(dist / unit_dist) * nms_radius;
					//neighborhood_k = (int)(unit_dist / dist * nearest_k));
				}
			}

			tree->radiusSearch(cloud_in->points[id], non_max_radius, search_indices, distances);

			for (int i = 0; i < search_indices.size(); i++)
				unVisitedPtId.erase(search_indices[i]);

		} while (!unVisitedPtId.empty());

		int pt_count_after_nms = cloud_out->points.size();

		std::chrono::steady_clock::time_point toc = std::chrono::steady_clock::now();
		std::chrono::duration<double> time_used = std::chrono::duration_cast<std::chrono::duration<double>>(toc - tic);

		//LOG(INFO) << "NMS done from [" << pt_count_before << "] to [" << pt_count_after_nms << "] points in [" << 1000.0 * time_used.count() << "] ms.";

		return true;
	}

	bool non_max_suppress(std::vector<pca_feature_t> &features, pcl::PointIndicesPtr &indices,
						  float nms_radius)
	{
		std::sort(features.begin(), features.end(), [](const pca_feature_t &a, const pca_feature_t &b) { return a.curvature > b.curvature; });

		pcl::PointCloud<pcl::PointNormal>::Ptr pointCloud(new pcl::PointCloud<pcl::PointNormal>());

		std::set<int, std::less<int>> unVisitedPtId;
		std::set<int, std::less<int>>::iterator iterUnseg;
		for (int i = 0; i < features.size(); ++i)
		{
			unVisitedPtId.insert(i);
			pointCloud->points.push_back(features[i].pt);
		}

		pcl::KdTreeFLANN<pcl::PointNormal> tree;
		tree.setInputCloud(pointCloud);

		std::vector<int> search_indices;
		std::vector<float> distances;

		int keypointNum = 0;
		do
		{
			keypointNum++;
			std::vector<int>().swap(search_indices);
			std::vector<float>().swap(distances);

			int id;
			iterUnseg = unVisitedPtId.begin();
			id = *iterUnseg;
			indices->indices.push_back(features[id].ptId);
			unVisitedPtId.erase(id);

			float curvature_non_max_radius = nms_radius;

			tree.radiusSearch(features[id].pt, curvature_non_max_radius, search_indices, distances);

			for (int i = 0; i < search_indices.size(); ++i)
			{
				unVisitedPtId.erase(search_indices[i]);
			}

		} while (!unVisitedPtId.empty());

		pointCloud.reset(new pcl::PointCloud<pcl::PointNormal>());

		return true;
	}

	int get_pts_ring_id(const PointT &pt) //for velodyne HDL 64
	{
		//Reference：https://github.com/luhongquan66/loam_velodyne/blob/master/src/lib/MultiScanRegistration.cpp
		double ver_ang;
		int ringID;
		ver_ang = atan(pt.z / sqrt(pt.x * pt.x + pt.y * pt.y));
		ringID = (int)(ver_ang * 134.18714161056457 + 58.81598513011153);
		if (ringID > 63)
			ringID = 63;
		else if (ringID < 0)
			ringID = 0;
		return ringID;
	}

	void get_pc_ring_ids(typename pcl::PointCloud<PointT>::Ptr &cloud)
	{
		for (int i = 0; i < cloud->points.size(); i++)
		{
			cloud->points[i].curvature = get_pts_ring_id(cloud->points[i]);
			//LOG(INFO) << i << "-ring-" << (int)(cloud->points[i].curvature);
		}
	}

	//detect curb point clouds (Deprecated)
	bool detect_curbs(const typename pcl::PointCloud<PointT>::Ptr &cloud_in, typename pcl::PointCloud<PointT>::Ptr &cloud_curb)
	{
		//assign cloud to rings
		int ring_number = 64;
		int ring_pts_count_thre = 200;
		int min_ring_num = 28;
		int max_ring_num = 48;

		std::vector<typename pcl::PointCloud<PointT>::Ptr> rings(ring_number);
		for (int i = 0; i < ring_number; i++)
			rings[i] = boost::make_shared<typename pcl::PointCloud<PointT>>();

		for (int i = 0; i < cloud_in->points.size(); i++)
			rings[(unsigned int)(cloud_in->points[i].curvature)]->points.push_back(cloud_in->points[i]); //curvature store the ring number here

		//cloud_curb = rings[40];

		for (int i = 0; i < ring_number; i++)
		{
			//LOG(INFO) << "Ring [" << i << "]: [" << rings[i]->points.size() << "] ground points.";
			if (rings[i]->points.size() > ring_pts_count_thre && i > min_ring_num && i < max_ring_num)
				detect_curbs_on_scanline(rings[i], cloud_curb); // append into cloud_curb
		}

		return true;
	}

	static bool compare_hor_ang(const PointT &pt_a, const PointT &pt_b) // from 0 to 360
	{
		return pt_a.curvature < pt_b.curvature; // now curvature is used to store the horizontal angle
	}

	//detect curb points on each scanline (Deprecated)
	bool detect_curbs_on_scanline(typename pcl::PointCloud<PointT>::Ptr &ring, typename pcl::PointCloud<PointT>::Ptr &cloud_curb,
								  float delta_z_thre = 0.03, float delta_z_thre_max = 0.08, float scan_begin_ang_anticlock_x_positive_deg = 180.0)
	{

		int ring_id = (int)(ring->points[0].curvature);

		//atan2 (x,y)  --> [-pi , pi]
		//I:   x+ y+ --> [0,   pi/2]
		//II:  x- y+ --> [pi/2,  pi]
		//III: x- y- --> [-pi,-pi/2]
		//IV:  x+ y- --> [-pi/2,  0]

		float hor_ang_deg;
		for (int i = 0; i < ring->points.size(); i++) // now curvature is used to store the horizontal angle
		{
			hor_ang_deg = std::atan2(ring->points[i].y, ring->points[i].x) * 180.0 / M_PI;
			if (hor_ang_deg < 0)
				hor_ang_deg += 360.0; // --> transform to the anti-clockwise angle from +x axis

			ring->points[i].curvature = hor_ang_deg;
		}

		//sort according to horizontal angle
		std::sort(ring->points.begin(), ring->points.end(), compare_hor_ang);

		typename pcl::PointCloud<PointT>::Ptr ring_curb_pts(new pcl::PointCloud<PointT>);

		//using sliding window
		int window_step = 8;
		int window_size = 12;
		float hor_ang_deg_diff_thre = 360.0 / 1800 * window_size * 1.5; //360/1800*window_size * 1.5
		int curb_flag = 0;
		int curb_count = 0;
		int curb_count_max = 8; //road crossing
		int i = 0;

		//some important parameters influence the final performance.
		float xy_thresh = 0.06;
		float z_thresh_min = 0.06;
		float z_thresh_max = 0.12;

		int points_num = ring->points.size();

		while ((i + window_size) < points_num)
		{
			float hor_ang_window = ring->points[i + window_size - 1].curvature - ring->points[i].curvature;
			bool is_spining_begin_end = (ring->points[i].curvature > (scan_begin_ang_anticlock_x_positive_deg - 2 * hor_ang_deg_diff_thre) &&
										 ring->points[i + window_size - 1].curvature < (scan_begin_ang_anticlock_x_positive_deg + 2 * hor_ang_deg_diff_thre));

			if (hor_ang_window > hor_ang_deg_diff_thre || is_spining_begin_end)
			{
				i += window_step;
				continue;
			}

			float z_max = -FLT_MAX;
			float z_min = FLT_MAX;

			int candidate_curb_idx = 0;
			float delta_z_max = 0;

			for (int j = 0; j < window_size; j++)
			{
				// float delta_z = std::fabs(ring->points[i + j].z - ring->points[i + j + 1].z);
				// if (delta_z > delta_z_max)
				// {
				// 	delta_z_max = delta_z;
				// 	candidate_curb_idx = i + j; //record candidate curb point in the window
				// }

				z_min = min_(ring->points[i + j].z, z_min);
				z_max = max_(ring->points[i + j].z, z_max);
			}

			float window_delta_z = std::fabs(z_max - z_min);

			//LOG(INFO) << "window [" << i << "] : " << window_delta_z;

			if (window_delta_z >= z_thresh_min && window_delta_z < z_thresh_max) //candidate curb window
			{
				//LOG(INFO) << "candidate window [" << i << "]";

				for (int j = 0; j < window_size - 1; j++)
				{
					//horizontal_distance between adjacent points
					float adjacent_hor_dist2 = (ring->points[i + j].y - ring->points[i + j + 1].y) * (ring->points[i + j].y - ring->points[i + j + 1].y) +
											   (ring->points[i + j].x - ring->points[i + j + 1].x) * (ring->points[i + j].x - ring->points[i + j + 1].x);
					if (adjacent_hor_dist2 >= xy_thresh * xy_thresh)
					{
						ring_curb_pts->points.push_back(ring->points[i + j]);
						curb_count++;
						i += 5 * window_step;
						break;
					}
				}
				// ring_curb_pts->points.push_back(ring->points[candidate_curb_idx]);
				// curb_count++;
			}

			if (curb_count > curb_count_max)
				return false;

			i += window_step;
		}

		for (int i = 0; i < points_num; i++) //store ring_id again
			ring->points[i].curvature = ring_id;

		//LOG(INFO) << "Ring [" << ring_id << "]: [" << curb_count << "] curb points.";

		cloud_curb->points.insert(cloud_curb->points.end(), ring_curb_pts->points.begin(), ring_curb_pts->points.end());

		return true;
	}

	//simple implement of bi-threshold ground segmentation without further processing and downsampling
	bool ground_seg(const typename pcl::PointCloud<PointT>::Ptr &cloud_in,
					typename pcl::PointCloud<PointT>::Ptr &cloud_ground,
					typename pcl::PointCloud<PointT>::Ptr &cloud_unground,
					int min_grid_pt_num, float grid_resolution, float max_height_difference, float neighbor_height_diff,
					float max_ground_height, float min_ground_height)
	{

		std::chrono::steady_clock::time_point tic = std::chrono::steady_clock::now();

		bounds_t bounds;
		centerpoint_t center_pt;
		this->get_cloud_bbx_cpt(cloud_in, bounds, center_pt); //Inherited from its parent class, use this->

		//Construct Grid
		int row, col, num_grid;
		row = ceil((bounds.max_y - bounds.min_y) / grid_resolution);
		col = ceil((bounds.max_x - bounds.min_x) / grid_resolution);
		num_grid = row * col;

		grid_t *grid = new grid_t[num_grid];

		//Each grid
		for (int i = 0; i < num_grid; i++)
		{
			grid[i].min_z = FLT_MAX;
			grid[i].neighbor_min_z = FLT_MAX;
		}

		//Each point ---> determine the grid to which the point belongs
		for (int j = 0; j < cloud_in->points.size(); j++)
		{
			int temp_row, temp_col, temp_id;
			temp_col = floor((cloud_in->points[j].x - bounds.min_x) / grid_resolution);
			temp_row = floor((cloud_in->points[j].y - bounds.min_y) / grid_resolution);
			temp_id = temp_row * col + temp_col;
			if (temp_id >= 0 && temp_id < num_grid)
			{
				grid[temp_id].pts_count++;
				if (cloud_in->points[j].z > max_ground_height)
					cloud_unground->points.push_back(cloud_in->points[j]);
				else
				{
					grid[temp_id].point_id.push_back(j);
					if (cloud_in->points[j].z < grid[temp_id].min_z && cloud_in->points[j].z > min_ground_height)
					{
						grid[temp_id].min_z = cloud_in->points[j].z;
						grid[temp_id].neighbor_min_z = cloud_in->points[j].z;
					}
				}
			}
		}

		//Each grid
		for (int m = 0; m < num_grid; m++)
		{
			int temp_row, temp_col;
			temp_row = m / col;
			temp_col = m % col;
			if (temp_row >= 1 && temp_row <= row - 2 && temp_col >= 1 && temp_col <= col - 2)
			{
				for (int j = -1; j <= 1; j++) //row
				{
					for (int k = -1; k <= 1; k++) //col
					{
						if (grid[m].neighbor_min_z > grid[m + j * col + k].min_z)
							grid[m].neighbor_min_z = grid[m + j * col + k].min_z;
					}
				}
			}
		}

		//For each grid
		for (int i = 0; i < num_grid; i++)
		{
			//Filtering some grids with too little points
			if (grid[i].pts_count >= min_grid_pt_num)
			{
				if (grid[i].min_z - grid[i].neighbor_min_z < neighbor_height_diff)
				{
					for (int j = 0; j < grid[i].point_id.size(); j++)
					{
						if (cloud_in->points[grid[i].point_id[j]].z - grid[i].min_z < max_height_difference &&
							cloud_in->points[grid[i].point_id[j]].z > min_ground_height)
							cloud_ground->points.push_back(cloud_in->points[grid[i].point_id[j]]); //Add to ground points
						else
							cloud_unground->points.push_back(cloud_in->points[grid[i].point_id[j]]); //Add to nonground points
					}
				}
				else
				{
					for (int j = 0; j < grid[i].point_id.size(); j++)
						cloud_unground->points.push_back(cloud_in->points[grid[i].point_id[j]]); //Add to nonground points
				}
			}
		}

		//free memory
		delete[] grid;

		std::chrono::steady_clock::time_point toc = std::chrono::steady_clock::now();

		LOG(INFO) << "Ground: [" << cloud_ground->points.size() << "] Unground: [" << cloud_unground->points.size() << "].";
		std::chrono::duration<double> ground_seg_time = std::chrono::duration_cast<std::chrono::duration<double>>(toc - tic);

		LOG(INFO) << "Ground segmentation done in [" << ground_seg_time.count() * 1000.0 << "] ms.";

		return 1;
	}

	// -------------------------------------------------------------------------------------------------------------------//
	// Two threshold Fast Ground filter
	// 1.Construct 2D grid
	// 2.Calculate the Minimum Z value in each grid
	// 3.For each grid, if its 8 neighbor grids' Minimum Z value is less than current grid's Minimum Z minus threshold1, then all the points in current grid would be seen as unground points
	// 4.Or, points whose Z value is larger than grid's Minimum Z plus threshold2 would be regarded as unground points. The rest points in the grid would be ground points.
	// (Estimate Ground Points' normal at the same time)
	// for multiple scan line mobile scanning data, we can detect the curb points at the same time according to ring information
	// ground_random_down_rate: 1 ground point in [ground_random_down_rate] ground points of each grid would be kept (for example, 20)
	// ground_random_down_down_rate: 1 ground point in [ground_random_down_down_rate] ground points of the already downsampled ground points would be kept (for example, 2)
	// Reference paper: Two-step adaptive extraction method for ground points and breaklines from lidar point clouds, ISPRS-J, Yang.B, Huang.R, et al.
	// -------------------------------------------------------------------------------------------------------------------//
	bool fast_ground_filter(const typename pcl::PointCloud<PointT>::Ptr &cloud_in,
							typename pcl::PointCloud<PointT>::Ptr &cloud_ground,
							typename pcl::PointCloud<PointT>::Ptr &cloud_ground_down,
							typename pcl::PointCloud<PointT>::Ptr &cloud_unground,
							typename pcl::PointCloud<PointT>::Ptr &cloud_curb,
							int min_grid_pt_num, float grid_resolution, float max_height_difference,
							float neighbor_height_diff, float max_ground_height,
							int ground_random_down_rate, int ground_random_down_down_rate, int nonground_random_down_rate, int reliable_neighbor_grid_num_thre,
							int estimate_ground_normal_method, float normal_estimation_radius, //estimate_ground_normal_method, 0: directly use (0,0,1), 1: estimate normal in fix radius neighborhood , 2: estimate normal in k nearest neighborhood, 3: use ransac to estimate plane coeffs in a grid
							int distance_weight_downsampling_method, float standard_distance,  //standard distance: the distance where the distance_weight is 1
							bool fixed_num_downsampling = false, int down_ground_fixed_num = 1000,
							bool detect_curb_or_not = false, float intensity_thre = FLT_MAX,
							bool apply_grid_wise_outlier_filter = false, float outlier_std_scale = 3.0) //current intensity_thre is for kitti dataset (TODO: disable it)
	{

		std::chrono::steady_clock::time_point tic = std::chrono::steady_clock::now();

		PrincipleComponentAnalysis<PointT> pca_estimator;

		typename pcl::PointCloud<PointT>::Ptr cloud_ground_full(new pcl::PointCloud<PointT>());

		int reliable_grid_pts_count_thre = min_grid_pt_num - 1;
		int count_checkpoint = 0;
		float sum_height = 0.001;
		float appro_mean_height;
		float min_ground_height = max_ground_height;
		float underground_noise_thre = -FLT_MAX;
		float non_ground_height_thre;
		float distance_weight;
		// int ground_random_down_rate_temp = ground_random_down_rate;
		// int nonground_random_down_rate_temp = nonground_random_down_rate;

		//For some points,  calculating the approximate mean height
		for (int j = 0; j < cloud_in->points.size(); j++)
		{
			if (j % 100 == 0)
			{
				sum_height += cloud_in->points[j].z;
				count_checkpoint++;
			}
		}
		appro_mean_height = sum_height / count_checkpoint;

		non_ground_height_thre = appro_mean_height + max_ground_height;
		//sometimes, there would be some underground ghost points (noise), however, these points would be removed by scanner filter
		//float underground_noise_thre = appro_mean_height - max_ground_height;  // this is a keyparameter.

		bounds_t bounds;
		centerpoint_t center_pt;
		this->get_cloud_bbx_cpt(cloud_in, bounds, center_pt); //Inherited from its parent class, use this->

		//Construct Grid
		int row, col, num_grid;
		row = ceil((bounds.max_y - bounds.min_y) / grid_resolution);
		col = ceil((bounds.max_x - bounds.min_x) / grid_resolution);
		num_grid = row * col;

		std::chrono::steady_clock::time_point toc_1_1 = std::chrono::steady_clock::now();

		grid_t *grid = new grid_t[num_grid];

		//Each grid
		for (int i = 0; i < num_grid; i++)
		{
			grid[i].min_z = FLT_MAX;
			grid[i].neighbor_min_z = FLT_MAX;
		}

		//Each point ---> determine the grid to which the point belongs
		for (int j = 0; j < cloud_in->points.size(); j++)
		{
			int temp_row, temp_col, temp_id;
			temp_col = floor((cloud_in->points[j].x - bounds.min_x) / grid_resolution);
			temp_row = floor((cloud_in->points[j].y - bounds.min_y) / grid_resolution);
			temp_id = temp_row * col + temp_col;
			if (temp_id >= 0 && temp_id < num_grid)
			{
				if (distance_weight_downsampling_method > 0 && !grid[temp_id].pts_count)
				{
					grid[temp_id].dist2station = std::sqrt(cloud_in->points[j].x * cloud_in->points[j].x + cloud_in->points[j].y * cloud_in->points[j].y + cloud_in->points[j].z * cloud_in->points[j].z);
				}

				if (cloud_in->points[j].z > non_ground_height_thre)
				{
					distance_weight = 1.0 * standard_distance / (grid[temp_id].dist2station + 0.0001); //avoiding Floating point exception
					int nonground_random_down_rate_temp = nonground_random_down_rate;
					if (distance_weight_downsampling_method == 1) //linear weight
						nonground_random_down_rate_temp = (int)(distance_weight * nonground_random_down_rate + 1);
					else if (distance_weight_downsampling_method == 2) //quadratic weight
						nonground_random_down_rate_temp = (int)(distance_weight * distance_weight * nonground_random_down_rate + 1);

					if (j % nonground_random_down_rate_temp == 0 || cloud_in->points[j].intensity > intensity_thre)
					{
						cloud_in->points[j].data[3] = cloud_in->points[j].z - (appro_mean_height - 3.0); //data[3] stores the approximate point height above ground
						cloud_unground->points.push_back(cloud_in->points[j]);
					}
				}
				else if (cloud_in->points[j].z > underground_noise_thre)
				{
					grid[temp_id].pts_count++;
					grid[temp_id].point_id.push_back(j);
					if (cloud_in->points[j].z < grid[temp_id].min_z) //
					{
						grid[temp_id].min_z = cloud_in->points[j].z;
						grid[temp_id].neighbor_min_z = cloud_in->points[j].z;
					}
				}
			}
		}
		std::chrono::steady_clock::time_point toc_1_2 = std::chrono::steady_clock::now();

		if (apply_grid_wise_outlier_filter)
		{
			//Each grid: Check outlier //calculate mean and standard deviation of z in one grid, then set mean-2*std as the threshold for outliers
			for (int i = 0; i < num_grid; i++)
			{
				if (grid[i].pts_count >= min_grid_pt_num)
				{
					double sum_z = 0, sum_z2 = 0, std_z = 0, mean_z = 0;
					for (int j = 0; j < grid[i].point_id.size(); j++)
						sum_z += cloud_in->points[grid[i].point_id[j]].z;
					mean_z = sum_z / grid[i].pts_count;
					for (int j = 0; j < grid[i].point_id.size(); j++)
						sum_z2 += (cloud_in->points[grid[i].point_id[j]].z - mean_z) * (cloud_in->points[grid[i].point_id[j]].z - mean_z);
					std_z = std::sqrt(sum_z2 / grid[i].pts_count);
					grid[i].min_z_outlier_thre = mean_z - outlier_std_scale * std_z;
					grid[i].min_z = max_(grid[i].min_z, grid[i].min_z_outlier_thre);
					grid[i].neighbor_min_z = grid[i].min_z;
				}
			}
		}

		std::chrono::steady_clock::time_point toc_1_3 = std::chrono::steady_clock::now();

		//Each grid
		for (int m = 0; m < num_grid; m++)
		{
			int temp_row, temp_col;
			temp_row = m / col;
			temp_col = m % col;
			if (temp_row >= 1 && temp_row <= row - 2 && temp_col >= 1 && temp_col <= col - 2)
			{
				for (int j = -1; j <= 1; j++) //row
				{
					for (int k = -1; k <= 1; k++) //col
					{
						grid[m].neighbor_min_z = min_(grid[m].neighbor_min_z, grid[m + j * col + k].min_z);
						if (grid[m + j * col + k].pts_count > reliable_grid_pts_count_thre)
							grid[m].reliable_neighbor_grid_num++;
					}
				}
			}
		}

		double consuming_time_ransac = 0.0;

		std::chrono::steady_clock::time_point toc_1_4 = std::chrono::steady_clock::now();

		std::vector<typename pcl::PointCloud<PointT>::Ptr> grid_ground_pcs(num_grid);
		std::vector<typename pcl::PointCloud<PointT>::Ptr> grid_unground_pcs(num_grid);
		for (int i = 0; i < num_grid; i++)
		{
			typename pcl::PointCloud<PointT>::Ptr grid_ground_pc_temp(new pcl::PointCloud<PointT>);
			grid_ground_pcs[i] = grid_ground_pc_temp;
			typename pcl::PointCloud<PointT>::Ptr grid_unground_pc_temp(new pcl::PointCloud<PointT>);
			grid_unground_pcs[i] = grid_unground_pc_temp;
		}

		std::chrono::steady_clock::time_point toc_1 = std::chrono::steady_clock::now();

		//For each grid
		omp_set_num_threads(min_(6, omp_get_max_threads()));
#pragma omp parallel for
		for (int i = 0; i < num_grid; i++)
		{
			typename pcl::PointCloud<PointT>::Ptr grid_ground(new pcl::PointCloud<PointT>);
			//Filtering some grids with too little points
			if (grid[i].pts_count >= min_grid_pt_num && grid[i].reliable_neighbor_grid_num >= reliable_neighbor_grid_num_thre)
			{
				int ground_random_down_rate_temp = ground_random_down_rate;
				int nonground_random_down_rate_temp = nonground_random_down_rate;
				distance_weight = 1.0 * standard_distance / (grid[i].dist2station + 0.0001);
				if (distance_weight_downsampling_method == 1) //linear weight
				{
					ground_random_down_rate_temp = (int)(distance_weight * ground_random_down_rate + 1);
					nonground_random_down_rate_temp = (int)(distance_weight * nonground_random_down_rate + 1);
				}
				else if (distance_weight_downsampling_method == 2) //quadratic weight
				{
					ground_random_down_rate_temp = (int)(distance_weight * distance_weight * ground_random_down_rate + 1);
					nonground_random_down_rate_temp = (int)(distance_weight * distance_weight * nonground_random_down_rate + 1);
				}
				//LOG(WARNING) << ground_random_down_rate_temp << "," << nonground_random_down_rate_temp;
				if (grid[i].min_z - grid[i].neighbor_min_z < neighbor_height_diff)
				{
					for (int j = 0; j < grid[i].point_id.size(); j++)
					{
						if (cloud_in->points[grid[i].point_id[j]].z > grid[i].min_z_outlier_thre)
						{
							if (cloud_in->points[grid[i].point_id[j]].z - grid[i].min_z < max_height_difference)
							{
								//cloud_ground_full->points.push_back(cloud_in->points[grid[i].point_id[j]]);
								if (estimate_ground_normal_method == 3)
									grid_ground->points.push_back(cloud_in->points[grid[i].point_id[j]]);
								else
								{
									if (j % ground_random_down_rate_temp == 0) // for example 10
									{
										if (estimate_ground_normal_method == 0)
										{
											cloud_in->points[grid[i].point_id[j]].normal_x = 0.0;
											cloud_in->points[grid[i].point_id[j]].normal_y = 0.0;
											cloud_in->points[grid[i].point_id[j]].normal_z = 1.0;
										}
										grid_ground_pcs[i]->points.push_back(cloud_in->points[grid[i].point_id[j]]);
										//cloud_ground->points.push_back(cloud_in->points[grid[i].point_id[j]]); //Add to ground points
									}
								}
							}
							else // inner grid unground points
							{
								if (j % nonground_random_down_rate_temp == 0 || cloud_in->points[grid[i].point_id[j]].intensity > intensity_thre) //extract more points on signs and vehicle license plate
								{
									cloud_in->points[grid[i].point_id[j]].data[3] = cloud_in->points[grid[i].point_id[j]].z - grid[i].min_z; //data[3] stores the point height above ground
									grid_unground_pcs[i]->points.push_back(cloud_in->points[grid[i].point_id[j]]);
									//cloud_unground->points.push_back(cloud_in->points[grid[i].point_id[j]]); //Add to nonground points
								}
							}
						}
					}
				}
				else //unground grid
				{
					for (int j = 0; j < grid[i].point_id.size(); j++)
					{
						if (cloud_in->points[grid[i].point_id[j]].z > grid[i].min_z_outlier_thre &&
							(j % nonground_random_down_rate_temp == 0 || cloud_in->points[grid[i].point_id[j]].intensity > intensity_thre))
						{
							cloud_in->points[grid[i].point_id[j]].data[3] = cloud_in->points[grid[i].point_id[j]].z - grid[i].neighbor_min_z; //data[3] stores the point height above ground
							grid_unground_pcs[i]->points.push_back(cloud_in->points[grid[i].point_id[j]]);
							//cloud_unground->points.push_back(cloud_in->points[grid[i].point_id[j]]); //Add to nonground points
						}
					}
				}
				if (estimate_ground_normal_method == 3 && grid_ground->points.size() >= min_grid_pt_num)
				{
					std::chrono::steady_clock::time_point tic_ransac = std::chrono::steady_clock::now();
					float normal_x, normal_y, normal_z;

					//RANSAC iteration number equation: p=1-(1-r^N)^M,
					//r is the inlier ratio (> 0.75 in our case), N is 3 in our case (3 points can fit a plane), to get a confidence > 0.99, we need about 20 iteration (M=20)
					estimate_ground_normal_by_ransac(grid_ground, 0.3 * max_height_difference, 20, normal_x, normal_y, normal_z);

					for (int j = 0; j < grid_ground->points.size(); j++)
					{
						if (j % ground_random_down_rate_temp == 0 && std::abs(normal_z) > 0.8) //53 deg
						{
							grid_ground->points[j].normal_x = normal_x;
							grid_ground->points[j].normal_y = normal_y;
							grid_ground->points[j].normal_z = normal_z;
							grid_ground_pcs[i]->points.push_back(grid_ground->points[j]); //Add to ground points
																						  //cloud_ground->points.push_back(grid_ground->points[j]); //Add to ground points
						}
					}
					std::chrono::steady_clock::time_point toc_ransac = std::chrono::steady_clock::now();
					std::chrono::duration<double> ground_ransac_time_per_grid = std::chrono::duration_cast<std::chrono::duration<double>>(toc_ransac - tic_ransac);
					consuming_time_ransac += ground_ransac_time_per_grid.count() * 1000.0; //unit: ms
				}
				pcl::PointCloud<PointT>().swap(*grid_ground);
			}
		}

		//combine the ground and unground points
		for (int i = 0; i < num_grid; i++)
		{
			cloud_ground->points.insert(cloud_ground->points.end(), grid_ground_pcs[i]->points.begin(), grid_ground_pcs[i]->points.end());
			cloud_unground->points.insert(cloud_unground->points.end(), grid_unground_pcs[i]->points.begin(), grid_unground_pcs[i]->points.end());
		}

		//free memory
		delete[] grid;

		std::chrono::steady_clock::time_point toc_2 = std::chrono::steady_clock::now();

		int normal_estimation_neighbor_k = 2 * min_grid_pt_num;
		pcl::PointCloud<pcl::Normal>::Ptr ground_normal(new pcl::PointCloud<pcl::Normal>);
		if (estimate_ground_normal_method == 1)
			pca_estimator.get_normal_pcar(cloud_ground, normal_estimation_radius, ground_normal);
		else if (estimate_ground_normal_method == 2)
			pca_estimator.get_normal_pcak(cloud_ground, normal_estimation_neighbor_k, ground_normal);

		for (int i = 0; i < cloud_ground->points.size(); i++)
		{
			if (estimate_ground_normal_method == 1 || estimate_ground_normal_method == 2)
			{
				cloud_ground->points[i].normal_x = ground_normal->points[i].normal_x;
				cloud_ground->points[i].normal_y = ground_normal->points[i].normal_y;
				cloud_ground->points[i].normal_z = ground_normal->points[i].normal_z;
			}
			if (!fixed_num_downsampling)
			{
				//LOG(INFO)<<cloud_ground->points[i].normal_x << "," << cloud_ground->points[i].normal_y << "," << cloud_ground->points[i].normal_z;
				if (i % ground_random_down_down_rate == 0)
					cloud_ground_down->points.push_back(cloud_ground->points[i]);
			}
		}

		if (fixed_num_downsampling)
			random_downsample_pcl(cloud_ground, cloud_ground_down, down_ground_fixed_num);

		pcl::PointCloud<pcl::Normal>().swap(*ground_normal);

		std::chrono::steady_clock::time_point toc_3 = std::chrono::steady_clock::now();
		std::chrono::duration<double> ground_seg_time = std::chrono::duration_cast<std::chrono::duration<double>>(toc_2 - tic);
		std::chrono::duration<double> ground_seg_prepare_time = std::chrono::duration_cast<std::chrono::duration<double>>(toc_1 - tic);
		std::chrono::duration<double> ground_normal_time = std::chrono::duration_cast<std::chrono::duration<double>>(toc_3 - toc_2);

		// std::chrono::duration<double> prepare_1 = std::chrono::duration_cast<std::chrono::duration<double>>(toc_1_1 - tic);
		// std::chrono::duration<double> prepare_2 = std::chrono::duration_cast<std::chrono::duration<double>>(toc_1_2 - toc_1_1);
		// std::chrono::duration<double> prepare_3 = std::chrono::duration_cast<std::chrono::duration<double>>(toc_1_3 - toc_1_2);
		// std::chrono::duration<double> prepare_4 = std::chrono::duration_cast<std::chrono::duration<double>>(toc_1_4 - toc_1_3);
		// std::chrono::duration<double> prepare_5 = std::chrono::duration_cast<std::chrono::duration<double>>(toc_1 - toc_1_4);

		LOG(INFO) << "Ground: [" << cloud_ground->points.size() << " | " << cloud_ground_down->points.size() << "] Unground: [" << cloud_unground->points.size() << "].";

		if (estimate_ground_normal_method == 3)
		{
			// LOG(INFO) << "Ground segmentation done in [" << ground_seg_time.count() * 1000.0 - consuming_time_ransac << "] ms.";
			// LOG(INFO) << "Ground Normal Estimation done in [" << consuming_time_ransac << "] ms.";
			LOG(INFO) << "Ground segmentation and normal estimation in [" << ground_seg_time.count() * 1000.0 << "] ms."
					  << ",in which preparation costs [" << ground_seg_prepare_time.count() * 1000.0 << "] ms.";
			//output detailed consuming time
			//LOG(INFO) << prepare_1.count() * 1000.0 << "," << prepare_2.count() * 1000.0 << "," << prepare_3.count() * 1000.0 << "," << prepare_4.count() * 1000.0 << "," << prepare_5.count() * 1000.0;
		}
		else
		{
			LOG(INFO) << "Ground segmentation done in [" << ground_seg_time.count() * 1000.0 << "] ms.";
			LOG(INFO) << "Ground Normal Estimation done in [" << ground_normal_time.count() * 1000.0 << "] ms."
					  << " preparation in [" << ground_seg_prepare_time.count() * 1000.0 << "] ms.";
		}
#if 0 //curb detection (deprecated)
			if (detect_curb_or_not)
			{
				//detect curb points
				std::vector<pca_feature_t> curb_pca_features;
				typename pcl::PointCloud<PointT>::Ptr cloud_curb_candidate(new pcl::PointCloud<PointT>());
				float pca_radius_curb = normal_estimation_radius;
				int pca_k_curb = normal_estimation_neighbor_k;
				int pca_min_pt_num = 4;
				float curb_linearity_thre = 0.7;
				float direction_z_max = 0.1;

				detect_curbs(cloud_ground_full, cloud_curb_candidate);

				pca_estimator.get_pc_pca_feature(cloud_curb_candidate, curb_pca_features, pca_radius_curb, pca_k_curb);
				for (int i = 0; i < cloud_curb_candidate->points.size(); i++)
				{
					if (curb_pca_features[i].pt_num >= pca_min_pt_num &&
						curb_pca_features[i].linear_2 > curb_linearity_thre &&
						std::abs(curb_pca_features[i].vectors.principalDirection.z()) < direction_z_max)
					{
						pca_estimator.assign_normal(cloud_curb_candidate->points[i], curb_pca_features[i], false); //assign primary direction vector
						cloud_curb->points.push_back(cloud_curb_candidate->points[i]);
					}
				}

				pcl::PointCloud<PointT>().swap(*cloud_curb_candidate);
				std::vector<pca_feature_t>().swap(curb_pca_features);

				std::chrono::steady_clock::time_point toc_3 = std::chrono::steady_clock::now();

				std::chrono::duration<double> curb_time = std::chrono::duration_cast<std::chrono::duration<double>>(toc_3 - toc_2);

				LOG(INFO) << "[" << cloud_curb->points.size() << "] curb points detected in [" << curb_time.count() * 1000.0 << "] ms.";
			}
#endif
		// pcl::PointCloud<PointT>().swap(*cloud_ground_full);
		return 1;
	}

	bool estimate_ground_normal_by_ransac(typename pcl::PointCloud<PointT>::Ptr &grid_ground,
										  float dist_thre, int max_iter, float &nx, float &ny, float &nz)
	{
		CProceesing<PointT> cpro;

		typename pcl::PointCloud<PointT>::Ptr grid_ground_fit(new pcl::PointCloud<PointT>);
		pcl::ModelCoefficients::Ptr grid_coeff(new pcl::ModelCoefficients);
		cpro.plane_seg_ransac(grid_ground, dist_thre, max_iter, grid_ground_fit, grid_coeff);

		grid_ground.swap(grid_ground_fit);
		nx = grid_coeff->values[0];
		ny = grid_coeff->values[1];
		nz = grid_coeff->values[2];

		//LOG(INFO) << nx << "," << ny << "," << nz;
		return 1;
	}

	//Brief: Classfiy the downsampled non-ground points into several types (Pillar, Beam, Facade, Roof, Vertex)
	//according to the pca features (combination of eigen values and eigen vectors)
	bool classify_nground_pts(typename pcl::PointCloud<PointT>::Ptr &cloud_in,
							  typename pcl::PointCloud<PointT>::Ptr &cloud_pillar,
							  typename pcl::PointCloud<PointT>::Ptr &cloud_beam,
							  typename pcl::PointCloud<PointT>::Ptr &cloud_facade,
							  typename pcl::PointCloud<PointT>::Ptr &cloud_roof,
							  typename pcl::PointCloud<PointT>::Ptr &cloud_pillar_down,
							  typename pcl::PointCloud<PointT>::Ptr &cloud_beam_down,
							  typename pcl::PointCloud<PointT>::Ptr &cloud_facade_down,
							  typename pcl::PointCloud<PointT>::Ptr &cloud_roof_down,
							  typename pcl::PointCloud<PointT>::Ptr &cloud_vertex,
							  float neighbor_searching_radius, int neighbor_k, int neigh_k_min, int pca_down_rate, // one in ${pca_down_rate} unground points would be select as the query points for calculating pca, the else would only be used as neighborhood points
							  float edge_thre, float planar_thre, float edge_thre_down, float planar_thre_down,
							  int extract_vertex_points_method, float curvature_thre, float vertex_curvature_non_max_radius,
							  float linear_vertical_sin_high_thre, float linear_vertical_sin_low_thre,
							  float planar_vertical_sin_high_thre, float planar_vertical_sin_low_thre,
							  bool fixed_num_downsampling = false, int pillar_down_fixed_num = 200, int facade_down_fixed_num = 800, int beam_down_fixed_num = 200,
							  int roof_down_fixed_num = 100, int unground_down_fixed_num = 20000,
							  float beam_height_max = FLT_MAX, float roof_height_min = -FLT_MAX,
							  float feature_pts_ratio_guess = 0.3, bool sharpen_with_nms = true,
							  bool use_distance_adaptive_pca = false)

	{

		std::chrono::steady_clock::time_point tic = std::chrono::steady_clock::now();

		if (fixed_num_downsampling)
			random_downsample_pcl(cloud_in, unground_down_fixed_num);

		//Do PCA
		PrincipleComponentAnalysis<PointT> pca_estimator;
		std::vector<pca_feature_t> cloud_features;

		typename pcl::KdTreeFLANN<PointT>::Ptr tree(new pcl::KdTreeFLANN<PointT>);
		tree->setInputCloud(cloud_in);

		float unit_distance = 30.0;
		pca_estimator.get_pc_pca_feature(cloud_in, cloud_features, tree, neighbor_searching_radius, neighbor_k, 1, pca_down_rate, use_distance_adaptive_pca, unit_distance);
		//LOG(WARNING)<< "PCA done";

		std::chrono::steady_clock::time_point toc_pca = std::chrono::steady_clock::now();

		//the radius should be larger for far away points
		std::vector<int> index_with_feature(cloud_in->points.size(), 0); // 0 - not special points, 1 - pillar, 2 - beam, 3 - facade, 4 - roof

		for (int i = 0; i < cloud_in->points.size(); i++)
		{
			if (cloud_features[i].pt_num > neigh_k_min)
			{
				if (cloud_features[i].linear_2 > edge_thre)
				{
					if (std::abs(cloud_features[i].vectors.principalDirection.z()) > linear_vertical_sin_high_thre)
					{
						pca_estimator.assign_normal(cloud_in->points[i], cloud_features[i], false);
						cloud_pillar->points.push_back(cloud_in->points[i]);
						index_with_feature[i] = 1;
					}
					else if (std::abs(cloud_features[i].vectors.principalDirection.z()) < linear_vertical_sin_low_thre && cloud_in->points[i].z < beam_height_max)
					{
						pca_estimator.assign_normal(cloud_in->points[i], cloud_features[i], false);
						cloud_beam->points.push_back(cloud_in->points[i]);
						index_with_feature[i] = 2;
					}
					else
					{
						;
					}

					if (!sharpen_with_nms && cloud_features[i].linear_2 > edge_thre_down)
					{
						if (std::abs(cloud_features[i].vectors.principalDirection.z()) > linear_vertical_sin_high_thre)
							cloud_pillar_down->points.push_back(cloud_in->points[i]);
						else if (std::abs(cloud_features[i].vectors.principalDirection.z()) < linear_vertical_sin_low_thre && cloud_in->points[i].z < beam_height_max)
							cloud_beam_down->points.push_back(cloud_in->points[i]);
						else
						{
							;
						}
					}
				}

				else if (cloud_features[i].planar_2 > planar_thre)
				{
					if (std::abs(cloud_features[i].vectors.normalDirection.z()) > planar_vertical_sin_high_thre && cloud_in->points[i].z > roof_height_min)
					{
						pca_estimator.assign_normal(cloud_in->points[i], cloud_features[i], true);
						cloud_roof->points.push_back(cloud_in->points[i]);
						index_with_feature[i] = 4;
					}
					else if (std::abs(cloud_features[i].vectors.normalDirection.z()) < planar_vertical_sin_low_thre)
					{
						pca_estimator.assign_normal(cloud_in->points[i], cloud_features[i], true);
						cloud_facade->points.push_back(cloud_in->points[i]);
						index_with_feature[i] = 3;
					}
					else
					{
						;
					}
					if (!sharpen_with_nms && cloud_features[i].planar_2 > planar_thre_down)
					{
						if (std::abs(cloud_features[i].vectors.normalDirection.z()) > planar_vertical_sin_high_thre && cloud_in->points[i].z > roof_height_min)
							cloud_roof_down->points.push_back(cloud_in->points[i]);
						else if (std::abs(cloud_features[i].vectors.normalDirection.z()) < planar_vertical_sin_low_thre)
							cloud_facade_down->points.push_back(cloud_in->points[i]);
						else
						{
							;
						}
					}
				}
			}
		}

		//According to the parameter 'extract_vertex_points_method' (0,1,2...)
		if (curvature_thre < 1e-8) // set stablilty_thre as 0 to disable the vertex extraction
			extract_vertex_points_method = 0;

		//Find Edge points by picking high curvature points among the neighborhood of unground geometric feature points (2)
		if (extract_vertex_points_method == 2)
		{
			float vertex_feature_ratio_thre = feature_pts_ratio_guess / pca_down_rate;
			for (int i = 0; i < cloud_in->points.size(); i++)
			{
				// if (index_with_feature[i] == 0)
				// 	cloud_vertex->points.push_back(cloud_in->points[i]);

				if (index_with_feature[i] == 0 && cloud_features[i].pt_num > neigh_k_min && cloud_features[i].curvature > curvature_thre) //curvature_thre means curvature_thre here
				{
					int geo_feature_point_count = 0;
					for (int j = 0; j < cloud_features[i].neighbor_indices.size(); j++)
					{
						if (index_with_feature[cloud_features[i].neighbor_indices[j]])
							geo_feature_point_count++;
					}
					//LOG(INFO)<< "facade neighbor num: " <<geo_feature_point_count;
					if (1.0 * geo_feature_point_count / cloud_features[i].pt_num > vertex_feature_ratio_thre) //most of the neighbors are feature points
					{
						//cloud_vertex->points.push_back(cloud_in->points[i]);

						pca_estimator.assign_normal(cloud_in->points[i], cloud_features[i], false);
						cloud_in->points[i].normal[3] = 5.0 * cloud_features[i].curvature; //save in the un-used normal[3]  (PointNormal4D)
						if (std::abs(cloud_features[i].vectors.principalDirection.z()) > linear_vertical_sin_high_thre)
						{
							cloud_pillar->points.push_back(cloud_in->points[i]);
							//cloud_pillar_down->points.push_back(cloud_in->points[i]);
							index_with_feature[i] = 1;
						}
						else if (std::abs(cloud_features[i].vectors.principalDirection.z()) < linear_vertical_sin_low_thre && cloud_in->points[i].z < beam_height_max)
						{
							cloud_beam->points.push_back(cloud_in->points[i]);
							//cloud_beam_down->points.push_back(cloud_in->points[i]);
							index_with_feature[i] = 2;
						}
					}
				}
			}
		}

		//if extract_vertex_points_method == 0 ---> do not extract vertex points (0)
		std::chrono::steady_clock::time_point toc_1 = std::chrono::steady_clock::now();
		//extract neighborhood feature descriptor for pillar points
		//Find Vertex (Edge) points by picking points with maximum local curvature (1)
		//if (extract_vertex_points_method == 1) //Deprecated
		//detect_key_pts(cloud_in, cloud_features, index_with_feature,cloud_vertex, 4.0 * curvature_thre, vertex_curvature_non_max_radius, 0.5 * curvature_thre);
		int min_neighbor_feature_pts = (int)(feature_pts_ratio_guess / pca_down_rate * neighbor_k) - 1;

		//get the vertex keypoints and encode its neighborhood in a simple descriptor
		encode_stable_points(cloud_in, cloud_vertex, cloud_features, index_with_feature,
							 0.3 * curvature_thre, min_neighbor_feature_pts, neigh_k_min); //encode the keypoints, we will get a simple descriptor of the putable keypoints

		//LOG(WARNING)<< "encode ncc feature descriptor done";

		std::chrono::steady_clock::time_point toc_2 = std::chrono::steady_clock::now();

		//Non_max_suppression of the feature points //TODO: add already built-kd tree here
		if (sharpen_with_nms)
		{
			float nms_radius = 0.25 * neighbor_searching_radius;
#pragma omp parallel sections
			{
#pragma omp section
				{
					if (pillar_down_fixed_num > 0)
						non_max_suppress(cloud_pillar, cloud_pillar_down, nms_radius);
				}
#pragma omp section
				{
					if (facade_down_fixed_num > 0)
						non_max_suppress(cloud_facade, cloud_facade_down, nms_radius);
				}
#pragma omp section
				{
					if (beam_down_fixed_num > 0)
						non_max_suppress(cloud_beam, cloud_beam_down, nms_radius);

					if (roof_down_fixed_num > 0)
						non_max_suppress(cloud_roof, cloud_roof_down, nms_radius);
				}
			}
		}

		std::chrono::steady_clock::time_point toc_3 = std::chrono::steady_clock::now();

		if (fixed_num_downsampling)
		{
			random_downsample_pcl(cloud_pillar_down, pillar_down_fixed_num);
			int sector_num = 4;
			xy_normal_balanced_downsample(cloud_facade_down, (int)(facade_down_fixed_num / sector_num), sector_num);

			xy_normal_balanced_downsample(cloud_beam_down, (int)(beam_down_fixed_num / sector_num), sector_num); // here the normal is the primary vector
																												 //random_downsample_pcl(cloud_roof_down, 100);
			random_downsample_pcl(cloud_roof_down, roof_down_fixed_num);
		}

		std::chrono::steady_clock::time_point toc_4 = std::chrono::steady_clock::now();
		std::chrono::duration<double> time_used_pca = std::chrono::duration_cast<std::chrono::duration<double>>(toc_pca - tic);
		std::chrono::duration<double> time_used_extract_geo_pts = std::chrono::duration_cast<std::chrono::duration<double>>(toc_1 - toc_pca);
		std::chrono::duration<double> time_used_encoding_key_pts = std::chrono::duration_cast<std::chrono::duration<double>>(toc_2 - toc_1);
		std::chrono::duration<double> time_used_nms_sharping = std::chrono::duration_cast<std::chrono::duration<double>>(toc_3 - toc_2);
		std::chrono::duration<double> time_used_fixed_num_downsampling = std::chrono::duration_cast<std::chrono::duration<double>>(toc_4 - toc_3);
		std::chrono::duration<double> time_used_all = std::chrono::duration_cast<std::chrono::duration<double>>(toc_4 - tic);

		//Free the memory
		std::vector<pca_feature_t>().swap(cloud_features);
		std::vector<int>().swap(index_with_feature);

		LOG(INFO) << "Unground geometric feature points extracted done in [" << time_used_all.count() * 1000.0 << "] ms.";
		LOG(INFO) << "Details: pca in [" << time_used_pca.count() * 1000.0 << "] ms, geometric feature points extracted in [" << time_used_extract_geo_pts.count() * 1000.0 << "] ms, encoding keypoints in [" << time_used_encoding_key_pts.count() * 1000.0 << "] ms, nms sharpen in [" << time_used_nms_sharping.count() * 1000.0 << "] ms, downsampling in [" << time_used_fixed_num_downsampling.count() * 1000.0 << "] ms.";
		LOG(INFO) << "Pillar: [" << cloud_pillar->points.size() << " | " << cloud_pillar_down->points.size() << "] Beam: [" << cloud_beam->points.size() << " | " << cloud_beam_down->points.size() << "] Facade: [" << cloud_facade->points.size() << " | " << cloud_facade_down->points.size() << "] Roof: [" << cloud_roof->points.size() << " | "
				  << cloud_roof_down->points.size() << "] Vertex: [" << cloud_vertex->points.size() << "].";

		return 1;
	}

	//Used in lidar odometry test
	//main entrance to geometric feature points extraction module
	//TODO: clear deprecated parameters
	bool extract_semantic_pts(cloudblock_Ptr in_block,
							  float vf_downsample_resolution, float gf_grid_resolution,
							  float gf_max_grid_height_diff, float gf_neighbor_height_diff, float gf_max_ground_height,
							  int &gf_down_rate_ground, int &gf_downsample_rate_nonground,
							  float pca_neighbor_radius, int pca_neighbor_k,
							  float edge_thre, float planar_thre, float curvature_thre,
							  float edge_thre_down, float planar_thre_down, bool use_distance_adaptive_pca = false,
							  int distance_inverse_sampling_method = 0, //distance_inverse_downsample, 0: disabled, 1: linear weight, 2: quadratic weight
							  float standard_distance = 15.0,			//the distance where the weight is 1, only useful when distance_inverse_downsample is on
							  int estimate_ground_normal_method = 3,	//estimate_ground_normal_method, 0: directly use (0,0,1), 1: estimate normal in fix radius neighborhood , 2: estimate normal in k nearest neighborhood, 3: use ransac to estimate plane coeffs in a grid
							  float normal_estimation_radius = 2.0,		//only when enabled when estimate_ground_normal_method = 1
							  bool use_adpative_parameters = false, bool apply_scanner_filter = false, bool extract_curb_or_not = false,
							  int extract_vertex_points_method = 2, //use the maximum curvature based keypoints
							  int gf_grid_pt_num_thre = 8, int gf_reliable_neighbor_grid_thre = 0,
							  int gf_down_down_rate_ground = 2, int pca_neighbor_k_min = 8, int pca_down_rate = 1,
							  float intensity_thre = FLT_MAX,														 //default intensity_thre means that highly-reflective objects would not be prefered
							  float linear_vertical_sin_high_thre = 0.94, float linear_vertical_sin_low_thre = 0.17, //70 degree (pillar), 10 degree (beam)
							  float planar_vertical_sin_high_thre = 0.98, float planar_vertical_sin_low_thre = 0.34, //80 degree (roof), 20 degree (facade)
							  bool sharpen_with_nms_on = true, bool fixed_num_downsampling = false, int ground_down_fixed_num = 500,
							  int pillar_down_fixed_num = 200, int facade_down_fixed_num = 800, int beam_down_fixed_num = 200,
							  int roof_down_fixed_num = 200, int unground_down_fixed_num = 20000, float beam_height_max = FLT_MAX, float roof_height_min = 0.0,
							  float approx_scanner_height = 2.0, float underground_thre = -7.0, float feature_pts_ratio_guess = 0.3,
							  bool semantic_assisted = false, bool apply_roi_filtering = false, float roi_min_y = 0.0, float roi_max_y = 0.0)
	{
		if (use_adpative_parameters)
			LOG(INFO) << "update parameters"
					  << "\ngf_down_rate_ground: " << gf_down_rate_ground
					  << "\ngf_downsample_rate_nonground: " << gf_downsample_rate_nonground
					  << "\npca_neighbor_k: " << pca_neighbor_k
					  << "\nedge_thre (down): " << edge_thre << " ( " << edge_thre_down << " )"
					  << "\nplanar_thre (down): " << planar_thre << " ( " << planar_thre_down << " )";

		//pre-processing

		//with semantic mask
		//remove dynamic objects and outliers (underground ghost points) using the semantic mask predicted by neural network
		if (semantic_assisted)
			filter_with_dynamic_object_mask_pre(in_block->pc_raw);

		else if (apply_scanner_filter)
		{
			float self_ring_radius = 1.75;
			float ghost_radius = 20.0;
			float z_min = -approx_scanner_height - 4.0;
			float z_min_min = -approx_scanner_height + underground_thre;
			// filter the point cloud of the back of the vehicle itself and the underground
			scanner_filter(in_block->pc_raw, self_ring_radius, ghost_radius, z_min, z_min_min);
			//dist_filter(in_block->pc_raw, self_ring_radius, false, z_min); // filter the point cloud of the back of the vehicle itself
		}

		//get_pc_ring_ids(in_block->pc_raw);

		voxel_downsample(in_block->pc_raw, in_block->pc_down, vf_downsample_resolution);

		random_downsample(in_block->pc_down, in_block->pc_sketch, in_block->pc_down->points.size() / 1024 + 1);

		//Used for UAV borne Lidar SLAM
		//rotate to world coordiante system (because we want to use some geometric constraints that would only apply on world coordinate system)
		//typename pcl::PointCloud<PointT>::Ptr rotated_pc_down(new pcl::PointCloud<PointT>);
		//pcl::transformPointCloud(*in_block->pc_down, *rotated_pc_down, in_block->pose_lo); //now the pose_lo stored only the rotation matrix of last frame

		//filtered point cloud --> ground & non-ground point cloud
		fast_ground_filter(in_block->pc_down, in_block->pc_ground, in_block->pc_ground_down, in_block->pc_unground, in_block->pc_vertex,
						   gf_grid_pt_num_thre, gf_grid_resolution, gf_max_grid_height_diff, gf_neighbor_height_diff,
						   gf_max_ground_height, gf_down_rate_ground, gf_down_down_rate_ground,
						   gf_downsample_rate_nonground, gf_reliable_neighbor_grid_thre, estimate_ground_normal_method, normal_estimation_radius,
						   distance_inverse_sampling_method, standard_distance, fixed_num_downsampling, ground_down_fixed_num, extract_curb_or_not,
						   intensity_thre, apply_scanner_filter);

		float vertex_curvature_non_max_r = 1.5 * pca_neighbor_radius;

# if 0  //ROI_filtering (Deprecated)
		if (apply_roi_filtering) 
		{
			bounds_t roi;
			roi.inf_z();
			roi.inf_x(); //moving dirction: x
			roi.min_y = roi_min_y;
			roi.max_y = roi_max_y;
			bbx_filter(in_block->pc_unground, roi, true);
		}
#endif

		//non-ground points --> planar (facade, roof) & linear (pillar, beam) & spherical (vertex) points
		classify_nground_pts(in_block->pc_unground, in_block->pc_pillar, in_block->pc_beam,
							 in_block->pc_facade, in_block->pc_roof,
							 in_block->pc_pillar_down, in_block->pc_beam_down, in_block->pc_facade_down,
							 in_block->pc_roof_down, in_block->pc_vertex,
							 pca_neighbor_radius, pca_neighbor_k, pca_neighbor_k_min, pca_down_rate,
							 edge_thre, planar_thre, edge_thre_down, planar_thre_down,
							 extract_vertex_points_method, curvature_thre, vertex_curvature_non_max_r,
							 linear_vertical_sin_high_thre, linear_vertical_sin_low_thre,
							 planar_vertical_sin_high_thre, planar_vertical_sin_low_thre,
							 fixed_num_downsampling, pillar_down_fixed_num, facade_down_fixed_num,
							 beam_down_fixed_num, roof_down_fixed_num, unground_down_fixed_num,
							 beam_height_max, roof_height_min, feature_pts_ratio_guess,
							 sharpen_with_nms_on, use_distance_adaptive_pca);

		//with semantic mask
		//using semantic mask predicted by neural network to refine the detected geometric feature points
		if (semantic_assisted) //Deprecated
			filter_with_semantic_mask(in_block); //currently disabled '000000'

		//transform the feature points back to the scanner's coordinate system
		//in_block->transform_feature(in_block->pose_lo.inverse(), true);

		in_block->down_feature_point_num = in_block->pc_ground_down->points.size() + in_block->pc_pillar_down->points.size() + in_block->pc_beam_down->points.size() +
										   in_block->pc_facade_down->points.size() + in_block->pc_roof_down->points.size() + in_block->pc_vertex->points.size();

		//update the parameters according to the situation
		if (use_adpative_parameters)
			update_parameters_self_adaptive(gf_down_rate_ground, gf_downsample_rate_nonground, pca_neighbor_radius,
											edge_thre, planar_thre, edge_thre_down, planar_thre_down,
											in_block->pc_ground_down->points.size(), in_block->pc_facade_down->points.size(),
											in_block->pc_pillar_down->points.size(), in_block->pc_beam_down->points.size());

		//pcl::PointCloud<PointT>().swap(*rotated_pc_down);
    return true;
	}

	//hard-coded (not a good way) to adjust the parameters on fly
	void update_parameters_self_adaptive(int &gf_down_rate_ground, int &gf_downsample_rate_nonground,
										 float &pca_neighbor_k,
										 float &edge_thre, float &planar_thre,
										 float &edge_thre_down, float &planar_thre_down,
										 int ground_down_num, int facade_down_num, int pillar_down_num, int beam_down_num,
										 int ground_down_num_min_expected = 500, int ground_down_num_max_expected = 1200,
										 int non_ground_num_min_expected = 200, int non_ground_num_max_expected = 1600,
										 int facade_down_num_expected = 400, int pillar_down_num_expected = 200, int beam_down_num_expected = 300)
	{
		//int non_ground_feature_num = facade_down_num + beam_down_num + pillar_down_num;
		int non_ground_feature_num = facade_down_num + pillar_down_num;

		// if (ground_down_num > ground_down_num_max_expected)
		// 	gf_down_rate_ground += ground_down_num / ground_down_num_max_expected;
		// else if (ground_down_num < ground_down_num_min_expected)
		// 	gf_down_rate_ground = max_(1, gf_down_rate_ground - ground_down_num_min_expected / ground_down_num);

		// if (non_ground_feature_num > non_ground_num_max_expected)
		// {
		// 	gf_downsample_rate_nonground = min_(6, gf_downsample_rate_nonground + non_ground_feature_num / non_ground_num_max_expected);
		// 	//pca_neighbor_k = min_(8, pca_neighbor_k - 1);
		// }

		if (non_ground_feature_num < non_ground_num_min_expected)
		{
			gf_downsample_rate_nonground = max_(1, gf_downsample_rate_nonground - non_ground_num_min_expected / non_ground_feature_num);
			//pca_neighbor_k = max_(20, pca_neighbor_k + 1);
		}
	}

	//Only works on Semantic KITTI dataset (Deprecated)
	//http://semantic-kitti.org/dataset.html
	bool extract_semantic_pts_from_mask(cloudblock_Ptr in_block, float vf_downsample_resolution,
										int ground_down_down_rate, int facade_down_down_rate,
										int pillar_down_down_rate, int beam_down_down_rate)
	{
		voxel_downsample(in_block->pc_raw, in_block->pc_down, vf_downsample_resolution);

		//feature points and the corresponding label
		//refer to semantic_kitti.yaml
		//ground: 40 (road), 44 (parking), 48 (sidewalk), 49 (other-ground), 60 (lane-marking), 72 (terrian)
		//facade: 50 (building), 51 (fence), 13 (bus)
		//pillar: 71 (trunk), 80 (pole), 81 (traffic-sign)
		//beam: 51 (fence)

		for (int i = 0; i < in_block->pc_down->points.size(); i++)
		{
			int label = (int)(in_block->pc_down->points[i].curvature);
			if (label == 40 || label == 44 || label == 48 || label == 49 || label == 72)
				in_block->pc_ground->push_back(in_block->pc_down->points[i]);
			else if (label == 50 || label == 13 || label == 51 || label == 10)
				in_block->pc_facade->push_back(in_block->pc_down->points[i]);
			else if (label == 71 || label == 80)
				in_block->pc_pillar->push_back(in_block->pc_down->points[i]);
			// else if (label == 51)
			// 	in_block->pc_beam->push_back(in_block->pc_down->points[i]);

			// if (i % 100 == 0)
			// 	std::cout << "example: point 10 (" << in_block->pc_down->points[i].normal_x << "," << in_block->pc_down->points[i].normal_y << "," << in_block->pc_down->points[i].normal_z << ").\n";
		}

		random_downsample(in_block->pc_ground, in_block->pc_ground_down, ground_down_down_rate);
		random_downsample(in_block->pc_facade, in_block->pc_facade_down, facade_down_down_rate);
		random_downsample(in_block->pc_pillar, in_block->pc_pillar_down, pillar_down_down_rate);
		//random_downsample(in_block->pc_beam, in_block->pc_beam_down, beam_down_down_rate);
		return 1;
	}

	//Only works on Semantic KITTI dataset (Deprecated)
	//http://semantic-kitti.org/dataset.html
	//actually, points that are farther than 50 m from the scanner would not be labeled. This would cause a lot of problem
	bool filter_with_dynamic_object_mask_pre(const typename pcl::PointCloud<PointT>::Ptr &cloud_in_out)
	{
		typename pcl::PointCloud<PointT>::Ptr cloud_temp(new pcl::PointCloud<PointT>);

		for (int i = 0; i < cloud_in_out->points.size(); i++)
		{
			if (cloud_in_out->points[i].curvature < 250 && cloud_in_out->points[i].curvature != 1) //block moving objects and outliers by semantic semantic_kitti masks
				cloud_temp->points.push_back(cloud_in_out->points[i]);
			// 	 252: "moving-car"  //   253: "moving-bicyclist"  //   254: "moving-person"
			//   255: "moving-motorcyclist" //   256: "moving-on-rails" //   257: "moving-bus"
			//   258: "moving-truck" //   259: "moving-other-vehicle"
			//    0 : "unlabeled" // 1 : "outlier"
		}

		cloud_temp->points.swap(cloud_in_out->points);

		return true;
	}

	//Only works on Semantic KITTI dataset (Deprecated)
	//http://semantic-kitti.org/dataset.html
	void filter_with_semantic_mask(cloudblock_Ptr in_block, const std::string mask_feature_type = "000000")
	{
		float labeled_radius = 45.0;

		if (mask_feature_type[0] == '1') //ground
		{
			typename pcl::PointCloud<PointT>::Ptr cloud_temp(new pcl::PointCloud<PointT>);
			for (int i = 0; i < in_block->pc_ground->points.size(); i++)
			{
				int label = (int)(in_block->pc_ground->points[i].curvature);
				float dist2 = in_block->pc_ground->points[i].x * in_block->pc_ground->points[i].x + in_block->pc_ground->points[i].y * in_block->pc_ground->points[i].y;
				if (label == 40 || label == 44 || label == 48 || label == 49 || label == 72 || dist2 > labeled_radius * labeled_radius)
					cloud_temp->points.push_back(in_block->pc_ground->points[i]);
			}
			cloud_temp->points.swap(in_block->pc_ground->points);

			typename pcl::PointCloud<PointT>::Ptr cloud_temp2(new pcl::PointCloud<PointT>);
			for (int i = 0; i < in_block->pc_ground_down->points.size(); i++)
			{
				int label = (int)(in_block->pc_ground_down->points[i].curvature);
				float dist2 = in_block->pc_ground_down->points[i].x * in_block->pc_ground_down->points[i].x + in_block->pc_ground_down->points[i].y * in_block->pc_ground_down->points[i].y;
				if (label == 40 || label == 44 || label == 48 || label == 49 || label == 72 || dist2 > labeled_radius * labeled_radius)
					cloud_temp2->points.push_back(in_block->pc_ground_down->points[i]);
			}
			cloud_temp2->points.swap(in_block->pc_ground_down->points);
		}

		if (mask_feature_type[2] == '1') //facade
		{
			typename pcl::PointCloud<PointT>::Ptr cloud_temp3(new pcl::PointCloud<PointT>);
			for (int i = 0; i < in_block->pc_facade->points.size(); i++)
			{
				int label = (int)(in_block->pc_facade->points[i].curvature);
				float dist2 = in_block->pc_facade->points[i].x * in_block->pc_facade->points[i].x + in_block->pc_facade->points[i].y * in_block->pc_facade->points[i].y;
				if (label == 50 || label == 13 || label == 51 || dist2 > labeled_radius * labeled_radius)
					cloud_temp3->points.push_back(in_block->pc_facade->points[i]);
			}
			cloud_temp3->points.swap(in_block->pc_facade->points);

			typename pcl::PointCloud<PointT>::Ptr cloud_temp4(new pcl::PointCloud<PointT>);
			for (int i = 0; i < in_block->pc_facade_down->points.size(); i++)
			{
				int label = (int)(in_block->pc_facade_down->points[i].curvature);
				float dist2 = in_block->pc_facade_down->points[i].x * in_block->pc_facade_down->points[i].x + in_block->pc_facade_down->points[i].y * in_block->pc_facade_down->points[i].y;
				if (label == 50 || label == 13 || label == 51 || dist2 > labeled_radius * labeled_radius)
					cloud_temp4->points.push_back(in_block->pc_facade_down->points[i]);
			}
			cloud_temp4->points.swap(in_block->pc_facade_down->points);
		}

		if (mask_feature_type[1] == '1') //pillar
		{
			typename pcl::PointCloud<PointT>::Ptr cloud_temp5(new pcl::PointCloud<PointT>);
			for (int i = 0; i < in_block->pc_pillar->points.size(); i++)
			{
				int label = (int)(in_block->pc_pillar->points[i].curvature);
				float dist2 = in_block->pc_pillar->points[i].x * in_block->pc_pillar->points[i].x + in_block->pc_pillar->points[i].y * in_block->pc_pillar->points[i].y;
				if (label == 71 || label == 80 || label == 81 || dist2 > labeled_radius * labeled_radius)
					cloud_temp5->points.push_back(in_block->pc_pillar->points[i]);
			}
			cloud_temp5->points.swap(in_block->pc_pillar->points);

			typename pcl::PointCloud<PointT>::Ptr cloud_temp6(new pcl::PointCloud<PointT>);
			for (int i = 0; i < in_block->pc_pillar_down->points.size(); i++)
			{
				int label = (int)(in_block->pc_pillar_down->points[i].curvature);
				float dist2 = in_block->pc_pillar_down->points[i].x * in_block->pc_pillar_down->points[i].x + in_block->pc_pillar_down->points[i].y * in_block->pc_pillar_down->points[i].y;
				if (label == 71 || label == 80 || label == 81 || dist2 > labeled_radius * labeled_radius)
					cloud_temp6->points.push_back(in_block->pc_pillar_down->points[i]);
			}
			cloud_temp6->points.swap(in_block->pc_pillar_down->points);
		}

		if (mask_feature_type[3] == '1') //beam
		{
			typename pcl::PointCloud<PointT>::Ptr cloud_temp7(new pcl::PointCloud<PointT>);
			for (int i = 0; i < in_block->pc_beam->points.size(); i++)
			{
				int label = (int)(in_block->pc_beam->points[i].curvature);
				float dist2 = in_block->pc_beam->points[i].x * in_block->pc_beam->points[i].x + in_block->pc_beam->points[i].y * in_block->pc_beam->points[i].y;
				if (label == 50 || label == 51 || label == 10)
					cloud_temp7->points.push_back(in_block->pc_beam->points[i]);
			}
			cloud_temp7->points.swap(in_block->pc_beam->points);

			typename pcl::PointCloud<PointT>::Ptr cloud_temp8(new pcl::PointCloud<PointT>);
			for (int i = 0; i < in_block->pc_beam_down->points.size(); i++)
			{
				int label = (int)(in_block->pc_beam_down->points[i].curvature);
				float dist2 = in_block->pc_beam_down->points[i].x * in_block->pc_beam_down->points[i].x + in_block->pc_beam_down->points[i].y * in_block->pc_beam_down->points[i].y;
				if (label == 50 || label == 51 || label == 10)
					cloud_temp8->points.push_back(in_block->pc_beam_down->points[i]);
			}
			cloud_temp8->points.swap(in_block->pc_beam_down->points);
		}

		LOG(INFO) << "Feature point number after semantic mask filtering: "
				  << "Ground: [" << in_block->pc_ground->points.size() << " | " << in_block->pc_ground_down->points.size()
				  << "] Pillar: [" << in_block->pc_pillar->points.size() << " | " << in_block->pc_pillar_down->points.size()
				  << "] Facade: [" << in_block->pc_facade->points.size() << " | " << in_block->pc_facade_down->points.size()
				  << "] Beam: [" << in_block->pc_beam->points.size() << " | " << in_block->pc_beam_down->points.size() << "]";
	}

	//Bridef: Used for the preprocessing of fine registration
	//Use the intersection bounding box to filter the outlier points (--> to speed up)
	bool get_cloud_pair_intersection(bounds_t &intersection_bbx,
									 typename pcl::PointCloud<PointT>::Ptr &pc_ground_tc,
									 typename pcl::PointCloud<PointT>::Ptr &pc_pillar_tc,
									 typename pcl::PointCloud<PointT>::Ptr &pc_beam_tc,
									 typename pcl::PointCloud<PointT>::Ptr &pc_facade_tc,
									 typename pcl::PointCloud<PointT>::Ptr &pc_roof_tc,
									 typename pcl::PointCloud<PointT>::Ptr &pc_vertex_tc,
									 typename pcl::PointCloud<PointT>::Ptr &pc_ground_sc,
									 typename pcl::PointCloud<PointT>::Ptr &pc_pillar_sc,
									 typename pcl::PointCloud<PointT>::Ptr &pc_beam_sc,
									 typename pcl::PointCloud<PointT>::Ptr &pc_facade_sc,
									 typename pcl::PointCloud<PointT>::Ptr &pc_roof_sc,
									 typename pcl::PointCloud<PointT>::Ptr &pc_vertex_sc,
									 bool use_more_points = false)
	{
		bbx_filter(pc_ground_tc, intersection_bbx);
		bbx_filter(pc_pillar_tc, intersection_bbx);
		bbx_filter(pc_beam_tc, intersection_bbx);
		bbx_filter(pc_facade_tc, intersection_bbx);
		bbx_filter(pc_roof_tc, intersection_bbx);
		bbx_filter(pc_vertex_tc, intersection_bbx);
		if (use_more_points)
		{
			bbx_filter(pc_ground_sc, intersection_bbx);
			bbx_filter(pc_pillar_sc, intersection_bbx);
			bbx_filter(pc_beam_sc, intersection_bbx);
			bbx_filter(pc_facade_sc, intersection_bbx);
			bbx_filter(pc_roof_sc, intersection_bbx);
			bbx_filter(pc_vertex_sc, intersection_bbx);
		}
		else
		{
			bbx_filter(pc_ground_sc, intersection_bbx);
			bbx_filter(pc_pillar_sc, intersection_bbx);
			bbx_filter(pc_beam_sc, intersection_bbx);
			bbx_filter(pc_facade_sc, intersection_bbx);
			bbx_filter(pc_roof_sc, intersection_bbx);
			bbx_filter(pc_vertex_sc, intersection_bbx);
		}

		//LOG(INFO) << "Intersection Bounding box filtering done";
		return 1;
	}

	//Bridef: Used for the preprocessing of fine registration
	//Use the intersection bounding box to filter the outlier points (--> to speed up)
	bool get_cloud_pair_intersection(bounds_t &intersection_bbx, typename pcl::PointCloud<PointT>::Ptr &pc_target, typename pcl::PointCloud<PointT>::Ptr &pc_source)
	{
		bbx_filter(pc_target, intersection_bbx);
		bbx_filter(pc_source, intersection_bbx);

		//LOG(INFO) << "Intersection Bounding box filtering done";
		return 1;
	}

	//Brief: Generate the crosssection of the point cloud for better visualization and comparison
	void crosssection_4_comp(const typename pcl::PointCloud<PointT>::Ptr &cloud_S,
							 const typename pcl::PointCloud<PointT>::Ptr &cloud_T,
							 const typename pcl::PointCloud<PointT>::Ptr &cloud_SR,
							 std::vector<typename pcl::PointCloud<PointT>::Ptr> &cross_section_S,
							 std::vector<typename pcl::PointCloud<PointT>::Ptr> &cross_section_T,
							 std::vector<typename pcl::PointCloud<PointT>::Ptr> &cross_section_SR,
							 float cross_section_width = 2.0)
	{
		centerpoint_t cp_S, cp_T;
		this->get_cloud_cpt(cloud_S, cp_S);
		this->get_cloud_cpt(cloud_T, cp_T);

		bounds_t cross_section_x, cross_section_y;
		cross_section_x.inf_xyz();
		cross_section_y.inf_xyz();

		cross_section_x.min_y = (cp_S.y + cp_T.y - cross_section_width) * 0.5;
		cross_section_x.max_y = (cp_S.y + cp_T.y + cross_section_width) * 0.5;

		cross_section_y.min_y = (cp_S.x + cp_T.x - cross_section_width) * 0.5;
		cross_section_y.max_y = (cp_S.x + cp_T.x + cross_section_width) * 0.5;

		typename pcl::PointCloud<PointT>::Ptr cross_S_x(new pcl::PointCloud<PointT>());
		typename pcl::PointCloud<PointT>::Ptr cross_S_y(new pcl::PointCloud<PointT>());
		typename pcl::PointCloud<PointT>::Ptr cross_T_x(new pcl::PointCloud<PointT>());
		typename pcl::PointCloud<PointT>::Ptr cross_T_y(new pcl::PointCloud<PointT>());
		typename pcl::PointCloud<PointT>::Ptr cross_SR_x(new pcl::PointCloud<PointT>());
		typename pcl::PointCloud<PointT>::Ptr cross_SR_y(new pcl::PointCloud<PointT>());

		bbx_filter(cloud_S, cross_S_x, cross_section_x);
		bbx_filter(cloud_S, cross_S_y, cross_section_y);
		bbx_filter(cloud_T, cross_T_x, cross_section_x);
		bbx_filter(cloud_T, cross_T_y, cross_section_y);
		bbx_filter(cloud_SR, cross_SR_x, cross_section_x);
		bbx_filter(cloud_SR, cross_SR_y, cross_section_y);

		cross_section_S.push_back(cross_S_x);
		cross_section_S.push_back(cross_S_y);
		cross_section_T.push_back(cross_T_x);
		cross_section_T.push_back(cross_T_y);
		cross_section_SR.push_back(cross_SR_x);
		cross_section_SR.push_back(cross_SR_y);
	}

#if OPENCV_ON
	bool pointcloud_to_rangeimage(typename pcl::PointCloud<PointT>::Ptr &point_cloud, cv::Mat &range_image)
	{
		//used for HDL 64 LiDAR
		int width = 900;
		int height = 64;
		float f_up = 3.0;  //deg
		float f_down = 25; //deg
		float max_distance = 70.0;

		range_image.create(height, width, CV_8UC1); // row, col

		range_image = cv::Mat::zeros(cv::Size(width, height), CV_8UC1);

		for (int i = 0; i < point_cloud->points.size(); i++)
		{
			PointT pt = point_cloud->points[i];
			float temp_dist = std::sqrt(pt.x * pt.x + pt.y * pt.y + pt.z * pt.z);
			float hor_ang = std::atan2(pt.y, pt.x);						// [-pi, pi]
			float ver_ang = std::asin(pt.z / temp_dist) / M_PI * 180.0; // to deg

			float u = 0.5 * (1 - hor_ang / M_PI) * width; // (0 width)
			float v = (1 - (f_up - ver_ang) / (f_up + f_down)) * height;

			int c = max_(min_(width - 1, (int)(u)), 0);  //col
			int r = max_(min_(height - 1, (int)(v)), 0); //row

			// if (i % 100 == 0)
			//     std::cout << r << "," << c << "," << point_cloud->points[i].d << std::endl;

			range_image.at<uchar>(63 - r, c) = 255.0 * min_(1.0, temp_dist / max_distance);
		}
		return true;
	}

	//copyright: https://github.com/koide3/pointcloud_to_2dmap
	//pointcloud_to_2dmap
	//m2pix means 1 meters has m2pix pixel
	cv::Mat generate_2d_map(const typename pcl::PointCloud<PointT>::Ptr &cloud,
							double m2pix, int map_width, int map_height, int min_points_in_pix, int max_points_in_pix, double min_height, double max_height, bool shift_or_not = false)
	{
		//TODO: fix problem here (shift)
		float shift_x;
		float shift_y;
		if (shift_or_not)
		{
			centerpoint_t cpt;
			this->get_cloud_cpt(cloud, cpt);
			shift_x = cpt.x;
			shift_y = cpt.y;
		}
		else
		{
			shift_x = 0.;
			shift_y = 0.;
		}
		cv::Mat map(map_height, map_width, CV_32SC1, cv::Scalar::all(0));

		for (int i = 0; i < cloud->points.size(); i++)
		{
			PointT point = cloud->points[i];

			if (point.z < min_height || point.z > max_height)
				continue;

			int x = (point.x - shift_x) * m2pix + map_width / 2;
			int y = -(point.y - shift_y) * m2pix + map_height / 2;

			if (x < 0 || x >= map_width || y < 0 || y >= map_height)
				continue;

			map.at<int>(y, x)++;
		}
		map -= min_points_in_pix;
		map.convertTo(map, CV_8UC1, -255.0 / (max_points_in_pix - min_points_in_pix), 255);

		return map;
	}
#endif
};

} // namespace lo

#endif //_INCLUDE_FILTER_HPP