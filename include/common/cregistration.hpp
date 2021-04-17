//
// This file is for the general implements of all kinds of registration methods
// The following registration methods are involoved:
// ICP (PCL), GICP, VGICP, NDT (Kenji Koide), FPFH-SAC(PCL), BSC-SAC (Zhen Dong et al.), TEASER (Heng Yang et al.), MMLLS-ICP (Yue Pan et al.)
// Dependent 3rd Libs: PCL (>1.7), TEASER++ (optional), Sophus (optional)
// By Yue Pan
//

#ifndef _INCLUDE_COMMON_REG_HPP
#define _INCLUDE_COMMON_REG_HPP

#include <math.h>

//pcl
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/gicp.h>
#include <pcl/registration/ndt.h>
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/correspondence_estimation_normal_shooting.h>
#include <pcl/registration/correspondence_rejection_features.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>
#include <pcl/registration/correspondence_rejection_trimmed.h>
#include <pcl/registration/correspondence_rejection_distance.h>
#include <pcl/registration/correspondence_rejection_var_trimmed.h>
#include <pcl/registration/transformation_estimation_svd.h>
#include <pcl/registration/transformation_estimation_lm.h>
#include <pcl/registration/transformation_estimation_point_to_plane_lls_weighted.h>
#include <pcl/registration/transformation_estimation_point_to_plane_weighted.h>
#include <pcl/registration/transformation_estimation_point_to_plane.h>
#include <pcl/registration/ia_ransac.h>

#if TEASER_ON
//teaser++ (global registration)
#include <teaser/ply_io.h>
#include <teaser/registration.h>
#include <teaser/certification.h>
#endif

//#include <unsupported/Eigen/MatrixFunctions>

#include "cfilter.hpp"
#include "utility.hpp"
#include "pca.hpp"
#include "map_viewer.h"

//koide reg_lib (local registration baselines)
#include "ndt_omp.h"
#include "gicp_omp.h"
#include "fast_vgicp.h"

namespace lo
{

enum TransformEstimationType
{
	SVD,
	LM,
	LLS
};
enum CorresEstimationType
{
	NN,
	NS
}; //NN: Nearest Neighbor ; NS: Normal Shooting
enum DistMetricType
{
	Point2Point,
	Point2Plane,
	Plane2Plane
};

template <typename PointT>
class CRegistration : public CloudUtility<PointT>
{
  public:
	//General implement of icp registration algorithm in pcl (baseline method)
	//(with different distance metrics, correspondence estimation, transformation estimation methods and parameters)
	//radius NN neighborhood search
	double icp_registration(const typename pcl::PointCloud<PointT>::Ptr &SourceCloud,
							const typename pcl::PointCloud<PointT>::Ptr &TargetCloud,
							typename pcl::PointCloud<PointT>::Ptr &TransformedSource,
							Eigen::Matrix4d &transformationS2T,
							DistMetricType metrics, CorresEstimationType ce, TransformEstimationType te,
							bool use_reciprocal_correspondence, bool use_trimmed_rejector,
							int max_iter, float thre_dis, float neighbor_radius)
	{
		clock_t t0, t1, t2;
		t0 = clock();

		double mae_nn;
		pcl::registration::CorrespondenceRejectorVarTrimmed::Ptr trimmed_cr(new pcl::registration::CorrespondenceRejectorVarTrimmed);

		switch (metrics)
		{
		case Point2Point:
		{
			t1 = clock();
			pcl::IterativeClosestPoint<PointT, PointT> icp;

			icp.setInputSource(SourceCloud);
			icp.setInputTarget(TargetCloud);

			typename pcl::registration::TransformationEstimationSVD<PointT, PointT, float>::Ptr te_svd(new pcl::registration::TransformationEstimationSVD<PointT, PointT, float>);
			typename pcl::registration::TransformationEstimationLM<PointT, PointT, float>::Ptr te_lm(new pcl::registration::TransformationEstimationLM<PointT, PointT, float>);

			switch (te)
			{
			case SVD:
				icp.setTransformationEstimation(te_svd); //Use SVD
				break;
			case LM:
				icp.setTransformationEstimation(te_lm); //Use L-M Non-Linear Optimization
				break;
			default: //Default svd
				break;
			}

			// Use Reciprocal Correspondences or not? [a -> b && b -> a]
			icp.setUseReciprocalCorrespondences(use_reciprocal_correspondence);

			// Trimmed or not?
			if (use_trimmed_rejector)
				icp.addCorrespondenceRejector(trimmed_cr);
			else
				icp.setMaxCorrespondenceDistance(thre_dis);

			// Converge criterion ( 'Or' Relation )
			// Set the maximum number of iterations [ n>x ] (criterion 1)
			icp.setMaximumIterations(max_iter); //Most likely to happen
			// Set the transformation difference threshold [delta_t<sqrt(x) or delta_ang<arccos(1-x)] (criterion 2)
			icp.setTransformationEpsilon(1e-8); //Quite hard to happen
			// Set the relative RMS difference between two consecutive iterations [ RMS(n)-RMS(n+1)<x*RMS(n) ] (criterion 3)
			icp.setEuclideanFitnessEpsilon(1e-5); //Quite hard to happen

			icp.align(*TransformedSource);

			transformationS2T = icp.getFinalTransformation();

			t2 = clock();

			if (use_trimmed_rejector)
				thre_dis = trimmed_cr->getTrimmedDistance();
			printf("Estimated trimmed distance threshold is %lf.\n", thre_dis);

			mae_nn = icp.getFitnessScore(thre_dis); //Get the Mean Absolute Error (MAE) after registration calculated from Nearest Neighbor Search

			// Commented these out if you don't want to output the registration log
			cout << "SCloud point # " << SourceCloud->points.size() << " , TCloud point # " << TargetCloud->points.size() << endl;
			cout << "Point to Point ICP done in " << float(t2 - t0) / CLOCKS_PER_SEC << " s" << endl;
			cout << "The fitness score of this registration is " << mae_nn << endl
				 << transformationS2T << endl;

			break;
		}
		case Point2Plane:
		{
			pcl::PointCloud<pcl::PointXYZ>::Ptr SourceCloudXYZ(new pcl::PointCloud<pcl::PointXYZ>());
			pcl::PointCloud<pcl::PointXYZ>::Ptr TargetCloudXYZ(new pcl::PointCloud<pcl::PointXYZ>());
			copyPointCloud(*SourceCloud, *SourceCloudXYZ);
			copyPointCloud(*TargetCloud, *TargetCloudXYZ);
			// In this case, The Cloud's Normal hasn't been calculated yet.

			pcl::PointCloud<pcl::PointNormal>::Ptr SourceNormal(new pcl::PointCloud<pcl::PointNormal>());
			pcl::PointCloud<pcl::PointNormal>::Ptr TargetNormal(new pcl::PointCloud<pcl::PointNormal>());
			pcl::PointCloud<pcl::PointNormal>::Ptr TransformedSourceN(new pcl::PointCloud<pcl::PointNormal>());

			//Estimate Normal Multi-thread
			PrincipleComponentAnalysis<pcl::PointXYZ> pca_estimator;

			//Radius search
			pca_estimator.get_pc_normal_pcar(SourceCloudXYZ, neighbor_radius, SourceNormal);
			pca_estimator.get_pc_normal_pcar(TargetCloudXYZ, neighbor_radius, TargetNormal);
			//Or
			//KNN search
			//pca_estimator.get_pc_normal_pcak(SourceCloud, covariance_K, SourceNormal);
			//pca_estimator.get_pc_normal_pcak(TargetCloud, covariance_K, TargetNormal);
			t1 = clock();

			pcl::IterativeClosestPointWithNormals<pcl::PointNormal, pcl::PointNormal> icp;

			icp.setInputSource(SourceNormal);
			icp.setInputTarget(TargetNormal);

			if (ce == NS) //Normal Shooting
			{
				pcl::registration::CorrespondenceEstimationNormalShooting<pcl::PointNormal, pcl::PointNormal, pcl::PointNormal>::Ptr ns_est(new pcl::registration::CorrespondenceEstimationNormalShooting<pcl::PointNormal, pcl::PointNormal, pcl::PointNormal>);
				ns_est->setInputSource(SourceNormal);
				ns_est->setSourceNormals(SourceNormal);
				ns_est->setInputTarget(TargetNormal);
				ns_est->setKSearch(5);
				icp.setCorrespondenceEstimation(ns_est);
			}

			pcl::registration::TransformationEstimationPointToPlane<pcl::PointNormal, pcl::PointNormal, float>::Ptr te_lmn(new pcl::registration::TransformationEstimationPointToPlane<pcl::PointNormal, pcl::PointNormal, float>);
			pcl::registration::TransformationEstimationPointToPlaneLLS<pcl::PointNormal, pcl::PointNormal, float>::Ptr te_lls(new pcl::registration::TransformationEstimationPointToPlaneLLS<pcl::PointNormal, pcl::PointNormal, float>);
			pcl::registration::TransformationEstimationPointToPlaneLLSWeighted<pcl::PointNormal, pcl::PointNormal, float>::Ptr te_lls_weight(new pcl::registration::TransformationEstimationPointToPlaneLLSWeighted<pcl::PointNormal, pcl::PointNormal, float>);
			switch (te)
			{
			case LLS:
				icp.setTransformationEstimation(te_lls); //Use Linear Least Square
				break;
			case LM:
				icp.setTransformationEstimation(te_lmn); //Use L-M Non-Linear Optimization
				break;
			default: //Default lls
				break;
			}

			// Use Reciprocal Correspondences or not? [a -> b && b -> a]
			icp.setUseReciprocalCorrespondences(use_reciprocal_correspondence);

			// Trimmed or not?
			if (use_trimmed_rejector)
				icp.addCorrespondenceRejector(trimmed_cr);
			else
				icp.setMaxCorrespondenceDistance(thre_dis);

			// Converge criterion ( 'Or' Relation )
			// Set the maximum number of iterations [ n>x ] (criterion 1)
			icp.setMaximumIterations(max_iter); //Most likely to happen
			// Set the transformation difference threshold [delta_t<sqrt(x) or delta_ang<arccos(1-x)] (criterion 2)
			icp.setTransformationEpsilon(1e-8); //Quite hard to happen
			// Set the relative RMS difference between two consecutive iterations [ RMS(n)-RMS(n+1)<x*RMS(n) ] (criterion 3)
			icp.setEuclideanFitnessEpsilon(1e-5); //Quite hard to happen

			icp.align(*TransformedSourceN);

			transformationS2T = icp.getFinalTransformation();

			t2 = clock();

			if (use_trimmed_rejector)
				thre_dis = trimmed_cr->getTrimmedDistance();
			printf("Estimated trimmed distance threshold is %lf.\n", thre_dis);

			mae_nn = icp.getFitnessScore(thre_dis); //Get the Mean Absolute Error (MAE) after registration calculated from Nearest Neighbor Search

			// Commented these out if you don't want to output the registration log
			cout << "SCloud point # " << SourceCloud->points.size() << " , TCloud point # " << TargetCloud->points.size() << endl;
			cout << "Point-to-Plane ICP done in " << float(t2 - t0) / CLOCKS_PER_SEC << " s" << endl;
			cout << "Normal Estimation in " << float(t1 - t0) / CLOCKS_PER_SEC << " s, "
				 << "registration in " << float(t2 - t1) / CLOCKS_PER_SEC << " s." << endl;
			cout << "The fitness score of this registration is " << mae_nn << endl
				 << transformationS2T << endl;

			copyPointCloud(*TransformedSourceN, *TransformedSource);
			pcl::PointCloud<pcl::PointNormal>().swap(*TransformedSourceN); // Free the Memory
			pcl::PointCloud<pcl::PointNormal>().swap(*SourceNormal);	   // Free the Memory
			pcl::PointCloud<pcl::PointNormal>().swap(*TargetNormal);	   // Free the Memory
			pcl::PointCloud<pcl::PointXYZ>().swap(*SourceCloudXYZ);		   // Free the Memory
			pcl::PointCloud<pcl::PointXYZ>().swap(*TargetCloudXYZ);		   // Free the Memory

			break;
		}
		case Plane2Plane:
		{
			t1 = clock();
			pcl::GeneralizedIterativeClosestPoint<PointT, PointT> icp;

			// Set the number of points used to calculated the covariance of a point
			// icp.setCorrespondenceRandomness(covariance_K);
			icp.setCorrespondenceRandomness(10);

			icp.setInputSource(SourceCloud);
			icp.setInputTarget(TargetCloud);

			// Use Reciprocal Correspondences or not? [a -> b && b -> a]
			icp.setUseReciprocalCorrespondences(use_reciprocal_correspondence);

			// Trimmed or not?
			if (use_trimmed_rejector)
				icp.addCorrespondenceRejector(trimmed_cr);
			else
				icp.setMaxCorrespondenceDistance(thre_dis);

			icp.setMaximumOptimizerIterations(10);

			// Converge criterion ( 'Or' Relation )
			// Set the maximum number of iterations [ n>x ] (criterion 1)
			icp.setMaximumIterations(max_iter); //Most likely to happen
			// Set the transformation difference threshold [delta_t<sqrt(x) or delta_ang<arccos(1-x)] (criterion 2)
			icp.setTransformationEpsilon(1e-8); //Quite hard to happen
			// Set the relative RMS difference between two consecutive iterations [ RMS(n)-RMS(n+1)<x*RMS(n) ] (criterion 3)
			icp.setEuclideanFitnessEpsilon(1e-5); //Quite hard to happen

			icp.align(*TransformedSource);

			transformationS2T = icp.getFinalTransformation();

			t2 = clock();

			if (use_trimmed_rejector)
				thre_dis = trimmed_cr->getTrimmedDistance();
			printf("Estimated trimmed distance threshold is %lf.\n", thre_dis);

			mae_nn = icp.getFitnessScore(thre_dis); //Get the Mean Absolute Error (MAE) after registration calculated from Nearest Neighbor Search

			// Commented these out if you don't want to output the registration log
			cout << "SCloud point # " << SourceCloud->points.size() << " , TCloud point # " << TargetCloud->points.size() << endl;
			cout << "Plane-to-Plane ICP done in " << float(t2 - t0) / CLOCKS_PER_SEC << " s" << endl;
			cout << "The fitness score of this registration is " << mae_nn << endl
				 << transformationS2T << endl;
			break;
		}
		default:
			return -1;
		}

		return mae_nn;
	}

	/**
		* \brief Estimated the approximate overlapping ratio for Cloud1 considering Cloud2
		* \param[in]  Cloud1 : A pointer of the Point Cloud used for overlap ratio calculation
		* \param[in]  Cloud2 : A pointer of the Point Cloud overlapped with Cloud1
		* \param[out] thre_dis : It acts as the search radius of overlapping estimation
		* \return : The estimated overlap ratio [from 0 to 1]
		*/
	float get_overlap_ratio(const typename pcl::PointCloud<PointT>::Ptr &Cloud1,
							const typename pcl::PointCloud<PointT>::Ptr &Cloud2,
							float thre_dis)
	{
		int overlap_point_num = 0;
		float overlap_ratio;

		pcl::search::KdTree<PointT> kdtree;
		kdtree.setInputCloud(Cloud2);

		std::vector<int> pointIdxRadiusSearch;
		std::vector<float> pointRadiusSquaredDistance;

		for (int i = 0; i < Cloud1->size(); i++)
		{
			if (kdtree.radiusSearch(Cloud1->points[i], thre_dis, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0)
				overlap_point_num++;
		}

		overlap_ratio = (0.01 + overlap_point_num) / Cloud1->size();
		//cout << "The estimated approximate overlap ratio of Cloud 1 is " << overlap_ratio << endl;
		LOG(INFO) << "The estimated approximate overlap ratio of Cloud 1 is " << overlap_ratio;

		return overlap_ratio;
	}

	//brief: Compute fpfh_feature
	void compute_fpfh_feature(const typename pcl::PointCloud<PointT>::Ptr &input_cloud,
							  fpfhPtr &cloud_fpfh,
							  float search_radius)
	{
		// Calculate the Point Normal
		// pcl::PointCloud<pcl::PointNormal>::Ptr cloud_normal(new pcl::PointCloud<pcl::PointNormal>);
		// calNormal(input_cloud, cloud_normal, search_radius);

		// Estimate FPFH Feature
		typename pcl::FPFHEstimationOMP<PointT, PointT, pcl::FPFHSignature33> est_fpfh;
		est_fpfh.setNumberOfThreads(4);
		est_fpfh.setInputCloud(input_cloud);
		est_fpfh.setInputNormals(input_cloud);
		typename pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>());
		est_fpfh.setSearchMethod(tree);
		//est_fpfh.setKSearch(20);
		est_fpfh.setRadiusSearch(2.0 * search_radius);
		est_fpfh.compute(*cloud_fpfh);
	}

	//brief: Accomplish Coarse registration using FPFH SAC
	double coarse_reg_fpfhsac(const typename pcl::PointCloud<PointT>::Ptr &source_cloud,
							  const typename pcl::PointCloud<PointT>::Ptr &target_cloud,
							  typename pcl::PointCloud<PointT>::Ptr &traned_source,
							  Eigen::Matrix4d &transformationS2T,
							  float search_radius)
	{
		std::chrono::steady_clock::time_point tic = std::chrono::steady_clock::now();

		fpfhPtr source_fpfh(new fpfh());
		fpfhPtr target_fpfh(new fpfh());

		compute_fpfh_feature(source_cloud, source_fpfh, search_radius);
		compute_fpfh_feature(target_cloud, target_fpfh, search_radius);

		typename pcl::SampleConsensusInitialAlignment<PointT, PointT, pcl::FPFHSignature33> sac_ia;
		sac_ia.setInputSource(source_cloud);
		sac_ia.setSourceFeatures(source_fpfh);
		sac_ia.setInputTarget(target_cloud);
		sac_ia.setTargetFeatures(target_fpfh);
		//sac_ia.setNumberOfSamples(20);
		sac_ia.setCorrespondenceRandomness(15);
		sac_ia.align(*traned_source);
		transformationS2T = sac_ia.getFinalTransformation().template cast<double>();
		double fitness_score = sac_ia.getFitnessScore();

		std::chrono::steady_clock::time_point toc = std::chrono::steady_clock::now();
		std::chrono::duration<double> time_used = std::chrono::duration_cast<std::chrono::duration<double>>(toc - tic);

		// Commented these out if you don't want to output the registration log
		LOG(WARNING) << "FPFH-SAC coarse registration done in [" << time_used.count() * 1000.0 << "] ms";
		LOG(WARNING) << "The fitness score of this registration is " << fitness_score;
		LOG(WARNING) << "Transformation:\n " << transformationS2T;

		return fitness_score;
	}
#if 0
	bool find_putable_feature_correspondence_bsc(const doubleVectorSBF &target_bscs, const doubleVectorSBF &source_bscs, int dof,
												 const typename pcl::PointCloud<PointT>::Ptr &target_kpts, const typename pcl::PointCloud<PointT>::Ptr &source_kpts,
												 typename pcl::PointCloud<PointT>::Ptr &target_corrs, typename pcl::PointCloud<PointT>::Ptr &source_corrs,
												 int corr_num)
	{
		//reference: A novel binary shape context for 3D local surface description , ISPRS Journal 2017, Zhen Dong et al.
		LOG(INFO) << "[" << target_bscs[0].size() << "] bsc features in target point cloud and [" << source_bscs[0].size() << "] bsc features in source point cloud.";

		std::chrono::steady_clock::time_point tic = std::chrono::steady_clock::now();

		std::vector<std::vector<int>> dist_table(target_bscs[0].size());
		for (int i = 0; i < target_bscs[0].size(); i++)
			dist_table[i].resize(source_bscs[0].size());

		StereoBinaryFeature sbf_operator;

		int num_bsc;
		if (dof < 4)
			num_bsc = 1;
		else if (dof == 4)
			num_bsc = 2;
		else
			num_bsc = 4;

		int i;
#pragma omp parallel for private(i) //Multi-thread
		for (i = 0; i < target_bscs[0].size(); i++)
		{
			for (int j = 0; j < source_bscs[0].size(); j++)
			{
				if (target_bscs[0][i].keypointIndex_ == -1 || source_bscs[0][j].keypointIndex_ == -1)
					dist_table[i][j] = 99999;
				else
				{
					int min_dist_index = 0;
					int min_dist = 99999;
					for (int k = 0; k < num_bsc; k++)
					{
						int dist_temp = sbf_operator.hammingDistance(target_bscs[0][i], source_bscs[k][j]);
						//LOG(INFO) << "hamming distance:" << dist_temp;
						if (dist_temp < min_dist)
						{
							min_dist = dist_temp;
							min_dist_index = k;
						}
					}
					dist_table[i][j] = min_dist;
				}
			}
		}
		std::vector<std::pair<int, int>> dist_array;

		for (int i = 0; i < target_bscs[0].size(); i++)
		{
			for (int j = 0; j < source_bscs[0].size(); j++)
			{
				std::pair<int, int> temp_pair;
				temp_pair.first = i * source_bscs[0].size() + j;
				temp_pair.second = dist_table[i][j];
				dist_array.push_back(temp_pair);
			}
		}

		std::sort(dist_array.begin(), dist_array.end(), [](const std::pair<int, int> &a, const std::pair<int, int> &b) { return a.second < b.second; });

		corr_num = min_(corr_num, dist_array.size());

		for (int k = 0; k < corr_num; k++)
		{
			int index = dist_array[k].first;
			int i = index / source_bscs[0].size();
			int j = index % source_bscs[0].size();

			target_corrs->points.push_back(target_kpts->points[i]);
			source_corrs->points.push_back(source_kpts->points[j]);
		}

		//find correspondence
		// for (int i = 0; i < target_bscs[0].size(); i++)
		// {
		// 	if (target_bscs[0][i].keypointIndex_ == -1)
		// 	{
		// 		//LOG(WARNING) << "not an correct index";
		// 		continue;
		// 	}

		// 	for (int j = 0; j < source_bscs[0].size(); j++)
		// 	{
		// 		if (dist_table[i][j] < max_hamming_dist)
		// 		{
		// 			target_corrs->points.push_back(target_kpts->points[i]);
		// 			source_corrs->points.push_back(source_kpts->points[j]);
		// 		}
		// 	}
		// }

		std::vector<std::vector<int>>().swap(dist_table);
		std::vector<std::pair<int, int>>().swap(dist_array);

		std::chrono::steady_clock::time_point toc = std::chrono::steady_clock::now();
		std::chrono::duration<double> time_used = std::chrono::duration_cast<std::chrono::duration<double>>(toc - tic);

		LOG(INFO) << "[" << source_corrs->points.size() << "] correspondences found by BSC feature matching in [" << time_used.count() * 1000.0 << "] ms";

		return true;
	}
#endif

	//NCC: neighborhood category context descriptor
	bool find_feature_correspondence_ncc(const typename pcl::PointCloud<PointT>::Ptr &target_kpts, const typename pcl::PointCloud<PointT>::Ptr &source_kpts,
										 typename pcl::PointCloud<PointT>::Ptr &target_corrs, typename pcl::PointCloud<PointT>::Ptr &source_corrs,
										 bool fixed_num_corr = false, int corr_num = 2000, bool reciprocal_on = true)
										 // to enable reciprocal correspondence, you need to disable fixed_num_corr. 
										 // once fixed_num_cor is enabled, reciprocal correspondence would be automatically disabled
	{
		int target_kpts_num = target_kpts->points.size();
		int source_kpts_num = source_kpts->points.size();
		float dist_margin_thre = 0.0;

		LOG(INFO) << "[" << target_kpts_num << "] key points in target point cloud and [" << source_kpts_num << "] key points in source point cloud.";

		if (target_kpts_num < 10 || source_kpts_num < 10)
		{
			LOG(WARNING) << "Too few key points\n";
			return false;
		}

		std::chrono::steady_clock::time_point tic = std::chrono::steady_clock::now();

		//first get descriptor
		//std::vector<std::vector<int>> target_kpts_descriptors;
		std::vector<Eigen::VectorXf, Eigen::aligned_allocator<Eigen::VectorXf>> target_kpts_descriptors;

		float intensity_min = FLT_MAX;
		float intensity_max = 0;

		for (int i = 0; i < target_kpts_num; i++)
		{
			float cur_i = target_kpts->points[i].intensity;

			intensity_min = min_(intensity_min, cur_i);
			intensity_max = max_(intensity_max, cur_i);
		}

		for (int i = 0; i < target_kpts_num; i++)
		{
			Eigen::VectorXf temp_descriptor(11);
			int temp_descriptor_close = (int)target_kpts->points[i].normal[0];
			int temp_descriptor_far = (int)target_kpts->points[i].normal[1];
			// neighborhood category with its distance to the query point
			temp_descriptor(0) = temp_descriptor_close / 1000000;
			temp_descriptor(1) = (temp_descriptor_close % 1000000) / 10000;
			temp_descriptor(2) = (temp_descriptor_close % 10000) / 100;
			temp_descriptor(3) = temp_descriptor_close % 100;
			temp_descriptor(4) = temp_descriptor_far / 1000000;
			temp_descriptor(5) = (temp_descriptor_far % 1000000) / 10000;
			temp_descriptor(6) = (temp_descriptor_far % 10000) / 100;
			temp_descriptor(7) = temp_descriptor_far % 100;
			// other properties
			float cur_i = target_kpts->points[i].intensity;
			temp_descriptor(8) = (cur_i - intensity_min) / (intensity_max - intensity_min) * 255.0; //[0 - 255] //normalized intensity 
			temp_descriptor(9) = target_kpts->points[i].normal[3] * 100;							//[0 - 100] //curvature
			temp_descriptor(10) = target_kpts->points[i].data[3] * 30;								//[0 - 100] //height above ground
			//LOG(INFO) << temp_descriptor[1] << "," << temp_descriptor[2] << "," << temp_descriptor[3] << "," << temp_descriptor[4] << "," << temp_descriptor[5] << "," << temp_descriptor[6] << "," << temp_descriptor[7] << "," << temp_descriptor[8] << "," << temp_descriptor[9] << "," << temp_descriptor[10] << "," << temp_descriptor[11];
			target_kpts_descriptors.push_back(temp_descriptor);
		}

		std::vector<Eigen::VectorXf, Eigen::aligned_allocator<Eigen::VectorXf>> source_kpts_descriptors;
		for (int i = 0; i < source_kpts_num; i++)
		{
			Eigen::VectorXf temp_descriptor(11);
			int temp_descriptor_close = (int)source_kpts->points[i].normal[0];
			int temp_descriptor_far = (int)source_kpts->points[i].normal[1];
			// neighborhood category with its distance to the query point
			temp_descriptor(0) = temp_descriptor_close / 1000000;
			temp_descriptor(1) = (temp_descriptor_close % 1000000) / 10000;
			temp_descriptor(2) = (temp_descriptor_close % 10000) / 100;
			temp_descriptor(3) = temp_descriptor_close % 100;
			temp_descriptor(4) = temp_descriptor_far / 1000000;
			temp_descriptor(5) = (temp_descriptor_far % 1000000) / 10000;
			temp_descriptor(6) = (temp_descriptor_far % 10000) / 100;
			temp_descriptor(7) = temp_descriptor_far % 100;
			// other properties
			float cur_i = source_kpts->points[i].intensity;
			temp_descriptor(8) = (cur_i - intensity_min) / (intensity_max - intensity_min) * 255.0; //[0 - 255] //normalized intensity 
			temp_descriptor(9) = source_kpts->points[i].normal[3] * 100; //[0 - 100] //curvature
			temp_descriptor(10) = source_kpts->points[i].data[3] * 30;   //[0 - 100] //height above ground
			//LOG(INFO) << temp_descriptor[1] << "," << temp_descriptor[2] << "," << temp_descriptor[3] << "," << temp_descriptor[4] << "," << temp_descriptor[5] << "," << temp_descriptor[6] << "," << temp_descriptor[7] << "," << temp_descriptor[8] << "," << temp_descriptor[9] << "," << temp_descriptor[10] << "," << temp_descriptor[11];
			source_kpts_descriptors.push_back(temp_descriptor);
		}

		std::vector<std::vector<float>> dist_table(target_kpts_num);
		for (int i = 0; i < target_kpts_num; i++)
			dist_table[i].resize(source_kpts_num);

		std::vector<std::pair<int, float>> dist_array;
        

        //TODO: speed up
		omp_set_num_threads(min_(6, omp_get_max_threads()));
#pragma omp parallel for  //Multi-thread
		for (int i = 0; i < target_kpts_num; i++)
		{
			for (int j = 0; j < source_kpts_num; j++)
			{
				//Method 1. directly use L1 distance (use the features from 0 to 11)
				for (int k = 0; k < 11; k++)
					dist_table[i][j] += std::abs(target_kpts_descriptors[i](k) - source_kpts_descriptors[j](k));

				//Method 2. use cosine similarity instead
				//dist_table[i][j] =
				//target_kpts_descriptors[i].norm() * source_kpts_descriptors[j].norm() / target_kpts_descriptors[i].dot(source_kpts_descriptors[j]);

				//Method 3. use K-L divergence instead (use only the histogram (distribution)
				//for (int k = 0; k < 8; k++)
				//	dist_table[i][j] += 1.0 * target_kpts_descriptors[i](k) * std::log((1.0 * target_kpts_descriptors[i](k) + 0.001) / (1.0 * source_kpts_descriptors[j](k) + 0.001));
			}
		}
		if (!fixed_num_corr)
		{
			//find correspondence
			for (int i = 0; i < target_kpts_num; i++)
			{
				//LOG(INFO) << "keypoint indice: " << target_bscs[0][i].keypointIndex_;
				int min_dist_col_index = 0;
				float min_dist_row = FLT_MAX;
				for (int j = 0; j < source_kpts_num; j++)
				{
					if (dist_table[i][j] < min_dist_row)
					{
						min_dist_row = dist_table[i][j];
						min_dist_col_index = j;
					}
				}
				bool refined_corr = true;
				if (reciprocal_on) //reciprocal nearest neighbor correspondnece
				{
					for (int j = 0; j < target_kpts_num; j++)
					{
						if (min_dist_row > dist_table[j][min_dist_col_index] + dist_margin_thre)
						{
							refined_corr = false;
							break;
						}
					}
				}
				if (refined_corr)
				{
					//LOG(INFO) << "[" << i << "] - [" << min_dist_col_index << "]:" << min_dist_row;
					target_corrs->points.push_back(target_kpts->points[i]);
					source_corrs->points.push_back(source_kpts->points[min_dist_col_index]);
				}
			}
		}
		else //fixed num correspondence
		{
			for (int i = 0; i < target_kpts_num; i++)
			{
				for (int j = 0; j < source_kpts_num; j++)
				{
					std::pair<int, float> temp_pair;
					temp_pair.first = i * source_kpts_num + j;
					temp_pair.second = dist_table[i][j];
					dist_array.push_back(temp_pair);
				}
			}
			std::sort(dist_array.begin(), dist_array.end(), [](const std::pair<int, float> &a, const std::pair<int, float> &b) { return a.second < b.second; });
			corr_num = min_(corr_num, dist_array.size()); //take the k shortest distance

			std::vector<int> count_target_kpt(target_kpts_num, 0);
			std::vector<int> count_source_kpt(source_kpts_num, 0);

			int max_corr_num = 6;

			for (int k = 0; k < corr_num; k++)
			{
				int index = dist_array[k].first;
				int i = index / source_kpts_num;
				int j = index % source_kpts_num;

				if (count_target_kpt[i] > max_corr_num || count_source_kpt[j] > max_corr_num) //we only keep the first max_corr_num candidate correspondence of a single point in either source or target point cloud
					continue;

				count_target_kpt[i]++;
				count_source_kpt[j]++;

				target_corrs->points.push_back(target_kpts->points[i]);
				source_corrs->points.push_back(source_kpts->points[j]);
			}
		}

		std::chrono::steady_clock::time_point toc = std::chrono::steady_clock::now();
		std::chrono::duration<double> time_used = std::chrono::duration_cast<std::chrono::duration<double>>(toc - tic);

		//free memory
		std::vector<Eigen::VectorXf, Eigen::aligned_allocator<Eigen::VectorXf>>().swap(target_kpts_descriptors);
		std::vector<Eigen::VectorXf, Eigen::aligned_allocator<Eigen::VectorXf>>().swap(source_kpts_descriptors);
		std::vector<std::vector<float>>().swap(dist_table);
		std::vector<std::pair<int, float>>().swap(dist_array);

		LOG(INFO) << "[" << source_corrs->points.size() << "] correspondences found in [" << time_used.count() * 1000.0 << "] ms";

		return true;
	}
    

    //coarse global registration using RANSAC 
	int coarse_reg_ransac(const typename pcl::PointCloud<PointT>::Ptr &target_pts,
						  const typename pcl::PointCloud<PointT>::Ptr &source_pts,
						  Eigen::Matrix4d &tran_mat, float noise_bound = 0.2, int min_inlier_num = 8, int max_iter_num = 20000)
	{
        std::chrono::steady_clock::time_point tic = std::chrono::steady_clock::now();
		
        int N = target_pts->points.size();

	    pcl::registration::CorrespondenceRejectorSampleConsensus<PointT> ransac_rej;
		ransac_rej.setInputSource(source_pts);
		ransac_rej.setInputTarget(target_pts);
		ransac_rej.setInlierThreshold(noise_bound);
		ransac_rej.setMaximumIterations(max_iter_num);
		ransac_rej.setRefineModel(true);//false
        
		boost::shared_ptr<pcl::Correspondences> init_corres(new pcl::Correspondences);
        for (int i=0; i< N; i++)
		{
			pcl::Correspondence cur_corr;
			cur_corr.index_query=i;
			cur_corr.index_match=i;
			init_corres->push_back(cur_corr);
		}

		boost::shared_ptr<pcl::Correspondences> final_corres(new pcl::Correspondences);

		ransac_rej.setInputCorrespondences(init_corres);
		ransac_rej.getCorrespondences(*final_corres);

		std::chrono::steady_clock::time_point toc = std::chrono::steady_clock::now();
		std::chrono::duration<double> time_used = std::chrono::duration_cast<std::chrono::duration<double>>(toc - tic);

		LOG(INFO) << "----------------------------------------------------------------------------";
		LOG(INFO) << "Begin RANSAC global coarse registration with [" << N << "] pairs of correspondence";
		LOG(INFO) << "RANSAC global coarse registration done in [" << time_used.count() * 1000.0 << "] ms.";
		LOG(INFO) << "[" << final_corres->size() << "] inlier correspondences found.";

		if(final_corres->size() >= min_inlier_num)
		{
            Eigen::Matrix4f best_tran =ransac_rej.getBestTransformation();

		    tran_mat = best_tran.cast<double>();

			LOG(INFO) << "Estimated transformation by RANSAC is :\n"
					  << tran_mat;

            if (final_corres->size() >= 2 * min_inlier_num)
				return (1); //reliable
			else
				return (0); //need check
		}
		else
		{
			LOG(WARNING) << "RANSAC failed";
			return (-1);
		}
	}

	//coarse global registration using TEASER ++  (faster and more robust to outlier than RANSAC)
	int coarse_reg_teaser(const typename pcl::PointCloud<PointT>::Ptr &target_pts,
						  const typename pcl::PointCloud<PointT>::Ptr &source_pts,
						  Eigen::Matrix4d &tran_mat, float noise_bound = 0.2, int min_inlier_num = 8)
	{
		//reference: https://github.com/MIT-SPARK/TEASER-plusplus
		//TEASER: Fast and Certifiable Point Cloud Registration, TRO, Heng Yang et al.

#if TEASER_ON

		int teaser_state = 0; //(failed: -1, successful[need check]: 0, successful[reliable]: 1)

		if (target_pts->points.size() != source_pts->points.size())
		{
			LOG(ERROR) << "source points number != target points number";
			return (-1); //fail
		}

		if (target_pts->points.size() <= 3)
		{
			LOG(ERROR) << "too few correspondences";
			return (-1);
		}

		int N = target_pts->points.size();
		float min_inlier_ratio = 0.01;

		Eigen::Matrix<double, 3, Eigen::Dynamic> src(3, N);
		Eigen::Matrix<double, 3, Eigen::Dynamic> tgt(3, N);

		for (int i = 0; i < N; ++i)
		{
			src.col(i) << source_pts->points[i].x, source_pts->points[i].y, source_pts->points[i].z;
			tgt.col(i) << target_pts->points[i].x, target_pts->points[i].y, target_pts->points[i].z;
		}

		// Run TEASER++ registration
		// Prepare solver parameters
		teaser::RobustRegistrationSolver::Params params;
		params.noise_bound = noise_bound;
		params.cbar2 = 1.0;
		params.estimate_scaling = false;
		params.rotation_max_iterations = 100;
		params.rotation_gnc_factor = 1.4;
		params.rotation_estimation_algorithm =
			teaser::RobustRegistrationSolver::ROTATION_ESTIMATION_ALGORITHM::GNC_TLS;
		params.use_max_clique = true;
		params.kcore_heuristic_threshold = 0.5;
		params.rotation_cost_threshold = 0.005; //1e-6

		// Solve with TEASER++
		LOG(INFO) << "----------------------------------------------------------------------------";
		LOG(INFO) << "Begin TEASER global coarse registration with [" << N << "] pairs of correspondence";
		teaser::RobustRegistrationSolver solver(params);
		std::chrono::steady_clock::time_point tic = std::chrono::steady_clock::now();
		solver.solve(src, tgt);
		std::chrono::steady_clock::time_point toc = std::chrono::steady_clock::now();
		std::chrono::duration<double> time_used = std::chrono::duration_cast<std::chrono::duration<double>>(toc - tic);

		LOG(INFO) << "TEASER global coarse registration done in [" << time_used.count() * 1000.0 << "] ms.";

		auto solution = solver.getSolution();
		std::vector<int> inliers;
		//inliers = solver.getTranslationInliers();
		inliers = solver.getRotationInliers();

		LOG(INFO) << "[" << inliers.size() << "] inlier correspondences found.";

		//if (solution.valid && 1.0 * inliers.size() / N >= min_inlier_ratio)
		if (solution.valid && inliers.size() >= min_inlier_num)
		{
			tran_mat.setIdentity();
			tran_mat.block<3, 3>(0, 0) = solution.rotation;
			tran_mat.block<3, 1>(0, 3) = solution.translation;

			LOG(INFO) << "Estimated transformation by TEASER is :\n"
					  << tran_mat;

			// certificate the result here
			// teaser::DRSCertifier::Params cer_params;
			// teaser::DRSCertifier certifier(cer_params);
			// auto certification_result = certifier.certify(tran_mat,src,dst, theta);

			if (inliers.size() >= 2 * min_inlier_num)
				return (1); //reliable
			else
				return (0); //need check
		}
		else
		{
			LOG(WARNING) << "TEASER failed";
			return (-1);
		}

#endif
		return (-1);
	}

	// registration basic interface
	double base_align(typename pcl::Registration<PointT, PointT>::Ptr registration,
					  const typename pcl::PointCloud<PointT>::Ptr &target_cloud,
					  const typename pcl::PointCloud<PointT>::Ptr &source_cloud,
					  typename pcl::PointCloud<PointT>::Ptr &transformed_source,
					  Eigen::Matrix4d &Trans)
	{
		std::chrono::steady_clock::time_point tic = std::chrono::steady_clock::now();

		registration->setInputTarget(target_cloud);
		registration->setInputSource(source_cloud);

		registration->align(*transformed_source);
		Trans = registration->getFinalTransformation().template cast<double>();

		double fitness_score = registration->getFitnessScore();

		std::chrono::steady_clock::time_point toc = std::chrono::steady_clock::now();
		std::chrono::duration<double> time_used = std::chrono::duration_cast<std::chrono::duration<double>>(toc - tic);
		LOG(INFO) << "registration time : " << time_used.count() * 1000.0 << "[ms]";
		LOG(INFO) << "fitness score: " << fitness_score;
		LOG(INFO) << "Transformation:\n " << Trans;

		return fitness_score;
	}

	//for voxel based fast gicp (by koide3)
	double base_align(typename pcl::Registration<PointT, PointT, double>::Ptr registration,
					  const typename pcl::PointCloud<PointT>::Ptr &target_cloud,
					  const typename pcl::PointCloud<PointT>::Ptr &source_cloud,
					  typename pcl::PointCloud<PointT>::Ptr &transformed_source,
					  Eigen::Matrix4d &Trans, bool only_estimate_source_covariance = false)
	{
		std::chrono::steady_clock::time_point tic = std::chrono::steady_clock::now();

		registration->setInputTarget(target_cloud); //covariance is calculated here
		registration->setInputSource(source_cloud);

		std::chrono::steady_clock::time_point toc_1 = std::chrono::steady_clock::now();

		registration->align(*transformed_source);
		Trans = registration->getFinalTransformation().template cast<double>();

		double fitness_score = registration->getFitnessScore();

		std::chrono::steady_clock::time_point toc_2 = std::chrono::steady_clock::now();
		std::chrono::duration<double> time_used_covariance = std::chrono::duration_cast<std::chrono::duration<double>>(toc_1 - tic);
		std::chrono::duration<double> time_used_align = std::chrono::duration_cast<std::chrono::duration<double>>(toc_2 - toc_1);
		LOG(INFO) << "covariance calculation done in [" << time_used_covariance.count() * 1000.0 << "] ms.";
		LOG(INFO) << "registration done in [" << time_used_align.count() * 1000.0 << "] ms.";
		LOG(INFO) << "fitness score: " << fitness_score;
		LOG(INFO) << "Transformation:\n " << Trans;

		return fitness_score;
	}

	//pertubate the point cloud with a small translation
	void pertubate_cloud(const typename pcl::PointCloud<PointT>::Ptr &cloud_in,
						 typename pcl::PointCloud<PointT>::Ptr &cloud_out,
						 float pertubate_value, std::vector<float> &pertubate_vector)
	{
		pertubate_vector.resize(3);
		pertubate_vector[0] = 0.5 * pertubate_value - pertubate_value * ((double)rand() / RAND_MAX);
		pertubate_vector[1] = 0.5 * pertubate_value - pertubate_value * ((double)rand() / RAND_MAX);
		pertubate_vector[2] = 0.5 * pertubate_value - pertubate_value * ((double)rand() / RAND_MAX);

		for (size_t i = 0; i < cloud_in->size(); i++)
		{
			PointT pt;
			pt.x = cloud_in->points[i].x + pertubate_vector[0];
			pt.y = cloud_in->points[i].y + pertubate_vector[1];
			pt.z = cloud_in->points[i].z + pertubate_vector[2];
			cloud_out->push_back(pt);
		}
		LOG(INFO) << "The pertubation vector is:  X " << pertubate_vector[0] << " , Y " << pertubate_vector[1] << " , Z " << pertubate_vector[2];
	}

	//determine which one is the source point cloud for registration (according to the number of points)
	bool determine_source_target_cloud(cloudblock_Ptr &block_1, cloudblock_Ptr &block_2, cloudblock_Ptr &block_sc, cloudblock_Ptr &block_tc,
									   constraint_t &registration_cons)
	{
		if (block_1->down_feature_point_num > block_2->down_feature_point_num)
		{
			block_sc = block_2;
			block_tc = block_1;
		}
		else
		{
			block_sc = block_1;
			block_tc = block_2;
		}
		registration_cons.block1 = block_tc;
		registration_cons.block2 = block_sc;
		return 1;
	}

	bool determine_source_target_cloud(const cloudblock_Ptr &block_1, const cloudblock_Ptr &block_2, constraint_t &registration_cons)
	{
		if (block_1->down_feature_point_num > block_2->down_feature_point_num)
		{
			registration_cons.block1 = block_1;
			registration_cons.block2 = block_2;
		}
		else
		{
			registration_cons.block1 = block_2;
			registration_cons.block2 = block_1;
		}
		return 1;
	}

	//Target point cloud: block1
	//Source point cloud: block2
	bool assign_source_target_cloud(const cloudblock_Ptr &block_1, const cloudblock_Ptr &block_2, constraint_t &registration_cons)
	{
		registration_cons.block1 = block_1; //target
		registration_cons.block2 = block_2; //source
	}

	//interface for the implement of basic icp algorithm using pcl
	int pcl_icp(constraint_t &registration_cons,
				int max_iter_num, float dis_thre_unit,
				DistMetricType metrics, CorresEstimationType ce, TransformEstimationType te,
				bool use_reciprocal_correspondence, bool use_trimmed_rejector, float neighbor_radius = 2.0,
				Eigen::Matrix4d initial_guess = Eigen::Matrix4d::Identity(), bool apply_intersection_filter = true,
				float fitness_score_thre = 10.0)
	{
		CFilter<PointT> cfilter;

		int process_code = 0;

		typename pcl::PointCloud<PointT>::Ptr cloud_t_down(new pcl::PointCloud<PointT>);
		typename pcl::PointCloud<PointT>::Ptr cloud_s_down(new pcl::PointCloud<PointT>);
		typename pcl::PointCloud<PointT>::Ptr cloud_s_guess(new pcl::PointCloud<PointT>);
		typename pcl::PointCloud<PointT>::Ptr cloud_s_tran(new pcl::PointCloud<PointT>);

		registration_cons.block1->clone_cloud(cloud_t_down, true); //target
		registration_cons.block2->clone_cloud(cloud_s_down, true); //source

		bounds_t intersection_bbx, source_guess_bbx;
		bool apply_source_initial_guess = false;
		if (!initial_guess.isIdentity(1e-6))
		{
			//Transform the Source pointcloud
			pcl::transformPointCloudWithNormals(*cloud_s_down, *cloud_s_guess, initial_guess);
			cfilter.get_cloud_bbx(cloud_s_guess, source_guess_bbx);
			cfilter.get_intersection_bbx(registration_cons.block1->local_bound, source_guess_bbx, intersection_bbx);
			LOG(INFO) << "Apply initial guess transformation\n"
					  << initial_guess;
			apply_source_initial_guess = true;
		}
		else
			cfilter.get_intersection_bbx(registration_cons.block1->local_bound, registration_cons.block2->local_bound, intersection_bbx);

		if (apply_intersection_filter)
			cfilter.get_cloud_pair_intersection(intersection_bbx, cloud_t_down, cloud_s_guess);

		Eigen::Matrix4d Trans_t_sg;
		double fitness_score;
		fitness_score = icp_registration(cloud_s_guess, cloud_t_down,
										 cloud_s_tran, Trans_t_sg, metrics, ce, te,
										 use_reciprocal_correspondence, use_trimmed_rejector,
										 max_iter_num, dis_thre_unit, neighbor_radius);

		if (fitness_score > fitness_score_thre)
			process_code = -3;
		else
			process_code = 1;

		if (apply_source_initial_guess)
			Trans_t_sg = Trans_t_sg * initial_guess;

		registration_cons.Trans1_2 = Trans_t_sg;

		pcl::PointCloud<PointT>().swap(*cloud_s_guess);
		pcl::PointCloud<PointT>().swap(*cloud_s_tran);
		pcl::PointCloud<PointT>().swap(*cloud_t_down);
		pcl::PointCloud<PointT>().swap(*cloud_s_down);

		return process_code;
	}

	//interface for the implement of fast ndt algorithm (https://github.com/koide3/ndt_omp)
	int omp_ndt(constraint_t &registration_cons, float ndt_resolution = 1.0, bool use_direct_search = true,
				Eigen::Matrix4d initial_guess = Eigen::Matrix4d::Identity(), bool apply_intersection_filter = true,
				float fitness_score_thre = 10.0)
	{
		CFilter<PointT> cfilter;

		int process_code = 0;

		std::chrono::steady_clock::time_point tic = std::chrono::steady_clock::now();

		typename pcl::PointCloud<PointT>::Ptr cloud_t_down(new pcl::PointCloud<PointT>);
		typename pcl::PointCloud<PointT>::Ptr cloud_s_down(new pcl::PointCloud<PointT>);
		typename pcl::PointCloud<PointT>::Ptr cloud_s_tran(new pcl::PointCloud<PointT>);

		registration_cons.block1->clone_cloud(cloud_t_down, true); //target
		registration_cons.block2->clone_cloud(cloud_s_down, true); //source

		bounds_t intersection_bbx, source_guess_bbx;
		bool apply_source_initial_guess = false;
		if (!initial_guess.isIdentity(1e-6))
		{
			//Transform the Source pointcloud
			pcl::transformPointCloudWithNormals(*cloud_s_down, *cloud_s_down, initial_guess);
			cfilter.get_cloud_bbx(cloud_s_down, source_guess_bbx);
			cfilter.get_intersection_bbx(registration_cons.block1->local_bound, source_guess_bbx, intersection_bbx);
			LOG(INFO) << "Apply initial guess transformation\n"
					  << initial_guess;
			apply_source_initial_guess = true;
		}
		else
			cfilter.get_intersection_bbx(registration_cons.block1->local_bound, registration_cons.block2->local_bound, intersection_bbx);

		if (apply_intersection_filter)
			cfilter.get_cloud_pair_intersection(intersection_bbx, cloud_t_down, cloud_s_down);

		std::chrono::steady_clock::time_point toc_1 = std::chrono::steady_clock::now();

		Eigen::Matrix4d Trans_t_sg;
		double fitness_score;

		int num_thread = omp_get_max_threads();
		std::pair<std::string, koide_reg::NeighborSearchMethod> search_method;
		if (use_direct_search)
			search_method = {"DIRECT7", koide_reg::DIRECT7};
		else
			search_method = {"KDTREE", koide_reg::KDTREE};

		typename koide_reg::NormalDistributionsTransform<Point_T, Point_T>::Ptr koide_ndt_omp(new koide_reg::NormalDistributionsTransform<Point_T, Point_T>());
		koide_ndt_omp->setResolution(ndt_resolution);
		koide_ndt_omp->setNumThreads(num_thread);
		koide_ndt_omp->setNeighborhoodSearchMethod(search_method.second);

		fitness_score = base_align(koide_ndt_omp, cloud_t_down, cloud_s_down, cloud_s_tran, Trans_t_sg);

		if (fitness_score > fitness_score_thre)
			process_code = -3;
		else
			process_code = 1;

		if (apply_source_initial_guess)
			Trans_t_sg = Trans_t_sg * initial_guess;

		registration_cons.Trans1_2 = Trans_t_sg;

		std::chrono::steady_clock::time_point toc_2 = std::chrono::steady_clock::now();
		std::chrono::duration<double> reg_prepare_time = std::chrono::duration_cast<std::chrono::duration<double>>(toc_1 - tic);
		std::chrono::duration<double> reg_time = std::chrono::duration_cast<std::chrono::duration<double>>(toc_2 - toc_1);

		LOG(INFO) << "Registration preparation done in [" << reg_prepare_time.count() * 1000.0 << "] ms.";
		LOG(INFO) << "Registration done in [" << reg_time.count() * 1000.0 << "] ms.";

		pcl::PointCloud<PointT>().swap(*cloud_s_tran);
		pcl::PointCloud<PointT>().swap(*cloud_t_down);
		pcl::PointCloud<PointT>().swap(*cloud_s_down);

		return process_code;
	}

	//interface for the implement of fast gicp algorithm (https://github.com/koide3/ndt_omp)
	int omp_gicp(constraint_t &registration_cons,
				 int max_iter_num = 20, float dis_thre_unit = 1.5, bool using_voxel_gicp = true, float voxel_size = 1.0,
				 Eigen::Matrix4d initial_guess = Eigen::Matrix4d::Identity(), bool apply_intersection_filter = false,
				 float fitness_score_thre = 10.0)
	{
		CFilter<PointT> cfilter;

		int process_code = 0;

		std::chrono::steady_clock::time_point tic = std::chrono::steady_clock::now();

		typename pcl::PointCloud<PointT>::Ptr cloud_t_down(new pcl::PointCloud<PointT>);
		typename pcl::PointCloud<PointT>::Ptr cloud_s_down(new pcl::PointCloud<PointT>);
		typename pcl::PointCloud<PointT>::Ptr cloud_s_tran(new pcl::PointCloud<PointT>);

		registration_cons.block1->clone_cloud(cloud_t_down, true); //target
		registration_cons.block2->clone_cloud(cloud_s_down, true); //source

		bounds_t intersection_bbx, source_guess_bbx;
		bool apply_source_initial_guess = false;
		if (!initial_guess.isIdentity(1e-6))
		{
			//Transform the Source pointcloud
			pcl::transformPointCloudWithNormals(*cloud_s_down, *cloud_s_down, initial_guess);
			cfilter.get_cloud_bbx(cloud_s_down, source_guess_bbx);
			cfilter.get_intersection_bbx(registration_cons.block1->local_bound, source_guess_bbx, intersection_bbx);
			LOG(INFO) << "Apply initial guess transformation\n"
					  << initial_guess;
			apply_source_initial_guess = true;
		}
		else
			cfilter.get_intersection_bbx(registration_cons.block1->local_bound, registration_cons.block2->local_bound, intersection_bbx);

		if (apply_intersection_filter)
			cfilter.get_cloud_pair_intersection(intersection_bbx, cloud_t_down, cloud_s_down);

		std::chrono::steady_clock::time_point toc_1 = std::chrono::steady_clock::now();
		std::chrono::duration<double> reg_prepare_time = std::chrono::duration_cast<std::chrono::duration<double>>(toc_1 - tic);
		LOG(INFO) << "Registration preparation done in [" << reg_prepare_time.count() * 1000.0 << "] ms.";

		Eigen::Matrix4d Trans_t_sg;
		double fitness_score;

		typename koide_reg::GeneralizedIterativeClosestPoint<PointT, PointT>::Ptr koide_gicp_omp(new koide_reg::GeneralizedIterativeClosestPoint<PointT, PointT>());
		typename koide_reg::FastVGICP<PointT, PointT>::Ptr koide_vgicp_omp(new koide_reg::FastVGICP<PointT, PointT>());

		koide_gicp_omp->setMaximumOptimizerIterations(max_iter_num);
		//koide_vgicp_omp->setMaximumOptimizerIterations(max_iter_num);
		koide_vgicp_omp->setResolution(voxel_size);
		koide_vgicp_omp->setNumThreads(omp_get_max_threads());

		if (using_voxel_gicp)
			fitness_score = base_align(koide_vgicp_omp, cloud_t_down, cloud_s_down, cloud_s_tran, Trans_t_sg);
		else
			fitness_score = base_align(koide_gicp_omp, cloud_t_down, cloud_s_down, cloud_s_tran, Trans_t_sg);

		if (fitness_score > fitness_score_thre)
			process_code = -3;
		else
			process_code = 1;

		if (apply_source_initial_guess)
			Trans_t_sg = Trans_t_sg * initial_guess;

		registration_cons.Trans1_2 = Trans_t_sg;

		std::chrono::steady_clock::time_point toc_2 = std::chrono::steady_clock::now();
		std::chrono::duration<double> reg_time = std::chrono::duration_cast<std::chrono::duration<double>>(toc_2 - toc_1);

		pcl::PointCloud<PointT>().swap(*cloud_s_tran);
		pcl::PointCloud<PointT>().swap(*cloud_t_down);
		pcl::PointCloud<PointT>().swap(*cloud_s_down);

		return process_code;
	}

	//--------------------------------------------------------------------------------------------------------------------------//
	//Multi-metrics Linear Least Square ICP (MULLS-ICP)
	//An efficient cross-template point cloud fine registration method
	//Author: Yue Pan
	//Outlines:
	//1.Preprocessing: Downsample the raw point cloud and classify it into several categories
	//  (Planar: Ground, Roof, Facade, Linear: Pillar, Beam, Sphere: Vertex), refer to the extract_semantic_pts function in 'filter.hpp'
	//2.Find correspondences within the same category with a trimmed strategy
	//3.Estimate the transformation that minimizes the weighted (x,y,z balanced) distance after applying the transformation,
	//  We use the point-to-point, point-to-line and point-to-plane distance metrics for Sphere, Linear and Planar points respectivly.
	//  This is sloved very efficiently by Linear Least Square (take tx,ty,tz,roll ,pitch,yaw as unknowns)
	//4.Update the Source Point Cloud and keep iterating
	//5.Till converge, Output the 4*4 final transformation matrix and the 6*6 information matrix
	//TODO polish the codes --> delete some unnecssary codes and also encapsulate some codes in private functions
	int mm_lls_icp(constraint_t &registration_cons, // cblock_1 (target point cloud), cblock_2 (source point cloud)
				   int max_iter_num = 20, float dis_thre_unit = 1.5,
				   float converge_translation = 0.002, float converge_rotation_d = 0.01,
				   float dis_thre_min = 0.4, float dis_thre_update_rate = 1.1, std::string used_feature_type = "111110",
				   std::string weight_strategy = "1101", float z_xy_balanced_ratio = 1.0,
				   float pt2pt_residual_window = 0.1, float pt2pl_residual_window = 0.1, float pt2li_residual_window = 0.1,
				   Eigen::Matrix4d initial_guess = Eigen::Matrix4d::Identity(), //used_feature_type (1: on, 0: off, order: ground, pillar, facade, beam, roof, vetrex)
				   bool apply_intersection_filter = true, bool apply_motion_undistortion_while_registration = false,
				   bool normal_shooting_on = false, float normal_bearing = 45.0, bool use_more_points = false,
				   bool keep_less_source_points = false, float sigma_thre = 0.5, float min_neccessary_corr_ratio = 0.03, float max_bearable_rotation_d = 45.0) //sigma_thre means the maximum threshold of the posterior standar deviation of the registration LLS (unit:m)

	{
		//LOG(INFO) << "Begin registration";
		std::chrono::steady_clock::time_point tic = std::chrono::steady_clock::now();

		CFilter<PointT> cfilter;

		//code that indicate the status of the registration
		//successful registration                ---   process_code= 1
		//too large tran or rot for one iter.    ---   process_code=-1
		//too few correspondences (ratio)        ---   process_code=-2
		//final standard deviation is too large  ---   process_code=-3
		int process_code = 0;

		//at least ${min_neccessary_corr_ratio} source points should have a match
		int min_total_corr_num = 40;
		int min_neccessary_corr_num = 20;

		float neccessary_corr_ratio = 1.0; //posterior unground points overlapping ratio

		Eigen::Matrix4d transformationS2T = Eigen::Matrix4d::Identity();
		Eigen::Matrix4d inv_init_guess_mat;
		Matrix6d cofactor_matrix;
		Matrix6d information_matrix;
		double sigma_square_post;
		Eigen::Matrix4d TempTran = Eigen::Matrix4d::Identity();
		Vector6d transform_x;

		cofactor_matrix.setIdentity();
		information_matrix.setIdentity();

		float dis_thre_ground = dis_thre_unit;
		float dis_thre_facade = dis_thre_unit;
		float dis_thre_roof = dis_thre_unit;
		float dis_thre_pillar = dis_thre_unit;
		float dis_thre_beam = dis_thre_unit;
		float dis_thre_vertex = dis_thre_unit;

		float max_bearable_translation = 2.0 * dis_thre_unit;
		float converge_rotation = converge_rotation_d / 180.0 * M_PI;
		float max_bearable_rotation = max_bearable_rotation_d / 180.0 * M_PI;

		//clone the point cloud
		typename pcl::PointCloud<PointT>::Ptr pc_ground_sc(new pcl::PointCloud<PointT>);
		typename pcl::PointCloud<PointT>::Ptr pc_ground_tc(new pcl::PointCloud<PointT>);
		typename pcl::PointCloud<PointT>::Ptr pc_pillar_sc(new pcl::PointCloud<PointT>);
		typename pcl::PointCloud<PointT>::Ptr pc_pillar_tc(new pcl::PointCloud<PointT>);
		typename pcl::PointCloud<PointT>::Ptr pc_beam_sc(new pcl::PointCloud<PointT>);
		typename pcl::PointCloud<PointT>::Ptr pc_beam_tc(new pcl::PointCloud<PointT>);
		typename pcl::PointCloud<PointT>::Ptr pc_facade_sc(new pcl::PointCloud<PointT>);
		typename pcl::PointCloud<PointT>::Ptr pc_facade_tc(new pcl::PointCloud<PointT>);
		typename pcl::PointCloud<PointT>::Ptr pc_roof_sc(new pcl::PointCloud<PointT>);
		typename pcl::PointCloud<PointT>::Ptr pc_roof_tc(new pcl::PointCloud<PointT>);
		typename pcl::PointCloud<PointT>::Ptr pc_vertex_sc(new pcl::PointCloud<PointT>);
		typename pcl::PointCloud<PointT>::Ptr pc_vertex_tc(new pcl::PointCloud<PointT>);

		registration_cons.block1->clone_feature(pc_ground_tc, pc_pillar_tc, pc_beam_tc, pc_facade_tc, pc_roof_tc, pc_vertex_tc, false);			   //target (would not change any more during the iteration)
		registration_cons.block2->clone_feature(pc_ground_sc, pc_pillar_sc, pc_beam_sc, pc_facade_sc, pc_roof_sc, pc_vertex_sc, !use_more_points); //source (point cloud used for a specific iteration)

		batch_transform_feature_points(pc_ground_sc, pc_pillar_sc, pc_beam_sc, pc_facade_sc, pc_roof_sc, pc_vertex_sc, initial_guess); //apply initial guess
        
		//Filter the point cloud laying far away from the overlapping region
		if (apply_intersection_filter && !apply_motion_undistortion_while_registration)
			intersection_filter(registration_cons, pc_ground_tc, pc_pillar_tc, pc_beam_tc, pc_facade_tc, pc_roof_tc, pc_vertex_tc,
								pc_ground_sc, pc_pillar_sc, pc_beam_sc, pc_facade_sc, pc_roof_sc, pc_vertex_sc);

		//Downsample source cloud if its point number is larger than target's
		if (keep_less_source_points && !apply_motion_undistortion_while_registration)
			keep_less_source_pts(pc_ground_tc, pc_pillar_tc, pc_beam_tc, pc_facade_tc, pc_roof_tc, pc_vertex_tc,
								 pc_ground_sc, pc_pillar_sc, pc_beam_sc, pc_facade_sc, pc_roof_sc, pc_vertex_sc);

		int source_feature_points_count = 0;
		if (used_feature_type[1] == '1')
			source_feature_points_count += pc_pillar_sc->points.size();
		if (used_feature_type[2] == '1')
			source_feature_points_count += pc_facade_sc->points.size();
		if (used_feature_type[3] == '1')
			source_feature_points_count += pc_beam_sc->points.size();

		//Correspondence
		boost::shared_ptr<pcl::Correspondences> corrs_ground(new pcl::Correspondences),corrs_pillar(new pcl::Correspondences),corrs_beam(new pcl::Correspondences),corrs_facade(new pcl::Correspondences),corrs_roof(new pcl::Correspondences),corrs_vertex(new pcl::Correspondences);

		std::chrono::steady_clock::time_point tic_kdtree_iter = std::chrono::steady_clock::now();

		//build kd-tree in target point cloud
#pragma omp parallel sections
		{
#pragma omp section
			{
				if (used_feature_type[0] == '1' && pc_ground_tc->size() > 0)
					registration_cons.block1->tree_ground->setInputCloud(pc_ground_tc);
				if (used_feature_type[4] == '1' && pc_roof_tc->size() > 0)
					registration_cons.block1->tree_roof->setInputCloud(pc_roof_tc);
			}
#pragma omp section
			{
				if (used_feature_type[1] == '1' && pc_pillar_tc->size() > 0)
					registration_cons.block1->tree_pillar->setInputCloud(pc_pillar_tc);
				if (used_feature_type[3] == '1' && pc_beam_tc->size() > 0)
					registration_cons.block1->tree_beam->setInputCloud(pc_beam_tc);
			}
#pragma omp section
			{
				if (used_feature_type[2] == '1' && pc_facade_tc->size() > 0)
					registration_cons.block1->tree_facade->setInputCloud(pc_facade_tc);
			}
		}
		if (used_feature_type[5] == '1' && pc_vertex_tc->size() > 0)
			registration_cons.block1->tree_vertex->setInputCloud(pc_vertex_tc);

		std::chrono::steady_clock::time_point toc_kdtree_iter = std::chrono::steady_clock::now();
		std::chrono::duration<double> build_kdtree_time = std::chrono::duration_cast<std::chrono::duration<double>>(toc_kdtree_iter - tic_kdtree_iter);
		LOG(INFO) << "Build Kdtree done in [" << build_kdtree_time.count() * 1000.0 << "] ms";

		//Iteration Loop
		for (int i = 0; i < max_iter_num; i++)
		{
			std::chrono::steady_clock::time_point tic_iter = std::chrono::steady_clock::now();
			// Target (Dense): Cloud1,  Source (Sparse): Cloud2
			if (i == 0)
				LOG(INFO) << "Apply initial guess transformation\n"
						  << initial_guess;

			//apply motion undistortion
			inv_init_guess_mat = initial_guess.inverse();
			if (apply_motion_undistortion_while_registration && i == 0) //do undistortion at the first iteration
			{
				cfilter.batch_apply_motion_compensation(registration_cons.block2->pc_ground_down, registration_cons.block2->pc_pillar_down, registration_cons.block2->pc_beam_down,
														registration_cons.block2->pc_facade_down, registration_cons.block2->pc_roof_down, registration_cons.block2->pc_vertex,
														pc_ground_sc, pc_pillar_sc, pc_beam_sc, pc_facade_sc, pc_roof_sc, pc_vertex_sc, inv_init_guess_mat); //for source point cloud

				//apply initial_guess
				//Transform the Source pointcloud
				batch_transform_feature_points(pc_ground_sc, pc_pillar_sc, pc_beam_sc, pc_facade_sc, pc_roof_sc, pc_vertex_sc, initial_guess);
			}
			else
				batch_transform_feature_points(pc_ground_sc, pc_pillar_sc, pc_beam_sc, pc_facade_sc, pc_roof_sc, pc_vertex_sc, TempTran);

			std::chrono::steady_clock::time_point toc_1_iter = std::chrono::steady_clock::now();
			std::chrono::duration<double> update_time = std::chrono::duration_cast<std::chrono::duration<double>>(toc_1_iter - tic_iter);
			LOG(INFO) << "Source point cloud updating done in [" << update_time.count() * 1000.0 << "] ms for iteration [" << i << "]";

			// First, find coorespondences (nearest neighbor [for line] or normal shooting [for plane])
			// We need to check the normal compatibility of the plane correspondences
			#pragma omp parallel sections
			{
				#pragma omp section
				{
					if (used_feature_type[0] == '1' && pc_ground_sc->size() > 0)
						determine_corres(pc_ground_sc, pc_ground_tc, registration_cons.block1->tree_ground, dis_thre_ground, corrs_ground, normal_shooting_on, true, normal_bearing); //Check normal vector
				}			
				#pragma omp section
				{
					if (used_feature_type[1] == '1' && pc_pillar_sc->size() > 0)
						determine_corres(pc_pillar_sc, pc_pillar_tc, registration_cons.block1->tree_pillar, dis_thre_pillar, corrs_pillar, false, true, normal_bearing); //Check primary vector
				}
				#pragma omp section
				{
					if (used_feature_type[2] == '1' && pc_facade_sc->size() > 0)
						determine_corres(pc_facade_sc, pc_facade_tc, registration_cons.block1->tree_facade, dis_thre_facade, corrs_facade, normal_shooting_on, true, normal_bearing); //Check normal vector

					if (used_feature_type[3] == '1' && pc_beam_sc->size() > 0)
						determine_corres(pc_beam_sc, pc_beam_tc, registration_cons.block1->tree_beam, dis_thre_beam, corrs_beam, false, true, normal_bearing); //Check primary vector
				}
			}
			if (used_feature_type[4] == '1' && pc_roof_sc->size() > 0)
				determine_corres(pc_roof_sc, pc_roof_tc, registration_cons.block1->tree_roof, dis_thre_roof, corrs_roof, normal_shooting_on, true, normal_bearing); //Check normal vector
			if (used_feature_type[5] == '1' && pc_vertex_sc->size() > 0)
				determine_corres(pc_vertex_sc, pc_vertex_tc, registration_cons.block1->tree_vertex, dis_thre_vertex, corrs_vertex, false, false);

			std::chrono::steady_clock::time_point toc_2_iter = std::chrono::steady_clock::now();
			std::chrono::duration<double> corre_search_time = std::chrono::duration_cast<std::chrono::duration<double>>(toc_2_iter - toc_1_iter);

			LOG(INFO) << "Used correspondences [G: " << (*corrs_ground).size() << " P: " << (*corrs_pillar).size() << " B: " << (*corrs_beam).size()
					  << " F: " << (*corrs_facade).size() << " R: " << (*corrs_roof).size() << " V: " << (*corrs_vertex).size() << " ].";
			LOG(INFO) << "Correspondence searching done in [" << corre_search_time.count() * 1000.0 << "] ms for iteration [" << i << "]";

			int total_corr_num = (*corrs_ground).size() + (*corrs_pillar).size() + (*corrs_beam).size() + (*corrs_facade).size() + (*corrs_roof).size() + (*corrs_vertex).size();
			int neccessary_corr_num = (*corrs_pillar).size() + (*corrs_beam).size() + (*corrs_facade).size();
			neccessary_corr_ratio = 1.0 * neccessary_corr_num / source_feature_points_count;

			if (total_corr_num < min_total_corr_num || neccessary_corr_num < min_neccessary_corr_num || neccessary_corr_ratio < min_neccessary_corr_ratio)
			{
				process_code = -2;
				TempTran.setIdentity();
				LOG(WARNING) << "Too few neccessary correspondences";
				break;
			}

			//update (decrease correspondence threshold)
			update_corr_dist_thre(dis_thre_ground, dis_thre_pillar, dis_thre_beam, dis_thre_facade, dis_thre_roof, dis_thre_vertex,
								  dis_thre_update_rate, dis_thre_min);

			//Estimate Transformation
			multi_metrics_lls_tran_estimation(pc_ground_sc, pc_ground_tc, corrs_ground,
											  pc_pillar_sc, pc_pillar_tc, corrs_pillar,
											  pc_beam_sc, pc_beam_tc, corrs_beam,
											  pc_facade_sc, pc_facade_tc, corrs_facade,
											  pc_roof_sc, pc_roof_tc, corrs_roof,
											  pc_vertex_sc, pc_vertex_tc, corrs_vertex,
											  transform_x, cofactor_matrix,
											  i, weight_strategy, z_xy_balanced_ratio,
											  pt2pt_residual_window, pt2pl_residual_window, pt2li_residual_window);

			//About weight strategy:
			//0000: equal weight, // 1000: x,y,z balanced weight, //0100: residual weight, //0010: distance weight (adaptive), //0001: intensity weight	
			//....  //1111: all in
			
			//transform_x [6x1]: tx, ty, tz, roll, pitch, yaw --> Transformation matrix TempTran [4x4]
			construct_trans_a(transform_x(0), transform_x(1), transform_x(2), transform_x(3), transform_x(4), transform_x(5), TempTran);

			LOG(INFO) << "tx(m):" << transform_x(0) << ",ty(m):" << transform_x(1) << ",tz(m):" << transform_x(2)
					  << ",roll(deg):" << transform_x(3) * 180.0 / M_PI << ",pitch(deg):"
					  << transform_x(4) * 180.0 / M_PI << ",yaw(deg):" << transform_x(5) * 180.0 / M_PI;

			std::chrono::steady_clock::time_point toc_3_iter = std::chrono::steady_clock::now();
			std::chrono::duration<double> tran_estimation_time = std::chrono::duration_cast<std::chrono::duration<double>>(toc_3_iter - toc_2_iter);

			LOG(INFO) << "Transformation estimation done in [" << tran_estimation_time.count() * 1000.0 << "] ms for iteration [" << i << "]";

			Eigen::Vector3d ts(TempTran(0, 3), TempTran(1, 3), TempTran(2, 3));
			Eigen::AngleAxisd rs(TempTran.block<3, 3>(0, 0));

			//LOG(INFO) << "translation(m):" << ts.norm() << " , rotation(deg):" << std::abs(rs.angle()) / M_PI * 180.0;
			if (ts.norm() > max_bearable_translation || std::abs(rs.angle()) > max_bearable_rotation)
			{
				process_code = -1;
				TempTran.setIdentity();
				LOG(WARNING) << "Too large translation or rotation for one iteration";
				break;
			}

			//Judge converged or not
			if (i == max_iter_num - 1 || (i > 2 && ts.norm() < converge_translation && std::abs(rs.angle()) < converge_rotation))
			{
				LOG(INFO) << "Converged. Calculate statistic information, break out.";

				//Calculate converged residual and information matrix
				if (get_multi_metrics_lls_residual(pc_ground_sc, pc_ground_tc, corrs_ground,
												   pc_pillar_sc, pc_pillar_tc, corrs_pillar,
												   pc_beam_sc, pc_beam_tc, corrs_beam,
												   pc_facade_sc, pc_facade_tc, corrs_facade,
												   pc_roof_sc, pc_roof_tc, corrs_roof,
												   pc_vertex_sc, pc_vertex_tc, corrs_vertex,
												   transform_x, sigma_square_post, sigma_thre))
					process_code = 1;
				else
				{
					process_code = -3;
					LOG(WARNING) << "The posterior standard deviation of estimated value is too large";
				}

				//Calculate the information matrix
				//Background Knowledge:
				//For the adjustment problem : v=Ax-b
				//A is the design matrix, b is the observation matrix, x is the vector for estimation, P is the original weight matrix
				//Note that the information matrix is the inverse of the variance-covariance matrix ( Dxx ^ -1 ) of the estimated value (x,y,z,roll,pitch,yaw)
				//Dxx = Qxx * (sigma_post)^2 = ATPA * VTPV/(n-t) and Qxx = ATPA	
				//sigma_post^2 = (vTPv)/(n-t) = VTPV/(n-t)
				//v is the residual vector, n is the number of the observation equations and t is the number of the neccessary observation
				//information_matrix = (Dxx) ^(-1) =  (Qxx * (sigma_post)^2)^(-1) = ATPA / (sigma_post)^2  = (n-t)/(VTPV)*(ATPA)
				//because cofactor_matrix =  (ATPA)^(-1), so we get
				information_matrix = (1.0 / sigma_square_post) * cofactor_matrix.inverse();

				Matrix6d vc_matrix = information_matrix.inverse(); //variance covariance matrix

				LOG(INFO) << "Standard deviation of the registration result: \n"
						  << "tx(m):" << sqrt(vc_matrix(0, 0)) << " ,ty(m):" << sqrt(vc_matrix(1, 1)) << " ,tz(m):" << sqrt(vc_matrix(2, 2));
				//LOG(INFO)<< "\nroll(degree):" << 180.0 / M_PI * sqrt(vc_matrix(3, 3)) << " ,pitch(degree):" << 180.0 / M_PI * sqrt(vc_matrix(4, 4)) << " ,yaw(degree):" << 180.0 / M_PI * sqrt(vc_matrix(5, 5));

				break; 
			}
			//Update the source pointcloud
			//batch_transform_feature_points(pc_ground_sc, pc_pillar_sc, pc_beam_sc, pc_facade_sc, pc_roof_sc, pc_vertex_sc, TempTran);

			//Update the transformation matrix till-now
			initial_guess = TempTran * initial_guess;
		}

		initial_guess = TempTran * initial_guess; //Update the last iteration's transformation

		registration_cons.Trans1_2 = initial_guess;
#if 0
			//deprecated, only consider this when you are deal with the point cloud in world coordinate system such as WGS84
			//Now the registration transformation is in the local shifted coordinate system, not in the global-shifted map coordinate system
			//the transaltion would change
			//R21m=R21l, t21m=(R21l-I)*tlm+t21l
			Eigen::Matrix3d Idenity3d;
			Idenity3d.setIdentity();
			registration_cons.Trans1_2.block<3, 1>(0, 3) =
				(registration_cons.Trans1_2.block<3, 3>(0, 0) - Idenity3d) * local_shift_mat.block<3, 1>(0, 3) + registration_cons.Trans1_2.block<3, 1>(0, 3);
#endif

		//Multiple evalualtion metrics
		registration_cons.information_matrix = information_matrix; //Final information matrix
		registration_cons.sigma = std::sqrt(sigma_square_post);	//Final unit weight standard deviation
		registration_cons.confidence = neccessary_corr_ratio;	  //posterior unground points overlapping ratio

		LOG(INFO) << "The posterior overlapping ratio is [" << registration_cons.confidence << "]";

		std::chrono::steady_clock::time_point toc = std::chrono::steady_clock::now();
		std::chrono::duration<double> reg_time_used = std::chrono::duration_cast<std::chrono::duration<double>>(toc - tic);

		LOG(INFO) << "Registration done in [" << reg_time_used.count() * 1000.0 << "] ms.";
		LOG(INFO) << "Final tran. matrix:\n"
				  << registration_cons.Trans1_2;

		//free mannually
		corrs_ground.reset(new pcl::Correspondences);
		corrs_pillar.reset(new pcl::Correspondences);
		corrs_beam.reset(new pcl::Correspondences);
		corrs_facade.reset(new pcl::Correspondences);
		corrs_roof.reset(new pcl::Correspondences);
		corrs_vertex.reset(new pcl::Correspondences);

		return process_code;
	}

	// This is for the comparison between LeGO-LOAM , which used the two step LM. We'd like to try the two step LLS for the same task
	bool lls_icp_3dof_ground(constraint_t &registration_cons, int max_iter_num = 20, float dis_thre_unit = 1.5, float converge_translation = 0.002,
							 float converge_rotation_d = 0.01, float dis_thre_min = 0.4, float dis_thre_update_rate = 1.1, std::string weight_strategy = "1111",
							 Eigen::Matrix4d initial_guess = Eigen::Matrix4d::Identity(), bool keep_less_source_points = false, float max_bearable_rotation_d = 10.0)
	{
		LOG(INFO) << "Begin registration";
		std::chrono::steady_clock::time_point tic = std::chrono::steady_clock::now();

		CFilter<PointT> cfilter;

		int process_code = 0;
		//code that indicate the status of the registration
		//successful registration  ---   process_code=1
		//too large tran or rot for one iter.  ---  process_code=-1
		//too few correspondences  ---   process_code=-2
		//final standard deviation is too large  ---  process_code=-3

		int min_total_corr_num = 100;
		int down_rate = 3;

		Eigen::Matrix4d transformationS2T = Eigen::Matrix4d::Identity();
		Eigen::Matrix4d TempTran;
		Eigen::Vector3d transform_x_3dof;
		Eigen::Matrix3d cofactor_matrix;
		double sigma_square_post;

		TempTran.setIdentity();

		float dis_thre_ground = dis_thre_unit;

		float max_bearable_translation = 2.0 * dis_thre_unit;
		float converge_rotation = converge_rotation_d / 180.0 * M_PI;
		float max_bearable_rotation = max_bearable_rotation_d / 180.0 * M_PI;

		//clone the point cloud
		typename pcl::PointCloud<PointT>::Ptr pc_ground_sc(new pcl::PointCloud<PointT>);
		typename pcl::PointCloud<PointT>::Ptr pc_ground_tc(new pcl::PointCloud<PointT>);

		*pc_ground_sc = *registration_cons.block2->pc_ground_down;
		*pc_ground_tc = *registration_cons.block1->pc_ground;

		pcl::transformPointCloudWithNormals(*pc_ground_sc, *pc_ground_sc, initial_guess);

		if (keep_less_source_points)
			cfilter.random_downsample_pcl(pc_ground_sc, (int)(pc_ground_tc->points.size() / down_rate));

		//Correspondence
		boost::shared_ptr<pcl::Correspondences> corrs_ground_f(new pcl::Correspondences);
		typename pcl::search::KdTree<PointT>::Ptr tree_ground_tc(new pcl::search::KdTree<PointT>);
		tree_ground_tc->setInputCloud(pc_ground_tc);

		//Iteration Loop
		for (int i = 0; i < max_iter_num; i++)
		{
			std::chrono::steady_clock::time_point tic_iter = std::chrono::steady_clock::now();

			// Target (Dense): Cloud1,  Source (Sparse): Cloud2

			// First, find coorespondences
			// We need to check the normal compatibility of the plane correspondences
			determine_corres(pc_ground_sc, pc_ground_tc, tree_ground_tc, dis_thre_ground, corrs_ground_f, false, true); //Check normal

			std::chrono::steady_clock::time_point toc_1_iter = std::chrono::steady_clock::now();

			std::chrono::duration<double> corre_search_time = std::chrono::duration_cast<std::chrono::duration<double>>(toc_1_iter - tic_iter);

			LOG(INFO) << "Used correspondences # [G: " << (*corrs_ground_f).size() << "].";
			LOG(INFO) << "Correspondence searching done in [" << corre_search_time.count() * 1000.0 << "] ms for iteration [" << i << "]";

			if ((*corrs_ground_f).size() < min_total_corr_num)
			{
				process_code = -2;
				LOG(WARNING) << "Too few neccessary correspondences";
				break;
			}

			//update (decrease correspondence threshold)
			dis_thre_ground = max_(1.0 * dis_thre_ground / dis_thre_update_rate, dis_thre_min);

			//Estimate Transformation

			ground_3dof_lls_tran_estimation(pc_ground_sc, pc_ground_tc, corrs_ground_f, transform_x_3dof, cofactor_matrix, i, weight_strategy);

			construct_trans_a(0, 0, transform_x_3dof(2), transform_x_3dof(0), transform_x_3dof(1), 0, TempTran);

			std::chrono::steady_clock::time_point toc_2_iter = std::chrono::steady_clock::now();
			std::chrono::duration<double> tran_estimation_time = std::chrono::duration_cast<std::chrono::duration<double>>(toc_2_iter - toc_1_iter);

			LOG(INFO) << "Transformation estimation done in [" << tran_estimation_time.count() * 1000.0 << "] ms for iteration [" << i << "]";
			LOG(INFO) << "Transformation matrix of iteration [" << i << "]\n"
					  << TempTran;

			Eigen::Vector3d ts(TempTran(0, 3), TempTran(1, 3), TempTran(2, 3));
			Eigen::AngleAxisd rs(TempTran.block<3, 3>(0, 0));

			LOG(INFO) << "translation(m):" << ts.norm() << " , rotation(deg):" << std::abs(rs.angle()) / M_PI * 180.0;

			if (ts.norm() > max_bearable_translation || std::abs(rs.angle()) > max_bearable_rotation)
			{
				process_code = -1;
				TempTran.setIdentity();
				LOG(WARNING) << "Too large translation or rotation for one iteration";
				break;
			}

			//Judge converged or not
			if (i == max_iter_num - 1 || (i > 2 && ts.norm() < converge_translation && std::abs(rs.angle()) < converge_rotation))
			{
				LOG(INFO) << "Converged. Break out.";
				process_code = 1;

				break; //OUT
			}

			//Update the Source pointcloud
			pcl::transformPointCloudWithNormals(*pc_ground_sc, *pc_ground_sc, TempTran);

			//Update till-now transformation matrix
			transformationS2T = TempTran * transformationS2T;

			std::chrono::steady_clock::time_point toc_3_iter = std::chrono::steady_clock::now();
			std::chrono::duration<double> update_time = std::chrono::duration_cast<std::chrono::duration<double>>(toc_3_iter - toc_2_iter);
			LOG(INFO) << "Source point cloud updating done in [" << update_time.count() * 1000.0 << "] ms for iteration [" << i << "]";
		}

		transformationS2T = TempTran * transformationS2T; //Update the last iteration's transformation

		//get the final transformation in shifted local system (2 -> Source , 1 -> Target)
		transformationS2T = transformationS2T * initial_guess; //with initial guess 2(S) --> 2' (S') -->  1(T)  T12= T12' * T2'2 , T2'2 is the initial_guess transformation

		registration_cons.Trans1_2 = transformationS2T;

		std::chrono::steady_clock::time_point toc = std::chrono::steady_clock::now();
		std::chrono::duration<double> reg_time_used = std::chrono::duration_cast<std::chrono::duration<double>>(toc - tic);

		LOG(INFO) << "Registration done in [" << reg_time_used.count() * 1000.0 << "] ms.";
		LOG(INFO) << "Final tran. matrix:\n"
				  << registration_cons.Trans1_2;

		return process_code;
	}

	bool mm_lls_icp_4dof_global(constraint_t &registration_con, float heading_step_d,
								int max_iter_num = 20, float dis_thre_unit = 1.5, float converge_translation = 0.005, float converge_rotation_d = 0.05,
								float dis_thre_min = 0.5, float dis_thre_update_rate = 1.05, float max_bearable_rotation_d = 15.0)
	{

		cloudblock_t rot_block2; //rotated block2
		constraint_t rot_con;
		float current_best_sigma = FLT_MAX;
		float current_best_score = 0; //score =  confidence / sigma, (the larger, the better)
		float current_best_heading_d;

		float heading_d = 0.0;
		float heading_rad;
		int count = 0;

		Eigen::Matrix4d temp_rot_z_mat, tran_mat_g2s, tran_mat_s2g;
		bool successful_reg = false;

		rot_con.block1 = registration_con.block1; //bbx is also included
		rot_con.block2 = registration_con.block2; //bbx is also included

		std::chrono::steady_clock::time_point tic = std::chrono::steady_clock::now();

		while (heading_d < 360.0)
		{
			rot_block2.init();

			heading_rad = heading_d * M_PI / 180.0;

			LOG(WARNING) << "Check the heading angle for trial [" << heading_d << "] degree";

			//Rotation should be apply to the points in station centric coordinate system
			//GeoCS -> SCS -> rot -> GeoCS

			temp_rot_z_mat = Eigen::Matrix4d::Identity(4, 4);
			tran_mat_g2s = Eigen::Matrix4d::Identity(4, 4);
			tran_mat_s2g = Eigen::Matrix4d::Identity(4, 4);

			temp_rot_z_mat(0, 0) = cos(heading_rad);
			temp_rot_z_mat(0, 1) = sin(heading_rad);
			temp_rot_z_mat(1, 0) = -sin(heading_rad);
			temp_rot_z_mat(1, 1) = cos(heading_rad);

			tran_mat_g2s(0, 3) = -registration_con.block2->local_station.x;
			tran_mat_g2s(1, 3) = -registration_con.block2->local_station.y;
			tran_mat_g2s(2, 3) = -registration_con.block2->local_station.z;

			tran_mat_s2g(0, 3) = registration_con.block2->local_station.x;
			tran_mat_s2g(1, 3) = registration_con.block2->local_station.y;
			tran_mat_s2g(2, 3) = registration_con.block2->local_station.z;

			temp_rot_z_mat = tran_mat_s2g * temp_rot_z_mat * tran_mat_g2s;

			LOG(INFO) << "Heading rotation matrix" << std::endl
					  << temp_rot_z_mat;

			if (mm_lls_icp(rot_con, max_iter_num, dis_thre_unit, converge_translation,
						   converge_translation, dis_thre_min,
						   dis_thre_update_rate, "111110", "1001", 1.0, 0.1, 0.1, 0.1, temp_rot_z_mat) > 0)
			{
				float cur_score = rot_con.confidence / rot_con.sigma;
				if (cur_score > current_best_score)
				{
					registration_con.Trans1_2 = rot_con.Trans1_2;
					registration_con.sigma = rot_con.sigma;
					registration_con.information_matrix = rot_con.information_matrix;
					registration_con.confidence = rot_con.confidence;

					current_best_score = cur_score;
					current_best_heading_d = heading_d;
				}
				successful_reg = true;
			}

			heading_d += heading_step_d;

			count++;
		}

		std::chrono::steady_clock::time_point toc = std::chrono::steady_clock::now();
		std::chrono::duration<double> trialing_time = std::chrono::duration_cast<std::chrono::duration<double>>(toc - tic);
		LOG(INFO) << "Global registration done in [" << trialing_time.count() << "] s with " << count << " trials";

		if (!successful_reg)
		{
			LOG(WARNING) << "Failed to find a good enough registration";
			return false;
		}

		LOG(INFO) << "Aprroximate heading angle (degree): " << current_best_heading_d;
		LOG(INFO) << "square of the posterior standard deviation (m^2): " << registration_con.sigma;
		LOG(INFO) << "posterior overlapping ratio: " << registration_con.confidence;
		LOG(INFO) << "registration score: " << current_best_score;
		LOG(INFO) << "Final transformation matrix from Block_2 to Block_1:" << std::endl
				  << registration_con.Trans1_2;

		return true;
	}

  protected:
  private:
	void batch_transform_feature_points(typename pcl::PointCloud<PointT>::Ptr pc_ground, typename pcl::PointCloud<PointT>::Ptr pc_pillar,
										typename pcl::PointCloud<PointT>::Ptr pc_beam, typename pcl::PointCloud<PointT>::Ptr pc_facade,
										typename pcl::PointCloud<PointT>::Ptr pc_roof, typename pcl::PointCloud<PointT>::Ptr pc_vertex,
										Eigen::Matrix4d &Tran)
	{
		pcl::transformPointCloudWithNormals(*pc_ground, *pc_ground, Tran);
		pcl::transformPointCloudWithNormals(*pc_pillar, *pc_pillar, Tran);
		pcl::transformPointCloudWithNormals(*pc_beam, *pc_beam, Tran);
		pcl::transformPointCloudWithNormals(*pc_facade, *pc_facade, Tran);
		pcl::transformPointCloudWithNormals(*pc_roof, *pc_roof, Tran);
		pcl::transformPointCloudWithNormals(*pc_vertex, *pc_vertex, Tran);
	}

	//Time complexity of kdtree (in this case, the target point cloud [n points] is used for construct the tree while each point in source point cloud acts as a query point)
	//build tree: O(nlogn) ---> so it's better to build the tree only once
	//searching 1-nearest neighbor: O(logn) in average ---> so we can bear a larger number of target points
	bool determine_corres(typename pcl::PointCloud<PointT>::Ptr &Source_Cloud, typename pcl::PointCloud<PointT>::Ptr &Target_Cloud,
						  const typename pcl::search::KdTree<PointT>::Ptr &target_kdtree, float dis_thre,
						  boost::shared_ptr<pcl::Correspondences> &Corr_f, bool normal_shooting_on, bool normal_check = true,
						  float angle_thre_degree = 40, bool duplicate_check = true, int K_filter_distant_point = 500)
	{
		int K_min = 3;
		float filter_dis_times = 2.5;
		int normal_shooting_candidate_count = 10;

		typename pcl::registration::CorrespondenceEstimation<PointT, PointT> corr_est; //for nearest neighbor searching

		//CorrespondenceEstimationNormalShooting computes correspondences as points in the target cloud which have minimum distance to normals computed on the input cloud
		typename pcl::registration::CorrespondenceEstimationNormalShooting<PointT, PointT, PointT> corr_est_ns; //for normal shooting searching

		pcl::registration::CorrespondenceRejectorDistance corr_rej_dist;

		// Add a median distance rejector (deprecated)
		// pcl::registration::CorrespondenceRejectorMedianDistance::Ptr corr_rej_med (new pcl::registration::CorrespondenceRejectorMedianDistance);
		// rej_med->setMedianFactor (4.0);
		// reg.addCorrespondenceRejector (rej_med);

		boost::shared_ptr<pcl::Correspondences> Corr(new pcl::Correspondences);

		typename pcl::PointCloud<PointT>::Ptr Source_Cloud_f(new pcl::PointCloud<PointT>);
		typename pcl::PointCloud<PointT>::Ptr Target_Cloud_f(new pcl::PointCloud<PointT>); //target point cloud would never change

		if (Source_Cloud->points.size() >= K_min &&
			Target_Cloud->points.size() >= K_min)
		{
			if (normal_shooting_on) // Normal Shooting
			{
				corr_est_ns.setInputSource(Source_Cloud);
				corr_est_ns.setInputTarget(Target_Cloud);
				corr_est_ns.setSourceNormals(Source_Cloud);
				corr_est_ns.setSearchMethodTarget(target_kdtree, true);					  //saving the time of rebuilding kd-tree
				corr_est_ns.setKSearch(normal_shooting_candidate_count);				  // Among the K nearest neighbours find the one with minimum perpendicular distance to the normal
				corr_est_ns.determineCorrespondences(*Corr, filter_dis_times * dis_thre); //base on KDtreeNSearch
																						  //corr_est_ns.determineReciprocalCorrespondences(*Corr);
			}
			else //Nearest Neighbor
			{
				corr_est.setInputCloud(Source_Cloud);
				corr_est.setInputTarget(Target_Cloud);
				corr_est.setSearchMethodTarget(target_kdtree, true);				   //saving the time of rebuilding kd-tree
				corr_est.determineCorrespondences(*Corr, filter_dis_times * dis_thre); //base on KDtreeNSearch
																					   //corr_est.determineReciprocalCorrespondences(*Corr);
			}

			// std::chrono::steady_clock::time_point toc = std::chrono::steady_clock::now();
			// std::chrono::duration<double> determine_corr_time = std::chrono::duration_cast<std::chrono::duration<double>>(toc - tic);
			// LOG(WARNING) << "Correspondence time 1: [" << determine_corr_time.count() * 1000 << "] ms";

			//Filter outlier source points (they would not appear throughout the registration anymore)
#if 1
			if (Source_Cloud->points.size() >= K_filter_distant_point)
			{
				int count = 0;

				//duplicate check -> just keep one source point corresponding to one target point
				std::vector<unsigned int> duplicate_check_table(Target_Cloud->points.size(), 0);

				for (auto iter = Corr->begin(); iter != Corr->end();)
				{
					int s_index, t_index;
					s_index = (*iter).index_query;
					t_index = (*iter).index_match;

					if (t_index != -1)
					{
						if (duplicate_check && duplicate_check_table[t_index] > 0)
							iter = Corr->erase(iter);
						else
						{
							duplicate_check_table[t_index]++;

							Source_Cloud_f->points.push_back(Source_Cloud->points[s_index]);
							//Target_Cloud_f->points.push_back(Target_Cloud->points[t_index]);
							(*iter).index_query = count;
							//(*iter).index_match = count;
							count++;
							iter++;
						}
					}
					else
						iter++;
				}
				Corr->resize(count);

				Source_Cloud_f->points.swap(Source_Cloud->points);
				//Target_Cloud_f->points.swap(Target_Cloud->points);
				std::vector<unsigned int>().swap(duplicate_check_table);
			}
#endif
			corr_rej_dist.setInputCorrespondences(Corr);
			corr_rej_dist.setMaximumDistance(dis_thre);
			corr_rej_dist.getCorrespondences(*Corr_f);

			if (normal_check) //only for planar points
			{
				int count = 0;
				//Normal direction consistency check
				for (auto iter = Corr_f->begin(); iter != Corr_f->end();)
				{
					int s_index, t_index;
					s_index = (*iter).index_query;
					t_index = (*iter).index_match;

					if (t_index != -1)
					{

						Eigen::Vector3d n1;
						Eigen::Vector3d n2;
						n1 << Source_Cloud->points[s_index].normal[0], Source_Cloud->points[s_index].normal[1], Source_Cloud->points[s_index].normal[2];
						n2 << Target_Cloud->points[t_index].normal[0], Target_Cloud->points[t_index].normal[1], Target_Cloud->points[t_index].normal[2];

						float cos_intersection_angle = std::abs(n1.dot(n2)); // n1.norm()=n2.norm()=1

						if (cos_intersection_angle < cos(angle_thre_degree / 180.0 * M_PI))
						{
							count++;
							iter = Corr_f->erase(iter);
						}
						else
							iter++;
					}
					else
						iter++;
				}
				//LOG(INFO) << count << " correspondences are rejected by normal check";
			}
		}
		else
			return 0;
		return 1;
	}

	bool add_corre_points(const typename pcl::PointCloud<PointT>::Ptr &sc, const typename pcl::PointCloud<PointT>::Ptr &tc, boost::shared_ptr<pcl::Correspondences> &corrs,
						  const typename pcl::PointCloud<PointT>::Ptr &pc_sc_temp, const typename pcl::PointCloud<PointT>::Ptr &pc_tc_temp)
	{
		for (int i = 0; i < (*corrs).size(); i++)
		{
			int s_index, t_index;
			s_index = (*corrs)[i].index_query;
			t_index = (*corrs)[i].index_match;

			if (t_index != -1)
			{
				pc_sc_temp->points.push_back(sc->points[s_index]);
				pc_tc_temp->points.push_back(tc->points[t_index]);
			}
		}
		return 1;
	}

	void update_corr_dist_thre(float &dis_thre_ground, float &dis_thre_pillar, float &dis_thre_beam,
							   float &dis_thre_facade, float &dis_thre_roof, float &dis_thre_vertex,
							   float dis_thre_update_rate, float dis_thre_min)

	{
		dis_thre_ground = max_(1.0 * dis_thre_ground / dis_thre_update_rate, dis_thre_min);
		dis_thre_facade = max_(1.0 * dis_thre_facade / dis_thre_update_rate, dis_thre_min);
		dis_thre_roof = max_(1.0 * dis_thre_roof / dis_thre_update_rate, dis_thre_min);
		dis_thre_pillar = max_(1.0 * dis_thre_pillar / dis_thre_update_rate, dis_thre_min);
		dis_thre_beam = max_(1.0 * dis_thre_beam / dis_thre_update_rate, dis_thre_min);
		dis_thre_vertex = max_(1.0 * dis_thre_vertex / dis_thre_update_rate, dis_thre_min);
	}

	//brief: entrance to mulls transformation estimation
	bool multi_metrics_lls_tran_estimation(const typename pcl::PointCloud<PointT>::Ptr &Source_Ground, const typename pcl::PointCloud<PointT>::Ptr &Target_Ground, boost::shared_ptr<pcl::Correspondences> &Corr_Ground,
										   const typename pcl::PointCloud<PointT>::Ptr &Source_Pillar, const typename pcl::PointCloud<PointT>::Ptr &Target_Pillar, boost::shared_ptr<pcl::Correspondences> &Corr_Pillar,
										   const typename pcl::PointCloud<PointT>::Ptr &Source_Beam, const typename pcl::PointCloud<PointT>::Ptr &Target_Beam, boost::shared_ptr<pcl::Correspondences> &Corr_Beam,
										   const typename pcl::PointCloud<PointT>::Ptr &Source_Facade, const typename pcl::PointCloud<PointT>::Ptr &Target_Facade, boost::shared_ptr<pcl::Correspondences> &Corr_Facade,
										   const typename pcl::PointCloud<PointT>::Ptr &Source_Roof, const typename pcl::PointCloud<PointT>::Ptr &Target_Roof, boost::shared_ptr<pcl::Correspondences> &Corr_Roof,
										   const typename pcl::PointCloud<PointT>::Ptr &Source_Vertex, const typename pcl::PointCloud<PointT>::Ptr &Target_Vertex, boost::shared_ptr<pcl::Correspondences> &Corr_Vertex,
										   Vector6d &unknown_x, Matrix6d &cofactor_matrix, int iter_num, std::string weight_strategy, float z_xy_balance_ratio = 1.0,
										   float pt2pt_residual_window = 0.1, float pt2pl_residual_window = 0.1, float pt2li_residual_window = 0.1)
	{
		Matrix6d ATPA;
		Vector6d ATPb;
		ATPA.setZero();
		ATPb.setZero();

		//Deal with weight (contribution of each correspondence in the transformation estimation)
		float w_ground = 1.0, w_facade = 1.0, w_roof = 1.0, w_pillar = 1.0, w_beam = 1.0, w_vertex = 1.0; //initialization

		int m1 = (*Corr_Ground).size() + (*Corr_Roof).size();
		int m2 = (*Corr_Facade).size();
		int m3 = (*Corr_Pillar).size();
		int m4 = (*Corr_Beam).size();
		int m5 = (*Corr_Vertex).size();

		if (weight_strategy[0] == '1') //x,y,z directional balanced weighting (guarantee the observability of the scene)
		{
			w_ground = max_(0.01, z_xy_balance_ratio * (m2 + 2 * m3 - m4) / (0.0001 + 2.0 * m1)); // x <-> y <-> z
			w_roof = w_ground;
			w_facade = 1.0;
			w_pillar = 1.0;
			w_beam = 1.0;
			w_vertex = 1.0;
		}

		bool dist_weight = false;
		bool residual_weight = false;
		bool intensity_weight = false;
		int iter_thre = 2; //the residual based weighting would only be applied after this number of iteration
		if (weight_strategy[1] == '1' && iter_num > iter_thre) //weight according to residual 
			residual_weight = true;
		if (weight_strategy[2] == '1') //weight according to distance
			dist_weight = true;
		if (weight_strategy[3] == '1') //weight according to intensity
			intensity_weight = true;

		//point to plane
		pt2pl_lls_summation(Source_Ground, Target_Ground, Corr_Ground, ATPA, ATPb, iter_num, w_ground, dist_weight, residual_weight, intensity_weight, pt2pl_residual_window);
		pt2pl_lls_summation(Source_Facade, Target_Facade, Corr_Facade, ATPA, ATPb, iter_num, w_facade, dist_weight, residual_weight, intensity_weight, pt2pl_residual_window);
		pt2pl_lls_summation(Source_Roof, Target_Roof, Corr_Roof, ATPA, ATPb, iter_num, w_roof, dist_weight, residual_weight, intensity_weight, pt2pl_residual_window);
		//point to line
		pt2li_lls_pri_direction_summation(Source_Pillar, Target_Pillar, Corr_Pillar, ATPA, ATPb, iter_num, w_pillar, dist_weight, residual_weight, intensity_weight, pt2li_residual_window);
		pt2li_lls_pri_direction_summation(Source_Beam, Target_Beam, Corr_Beam, ATPA, ATPb, iter_num, w_beam, dist_weight, residual_weight, intensity_weight, pt2li_residual_window);
		//point to point
		pt2pt_lls_summation(Source_Vertex, Target_Vertex, Corr_Vertex, ATPA, ATPb, iter_num, w_vertex, dist_weight, residual_weight, intensity_weight, pt2pt_residual_window);

		//ATPA is a symmetric matrix
		ATPA.coeffRef(6) = ATPA.coeffRef(1);
		ATPA.coeffRef(12) = ATPA.coeffRef(2);
		ATPA.coeffRef(13) = ATPA.coeffRef(8);
		ATPA.coeffRef(18) = ATPA.coeffRef(3);
		ATPA.coeffRef(19) = ATPA.coeffRef(9);
		ATPA.coeffRef(20) = ATPA.coeffRef(15);
		ATPA.coeffRef(24) = ATPA.coeffRef(4);
		ATPA.coeffRef(25) = ATPA.coeffRef(10);
		ATPA.coeffRef(26) = ATPA.coeffRef(16);
		ATPA.coeffRef(27) = ATPA.coeffRef(22);
		ATPA.coeffRef(30) = ATPA.coeffRef(5);
		ATPA.coeffRef(31) = ATPA.coeffRef(11);
		ATPA.coeffRef(32) = ATPA.coeffRef(17);
		ATPA.coeffRef(33) = ATPA.coeffRef(23);
		ATPA.coeffRef(34) = ATPA.coeffRef(29);

		//LOG(INFO) << "ATPA=" << std::endl << ATPA;
		//LOG(INFO) << "ATPb=" << std::endl << ATPb;
	
		// Solve A*x = b  x= (ATPA)^(-1)ATPb
		// x: tx ty tz alpha beta gamma (alpha beta gamma corresponding to roll, pitch and yaw)
		// the approximated rotation matrix is
		// |   1    -gamma   beta  |
		// | gamma     1    -alpha |
		// | -beta   alpha     1   |
		//reference: A Review of Point Cloud Registration Algorithms for Mobile Robotics, Appendix

		unknown_x = ATPA.inverse() * ATPb;

		Eigen::Vector3d euler_angle(unknown_x(3), unknown_x(4), unknown_x(5));
		Eigen::Matrix3d Jacobi;
		get_quat_euler_jacobi(euler_angle, Jacobi);

		//Qxx=(ATPA)^-1
		//information matrix = Dxx^(-1)=Qxx^(-1)/(sigma_post)^2=ATPA/(sigma_post)^2
		cofactor_matrix = ATPA.inverse();

		//convert to the cofactor matrix with regard to quaternion from euler angle
		cofactor_matrix.block<3, 3>(3, 3) = Jacobi * cofactor_matrix.block<3, 3>(3, 3) * Jacobi.transpose();
		cofactor_matrix.block<3, 3>(0, 3) = cofactor_matrix.block<3, 3>(0, 3) * Jacobi.transpose();
		cofactor_matrix.block<3, 3>(3, 0) = Jacobi * cofactor_matrix.block<3, 3>(3, 0);

		return 1;
	}

	//Linearization of Rotation Matrix
	//R = I + (alpha, beta, gamma) ^
	//  = | 1      -gamma    beta |
	//    | gamma   1       -alpha|
	//    |-beta    alpha     1   |

	//point-to-point LLS
	bool pt2pt_lls_summation(const typename pcl::PointCloud<PointT>::Ptr &Source_Cloud, const typename pcl::PointCloud<PointT>::Ptr &Target_Cloud,
							 boost::shared_ptr<pcl::Correspondences> &Corr, Matrix6d &ATPA, Vector6d &ATPb, int iter_num,
							 float weight, bool dist_weight_or_not = false, bool residual_weight_or_not = false,
							 bool intensity_weight_or_not = false,
							 float residual_window_size = 0.1)
	{
		for (int i = 0; i < (*Corr).size(); i++)
		{
			int s_index, t_index;
			s_index = (*Corr)[i].index_query;
			t_index = (*Corr)[i].index_match;

			if (t_index != -1)
			{

				float px = Source_Cloud->points[s_index].x;
				float py = Source_Cloud->points[s_index].y;
				float pz = Source_Cloud->points[s_index].z;
				float qx = Target_Cloud->points[t_index].x;
				float qy = Target_Cloud->points[t_index].y;
				float qz = Target_Cloud->points[t_index].z;

				float pi = Source_Cloud->points[s_index].intensity;
				float qi = Target_Cloud->points[t_index].intensity;

				float dx = px - qx;
				float dy = py - qy;
				float dz = pz - qz;

				float wx, wy, wz;
				wx = weight;

				float dist = std::sqrt(qx * qx + qy * qy + qz * qz);

				if (dist_weight_or_not)
					wx = wx * get_weight_by_dist_adaptive(dist, iter_num);
				//wx = wx * get_weight_by_dist(dist);

				if (residual_weight_or_not)
					wx = wx * get_weight_by_residual(std::sqrt(dx * dx + dy * dy + dz * dz), residual_window_size);
				if (intensity_weight_or_not)
					wx = wx * get_weight_by_intensity(pi + 0.0001, qi + 0.0001);

				wy = wx;
				wz = wx;
                
				// unknown x: [tx ty tz alpha beta gama] 

				//    0  1  2  3  4  5
				//    6  7  8  9 10 11
				//   12 13 14 15 16 17
				//   18 19 20 21 22 23
				//   24 25 26 27 28 29
				//   30 31 32 33 34 35

				ATPA.coeffRef(0) += wx;
				ATPA.coeffRef(1) += 0;
				ATPA.coeffRef(2) += 0;
				ATPA.coeffRef(3) += 0;
				ATPA.coeffRef(4) += wx * pz;
				ATPA.coeffRef(5) += (-wx * py);
				ATPA.coeffRef(7) += wy;
				ATPA.coeffRef(8) += 0;
				ATPA.coeffRef(9) += (-wy * pz);
				ATPA.coeffRef(10) += 0;
				ATPA.coeffRef(11) += wy * px;
				ATPA.coeffRef(14) += wz;
				ATPA.coeffRef(15) += wz * py;
				ATPA.coeffRef(16) += (-wz * px);
				ATPA.coeffRef(17) += 0;
				ATPA.coeffRef(21) += wy * pz * pz + wz * py * py;
				ATPA.coeffRef(22) += (-wz * px * py);
				ATPA.coeffRef(23) += (-wy * px * pz);
				ATPA.coeffRef(28) += wx * pz * pz + wz * px * px;
				ATPA.coeffRef(29) += (-wx * py * pz);
				ATPA.coeffRef(35) += wx * py * py + wy * px * px;

				ATPb.coeffRef(0) += (-wx * dx);
				ATPb.coeffRef(1) += (-wy * dy);
				ATPb.coeffRef(2) += (-wz * dz);
				ATPb.coeffRef(3) += wy * pz * dy - wz * py * dz;
				ATPb.coeffRef(4) += wz * px * dz - wx * pz * dx;
				ATPb.coeffRef(5) += wx * py * dx - wy * px * dy;
			}
		}

		return 1;
	}

	//point-to-plane LLS
	bool pt2pl_lls_summation(const typename pcl::PointCloud<PointT>::Ptr &Source_Cloud, const typename pcl::PointCloud<PointT>::Ptr &Target_Cloud,
							 boost::shared_ptr<pcl::Correspondences> &Corr, Matrix6d &ATPA, Vector6d &ATPb, int iter_num,
							 float weight, bool dist_weight_or_not = false, bool residual_weight_or_not = false,
							 bool intensity_weight_or_not = false,
							 float residual_window_size = 0.1)
	{
		for (int i = 0; i < (*Corr).size(); i++)
		{
			int s_index, t_index;
			s_index = (*Corr)[i].index_query;
			t_index = (*Corr)[i].index_match;

			if (t_index != -1)
			{
				float px = Source_Cloud->points[s_index].x;
				float py = Source_Cloud->points[s_index].y;
				float pz = Source_Cloud->points[s_index].z;
				float qx = Target_Cloud->points[t_index].x;
				float qy = Target_Cloud->points[t_index].y;
				float qz = Target_Cloud->points[t_index].z;
				float ntx = Target_Cloud->points[t_index].normal_x;
				float nty = Target_Cloud->points[t_index].normal_y;
				float ntz = Target_Cloud->points[t_index].normal_z;

				float pi = Source_Cloud->points[s_index].intensity;
				float qi = Target_Cloud->points[t_index].intensity;

				float w = weight;

				float a = ntz * py - nty * pz;
				float b = ntx * pz - ntz * px;
				float c = nty * px - ntx * py;

				float d = ntx * qx + nty * qy + ntz * qz - ntx * px - nty * py - ntz * pz;

				float dist = std::sqrt(qx * qx + qy * qy + qz * qz);

				if (dist_weight_or_not)
					w = w * get_weight_by_dist_adaptive(dist, iter_num);
				//w = w * get_weight_by_dist(dist);

				if (residual_weight_or_not)
					w = w * get_weight_by_residual(std::abs(d), residual_window_size);
				//w = w * get_weight_by_residual_general(std::abs(d), residual_window_size, 1.0);

				if (intensity_weight_or_not)
					w = w * get_weight_by_intensity(pi + 0.0001, qi + 0.0001);

				(*Corr)[i].weight = w;


				//    0  1  2  3  4  5
				//    6  7  8  9 10 11
				//   12 13 14 15 16 17
				//   18 19 20 21 22 23
				//   24 25 26 27 28 29
				//   30 31 32 33 34 35

				ATPA.coeffRef(0) += w * ntx * ntx;
				ATPA.coeffRef(1) += w * ntx * nty;
				ATPA.coeffRef(2) += w * ntx * ntz;
				ATPA.coeffRef(3) += w * a * ntx;
				ATPA.coeffRef(4) += w * b * ntx;
				ATPA.coeffRef(5) += w * c * ntx;
				ATPA.coeffRef(7) += w * nty * nty;
				ATPA.coeffRef(8) += w * nty * ntz;
				ATPA.coeffRef(9) += w * a * nty;
				ATPA.coeffRef(10) += w * b * nty;
				ATPA.coeffRef(11) += w * c * nty;
				ATPA.coeffRef(14) += w * ntz * ntz;
				ATPA.coeffRef(15) += w * a * ntz;
				ATPA.coeffRef(16) += w * b * ntz;
				ATPA.coeffRef(17) += w * c * ntz;
				ATPA.coeffRef(21) += w * a * a;
				ATPA.coeffRef(22) += w * a * b;
				ATPA.coeffRef(23) += w * a * c;
				ATPA.coeffRef(28) += w * b * b;
				ATPA.coeffRef(29) += w * b * c;
				ATPA.coeffRef(35) += w * c * c;

				ATPb.coeffRef(0) += w * d * ntx;
				ATPb.coeffRef(1) += w * d * nty;
				ATPb.coeffRef(2) += w * d * ntz;
				ATPb.coeffRef(3) += w * d * a;
				ATPb.coeffRef(4) += w * d * b;
				ATPb.coeffRef(5) += w * d * c;
			}
		}

		return 1;
	}

	//point-to-line LLS (calculated using primary direction vector), used now
	//the normal vector here actually stores the primary direcyion vector (for easier calculation of the residual)
	bool pt2li_lls_pri_direction_summation(const typename pcl::PointCloud<PointT>::Ptr &Source_Cloud, const typename pcl::PointCloud<PointT>::Ptr &Target_Cloud,
										   boost::shared_ptr<pcl::Correspondences> &Corr, Matrix6d &ATPA, Vector6d &ATPb, int iter_num,
										   float weight, bool dist_weight_or_not = false, bool residual_weight_or_not = false,
										   bool intensity_weight_or_not = false,
										   float residual_window_size = 0.1)
	{
		for (int i = 0; i < (*Corr).size(); i++)
		{
			int s_index, t_index;
			s_index = (*Corr)[i].index_query;
			t_index = (*Corr)[i].index_match;

			if (t_index != -1)
			{
				float px = Source_Cloud->points[s_index].x;
				float py = Source_Cloud->points[s_index].y;
				float pz = Source_Cloud->points[s_index].z;
				float qx = Target_Cloud->points[t_index].x;
				float qy = Target_Cloud->points[t_index].y;
				float qz = Target_Cloud->points[t_index].z;

				//primary direction (in this case, we save the primary direction of linear feature points in the normal vector)
				float vx = Target_Cloud->points[t_index].normal_x;
				float vy = Target_Cloud->points[t_index].normal_y;
				float vz = Target_Cloud->points[t_index].normal_z;

				//LOG(INFO) << nx << "," << ny<< "," <<nz;

				float pi = Source_Cloud->points[s_index].intensity;
				float qi = Target_Cloud->points[t_index].intensity;

				float dx = px - qx;
				float dy = py - qy;
				float dz = pz - qz;

				Eigen::Matrix<double, 3, 6> Amat;
				Eigen::Matrix<double, 3, 1> bvec;
				Eigen::Matrix<double, 3, 1> evec;
				Eigen::Matrix<double, 3, 3> Imat;
				Eigen::Matrix<double, 3, 3> Wmat;
				Imat.setIdentity();
				Wmat.setIdentity();

				Amat(0, 0) = 0;
				Amat(0, 1) = -vz;
				Amat(0, 2) = vy;
				Amat(0, 3) = vy * py + vz * pz;
				Amat(0, 4) = -vy * px;
				Amat(0, 5) = -vz * px;
				Amat(1, 0) = vz;
				Amat(1, 1) = 0;
				Amat(1, 2) = -vx;
				Amat(1, 3) = -vx * py;
				Amat(1, 4) = vz * pz + vx * px;
				Amat(1, 5) = -vz * py;
				Amat(2, 0) = -vy;
				Amat(2, 1) = vx;
				Amat(2, 2) = 0;
				Amat(2, 3) = -vx * pz;
				Amat(2, 4) = -vy * pz;
				Amat(2, 5) = vx * px + vy * py;

				bvec(0, 0) = -vy * dz + vz * dy;
				bvec(1, 0) = -vz * dx + vx * dz;
				bvec(2, 0) = -vx * dy + vy * dx;

				//evec = (Amat * (Amat.transpose() * Amat).inverse() * Amat.transpose() - Imat) * bvec; //posterior residual
				//we'd like to directly use the prior residual
				float ex = std::abs(bvec(0, 0));
				float ey = std::abs(bvec(1, 0));
				float ez = std::abs(bvec(2, 0));
				float ed = std::sqrt(ex * ex + ey * ey + ez * ez);

				float wx, wy, wz, w;
				wx = weight;

				float dist = std::sqrt(qx * qx + qy * qy + qz * qz);

				if (dist_weight_or_not)
					//wx *= get_weight_by_dist(dist);
					wx *= get_weight_by_dist_adaptive(dist, iter_num);

				if (intensity_weight_or_not)
					wx *= get_weight_by_intensity(pi + 0.0001, qi + 0.0001);

				if (residual_weight_or_not)
				{
					// wx *= get_weight_by_residual(ex, residual_window_size);
					// wy *= get_weight_by_residual(ey, residual_window_size);
					// wz *= get_weight_by_residual(ez, residual_window_size);

					wx = wx * get_weight_by_residual(ed, residual_window_size); //original huber
																				// wx = wx * get_weight_by_residual_general(ed, residual_window_size, 1.0);
				}
				wy = wx;
				wz = wx;
				(*Corr)[i].weight = wx;

				Wmat(0, 0) = std::sqrt(wx);
				Wmat(1, 1) = std::sqrt(wy);
				Wmat(2, 2) = std::sqrt(wz);

				Amat = Wmat * Amat;
				bvec = Wmat * bvec;

				for (int j = 0; j < 6; j++)
				{
					for (int k = j; k < 6; k++)
						ATPA(j, k) += ((Amat.block<3, 1>(0, j)).transpose() * (Amat.block<3, 1>(0, k)));
				}
				for (int j = 0; j < 6; j++)
					ATPb.coeffRef(j) += ((Amat.block<3, 1>(0, j)).transpose() * (bvec));
			}
		}
		return 1;
	}


	bool ground_3dof_lls_tran_estimation(const typename pcl::PointCloud<PointT>::Ptr &Source_Ground,
										 const typename pcl::PointCloud<PointT>::Ptr &Target_Ground,
										 boost::shared_ptr<pcl::Correspondences> &Corr_Ground,
										 Eigen::Vector3d &unknown_x, Eigen::Matrix3d &cofactor_matrix,
										 int iter_num, std::string weight_strategy)
	{
		Eigen::Matrix3d ATPA;
		Eigen::Vector3d ATPb;
		ATPA.setZero();
		ATPb.setZero();

		bool dist_weight = false;
		bool residual_weight = false;
		bool intensity_weight = false;

		if (weight_strategy[1] == '1') //weight according to residual
			residual_weight = true;

		if (weight_strategy[2] == '1') //weight according to distance
			dist_weight = true;

		if (weight_strategy[3] == '1') //weight according to intensity
			intensity_weight = true;

		pt2pl_ground_3dof_lls_summation(Source_Ground, Target_Ground, Corr_Ground, ATPA, ATPb, iter_num, 1.0, dist_weight, residual_weight, intensity_weight);

		//ATPA is a symmetric matrix
		//    0  1  2
		//   [3] 4  5
		//   [6][7] 8
		ATPA.coeffRef(3) = ATPA.coeffRef(1);
		ATPA.coeffRef(6) = ATPA.coeffRef(2);
		ATPA.coeffRef(7) = ATPA.coeffRef(5);

		// Solve A*x = b  x= (ATPA)^(-1)ATPb
		// x: tx ty tz alpha beta gamma
		unknown_x = ATPA.inverse() * ATPb;

		return 1;
	}

	//ground 3dof : roll, pitch, z
	bool pt2pl_ground_3dof_lls_summation(const typename pcl::PointCloud<PointT>::Ptr &Source_Cloud, const typename pcl::PointCloud<PointT>::Ptr &Target_Cloud,
										 boost::shared_ptr<pcl::Correspondences> &Corr, Eigen::Matrix3d &ATPA, Eigen::Vector3d &ATPb, int iter_num,
										 float weight, bool dist_weight_or_not = false, bool residual_weight_or_not = false, bool intensity_weight_or_not = false,
										 float residual_window_size = 0.1)
	{
		//unknown : roll (alpha), picth (beta) and tz
		for (int i = 0; i < (*Corr).size(); i++)
		{
			int s_index, t_index;
			s_index = (*Corr)[i].index_query;
			t_index = (*Corr)[i].index_match;

			if (t_index != -1)
			{

				float px = Source_Cloud->points[s_index].x;
				float py = Source_Cloud->points[s_index].y;
				float pz = Source_Cloud->points[s_index].z;
				float qx = Target_Cloud->points[t_index].x;
				float qy = Target_Cloud->points[t_index].y;
				float qz = Target_Cloud->points[t_index].z;
				float ntx = Target_Cloud->points[t_index].normal_x;
				float nty = Target_Cloud->points[t_index].normal_y;
				float ntz = Target_Cloud->points[t_index].normal_z;

				float pi = Source_Cloud->points[s_index].intensity;
				float qi = Target_Cloud->points[t_index].intensity;

				float w = weight;

				float a = ntz * py - nty * pz;
				float b = ntx * pz - ntz * px;

				float d = ntx * qx + nty * qy + ntz * qz - ntx * px - nty * py - ntz * pz;

				float dist = std::sqrt(qx * qx + qy * qy + qz * qz);
				if (dist_weight_or_not)
					w = w * get_weight_by_dist_adaptive(dist, iter_num);
				//w = w * get_weight_by_dist(dist);

				if (residual_weight_or_not)
					w = w * get_weight_by_residual(std::abs(d), residual_window_size);

				if (intensity_weight_or_not)
					w = w * get_weight_by_intensity(pi + 0.0001, qi + 0.0001);

				//    0  1  2
				//    3  4  5
				//    6  7  8

				(*Corr)[i].weight = w;

				ATPA.coeffRef(0) += w * a * a;
				ATPA.coeffRef(1) += w * a * b;
				ATPA.coeffRef(2) += w * a * ntz;
				ATPA.coeffRef(4) += w * b * b;
				ATPA.coeffRef(5) += w * b * ntz;
				ATPA.coeffRef(8) += w * ntz * ntz;

				ATPb.coeffRef(0) += w * d * a;
				ATPb.coeffRef(1) += w * d * b;
				ATPb.coeffRef(2) += w * d * ntz;
			}
		}

		return 1;
	}

	//point-to-line LLS (calculated using normal vector) , Deprecated
	bool pt2li_lls_summation(const typename pcl::PointCloud<PointT>::Ptr &Source_Cloud, const typename pcl::PointCloud<PointT>::Ptr &Target_Cloud,
							 boost::shared_ptr<pcl::Correspondences> &Corr, Matrix6d &ATPA, Vector6d &ATPb, int iter_num,
							 float weight, bool dist_weight_or_not = false, bool residual_weight_or_not = false,
							 bool intensity_weight_or_not = false,
							 float residual_window_size = 0.1)
	{
		for (int i = 0; i < (*Corr).size(); i++)
		{
			int s_index, t_index;
			s_index = (*Corr)[i].index_query;
			t_index = (*Corr)[i].index_match;

			if (t_index != -1)
			{
				float px = Source_Cloud->points[s_index].x;
				float py = Source_Cloud->points[s_index].y;
				float pz = Source_Cloud->points[s_index].z;
				float qx = Target_Cloud->points[t_index].x;
				float qy = Target_Cloud->points[t_index].y;
				float qz = Target_Cloud->points[t_index].z;
				float ntx = Target_Cloud->points[t_index].normal[0];
				float nty = Target_Cloud->points[t_index].normal[1];
				float ntz = Target_Cloud->points[t_index].normal[2];
				float nsx = Source_Cloud->points[s_index].normal[0];
				float nsy = Source_Cloud->points[s_index].normal[1];
				float nsz = Source_Cloud->points[s_index].normal[2];

				float pi = Source_Cloud->points[s_index].intensity;
				float qi = Target_Cloud->points[t_index].intensity;

				float dx = px - qx;
				float dy = py - qy;
				float dz = pz - qz;

				// norm (nt * ns)
				float nx = nty * nsz - ntz * nsy;
				float ny = ntz * nsx - ntx * nsz;
				float nz = ntx * nsy - nty * nsx;
				float nd = sqrt(nx * nx + ny * ny + nz * nz);
				nx /= nd;
				ny /= nd;
				nz /= nd; //normalize

				double nxy = nx * ny;
				double nxz = nx * nz;
				double nyz = ny * nz;
				double nx2 = nx * nx;
				double ny2 = ny * ny;
				double nz2 = nz * nz;
				double px2 = px * px;
				double py2 = py * py;
				double pz2 = pz * pz;
				double pxy = px * py;
				double pxz = px * pz;
				double pyz = py * pz;

				float d = std::sqrt((nxz * dz - nz2 * dx - ny2 * dx + nxy * dy) * (nxz * dz - nz2 * dx - ny2 * dx + nxy * dy) +
									(-nz2 * dy + nyz * dz + nxy * dx - nx2 * dy) * (-nz2 * dy + nyz * dz + nxy * dx - nx2 * dy) +
									(nyz * dy - ny2 * dz - nx2 * dz + nxz * dx) * (nyz * dy - ny2 * dz - nx2 * dz + nxz * dx));

				// float ex = std::abs(nxz * dz - nz2 * dx - ny2 * dx + nxy * dy);
				// float ey = std::abs(-nz2 * dy + nyz * dz + nxy * dx - nx2 * dy);
				// float ez = std::abs(nyz * dy - ny2 * dz - nx2 * dz + nxz * dx);

				float wx, wy, wz;
				wx = weight;

				float dist = std::sqrt(qx * qx + qy * qy + qz * qz);
				if (dist_weight_or_not)
					wx = wx * get_weight_by_dist_adaptive(dist, iter_num);
				//wx = wx * get_weight_by_dist(dist);

				if (intensity_weight_or_not)
					wx = wx * get_weight_by_intensity(pi + 0.0001, qi + 0.0001);

				if (residual_weight_or_not)
				{
					wx = wx * get_weight_by_residual(d, residual_window_size);
					// wy = wy * get_weight_by_residual(std::abs(evec(1, 0)), residual_window_size);
					// wz = wz * get_weight_by_residual(std::abs(evec(2, 0)), residual_window_size);
				}

				wy = wx;
				wz = wx;

				(*Corr)[i].weight = wx;

				//    0  1  2  3  4  5
				//    6  7  8  9 10 11
				//   12 13 14 15 16 17
				//   18 19 20 21 22 23
				//   24 25 26 27 28 29
				//   30 31 32 33 34 35

				ATPA.coeffRef(0) += (wy * nz2 + wz * ny2);
				ATPA.coeffRef(1) += (-wz * nxy);
				ATPA.coeffRef(2) += (-wy * nxz);
				ATPA.coeffRef(3) += (-wy * nxz * py + wz * nxy * pz);
				ATPA.coeffRef(4) += (wy * nxz * px + wy * nz2 * pz + wz * ny2 * pz);
				ATPA.coeffRef(5) += (-wy * nz2 * py - wz * ny2 * py + wz * nxy * px);
				ATPA.coeffRef(7) += (wx * nz2 + wz * nx2);
				ATPA.coeffRef(8) += (-wx * nyz);
				ATPA.coeffRef(9) += (-wx * nz2 * pz - wx * nyz * py - wz * nz2 * pz);
				ATPA.coeffRef(10) += (wx * nyz * px - wz * nxy * pz);
				ATPA.coeffRef(11) += (wx * nz2 * px + wz * nxy * py + wz * nx2 * px);
				ATPA.coeffRef(14) += (wx * ny2 + wy * nx2);
				ATPA.coeffRef(15) += (wx * nyz * pz + wx * ny2 * py + wy * nx2 * py);
				ATPA.coeffRef(16) += (-wx * ny2 * px - wy * nx2 * px - wy * nxz * pz);
				ATPA.coeffRef(17) += (-wx * nyz * px + wy * nxz * py);
				ATPA.coeffRef(21) += (wx * (nz2 * pz2 + ny2 * py2 + 2 * nyz * pyz) + wy * nx2 * py2 + wz * nx2 * pz2);
				ATPA.coeffRef(22) += (-wx * (nyz * pxz + ny2 * pxy) - wy * (nx2 * pxy + nxz * pyz) + wz * nxy * pz2);
				ATPA.coeffRef(23) += (-wx * (nz2 * pxz + nyz * pxy) + wy * nxz * py2 - wz * (nxy * pyz + nx2 * pxz));
				ATPA.coeffRef(28) += (wx * ny2 * px2 + wy * (nx2 * px2 + nz2 * pz2 + 2 * nxz * pxz) + wz * ny2 * pz2);
				ATPA.coeffRef(29) += (wx * nyz * px2 - wy * (nxz * pxy + nz2 * pyz) - wz * (ny2 * pyz + nxy * pxz));
				ATPA.coeffRef(35) += (wx * nz2 * px2 + wy * nz2 * py2 + wz * (ny2 * py2 + nx2 * px2 + 2 * nxy * pxy));

				ATPb.coeffRef(0) += (wy * (nxz * dz - nz2 * dx) + wz * (-ny2 * dx + nxy * dy));
				ATPb.coeffRef(1) += (wx * (-nz2 * dy + nyz * dz) + wz * (nxy * dx - nx2 * dy));
				ATPb.coeffRef(2) += (wx * (nyz * dy - ny2 * dz) + wy * (-nx2 * dz + nxz * dx));
				ATPb.coeffRef(3) += (wx * (nz * pz * ny * py) * (nz * dy - ny * dz) + wy * nx * py * (-nx * dz + nz * dx) + wz * nx * pz * (-ny * dx + nx * dy));
				ATPb.coeffRef(4) += (wx * ny * px * (-nz * dy + ny * dz) + wy * (nx * px + nz * pz) * (nx * dz - nz * dx) + wz * ny * pz * (-ny * dx + nx * dy));
				ATPb.coeffRef(5) += (wx * nz * px * (-nz * dy + ny * dz) + wy * nz * py * (-nx * dz + nz * dx) + wz * (ny * py + nx * px) * (ny * dx - nx * dy));
			}
		}

		return 1;
	}

	//calculate residual v
	bool get_multi_metrics_lls_residual(const typename pcl::PointCloud<PointT>::Ptr &Source_Ground, const typename pcl::PointCloud<PointT>::Ptr &Target_Ground, boost::shared_ptr<pcl::Correspondences> &Corr_Ground,
										const typename pcl::PointCloud<PointT>::Ptr &Source_Pillar, const typename pcl::PointCloud<PointT>::Ptr &Target_Pillar, boost::shared_ptr<pcl::Correspondences> &Corr_Pillar,
										const typename pcl::PointCloud<PointT>::Ptr &Source_Beam, const typename pcl::PointCloud<PointT>::Ptr &Target_Beam, boost::shared_ptr<pcl::Correspondences> &Corr_Beam,
										const typename pcl::PointCloud<PointT>::Ptr &Source_Facade, const typename pcl::PointCloud<PointT>::Ptr &Target_Facade, boost::shared_ptr<pcl::Correspondences> &Corr_Facade,
										const typename pcl::PointCloud<PointT>::Ptr &Source_Roof, const typename pcl::PointCloud<PointT>::Ptr &Target_Roof, boost::shared_ptr<pcl::Correspondences> &Corr_Roof,
										const typename pcl::PointCloud<PointT>::Ptr &Source_Vertex, const typename pcl::PointCloud<PointT>::Ptr &Target_Vertex, boost::shared_ptr<pcl::Correspondences> &Corr_Vertex,
										const Vector6d &transform_x, double &sigma_square_post, double sigma_thre = 0.2)
	{
		double VTPV = 0;
		int obeservation_count = 0;

		pt2pl_lls_residual(Source_Ground, Target_Ground, Corr_Ground, transform_x, VTPV, obeservation_count);
		pt2pl_lls_residual(Source_Facade, Target_Facade, Corr_Facade, transform_x, VTPV, obeservation_count);
		pt2pl_lls_residual(Source_Roof, Target_Roof, Corr_Roof, transform_x, VTPV, obeservation_count);
		pt2li_lls_residual(Source_Pillar, Target_Pillar, Corr_Pillar, transform_x, VTPV, obeservation_count);
		pt2li_lls_residual(Source_Beam, Target_Beam, Corr_Beam, transform_x, VTPV, obeservation_count);
		pt2pt_lls_residual(Source_Vertex, Target_Vertex, Corr_Vertex, transform_x, VTPV, obeservation_count);

		sigma_square_post = VTPV / (obeservation_count - 6); //   VTPV/(n-t) , t is the neccessary observation number (dof), here, t=6

		LOG(INFO) << "The posterior unit weight standard deviation (m) is " << sqrt(sigma_square_post);

		if (sqrt(sigma_square_post) < sigma_thre)
			return 1;
		else
			return 0;
	}

	bool pt2pt_lls_residual(const typename pcl::PointCloud<PointT>::Ptr &Source_Cloud, const typename pcl::PointCloud<PointT>::Ptr &Target_Cloud,
							boost::shared_ptr<pcl::Correspondences> &Corr, const Vector6d &transform_x, double &VTPV, int &observation_count)
	{
		//point-to-plane distance metrics
		//3 observation equation for 1 pair of correspondence
		for (int i = 0; i < (*Corr).size(); i++)
		{
			int s_index, t_index;
			s_index = (*Corr)[i].index_query;
			t_index = (*Corr)[i].index_match;

			if (t_index != -1)
			{
				float px = Source_Cloud->points[s_index].x;
				float py = Source_Cloud->points[s_index].y;
				float pz = Source_Cloud->points[s_index].z;
				float qx = Target_Cloud->points[t_index].x;
				float qy = Target_Cloud->points[t_index].y;
				float qz = Target_Cloud->points[t_index].z;

				float dx = px - qx;
				float dy = py - qy;
				float dz = pz - qz;

				Eigen::Matrix<double, 3, 6> A_Matrix;
				Eigen::Matrix<double, 3, 1> b_vector;
				Eigen::Matrix<double, 3, 1> residual_vector;

				A_Matrix << 1, 0, 0, 0, pz, -py,
					0, 1, 0, -pz, 0, px,
					0, 0, 1, py, -px, 0;
				b_vector << -dx, -dy, -dz;

				residual_vector = A_Matrix * transform_x - b_vector;

				VTPV += (*Corr)[i].weight * (residual_vector(0) * residual_vector(0) + residual_vector(1) * residual_vector(1) + residual_vector(2) * residual_vector(2));

				observation_count += 3;
			}
		}

		return 1;
	}

	bool pt2pl_lls_residual(const typename pcl::PointCloud<PointT>::Ptr &Source_Cloud, const typename pcl::PointCloud<PointT>::Ptr &Target_Cloud,
							boost::shared_ptr<pcl::Correspondences> &Corr, const Vector6d &transform_x, double &VTPV, int &observation_count)
	{
		//point-to-plane distance metrics
		//1 observation equation for 1 pair of correspondence
		for (int i = 0; i < (*Corr).size(); i++)
		{
			int s_index, t_index;
			s_index = (*Corr)[i].index_query;
			t_index = (*Corr)[i].index_match;
			if (t_index != -1)
			{
				float px = Source_Cloud->points[s_index].x;
				float py = Source_Cloud->points[s_index].y;
				float pz = Source_Cloud->points[s_index].z;
				float qx = Target_Cloud->points[t_index].x;
				float qy = Target_Cloud->points[t_index].y;
				float qz = Target_Cloud->points[t_index].z;
				float ntx = Target_Cloud->points[t_index].normal_x;
				float nty = Target_Cloud->points[t_index].normal_y;
				float ntz = Target_Cloud->points[t_index].normal_z;

				float a = ntz * py - nty * pz;
				float b = ntx * pz - ntz * px;
				float c = nty * px - ntx * py;
				float d = ntx * qx + nty * qy + ntz * qz - ntx * px - nty * py - ntz * pz;

				float residual = ntx * transform_x(0) + nty * transform_x(1) + ntz * transform_x(2) + a * transform_x(3) + b * transform_x(4) + c * transform_x(5) - d;

				//LOG(INFO) << "final weight (pt-pl): " << (*Corr)[i].weight;

				VTPV += (*Corr)[i].weight * residual * residual;

				observation_count++;
			}
		}

		return 1;
	}

	//primary vector stored as point normal
	bool pt2li_lls_residual(const typename pcl::PointCloud<PointT>::Ptr &Source_Cloud, const typename pcl::PointCloud<PointT>::Ptr &Target_Cloud,
							boost::shared_ptr<pcl::Correspondences> &Corr, const Vector6d &transform_x, double &VTPV, int &observation_count)
	{
		//point-to-line distance metrics
		//3 observation equation for 1 pair of correspondence
		for (int i = 0; i < (*Corr).size(); i++)
		{
			int s_index, t_index;
			s_index = (*Corr)[i].index_query;
			t_index = (*Corr)[i].index_match;
			if (t_index != -1)
			{
				float px = Source_Cloud->points[s_index].x;
				float py = Source_Cloud->points[s_index].y;
				float pz = Source_Cloud->points[s_index].z;
				float qx = Target_Cloud->points[t_index].x;
				float qy = Target_Cloud->points[t_index].y;
				float qz = Target_Cloud->points[t_index].z;
				float vx = Target_Cloud->points[t_index].normal_x; //actually primary directional vector 
				float vy = Target_Cloud->points[t_index].normal_y;
				float vz = Target_Cloud->points[t_index].normal_z;

				float dx = px - qx;
				float dy = py - qy;
				float dz = pz - qz;

				Eigen::Matrix<double, 3, 6> A_Matrix;
				Eigen::Matrix<double, 3, 1> b_vector;
				Eigen::Matrix<double, 3, 1> residual_vector;

				A_Matrix << 0, vz, -vy, -vz * pz - vy * py, vy * px, vz * px,
					-vz, 0, vx, vx * py, -vx * px - vz * pz, vz * py,
					vy, -vx, 0, vx * pz, vy * pz, -vy * py - vx * px;
				b_vector << -vz * dy + vy * dz, -vx * dz + vz * dx, -vy * dx + vx * dy;

				residual_vector = A_Matrix * transform_x - b_vector;

				//LOG(INFO) << "final weight (pt-li): " << (*Corr)[i].weight;

				//VTPV is the sum of square of the residuals
				VTPV += (*Corr)[i].weight * (residual_vector(0) * residual_vector(0) + residual_vector(1) * residual_vector(1) + residual_vector(2) * residual_vector(2));

				observation_count += 3;
			}
		}
		return 1;
	}

	// the following lines contain the various weighting functions: distance weight, intensity-compatible weight and residual weight

	// Intuition: three part, near , medium , far
	// near used to control translation
	// far used to control rotation
	// medium used to control both
	//TODO: change the distance weight according to the iteration number (mathematic deducing)
	float get_weight_by_dist_adaptive(float dist, int iter_num, float unit_dist = 30.0, float b_min = 0.7, float b_max = 1.3, float b_step = 0.05)
	{
		float b_current = min_(b_min + b_step * iter_num, b_max);
		float temp_weight = b_current + (1.0 - b_current) * dist / unit_dist;
		temp_weight = max_(temp_weight, 0.01);
		return temp_weight;
	}

	//standard
	inline float get_weight_by_dist(float dist, float unit_dist = 60.0, float base_value = 0.7) //unit_dist = 60.0 (is just a multiplier constant)
	{
		return (base_value + (1 - base_value) * dist / unit_dist);
		//return (base_value + (1 - base_value) * unit_dist / dist);
	}

	inline float get_weight_by_intensity(float intensity_1, float intensity_2, float base_value = 0.6, float intensity_scale = 255.0)
	{
		float intensity_diff_ratio = std::fabs(intensity_1 - intensity_2) / intensity_scale;
		float intensity_weight = std::exp(-1.0 * intensity_diff_ratio);
		return intensity_weight;
		//return (base_value + (1 - base_value) * min_(intensity_1 / intensity_2, intensity_2 / intensity_1));
	}

	//By huber loss function
	inline float get_weight_by_residual(float res, float huber_thre = 0.05, int delta = 1) //test different kind of robust kernel function here
	{
		//return ((res > huber_thre) ? (std::sqrt(1.0 + (res / huber_thre - 1.0)) / (res / huber_thre)) : (1.0));
		//return ((res > huber_thre) ? (1.0 + (res / huber_thre - 1.0) / (res / huber_thre) / (res / huber_thre)) : (1.0)); //this function (naive huber) is better
		//return ((res > huber_thre) ? (huber_thre / res) : (1.0));

		//Huber Loss
		//y=0.5*x^2        , x<d
		//y=0.5*d^2+|x|-d  , x>=d
		//d= 1, |x|= res/huber_thre
		//weight=(0.5*d^2+|x|-d)/(0.5*x^2) = (2*res*huber_thre-huber_thre*huber_thre)/res/res)
		return ((res > huber_thre) ? ((2 * res * huber_thre + (delta * delta - 2 * delta) * (huber_thre * huber_thre)) / res / res) : (1.0));
	}

	//general function for m-estimation
	float get_weight_by_residual_general(float res, float thre = 0.05, float alpha = 2.0) //test different kind of robust kernel function here
	{
		float weight;
		res = res / thre;
		if (alpha == 2)
			weight = 1.0;
		else if (alpha == 0)
			weight = 2.0 / (res * res + 2.0);
		else
			weight = 1.0 * std::pow((res * res / std::abs(alpha - 2.0) + 1.0), (alpha * 0.5 - 1.0));

		return weight;
	}

	//roll - pitch - yaw rotation (x - y' - z'') ----> this is for our tiny angle approximation
	bool construct_trans_a(const double &tx, const double &ty, const double &tz,
						   const double &alpha, const double &beta, const double &gamma,
						   Eigen::Matrix4d &transformation_matrix)
	{
		// Construct the transformation matrix from rotation and translation
		transformation_matrix = Eigen::Matrix<double, 4, 4>::Zero();
		// From euler angle to rotation matrix

		transformation_matrix(0, 0) = std::cos(gamma) * std::cos(beta);
		transformation_matrix(0, 1) = -std::sin(gamma) * std::cos(alpha) + std::cos(gamma) * std::sin(beta) * std::sin(alpha);
		transformation_matrix(0, 2) = std::sin(gamma) * std::sin(alpha) + std::cos(gamma) * std::sin(beta) * std::cos(alpha);
		transformation_matrix(1, 0) = std::sin(gamma) * std::cos(beta);
		transformation_matrix(1, 1) = std::cos(gamma) * std::cos(alpha) + std::sin(gamma) * std::sin(beta) * std::sin(alpha);
		transformation_matrix(1, 2) = -std::cos(gamma) * std::sin(alpha) + std::sin(gamma) * std::sin(beta) * std::cos(alpha);
		transformation_matrix(2, 0) = -std::sin(beta);
		transformation_matrix(2, 1) = std::cos(beta) * std::sin(alpha);
		transformation_matrix(2, 2) = std::cos(beta) * std::cos(alpha);

		transformation_matrix(0, 3) = tx;
		transformation_matrix(1, 3) = ty;
		transformation_matrix(2, 3) = tz;
		transformation_matrix(3, 3) = 1.0;

		return 1;
	}

	bool construct_trans_b(const float &tx, const float &ty, const float &tz,
						   const float &alpha, const float &beta, const float &gamma,
						   Eigen::Matrix4d &transformation_matrix)
	{
		// Construct the transformation matrix from rotation and translation
		transformation_matrix.setIdentity();

		//tiny angle simplified version
		transformation_matrix(0, 1) = -gamma;
		transformation_matrix(0, 2) = beta;
		transformation_matrix(1, 0) = gamma;
		transformation_matrix(1, 2) = -alpha;
		transformation_matrix(2, 0) = -beta;
		transformation_matrix(2, 1) = alpha;

		transformation_matrix(0, 3) = static_cast<double>(tx);
		transformation_matrix(1, 3) = static_cast<double>(ty);
		transformation_matrix(2, 3) = static_cast<double>(tz);
		transformation_matrix(3, 3) = static_cast<double>(1);

		return 1;
	}

	//Brief: calculate the Jacobi Matrix of the imaginary part of a quaternion (q1,q2,q3) with regard to its corresponding euler angle (raw,pitch,yaw)
	//for converting the euler angle variance-covariance matrix to quaternion variance-covariance matrix using variance-covariance propagation law
	//Log: Pay attetion to the euler angle order here. I originally use yaw, pitch, roll (z, y', x'') here, the correct one should be roll, pitch, yaw (x, y', z'')
	//reference:
	//1 . http://easyspin.org/easyspin/documentation/eulerangles.html
	//2 . https://en.wikipedia.org/wiki/Euler_angles
	bool get_quat_euler_jacobi(const Eigen::Vector3d &euler_angle, Eigen::Matrix3d &Jacobi, bool xyz_sequence_or_not = true)
	{
		float sin_half_roll, cos_half_roll, sin_half_pitch, cos_half_pitch, sin_half_yaw, cos_half_yaw;

		sin_half_roll = sin(0.5 * euler_angle(0));
		sin_half_pitch = sin(0.5 * euler_angle(1));
		sin_half_yaw = sin(0.5 * euler_angle(2));
		cos_half_roll = cos(0.5 * euler_angle(0));
		cos_half_pitch = cos(0.5 * euler_angle(1));
		cos_half_yaw = cos(0.5 * euler_angle(2));

		//roll pitch yaw (x, y', z'') , used
		if(xyz_sequence_or_not)
		{
			Jacobi(0, 0) = 0.5 * (cos_half_roll * cos_half_pitch * cos_half_yaw + sin_half_roll * sin_half_pitch * sin_half_yaw);
			Jacobi(0, 1) = 0.5 * (-sin_half_roll * sin_half_pitch * cos_half_yaw - cos_half_roll * cos_half_pitch * sin_half_yaw);
			Jacobi(0, 2) = 0.5 * (-sin_half_roll * cos_half_pitch * sin_half_yaw - cos_half_roll * sin_half_pitch * cos_half_yaw);

			Jacobi(1, 0) = 0.5 * (-sin_half_roll * sin_half_pitch * cos_half_yaw + cos_half_roll * cos_half_pitch * sin_half_yaw);
			Jacobi(1, 1) = 0.5 * (cos_half_roll * cos_half_pitch * cos_half_yaw - sin_half_roll * sin_half_pitch * sin_half_yaw);
			Jacobi(1, 2) = 0.5 * (-cos_half_roll * sin_half_pitch * sin_half_yaw + sin_half_roll * cos_half_pitch * cos_half_yaw);

			Jacobi(2, 0) = 0.5 * (-sin_half_roll * cos_half_pitch * sin_half_yaw - cos_half_roll * sin_half_pitch * cos_half_yaw);
			Jacobi(2, 1) = 0.5 * (-cos_half_roll * sin_half_pitch * sin_half_yaw - sin_half_roll * cos_half_pitch * cos_half_yaw);
			Jacobi(2, 2) = 0.5 * (cos_half_roll * cos_half_pitch * cos_half_yaw + sin_half_roll * sin_half_pitch * sin_half_yaw);
		}
		else //yaw pitch roll (z, y', x'') , not used
		{
		    Jacobi(0, 0) = 0.5 * (-sin_half_roll * cos_half_pitch * sin_half_yaw - cos_half_roll * sin_half_pitch * cos_half_yaw);
			Jacobi(0, 1) = 0.5 * (-cos_half_roll * sin_half_pitch * sin_half_yaw - sin_half_roll * cos_half_pitch * cos_half_yaw);
			Jacobi(0, 2) = 0.5 * (cos_half_roll * cos_half_pitch * cos_half_yaw + sin_half_roll * sin_half_pitch * sin_half_yaw);

			Jacobi(1, 0) = 0.5 * (cos_half_roll * cos_half_pitch * sin_half_yaw - sin_half_roll * sin_half_pitch * cos_half_yaw);
			Jacobi(1, 1) = 0.5 * (-sin_half_roll * sin_half_pitch * sin_half_yaw + cos_half_roll * cos_half_pitch * cos_half_yaw);
			Jacobi(1, 2) = 0.5 * (sin_half_roll * cos_half_pitch * cos_half_yaw - cos_half_roll * sin_half_pitch * sin_half_yaw);

			Jacobi(2, 0) = 0.5 * (cos_half_roll * cos_half_pitch * cos_half_yaw - sin_half_roll * sin_half_pitch * sin_half_yaw);
			Jacobi(2, 1) = 0.5 * (-sin_half_roll * sin_half_pitch * cos_half_yaw + cos_half_roll * cos_half_pitch * sin_half_yaw);
			Jacobi(2, 2) = 0.5 * (-sin_half_roll * cos_half_pitch * sin_half_yaw + cos_half_roll * sin_half_pitch * cos_half_yaw);
		}
		return 1;
	}

	bool get_translation_in_station_coor_sys(const Eigen::Matrix4d &T_world, const centerpoint_t &station_in_world, Eigen::Vector3d &t_station)
	{
		Eigen::Vector3d t_ws(station_in_world.x, station_in_world.y, station_in_world.z);
		Eigen::Vector3d t_w(T_world(0, 3), T_world(1, 3), T_world(2, 3));
		Eigen::Matrix3d R_w = T_world.block<3, 3>(0, 0);

		t_station = t_w + R_w * t_ws - t_ws;

		return 1;
	}

	void apply_cloudclock_cp_local_shift(cloudblock_Ptr &block, float shift_x, float shift_y, float shift_z)
	{
		if (block->station_position_available)
		{
			block->local_station.x += shift_x;
			block->local_station.y += shift_y;
			block->local_station.z += shift_z;
		}
		else
		{
			block->local_cp.x += shift_x;
			block->local_cp.y += shift_y;
			block->local_cp.z += shift_z;
		}
	}

	//this function is for speed up the registration process when the point number is a bit too big
	bool keep_less_source_pts(typename pcl::PointCloud<PointT>::Ptr &pc_ground_tc,
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
							  int ground_down_rate = 4, int facade_down_rate = 2, int target_down_rate = 2)
	{
		CFilter<PointT> cfilter;

		cfilter.random_downsample_pcl(pc_ground_tc, (int)(pc_ground_tc->points.size() / target_down_rate));
		cfilter.random_downsample_pcl(pc_facade_tc, (int)(pc_facade_tc->points.size() / target_down_rate));

		cfilter.random_downsample_pcl(pc_ground_sc, (int)(pc_ground_tc->points.size() / ground_down_rate));
		cfilter.random_downsample_pcl(pc_facade_sc, (int)(pc_facade_tc->points.size() / facade_down_rate));
		cfilter.random_downsample_pcl(pc_pillar_sc, (int)(pc_pillar_tc->points.size()));
		cfilter.random_downsample_pcl(pc_beam_sc, (int)(pc_beam_tc->points.size()));
		cfilter.random_downsample_pcl(pc_roof_sc, (int)(pc_roof_tc->points.size()));
		cfilter.random_downsample_pcl(pc_vertex_sc, (int)(pc_vertex_tc->points.size()));
		return true;
	}

	bool intersection_filter( constraint_t &registration_cons,
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
							  float bbx_pad = 1.0)
	{
		CFilter<PointT> cfilter;
		bounds_t intersection_bbx, source_init_guess_bbx_merged;
		std::vector<bounds_t> source_init_guess_bbxs(3);
		cfilter.get_cloud_bbx(pc_ground_sc, source_init_guess_bbxs[0]);
		cfilter.get_cloud_bbx(pc_pillar_sc, source_init_guess_bbxs[1]);
		cfilter.get_cloud_bbx(pc_facade_sc, source_init_guess_bbxs[2]);
		cfilter.merge_bbx(source_init_guess_bbxs, source_init_guess_bbx_merged);
		cfilter.get_intersection_bbx(registration_cons.block1->local_bound, source_init_guess_bbx_merged, intersection_bbx, bbx_pad);
		cfilter.get_cloud_pair_intersection(intersection_bbx,
											pc_ground_tc, pc_pillar_tc, pc_beam_tc, pc_facade_tc, pc_roof_tc, pc_vertex_tc,
											pc_ground_sc, pc_pillar_sc, pc_beam_sc, pc_facade_sc, pc_roof_sc, pc_vertex_sc);
		LOG(INFO) << "Intersection local bounding box filtering done";
	}

	//Coordinate system covertation related functions
	//Brief: Using the Gauss-Newton Least Square Method to solve 4 Degree of Freedom Transformation from no less than 2 points
	//cp_number is the control points' number for LLS calculation, the rest of points are used to check the accuracy;
	bool coord_system_tran_4dof_lls(const std::vector<std::vector<double>> &coordinatesA, const std::vector<std::vector<double>> &coordinatesB,
									Eigen::Matrix4d &TransMatrixA2B, int cp_number, double theta0_degree) //X Y Z yaw
	{
		Vector4d transAB;
		Vector4d temp_trans;

		int iter_num = 0;

		double theta, theta0, dtheta, eps; // in rad
		double yaw_degree;				   // in degree
		double tx, ty, tz;				   // in meter
		double RMSE_check, sum_squaredist;
		int pointnumberA, pointnumberB, pointnumbercheck;
		//std::vector <std::vector<double>> coordinatesAT;

		//cout << "Input the approximate yaw angle in degree" << endl;
		//cin >> theta0_degree;

		dtheta = 9999;
		eps = 1e-9;

		theta0 = theta0_degree / 180 * M_PI; //Original Guess

		pointnumberA = coordinatesA.size();
		pointnumberB = coordinatesB.size();

		sum_squaredist = 0;

		if (cp_number < 2)
		{
			cout << "Error ! Not enough control point number ..." << endl;
			return 0;
		}

		while (abs(dtheta) > eps)
		{

			MatrixXd A_;
			VectorXd b_;

			A_.resize(cp_number * 3, 4);
			b_.resize(cp_number * 3, 1);

			for (int j = 0; j < cp_number; j++)
			{
				// A Matrix
				A_(j * 3, 0) = -coordinatesA[j][0] * sin(theta0) - coordinatesA[j][1] * cos(theta0);
				A_(j * 3, 1) = 1;
				A_(j * 3, 2) = 0;
				A_(j * 3, 3) = 0;

				A_(j * 3 + 1, 0) = coordinatesA[j][0] * cos(theta0) - coordinatesA[j][1] * sin(theta0);
				A_(j * 3 + 1, 1) = 0;
				A_(j * 3 + 1, 2) = 1;
				A_(j * 3 + 1, 3) = 0;

				A_(j * 3 + 2, 0) = 0;
				A_(j * 3 + 2, 1) = 0;
				A_(j * 3 + 2, 2) = 0;
				A_(j * 3 + 2, 3) = 1;

				//b Vector
				b_(j * 3, 0) = coordinatesB[j][0] - coordinatesA[j][0] * cos(theta0) + coordinatesA[j][1] * sin(theta0);
				b_(j * 3 + 1, 0) = coordinatesB[j][1] - coordinatesA[j][0] * sin(theta0) - coordinatesA[j][1] * cos(theta0);
				b_(j * 3 + 2, 0) = coordinatesB[j][2] - coordinatesA[j][2];
			}

			//x=(ATPA)-1(ATPb)
			temp_trans = ((A_.transpose() * A_).inverse()) * A_.transpose() * b_;
			dtheta = temp_trans(0, 0);

			theta0 += dtheta;

			iter_num++;

			//cout << "Result for iteration " << iter_num << " is " << endl
			//<< temp_trans(1, 0) << " , " << temp_trans(2, 0) << " , " << temp_trans(3, 0) << " , " << theta0 * 180 / M_PI <<endl;
		}

		transAB = temp_trans;

		theta = theta0;
		yaw_degree = theta * 180 / M_PI;

		tx = transAB(1, 0);
		ty = transAB(2, 0);
		tz = transAB(3, 0);

		cout.setf(ios::showpoint);
		cout.precision(12);

		cout << "Calculated by Linear Least Square" << endl
			 << "Converged in " << iter_num << " iterations ..." << endl
			 << "Station B 's Coordinate and Orientation in A's System is:" << endl
			 << "X: " << tx << " m" << endl
			 << "Y: " << ty << " m" << endl
			 << "Z: " << tz << " m" << endl
			 << "yaw: " << yaw_degree << " degree" << endl;

		TransMatrixA2B(0, 0) = cos(theta);
		TransMatrixA2B(0, 1) = -sin(theta);
		TransMatrixA2B(0, 2) = 0;
		TransMatrixA2B(0, 3) = tx;

		TransMatrixA2B(1, 0) = sin(theta);
		TransMatrixA2B(1, 1) = cos(theta);
		TransMatrixA2B(1, 2) = 0;
		TransMatrixA2B(1, 3) = ty;

		TransMatrixA2B(2, 0) = 0;
		TransMatrixA2B(2, 1) = 0;
		TransMatrixA2B(2, 2) = 1;
		TransMatrixA2B(2, 3) = tz;

		TransMatrixA2B(3, 0) = 0;
		TransMatrixA2B(3, 1) = 0;
		TransMatrixA2B(3, 2) = 0;
		TransMatrixA2B(3, 3) = 1;

		cout << "The Transformation Matrix from Coordinate System A to B is: " << endl
			 << TransMatrixA2B << endl;

		// Checking
		if (pointnumberA >= pointnumberB)
			pointnumbercheck = pointnumberB;
		else
			pointnumbercheck = pointnumberA;

		if (pointnumbercheck <= cp_number)
			cout << "Not enough points for check ..." << endl;
		else
		{
			pointnumbercheck -= cp_number;
			for (int j = 0; j < pointnumbercheck; j++)
			{
				double X_tran, Y_tran, Z_tran, squaredist;
				X_tran = cos(theta) * coordinatesA[j + cp_number][0] - sin(theta) * coordinatesA[j + cp_number][1] + tx;
				Y_tran = sin(theta) * coordinatesA[j + cp_number][0] + cos(theta) * coordinatesA[j + cp_number][1] + ty;
				Z_tran = coordinatesA[j + cp_number][2] + tz;

				squaredist = (X_tran - coordinatesB[j + cp_number][0]) * (X_tran - coordinatesB[j + cp_number][0]) +
							 (Y_tran - coordinatesB[j + cp_number][1]) * (Y_tran - coordinatesB[j + cp_number][1]) +
							 (Z_tran - coordinatesB[j + cp_number][2]) * (Z_tran - coordinatesB[j + cp_number][2]);
				sum_squaredist += squaredist;
			}

			RMSE_check = sqrt(sum_squaredist / pointnumbercheck);

			cout << "Calculated from " << pointnumbercheck << " points, the RMSE is " << RMSE_check << endl;
		}

		return 1;
	}

	//Brief: 6DOF transofrmation estimation useing SVD
	bool coord_system_tran_6dof_svd(const std::vector<std::vector<double>> &coordinatesA, const std::vector<std::vector<double>> &coordinatesB, Matrix4d &TransMatrixA2B, int cp_number) //X Y Z roll pitch yaw
	{
		Matrix4d transAB2D;
		pcl::PointCloud<PointT> Points2D_A, Points2D_B;
		double ZAB_mean, ZAB_sum;
		int pointnumberA, pointnumberB, pointnumbercheck;
		double RMSE_check, sum_squaredist;

		pointnumberA = coordinatesA.size();
		pointnumberB = coordinatesB.size();
		ZAB_sum = 0;
		sum_squaredist = 0;

		for (size_t i = 0; i < cp_number; i++)
		{
			PointT PtA, PtB;
			PtA.x = coordinatesA[i][0];
			PtA.y = coordinatesA[i][1];
			PtA.z = coordinatesA[i][2];

			PtB.x = coordinatesB[i][0];
			PtB.y = coordinatesB[i][1];
			PtB.z = coordinatesB[i][2];

			Points2D_A.push_back(PtA);
			Points2D_B.push_back(PtB);
			ZAB_sum += (coordinatesB[i][2] - coordinatesA[i][2]);
		}

		ZAB_mean = ZAB_sum / cp_number;

		if (cp_number < 2)
		{
			cout << "Error ! Not enough control point number ..." << endl;
			return 0;
		}

		pcl::registration::TransformationEstimationSVD<PointT, PointT> svd_estimator;
		svd_estimator.estimateRigidTransformation(Points2D_A, Points2D_B, transAB2D);

		TransMatrixA2B = transAB2D.cast<double>();

		double tx, ty, tz, yaw_rad, yaw_degree;

		tx = TransMatrixA2B(0, 3);
		ty = TransMatrixA2B(1, 3);
		tz = TransMatrixA2B(2, 3);
		yaw_rad = acos(TransMatrixA2B(0, 0));
		if (TransMatrixA2B(1, 0) < 0)
			yaw_rad = -yaw_rad;
		yaw_degree = yaw_rad / M_PI * 180;

		cout << "Calculated by SVD" << endl
			 << "Station B 's Coordinate and Orientation in A's System is:" << endl
			 << "X: " << tx << " m" << endl
			 << "Y: " << ty << " m" << endl
			 << "Z: " << tz << " m" << endl
			 << "yaw: " << yaw_degree << " degree" << endl;

		cout << "The Transformation Matrix from Coordinate System A to B is: " << endl
			 << TransMatrixA2B << endl;

		// Checking
		if (pointnumberA >= pointnumberB)
			pointnumbercheck = pointnumberB;
		else
			pointnumbercheck = pointnumberA;

		if (pointnumbercheck <= cp_number)
			cout << "Not enough points for check ..." << endl;
		else
		{
			pointnumbercheck -= cp_number;
			for (int j = 0; j < pointnumbercheck; j++)
			{
				double X_tran, Y_tran, Z_tran, squaredist;
				X_tran = TransMatrixA2B(0, 0) * coordinatesA[j + cp_number][0] + TransMatrixA2B(0, 1) * coordinatesA[j + cp_number][1] + TransMatrixA2B(0, 2) * coordinatesA[j + cp_number][2] + tx;
				Y_tran = TransMatrixA2B(1, 0) * coordinatesA[j + cp_number][0] + TransMatrixA2B(1, 1) * coordinatesA[j + cp_number][1] + TransMatrixA2B(1, 2) * coordinatesA[j + cp_number][2] + ty;
				Z_tran = TransMatrixA2B(2, 0) * coordinatesA[j + cp_number][0] + TransMatrixA2B(2, 1) * coordinatesA[j + cp_number][1] + TransMatrixA2B(2, 2) * coordinatesA[j + cp_number][2] + tz;

				squaredist = (X_tran - coordinatesB[j + cp_number][0]) * (X_tran - coordinatesB[j + cp_number][0]) +
							 (Y_tran - coordinatesB[j + cp_number][1]) * (Y_tran - coordinatesB[j + cp_number][1]) +
							 (Z_tran - coordinatesB[j + cp_number][2]) * (Z_tran - coordinatesB[j + cp_number][2]);
				sum_squaredist += squaredist;
			}

			RMSE_check = sqrt(sum_squaredist / pointnumbercheck);

			cout << "Calculated from " << pointnumbercheck << " points, the RMSE is " << RMSE_check << endl;
		}
	}

	//Brief: 4 parameters geo-coordiante transformation (tx,ty,r_xy,s)
	bool coord_system_tran_4dof(const std::vector<std::vector<double>> &coordinatesA, const std::vector<std::vector<double>> &coordinatesB, std::vector<double> &transpara, int cp_number) // X Y yaw scale
	{
		double tx, ty, a, b; // 4 parameters
		double s, rot_rad, rot_degree;
		double RMSE_check, sum_squaredist;
		int pointnumberA, pointnumberB, pointnumbercheck;
		Vector4d transAB;
		transpara.resize(5);

		pointnumberA = coordinatesA.size();
		pointnumberB = coordinatesB.size();

		sum_squaredist = 0;

		if (cp_number < 3)
		{
			cout << "Error ! Not enough control point number ..." << endl;
			return 0;
		}

		MatrixXd A_;
		VectorXd b_;

		A_.resize(cp_number * 2, 4);
		b_.resize(cp_number * 2, 1);

		for (int j = 0; j < cp_number; j++)
		{
			// A Matrix
			A_(j * 2, 0) = 1;
			A_(j * 2, 1) = 0;
			A_(j * 2, 2) = coordinatesA[j][0];
			A_(j * 2, 3) = -coordinatesA[j][1];

			A_(j * 2 + 1, 0) = 0;
			A_(j * 2 + 1, 1) = 1;
			A_(j * 2 + 1, 2) = coordinatesA[j][1];
			A_(j * 2 + 1, 3) = coordinatesA[j][0];

			//b Vector
			b_(j * 2, 0) = coordinatesB[j][0];
			b_(j * 2 + 1, 0) = coordinatesB[j][1];
		}
		transAB = ((A_.transpose() * A_).inverse()) * A_.transpose() * b_;

		tx = transAB(0, 0);
		ty = transAB(1, 0);
		a = transAB(2, 0);
		b = transAB(3, 0);
		s = sqrt(a * a + b * b);

		transpara[0] = tx;
		transpara[1] = ty;
		transpara[2] = s;
		transpara[3] = b / s; //sin (ang)
		transpara[4] = a / s; //cos (ang)

		cout.setf(ios::showpoint);
		cout.precision(12);

		cout << "Estimated Transformation From A to B" << endl
			 << "tx: " << tx << " m" << endl
			 << "ty: " << ty << " m" << endl
			 << "scale: " << s << endl;

		// Checking
		if (pointnumberA >= pointnumberB)
			pointnumbercheck = pointnumberB;
		else
			pointnumbercheck = pointnumberA;

		if (pointnumbercheck <= cp_number)
			cout << "Not enough points for check ..." << endl;
		else
		{
			pointnumbercheck -= cp_number;
			for (int j = 0; j < pointnumbercheck; j++)
			{
				double X_tran, Y_tran, squaredist;
				X_tran = transpara[2] * transpara[4] * coordinatesA[j + cp_number][0] - transpara[2] * transpara[3] * coordinatesA[j + cp_number][1] + transpara[0];
				Y_tran = transpara[2] * transpara[3] * coordinatesA[j + cp_number][0] + transpara[2] * transpara[4] * coordinatesA[j + cp_number][1] + transpara[1];
				squaredist = (X_tran - coordinatesB[j + cp_number][0]) * (X_tran - coordinatesB[j + cp_number][0]) +
							 (Y_tran - coordinatesB[j + cp_number][1]) * (Y_tran - coordinatesB[j + cp_number][1]);
				sum_squaredist += squaredist;
			}

			RMSE_check = sqrt(sum_squaredist / pointnumbercheck);

			cout << "Calculated from " << pointnumbercheck << " points, the RMSE is " << RMSE_check << endl;
		}

		return 1;
	}

	//Brief: 7 parameters geo-coordiante transformation (tx,ty,tz,rx,ry,rz,s)
	bool coord_system_tran_7dof(const std::vector<std::vector<double>> &coordinatesA,
								const std::vector<std::vector<double>> &coordinatesB, std::vector<double> &transpara, int cp_number) // X Y Z roll pitch yaw scale
	{
		double RMSE_check, sum_squaredist;
		int pointnumberA, pointnumberB, pointnumbercheck;
		VectorXd transAB;
		transAB.resize(7);
		transpara.resize(7);

		pointnumberA = coordinatesA.size();
		pointnumberB = coordinatesB.size();

		sum_squaredist = 0;

		if (cp_number < 4)
		{
			cout << "Error ! Not enough control point number ..." << endl;
			return 0;
		}

		MatrixXd A_;
		VectorXd b_;

		A_.resize(cp_number * 3, 7);
		b_.resize(cp_number * 3, 1);

		for (int j = 0; j < cp_number; j++)
		{
			// A Matrix   tx ty tz rx ry rz s
			A_(j * 3, 0) = 1;
			A_(j * 3, 1) = 0;
			A_(j * 3, 2) = 0;
			A_(j * 3, 3) = 0;
			A_(j * 3, 4) = -coordinatesA[j][2];
			A_(j * 3, 5) = coordinatesA[j][1];
			A_(j * 3, 6) = coordinatesA[j][0];

			A_(j * 3 + 1, 0) = 0;
			A_(j * 3 + 1, 1) = 1;
			A_(j * 3 + 1, 2) = 0;
			A_(j * 3 + 1, 3) = coordinatesA[j][2];
			A_(j * 3 + 1, 4) = 0;
			A_(j * 3 + 1, 5) = -coordinatesA[j][0];
			A_(j * 3 + 1, 6) = coordinatesA[j][1];

			A_(j * 3 + 2, 0) = 0;
			A_(j * 3 + 2, 1) = 0;
			A_(j * 3 + 2, 2) = 1;
			A_(j * 3 + 2, 3) = -coordinatesA[j][1];
			A_(j * 3 + 2, 4) = coordinatesA[j][0];
			A_(j * 3 + 2, 5) = 0;
			A_(j * 3 + 2, 6) = coordinatesA[j][2];

			//b Vector
			b_(j * 3, 0) = coordinatesB[j][0];
			b_(j * 3 + 1, 0) = coordinatesB[j][1];
			b_(j * 3 + 2, 0) = coordinatesB[j][2];
		}
		transAB = ((A_.transpose() * A_).inverse()) * A_.transpose() * b_;

		transpara[0] = transAB(0);
		transpara[1] = transAB(1);
		transpara[2] = transAB(2);
		transpara[3] = transAB(3);
		transpara[4] = transAB(4);
		transpara[5] = transAB(5);
		transpara[6] = transAB(6);

		cout.setf(ios::showpoint);
		cout.precision(10);

		cout << "Estimated Transformation From A to B" << endl
			 << "tx: " << transpara[0] << " m" << endl
			 << "ty: " << transpara[1] << " m" << endl
			 << "tz: " << transpara[2] << " m" << endl
			 << "rx: " << transpara[3] << endl
			 << "ry: " << transpara[4] << endl
			 << "rz: " << transpara[5] << endl
			 << "scale: " << transpara[6] << endl;

		// Checking
		if (pointnumberA >= pointnumberB)
			pointnumbercheck = pointnumberB;
		else
			pointnumbercheck = pointnumberA;

		if (pointnumbercheck <= cp_number)
			cout << "Not enough points for check ..." << endl;
		else
		{
			pointnumbercheck -= cp_number;
			for (int j = 0; j < pointnumbercheck; j++)
			{
				double X_tran, Y_tran, Z_tran, squaredist;
				X_tran = transpara[0] + transpara[6] * coordinatesA[j + cp_number][0] + transpara[5] * coordinatesA[j + cp_number][1] - transpara[4] * coordinatesA[j + cp_number][2];
				Y_tran = transpara[1] + transpara[6] * coordinatesA[j + cp_number][1] - transpara[5] * coordinatesA[j + cp_number][0] + transpara[3] * coordinatesA[j + cp_number][2];
				Z_tran = transpara[2] + transpara[6] * coordinatesA[j + cp_number][2] + transpara[4] * coordinatesA[j + cp_number][0] - transpara[3] * coordinatesA[j + cp_number][1];
				squaredist = (X_tran - coordinatesB[j + cp_number][0]) * (X_tran - coordinatesB[j + cp_number][0]) +
							 (Y_tran - coordinatesB[j + cp_number][1]) * (Y_tran - coordinatesB[j + cp_number][1]) +
							 (Z_tran - coordinatesB[j + cp_number][2]) * (Z_tran - coordinatesB[j + cp_number][2]);
				sum_squaredist += squaredist;
			}

			RMSE_check = sqrt(sum_squaredist / pointnumbercheck);

			cout << "Calculated from " << pointnumbercheck << " points, the RMSE is " << RMSE_check << endl;
		}

		return 1;
	}
};

} // namespace lo

#endif //_INCLUDE_COMMON_REG_HPP
