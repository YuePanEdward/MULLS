//
// This file is for the general implements of several famous point cloud processing algorithms
// Dependent 3rd Libs: PCL (>1.7)
// By Yue Pan 
//

#ifndef _INCLUDE_CLOUD_PROCESSING_HPP
#define _INCLUDE_CLOUD_PROCESSING_HPP

//PCL
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/progressive_morphological_filter.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/project_inliers.h>

#include "utility.hpp"

using namespace std;
using namespace Eigen;

namespace lo
{

template <typename PointT>
class CProceesing : public CloudUtility<PointT>
{
public:
	bool ground_filter_pmf(const typename pcl::PointCloud<PointT>::Ptr &cloud, typename pcl::PointCloud<PointT>::Ptr &gcloud,
						   typename pcl::PointCloud<PointT>::Ptr &ngcloud, int max_window_size,
						   float slope, float initial_distance, float max_distance)
	{
		pcl::PointIndicesPtr ground_points(new pcl::PointIndices);
		pcl::ProgressiveMorphologicalFilter<PointT> pmf;
		pmf.setInputCloud(cloud);
		pmf.setMaxWindowSize(max_window_size);	  //20
		pmf.setSlope(slope);					  //1.0f
		pmf.setInitialDistance(initial_distance); //0.5f
		pmf.setMaxDistance(max_distance);		  //3.0f
		pmf.extract(ground_points->indices);

		// Create the filtering object
		pcl::ExtractIndices<PointT> extract;
		extract.setInputCloud(cloud);
		extract.setIndices(ground_points);
		extract.filter(*gcloud);

		//std::cout << "Ground cloud after filtering (PMF): " << std::endl;
		//std::cout << *gcloud << std::endl;

		// Extract non-ground returns
		extract.setNegative(true);
		extract.filter(*ngcloud);

		//std::out << "Non-ground cloud after filtering (PMF): " << std::endl;
		//std::out << *ngcloud << std::endl;

		return 1;
	}

	bool plane_seg_ransac(const typename pcl::PointCloud<PointT>::Ptr &cloud,
						  float threshold, int max_iter, 
						  typename pcl::PointCloud<PointT>::Ptr &planecloud, 
						  pcl::ModelCoefficients::Ptr &coefficients) //Ransac
	{
		pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

		// Create the segmentation object
		pcl::SACSegmentation<PointT> sacseg;

		// Optional
		sacseg.setOptimizeCoefficients(true);

		// Mandatory
		sacseg.setModelType(pcl::SACMODEL_PLANE);
		sacseg.setMethodType(pcl::SAC_RANSAC);
		sacseg.setDistanceThreshold(threshold);
		sacseg.setMaxIterations(max_iter);

		sacseg.setInputCloud(cloud);
		sacseg.segment(*inliers, *coefficients);

		if (inliers->indices.size() == 0)
		{
			PCL_ERROR("Could not estimate a planar model for the given dataset.");
		}

		/*cout << "Model coefficients: " << coefficients->values[0] << " "
		<< coefficients->values[1] << " "
		<< coefficients->values[2] << " "
		<< coefficients->values[3] << std::endl;*/

		//LOG(INFO) << "Model inliers number: " << inliers->indices.size() << std::endl;

		for (size_t i = 0; i < inliers->indices.size(); ++i)
		{
			planecloud->push_back(cloud->points[inliers->indices[i]]);
		}
		return 1;
	}

	bool ground_projection(const typename pcl::PointCloud<PointT>::Ptr &cloud,
						   typename pcl::PointCloud<PointT>::Ptr &projcloud)
	{
		// Create a set of planar coefficients with X=Y=Z=0
		pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
		coefficients->values.resize(4);
		coefficients->values[0] = coefficients->values[1] = coefficients->values[3] = 0;
		coefficients->values[2] = cloud->points[0].z;

		// Create the filtering object
		pcl::ProjectInliers<PointT> proj;
		proj.setModelType(pcl::SACMODEL_PLANE);
		proj.setInputCloud(cloud);
		proj.setModelCoefficients(coefficients);
		proj.filter(*projcloud);

		//cout << "Cloud projection completed" << endl;

		return 1;
	}

	bool alpha_shape(const typename pcl::PointCloud<PointT>::Ptr &cloud, float alpha_value,
					 typename pcl::PointCloud<PointT>::Ptr &boundary_cloud) //Concave Hull Generation with alpha_shape
	{
		pcl::ConcaveHull<PointT> chull;
		chull.setInputCloud(cloud);
		chull.setAlpha(alpha_value);
		chull.reconstruct(boundary_cloud);
		//std::cout<< "Concave hull has: " << boundary_cloud->points.size() << " data points." << endl;
		return 1;
	}

	bool cornerpoint_knn(const typename pcl::PointCloud<PointT>::Ptr &boundary_cloud, int K, float disthreshold, float maxcos,
						 typename pcl::PointCloud<PointT>::Ptr &corner_cloud) //KNN corner point extraction
	{
		pcl::KdTreeFLANN<PointT> kdtree;
		kdtree.setInputCloud(boundary_cloud);

		vector<int> pointIdxNKNSearch(K);		  //index in the order of the distance
		vector<float> pointNKNSquaredDistance(K); //distance square

		for (int i = 0; i < boundary_cloud->size(); i++)
		{

			kdtree.nearestKSearch(boundary_cloud->points[i], K, pointIdxNKNSearch, pointNKNSquaredDistance); // K NN search result
			float max1_max2;
			max1_max2 = sqrt(pointNKNSquaredDistance[K - 1]) - sqrt(pointNKNSquaredDistance[K - 2]);

			float Xa, Xb, Xo, Ya, Yb, Yo, AOpBO, AO, BO, cosAOB;

			Xo = boundary_cloud->points[i].x;
			Yo = boundary_cloud->points[i].y;
			Xa = boundary_cloud->points[pointIdxNKNSearch[K - 1]].x;
			Ya = boundary_cloud->points[pointIdxNKNSearch[K - 1]].y;

			if (max1_max2 < disthreshold) //If the distance between the farthest and second farthest point is smaller than a threshold, then regard them points on the same side
			{
				float maxdis = 0;
				int maxindex = -1;
				float Xc, Yc, Xd, Yd;
				Xc = boundary_cloud->points[pointIdxNKNSearch[K - 2]].x;
				Yc = boundary_cloud->points[pointIdxNKNSearch[K - 2]].y;
				//The second farthest point find the farthest point in the former neighborhood
				for (int j = 0; j < K - 2; j++)
				{
					Xd = boundary_cloud->points[pointIdxNKNSearch[j]].x;
					Yd = boundary_cloud->points[pointIdxNKNSearch[j]].y;

					float dis = sqrt((Xd - Xc) * (Xd - Xc) + (Yd - Yc) * (Yd - Yc));

					if (dis > maxdis)
					{
						maxdis = dis;
						maxindex = j;
					}
				}
				Xb = boundary_cloud->points[pointIdxNKNSearch[maxindex]].x;
				Yb = boundary_cloud->points[pointIdxNKNSearch[maxindex]].y;
			}

			else
			{
				Xb = boundary_cloud->points[pointIdxNKNSearch[K - 2]].x;
				Yb = boundary_cloud->points[pointIdxNKNSearch[K - 2]].y;
			}
			//Calculate the intersection angle
			AOpBO = (Xa - Xo) * (Xb - Xo) + (Ya - Yo) * (Yb - Yo);
			AO = sqrt((Xa - Xo) * (Xa - Xo) + (Ya - Yo) * (Ya - Yo));
			BO = sqrt((Xb - Xo) * (Xb - Xo) + (Yb - Yo) * (Yb - Yo));
			cosAOB = abs(AOpBO / AO / BO);

			if (cosAOB < maxcos)
				corner_cloud->push_back(boundary_cloud->points[i]); //if the angle is smaller than a threshold, we regard it as a corner point
		}

		return 1;
	}

	bool cornerpoint_rnn(const typename pcl::PointCloud<PointT>::Ptr &boundary_cloud,
						 float radius, float disthreshold, float maxcos, typename pcl::PointCloud<PointT>::Ptr &corner_cloud) //Radius corner point extraction
	{
		pcl::KdTreeFLANN<PointT> kdtree;
		kdtree.setInputCloud(boundary_cloud);

		// Neighbors within radius search
		std::vector<int> pointIdxRadiusSearch;
		std::vector<float> pointRadiusSquaredDistance;

		for (int i = 0; i < boundary_cloud->size(); i++)
		{

			if (kdtree.radiusSearch(boundary_cloud->points[i], radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 2)
			{

				int K = pointIdxRadiusSearch.size();

				float max1_max2;
				max1_max2 = sqrt(pointRadiusSquaredDistance[K - 1]) - sqrt(pointRadiusSquaredDistance[K - 2]);

				float Xa, Xb, Xo, Ya, Yb, Yo, AOpBO, AO, BO, cosAOB;

				Xo = boundary_cloud->points[i].x;
				Yo = boundary_cloud->points[i].y;
				Xa = boundary_cloud->points[pointIdxRadiusSearch[K - 1]].x;
				Ya = boundary_cloud->points[pointIdxRadiusSearch[K - 1]].y;

				if (max1_max2 < disthreshold) //If the distance between the farthest and second farthest point is smaller than a threshold, then regard them points on the same side
				{
					float maxdis = 0;
					int maxindex = -1;
					float Xc, Yc, Xd, Yd;
					Xc = boundary_cloud->points[pointIdxRadiusSearch[K - 2]].x;
					Yc = boundary_cloud->points[pointIdxRadiusSearch[K - 2]].y;
					//The second farthest point find the farthest point in the former neighborhood
					for (int j = 0; j < K - 2; j++)
					{
						Xd = boundary_cloud->points[pointIdxRadiusSearch[j]].x;
						Yd = boundary_cloud->points[pointIdxRadiusSearch[j]].y;

						float dis = sqrt((Xd - Xc) * (Xd - Xc) + (Yd - Yc) * (Yd - Yc));

						if (dis > maxdis)
						{
							maxdis = dis;
							maxindex = j;
						}
					}
					Xb = boundary_cloud->points[pointIdxRadiusSearch[maxindex]].x;
					Yb = boundary_cloud->points[pointIdxRadiusSearch[maxindex]].y;
				}

				else
				{
					Xb = boundary_cloud->points[pointIdxRadiusSearch[K - 2]].x;
					Yb = boundary_cloud->points[pointIdxRadiusSearch[K - 2]].y;
				}
				//Calculate the intersection angle
				AOpBO = (Xa - Xo) * (Xb - Xo) + (Ya - Yo) * (Yb - Yo);
				AO = sqrt((Xa - Xo) * (Xa - Xo) + (Ya - Yo) * (Ya - Yo));
				BO = sqrt((Xb - Xo) * (Xb - Xo) + (Yb - Yo) * (Yb - Yo));
				cosAOB = abs(AOpBO / AO / BO);

				if (cosAOB < maxcos)
					corner_cloud->push_back(boundary_cloud->points[i]); //if the angle is smaller than a threshold, we regard it as a corner point
			}
		}
		return 1;
	}


protected:
private:
};
} // namespace lo
#endif //_INCLUDE_CLOUD_PROCESSING_HPP