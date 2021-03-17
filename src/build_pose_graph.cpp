// Codes for pose graph construction in LiDAR SLAM
// By Yue Pan 

#include "build_pose_graph.h"

//PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <boost/make_shared.hpp>

namespace lo
{
	bool Constraint_Finder::find_adjacent_constraint_in_strip(strip &blocks_strip, constraints &innerstrip_cons)
	{
		if (!blocks_strip[0].is_single_scanline) //if the cloud block is collected by a multi-scanline sensor, there would be overlapping aera for adjacent cloud blocks, so registration can be applied
			return 0;

		for (int i = 0; i < blocks_strip.size() - 1; i++)
		{
			constraint_t con;
			con.block1 = boost::make_shared<cloudblock_t>(blocks_strip[i]);
			con.block2 = boost::make_shared<cloudblock_t>(blocks_strip[i + 1]);
			con.con_type = ADJACENT; //Adjacent
			innerstrip_cons.push_back(con);
			//problem: make_shared(struct containing eigen)
		}
		return 1;
	}

	bool Constraint_Finder::find_strip_adjacent_constraint(strips &blocks_all, constraints &innerstrip_cons_all)
	{
		for (int i = 0; i < blocks_all.size(); i++)
		{
			constraints innerstrip_cons;

			if (blocks_all[i].size() > 1)
				find_adjacent_constraint_in_strip(blocks_all[i], innerstrip_cons);

			for (int j = 0; j < innerstrip_cons.size(); j++)
			{
				innerstrip_cons_all.push_back(innerstrip_cons[j]);
			}
			innerstrip_cons.clear();
		}

		return 1;
	}

	bool Constraint_Finder::add_adjacent_constraint(cloudblock_Ptrs &blocks, constraints &cons, int node_count)
	{
		if (node_count < 2)
			return false;

		constraint_t adjacent_con;

		adjacent_con.block1 = blocks[node_count - 2];
		adjacent_con.block2 = blocks[node_count - 1];
		adjacent_con.con_type = ADJACENT;
		adjacent_con.Trans1_2 = adjacent_con.block1->pose_lo.inverse() * adjacent_con.block2->pose_lo;
		adjacent_con.information_matrix = blocks[node_count - 2]->information_matrix_to_next;
		cons.push_back(adjacent_con);
		//LOG(WARNING) << "[1] adjacent edge added";

		return true;
	}

	bool Constraint_Finder::add_adjacent_constraint(cloudblock_Ptrs &blocks, constraints &cons, Eigen::Matrix4d &tran1_2, int node_count)
	{
		constraint_t adjacent_con;

		adjacent_con.block1 = blocks[node_count - 2];
		adjacent_con.block2 = blocks[node_count - 1];
		adjacent_con.con_type = ADJACENT;
		adjacent_con.Trans1_2 = tran1_2;
		adjacent_con.information_matrix = blocks[node_count - 2]->information_matrix_to_next;
		cons.push_back(adjacent_con);

		//LOG(WARNING) << "[1] adjacent edge added";

		return true;
	}

	bool Constraint_Finder::clear_registration_constraint(constraints &cons)
	{
		int con_count = cons.size();
		for (int i = 0; i < con_count; i++)
		{
			if (cons[i].con_type == REGISTRATION)
			{
				cons.erase(cons.begin() + i);
				i--;
				con_count--;
			}
		}
		return true;
	}

	bool Constraint_Finder::cancel_registration_constraint(constraints &cons)
	{
		int con_count = cons.size();
		float confidence_thre = 0.2;
		float sigma_thre = 0.3;
		for (int i = 0; i < con_count; i++)
		{
			if (cons[i].con_type == REGISTRATION) //Registration ----> history  //or we will keep the registration edge
			{
				if (cons[i].confidence < confidence_thre || cons[i].sigma > sigma_thre) //Not very reliable edges
					cons[i].con_type = HISTORY;
			}

			if (cons[i].con_type == NONE) //None -----> deleted
			{
				cons.erase(cons.begin() + i);
				i--;
				con_count--;
			}
		}
		return true;
	}

	int Constraint_Finder::find_overlap_registration_constraint(cloudblock_Ptrs &blocks, constraints &cons,
																float neighbor_radius, float min_iou_thre, int adjacent_id_thre,
																bool search_neighbor_2d, int max_neighbor)
	{
		if (blocks.size() < adjacent_id_thre + 2)
			return 0;

		std::vector<int> pointIdx;
		std::vector<float> pointSquaredDistance;
		int count_reg_edge = 0;
		int cur_id = blocks.size() - 1;

		if (search_neighbor_2d) //2D: x,y
		{
			pcl::PointCloud<pcl::PointXY>::Ptr cp_cloud(new pcl::PointCloud<pcl::PointXY>());

			for (int i = 0; i < blocks.size() - 1; i++)
			{
				pcl::PointXY cp;
				cp.x = blocks[i]->pose_lo(0, 3);
				cp.y = blocks[i]->pose_lo(1, 3);
				cp_cloud->push_back(cp);
			}

			pcl::KdTreeFLANN<pcl::PointXY> kdtree;
			kdtree.setInputCloud(cp_cloud);

			pcl::PointXY cp_search;
			cp_search.x = blocks[cur_id]->pose_lo(0, 3);
			cp_search.y = blocks[cur_id]->pose_lo(1, 3);

			kdtree.radiusSearch(cp_search, neighbor_radius, pointIdx, pointSquaredDistance, max_neighbor);
		}
		else //3D: x,y,z
		{
			pcl::PointCloud<pcl::PointXYZ>::Ptr cp_cloud(new pcl::PointCloud<pcl::PointXYZ>());

			for (int i = 0; i < blocks.size() - 1; i++)
			{
				pcl::PointXYZ cp;
				cp.x = blocks[i]->pose_lo(0, 3);
				cp.y = blocks[i]->pose_lo(1, 3);
				cp.z = blocks[i]->pose_lo(2, 3);
				cp_cloud->push_back(cp);
			}

			pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
			kdtree.setInputCloud(cp_cloud);

			pcl::PointXYZ cp_search;
			cp_search.x = blocks[cur_id]->pose_lo(0, 3);
			cp_search.y = blocks[cur_id]->pose_lo(1, 3);
			cp_search.z = blocks[cur_id]->pose_lo(2, 3);

			kdtree.radiusSearch(cp_search, neighbor_radius, pointIdx, pointSquaredDistance, max_neighbor);
		}

		for (int j = 0; j < pointIdx.size(); j++)
		{
			double iou = calculate_iou(blocks[cur_id]->bound, blocks[pointIdx[j]]->bound); //use (global) bbx instead of (local) bbx

			bool is_adjacent = judge_adjacent_by_id(blocks[cur_id]->id_in_strip, blocks[pointIdx[j]]->id_in_strip, adjacent_id_thre);

			if (!is_adjacent)
				LOG(WARNING) << "IOU of candidate registration edge:" << iou;

			//enough IoU of bbx and not adjacent blocks
			if ((!is_adjacent) && (iou > min_iou_thre))
			{
				constraint_t registration_con;
				registration_con.block1 = blocks[pointIdx[j]]; //history map (target)
				registration_con.block2 = blocks[cur_id];	   //current map (source)
				registration_con.con_type = REGISTRATION;
				registration_con.Trans1_2 = registration_con.block1->pose_lo.inverse() * registration_con.block2->pose_lo;
				registration_con.overlapping_ratio = iou;

				cons.push_back(registration_con);
				count_reg_edge++;
			}
		}

		std::sort(cons.begin(), cons.end(), [](const constraint_t &con_a, const constraint_t &con_b) { return con_a.overlapping_ratio > con_b.overlapping_ratio; }); //sort (larger overlapping ratio edge would be registered at first)

		LOG(WARNING) << "[" << count_reg_edge << "] candidate registration edge found";

		return count_reg_edge;
	}

	bool Constraint_Finder::double_check_tran(Eigen::Matrix4d &global_reg_tran, Eigen::Matrix4d &lo_predicted_tran,Eigen::Matrix4d &trusted_tran,
														 double translation_thre, double rotation_deg_thre)
	{

		Eigen::Matrix4d tran_diff = global_reg_tran.inverse() * lo_predicted_tran;

		Eigen::AngleAxisd rs(tran_diff.block<3, 3>(0, 0));
		double rotation_diff_deg = std::abs(rs.angle()) * 180.0 / M_PI;

		Eigen::Vector3d translation_vec = tran_diff.block<3, 1>(0, 3);
		double translation_diff = translation_vec.norm();

		if (rotation_diff_deg > rotation_deg_thre || translation_diff > translation_thre) // we' d like to trust the lo_predicted_tran
		{
			LOG(WARNING) << "Too much difference between global registration and odometry prediction, we' would trust odometry";
			trusted_tran = lo_predicted_tran;
			return false;
		}
		else
		{
			LOG(INFO) << "We' would trust global registration";
			trusted_tran = global_reg_tran;
			return true;
		}
	}

#if 0

    // For the pose graph, use BFS to read the edge's point cloud and do the regsitration (deprecated)
	// This would decrease the number of repeated point cloud import and improve the time efficieny

	//Introduction to the invloved coordinate system:
	//1. World system (UTM or GK or ...), geo-referenced
	//2. Global-shifted Map system, shifted from world system origin to the first seed cloud block's center point
	//3. Local_shifted Local system, shifted from map system to the intersection bbx of the cloud block pair for registration

	//For a certain transformation T {R,t}
	//the element of T in different coordinate system is different
	//the rotation matrix is the same: Rm = Rw = R  (take m and w as two different coordinate system)
	//the translation vector is the same only when R = I
	//tm = ( R - I ) * twm + tw
	bool Constraint_Finder::batch_add_registration_edge_bfs(strip &all_blocks, constraints &all_cons, int visualization_level,
															boost::shared_ptr<pcl::visualization::PCLVisualizer> &pg_viewer)

	{
		DataIo<Point_T> dataio;
		CFilter<Point_T> cfilter;
		CRegistration<Point_T> creg;
		MapViewer<Point_T> mviewer;

		//Visualization setting begin
		bool clear_raw_cloud = true;
		bool launch_real_time_corr_viewer = false;
		bool launch_real_time_reg_viewer = false;

		//Launch realtime viewer
		boost::shared_ptr<pcl::visualization::PCLVisualizer> feature_points_viewer;
		boost::shared_ptr<pcl::visualization::PCLVisualizer> registration_viewer;
		boost::shared_ptr<pcl::visualization::PCLVisualizer> crosssection_viewer;
		if (visualization_level > 1)
		{
			feature_points_viewer = boost::shared_ptr<pcl::visualization::PCLVisualizer>(new pcl::visualization::PCLVisualizer("Feature points viewer [Left: Target Point Cloud, Right: Source Point Cloud]"));
			registration_viewer = boost::shared_ptr<pcl::visualization::PCLVisualizer>(new pcl::visualization::PCLVisualizer("Registration result viewer [Left: before registration, Right: after registration]"));
			crosssection_viewer = boost::shared_ptr<pcl::visualization::PCLVisualizer>(new pcl::visualization::PCLVisualizer("Crosssection viewer [Left: before registration, Right: after registratio]"));
			mviewer.set_interactive_events(feature_points_viewer);
			mviewer.set_interactive_events(registration_viewer);
			mviewer.set_interactive_events(crosssection_viewer);
			clear_raw_cloud = false;
		}

		if (visualization_level > 2)
			launch_real_time_corr_viewer = true;
		if (visualization_level > 3)
			launch_real_time_reg_viewer = true;

		//Visualization setting end

		//Begin
		cloudblock_Ptrs all_block_ptrs;
		for (int i = 0; i < all_blocks.size(); i++)
		{
			cloudblock_Ptr cbp = boost::make_shared<cloudblock_t>(all_blocks[i]);
			all_block_ptrs.push_back(cbp);
		}

		if (all_cons.size() < 1)
		{
			LOG(WARNING) << "No edges has been found";
			return false;
		}

		LOG(INFO) << "Begin batch registration based on BFS";

		int current_edge_number = 0;

		int blocks_num = all_blocks.size();

		if (blocks_num == 0)
		{
			return false;
		}
		else
		{
			// construct adjacent matrix of graph
			std::vector<std::vector<int>> GraphMatrix(blocks_num, std::vector<int>(blocks_num, -1));

			// Assign edge index
			for (int i = 0; i < all_cons.size(); i++)
			{
				if (all_cons[i].con_type == REGISTRATION)
				{
					GraphMatrix[all_cons[i].block1->unique_id][all_cons[i].block2->unique_id] = i;
					GraphMatrix[all_cons[i].block2->unique_id][all_cons[i].block1->unique_id] = i;
				}
			}

			// bfs preparation
			std::queue<int> q;

			std::vector<bool> visited(blocks_num, 0);
			std::vector<int> edge_num(blocks_num, 0);
			int seed_index = 0;

			// count edge number connected to the block
			for (int i = 0; i < blocks_num; i++)
			{
				for (int j = 0; j < blocks_num; j++)
				{
					if (GraphMatrix[i][j] >= 0)
						edge_num[i]++;
				}
			}

			// Get seed block for bfs (seed block is the one with most number of registration edges connected to it up to now)
			auto biggest = std::max_element(edge_num.begin(), edge_num.end());

			bool is_first_cloud_block = true;
			Eigen::Vector3f global_shift_world2map;

			// if no block has edge, stop (now every edge has been processed)
			while (*biggest > 0)
			{
				seed_index = std::distance(edge_num.begin(), biggest);

				LOG(WARNING) << "Seed block found";

				if (is_first_cloud_block)
				{
					dataio.read_pc_cloud_block_las(all_block_ptrs[seed_index], 1);

					dataio.get_global_shift(global_shift_world2map);

					//Write the cosntraint file head with the global shift
					dataio.write_constraint_file_head(param_.constraint_output_file, global_shift_world2map);

					LOG(INFO) << "the global shift from the world (such as UTM) to map coordinate system is:\n"
							  << "x:" << global_shift_world2map(0) << " ,y:" << global_shift_world2map(1) << " ,z:" << global_shift_world2map(2);

					is_first_cloud_block = false;
				}
				else
					dataio.read_pc_cloud_block_las(all_block_ptrs[seed_index], 0);

				cfilter.extract_semantic_pts(all_block_ptrs[seed_index], clear_raw_cloud, param_.vf_downsample_resolution_als, param_.vf_downsample_resolution_tls,
											 param_.vf_downsample_resolution_mls, param_.vf_downsample_resolution_bpls,
											 param_.gf_grid_resolution, param_.gf_max_grid_height_diff, param_.gf_neighbor_height_diff,
											 param_.ground_downsample_rate, param_.nonground_downsample_rate, 
											 param_.pca_neigh_r, param_.pca_neigh_k, param_.pca_linearity_thre, param_.pca_planarity_thre, param_.pca_stablity_thre);

				q.push(seed_index);

				visited[seed_index] = 1;

				LOG(INFO) << "Import block point cloud done [" << seed_index << "].";

				mviewer.is_seed_origin_ = 1;

				// For one connected sub-graph
				while (!q.empty())
				{
					int k = q.front(); // First one in the queue as the searching block

					// if there are some edges connected to this block
					if (edge_num[k] > 0)
					{
						// For each connected block to the searching block
						for (int i = 0; i < blocks_num; i++)
						{
							if (GraphMatrix[k][i] >= 0)
							{
								// the block has not been visited yet
								if (!visited[i])
								{
									dataio.read_pc_cloud_block_las(all_block_ptrs[i], 0);

									cfilter.extract_semantic_pts(all_block_ptrs[i], clear_raw_cloud, param_.vf_downsample_resolution_als, param_.vf_downsample_resolution_tls,
																 param_.vf_downsample_resolution_mls, param_.vf_downsample_resolution_bpls,
																 param_.gf_grid_resolution, param_.gf_max_grid_height_diff, param_.gf_neighbor_height_diff,
																 param_.ground_downsample_rate, param_.nonground_downsample_rate,
																 param_.pca_neigh_r, param_.pca_neigh_k, param_.pca_linearity_thre, param_.pca_planarity_thre, param_.pca_stablity_thre);

									LOG(INFO) << "Import block point cloud done [" << i << "].";

									// The queue is not full
									if (q.size() < param_.cloud_block_capacity)
									{
										q.push(i);
										LOG(INFO) << "Now there are " << q.size() << " blocks in the memory pool [max number: " << param_.cloud_block_capacity << "]";

										visited[i] = 1; //now this block has been visited
									}
									else
									{
										LOG(INFO) << "Now there are " << q.size() << " blocks in the memory pool [max number: " << param_.cloud_block_capacity << "]";
										LOG(WARNING) << "Warnning. Out of Memory.";
										visited[k] = 0; //If there are new block connected to this block, it would be loaded and registered later
										continue;
									}
								}
								//(else) already in the queue, no need to import the point cloud again

								//Registration begin
								LOG(INFO) << "Do the registration between block [" << k << " - " << i << "].\n"
										  << "[" << all_cons[GraphMatrix[k][i]].block1->filename << "] - [" << all_cons[GraphMatrix[k][i]].block2->filename << "]";

								creg.determine_source_target_cloud(all_block_ptrs[k], all_block_ptrs[i],
																   all_cons[GraphMatrix[k][i]]);

								if (visualization_level > 1)
								{
									LOG(WARNING) << "Ground - Silver, Pillar - Green, Beam - Yellow, Facade - Blue, Roof - Red, Vertex - Purple";
									mviewer.display_feature_pts_compare_realtime(all_cons[GraphMatrix[k][i]].block1,
																				 all_cons[GraphMatrix[k][i]].block2,
																				 feature_points_viewer, 200);
								}

								float reg_corr_dis_thre_min_thre = 2.0 * param_.nonground_downsample_rate * param_.vf_downsample_resolution_als;

								if ((!all_cons[GraphMatrix[k][i]].block1->station_position_available) && (!all_cons[GraphMatrix[k][i]].block2->station_position_available))
								{
									//Registration entry
									if (creg.mm_lls_icp(all_cons[GraphMatrix[k][i]], param_.reg_max_iteration_num, param_.reg_corr_dis_thre,
														param_.converge_tran, param_.converge_rot_d, reg_corr_dis_thre_min_thre) < 0)
									{ //process_code < 0 -> registration may have some problem
										LOG(WARNING) << "Regsitration failed, this edge would be deleted.";
										all_cons[GraphMatrix[k][i]].con_type = NONE;
										all_cons[GraphMatrix[k][i]].Trans1_2.setIdentity();
										all_cons[GraphMatrix[k][i]].information_matrix.setIdentity();
									}
								}
								else
								{
									//Add the global registration (mm_lls_icp_4dof_global)
								}

								//save the registration transformation in global shifted world coordinate system

								//Write the edge data
								dataio.write_constraint_file(param_.constraint_output_file, all_cons[GraphMatrix[k][i]]);

								if (visualization_level > 1)
								{
									pcl::PointCloud<Point_T>::Ptr pointCloudS_reg(new pcl::PointCloud<Point_T>());
									pcl::transformPointCloud(*all_cons[GraphMatrix[k][i]].block2->pc_raw, *pointCloudS_reg, all_cons[GraphMatrix[k][i]].Trans1_2);

									mviewer.display_2_pc_compare_realtime(all_cons[GraphMatrix[k][i]].block2->pc_raw, all_cons[GraphMatrix[k][i]].block1->pc_raw,
																		  pointCloudS_reg, all_cons[GraphMatrix[k][i]].block1->pc_raw,
																		  registration_viewer, 200); //Show the registration result

									if (visualization_level > 2)
									{
										std::vector<pcl::PointCloud<Point_T>::Ptr> cross_section_S, cross_section_T, cross_section_SR;
										cfilter.crosssection_4_comp(all_cons[GraphMatrix[k][i]].block2->pc_raw, all_cons[GraphMatrix[k][i]].block1->pc_raw, pointCloudS_reg,
																	cross_section_S, cross_section_T, cross_section_SR);

										mviewer.display_cross_section_compare_realtime(cross_section_S, cross_section_T, cross_section_SR, crosssection_viewer, 100, 10);
									}
								}

								if (visualization_level)
									mviewer.update_pg_realtime(all_cons[GraphMatrix[k][i]], pg_viewer);

								mviewer.is_seed_origin_ = 0;

								current_edge_number++;

								// Report progress
								LOG(INFO) << "Current rate of progress: " << current_edge_number << " / " << registration_con_count_ << " [ " << 100.0 * current_edge_number / registration_con_count_ << " % ]";

								// Free memory
								//all_cons[GraphMatrix[k][i]].free_cloud();

								//Update the adjacent matrix
								GraphMatrix[k][i] = -1;
								GraphMatrix[i][k] = -1;

								//update edge_num vector
								edge_num[k]--;
								edge_num[i]--;
							}
						}
					}

					LOG(INFO) << "Pop out cloud block [" << k << "]";

					all_block_ptrs[q.front()]->free_all(); // free the searching block

					q.pop(); // pop the first block out of the queue

				} // end of a connected sub-graph

				biggest = std::max_element(edge_num.begin(), edge_num.end()); // update the seed point (iter)
			}
		}

		// Write the adjacent edges
		// Assign edge index
		for (int i = 0; i < all_cons.size(); i++)
		{
			if (all_cons[i].con_type == ADJACENT)
			{
				all_cons[i].Trans1_2.setIdentity();
				all_cons[i].information_matrix.setIdentity();
				dataio.write_constraint_file(param_.constraint_output_file, all_cons[i]);
			}
		}

		LOG(INFO) << "Cloud blocks registration based on BFS done";

		return true;
	}

#endif

	bool
	Constraint_Finder::assign_block2constraint(strip &all_blocks, constraints &all_cons)
	{
		int current_cons_count = all_cons.size();

		for (int i = 0; i < current_cons_count; i++)
		{
			all_cons[i].block1 = boost::make_shared<cloudblock_t>(all_blocks[all_cons[i].block1->unique_id]);
			all_cons[i].block2 = boost::make_shared<cloudblock_t>(all_blocks[all_cons[i].block2->unique_id]);
			//LOG(INFO)<<"Assign "<< all_cons[i].unique_id;
		}

		return 1;
	}

	bool Constraint_Finder::judge_adjacent_by_id(int id_1, int id_2, int submap_id_diff)
	{
		bool is_adjacent = false;

		if ((id_1 <= id_2 + submap_id_diff) && (id_1 >= id_2 - submap_id_diff))
		{
			is_adjacent = true;
		}
		return is_adjacent;
	}

	double Constraint_Finder::calculate_iou(bounds_t &bound1, bounds_t &bound2) //2d bbx iou
	{
		double area1 = (bound1.max_x - bound1.min_x) * (bound1.max_y - bound1.min_y);
		double area2 = (bound2.max_x - bound2.min_x) * (bound2.max_y - bound2.min_y);

		double x1 = max_(bound1.min_x, bound2.min_x);
		double y1 = max_(bound1.min_y, bound2.min_y);

		double x2 = min_(bound1.max_x, bound2.max_x);
		double y2 = min_(bound1.max_y, bound2.max_y);

		double w = max_(0, x2 - x1);
		double h = max_(0, y2 - y1);

		double area1i2 = w * h;

		double iou = area1i2 / (area1 + area2 - area1i2);

		return iou;
	}

} // namespace lo