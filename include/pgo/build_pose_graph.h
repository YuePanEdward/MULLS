// Codes for pose graph construction in LiDAR SLAM
// By Yue Pan 

#ifndef _INCLUDE_FIND_CONSTRAINT_H
#define _INCLUDE_FIND_CONSTRAINT_H

#include <vector>
#include <queue>

#include "utility.hpp"
#include "cfilter.hpp"
#include "cregistration.hpp"
#include "dataio.hpp"
#include "map_viewer.hpp"

using namespace std;

namespace lo
{
	class Constraint_Finder
	{
	public:
		Constraint_Finder()
		{
			;
		}

		Constraint_Finder(constraint_param_t &param)
		{
			param_ = param;
		}

		// Constraint Type 1
		bool find_strip_adjacent_constraint(strips &blocks_all, constraints &innerstrip_cons_all);
		bool find_adjacent_constraint_in_strip(strip &blocks_strip, constraints &innerstrip_cons);

		bool add_adjacent_constraint(cloudblock_Ptrs &blocks, constraints &cons, int node_count); //add to cons

		bool add_adjacent_constraint(cloudblock_Ptrs &blocks, constraints &cons, Eigen::Matrix4d &tran1_2, int node_index);

		// Constraint Type 2
		int find_overlap_registration_constraint(cloudblock_Ptrs &blocks, constraints &cons,
												 float neighbor_radius = 50.0, float min_iou_thre = 0.25, int adjacent_id_thre = 3,
												 bool search_neighbor_2d = true, int max_neighbor = 10); //add to cons [search_neighbor_2d: true 2d, false 3d]

		bool clear_registration_constraint(constraints &cons);

		bool cancel_registration_constraint(constraints &cons);

		bool batch_add_registration_edge_bfs(strip &all_blocks, constraints &all_cons, int visualization_level,
											 boost::shared_ptr<pcl::visualization::PCLVisualizer> &pg_viewer);

		bool assign_block2constraint(strip &all_blocks, constraints &all_cons);

		bool double_check_tran(Eigen::Matrix4d &global_reg_tran, Eigen::Matrix4d &lo_predicted_tran, Eigen::Matrix4d &trusted_tran,
												  double translation_thre = 8.0, double rotation_deg_thre = 45.0);

	protected:
	private:
		void find_neighbor_k_cps(centerpoint_t &cp_search, vector<centerpoint_t> &cp_points, int k);
		void find_neighbor_r_cps(centerpoint_t &cp_search, vector<centerpoint_t> &cp_points, float r);

		double calculate_iou(bounds_t &bound1, bounds_t &bound2);
		bool judge_adjacent_by_id(int id_1, int id_2, int submap_id_diff = 3);

		constraint_param_t param_;

		int total_con_count_;
		int registration_con_count_;
		int adjacent_con_count_;
		int ALS_adjacent_con_count_;
		int MLS_adjacent_con_count_;
		int BPLS_adjacent_con_count_;
	};
} // namespace lo
#endif //F_CONSTRAINT_H