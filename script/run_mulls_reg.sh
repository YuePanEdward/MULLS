#!/bin/sh
#########################################################################################
#                     MULLS pairwise point cloud registration                           #
############################# part to configure (down)###################################

#data path (*.pcd, *.las, *.ply, *.txt, *.h5)
tpc_path=xxxx/dummy_target_point_cloud.xxx
spc_path=xxxx/dummy_source_point_cloud.xxx
opc_path=xxxx/dummy_transformed_source_point_cloud.xxx

#run
#gdb --args \
./bin/mmlls_reg \
--colorlogtostderr=true \
-stderrthreshold 0 \
-log_dir ./log/test \
--v=10 \
--point_cloud_1_path=${tpc_path} \
--point_cloud_2_path=${spc_path} \
--output_point_cloud_path=${opc_path} \
--appro_coordinate_file=${station_pos} \
--cloud_1_down_res=0.08 \
--cloud_2_down_res=0.08 \
--pca_neighbor_radius=1.0 \
--pca_neighbor_count=50 \
--gf_grid_size=2.0 \
--gf_in_grid_h_thre=0.25 \
--gf_neigh_grid_h_thre=1.2 \
--gf_ground_down_rate=10 \
--gf_nonground_down_rate=3 \
--linearity_thre=0.62 \
--planarity_thre=0.65 \
--curvature_thre=0.1 \
--fixed_num_corr_on=true \
--corr_num=5000 \
--corr_dis_thre=2.5 \
--converge_tran=0.0005 \
--converge_rot_d=0.002 \
--reg_max_iter_num=20 \
--teaser_on=true \
--realtime_viewer_on=true \
