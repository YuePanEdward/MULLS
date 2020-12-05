#!/bin/sh

sequence_id=00
semantic_kitti_input_folder=xxxx/${sequence_id}/
semantic_kitti_output_folder=xxxx//${sequence_id}/label_pcd/

mkdir ${semantic_kitti_output_folder}

./bin/labelbin2pcd ${semantic_kitti_input_folder} ${semantic_kitti_output_folder}