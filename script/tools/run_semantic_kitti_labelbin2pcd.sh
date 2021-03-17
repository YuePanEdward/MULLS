#!/bin/sh

sequence_id=02
semantic_kitti_input_folder=/media/edward/BackupPlus/Data/kitti-dataset/sequences/${sequence_id}/
semantic_kitti_output_folder=/media/edward/BackupPlus/Data/kitti-dataset/sequences/${sequence_id}/label_pcd/

mkdir ${semantic_kitti_output_folder}

./bin/labelbin2pcd ${semantic_kitti_input_folder} ${semantic_kitti_output_folder}