#!/bin/sh

sequence_id=03
infolder=/media/edward/BackupPlus/Data/kitti-dataset/sequences/${sequence_id}
mkdir ${infolder}/pcd
for file in ${infolder}/velodyne/*.bin
do 
	./bin/bin2pcd $file ${infolder}/pcd/`basename $file .bin`.pcd
done
