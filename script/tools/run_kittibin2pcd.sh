#!/bin/sh

sequence_id=00
infolder=xxxxx/${sequence_id}
mkdir ${infolder}/pcd
for file in ${infolder}/velodyne/*.bin
do 
	./bin/bin2pcd $file ${infolder}/pcd/`basename $file .bin`.pcd
done
