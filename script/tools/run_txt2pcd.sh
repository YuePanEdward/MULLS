#!/bin/sh

infolder=xxxx
mkdir ${infolder}/pcd
for file in ${infolder}/txt/*.txt
do 
	./bin/txt2pcd $file ${infolder}/pcd/`basename $file .txt`.pcd
done
