#!/bin/sh

#install neccessary (optional) dependent libs
#test pass on Ubuntu 16.04
echo "Make sure your OS is Ubuntu 16.04, or you have to install these dependence on your own"
echo "Begin to intsall all the dependent libs"

mkdir dependent_libs
echo "Create a new folder called dependent_libs at current path"

sudo apt-get update
# you'd better to use the higher version of cmake for compiling TEASER (which may not be installed by apt-get install)
sudo apt-get install cmake 
# ccmake (terminal gui)
sudo apt-get install cmake-curses-gui
sudo apt-get install protobuf-compiler libprotobuf-dev
# google-glog 
sudo apt-get install libgoogle-glog-dev 
# google-gflag
sudo apt-get install libgflags2 libgflags-dev

echo "install [eigen] 3"
sudo apt-get install libeigen3-dev
pkg-config --modversion eigen3
echo "install [eigen] done"

echo "install [boost] 1.58"
sudo apt-get install libboost-all-dev
echo "install [boost] done"

echo "install [pcl] 1.7"
echo "eigen, boost, flann, vtk involved in pcl"
sudo apt-get install libpcl-dev pcl-tools libproj-dev
echo "install [pcl] done"

echo "install [libpcap]"
sudo apt-get install libpcap-dev
echo "install [libpcap] done"

cd dependent_libs

# echo "install [g2o] 20160424 version"
# # the original version may encounter segfault on linux, use the following commit
# #git clone -b 20170730_git https://github.com/RainerKuemmerle/g2o.git 
# git clone -b 20160424_git https://github.com/RainerKuemmerle/g2o.git 
# cd g2o
# mkdir build
# cd build
# cmake ..
# make -j4
# sudo make install
# cd ../..
# echo "install [g2o] done"

echo "install [ceres] 1.14"
echo "install ceres dependent libs: glog, gflags, suitesparse"
# BLAS & LAPACK
sudo apt-get install libatlas-base-dev
# SuiteSparse and CXSparse (optional)
sudo apt-get install libsuitesparse-dev
#clone ceres to local
git clone -b 1.14.0 https://github.com/ceres-solver/ceres-solver.git
cd ceres-solver
mkdir build
cd build
cmake ..
make -j4
sudo make install
cd ../..
echo "install [ceres] done"

# echo "install [gtsam] 4.0"
# # tbb
# sudo apt-get install libtbb-dev
# #clone gtsam to local
# git clone https://bitbucket.org/gtborg/gtsam.git
# cd gtsam
# mkdir build
# cd build
# cmake ..
# make -j4
# sudo make install
# cd ../..
# echo "install [gtsam] done"

echo "install [sophus]"
git clone -b v1.0.0 https://github.com/strasdat/Sophus.git
cd Sophus
mkdir build
cd build
cmake ..
make -j4
sudo make install
cd ../..
echo "install [sophus] done"

echo "install [libLAS] 1.8"
echo "install libLAS dependent libs: geotiff"
sudo apt-get install libgeotiff-dev 
#clone LibLAS to local
git clone https://github.com/libLAS/libLAS.git 
cd libLAS
mkdir build
cd build
cmake ..
make -j4
sudo make install
cd ../..
echo "install [libLAS] done"

echo "install [TEASER++]"
echo "Cmake version >= 3.10 required"
git clone git@github.com:MIT-SPARK/TEASER-plusplus.git
cd TEASER-plusplus
mkdir build
cd build
cmake ..
make -j4
sudo make install
cd ../..
echo "install [TEASER++] done"

echo "install [OpenCV] 3"
echo "install OpenCV dependent libs"
sudo apt-get install build-essential libgtk2.0-dev libavcodec-dev libavformat-dev libpng16-16 libjpeg9 libjpeg.dev libtiff4.dev libswscale-dev libjasper-dev 
git clone https://github.com/opencv/opencv.git
cd opencv
git checkout "3.4.4"
mkdir build
cd build
cmake ..
make -j4
sudo make install
cd ../..
echo "install [OpenCV] done"

echo "install [HDF] 5"
sudo apt-get install libhdf5-serial-dev
sudo apt-get install hdfview
sudo apt-get install libopenmpi-dev openmpi-bin libhdf5-openmpi-dev
echo "install [HDF] done"

# for pcap related operation
sudo apt-get install libpcap-dev

# echo "Optional: install [ros] kinetic"
# echo "Make sure your os is ubuntu 16.04. or you should install other distribution of ros"
# #refer to http://wiki.ros.org/kinetic/Installation/Ubuntu
# sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
# sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
# sudo apt-get update
# sudo apt-get install ros-kinetic-desktop-full
# echo "Please make sure there's no other distribution (not konetic) of ros on your system"
# echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
# source ~/.bashrc
# sudo apt install python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential
# sudo rosdep init
# rosdep update
# echo "install [ros] done"

cd ..

echo "For the python toolboxs of the project"
echo "install python dependence"
sudo pip3 install -r ./python/requirements.txt
echo "install python dependence done"
#use pip freeze >requirements.txt to update the requirements
echo "install evaluation tool"
pip install evo --upgrade --no-binary evo
echo "install evaluation tool done"

echo "Finished"

# you might then delete the dependent_libs folder
sudo rm -rf ./dependent_libs

# test pass on Ubuntu 16.04