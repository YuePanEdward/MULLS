#!/bin/bash
# exit on errors
set -e

# can set the number of cores to build with, mostly to limit memory usage on CI
NPROC=${NPROC:-$(nproc)}
# can also set $KEEP to not remove the build dirs when finished

#install necessary (optional) dependent libs
#test pass on Ubuntu 16.04
echo "Make sure your OS is Ubuntu 16.04, or you have to install these dependence on your own"
echo "Begin to install all the dependent libs"

mkdir dependent_libs
cd dependent_libs
echo "Create a new folder called dependent_libs at current path"

# you'd better to use the higher version of cmake for compiling TEASER (which may not be installed by apt-get install)
sudo apt-get install -y cmake cmake-curses-gui

sudo apt-get install -y protobuf-compiler libprotobuf-dev libgoogle-glog-dev libgflags-dev

function checkinstall-auto {
  name=$1
  version=$2
  shift 2
  checkinstall -y --install=no --fstrans=yes --nodoc --exclude=/home --pkgname="$name" --pkgversion="$version" "$@"
  sudo apt install -y ./"$name"_*.deb
}

echo "install [eigen] 3"
sudo apt-get install -y libeigen3-dev
pkg-config --modversion eigen3
echo "install [eigen] done"

echo "install [boost]"
sudo apt-get install -y libboost-thread-dev
echo "install [boost] done"

echo "install [pcl]"
echo "eigen, boost, flann, vtk involved in pcl"
sudo apt-get install -y libpcl-dev libproj-dev
echo "install [pcl] done"

# echo "install [g2o] 20160424 version"
# # the original version may encounter segfault on linux, use the following commit
# #git clone -b 20170730_git https://github.com/RainerKuemmerle/g2o.git
# git clone -b 20160424_git https://github.com/RainerKuemmerle/g2o.git
# (
#   cd g2o
#   mkdir build
#   cd build
#   cmake ..
#   make -j $NPROC
#   checkinstall-auto libg2o-dev 0.0.0
# )
# [ -z "$KEEP" ] && rm -rf g2o
# echo "install [g2o] done"

echo "install [ceres]"
# BLAS & LAPACK
sudo apt-get install -y libatlas-base-dev
# SuiteSparse and CXSparse (optional)
sudo apt-get install -y libsuitesparse-dev
#clone ceres to local
git clone -b 2.0.0 --depth 1 https://github.com/ceres-solver/ceres-solver.git
(
  cd ceres-solver
  mkdir build
  cd build
  cmake .. -DBUILD_TESTING=OFF -DBUILD_EXAMPLES=OFF -DBUILD_BENCHMARKS=OFF
  make -j $NPROC
  checkinstall-auto libceres-dev 2.0.0
)
[ -z "$KEEP" ] && rm -rf ceres-solver
echo "install [ceres] done"

# echo "install [gtsam] 4.0"
# # tbb
# sudo apt-get install -y libtbb-dev
# #clone gtsam to local
# git clone --depth 1 https://bitbucket.org/gtborg/gtsam.git
# (
#   cd gtsam
#   mkdir build
#   cd build
#   cmake ..
#   make -j $NPROC
#   checkinstall-auto libgtsam-dev 1.14.0
# )
# [ -z "$KEEP" ] && rm -rf gtsam
# echo "install [gtsam] done"

echo "install [sophus]"
git clone --depth 1 https://github.com/strasdat/Sophus.git
(
  cd Sophus
  mkdir build
  cd build
  cmake .. -DBUILD_TESTS=OFF -DBUILD_EXAMPLES=OFF
  make -j $NPROC
  checkinstall-auto libsophus-dev 0.0.0
)
[ -z "$KEEP" ] && rm -rf Sophus
echo "install [sophus] done"

echo "install [libLAS]"
echo "install libLAS dependent libs: geotiff"
sudo apt-get install -y libgeotiff-dev
#clone LibLAS to local
git clone --depth 1 https://github.com/libLAS/libLAS.git
(
  cd libLAS
  mkdir build
  cd build
  cmake .. -DWITH_TESTS=OFF
  make -j $NPROC
  checkinstall-auto liblas-dev 0.0.0
)
[ -z "$KEEP" ] && rm -rf libLAS
echo "install [libLAS] done"

echo "install [TEASER++]"
echo "Cmake version >= 3.10 required"
git clone --depth 1 https://github.com/MIT-SPARK/TEASER-plusplus.git
(
  cd TEASER-plusplus
  mkdir build
  cd build
  cmake .. -DBUILD_TESTS=OFF
  make -j $NPROC
  checkinstall-auto libteaser-dev 0.0.0
  sudo ldconfig
)
[ -z "$KEEP" ] && rm -rf TEASER-plusplus
echo "install [TEASER++] done"

echo "install [OpenCV]"
echo "install OpenCV dependent libs"
sudo apt-get install -y libopencv-dev
echo "install [OpenCV] done"

echo "install [HDF] 5"
sudo apt-get install -y libhdf5-serial-dev libopenmpi-dev openmpi-bin libhdf5-openmpi-dev
echo "install [HDF] done"

# echo "Optional: install [ros] kinetic"
# echo "Make sure your os is ubuntu 16.04. or you should install other distribution of ros"
# #refer to http://wiki.ros.org/kinetic/Installation/Ubuntu
# sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
# sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
# sudo apt-get update
# sudo apt-get install -y ros-kinetic-desktop-full
# echo "Please make sure there's no other distribution (not konetic) of ros on your system"
# echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
# source ~/.bashrc
# sudo apt install python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential
# sudo rosdep init
# rosdep update
# echo "install [ros] done"
echo "Finished"

cd ..

echo "For the python toolboxes of the project"
echo "install python dependence"
python3 -m pip install --upgrade -r ./python/requirements.txt
echo "install python dependence done"

# you might then delete the dependent_libs folder
[ -z "$KEEP" ] && rm -rf ./dependent_libs

# test pass on Ubuntu 16.04