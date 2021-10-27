cmake_minimum_required(VERSION 3.4)
set(CMAKE_CXX_STANDARD 14)

message("Current Cmake version is : " ${CMAKE_VERSION})

project(lidar_odom CXX)

########################
#        brief         #
########################
#Main CMAKEList file for Lidar SLAM 
#By Yue Pan @ETHZ 

#Please follow the README and the install_dep_lib.sh to install the required libs 

########################
# set compile options  #
########################
#delete the cache to rebuild after changing the options

option(BUILD_WITH_CUDA "Build with CUDA for GPU computation" OFF)
option(BUILD_WITH_OPENCV "Build with OpenCV2 for Image related Processing" ON)
option(BUILD_WITH_LIBLAS "Build with LIBLAS for *.LAS point cloud data IO" OFF)
option(BUILD_WITH_HDF5 "Build with HDF5 for *.H5 point cloud data IO" ON)
option(BUILD_WITH_PROJ4 "Build with PROJ4 for Geo-coordinate Projection" OFF)
option(BUILD_WITH_CERES "Build with CERES for Non-linear Optimization" ON)
option(BUILD_WITH_G2O "Build with G2O for Non-linear Optimization" OFF)
option(BUILD_WITH_GTSAM "Build with GTSAM for Non-linear Optimization" OFF)
option(BUILD_WITH_SOPHUS "Build with SOPHUS for Lie-Group related Operations" OFF)
option(BUILD_WITH_TEASER "Build with TEASER++ for faster Global Registration (or RANSAC would be used)" ON)

option(BUILD_TOOLS "Build the point cloud format transformation tools" ON)
option(BUILD_PAIRWISE_REG "Build the pairwise registration test" ON)
option(BUILD_REPLAY "Build the Lidar SLAM replaying tool" ON)

if(NOT CMAKE_BUILD_TYPE)
  message(STATUS "No build type selected, default to Release")
  set(CMAKE_BUILD_TYPE "Release")
endif()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} $ENV{CXXFLAGS} -Wall -Wno-unused-variable -Werror=return-type")
set(CMAKE_CXX_FLAGS_DEBUG "-O0 -ggdb3")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -DNDEBUG")
set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "-O3 -ggdb3 -DNDEBUG")

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
list(APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake_modules")

########################
# find libs & include  #
########################

set(THREADS_PREFER_PTHREAD_FLAG TRUE)
find_package(Threads REQUIRED)
find_package(Boost REQUIRED COMPONENTS thread)

find_package(OpenMP REQUIRED)
include_directories(${OpenMP_FOUND_INCLUDE_DIRS})
message("OpenMP [OK]:" ${OpenMP_FOUND_INCLUDE_DIRS})

#Eigen (involved in pcl, not needed)
find_package(Eigen3 REQUIRED NO_MODULE QUIET)
message("Eigen3 [OK]")

#PCL (neccessary)
#Boost, vtk, flann, eigen are involved in PCL
find_package(PCL REQUIRED QUIET)
include_directories(${PCL_INCLUDE_DIRS})
add_definitions(${PCL_DEFINITIONS})
list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4")
message("PCL [OK]:" ${PCL_INCLUDE_DIRS})

#GFLAG (neccessary)
find_package(GFLAGS REQUIRED QUIET)
include_directories(${GFLAGS_INCLUDE_DIRS})
message("GFLAGS [OK]:" ${GFLAGS_INCLUDE_DIRS})

#GLOG (neccessary)
find_package(GLOG REQUIRED QUIET)
include_directories(${GLOG_INCLUDE_DIRS})
message("GLOG [OK]:" ${GLOG_INCLUDE_DIRS})

#CUDA (not used)
if (BUILD_WITH_CUDA)
  find_package(CUDA 9.0 REQUIRED QUIET)
  message("CUDA [OK]")
endif (BUILD_WITH_CUDA)

#HDF5 (optional: enable if you'd like to use H5 format IO )
if (BUILD_WITH_HDF5)
  # DISABLE FIND_PACKAGE HERE BECAUSE WE MAY FIND THE WRONG HDF5 IN ANACONDA
  #  find_package(HDF5 REQUIRED QUIET)
  #  if (HDF5_FOUND)
  #    include_directories(${HDF5_INCLUDE_DIR})
  #    message("HDF5 [OK]")
  #    add_definitions(-DHDF5_ON)
  #  else ()
  set(HDF5_INCLUDE_DIR /usr/include/hdf5/serial)
  include_directories(${HDF5_INCLUDE_DIR})
  message("HDF5 [OK]")
  add_definitions(-DHDF5_ON)
  #  endif (HDF5_FOUND)
endif (BUILD_WITH_HDF5)

#LIBLAS (optional: enable if your'd like to use LAS format IO)
if (BUILD_WITH_LIBLAS)
  find_package(LIBLAS REQUIRED QUIET)
  include_directories(${LIBLAS_INCLUDE_DIR})
  message("LIBLAS [OK]: " ${LIBLAS_INCLUDE_DIR})
  add_definitions(-DLIBLAS_ON)
endif (BUILD_WITH_LIBLAS)

#PROJ4 (optional: enable if you'd like to do geo-projection)
if (BUILD_WITH_PROJ4)
  find_package(PROJ4 REQUIRED QUIET)
  if (PROJ4_FOUND)
    include_directories(${PROJ4_INCLUDE_DIR})
    message("PROJ4 [OK]")
    add_definitions(-DPROJ4_ON)
  else (PROJ4_FOUND)
    set(PROJ4_ROOT /usr/local/include/proj)
    include_directories(${PROJ4_INCLUDE_DIR})
    message("PROJ4 [OK]")
    add_definitions(-DPROJ4_ON)
  endif (PROJ4_FOUND)
endif (BUILD_WITH_PROJ4)

#CERES (optional)
# glog and gflag are involved in ceres
if (BUILD_WITH_CERES)
  find_package(Ceres REQUIRED QUIET)
  include_directories(${CERES_INCLUDE_DIRS})
  message("CERES [OK]:" ${CERES_INCLUDE_DIRS})
  add_definitions(-DCERES_ON)
endif (BUILD_WITH_CERES)

#G2O (optional)
if (BUILD_WITH_G2O)
  find_package(G2O REQUIRED QUIET)
  message("G2O [OK]")
  include_directories(${G2O_INCLUDE_DIR})
  add_definitions(-DG2O_ON)
  find_package(SuiteSparse REQUIRED QUIET)
  if (SuiteSparse_FOUND)
    message("SUITESPARSE_FOUND [OK]")
    include_directories(${CSPARSE_INCLUDE_DIR})
  else (SuiteSparse_FOUND)
    include_directories(/usr/include/suitesparse)
  endif (SuiteSparse_FOUND)
endif (BUILD_WITH_G2O)

#GTSAM (optional)
if (BUILD_WITH_GTSAM)
  find_package(GTSAM REQUIRED QUIET)
  message("GTSAM [OK]: " ${GTSAM_INCLUDE_DIR})
  include_directories(${GTSAM_INCLUDE_DIR})
  add_definitions(-DGTSAM_ON)
endif (BUILD_WITH_GTSAM)

# Sophus (optional, only used in baseline registration method fast vgicp)
if (BUILD_WITH_SOPHUS)
  find_package(Sophus REQUIRED QUIET)
  message("Sophus [OK]")
  include_directories(${Sophus_INCLUDE_DIRS})
  add_definitions(-DSOPHUS_ON)
endif (BUILD_WITH_SOPHUS)

#TEASER++
if (BUILD_WITH_TEASER)
  find_package(teaserpp REQUIRED QUIET)
  include_directories(${teaserpp_INCLUDE_DIRS})
  message("Teaser++ [OK]")
  add_definitions(-DTEASER_ON)
endif (BUILD_WITH_TEASER)

#OpenCV2
if (BUILD_WITH_OPENCV)
  find_package(OpenCV REQUIRED QUIET)
  include_directories(${OpenCV_INCLUDE_DIRS})
  message("OPENCV [OK]: " ${OpenCV_INCLUDE_DIRS})
  add_definitions(-DOPENCV_ON)
endif (BUILD_WITH_OPENCV)

# include folder
include_directories(${PROJECT_SOURCE_DIR}/include)
include_directories(${PROJECT_SOURCE_DIR}/include/common)
include_directories(${PROJECT_SOURCE_DIR}/include/nav)
include_directories(${PROJECT_SOURCE_DIR}/include/pgo)
include_directories(${PROJECT_SOURCE_DIR}/include/baseline_reg)
include_directories(${PROJECT_SOURCE_DIR}/include/tools)

# source folder
set(SRC_LIST
  include/baseline_reg/fast_vgicp.h
  include/baseline_reg/fast_vgicp_impl.hpp
  include/baseline_reg/fast_vgicp_utility.h
  include/baseline_reg/fast_vgicp_voxel.h
  include/baseline_reg/gicp_omp.h
  include/baseline_reg/gicp_omp_impl.hpp
  include/baseline_reg/ndt_omp.h
  include/baseline_reg/ndt_omp_impl.hpp
  include/baseline_reg/voxel_grid_covariance_omp.h
  include/baseline_reg/voxel_grid_covariance_omp_impl.hpp
  include/common/cfilter.hpp
  include/common/cprocessing.hpp
  include/common/cregistration.hpp
  include/common/dataio.hpp
  include/common/h5_io.hpp
  include/common/map_viewer.h
  include/common/map_viewer.hpp
  include/common/pca.hpp
  include/common/utility.hpp
  include/nav/common_nav.h
  include/nav/geo_tran.h
  include/nav/odom_error_compute.h
  include/pgo/build_pose_graph.h
  include/pgo/graph_optimizer.h
  include/pgo/map_manager.h

  src/build_pose_graph.cpp
  src/common_nav.cpp
  src/graph_optimizer.cpp
  src/map_manager.cpp
  )

########################
#      link libs       #
########################

set(DEP_LIBS ${DEP_LIBS} Threads::Threads Boost::thread OpenMP::OpenMP_CXX)
set(DEP_LIBS ${DEP_LIBS} Eigen3::Eigen)

#link pcl lib (neccessary)
set(DEP_LIBS ${DEP_LIBS} ${PCL_LIBRARIES})
set(DEP_LIBS ${DEP_LIBS} ${GFLAGS_LIBRARIES})
set(DEP_LIBS ${DEP_LIBS} ${GLOG_LIBRARIES})

if (BUILD_WITH_HDF5)
  #link libhdf5 lib (optional)
  #link openmpi [needed by hdf5]
  set(DEP_LIBS ${DEP_LIBS} mpi_cxx hdf5_serial libhdf5_cpp.so)
endif (BUILD_WITH_HDF5)

if (BUILD_WITH_PROJ4)
  #link proj4
  set(DEP_LIBS ${DEP_LIBS} ${PROJ4_LIBRARIES})
endif (BUILD_WITH_PROJ4)

if (BUILD_WITH_LIBLAS)
  #link Liblas
  set(DEP_LIBS ${DEP_LIBS} ${LIBLAS_LIBRARY} liblas.so.2.4.0)
endif (BUILD_WITH_LIBLAS)

if (BUILD_WITH_CERES)
  #link ceres lib (optional)
  set(DEP_LIBS ${DEP_LIBS} ${CERES_LIBRARIES})
endif (BUILD_WITH_CERES)

if (BUILD_WITH_SOPHUS)
  #link sophus lib (optional)
  set(DEP_LIBS ${DEP_LIBS} ${Sophus_LIBRARIES})
endif (BUILD_WITH_SOPHUS)

if (BUILD_WITH_G2O)
  #manually link g2o_libs (optional)
  set(DEP_LIBS ${DEP_LIBS} g2o_types_slam3d g2o_core g2o_stuff g2o_types_sba g2o_csparse_extension)
endif (BUILD_WITH_G2O)

if (BUILD_WITH_GTSAM)
  #link gtsam lib (optional)
  set(DEP_LIBS ${DEP_LIBS} ${GTSAM_LIBRARIES} gtsam)
endif (BUILD_WITH_GTSAM)

if (BUILD_WITH_TEASER)
  #link teaser ++ (optional)
  set(DEP_LIBS ${DEP_LIBS} teaserpp::teaser_registration teaserpp::teaser_io)
endif (BUILD_WITH_TEASER)

if (BUILD_WITH_OPENCV)
  #link opencv (optional)
  set(DEP_LIBS ${DEP_LIBS} ${OpenCV_LIBS})
endif (BUILD_WITH_OPENCV)

########################
#   add executables    #
########################

#test lidar odometry
add_executable(mulls_slam ${PROJECT_SOURCE_DIR}/test/mulls_slam.cpp ${SRC_LIST} ${REG_SRC_LIST})
target_link_libraries(mulls_slam ${DEP_LIBS})

if (BUILD_PAIRWISE_REG)
  #test pairwise registration (both global and local) [not used]
  add_executable(mulls_reg ${PROJECT_SOURCE_DIR}/test/mulls_reg.cpp ${SRC_LIST})
  target_link_libraries(mulls_reg ${DEP_LIBS})
endif ()

if (BUILD_REPLAY)
  #test slam recapping and checking [compile at the first time]
  add_executable(replay_slam ${PROJECT_SOURCE_DIR}/test/vis_slam.cpp)
  target_link_libraries(replay_slam ${DEP_LIBS})
endif ()

if (BUILD_TOOLS)
  #format_transformer tool for kitti [compile at the first time]
  add_executable(bin2pcd ${PROJECT_SOURCE_DIR}/test/format_transformer/kitti_bin2pcd.cpp)
  target_link_libraries(bin2pcd ${DEP_LIBS})

  #format_transformer tool for semantic kitti [compile at the first time]
  add_executable(labelbin2pcd ${PROJECT_SOURCE_DIR}/test/format_transformer/semantic_kitti_label2pcd.cpp)
  target_link_libraries(labelbin2pcd ${DEP_LIBS})

  add_executable(txt2pcd ${PROJECT_SOURCE_DIR}/test/format_transformer/txt2pcd.cpp)
  target_link_libraries(txt2pcd ${DEP_LIBS})

endif ()