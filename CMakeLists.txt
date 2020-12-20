
cmake_minimum_required(VERSION 2.8)
SET(CMAKE_CXX_STANDARD 14)

message("Current Cmake version is : " ${CMAKE_VERSION})

project(lidar_odom)

########################
#        brief         #
########################
#Main CMAKEList file for Lidar SLAM 
#By Yue Pan @ETHZ 

#Please follow the README and the install_dep_lib.sh to install the required libs 

########################
# set compile options  #
########################
#delte the cache to rebuild after changing the options

OPTION(BUILD_WITH_CUDA "Build with CUDA for GPU computation" OFF)
OPTION(BUILD_WITH_OPENCV "Build with OpenCV2 for Image related Processing" OFF)
OPTION(BUILD_WITH_LIBLAS "Build with LIBLAS for *.LAS point cloud data IO" OFF)
OPTION(BUILD_WITH_HDF5 "Build with HDF5 for *.H5 point cloud data IO" OFF)
OPTION(BUILD_WITH_PROJ4 "Build with PROJ4 for Geo-coordinate Projection" OFF)
OPTION(BUILD_WITH_CERES "Build with CERES for Non-linear Optimization" ON)
OPTION(BUILD_WITH_G2O "Build with G2O for Non-linear Optimization" OFF)
OPTION(BUILD_WITH_GTSAM "Build with GTSAM for Non-linear Optimization" OFF)
OPTION(BUILD_WITH_SOPHUS "Build with SOPHUS for Lie-Group related Operations" OFF)
OPTION(BUILD_WITH_TEASER "Build with TEASER++ for Global Registration" ON)

OPTION(BUILD_TOOLS "Build the point cloud format transformation tools" ON)
OPTION(BUILD_PAIRWISE_REG "Build the pairwise registration test" ON)
OPTION(BUILD_REPLAY "Build the Lidar SLAM replaying tool" ON)
OPTION(DEBUG_ON "Turn on DEBUG Mode" OFF)

SET(CMAKE_CXX_FLAGS "-O3 -std=c++14 -w -fopenmp -pthread")
SET(CMAKE_C_FLAGS "-O3 -std=c++14 -fopenmp -pthread")

IF(DEBUG_ON)
MESSAGE("[Debug Mode]")
SET(CMAKE_BUILD_TYPE "Debug") 
SET(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g -ggdb")
ELSE()
MESSAGE("[Release Mode]")
SET(CMAKE_BUILD_TYPE "Release") 
SET(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall")
ENDIF()

SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
LIST(APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake_modules")

########################
# find libs & include  #
########################

#Eigen (involved in pcl, not needed)
FIND_PACKAGE(EIGEN3 REQUIRED QUIET)
IF(EIGEN3_FOUND) 
INCLUDE_DIRECTORIES(${EIGEN3_INCLUDE_DIR})
MESSAGE("EIGEN3 [OK]:" ${EIGEN3_INCLUDE_DIR})
ENDIF(EIGEN3_FOUND)

#PCL (neccessary)
#Boost, vtk, flann, eigen are involved in PCL
FIND_PACKAGE(PCL REQUIRED QUIET)
IF(PCL_FOUND) 
INCLUDE_DIRECTORIES(${PCL_INCLUDE_DIRS})
ADD_DEFINITIONS(${PCL_DEFINITIONS})
LIST(REMOVE_ITEM PCL_LIBRARIES "vtkproj4")
MESSAGE("PCL [OK]:" ${PCL_INCLUDE_DIRS})
ENDIF(PCL_FOUND)

#GFLAG (neccessary)
FIND_PACKAGE(GFLAGS REQUIRED QUIET)
IF(GFLAGS_FOUND)
INCLUDE_DIRECTORIES(${GFLAGS_INCLUDE_DIRS})
MESSAGE("GFLAGS [OK]:" ${GFLAGS_INCLUDE_DIRS}) 
ENDIF(GFLAGS_FOUND)

#GLOG (neccessary)
FIND_PACKAGE(GLOG REQUIRED QUIET)
IF(GLOG_FOUND)
INCLUDE_DIRECTORIES(${GLOG_INCLUDE_DIRS})
MESSAGE("GLOG [OK]:" ${GLOG_INCLUDE_DIRS}) 
ENDIF(GLOG_FOUND)

#CUDA (not used)
IF(BUILD_WITH_CUDA)
FIND_PACKAGE(CUDA 9.0 REQUIRED QUIET)
IF(CUDA_FOUND) 
MESSAGE("CUDA [OK]")
ENDIF(CUDA_FOUND)
ENDIF(BUILD_WITH_CUDA)

#HDF5 (optional: enable if your'd like to use H5 format IO )
IF(BUILD_WITH_HDF5)
# DISABLE FIND_PACKAGE HERE BECAUSE WE MAY FIND THE WRONG HDF5 IN ANODCONDA 
#FIND_PACKAGE(HDF5 REQUIRED QUIET) 
#IF(HDF5_FOUND)
#INCLUDE_DIRECTORIES(${HDF5_INCLUDE_DIR})
#MESSAGE("HDF5 [OK]") 
#ADD_DEFINITIONS(-DHDF5_ON)
#ELSE()
SET(HDF5_INCLUDE_DIR /usr/include/hdf5/serial )
INCLUDE_DIRECTORIES(${HDF5_INCLUDE_DIR})
MESSAGE("HDF5 [OK]") 
ADD_DEFINITIONS(-DHDF5_ON)
#ENDIF(HDF5_FOUND)
ENDIF(BUILD_WITH_HDF5)

#LIBLAS (optional: enable if your'd like to use LAS format IO)
IF(BUILD_WITH_LIBLAS)
FIND_PACKAGE(LIBLAS REQUIRED QUIET)
IF(LIBLAS_FOUND) 
INCLUDE_DIRECTORIES(${LIBLAS_INCLUDE_DIR})
MESSAGE("LIBLAS [OK]: " ${LIBLAS_INCLUDE_DIR}) 
ADD_DEFINITIONS(-DLIBLAS_ON)
ENDIF(LIBLAS_FOUND)
ENDIF(BUILD_WITH_LIBLAS)

#PROJ4 (optional: enable if your'd like to do geo-projection)
IF(BUILD_WITH_PROJ4)
FIND_PACKAGE(PROJ4 REQUIRED QUIET)
IF(PROJ4_FOUND)
INCLUDE_DIRECTORIES(${PROJ4_INCLUDE_DIR})
MESSAGE("PROJ4 [OK]")
ADD_DEFINITIONS(-DPROJ4_ON)
ELSE(PROJ4_FOUND)
SET( PROJ4_ROOT /usr/local/include/proj )
INCLUDE_DIRECTORIES(${PROJ4_INCLUDE_DIR})
MESSAGE("PROJ4 [OK]")
ADD_DEFINITIONS(-DPROJ4_ON)
ENDIF(PROJ4_FOUND)
ENDIF(BUILD_WITH_PROJ4)

#CERES (optional)
# glog and gflag are involved in ceres
IF(BUILD_WITH_CERES)
FIND_PACKAGE(Ceres REQUIRED QUIET)
IF(Ceres_FOUND)
INCLUDE_DIRECTORIES(${CERES_INCLUDE_DIRS})
MESSAGE("CERES [OK]:" ${CERES_INCLUDE_DIRS}) 
ADD_DEFINITIONS(-DCERES_ON)
ENDIF(Ceres_FOUND)
ENDIF(BUILD_WITH_CERES)

#G2O (optional)
IF(BUILD_WITH_G2O)
FIND_PACKAGE( G2O REQUIRED QUIET)
IF(G2O_FOUND)
MESSAGE("G2O [OK]") 
INCLUDE_DIRECTORIES( ${G2O_INCLUDE_DIR} )
ADD_DEFINITIONS(-DG2O_ON)
ENDIF(G2O_FOUND)
FIND_PACKAGE( SuiteSparse REQUIRED QUIET)
IF(SuiteSparse_FOUND)
MESSAGE("SUITESPARSE_FOUND [OK]") 
INCLUDE_DIRECTORIES( ${CSPARSE_INCLUDE_DIR} )
ELSE(SuiteSparse_FOUND)
INCLUDE_DIRECTORIES(/usr/include/suitesparse)
ENDIF(SuiteSparse_FOUND)
ENDIF(BUILD_WITH_G2O)

#GTSAM (optional)
IF (BUILD_WITH_GTSAM)
FIND_PACKAGE( GTSAM REQUIRED QUIET)
IF(GTSAM_FOUND)
MESSAGE("GTSAM [OK]: " ${GTSAM_INCLUDE_DIR}) 
INCLUDE_DIRECTORIES( ${GTSAM_INCLUDE_DIR} )
ADD_DEFINITIONS(-DGTSAM_ON)
ENDIF(GTSAM_FOUND)
ENDIF(BUILD_WITH_GTSAM)

# Sophus (optional, only used in baseline registration method fast vgicp)
IF(BUILD_WITH_SOPHUS)
FIND_PACKAGE(Sophus REQUIRED QUIET)
IF(Sophus_FOUND)
MESSAGE("Sophus [OK]")
INCLUDE_DIRECTORIES(${Sophus_INCLUDE_DIRS})
ADD_DEFINITIONS(-DSOPHUS_ON)
ENDIF(Sophus_FOUND)
ENDIF(BUILD_WITH_SOPHUS)

#TEASER++
IF(BUILD_WITH_TEASER)
FIND_PACKAGE(teaserpp REQUIRED QUIET)
IF(teaserpp_FOUND)
INCLUDE_DIRECTORIES(${teaserpp_INCLUDE_DIRS})
MESSAGE("Teaser++ [OK]")
ADD_DEFINITIONS(-DTEASER_ON)
ENDIF(teaserpp_FOUND)
ENDIF(BUILD_WITH_TEASER)

#OpenCV2
IF(BUILD_WITH_OPENCV)
FIND_PACKAGE(OpenCV 3.0 REQUIRED QUIET)
IF(OPENCV_FOUND) 
INCLUDE_DIRECTORIES(${OpenCV_INCLUDE_DIRS})
MESSAGE("OPENCV [OK]: " ${OpenCV_INCLUDE_DIRS}) 
ADD_DEFINITIONS(-DOPENCV_ON)
ENDIF(OPENCV_FOUND)
ENDIF(BUILD_WITH_OPENCV)

# include folder
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/include)
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/include/common)
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/include/nav)
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/include/pgo)
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/include/baseline_reg)
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/include/tools)

# source folder
AUX_SOURCE_DIRECTORY(${PROJECT_SOURCE_DIR}/src SRC_LIST)
MESSAGE("SRC_LIST is " ${SRC_LIST})

########################
#      link libs       #
########################

#link pcl lib (neccessary)
SET(DEP_LIBS ${DEP_LIBS} ${PCL_LIBRARIES})
SET(DEP_LIBS ${DEP_LIBS} ${GFLAGS_LIBRARIES})
SET(DEP_LIBS ${DEP_LIBS} ${GLOG_LIBRARIES})

IF(BUILD_WITH_HDF5)
#link libhdf5 lib (optional)
#link openmpi [needed by hdf5]
SET(DEP_LIBS ${DEP_LIBS} mpi_cxx hdf5_serial libhdf5_cpp.so)
ENDIF(BUILD_WITH_HDF5)

IF(BUILD_WITH_PROJ4)
#link proj4
SET(DEP_LIBS ${DEP_LIBS} ${PROJ4_LIBRARIES})
ENDIF(BUILD_WITH_PROJ4)

IF(BUILD_WITH_LIBLAS)
#link Liblas
SET(DEP_LIBS ${DEP_LIBS} ${LIBLAS_LIBRARY} liblas.so.2.4.0)
ENDIF(BUILD_WITH_LIBLAS)

IF(BUILD_WITH_CERES)
#link ceres lib (optional)
SET(DEP_LIBS ${DEP_LIBS} ${CERES_LIBRARIES})
ENDIF(BUILD_WITH_CERES)

IF(BUILD_WITH_SOPHUS)
#link sophus lib (optional)
SET(DEP_LIBS ${DEP_LIBS}  ${Sophus_LIBRARIES})
ENDIF(BUILD_WITH_SOPHUS)

IF(BUILD_WITH_G2O)
#manually link g2o_libs (optional)
SET(DEP_LIBS ${DEP_LIBS} g2o_types_slam3d g2o_core g2o_stuff g2o_types_sba g2o_csparse_extension)
ENDIF(BUILD_WITH_G2O)

IF(BUILD_WITH_GTSAM)
#link gtsam lib (optional)
SET(DEP_LIBS ${DEP_LIBS} ${GTSAM_LIBRARIES} gtsam)
ENDIF(BUILD_WITH_GTSAM)

IF(BUILD_WITH_TEASER)
#link teaser ++ (optional)
SET(DEP_LIBS ${DEP_LIBS} teaserpp::teaser_registration teaserpp::teaser_io)
ENDIF(BUILD_WITH_TEASER)

IF(BUILD_WITH_OPENCV)
#link opencv (optional)
SET(DEP_LIBS ${DEP_LIBS} ${OpenCV_LIBS})
ENDIF(BUILD_WITH_OPENCV)
      
########################
#   add executables    #
########################

#test lidar odometry
ADD_EXECUTABLE(mulls_slam ${PROJECT_SOURCE_DIR}/test/mulls_slam.cpp ${SRC_LIST} ${REG_SRC_LIST})
TARGET_LINK_LIBRARIES(mulls_slam ${DEP_LIBS})

IF(BUILD_PAIRWISE_REG)
#test pairwise registration (both global and local) [not used]
ADD_EXECUTABLE(mulls_reg ${PROJECT_SOURCE_DIR}/test/mulls_reg.cpp ${SRC_LIST})
TARGET_LINK_LIBRARIES(mulls_reg ${DEP_LIBS})
ENDIF()

IF(BUILD_REPLAY)
#test slam recapping and checking [compile at the first time]
ADD_EXECUTABLE(replay_slam ${PROJECT_SOURCE_DIR}/test/vis_slam.cpp)
TARGET_LINK_LIBRARIES(replay_slam ${DEP_LIBS})
ENDIF()

IF(BUILD_TOOLS)
#format_transformer tool for kitti [compile at the first time]
ADD_EXECUTABLE(bin2pcd ${PROJECT_SOURCE_DIR}/test/format_transformer/kitti_bin2pcd.cpp)
TARGET_LINK_LIBRARIES(bin2pcd ${DEP_LIBS})

#format_transformer tool for semantic kitti [compile at the first time]
ADD_EXECUTABLE(labelbin2pcd ${PROJECT_SOURCE_DIR}/test/format_transformer/semantic_kitti_label2pcd.cpp)
TARGET_LINK_LIBRARIES(labelbin2pcd ${DEP_LIBS})

ADD_EXECUTABLE(txt2pcd ${PROJECT_SOURCE_DIR}/test/format_transformer/txt2pcd.cpp)
TARGET_LINK_LIBRARIES(txt2pcd ${DEP_LIBS})

ENDIF()





