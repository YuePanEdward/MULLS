// By Yue Pan
#ifndef _INCLUDE_COMMON_NAV_H
#define _INCLUDE_COMMON_NAV_H

#include <vector>

#include "utility.hpp"

using namespace std;

namespace lo
{

  class Navigation
  {
  public:
    bool zupt_simple(Eigen::Matrix4d &Trans, float stop_hor_dis_thre = 0.04, float fix_hor_dis_thre = 0.02);

    double cal_velocity(Matrix4ds &history_Trans, int mean_frame_num = 20, int frame_per_second = 10); //10Hz (2s window)

    double cal_translation_from_tranmat(Eigen::Matrix4d &tran_mat);

    double cal_heading_deg_from_tranmat(Eigen::Matrix4d &tran_mat);

    double cal_rotation_deg_from_tranmat(Eigen::Matrix4d &tran_mat);

    //TODO add various filter (KF,EKF,UKF,PF.etc)
    
    //TODO add various motion model templates 
    //regarding the lidar odometry (scan matching) as observation, then together with the motion model, we can do the recursive estimation (filtering)

  protected:
  private:
  };

} // namespace lo

#endif //_INCLUDE_COMMON_NAV_H