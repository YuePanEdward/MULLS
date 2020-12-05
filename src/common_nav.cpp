// By Yue Pan
#include "common_nav.h"

namespace lo
{
    bool Navigation::zupt_simple(Eigen::Matrix4d &Trans, float stop_hor_dis_thre, float fix_hor_dis_thre)
    {
        float hor_displacement = std::sqrt(Trans(0, 3) * Trans(0, 3) + Trans(1, 3) * Trans(1, 3));
        // if (hor_displacement < stop_hor_dis_thre)
        // {
        //     LOG(INFO) << "Apply ZUPT";
        //     Trans(2, 3) = 0.0;
        //     if (hor_displacement < fix_hor_dis_thre)
        //         Trans.setIdentity();

        //     return true;
        // }

        Trans(2, 3) = 0.0;

        return true;
    }

    double Navigation::cal_velocity(Matrix4ds &Trans, int mean_frame_num, int frame_per_second)
    {
        int mean_frame_num_used;

        mean_frame_num_used = min_(mean_frame_num, Trans.size());

        double accumulate_tx = 0, accumulate_ty = 0, accumulate_tz = 0;

        int count_sum = 0;

        for (auto iter = Trans.end() - 1; iter >= Trans.end() - mean_frame_num_used; iter--)
        {
            //LOG(INFO) << *iter;
            Eigen::Matrix4d tempMat = *iter;
            //simple implement
            accumulate_tx += tempMat(0, 3);
            accumulate_ty += tempMat(1, 3);
            accumulate_tz += tempMat(2, 3);
            count_sum++;
        }

        double vx = accumulate_tx / mean_frame_num_used * frame_per_second;
        double vy = accumulate_ty / mean_frame_num_used * frame_per_second;
        double vz = accumulate_tz / mean_frame_num_used * frame_per_second;

        double mean_linear_velocity = std::sqrt(vx * vx + vy * vy + vz * vz);

        LOG(INFO) << "current approximate velocity: " << mean_linear_velocity * 3.6 << " (km/h)\n";

        return mean_linear_velocity;
    }

    double Navigation::cal_translation_from_tranmat(Eigen::Matrix4d &tran_mat)
    {
        Eigen::Vector3d translation_vec;
        translation_vec = tran_mat.block<3, 1>(0, 3);
        return (translation_vec.norm());
    }

    double Navigation::cal_heading_deg_from_tranmat(Eigen::Matrix4d &tran_mat)
    {
        Eigen::Vector3d euler_angle = (tran_mat.block<3, 3>(0, 0)).eulerAngles(0, 1, 2); //rotation axis : z,y',x''

        double roll_deg = std::abs(euler_angle(0) / M_PI * 180.0);
        double pitch_deg = std::abs(euler_angle(1) / M_PI * 180.0);
        double yaw_deg = std::abs(euler_angle(2) / M_PI * 180.0);

        if (roll_deg > 90)
            roll_deg = 180 - roll_deg;
        if (pitch_deg > 90)
            pitch_deg = 180 - pitch_deg;
        if (yaw_deg > 90)
            yaw_deg = 180 - yaw_deg;

        return yaw_deg;
    }

    double Navigation::cal_rotation_deg_from_tranmat(Eigen::Matrix4d &tran_mat)
    {
        Eigen::AngleAxisd rs(tran_mat.block<3, 3>(0, 0));

        double rotation_deg = std::abs(rs.angle()) * 180.0 / M_PI;
        return rotation_deg;
    }

    //TODO add the ground based extended kalman filter (ekf)

    //TODO add the scene flow (tracking of the active objects)

} // namespace lo