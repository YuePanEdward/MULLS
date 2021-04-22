"""
Python scripts for LiDAR SLAM evaluation and plotting
By Yue Pan et al.
"""

import sys
import numpy as np
import transforms3d
from matplotlib import pyplot as plt
import csv

class KittiEvalOdom():
    def __init__(self, LsRtFile, GtRtFile, AdjRtFile, CalibRtFile, TimingFile, EvalFile):
        self.plot_3d = False
        self.plot_loop_closure = True # False
        self.plot_loop_closure_adj_error = False
        self.rotate = False
        self.GtRtFile = GtRtFile #
        self.LsRtFile = LsRtFile #
        self.AdjRtFile = AdjRtFile #
        self.CalibRtFile = CalibRtFile #
        self.TimingFile = TimingFile #
        self.EvalFile = EvalFile #
        self.GtPose = [] # ground truth
        self.LsPose = [] # lidar odom
        self.LoPose = [] # lidar slam (with possible loop closure)
        self.GtPoseLidarframe = []
        self.LsPoseLidarframe = []
        self.LoPoseLidarframe = [] 
        self.CalibRT = 0
        self.AdjTran = [] # Adjacent transformation matrix
        self.TimingArray = []
        self.TimingArraySum = []
        self.frame_num_used = 0
        self.lengths = [100, 200, 300, 400, 500, 600, 700, 800]
        self.num_lengths = len(self.lengths)
        self.saving_path= AdjRtFile[:-4:]  #before .txt
        self.find_candidate_wrong_frame = False
        self.horizontal_error_thre = 0.1  # m
        self.vertical_error_thre = 0.1    # m
        self.yaw_error_thre = 0.5   # deg
        #self.exp_id = datetime.datetime.now().strftime("%H:%M:%S")

    def load_Kitti_calib(self):
        self.CalibRT = np.identity(4)
        with open(self.CalibRtFile,'r') as f:
            data = f.readlines()
            for line in data:
                if line[:2] == 'Tr':
                    RTtmp = np.array(line[3:].split()).reshape(3,4)
        self.CalibRT[:3,:] = RTtmp


    def load_GtPose(self):
        # load Ground truth pose (in camera/body frame)
        with open(self.GtRtFile,'r') as f:
            data = f.readlines()
            data = [i.split(' ') for i in data]
            data = np.array(data, dtype=np.float).reshape(-1, 3, 4)
            data_row4 = np.array([0, 0, 0, 1]).reshape(1, 1, 4).repeat(data.shape[0],axis=0)
            data = np.concatenate([data, data_row4], axis=1)
            self.GtPose = data
            # camera/body to lidar frame
            data_calib = np.zeros((data.shape[0], 4, 4))
            for i in range(data.shape[0]):
                data_calib[i] = np.linalg.inv(self.CalibRT).dot(data[i]).dot(self.CalibRT)
            self.GtPoseLidarframe = data_calib
        print('Total frame number (ground truth trajectory): ', self.GtPose.shape)

    def load_LoPose(self):
        # load Lidar Odometry (SLAM) pose (in camera/body frame)
        with open(self.LsRtFile,'r') as f:
            data = f.readlines()
            data = [i.split(' ') for i in data]
            data = np.array(data, dtype=np.float).reshape(-1, 3, 4)
            data_row4 = np.array([0, 0, 0, 1]).reshape(1, 1, 4).repeat(data.shape[0],axis=0)
            data = np.concatenate([data, data_row4], axis=1)
            self.LsPose = data
            # camera/body to lidar frame
            data_calib = np.zeros((data.shape[0], 4, 4))
            for i in range(data.shape[0]):
                data_calib[i] = np.linalg.inv(self.CalibRT).dot(data[i]).dot(self.CalibRT)
            self.LsPoseLidarframe = data_calib
        print('Total frame number (Lidar odometry/SLAM trajectory): ', self.LsPose.shape)
    
    def load_Adj_tran(self):
        with open(self.AdjRtFile,'r') as f:
            data = f.readlines()
            data = [i.split(' ') for i in data]
            data = np.array(data, dtype=np.float).reshape(-1, 3, 4)
            data_row4 = np.array([0, 0, 0, 1]).reshape(1, 1, 4).repeat(data.shape[0],axis=0)
            data = np.concatenate([data, data_row4], axis=1)
            self.AdjTran = data
            # get lidar odom
            data_post = np.zeros((data.shape[0]+1, 4, 4))
            data_post[0] = np.eye(4)
            for i in range(data.shape[0]):
                data_post[i+1] = data_post[i].dot(np.linalg.inv(data[i]))
            self.LoPoseLidarframe = data_post
            # lidar to camera/body frame    
            data_calib = np.zeros((data.shape[0]+1, 4, 4))
            for i in range(data.shape[0]+1):
                data_calib[i] = self.CalibRT.dot(self.LoPoseLidarframe[i]).dot(np.linalg.inv(self.CalibRT))
            self.LoPose = data_calib
    
    def load_Timing(self):
        with open(self.TimingFile,'r') as f:
            data = f.readlines()
            data = [i.split(' ') for i in data]
            data = np.array(data, dtype=np.float).reshape(-1, 4)
            data_sum = data
            for i in range(data.shape[0]):
                data_sum[i,1]= data_sum[i,0]+ data_sum[i,1]
                data_sum[i,2]= data_sum[i,1]+ data_sum[i,2]
                data_sum[i,3]= data_sum[i,2]+ data_sum[i,3]
            self.frame_num_used=data.shape[0]
            self.TimingArray = data
            self.TimingArraySum = data_sum

    def Pose2AdjTran(self,RTs):
        # from RT based first frame to RT between nearby two frames
        result = []
        for i in range(1,RTs.shape[0]):
            RTPre = RTs[i-1]
            RTCur = RTs[i]
            TmpRT = np.matmul(np.linalg.inv(RTPre), RTCur)
            TmpRT = np.linalg.inv(TmpRT)
            result.append(TmpRT)
        result = np.array(result).reshape(-1,4,4)
        return result

    def trajectory_distances(self, RTs):
        """Compute distance for each pose w.r.t frame-0
        Args:
            poses (dict): {idx: 4x4 array}
        Returns:
            dist (float list): distance of each pose w.r.t frame-0
        """
        dist = [0]

        for i in range(RTs.shape[0]-1):
            cur_frame_idx = i
            next_frame_idx = i+1
            P1 = RTs[cur_frame_idx]
            P2 = RTs[next_frame_idx]
            dx = P1[0, 3] - P2[0, 3]
            dy = P1[1, 3] - P2[1, 3]
            dz = P1[2, 3] - P2[2, 3]
            dist.append(dist[i] + np.sqrt(dx ** 2 + dy ** 2 + dz ** 2))
        return dist

    def rotation_error(self, pose_error):
        """Compute rotation error
        Args:
            pose_error (4x4 array): relative pose error
        Returns:
            rot_error (float): rotation error
        """
        a = pose_error[0, 0]
        b = pose_error[1, 1]
        c = pose_error[2, 2]
        d = 0.5 * (a + b + c - 1.0)
        rot_error = np.arccos(max(min(d, 1.0), -1.0))
        return rot_error

    def translation_error(self, pose_error):
        """Compute translation error
        Args:
            pose_error (4x4 array): relative pose error
        Returns:
            trans_error (float): translation error
        """
        dx = pose_error[0, 3]
        dy = pose_error[1, 3]
        dz = pose_error[2, 3]
        trans_error = np.sqrt(dx ** 2 + dy ** 2 + dz ** 2)
        return trans_error

    def last_frame_from_segment_length(self, dist, first_frame, length):
        """Find frame (index) that away from the first_frame with
        the required distance
        Args:
            dist (float list): distance of each pose w.r.t frame-0
            first_frame (int): start-frame index
            length (float): required distance
        Returns:
            i (int) / -1: end-frame index. if not found return -1
        """
        for i in range(first_frame, len(dist), 1):
            if dist[i] > (dist[first_frame] + length):
                return i
        return -1

    def calc_sequence_errors(self, poses_gt, poses_result):
        """calculate sequence error
        Args:
            poses_gt (dict): {idx: 4x4 array}, ground truth poses
            poses_result (dict): {idx: 4x4 array}, predicted poses
        Returns:
            err (list list): [first_frame, rotation error, translation error, length, speed]
                - first_frame: frist frame index
                - rotation error: rotation error per length
                - translation error: translation error per length
                - length: evaluation trajectory length
                - speed: car speed (#FIXME: 10FPS is assumed)
        """
        err = []
        dist = self.trajectory_distances(poses_gt)
        self.step_size = 10

        for first_frame in range(0, len(poses_gt), self.step_size):
            for i in range(self.num_lengths):
                len_ = self.lengths[i]
                last_frame = self.last_frame_from_segment_length(
                    dist, first_frame, len_
                )

                # Continue if sequence not long enough
                if last_frame == -1:
                    continue

                # compute rotational and translational errors
                pose_delta_gt = np.dot(
                    np.linalg.inv(poses_gt[first_frame]),
                    poses_gt[last_frame]
                )
                pose_delta_result = np.dot(
                    np.linalg.inv(poses_result[first_frame]),
                    poses_result[last_frame]
                )
                pose_error = np.dot(
                    np.linalg.inv(pose_delta_result),
                    pose_delta_gt
                )
                r_err = self.rotation_error(pose_error)
                t_err = self.translation_error(pose_error)

                # compute speed
                num_frames = last_frame - first_frame + 1.0
                speed = len_ / (0.1 * num_frames)

                err.append([first_frame, r_err / len_, t_err / len_, len_, speed])
        return err

    def plot_trajectory(self):
        
        font1 = {'family' : 'Times New Roman',
                 'weight' : 'normal',
                 'size'   : 20,
                }
        
        font2 = {'family' : 'Times New Roman',
                 'weight' : 'normal',
                 'size'   : 18,
                }

        fig = plt.figure(figsize=(10,10))
        
        if self.plot_3d:
            ax1 = fig.add_subplot(111, projection='3d')
        else:
            ax1 = fig.add_subplot(111)

        Gt_traj = self.GtPose[:, :3, 3]
        if self.rotate:
            rotate_theta = -np.pi / 4
            Gt_traj = np.dot(Gt_traj, np.array(
                [[np.cos(rotate_theta), -np.sin(rotate_theta)], [np.sin(rotate_theta), np.cos(rotate_theta)]]))
        if self.plot_3d:
            ax1.plot(Gt_traj[:, 0], Gt_traj[:, 1], Gt_traj[:, 2], '--', linewidth=4, color='k')
        else:
            ax1.plot(Gt_traj[:, 0], Gt_traj[:, 2],'--', linewidth=4, color='k')

        Lo_traj = self.LoPose[:, :3, 3]
        if self.rotate:
            rotate_theta = np.pi
            Lo_traj = np.dot(Lo_traj, np.array(
                [[np.cos(rotate_theta), -np.sin(rotate_theta)], [np.sin(rotate_theta), np.cos(rotate_theta)]]))
        if self.plot_3d:
            ax1.plot(Lo_traj[:, 0], Lo_traj[:, 1], Lo_traj[:, 2], linewidth=4, color='r')
        else:
            ax1.plot(Lo_traj[:, 0], Lo_traj[:, 2], linewidth=4, color='r')
        
        if self.plot_loop_closure:
            Ls_traj = self.LsPose[:, :3, 3]
            if self.rotate:
                rotate_theta = np.pi
                Ls_traj = np.dot(Ls_traj, np.array(
                    [[np.cos(rotate_theta), -np.sin(rotate_theta)], [np.sin(rotate_theta), np.cos(rotate_theta)]]))
            if self.plot_3d:
                ax1.plot(Ls_traj[:, 0], Ls_traj[:, 1], Ls_traj[:, 2], linewidth=4, color='b')
            else:
                ax1.plot(Ls_traj[:, 0], Ls_traj[:, 2], linewidth=4, color = 'b')

        plt.tick_params(labelsize=14)
        labels = ax1.get_xticklabels() + ax1.get_yticklabels()
        [label.set_fontname('Times New Roman') for label in labels]
    
        #plt.title('Comparison of trajectory')
        
        if self.plot_3d:
            plt.xlabel('x(m)', font2)
            plt.ylabel('y(m)', font2)
            plt.zlabel('z(m)', font2)
        else:
            plt.xlabel('x(m)', font2)
            plt.ylabel('z(m)', font2)
        
        plt.axis("equal")
        
        if self.plot_loop_closure:
            plt.legend(( 'Ground truth', 'Lidar odometry','Lidar SLAM'), prop=font1)
        else:
            plt.legend(('Ground truth', 'Lidar odometry'), prop=font1)

        plt.savefig(self.saving_path+"_trajectory.jpg", dpi=500)
        plt.show()
    
    '''
    todo: add the error 
    '''
    
    '''
    def plot_change_6dof(self):
        BinRT = self.Pose2AdjTran(self.BinRT)
        GpsRT = self.Pose2AdjTran(self.GpsRT)
        BinEuler = []
        GpsEuler = []
        for i in range(BinRT.shape[0]):
            TmpBinEuler = transforms3d.euler.mat2euler(BinRT[i], 'sxyz')
            TmpGpsEuler = transforms3d.euler.mat2euler(GpsRT[i], 'sxyz')
            BinEuler.append(TmpBinEuler)
            GpsEuler.append(TmpGpsEuler)
        BinEuler = np.array(BinEuler).reshape(-1,3)
        GpsEuler = np.array(GpsEuler).reshape(-1,3)
        fig = plt.figure(figsize=(16,10))
        
        #plt.title('Delta x,y,z,row,pitch,yaw of adjacent frames')
        x = np.arange(BinRT.shape[0])
        ax1 = fig.add_subplot(321)
        ax1.plot(x, BinRT[:, 0, 3], 'r')
        ax1.plot(x, GpsRT[:BinRT.shape[0], 0, 3], 'g')
        plt.ylabel('Delta X (m)')
        plt.xlabel('Frame #')
        plt.legend(('Lidar odometry', 'Ground truth'))
        #ax1.plot(x, np.abs(BinRT[:,0,3] - GpsRT[:BinRT.shape[0],0,3]),'g')
        ax2 = fig.add_subplot(323)
        ax2.plot(x, BinRT[:, 1, 3], 'r')
        ax2.plot(x, GpsRT[:BinRT.shape[0], 1, 3], 'g')
        plt.ylabel('Delta Y (m)')
        plt.xlabel('Frame #')
        plt.legend(('Lidar odometry', 'Ground truth'))
        #ax2.plot(x, np.abs(BinRT[:, 1, 3] - GpsRT[:BinRT.shape[0], 1, 3]), 'g')
        ax3 = fig.add_subplot(325)
        ax3.plot(x, BinRT[:, 2, 3], 'r')
        ax3.plot(x, GpsRT[:BinRT.shape[0], 2, 3], 'g')
        plt.ylabel('Delta Z (m)')
        plt.xlabel('Frame #')
        plt.legend(('Lidar odometry', 'Ground truth'))
        ax4 = fig.add_subplot(322)
        ax4.plot(x, BinEuler[:, 0] / np.pi * 180, 'r')
        ax4.plot(x, GpsEuler[:BinRT.shape[0], 0] / np.pi * 180, 'g')
        plt.ylabel('Delta Roll (deg)')
        plt.xlabel('Frame #')
        plt.legend(('Lidar odometry', 'Ground truth'))
        ax5 = fig.add_subplot(324)
        ax5.plot(x, BinEuler[:, 1]  / np.pi * 180, 'r')
        ax5.plot(x, GpsEuler[:BinRT.shape[0], 1] / np.pi * 180, 'g')
        plt.ylabel('Delta Pitch (deg)')
        plt.xlabel('Frame #')
        plt.legend(('Lidar odometry', 'Ground truth'))
        ax6 = fig.add_subplot(326)
        ax6.plot(x, BinEuler[:, 2] / np.pi * 180, 'r')
        ax6.plot(x, GpsEuler[:BinRT.shape[0], 2] / np.pi * 180, 'g')
        plt.ylabel('Delta Yaw (deg)')
        plt.xlabel('Frame #')
        plt.legend(('Lidar odometry', 'Ground truth'))
        
        plt.savefig(self.saving_path+"_6dof_comparing.jpg", dpi=200)
        #plt.show()
    
    '''
    def plot_timing_detail(self, with_loop=True):
        
        font1 = {'family' : 'Times New Roman',
                 'weight' : 'normal',
                 'size'   : 14,
                }
        
        font2 = {'family' : 'Times New Roman',
                 'weight' : 'normal',
                 'size'   : 16,
                }
        
        fig = plt.figure(figsize=(10,4.5))

        frame_count = self.frame_num_used
        frame_array =  np.arange(frame_count)
        realtime_limit = 100.0 * np.ones([frame_count,1])
        ax1 = fig.add_subplot(111)

        ax1.plot(frame_array, self.TimingArraySum[:, 0],linewidth=1.5, color='b')
        ax1.plot(frame_array, self.TimingArraySum[:, 1],linewidth=1.5, color='g')
        ax1.plot(frame_array, self.TimingArraySum[:, 2],linewidth=1.5, color='r')
        if with_loop==True:
             ax1.plot(frame_array, self.TimingArraySum[:, 3], "-",linewidth=0.2, color='c')
        ax1.plot(frame_array, realtime_limit, "--", linewidth=2, color = 'k')
        
        ax1.fill_between(frame_array, self.TimingArraySum[:, 0],facecolor='b', where=self.TimingArraySum[:, 0] >0 , alpha=0.5, interpolate=True)
        ax1.fill_between(frame_array, self.TimingArraySum[:, 0],self.TimingArraySum[:, 1], facecolor='g', where=self.TimingArraySum[:, 1] >self.TimingArraySum[:, 0] , alpha=0.5, interpolate=True)
        ax1.fill_between(frame_array, self.TimingArraySum[:, 1],self.TimingArraySum[:, 2], facecolor='r', where=self.TimingArraySum[:, 2] >self.TimingArraySum[:, 1] , alpha=0.5, interpolate=True)
        
        #if with_loop==True:
            # ax1.fill_between(frame_array, self.TimingArraySum[:, 2],self.TimingArraySum[:, 3], facecolor='c', where=self.TimingArraySum[:, 3] >self.TimingArraySum[:, 2] , alpha=0.5, interpolate=True)
        
        plt.tick_params(labelsize=12)
        labels = ax1.get_xticklabels() + ax1.get_yticklabels()
        [label.set_fontname('Times New Roman') for label in labels]

        plt.xlim((0,frame_count-1))
        plt.ylim((0,150))
        plt.xlabel('Frame ID', font2)
        plt.ylabel('Consuming time (ms)',font2)
        #plt.title('Timing table')
        
        if with_loop==True:
             plt.legend(( 'Feature extraction', 'Map updating', "Registration", "Loop closure"), prop=font2, loc=2)
        else:
             plt.legend(( 'Feature extraction', 'Map updating', "Registration"), prop=font2, markerscale=5.0, loc=2)
        
        plt.savefig(self.saving_path+"_timing_detail.jpg", dpi=500)
        plt.show()
        #print(self.TimingArraySum)

    def plot_error_change_6dof(self):

        GtAdjRt = self.Pose2AdjTran(self.GtPose)
        LoAdjRt = self.Pose2AdjTran(self.LoPose)
        LsAdjRt = self.Pose2AdjTran(self.LsPose)
         
        GtEuler = []
        LoEuler = []
        LsEuler = []
        
        for i in range(self.frame_num_used-1):
            TmpGtEuler = transforms3d.euler.mat2euler(GtAdjRt[i], 'sxyz')
            TmpLoEuler = transforms3d.euler.mat2euler(LoAdjRt[i], 'sxyz')
            TmpLsEuler = transforms3d.euler.mat2euler(LsAdjRt[i], 'sxyz')
            GtEuler.append(TmpGtEuler)
            LoEuler.append(TmpLoEuler)
            LsEuler.append(TmpLsEuler)
        
        GtEuler = np.array(GtEuler).reshape(-1,3)
        LoEuler = np.array(LoEuler).reshape(-1,3)
        LsEuler = np.array(LsEuler).reshape(-1,3)

        fig = plt.figure(figsize=(16,10))

        frame_count = self.frame_num_used-1
        frame_array =  np.arange(frame_count)

        lo_x_error = np.abs(LoAdjRt[:frame_count, 0, 3] - GtAdjRt[:frame_count, 0, 3])
        lo_y_error = np.abs(LoAdjRt[:frame_count, 1, 3] - GtAdjRt[:frame_count, 1, 3])
        lo_z_error = np.abs(LoAdjRt[:frame_count, 2, 3] - GtAdjRt[:frame_count, 2, 3])

        lo_roll_error = np.abs(LoEuler[:frame_count, 0] - GtEuler[:frame_count, 0]) / np.pi * 180
        lo_pitch_error = np.abs(LoEuler[:frame_count, 1] - GtEuler[:frame_count, 1]) / np.pi * 180
        lo_yaw_error = np.abs(LoEuler[:frame_count, 2] - GtEuler[:frame_count, 2]) / np.pi * 180

        ls_x_error = np.abs(LsAdjRt[:frame_count, 0, 3] - GtAdjRt[:frame_count, 0, 3])
        ls_y_error = np.abs(LsAdjRt[:frame_count, 1, 3] - GtAdjRt[:frame_count, 1, 3])
        ls_z_error = np.abs(LsAdjRt[:frame_count, 2, 3] - GtAdjRt[:frame_count, 2, 3])

        ls_roll_error = np.abs(LsEuler[:frame_count, 0] - GtEuler[:frame_count, 0]) / np.pi * 180
        ls_pitch_error = np.abs(LsEuler[:frame_count, 1] - GtEuler[:frame_count, 1]) / np.pi * 180
        ls_yaw_error = np.abs(LsEuler[:frame_count, 2] - GtEuler[:frame_count, 2]) / np.pi * 180
        
        
        ax1 = fig.add_subplot(321)
        ax1.plot(frame_array, 100*lo_x_error, 'b')
        if self.plot_loop_closure_adj_error:
            ax1.plot(frame_array, 100*ls_x_error, 'g')
        plt.ylabel('Error X (cm)')
        #plt.xlabel('Frame #')
        if self.plot_loop_closure_adj_error:
            plt.legend(( 'Lidar Odometry', 'Lidar SLAM'))
        else:
            plt.legend('Lidar Odometry')
    
        ax2 = fig.add_subplot(323)
        ax2.plot(frame_array, 100*lo_y_error, 'b')
        if self.plot_loop_closure_adj_error:
            ax2.plot(frame_array, 100*ls_y_error, 'g')
        plt.ylabel('Error Y (cm)')
        #plt.xlabel('Frame #')
        if self.plot_loop_closure_adj_error:
            plt.legend(( 'Lidar Odometry', 'Lidar SLAM'))
        else:
            plt.legend('Lidar Odometry')
      
        ax3 = fig.add_subplot(325)
        ax3.plot(frame_array, 100*lo_z_error, 'b')
        if self.plot_loop_closure_adj_error:
            ax3.plot(frame_array, 100*ls_z_error, 'g')
        plt.ylabel('Error Z (cm)')
        plt.xlabel('Frame #')
        if self.plot_loop_closure_adj_error:
            plt.legend(( 'Lidar Odometry', 'Lidar SLAM'))
        else:
            plt.legend('Lidar Odometry')

        ax4 = fig.add_subplot(322)
        ax4.plot(frame_array, lo_roll_error, 'b')
        if self.plot_loop_closure_adj_error:
            ax4.plot(frame_array, ls_roll_error, 'g')
        plt.ylabel('Error Roll (deg)')
        #plt.xlabel('Frame #')
        if self.plot_loop_closure_adj_error:
            plt.legend(( 'Lidar Odometry', 'Lidar SLAM'))
        else:
            plt.legend('Lidar Odometry')

        ax5 = fig.add_subplot(324)
        ax5.plot(frame_array, lo_pitch_error, 'b')
        if self.plot_loop_closure_adj_error:
            ax5.plot(frame_array, ls_pitch_error, 'g')
        plt.ylabel('Error Pitch (deg)')
        #plt.xlabel('Frame #')
        if self.plot_loop_closure_adj_error:
            plt.legend(( 'Lidar Odometry', 'Lidar SLAM'))
        else:
            plt.legend('Lidar Odometry')

        ax6 = fig.add_subplot(326)
        ax6.plot(frame_array, lo_yaw_error, 'b')
        if self.plot_loop_closure_adj_error:
            ax6.plot(frame_array, ls_yaw_error, 'g')
        plt.ylabel('Error Yaw (deg)')
        plt.xlabel('Frame #')
        if self.plot_loop_closure_adj_error:
            plt.legend(( 'Lidar Odometry', 'Lidar SLAM'))
        else:
            plt.legend('Lidar Odometry')
        
        plt.savefig(self.saving_path+"_6dof_error.jpg", dpi=400)
        plt.show()
        
        #check candidate problematic frames
        if self.find_candidate_wrong_frame:
            lo_hor_error=np.sqrt(lo_x_error**2+lo_y_error**2)
        
            hor_candidate_frames=np.asarray(np.nonzero(lo_hor_error>self.horizontal_error_thre))
            ver_candidate_frames=np.asarray(np.nonzero(lo_z_error>self.vertical_error_thre))
            yaw_candidate_frames=np.asarray(np.nonzero(lo_yaw_error>self.yaw_error_thre))

            print ('Candidate problematic frame')
            print ('horizontal translation error > '+ str(self.horizontal_error_thre) + ' m :')
            print (hor_candidate_frames)
            print ('vertical translation error > '+ str(self.vertical_error_thre) + ' m :')
            print (ver_candidate_frames)
            print ('yaw rotation error > '+ str(self.yaw_error_thre) + ' degree :')
            print (yaw_candidate_frames)
            print ('Write candidate problematic frames')
            f = open(self.saving_path+"_evaluation.txt", "a")
            f.write('Candidate problematic frame\n')
            f.write('horizontal translation error > '+ str(self.horizontal_error_thre) + ' m :\n')
            csv.writer(f, delimiter=' ').writerows(hor_candidate_frames)
            f.write('\nvertical translation error > '+ str(self.vertical_error_thre) + ' m :\n')
            csv.writer(f, delimiter=' ').writerows(ver_candidate_frames)
            f.write('\nyaw rotation error > '+ str(self.yaw_error_thre) + ' degree :\n')
            csv.writer(f, delimiter=' ').writerows(yaw_candidate_frames)
            f.write("\n------------------------------------------------------------------------\n")
            f.close()

    def eval(self):
        
        '''
        main evaluation function
        '''
        
        self.load_Timing()
        self.plot_timing_detail(False)

        self.load_Kitti_calib()
        self.load_GtPose()
        self.load_LoPose()
        self.load_Adj_tran()
        
        gt_frame_num=(self.GtPose).shape[0]
        lo_frame_num=(self.LoPose).shape[0]
        self.frame_num_used=min(gt_frame_num,lo_frame_num)

        self.GtPose=self.GtPose[0:self.frame_num_used,:]
        self.LoPose=self.LoPose[0:self.frame_num_used,:]

        err = self.calc_sequence_errors(self.GtPose, self.LoPose)
        t_err = 0
        r_err = 0
        for i in range(len(err)):
            r_err += err[i][1]
            t_err += err[i][2]
        r_err /= len(err)
        t_err /= len(err)
        # print(len(err))
        # print()
        # print("ATE (%)    :", t_err*100)
        # print("ARE (deg/m):", r_err/np.pi*180)
        # if self.ignore_z_error.lower() == 'true': 
        #     print("The error on Z axis is ignored")
        # print("Check the leaderboard at http://www.cvlibs.net/datasets/kitti/eval_odometry.php")
        
        f = open(EvalFile, "w")
        ate = str(t_err*100)
        are = str(r_err/np.pi*180)

        f.write('Lidar Odometry Evaluation Table\n')
        f.write("------------------------------------------------------------------------\n")
        f.write("ATE (%)    :"+ ate +"\n")
        f.write("ARE (deg/m):"+ are +"\n")
        f.write("check ranking at http://www.cvlibs.net/datasets/kitti/eval_odometry.php\n")
        f.write("------------------------------------------------------------------------\n")
        f.close()

        self.plot_trajectory()
        self.plot_error_change_6dof()
        
        #self.plot_change_6dof()
        

if __name__ == '__main__':
    
    PosesloFile = sys.argv[1] # lidar odometry (slam) pose
    PosesgtFile = sys.argv[2] # ground_truth_pose
    AdjacentFile = sys.argv[3]  # lo_estimated adjacent pose 
    CalibRtFile = sys.argv[4] # calib file
    TimingFile = sys.argv[5] # timing file
    EvalFile = sys.argv[6] # evaluation file
   
    Eval = KittiEvalOdom(PosesloFile, PosesgtFile, AdjacentFile, CalibRtFile, TimingFile, EvalFile)
    Eval.eval()


