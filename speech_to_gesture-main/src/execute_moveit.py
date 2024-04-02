#!/usr/bin/python

import pickle
from scipy.spatial.transform import Rotation as R
from scipy.signal import savgol_filter
import numpy as np
from time import sleep
import time
import json

import rospy
from control_msgs.msg import FollowJointTrajectoryActionGoal
from geometry_msgs.msg import Pose
from trajectory_msgs.msg import JointTrajectoryPoint

import sys
import copy
import moveit_commander
from moveit_commander.conversions import pose_to_list
import tf

import matplotlib.pyplot as plt
import multiprocessing as mp

import concurrent.futures


class ExecuteMoveItNode():
    def __init__(self):
        rospy.init_node('execute_moveit_node')

        # get shoulder positions
        # tf_listener = tf.TransformListener()
        # sleep(1)
        # self.left_shoulder_trans, _ = tf_listener.lookupTransform('base_footprint', 'arm_left_1_link', rospy.Time(0))
        # self.right_shoulder_trans, _ = tf_listener.lookupTransform('base_footprint', 'arm_right_1_link', rospy.Time(0))
        # print("left shoulder: ", self.left_shoulder_trans)
        # print("right shoulder: ", self.right_shoulder_trans)

        # to avoid delay, use hardcoded values
        self.left_shoulder_trans =  [-0.03644, 0.19, 1.0674999008122568]  # these values are from the torso lifted
        self.right_shoulder_trans =  [-0.03644, -0.19, 1.0674999008122568] # these values are from the torso lifted

        # read bvh file from disk
        self.bvh = read_bvh()

        # compute cartesian path
        left_trans_mat_list = self.compute_cart_path(which_arm='left')
        right_trans_mat_list = self.compute_cart_path(which_arm='right')

        # smooth cartesian path by applying low pass filter
        with open("/home/moritz/robotrust/src/speech_to_gesture/config/config.json", 'r') as f:
            config = json.load(f)
            filter_parameters = [tuple(x) for x in config["filter_parameters"]]

        print("filter parameters: ", filter_parameters)
        
        left_T_trans_dict = apply_filter(filter_parameters=filter_parameters, trans_mat_list=left_trans_mat_list, dt=self.bvh['Frame_Time'])
        right_T_trans_dict = apply_filter(filter_parameters=filter_parameters, trans_mat_list=right_trans_mat_list, dt=self.bvh['Frame_Time'])

        
        # choose best smoothing parameters
        # the best smoothing parameters are the ones that result in a trajectory that is closest to the desired time
        desired_time = self.bvh['Frames'] * self.bvh['Frame_Time']
        print("desired time: ", desired_time)

        best_left_traj = self.compute_best_traj(desired_time=desired_time, T_trans_dict = left_T_trans_dict, which_arm='left')
        best_right_traj = self.compute_best_traj(desired_time=desired_time, T_trans_dict = right_T_trans_dict, which_arm='right')
       
        with open(speech_to_gesture_path + "/config/arm_left_trajectory.pkl", 'wb') as f:
            pickle.dump(best_left_traj, f, pickle.HIGHEST_PROTOCOL)

        with open(speech_to_gesture_path + "/config/arm_right_trajectory.pkl", 'wb') as f:
            pickle.dump(best_right_traj, f, pickle.HIGHEST_PROTOCOL)


    def compute_cart_path(self, which_arm):
        trans_mat_list = []
        for i in range(0, self.bvh["Frames"], 1):  # use every frame
            trans_mat = self.compute_trans_endeffector(timestep=i, which_arm=which_arm)

            trans_mat_list.append(trans_mat)

        return trans_mat_list


    def execute_cartesian_path(self, trans_mat_list, which_arm):
        if which_arm == 'left': 
            move_group =  moveit_commander.MoveGroupCommander("arm_left")
        elif which_arm == 'right':
            move_group = moveit_commander.MoveGroupCommander("arm_right")
        else:
            print('wrong arm name')
            return
        
        plan, fraction = move_group.compute_cartesian_path([get_goal_pose(m) for m in trans_mat_list], 0.1, 0.0)

        # transform plan to joint trajectory action goal
        follow_joint_traj_action_goal = FollowJointTrajectoryActionGoal()

        # initialize header with joint names
        for i in range(1, 8):
            if which_arm == 'left':
                follow_joint_traj_action_goal.goal.trajectory.joint_names.append("arm_left_{0}_joint".format(str(i)))
            elif which_arm == 'right':
                follow_joint_traj_action_goal.goal.trajectory.joint_names.append("arm_right_{0}_joint".format(str(i)))

        for i in range(0, len(plan.joint_trajectory.points), 1):
            joint_traj_point = JointTrajectoryPoint()
            joint_traj_point.positions = plan.joint_trajectory.points[i].positions
            joint_traj_point.time_from_start = plan.joint_trajectory.points[i].time_from_start
            follow_joint_traj_action_goal.goal.trajectory.points.append(joint_traj_point)


        return follow_joint_traj_action_goal
    

    def compute_trans_endeffector(self, timestep, which_arm):
        if which_arm == 'left':
            idx = [52, 53, 54, 55, 56, 57]
            shoulder_cords = self.left_shoulder_trans
        elif which_arm == 'right':
            idx = [28, 29, 30, 31, 32, 33]
            shoulder_cords = self.right_shoulder_trans
        else:
            return None
        
        # initialize transformation matrix with a rotation of 90 degrees around the x-axis and 90 degrees around the z-axis
        if which_arm == 'left':
            trans =get_trans_mat(shoulder_cords[0] * 100, shoulder_cords[1]*100, shoulder_cords[2]*100, 90, 0, 90)
        elif which_arm == 'right':
            trans =get_trans_mat(shoulder_cords[0] * 100, shoulder_cords[1]*100, shoulder_cords[2]*100, 90, 0, -90)
        else:
            return None

        if which_arm == "right":
            id = 42
        else:
            id = 43

        for i in idx:
            trans_x = float(self.bvh['waypoints'][timestep].split()[i*6 + 0])*1.2
            trans_y = float(self.bvh['waypoints'][timestep].split()[i*6 + 1])*1.2
            trans_z = float(self.bvh['waypoints'][timestep].split()[i*6 + 2])*1.2

            rot_z = float(self.bvh['waypoints'][timestep].split()[i*6 + 3])
            rot_x = float(self.bvh['waypoints'][timestep].split()[i*6 + 4])
            rot_y = float(self.bvh['waypoints'][timestep].split()[i*6 + 5])

            if which_arm == "right":
                rot_x = -rot_x
                rot_z = -rot_z
                trans_x = -trans_x
                trans_z = -trans_z

            trans = np.matmul(trans, get_trans_mat(trans_x, trans_y, trans_z, rot_x, rot_y, rot_z))

        return trans
    
    def compute_best_traj(self, desired_time, T_trans_dict, which_arm):
        now = time.time()

        results = {}

        def process_trajectory(T, trans_mat_list, which_arm):
            traj = self.execute_cartesian_path(trans_mat_list, which_arm)
            exec_time = traj.goal.trajectory.points[-1].time_from_start.to_sec()
            time_delta = abs(exec_time - desired_time)
            return T, (time_delta, traj)

        with concurrent.futures.ThreadPoolExecutor(max_workers=16) as executor:
            futures = []
            for T, trans_mat_list in T_trans_dict.items():
                future = executor.submit(process_trajectory, T, trans_mat_list, which_arm)
                futures.append(future)

            for future in concurrent.futures.as_completed(futures):
                T, result = future.result()
                results[T] = result


        # choose best trajectory
        best_traj = None
        best_time_delta = np.inf
        for T, (time_delta, traj) in results.items():
            print("T: {0}, time_delta: {1}".format(T, time_delta))
            if time_delta < best_time_delta:
                best_traj = traj
                best_time_delta = time_delta
        return best_traj
    

def read_bvh():
    with open("path to /config.json", 'r') as f:
        config = json.load(f)
        venv_path = config["cospeech_gesture_generation_venv_path"]
        inference_path = config["cospeech_gesture_generation_inference_path"]
        network_path = config["cospeech_gesture_generation_network_path"]
        output_path = config["cospeech_gesture_generation_output_path"]


    bvh = dict()
    with open(output_path, 'r') as f:
        tmp = f.readlines()
        bvh['header'] = None
        bvh['Frames'] = None
        bvh['Frame_Time'] = None
        bvh['waypoints'] = []

        MOTION_idx = None
        for idx, line in enumerate(tmp):
            if 'MOTION' in line:
                MOTION_idx = idx
                break

        bvh['Frames'] = int(tmp[MOTION_idx + 1].split(':')[1])
        print("Frames = ", bvh['Frames'])

        bvh['Frame_Time'] = float(tmp[MOTION_idx + 2].split(':')[1])
        print("Frame Time = ", bvh['Frame_Time'])

        for i in range(MOTION_idx + 3, MOTION_idx + 3 + bvh['Frames']):
            bvh['waypoints'].append(tmp[i])

    return bvh


def get_goal_pose(trans):
    x = trans[0, 3] / 100
    y = trans[1, 3] / 100
    z = trans[2, 3] / 100

    rot = trans[0:3, 0:3]
    rot = R.from_dcm(rot)
    q_x, q_y, q_z, q_w = rot.as_quat()

    goal_pose = Pose()
    goal_pose.position.x = x
    goal_pose.position.y = y
    goal_pose.position.z = z

    goal_pose.orientation.x = q_x
    goal_pose.orientation.y = q_y
    goal_pose.orientation.z = q_z
    goal_pose.orientation.w = q_w

    return goal_pose


def get_trans_mat(trans_x=0, trans_y=0, tran_z=0, rot_x=0, rot_y=0, rot_z=0):
    r_z = R.from_euler('z', rot_z, degrees=True).as_dcm()
    r_x = R.from_euler('x', rot_x, degrees=True).as_dcm()
    r_y = R.from_euler('y', rot_y, degrees=True).as_dcm()
    r_zxy = np.matmul(r_z, np.matmul(r_x, r_y))

    trans = np.eye(4)
    trans[0:3, 0:3] = r_zxy
    trans[0, 3] = trans_x
    trans[1, 3] = trans_y
    trans[2, 3] = tran_z

    return trans

def apply_filter(filter_parameters, trans_mat_list, dt):
    T_trans_dict = {}
    for filter_parameter in filter_parameters:
        curr_trans = np.array(copy.deepcopy(trans_mat_list))
        signals = []

        signals.append(np.array(curr_trans[:,0,3]))  # x position
        signals.append(np.array(curr_trans[:,1,3]))  # y position
        signals.append(np.array(curr_trans[:,2,3]))  # z position

        # also filter rotation
        rot = curr_trans[:, 0:3, 0:3]

        rot_x = []
        rot_y = []
        rot_z = []
        
        rotations = [R.from_dcm(r) for r in rot]
        for r in rotations:
            x,y,z = (r.as_euler('xyz', degrees=True))
            rot_x.append(x)
            rot_y.append(y)
            rot_z.append(z)

        signals.append(np.array(rot_x))
        signals.append(np.array(rot_y))
        signals.append(np.array(rot_z))

        filtered_signals = []
        for i, signal in enumerate(signals):
            # lowpass
            # filtered_signals.append(low_pass_filter(signal, T, dt))

            # savgol
            filtered_signals.append(our_savgol_filter(signal, filter_parameter, dt))
        filtered_signals = np.array(filtered_signals)

        # show_plot(signals, filtered_signals, title = filter_parameter)

        rotations = []
        for curr_x, curr_y, curr_z in zip(filtered_signals[3], filtered_signals[4], filtered_signals[5]):
            new_rot = R.from_euler("xyz", [curr_x, curr_y, curr_z] ,degrees=True).as_dcm()
            rotations.append(new_rot)
        
        curr_trans[:, 0:3, 0:3] = rotations

        curr_trans[:,0,3] = filtered_signals[0].tolist()
        curr_trans[:,1,3] = filtered_signals[1].tolist()
        curr_trans[:,2,3] = filtered_signals[2].tolist()

        T_trans_dict[filter_parameter] = curr_trans

    return T_trans_dict


def low_pass_filter(x, T, dt):
    T = T / dt
    xFiltered = x.copy()

    for i in range(1, len(xFiltered)):
        xFiltered[i] = (1 / (T + 1)) * (xFiltered[i] + T * xFiltered[i - 1])

    return xFiltered

def our_savgol_filter(x, filter_parameter, dt):
    window_length = 2*int(filter_parameter[0]/2) + 1  # must be odd
    polyorder = filter_parameter[1]
    return savgol_filter(x, window_length=window_length, polyorder=polyorder)

def show_plot(signals, filtered_signals, title):
    n_plots = len(signals)
    fig = plt.figure()
    plt.suptitle("T = " + str(title))
    for i in range(n_plots):
        fig.add_subplot(n_plots, 1, i+1)
        plt.plot(signals[i], label="signal")
        plt.plot(filtered_signals[i], label="filtered signal")
        plt.legend()

    plt.show()

def main():
    global speech_to_gesture_path
    speech_to_gesture_path = "path to /speech_to_gesture"

    ExecuteMoveItNode()


if __name__ == '__main__':
    main()
