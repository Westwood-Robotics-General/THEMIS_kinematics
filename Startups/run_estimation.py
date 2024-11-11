#!usr/bin/env python
__author__    = "Westwood Robotics Corporation"
__email__     = "info@westwoodrobotics.io"
__copyright__ = "Copyright 2024 Westwood Robotics Corporation"
__date__      = "October 23, 2024"
__project__   = "BRUCE"
__version__   = "0.0.7"
__status__    = "Product"

'''
Script usage:
1. estimate robot states, e.g., body orientation, angular velocity, position, velocity, foot position, velocity, etc.
2. calculate robot model, e.g., kinematics (Jacobian and its derivative) and dynamics (equations of motion)
'''

import Settings.BRUCE_data as RDS
import Startups.memory_manager as MM
import Library.ROBOT_MODEL.BRUCE_dynamics as dyn
import Library.ROBOT_MODEL.BRUCE_kinematics as kin
import Library.STATE_ESTIMATION.BRUCE_estimation as est
from Play.config import *
from termcolor import colored
from Settings.BRUCE_macros import *
from Play.Walking.walking_macros import *


def check_sense_thread():
    error = True
    while error:
        thread_data = MM.THREAD_STATE.get()
        if thread_data['sense'][0] == 1 or thread_data['simulation'][0] == 1:
            error = False

def main_loop():
    # BRUCE Setup
    Bruce = RDS.BRUCE()

    # Check if sense is running
    check_sense_thread()

    # Parameters
    loop_freq     = 500  # run at 500 Hz
    loop_duration = 1. / loop_freq
    gravity_accel = 9.81 if HARDWARE else -9.8

    check_thread_frequency = 10
    check_thread_duration  = 1. / check_thread_frequency

    # Foot Contacts
    foot_contacts       = np.zeros(4)  # 0/1 indicate in air/contact (for right/left toe/heel)
    foot_contacts_count = np.zeros(4)  # indicate how long the foot is in contact

    # Initial Guess
    p_wb  = np.array([0.00, 0.00, 0.40])   # body position         - in world frame
    v_wb  = np.array([0.00, 0.00, 0.00])   # body velocity         - in world frame
    a_wb  = np.array([0.00, 0.00, 0.00])   # body acceleration     - in world frame
    R_wb  = np.eye(3)                      # body orientation      - in world frame
    v_bb  = R_wb.T @ v_wb                  # body velocity         - in  body frame
    w_bb  = np.array([0.00, 0.00, 0.00])   # body angular velocity - in  body frame
    b_acc = np.array([0.00, 0.00, 0.00])   # accelerometer bias    - in   IMU frame

    p_wt_r = np.array([+at, -0.06, 0.00])  # right toe   position  - in world frame
    p_wh_r = np.array([-ah, -0.06, 0.00])  # right heel  position  - in world frame
    p_wa_r = np.array([0.0, -0.06, 0.02])  # right ankle position  - in world frame
    p_wf_r = np.array([0.0, -0.06, 0.00])  # right foot  position  - in world frame

    c_wt_r = np.array([+at, -0.06, 0.00])  # right toe   position if in contact
    c_wh_r = np.array([-ah, -0.06, 0.00])  # right heel  position if in contact
    c_wa_r = np.array([0.0, -0.06, 0.02])  # right ankle position if in contact
    c_wf_r = np.array([0.0, -0.06, 0.00])  # right foot  position if in contact

    p_wt_l = np.array([+at,  0.06, 0.00])  # left  toe   position  - in world frame
    p_wh_l = np.array([-ah,  0.06, 0.00])  # left  heel  position  - in world frame
    p_wa_l = np.array([0.0,  0.06, 0.02])  # left  ankle position  - in world frame
    p_wf_l = np.array([0.0,  0.06, 0.00])  # left  foot  position  - in world frame

    c_wt_l = np.array([+at,  0.06, 0.00])  # left  toe   position if in contact
    c_wh_l = np.array([-ah,  0.06, 0.00])  # left  heel  position if in contact
    c_wa_l = np.array([0.0,  0.06, 0.02])  # left  ankle position if in contact
    c_wf_l = np.array([0.0,  0.06, 0.00])  # left  foot  position if in contact

    R_wi = np.eye(3)                       # imu orientation in world frame
    w_ii = np.zeros(3)                     # last gyroscope reading

    Po = np.eye(15) * 1e-2                 # Kalman filter state covariance matrix

    # Shared Memory Data
    estimation_data = {'body_rot_matrix': np.zeros((3, 3))}

    # Start Estimation
    print("====== The State Estimation Thread is running at", loop_freq, "Hz... ======")

    thread_run = False
    last_check_thread_time = Bruce.get_time()
    t0 = Bruce.get_time()
    while True:
        loop_start_time = Bruce.get_time()
        elapsed_time    = loop_start_time - t0

        # check threading issue
        last_check_thread_elapse = loop_start_time - last_check_thread_time
        if last_check_thread_elapse > check_thread_duration:
            last_check_thread_time = loop_start_time
            if Bruce.thread_error():
                Bruce.stop_threading()

        if not thread_run and elapsed_time > 1:
            print(colored('THREAD IS DOING GREAT!', 'green'))
            MM.THREAD_STATE.set({'estimation': np.array([1])}, opt='update')  # thread is running
            thread_run = True

        # update plan status
        Bruce.update_plan_status()

        # update sense status
        Bruce.update_sense_status()

        # get leg status
        leg_data = MM.LEG_STATE.get()
        q_leg    = leg_data['joint_positions']
        dq_leg   = leg_data['joint_velocities']
        q_leg_r  = q_leg[0:5]
        q_leg_l  = q_leg[5:10]
        dq_leg_r = dq_leg[0:5]
        dq_leg_l = dq_leg[5:10]

        # compute leg forward kinematics
        R_ba_r, w_ba_r, p_ba_r, v_ba_r, Jw_ba_r, dJwdq_ba_r, Jv_ba_r, dJvdq_ba_r = kin.leg_Jacobian(+1, 0, q_leg_r, dq_leg_r)
        R_ba_l, w_ba_l, p_ba_l, v_ba_l, Jw_ba_l, dJwdq_ba_l, Jv_ba_l, dJvdq_ba_l = kin.leg_Jacobian(-1, 0, q_leg_l, dq_leg_l)
        R_bf_r, w_bf_r, p_bf_r, v_bf_r, Jw_bf_r, dJwdq_bf_r, Jv_bf_r, dJvdq_bf_r = kin.leg_Jacobian(+1, 1, q_leg_r, dq_leg_r)
        R_bf_l, w_bf_l, p_bf_l, v_bf_l, Jw_bf_l, dJwdq_bf_l, Jv_bf_l, dJvdq_bf_l = kin.leg_Jacobian(-1, 1, q_leg_l, dq_leg_l)
        R_bt_r, w_bt_r, p_bt_r, v_bt_r, Jw_bt_r, dJwdq_bt_r, Jv_bt_r, dJvdq_bt_r = kin.leg_Jacobian(+1, 2, q_leg_r, dq_leg_r)
        R_bt_l, w_bt_l, p_bt_l, v_bt_l, Jw_bt_l, dJwdq_bt_l, Jv_bt_l, dJvdq_bt_l = kin.leg_Jacobian(-1, 2, q_leg_l, dq_leg_l)
        R_bh_r, w_bh_r, p_bh_r, v_bh_r, Jw_bh_r, dJwdq_bh_r, Jv_bh_r, dJvdq_bh_r = kin.leg_Jacobian(+1, 3, q_leg_r, dq_leg_r)
        R_bh_l, w_bh_l, p_bh_l, v_bh_l, Jw_bh_l, dJwdq_bh_l, Jv_bh_l, dJvdq_bh_l = kin.leg_Jacobian(-1, 3, q_leg_l, dq_leg_l)

        # state estimation
        if Bruce.phase == 0 or Bruce.phase == 4:
            foot_contacts[0:4] = np.ones(4)
        elif Bruce.phase == 1:
            foot_contacts[0:2] = np.ones(2)
            foot_contacts[2:4] = np.zeros(2)
        elif Bruce.phase == 2:
            foot_contacts[0:2] = np.zeros(2)
            foot_contacts[2:4] = np.ones(2)
        else:
            foot_contacts[0:4] = np.zeros(4)
        for idx in range(4):
            foot_contacts_count[idx] = foot_contacts_count[idx] + 1 if foot_contacts[idx] else 0

        if HARDWARE:
            if Bruce.mode == 0:
                kR = 0.002
            elif Bruce.mode == 1:
                kR = 0.001
            elif Bruce.mode == 2:
                kR = 0.000
            else:
                if Bruce.phase == 0:
                    kR = 0.001
                else:
                    kR = 0.000
            
            R_wi, w_ii = est.update_orientation(R_wi, w_ii,
                                                Bruce.imu_omega, Bruce.imu_accel, gravity_accel, kR)
        else:
            R_wi = Bruce.imu_rot_matrix
            w_ii = Bruce.imu_omega

        R_wb, w_bb, p_wb, v_wb, a_wb, b_acc, c_wa_r, c_wa_l, Po, \
        v_bb, yaw_angle = est.update_translation(R_wb, w_bb, p_wb, v_wb, a_wb, b_acc, c_wa_r, c_wa_l, Po,
                                                 p_ba_r, p_ba_l, v_ba_r, v_ba_l, p_wa_r, p_wa_l,
                                                 R_wi, w_ii, Bruce.imu_accel, foot_contacts_count[::2], gravity_accel)
            
        # compute robot forward kinematics
        R_wa_r, w_aa_r, p_wa_r, v_wa_r, Jw_aa_r, dJwdq_aa_r, Jv_wa_r, dJvdq_wa_r = kin.base2task(+1, R_wb, w_bb, p_wb, v_bb, R_ba_r, w_ba_r, p_ba_r, v_ba_r, Jw_ba_r, dJwdq_ba_r, Jv_ba_r, dJvdq_ba_r)
        R_wa_l, w_aa_l, p_wa_l, v_wa_l, Jw_aa_l, dJwdq_aa_l, Jv_wa_l, dJvdq_wa_l = kin.base2task(-1, R_wb, w_bb, p_wb, v_bb, R_ba_l, w_ba_l, p_ba_l, v_ba_l, Jw_ba_l, dJwdq_ba_l, Jv_ba_l, dJvdq_ba_l)
        R_wf_r, w_ff_r, p_wf_r, v_wf_r, Jw_ff_r, dJwdq_ff_r, Jv_wf_r, dJvdq_wf_r = kin.base2task(+1, R_wb, w_bb, p_wb, v_bb, R_bf_r, w_bf_r, p_bf_r, v_bf_r, Jw_bf_r, dJwdq_bf_r, Jv_bf_r, dJvdq_bf_r)
        R_wf_l, w_ff_l, p_wf_l, v_wf_l, Jw_ff_l, dJwdq_ff_l, Jv_wf_l, dJvdq_wf_l = kin.base2task(-1, R_wb, w_bb, p_wb, v_bb, R_bf_l, w_bf_l, p_bf_l, v_bf_l, Jw_bf_l, dJwdq_bf_l, Jv_bf_l, dJvdq_bf_l)
        R_wt_r, w_tt_r, p_wt_r, v_wt_r, Jw_tt_r, dJwdq_tt_r, Jv_wt_r, dJvdq_wt_r = kin.base2task(+1, R_wb, w_bb, p_wb, v_bb, R_bt_r, w_bt_r, p_bt_r, v_bt_r, Jw_bt_r, dJwdq_bt_r, Jv_bt_r, dJvdq_bt_r)
        R_wt_l, w_tt_l, p_wt_l, v_wt_l, Jw_tt_l, dJwdq_tt_l, Jv_wt_l, dJvdq_wt_l = kin.base2task(-1, R_wb, w_bb, p_wb, v_bb, R_bt_l, w_bt_l, p_bt_l, v_bt_l, Jw_bt_l, dJwdq_bt_l, Jv_bt_l, dJvdq_bt_l)
        R_wh_r, w_hh_r, p_wh_r, v_wh_r, Jw_hh_r, dJwdq_hh_r, Jv_wh_r, dJvdq_wh_r = kin.base2task(+1, R_wb, w_bb, p_wb, v_bb, R_bh_r, w_bh_r, p_bh_r, v_bh_r, Jw_bh_r, dJwdq_bh_r, Jv_bh_r, dJvdq_bh_r)
        R_wh_l, w_hh_l, p_wh_l, v_wh_l, Jw_hh_l, dJwdq_hh_l, Jv_wh_l, dJvdq_wh_l = kin.base2task(-1, R_wb, w_bb, p_wb, v_bb, R_bh_l, w_bh_l, p_bh_l, v_bh_l, Jw_bh_l, dJwdq_bh_l, Jv_bh_l, dJvdq_bh_l)

        # calculate robot dynamics
        H, CG, AG, dAGdq, p_wg, v_wg, k_wg = dyn.robot_ID(R_wb, p_wb, w_bb, v_bb,
                                                          q_leg_r, q_leg_l,
                                                          dq_leg_r, dq_leg_l)

        # save data
        estimation_data['time_stamp']        = np.array([elapsed_time])
        estimation_data['body_position']     = p_wb
        estimation_data['body_velocity']     = v_wb
        estimation_data['body_acceleration'] = a_wb
        estimation_data['body_rot_matrix']   = R_wb
        estimation_data['body_ang_rate']     = w_bb
        estimation_data['body_yaw_ang']      = np.array([yaw_angle])
        estimation_data['com_position']      = p_wg
        estimation_data['com_velocity']      = v_wg
        estimation_data['ang_momentum']      = k_wg
        estimation_data['H_matrix']          = H
        estimation_data['CG_vector']         = CG
        estimation_data['AG_matrix']         = AG
        estimation_data['dAGdq_vector']      = dAGdq
        estimation_data['foot_contacts']     = Bruce.foot_contacts

        estimation_data['right_foot_rot_matrix'] = R_wf_r
        estimation_data['right_foot_ang_rate']   = w_ff_r
        estimation_data['right_foot_Jw']         = Jw_ff_r
        estimation_data['right_foot_dJwdq']      = dJwdq_ff_r
        estimation_data['right_foot_position']   = p_wf_r
        estimation_data['right_foot_velocity']   = v_wf_r
        estimation_data['right_toe_position']    = p_wt_r
        estimation_data['right_toe_velocity']    = v_wt_r
        estimation_data['right_toe_Jv']          = Jv_wt_r
        estimation_data['right_toe_dJvdq']       = dJvdq_wt_r
        estimation_data['right_heel_position']   = p_wh_r
        estimation_data['right_heel_velocity']   = v_wh_r
        estimation_data['right_heel_Jv']         = Jv_wh_r
        estimation_data['right_heel_dJvdq']      = dJvdq_wh_r
        estimation_data['right_ankle_position']  = p_wa_r
        estimation_data['right_ankle_velocity']  = v_wa_r
        estimation_data['right_ankle_Jv']        = Jv_wa_r
        estimation_data['right_ankle_dJvdq']     = dJvdq_wa_r

        estimation_data['left_foot_rot_matrix']  = R_wf_l
        estimation_data['left_foot_ang_rate']    = w_ff_l
        estimation_data['left_foot_Jw']          = Jw_ff_l
        estimation_data['left_foot_dJwdq']       = dJwdq_ff_l
        estimation_data['left_foot_position']    = p_wf_l
        estimation_data['left_foot_velocity']    = v_wf_l
        estimation_data['left_toe_position']     = p_wt_l
        estimation_data['left_toe_velocity']     = v_wt_l
        estimation_data['left_toe_Jv']           = Jv_wt_l
        estimation_data['left_toe_dJvdq']        = dJvdq_wt_l
        estimation_data['left_heel_position']    = p_wh_l
        estimation_data['left_heel_velocity']    = v_wh_l
        estimation_data['left_heel_Jv']          = Jv_wh_l
        estimation_data['left_heel_dJvdq']       = dJvdq_wh_l
        estimation_data['left_ankle_position']   = p_wa_l
        estimation_data['left_ankle_velocity']   = v_wa_l
        estimation_data['left_ankle_Jv']         = Jv_wa_l
        estimation_data['left_ankle_dJvdq']      = dJvdq_wa_l

        MM.ESTIMATOR_STATE.set(estimation_data)

        # check time to ensure that the state estimator stays at a consistent running loop.
        loop_end_time = loop_start_time + loop_duration
        present_time  = Bruce.get_time()
        if present_time > loop_end_time:
            delay_time = 1000 * (present_time - loop_end_time)
            if delay_time > 1.:
                print(colored('Delayed ' + str(delay_time)[0:5] + ' ms at Te = ' + str(elapsed_time)[0:5] + ' s', 'yellow'))
        else:
            while Bruce.get_time() < loop_end_time:
                pass


if __name__ == '__main__':
    if ESTIMATION:
        try:
            main_loop()
        except NameError:
            print(colored('THREAD STOPPED PEACEFULLY!', 'light_grey'))
            MM.THREAD_STATE.set({'estimation': np.array([0])}, opt='update')  # thread is stopped
        except (Exception, KeyboardInterrupt) as error:
            print(error)
            print(colored('THREAD IN ERROR! TERMINATE NOW!', 'red'))
            MM.THREAD_STATE.set({'estimation': np.array([2])}, opt='update')  # thread in error