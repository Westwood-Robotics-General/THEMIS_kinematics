#!usr/bin/env python
__author__    = "Westwood Robotics Corporation"
__email__     = "info@westwoodrobotics.io"
__copyright__ = "Copyright 2024 Westwood Robotics Corporation"
__date__      = "October 23, 2024"
__project__   = "BRUCE"
__version__   = "0.0.7"
__status__    = "Product"

'''
Script for communication with Gazebo
'''

import time
import Settings.BRUCE_data as RDS
import Startups.memory_manager as MM
import Library.ROBOT_MODEL.BRUCE_dynamics as dyn
import Library.ROBOT_MODEL.BRUCE_kinematics as kin
from Play.config import *
from termcolor import colored
from Simulation.config import *
from Startups.reset_memory import *
from Settings.BRUCE_macros import *
from Play.Walking.walking_macros import *
from Library.BRUCE_GYM.GAZEBO_INTERFACE import Manager as gazint


class GazeboSimulator:
    def __init__(self, robot):
        # robot info
        self.num_legs = 2
        self.num_joints_per_leg = 5
        self.num_arms = 2
        self.num_joints_per_arms = 3
        self.num_joints = self.num_legs * self.num_joints_per_leg + self.num_arms * self.num_joints_per_arms
        self.num_contact_sensors = 4
        
        self.leg_p_gains = [265, 150,  80,  80,    30]
        self.leg_i_gains = [  0,   0,   0,   0,     0]
        self.leg_d_gains = [ 1., 2.3, 0.8, 0.8, 0.003]

        self.arm_p_gains = [ 1.6,  1.6,  1.6]
        self.arm_i_gains = [   0,    0,    0]
        self.arm_d_gains = [0.03, 0.03, 0.03]

        self.p_gains = self.leg_p_gains * 2 + self.arm_p_gains * 2  # the joint order matches the robot's sdf file
        self.i_gains = self.leg_i_gains * 2 + self.arm_i_gains * 2
        self.d_gains = self.leg_d_gains * 2 + self.arm_d_gains * 2

        self.robot = robot
        
        # simulator info
        self.simulator = None
        self.simulation_frequency = 1000  # Hz
        self.simulation_modes = {'torque': 0, 'position': 2}
        self.simulation_mode = self.simulation_modes['position']
        
    def initialize_simulator(self):
        self.simulator = gazint.GazeboInterface(robot_name='bruce', num_joints=self.num_joints, num_contact_sensors=self.num_contact_sensors)
        self.simulator.set_step_size(1. / self.simulation_frequency)
        self.simulator.set_operating_mode(self.simulation_mode)
        self.simulator.set_all_position_pid_gains(self.p_gains, self.i_gains, self.d_gains)

        # arm pose
        ar1, ar2, ar3 = -0.7,  1.3,  2.0
        al1, al2, al3 =  0.7, -1.3, -2.0

        # leg pose
        bpr = np.array([0.03, -0.07, -0.40])      # right foot position  in body frame
        bpl = np.array([0.03, +0.07, -0.40])      # left  foot position  in body frame
        bxr = MF.Rz(0.0) @ np.array([1., 0., 0.]) # right foot direction in body frame
        bxl = MF.Rz(0.0) @ np.array([1., 0., 0.]) # left  foot direction in body frame

        qr = self.robot.solve_leg_IK(leg='right', x=bxr, p=bpr, q0=np.zeros(5))
        ql = self.robot.solve_leg_IK(leg='left',  x=bxl, p=bpl, q0=np.zeros(5))

        initial_pose = list(qr) + list(ql) + [ar1, ar2, ar3, al1, al2, al3]
        if FIXED:
            self.simulator.set_command_position(initial_pose)
        else:
            self.simulator.reset_simulation(initial_pose=initial_pose)
        
        print("Gazebo Initialization Completed!")
        
    def write_position(self, leg_positions, arm_positions):
        """
        Send goal positions to the simulator.
        """
        goal_position = [leg_positions[0], leg_positions[1], leg_positions[2], leg_positions[3], leg_positions[4],
                         leg_positions[5], leg_positions[6], leg_positions[7], leg_positions[8], leg_positions[9],
                         arm_positions[0], arm_positions[1], arm_positions[2],
                         arm_positions[3], arm_positions[4], arm_positions[5]]
        if self.simulation_mode != self.simulation_modes['position']:
            self.simulation_mode = self.simulation_modes['position']
            self.simulator.set_operating_mode(self.simulation_mode)
        self.simulator.set_command_position(goal_position)

    def write_torque(self, leg_torques, arm_torques):
        """
        Send goal torques to the simulator.
        """
        goal_torque = [leg_torques[0], leg_torques[1], leg_torques[2], leg_torques[3], leg_torques[4],
                       leg_torques[5], leg_torques[6], leg_torques[7], leg_torques[8], leg_torques[9],
                       arm_torques[0], arm_torques[1], arm_torques[2],
                       arm_torques[3], arm_torques[4], arm_torques[5]]
        if self.simulation_mode != self.simulation_modes['torque']:
            self.simulation_mode = self.simulation_modes['torque']
            self.simulator.set_operating_mode(self.simulation_mode)
        self.simulator.set_command_torque(goal_torque)

    def get_arm_goal_torques(self, arm_positions, arm_velocities):
        """
        Calculate arm goal torques.
        """
        arm_goal_torque = np.zeros(6)
        for i in range(6):
            arm_goal_torque[i] = self.arm_p_gains[i % self.num_joints_per_arms] * (arm_positions[i] - self.q_arm[i]) + self.arm_d_gains[i % self.num_joints_per_arms] * (arm_velocities[i] - self.dq_arm[i])
        return arm_goal_torque
        
    def update_sensor_info(self):
        """
        Get sensor info and write it to shared memory.
        """
        # get sim time
        MM.SIMULATOR_STATE.set({'time_stamp': np.array([self.simulator.get_current_time()])})

        # get joint states
        q = self.simulator.get_current_position()
        dq = self.simulator.get_current_velocity()
        tau = self.simulator.get_current_force()
        
        self.q_leg  =  q[0:10]
        self.dq_leg = dq[0:10]
        self.q_arm  =  q[10:16]
        self.dq_arm = dq[10:16]

        leg_data = {'joint_positions':  self.q_leg,
                    'joint_velocities': self.dq_leg,
                    'joint_torques':    tau[0:10]}
        arm_data = {'joint_positions':  self.q_arm,
                    'joint_velocities': self.dq_arm}
        MM.LEG_STATE.set(leg_data)
        MM.ARM_STATE.set(arm_data)
        
        # get imu states
        self.rot_mat = self.simulator.get_body_rot_mat()
        self.accel   = self.simulator.get_imu_acceleration()
        self.omega   = self.rot_mat.T @ self.simulator.get_imu_angular_rate()
        self.foot_contacts = self.simulator.get_foot_contacts()
        
        sense_data = {'imu_acceleration': self.accel,
                      'imu_ang_rate':     self.omega,
                      'foot_contacts':    self.foot_contacts,
                      'imu_rot_matrix':   self.rot_mat}
        MM.SENSE_STATE.set(sense_data)
        
    def calculate_robot_model(self):
        """
        Calculate kinematics & dynamics and write it to shared memory.
        """
        q_leg_r  = self.q_leg[0:5]
        q_leg_l  = self.q_leg[5:10]
        dq_leg_r = self.dq_leg[0:5]
        dq_leg_l = self.dq_leg[5:10]

        R_wb = self.rot_mat
        w_bb = self.omega
        p_wb = self.simulator.get_body_position()
        v_wb = self.simulator.get_body_velocity()
        a_wb = R_wb @ self.accel
        v_bb = R_wb.T @ v_wb
        yaw_angle = np.arctan2(R_wb[1, 0], R_wb[0, 0])

        # compute leg forward kinematics
        R_ba_r, w_ba_r, p_ba_r, v_ba_r, Jw_ba_r, dJwdq_ba_r, Jv_ba_r, dJvdq_ba_r = kin.leg_Jacobian(+1, 0, q_leg_r, dq_leg_r)
        R_ba_l, w_ba_l, p_ba_l, v_ba_l, Jw_ba_l, dJwdq_ba_l, Jv_ba_l, dJvdq_ba_l = kin.leg_Jacobian(-1, 0, q_leg_l, dq_leg_l)
        R_bf_r, w_bf_r, p_bf_r, v_bf_r, Jw_bf_r, dJwdq_bf_r, Jv_bf_r, dJvdq_bf_r = kin.leg_Jacobian(+1, 1, q_leg_r, dq_leg_r)
        R_bf_l, w_bf_l, p_bf_l, v_bf_l, Jw_bf_l, dJwdq_bf_l, Jv_bf_l, dJvdq_bf_l = kin.leg_Jacobian(-1, 1, q_leg_l, dq_leg_l)
        R_bt_r, w_bt_r, p_bt_r, v_bt_r, Jw_bt_r, dJwdq_bt_r, Jv_bt_r, dJvdq_bt_r = kin.leg_Jacobian(+1, 2, q_leg_r, dq_leg_r)
        R_bt_l, w_bt_l, p_bt_l, v_bt_l, Jw_bt_l, dJwdq_bt_l, Jv_bt_l, dJvdq_bt_l = kin.leg_Jacobian(-1, 2, q_leg_l, dq_leg_l)
        R_bh_r, w_bh_r, p_bh_r, v_bh_r, Jw_bh_r, dJwdq_bh_r, Jv_bh_r, dJvdq_bh_r = kin.leg_Jacobian(+1, 3, q_leg_r, dq_leg_r)
        R_bh_l, w_bh_l, p_bh_l, v_bh_l, Jw_bh_l, dJwdq_bh_l, Jv_bh_l, dJvdq_bh_l = kin.leg_Jacobian(-1, 3, q_leg_l, dq_leg_l)

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
                                                           q_leg_r,  q_leg_l,
                                                          dq_leg_r, dq_leg_l)
        
        # save as estimation data
        estimation_data = {}
        estimation_data['time_stamp']        = np.array([self.simulator.get_current_time()])
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
        estimation_data['foot_contacts']     = self.foot_contacts

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


def main_loop():
    # parameters
    check_thread_frequency = 10
    check_thread_duration  = 1. / check_thread_frequency

    print("====== BRUCE Simulation Thread is running... ======")
    
    flag = False
    thread_run = False
    last_check_thread_time = time.time()
    t0 = time.time()
    while True:
        loop_start_time = time.time()

         # check threading issue
        last_check_thread_elapse = loop_start_time - last_check_thread_time
        if last_check_thread_elapse > check_thread_duration:
            last_check_thread_time = loop_start_time
            if Bruce.thread_error():
                Bruce.stop_threading()

        if not thread_run and loop_start_time - t0 > 1:
            print(colored('THREAD IS DOING GREAT!', 'green'))
            MM.THREAD_STATE.set({'simulation': np.array([1])}, opt='update')  # thread is running
            thread_run = True

        gs.update_sensor_info()
        if not ESTIMATION:
            gs.calculate_robot_model()

        leg_command = MM.LEG_COMMAND.get()
        arm_command = MM.ARM_COMMAND.get()

        if leg_command['BEAR_enable'][0] == 1:
            flag = True
            if leg_command['BEAR_mode'][0] == 2:  # position control
                gs.write_position(leg_command['goal_positions'], arm_command['goal_positions'])
            elif leg_command['BEAR_mode'][0] == 0 or 3:  # torque control
                gs.write_torque(leg_command['goal_torques'], gs.get_arm_goal_torques(arm_command['goal_positions'], arm_command['goal_velocities']))
        
        if flag and leg_command['BEAR_enable'][0] == 0:
            gs.write_torque([0] * 10, [0] * 6)

        gs.simulator.step_simulation()
        time.sleep(0.000)
        if ESTIMATION:
            time.sleep(0.001)  # delay if needed


if __name__ == "__main__":
    # BRUCE SETUP
    Bruce = RDS.BRUCE()

    # Gazebo
    gs = GazeboSimulator(robot=Bruce)
    gs.initialize_simulator()

    try:
        main_loop()
    except NameError:
        print(colored('THREAD STOPPED PEACEFULLY!', 'light_grey'))
    except (Exception, KeyboardInterrupt) as error:
        print(error)
        print(colored('THREAD IN ERROR! TERMINATE NOW!', 'red'))
        MM.THREAD_STATE.set({'simulation': np.array([2])}, opt='update')  # thread in error
    finally:
        MM.SIMULATOR_STATE.set({'time_stamp': np.array([gs.simulator.get_current_time() + 10.])})