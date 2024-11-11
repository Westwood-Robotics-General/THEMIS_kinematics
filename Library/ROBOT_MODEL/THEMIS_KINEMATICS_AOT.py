#!usr/bin/env python
__author__    = "Westwood Robotics Corporation"
__email__     = "info@westwoodrobotics.io"
__copyright__ = "Copyright 2024 Westwood Robotics Corporation"
__date__      = "October 23, 2024"
__project__   = "BRUCE"
__version__   = "0.0.7"
__status__    = "Product"

'''
Compile BRUCE kinematics ahead of time (AOT) using Numba
'''

from numba.pycc import CC
from Settings.BRUCE_macros import *


lx = COORDINATE_VECTOR[HIP_YAW_R][0]
ly = COORDINATE_VECTOR[HIP_YAW_R][1]
lz = COORDINATE_VECTOR[HIP_YAW_R][2]

d1 = COORDINATE_VECTOR[KNEE_PITCH_R][0]
d2_leg = COORDINATE_VECTOR[ANKLE_PITCH_R][0]
d3 = COORDINATE_VECTOR[KNEE_PITCH_R][0]
d4 = COORDINATE_VECTOR[ANKLE_PITCH_R][0]

p_b1_r = np.array([lx, +ly, lz])
p_b1_l = np.array([lx, -ly, lz])

T_5a = np.eye(4)
T_5a[0:3, 0:3] = MF.Ry(-PI_2) @ MF.Rz(PI_2)
T_5a[0:3,   3] = np.array([0, 0, 0])

T_5s = np.eye(4)
T_5s[0:3, 0:3] = MF.Ry(-PI_2) @ MF.Rz(PI_2)
T_5s[0:3,   3] = np.array([a5, 0, 0])

T_5t = np.eye(4)
T_5t[0:3, 0:3] = MF.Ry(-PI_2) @ MF.Rz(PI_2)
T_5t[0:3,   3] = np.array([a5, at, 0])

T_5h = np.eye(4)
T_5h[0:3, 0:3] = MF.Ry(-PI_2) @ MF.Rz(PI_2)
T_5h[0:3,   3] = np.array([a5, -ah, 0])


cc = CC('THEMIS_kinematics')
  # 使用数值常量 π
pi_val = np.pi

@cc.export('leg_FK', '(f8, f8, f8[:])')
def leg_FK(leg, option, q):
    # leg    - right (+1)
    #        -  left (-1)
    # option - ankle (0)
    #        - sole  (1)
    #        - toe   (2)
    #        - heel  (3)
    if option == 0:
        T_5f = T_5a
    elif option == 1:
        T_5f = T_5s
    elif option == 2:
        T_5f = T_5t
    elif option == 3:
        T_5f = T_5h

    T_b1 = np.eye(4)
    T_b1[0:3, 0:3] = MF.Ry(+PI_4) @ MF.Rz(q[0])
    T_b1[0:3,   3] = p_b1_r if leg == +1 else p_b1_l

    T_12 = np.eye(4)
    T_12[0:3, 0:3] = MF.Rx(-PI_2) @ MF.Rz(q[0])
    T_12[  0,   3] = d1

    T_23 = np.eye(4)
    T_23[0:3, 0:3] = MF.Rx(-PI_4) @ MF.Ry(PI_2)@ MF.Rz(q[0])
    T_23[  0,   3] = d2_leg

    T_34 = np.eye(4)
    T_34[0:3, 0:3] = MF.Rz(q[3])
    T_34[  0,   3] = d3

    T_45 = np.eye(4)  #初始化单位矩阵
    T_45[  0,   3] = d4

    T_bf = T_b1 @ T_12 @ T_23 @ T_34 @ T_45 
    R_bf = T_bf[0:3, 0:3]
    p_bf = T_bf[0:3,   3]

    return R_bf, p_bf


@cc.export('leg_IK', '(f8, f8, f8[:], f8[:], f8[:])')
def leg_IK(leg, option, x_bf, p_bf, q0):
    # leg    - right (+1)
    #        -  left (-1)
    # option - ankle only for now
    r13 = x_bf[0]
    r23 = x_bf[1]
    r33 = x_bf[2]
    px  = p_bf[0]
    py  = p_bf[1]
    pz  = p_bf[2]

    rx = px - lx
    ry = py - ly if leg == +1 else py + ly
    rz = pz - lz
 
    num_sol = 2
    q = np.zeros((num_sol, 5))
    for i in range(num_sol):
  
    
    # 1. 计算 theta2
      q[i, 1] = np.arctan2(r33, np.sqrt(1 - r33**2))
      q[i, 1] = pi_val - q[i, 1]  # 180° - theta2_1
    
    # 2. 计算 theta1
      b = -r13 / np.cos(q[i, 1])
      c = r23 / np.cos(q[i, 1])
      q[i, 0] = np.arctan2(b, c)
    
    # 3. 计算 A 和 B
      s2_val = np.sin(q[i, 1])
      c2_val = np.cos(q[i, 1])
      s1_val = np.sin(q[i, 0])
      c1_val = np.cos(q[i, 0])

      B = (d2_leg * s2_val - pz) / c2_val
      A = (px + s1_val * (-c2_val * d2_leg - s2_val * B)) / c1_val - d1

    # 4. 计算 theta4
      c4_expr = (A**2 + B**2 - d3**2 - d4**2) / (2 * d3 * d4)
      q[i, 3] = np.arctan2(np.sqrt(1 - c4_expr**2), c4_expr)

    # 5. 计算 theta3
      theta3_D = np.arctan2((d3 + d4 * c4_expr) * A - d4 * np.sin(q[i, 3]) * B,
                          (d3 + d4 * c4_expr) * B + d4 * np.sin(q[i, 3]) * A)

      q[i, 2] = theta3_D - pi_val / 4

    return q

