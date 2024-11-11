#!usr/bin/env python
__author__    = "Westwood Robotics Corporation"
__email__     = "info@westwoodrobotics.io"
__copyright__ = "Copyright 2024 Westwood Robotics Corporation"
__date__      = "October 23, 2024"
__project__   = "BRUCE"
__version__   = "0.0.7"
__status__    = "Product"

'''
Script that holds useful robot macros
'''

import numpy as np
import Util.math_function as MF


# ==================================================
# ==================================================
# CONSTANTS
# ==================================================
# ==================================================
PI   = np.pi
PI_2 = PI / 2.
PI_3 = PI / 3.
PI_4 = PI / 4.

# ==================================================
# ==================================================
# ROBOT INFO & MODELING
# ==================================================
# ==================================================

# ------------------------------
# JOINT NAMES AND NUMBER
# ------------------------------
# Body
BODY = 0

# Leg
HIP_YAW_R     = 1
HIP_PITCH_R   = 2
HIP_ROLL_R    = 3
KNEE_PITCH_R  = 4
ANKLE_PITCH_R = 5

HIP_YAW_L     = 6
HIP_PITCH_L   = 7
HIP_ROLL_L    = 8
KNEE_PITCH_L  = 9
ANKLE_PITCH_L = 10

# Arm
SHOULDER_PITCH_R = 11
SHOULDER_ROLL_R  = 12
ELBOW_YAW_R      = 13

SHOULDER_PITCH_L = 14
SHOULDER_ROLL_L  = 15
ELBOW_YAW_L      = 16

LEG_JOINT_LIST = [HIP_YAW_R, HIP_PITCH_R, HIP_ROLL_R, KNEE_PITCH_R, ANKLE_PITCH_R,
                  HIP_YAW_L, HIP_PITCH_L, HIP_ROLL_L, KNEE_PITCH_L, ANKLE_PITCH_L]

ROBOT_JOINT_LIST = [BODY] + LEG_JOINT_LIST

ARM_JOINT_LIST = [SHOULDER_PITCH_R, SHOULDER_ROLL_R, ELBOW_YAW_R,
                  SHOULDER_PITCH_L, SHOULDER_ROLL_L, ELBOW_YAW_L]

# ------------------------------
# JOINT LIMITS [rad]
# ------------------------------
# Leg
HIP_YAW_MAX = +PI_2
HIP_YAW_MIN = -PI_2

HIP_PITCH_MAX = +2.50
HIP_PITCH_MIN = -0.40

HIP_ROLL_MAX = +PI_4
HIP_ROLL_MIN = -PI_4

KNEE_PITCH_MAX =  0.00
KNEE_PITCH_MIN = -2.50

ANKLE_PITCH_MAX = +1.50
ANKLE_PITCH_MIN = -1.50

LEG_JOINT_POS_MAX = [HIP_YAW_MAX, HIP_PITCH_MAX, HIP_ROLL_MAX, KNEE_PITCH_MAX, ANKLE_PITCH_MAX,
                     HIP_YAW_MAX, HIP_PITCH_MAX, HIP_ROLL_MAX, KNEE_PITCH_MAX, ANKLE_PITCH_MAX]
LEG_JOINT_POS_MIN = [HIP_YAW_MIN, HIP_PITCH_MIN, HIP_ROLL_MIN, KNEE_PITCH_MIN, ANKLE_PITCH_MIN,
                     HIP_YAW_MIN, HIP_PITCH_MIN, HIP_ROLL_MIN, KNEE_PITCH_MIN, ANKLE_PITCH_MIN]

# Arm
SHOULDER_PITCH_MAX = +PI
SHOULDER_PITCH_MIN = -PI

SHOULDER_ROLL_MAX = +PI_2
SHOULDER_ROLL_MIN = -1.9

ELBOW_YAW_MAX = +2.3
ELBOW_YAW_MIN = -2.3

ARM_JOINT_POS_MAX = [SHOULDER_PITCH_MAX, -SHOULDER_ROLL_MIN, ELBOW_YAW_MAX,
                     SHOULDER_PITCH_MAX, +SHOULDER_ROLL_MAX, ELBOW_YAW_MAX]
ARM_JOINT_POS_MIN = [SHOULDER_PITCH_MIN, -SHOULDER_ROLL_MAX, ELBOW_YAW_MIN,
                     SHOULDER_PITCH_MIN, +SHOULDER_ROLL_MIN, ELBOW_YAW_MIN]

# ------------------------------
# KINEMATICS [m]
# ------------------------------
a5 = 0.024
at = 0.050
ah = 0.035

# ------------------------------
# MOTION SUBSPACE MATRIX
# ------------------------------
Sr = np.array([0, 0, 1, 0, 0, 0])  # for revolute joint
MOTION_SUBSPACE = {BODY:          np.eye(6),

                   HIP_YAW_R:     Sr,
                   HIP_PITCH_R:   Sr,
                   HIP_ROLL_R:    Sr,
                   KNEE_PITCH_R:  Sr,
                   ANKLE_PITCH_R: Sr,

                   HIP_YAW_L:     Sr,
                   HIP_PITCH_L:   Sr,
                   HIP_ROLL_L:    Sr,
                   KNEE_PITCH_L:  Sr,
                   ANKLE_PITCH_L: Sr}

# ------------------------------
# COORDINATE TRANSFORM MATRIX AX_B^*
# ------------------------------
COORDINATE_VECTOR = {BODY:          np.zeros(3),

                     HIP_YAW_R:     np.array([0.029216, -0.075856, -0.039765]),
                     HIP_PITCH_R:   np.zeros(3),
                     HIP_ROLL_R:    np.zeros(3),
                     KNEE_PITCH_R:  np.array([0.204949, 0., 0.]),
                     ANKLE_PITCH_R: np.array([0.199881, 0., 0.]),

                     HIP_YAW_L:     np.array([0.029216, +0.075856, -0.039765]),
                     HIP_PITCH_L:   np.zeros(3),
                     HIP_ROLL_L:    np.zeros(3),
                     KNEE_PITCH_L:  np.array([0.204949, 0., 0.]),
                     ANKLE_PITCH_L: np.array([0.199881, 0., 0.])}

# ------------------------------
# LINK COM [m]
# ------------------------------
c = [np.array([0.02134, 0.00033, 0.06115]),
     np.array([3e-06, -0.000248, 0.005224]), np.array([0., 0., 0.]), np.array([0.139,  0.000373, -0.00649]), np.array([0.072931, 0.017464,  0.002164]), np.array([0.012078, 0.001975,  0.000295]),
     np.array([0.000, -0.000208, 0.005280]), np.array([0., 0., 0.]), np.array([0.139, -0.000373, -0.00649]), np.array([0.073100, 0.017810, -0.002198]), np.array([0.012340, 0.002530, -0.000304])]

LINK_COM = {BODY:          c[0],

            HIP_YAW_R:     c[1],
            HIP_PITCH_R:   c[2],
            HIP_ROLL_R:    c[3],
            KNEE_PITCH_R:  c[4],
            ANKLE_PITCH_R: c[5],

            HIP_YAW_L:     c[6],
            HIP_PITCH_L:   c[7],
            HIP_ROLL_L:    c[8],
            KNEE_PITCH_L:  c[9],
            ANKLE_PITCH_L: c[10]}

# ------------------------------
# MASS & INERTIA [KMS]
# ------------------------------
m = [1.5757698,
     0.637563, 0.012955, 0.663519, 0.0956654, 0.0276265,
     0.637386, 0.012955, 0.663519, 0.0952654, 0.0274829]
MASS_BODY = m[0]
MASS_LEG  = m[1] + m[2] + m[3] + m[4] + m[5]
MASS_TOT  = sum(m)

MASS = {BODY:          m[0],

        HIP_YAW_R:     m[1],
        HIP_PITCH_R:   m[2],
        HIP_ROLL_R:    m[3],
        KNEE_PITCH_R:  m[4],
        ANKLE_PITCH_R: m[5],

        HIP_YAW_L:     m[6],
        HIP_PITCH_L:   m[7],
        HIP_ROLL_L:    m[8],
        KNEE_PITCH_L:  m[9],
        ANKLE_PITCH_L: m[10]}

# Link rotational inertia
Irl = [np.array([13941097.18, 1592.63, -440955.57, 5540225.32, 33160.57, 9665761.05]) * 1e-9, 

       np.array([0.00048541, 1.12e-06, 1.74e-06, 0.00161689, -7.02e-06, 0.00152129]),
       np.array([9.10e-07, 0., 0., 9.10e-07, 0., 4.70e-07]),
       np.array([0.000442, +7.29e-05, -0.000291, 0.016, +1.11e-06, 0.0157]),
       np.array([0.00010962, 5.28e-05, 1.57e-05, 0.00101941, -6.68e-06, 0.00108317]),
       np.array([2.04e-05, 2.87e-06, -3e-08, 6.2e-06, -2.4e-07, 2.59e-05]),

       np.array([0.0004852, 0., -1.6e-07, 0.0016181, -7.5e-06, 0.00152105]),
       np.array([9.10e-07, 0., 0., 9.10e-07, 0., 4.70e-07]),
       np.array([0.000442, -7.29e-05, -0.000291, 0.016, -1.11e-06, 0.0157]),
       np.array([0.00010799, 5.26e-05, -1.74e-05, 0.00101705, 6.07e-06, 0.0010792]),
       np.array([2.04e-05, 2.91e-06, 3e-08, 6.27e-06, 2.4e-07, 2.6e-05])]
Ir = []
for i in range(11):
    Iri = np.array([[Irl[i][0], Irl[i][1], Irl[i][2]],
                    [Irl[i][1], Irl[i][3], Irl[i][4]],
                    [Irl[i][2], Irl[i][4], Irl[i][5]]])
    Ir.append(Iri)

INERTIA_BODY = Ir[0]

INERTIA = {BODY:          Ir[0],

           HIP_YAW_R:     Ir[1],
           HIP_PITCH_R:   Ir[2],
           HIP_ROLL_R:    Ir[3],
           KNEE_PITCH_R:  Ir[4],
           ANKLE_PITCH_R: Ir[5],

           HIP_YAW_L:     Ir[6],
           HIP_PITCH_L:   Ir[7],
           HIP_ROLL_L:    Ir[8],
           KNEE_PITCH_L:  Ir[9],
           ANKLE_PITCH_L: Ir[10]}

# Spatial inertia
Is = []
for i in range(11):
    Isi = np.zeros((6, 6))
    c_hat = MF.hat(c[i])
    Isi[0:3, 0:3] = Ir[i] + m[i] * c_hat @ c_hat.T
    Isi[0:3, 3:6] = m[i] * c_hat
    Isi[3:6, 0:3] = m[i] * c_hat.T
    Isi[3:6, 3:6] = m[i] * np.eye(3)
    Is.append(Isi)

SPATIAL_INERTIA = {BODY:          Is[0],

                   HIP_YAW_R:     Is[1],
                   HIP_PITCH_R:   Is[2],
                   HIP_ROLL_R:    Is[3],
                   KNEE_PITCH_R:  Is[4],
                   ANKLE_PITCH_R: Is[5],

                   HIP_YAW_L:     Is[6],
                   HIP_PITCH_L:   Is[7],
                   HIP_ROLL_L:    Is[8],
                   KNEE_PITCH_L:  Is[9],
                   ANKLE_PITCH_L: Is[10]}

# ==================================================
# ==================================================
# HARDWARE SETTINGS
# ==================================================
# ==================================================
# ------------------------------
# SERIAL PORT
# ------------------------------
try:
    with open('/home/khadas/BRUCE/BRUCE_SERIAL_PORT') as f:
        lines = f.readlines()
    BEAR_port   = lines[0][0:-1]
    DXL_port    = lines[1][0:-1]
    PICO_port   = lines[2][0:-1]
    GAMEPAD_mac = lines[3][0:-1]
except:
    BEAR_port   = '/dev/ttyUSB0'
    DXL_port    = '/dev/ttyUSB1'
    PICO_port   = '/dev/ttyACM0'
    GAMEPAD_mac = 'E4:65:B8:74:C2:9A'

BEAR_baudrate = 8000000
DXL_baudrate  = 2000000
PICO_baudrate = 115200

# ------------------------------
# MOTOR ID
# ------------------------------
# Leg
BEAR_HIP1_R  = 1
BEAR_HIP2_R  = 2
BEAR_HIP3_R  = 3
BEAR_KNEE_R  = 4
BEAR_ANKLE_R = 5

BEAR_HIP1_L  = 6
BEAR_HIP2_L  = 7
BEAR_HIP3_L  = 8
BEAR_KNEE_L  = 9
BEAR_ANKLE_L = 10

BEAR_LIST = [BEAR_HIP1_R, BEAR_HIP2_R, BEAR_HIP3_R, BEAR_KNEE_R, BEAR_ANKLE_R,
             BEAR_HIP1_L, BEAR_HIP2_L, BEAR_HIP3_L, BEAR_KNEE_L, BEAR_ANKLE_L]

# Arm
DXL_SHOULDER1_R = 11
DXL_SHOULDER2_R = 12
DXL_ELBOW_R     = 13

DXL_SHOULDER1_L = 14
DXL_SHOULDER2_L = 15
DXL_ELBOW_L     = 16

DXL_LIST = [DXL_SHOULDER1_R, DXL_SHOULDER2_R, DXL_ELBOW_R,
            DXL_SHOULDER1_L, DXL_SHOULDER2_L, DXL_ELBOW_L]

# ------------------------------
# BEAR CHARACTERISTICS
# ------------------------------
REDUCTION = 9.0
IQ2TORQUE = 0.35
TORQUE2IQ = 1. / IQ2TORQUE
REFLECTED_INERTIA = 0.00182

# ------------------------------
# TORQUE LIMITS [Nm]
# ------------------------------
TORQUE_MAX = 10.5  # for Koala BEAR
TORQUE_MIN = -TORQUE_MAX

# ------------------------------
# IQ LIMITS [A]
# ------------------------------
IQ_MAX = 30.
IQ_MIN = -IQ_MAX

# ------------------------------
# BEAR ANGLE LIMITS [rad]
# ------------------------------
BEAR_HIP1_MAX = +PI_2
BEAR_HIP1_MIN = -PI_2

BEAR_HIP2_MAX = +3.0
BEAR_HIP2_MIN = -3.0

BEAR_HIP3_MAX = +3.0
BEAR_HIP3_MIN = -3.0

BEAR_KNEE_MAX = +3.0
BEAR_KNEE_MIN = -3.0

BEAR_ANKLE_MAX = +3.0
BEAR_ANKLE_MIN = -3.0

# ------------------------------
# DXL OFFSET [rad]
# ------------------------------
SHOULDER_PITCH_OFFSET = PI
SHOULDER_ROLL_OFFSET  = PI
ELBOW_YAW_OFFSET      = PI

DXL_OFFSET = [SHOULDER_PITCH_OFFSET, SHOULDER_ROLL_OFFSET, ELBOW_YAW_OFFSET,
              SHOULDER_PITCH_OFFSET, SHOULDER_ROLL_OFFSET, ELBOW_YAW_OFFSET]

# ------------------------------
# MOTOR PID GAINS
# ------------------------------
# Leg
pos_kp = [[60.0, 60.0, 60.0, 60.0, 60.0],
          [60.0, 60.0, 60.0, 60.0, 60.0]]
pos_ki = [[0.00, 0.00, 0.00, 0.00, 0.00],
          [0.00, 0.00, 0.00, 0.00, 0.00]]
pos_kd = [[1.00, 1.00, 1.00, 1.00, 1.00],
          [1.00, 1.00, 1.00, 1.00, 1.00]]

vel_kp = [[1.00, 1.00, 1.00, 1.00, 1.00],
          [1.00, 1.00, 1.00, 1.00, 1.00]]
vel_ki = [[0.00, 0.00, 0.00, 0.00, 0.00],
          [0.00, 0.00, 0.00, 0.00, 0.00]]
vel_kd = [[0.00, 0.00, 0.00, 0.00, 0.00],
          [0.00, 0.00, 0.00, 0.00, 0.00]]

for_kp = [[10.00, 10.00, 10.00, 10.00, 10.00],
          [10.00, 10.00, 10.00, 10.00, 10.00]]
for_ki = [[ 0.00,  0.00,  0.00,  0.00,  0.00],
          [ 0.00,  0.00,  0.00,  0.00,  0.00]]
for_kd = [[ 1.00,  1.00,  1.00,  1.00,  1.00],
          [ 1.00,  1.00,  1.00,  1.00,  1.00]]

# hip 1, 6
BEAR_HIP1_R_POS_P, BEAR_HIP1_L_POS_P = pos_kp[0][0], pos_kp[1][0]
BEAR_HIP1_R_POS_I, BEAR_HIP1_L_POS_I = pos_ki[0][0], pos_ki[1][0]
BEAR_HIP1_R_POS_D, BEAR_HIP1_L_POS_D = pos_kd[0][0], pos_kd[1][0]

BEAR_HIP1_R_VEL_P, BEAR_HIP1_L_VEL_P = vel_kp[0][0], vel_kp[1][0]
BEAR_HIP1_R_VEL_I, BEAR_HIP1_L_VEL_I = vel_ki[0][0], vel_ki[1][0]
BEAR_HIP1_R_VEL_D, BEAR_HIP1_L_VEL_D = vel_kd[0][0], vel_kd[1][0]

BEAR_HIP1_R_FOR_P, BEAR_HIP1_L_FOR_P = for_kp[0][0], for_kp[1][0]
BEAR_HIP1_R_FOR_I, BEAR_HIP1_L_FOR_I = for_ki[0][0], for_ki[1][0]
BEAR_HIP1_R_FOR_D, BEAR_HIP1_L_FOR_D = for_kd[0][0], for_kd[1][0]

# hip 2, 7
BEAR_HIP2_R_POS_P, BEAR_HIP2_L_POS_P = pos_kp[0][1], pos_kp[1][1]
BEAR_HIP2_R_POS_I, BEAR_HIP2_L_POS_I = pos_ki[0][1], pos_ki[1][1]
BEAR_HIP2_R_POS_D, BEAR_HIP2_L_POS_D = pos_kd[0][1], pos_kd[1][1]

BEAR_HIP2_R_VEL_P, BEAR_HIP2_L_VEL_P = vel_kp[0][1], vel_kp[1][1]
BEAR_HIP2_R_VEL_I, BEAR_HIP2_L_VEL_I = vel_ki[0][1], vel_ki[1][1]
BEAR_HIP2_R_VEL_D, BEAR_HIP2_L_VEL_D = vel_kd[0][1], vel_kd[1][1]

BEAR_HIP2_R_FOR_P, BEAR_HIP2_L_FOR_P = for_kp[0][1], for_kp[1][1]
BEAR_HIP2_R_FOR_I, BEAR_HIP2_L_FOR_I = for_ki[0][1], for_ki[1][1]
BEAR_HIP2_R_FOR_D, BEAR_HIP2_L_FOR_D = for_kd[0][1], for_kd[1][1]

# hip 3, 8
BEAR_HIP3_R_POS_P, BEAR_HIP3_L_POS_P = pos_kp[0][2], pos_kp[1][2]
BEAR_HIP3_R_POS_I, BEAR_HIP3_L_POS_I = pos_ki[0][2], pos_ki[1][2]
BEAR_HIP3_R_POS_D, BEAR_HIP3_L_POS_D = pos_kd[0][2], pos_kd[1][2]

BEAR_HIP3_R_VEL_P, BEAR_HIP3_L_VEL_P = vel_kp[0][2], vel_kp[1][2]
BEAR_HIP3_R_VEL_I, BEAR_HIP3_L_VEL_I = vel_ki[0][2], vel_ki[1][2]
BEAR_HIP3_R_VEL_D, BEAR_HIP3_L_VEL_D = vel_kd[0][2], vel_kd[1][2]

BEAR_HIP3_R_FOR_P, BEAR_HIP3_L_FOR_P = for_kp[0][2], for_kp[1][2]
BEAR_HIP3_R_FOR_I, BEAR_HIP3_L_FOR_I = for_ki[0][2], for_ki[1][2]
BEAR_HIP3_R_FOR_D, BEAR_HIP3_L_FOR_D = for_kd[0][2], for_kd[1][2]

# knee 4, 9
BEAR_KNEE_R_POS_P, BEAR_KNEE_L_POS_P = pos_kp[0][3], pos_kp[1][3]
BEAR_KNEE_R_POS_I, BEAR_KNEE_L_POS_I = pos_ki[0][3], pos_ki[1][3]
BEAR_KNEE_R_POS_D, BEAR_KNEE_L_POS_D = pos_kd[0][3], pos_kd[1][3]

BEAR_KNEE_R_VEL_P, BEAR_KNEE_L_VEL_P = vel_kp[0][3], vel_kp[1][3]
BEAR_KNEE_R_VEL_I, BEAR_KNEE_L_VEL_I = vel_ki[0][3], vel_ki[1][3]
BEAR_KNEE_R_VEL_D, BEAR_KNEE_L_VEL_D = vel_kd[0][3], vel_kd[1][3]

BEAR_KNEE_R_FOR_P, BEAR_KNEE_L_FOR_P = for_kp[0][3], for_kp[1][3]
BEAR_KNEE_R_FOR_I, BEAR_KNEE_L_FOR_I = for_ki[0][3], for_ki[1][3]
BEAR_KNEE_R_FOR_D, BEAR_KNEE_L_FOR_D = for_kd[0][3], for_kd[1][3]

# ankle 5, 10
BEAR_ANKLE_R_POS_P, BEAR_ANKLE_L_POS_P = pos_kp[0][4], pos_kp[1][4]
BEAR_ANKLE_R_POS_I, BEAR_ANKLE_L_POS_I = pos_ki[0][4], pos_ki[1][4]
BEAR_ANKLE_R_POS_D, BEAR_ANKLE_L_POS_D = pos_kd[0][4], pos_kd[1][4]

BEAR_ANKLE_R_VEL_P, BEAR_ANKLE_L_VEL_P = vel_kp[0][4], vel_kp[1][4]
BEAR_ANKLE_R_VEL_I, BEAR_ANKLE_L_VEL_I = vel_ki[0][4], vel_ki[1][4]
BEAR_ANKLE_R_VEL_D, BEAR_ANKLE_L_VEL_D = vel_kd[0][4], vel_kd[1][4]

BEAR_ANKLE_R_FOR_P, BEAR_ANKLE_L_FOR_P = for_kp[0][4], for_kp[1][4]
BEAR_ANKLE_R_FOR_I, BEAR_ANKLE_L_FOR_I = for_ki[0][4], for_ki[1][4]
BEAR_ANKLE_R_FOR_D, BEAR_ANKLE_L_FOR_D = for_kd[0][4], for_kd[1][4]

# Arm
DXL_SHOULDER1_POS_P = 800
DXL_SHOULDER1_POS_I = 0
DXL_SHOULDER1_POS_D = 0

DXL_SHOULDER2_POS_P = 800
DXL_SHOULDER2_POS_I = 0
DXL_SHOULDER2_POS_D = 0

DXL_ELBOW_POS_P = 800
DXL_ELBOW_POS_I = 0
DXL_ELBOW_POS_D = 0

DXL_POS_P = [DXL_SHOULDER1_POS_P, DXL_SHOULDER2_POS_P, DXL_ELBOW_POS_P,
             DXL_SHOULDER1_POS_P, DXL_SHOULDER2_POS_P, DXL_ELBOW_POS_P]

DXL_POS_I = [DXL_SHOULDER1_POS_I, DXL_SHOULDER2_POS_I, DXL_ELBOW_POS_I,
             DXL_SHOULDER1_POS_I, DXL_SHOULDER2_POS_I, DXL_ELBOW_POS_I]

DXL_POS_D = [DXL_SHOULDER1_POS_D, DXL_SHOULDER2_POS_D, DXL_ELBOW_POS_D,
             DXL_SHOULDER1_POS_D, DXL_SHOULDER2_POS_D, DXL_ELBOW_POS_D]

# IQ Gains
IQ_P = 0.277
IQ_I = 0.061
IQ_D = 0.00

# ID Gains
ID_P = IQ_P
ID_I = IQ_I
ID_D = 0.00