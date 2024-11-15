# test_themis_arm_kinematics.py

import numpy as np
from THEMIS_ARM_KINEMATICS_AOT import arm_IK, arm_FK  # 导入IK和FK函数



def test_arm_FK():
    q = np.array([-162.50409581*3.14159256/180,
                  -17.94305871*3.14159256/180,
                  -8.13197365*3.14159256/180,
                  10.0*3.14159256/180])  # 关节角度，单位为弧度

    # 调用 arm_FK 函数
    R_bf, p_bf = arm_FK(q)

    # 输出 arm_FK 的结果
    print("\nTesting arm_FK with example joint angles:")
    print("Returned position (p_bf):")
    print(np.round(p_bf, decimals=4))
    print("Returned rotation matrix (R_bf):\n")
    print(np.round( R_bf, decimals=4))
    

def test_arm_IK():
    # 测试参数
    x_bf = np.array([0.0435778713738291])  # 表示r13
    p_bf = np.array([28.4618126205338, -2.16644519075057, 5.2744665932482])  # 表示px, py, pz
    q0 = np.array([0.0, 0.0, 0.0, 0.0])  # 初始角度

    # 调用 arm_IK 函数
    q_solutions = arm_IK(x_bf, p_bf, q0)

    # 输出 arm_IK 的结果
    print("Testing arm_IK with example inputs:")
    print("Returned solutions (theta1, theta2, theta3, theta4):")
    for i, q in enumerate(q_solutions):
        print(f"Solution {i + 1}: {q*180/3.14159256}")

# 运行测试
if __name__ == "__main__":
    test_arm_IK()
    test_arm_FK()
