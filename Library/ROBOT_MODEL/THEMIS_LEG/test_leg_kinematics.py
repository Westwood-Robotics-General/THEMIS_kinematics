import numpy as np
from THEMIS_LEG_KINEMATICS_AOT import leg_FK, leg_IK  # 导入IK和FK函数




def test_leg_FK():
    q = np.array([20.0*3.14159256/180,
                  25.0*3.14159256/180,  
                  5.0*3.14159256/180,   #theta3=ik解+45度
                  -10.0*3.14159256/180])  # 关节角度，单位为弧度   转化为角度
    leg=1
    # 调用 arm_FK 函数
    R_bf, p_bf = leg_FK(leg,q)

    # 输出 arm_FK 的结果
    print("\nTesting arm_FK with example joint angles:")
    print("Returned position (p_bf):")
    print(np.round(p_bf, decimals=4))
    print("Returned rotation matrix (R_bf):\n")
    print(np.round( R_bf, decimals=4))

def test_leg_IK():
    # 测试目标输入参数
    x_bf = np.array([ 0.518022030372010, -0.851650739639147,  0.0796504470882295])   # 假设一些旋转矩阵的方向余弦
    p_bf = np.array([32.5122870666816, -14.0747274466706, -110.844738745994])   # 目标位置
    q0 = 0  # 初始关节角的估计值

    # 调用 arm_IK 函数
    q_solutions = leg_IK(x_bf, p_bf, q0)

    # 输出 arm_IK 的结果
    print("Testing arm_IK with example inputs:")
    print("Returned solutions (theta1, theta2, theta3, theta4):")
    for i, q in enumerate(q_solutions):
        print(f"Solution {i + 1}: {q*180/3.14159256}")

# 运行测试
if __name__ == "__main__":
    test_leg_IK()
    test_leg_FK()
