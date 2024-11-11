import sympy as sp

# 定义符号变量
r13, r23, r33, px, py, pz, d1, d2_leg, d3, d4 = sp.symbols('r13 r23 r33 px py pz d1 d2_leg d3 d4', real=True)
theta1, theta2, theta3, theta4 = sp.symbols('theta1 theta2 theta3 theta4')
c1, s1 = sp.cos(theta1), sp.sin(theta1)
c2, s2 = sp.cos(theta2), sp.sin(theta2)
c3, s3 = sp.cos(theta3), sp.sin(theta3)
c4, s4 = sp.cos(theta4), sp.sin(theta4)

# 定义数值（根据实际情况调整）
values = {
    r33: 0.500000000000000, 
    r13: 0.150383733180435,   
    r23: -0.852868531952443, 
    px: 56.4501873194021,     
    py: -56.4887360944457,     
    pz: -73.3333106650908,     
    d1: 20.0,     
    d2_leg: 20.0, 
    d3: 50.0,     
    d4: 50.0 
}

# 使用近似值替代 pi
pi_val = 3.141592653589793

# 求解 theta2
theta2_1 = sp.atan2(r33, sp.sqrt(1 - r33**2)) * 180 / pi_val
theta2_2 = 180 - theta2_1  # 两个解

# 定义 b, c 用于求解 theta1
b = -r13 / c2
c = r23 / c2
theta1_1 = sp.atan2(b, c) * 180 / pi_val
theta1_2 = sp.atan2(b, c) * 180 / pi_val

# 将所有的数值替换到表达式中
theta2_1_val = theta2_1.subs(values)
theta2_2_val = theta2_2.subs(values)
theta1_1_val = theta1_1.subs(values).subs({c2: sp.cos(theta2_1_val * pi_val / 180), s2: sp.sin(theta2_1_val * pi_val / 180)}).evalf()
theta1_2_val = theta1_2.subs(values).subs({c2: sp.cos(theta2_2_val * pi_val / 180), s2: sp.sin(theta2_2_val * pi_val / 180)}).evalf()

print("Solutions with values substituted:")
print(f"Theta2 solutions:\n theta2_1 = {theta2_1_val}\n theta2_2 = {theta2_2_val}")
print(f"Theta1 solutions:\n theta1_1 = {theta1_1_val}\n theta1_2 = {theta1_2_val}")

# 计算 A 和 B
A = (d2_leg * s2 - pz) / c2
B = (px + s1 * (-c2 * d2_leg - s2 * A)) / c1 - d1

# 将 theta1_1 和 theta2_1 的数值解代入 A 和 B 的表达式
A_val = A.subs(values).subs({
    s2: sp.sin(theta2_1_val * pi_val / 180), 
    c2: sp.cos(theta2_1_val * pi_val / 180),
    s1: sp.sin(theta1_2_val * pi_val / 180), 
    c1: sp.cos(theta1_2_val * pi_val / 180)
}).evalf()

B_val = B.subs(values).subs({
    s2: sp.sin(theta2_1_val * pi_val / 180), 
    c2: sp.cos(theta2_1_val * pi_val / 180),
    s1: sp.sin(theta1_2_val * pi_val / 180), 
    c1: sp.cos(theta1_2_val * pi_val / 180)
}).evalf()

# 计算 theta4
c4_expr = (A_val**2 + B_val**2 - d3**2 - d4**2) / (2 * d3 * d4)
theta4_1 = sp.atan2(sp.sqrt(1 - c4_expr**2), c4_expr) * 180 / pi_val

# 计算 theta3
theta3_D = sp.atan2((d3 + d4*c4_expr) * A_val - d4 * sp.sin(theta4_1 * pi_val / 180) * B_val,
                    (d3+ d4*c4_expr) * B_val + d4 *  sp.sin(theta4_1 * pi_val / 180) * A_val)

theta3_1 = (theta3_D - sp.pi / 4) * 180 / pi_val

# 代入数值计算
theta4_1_val = theta4_1.evalf()
theta3_1_val = theta3_1.evalf()

print(f"Theta4 solution with theta1 and theta2 substituted:\n theta4_1 = {theta4_1_val}")
print(f"Theta3 solution:\n theta3_1 = {theta3_1_val}")
