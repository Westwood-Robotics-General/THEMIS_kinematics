import sympy as sp

# 定义符号变量
theta1, theta2, theta3, theta4 = sp.symbols('theta1 theta2 theta3 theta4')
c1, s1 = sp.cos(theta1), sp.sin(theta1)
c2, s2 = sp.cos(theta2), sp.sin(theta2)
c3, s3 = sp.cos(theta3), sp.sin(theta3)
c4, s4 = sp.cos(theta4), sp.sin(theta4)
fx, fy, fz, d1, d2_leg, d3, d4, lx, ly_leg, lz = sp.symbols(
    'fx fy fz d1 d2_leg d3 d4 lx ly_leg lz', real=True
)

# 定义 sqrt(2)/2
half_sqrt2 = sp.sqrt(2) / 2

# 定义矩阵 {}^b T_f
Tb1 = sp.Matrix([
    [half_sqrt2 * c1, -half_sqrt2 * s1, half_sqrt2, lx],
    [s1, c1, 0, ly_leg],
    [-half_sqrt2 * c1, half_sqrt2 * s1, half_sqrt2, lz],
    [0, 0, 0, 1]
])

T12 = sp.Matrix([
    [0, 0, -1, d1],
    [-c2, s2, 0, 0],
    [s2, c2, 0, 0],
    [0, 0, 0, 1]
])

# 替换 c3 和 s3
T23 = sp.Matrix([
    [0, 0, 1, d2_leg],
    [half_sqrt2 * (half_sqrt2 * s3 - half_sqrt2 * c3) - half_sqrt2 * (half_sqrt2 * c3 + half_sqrt2 * s3),
     half_sqrt2 * (half_sqrt2 * s3 - half_sqrt2 * c3) + half_sqrt2 * (half_sqrt2 * c3 + half_sqrt2 * s3), 0, 0],
    [-half_sqrt2 * (half_sqrt2 * s3 - half_sqrt2 * c3) - half_sqrt2 * (half_sqrt2 * c3 + half_sqrt2 * s3),
     half_sqrt2 * (half_sqrt2 * s3 - half_sqrt2 * c3) - half_sqrt2 * (half_sqrt2 * c3 + half_sqrt2 * s3), 0, 0],
    [0, 0, 0, 1]
])

T34 = sp.Matrix([
    [c4, -s4, 0, d3],
    [s4, c4, 0, 0],
    [0, 0, 1, 0],
    [0, 0, 0, 1]
])

T45 = sp.Matrix([
    [1, 0, 0, d4],
    [0, 1, 0, 0],
    [0, 0, 1, 0],
    [0, 0, 0, 1]
])

# 计算 {}^b T_0 的逆
Tb0_inv = sp.Matrix([
    [half_sqrt2, 0, -half_sqrt2, -half_sqrt2 * lx + half_sqrt2 * lz],
    [0, 1, 0, -ly_leg],
    [half_sqrt2, 0, half_sqrt2, -half_sqrt2 * lx - half_sqrt2 * lz],
    [0, 0, 0, 1]
])

# 计算 {}^b T_f
Tb5 =Tb0_inv* Tb1 * T12 * T23 * T34 * T45


# 定义常值
values = {
    d1:20.0,     
    d2_leg: 20.0, 
    d3: 50.0,     
    d4: 50.0 ,     
    theta1: sp.rad(55),  # 假设 theta1 = 30 度
    theta2: sp.rad(30),  # 假设 theta2 = 45 度
    theta3: sp.rad(60),  # 假设 theta3 = 60 度
    theta4: sp.rad(15),  # 假设 theta4 = 90 度
    lx: 0.0,             # 假设 lx = 0.1
    ly_leg: 0.0,         # 假设 ly_leg = 0.2
    lz: 0.0              # 假设 lz = 0.3
}


# 将常值替换到位姿矩阵中
Tb5_numeric = Tb5.subs(values)

# 显示简化后的数值结果
for i in range(4):
    print(f"Row {i+1}:")
    for j in range(4):
        print(f"  Column {j+1}: {Tb5_numeric[i, j].evalf()}")
    print()  # 换行以区分行



