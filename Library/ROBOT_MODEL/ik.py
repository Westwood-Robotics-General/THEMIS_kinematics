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

# 定义矩阵 {}^b T_0
Tb0 = sp.Matrix([
    [half_sqrt2, 0, half_sqrt2, lx],
    [0, 1, 0, ly_leg],
    [-half_sqrt2, 0, half_sqrt2, lz],
    [0, 0, 0, 1]
])

# 计算 {}^b T_0 的逆
Tb0_inv = sp.Matrix([
    [half_sqrt2, 0, -half_sqrt2, -half_sqrt2 * lx + half_sqrt2 * lz],
    [0, 1, 0, -ly_leg],
    [half_sqrt2, 0, half_sqrt2, -half_sqrt2 * lx - half_sqrt2 * lz],
    [0, 0, 0, 1]
])

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


# 计算 {}^b T_f
Tb5 = Tb1 * T12 * T23 * T34 * T45

# 计算 {}^b T_f 的最终结果
result = Tb0_inv * Tb5

# 简化结果
result_simplified = sp.simplify(result)

# 分块显示结果
print("Simplified Result (Block by Block):")
for i in range(4):
    print(f"Row {i+1}:")
    for j in range(4):
        print(f"  Column {j+1}: {result_simplified[i, j]}")
    print()
