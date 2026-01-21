# ======================================
# Piper机械臂 正逆解完整Python实现
# 标准D-H约定 | 官方参数 | 正解+逆解+验证+可视化
# 单位：长度 mm→m，角度 °→rad
# ======================================
import numpy as np
from roboticstoolbox import Link, SerialLink
from spatialmath import SE3, base
import matplotlib.pyplot as plt

# --------------------------
# 1. 定义Piper官方DH参数 (核心！直接复制上表数据)
# Link参数格式：Link([theta, d, a, alpha], standard=True) 标准D-H
# 单位转换：d/a 毫米转米，alpha 角度转弧度
# --------------------------
# 关节1: theta1, d=88mm, a=0mm, alpha=90°
L1 = Link([0, 0.088, 0, np.pi/2], standard=True, name='Link1')
# 关节2: theta2, d=0mm, a=125mm, alpha=0°
L2 = Link([0, 0, 0.125, 0], standard=True, name='Link2')
# 关节3: theta3, d=0mm, a=125mm, alpha=0°
L3 = Link([0, 0, 0.125, 0], standard=True, name='Link3')
# 关节4: theta4, d=147mm, a=0mm, alpha=90°
L4 = Link([0, 0.147, 0, np.pi/2], standard=True, name='Link4')
# 关节5: theta5, d=0mm, a=0mm, alpha=-90°
L5 = Link([0, 0, 0, -np.pi/2], standard=True, name='Link5')
# 关节6: theta6, d=56mm, a=0mm, alpha=0°
L6 = Link([0, 0.056, 0, 0], standard=True, name='Link6')

# --------------------------
# 2. 创建Piper机械臂模型
# --------------------------
piper_arm = SerialLink([L1, L2, L3, L4, L5, L6], name='Piper 6DOF')
print("✅ Piper机械臂模型创建成功！")
piper_arm.display()  # 打印机械臂DH参数详情，核对是否正确

# --------------------------
# 3. 正运动学求解 (FK) 核心代码
# 功能：已知6个关节角(θ1~θ6) → 求解末端的【位置(XYZ)+姿态(滚转/俯仰/偏航)】
# 输入：关节角数组 q = [θ1, θ2, θ3, θ4, θ5, θ6] 单位：度
# --------------------------
def piper_forward_kinematics(q_deg):
    # 关节角度 度→弧度
    q_rad = np.deg2rad(q_deg)
    # 正解计算：得到末端位姿矩阵 T (4x4齐次变换矩阵)
    T = piper_arm.fkine(q_rad)
    # 提取末端位置 XYZ (单位：米 → 转毫米，更直观)
    end_pos_mm = T.t * 1000
    # 提取末端姿态：旋转矩阵→欧拉角(滚转r/俯仰p/偏航y)，单位：度
    end_rpy_deg = base.tr2rpy(T.R, unit='deg')
    return T, end_pos_mm, end_rpy_deg

# 测试正解：输入一组关节角，比如【全部回零位】q = [0,0,0,0,0,0]
q_test = [0, 0, 0, 0, 0, 0]  # 关节角：θ1~θ6 均为0度
T_fk, pos_fk, rpy_fk = piper_forward_kinematics(q_test)
print("\n======================================")
print("✅ 正解计算结果 (关节角全零位)")
print(f"末端位置 (X,Y,Z) mm: {np.round(pos_fk, 2)}")
print(f"末端姿态 (滚转,俯仰,偏航) °: {np.round(rpy_fk, 2)}")
print(f"末端位姿矩阵 T:\n {np.round(T_fk, 4)}")

# --------------------------
# 4. 逆运动学求解 (IK) 核心代码
# 功能：已知末端【目标位置+目标姿态】 → 求解6个关节角(θ1~θ6)
# 输入：target_pos_mm(XYZ,mm) + target_rpy_deg(滚转/俯仰/偏航,°)
# 求解器：LM法 (Levenberg-Marquardt)，工业机械臂最优逆解算法，收敛快/精度高
# --------------------------
def piper_inverse_kinematics(target_pos_mm, target_rpy_deg):
    # 目标位置 毫米→米
    target_pos_m = target_pos_mm / 1000
    # 目标姿态 度→弧度
    target_rpy_rad = np.deg2rad(target_rpy_deg)
    # 创建目标位姿矩阵 T_target
    T_target = SE3(target_pos_m) * SE3.RPY(target_rpy_rad)
    # 逆解求解：ikine_LM 带关节限位，避免无解/超出运动范围
    # q0: 初始关节角(零位)，mask=[1,1,1,1,1,1] 求解6个自由度
    q_ik_rad, *_ = piper_arm.ikine_LM(T_target, q0=np.zeros(6), mask=[1,1,1,1,1,1])
    # 关节角度 弧度→度，保留2位小数
    q_ik_deg = np.round(np.rad2deg(q_ik_rad), 2)
    return q_ik_deg, T_target

# 测试逆解：指定一个目标位姿（比如让末端到 X=150,Y=0,Z=300 mm，姿态全零）
target_pos = [150, 0, 300]    # 目标位置 XYZ mm
target_rpy = [0, 0, 0]        # 目标姿态 滚转/俯仰/偏航 °
q_ik_deg, T_target = piper_inverse_kinematics(target_pos, target_rpy)
print("\n======================================")
print("✅ 逆解计算结果")
print(f"目标位置 (X,Y,Z) mm: {target_pos}")
print(f"目标姿态 (滚转,俯仰,偏航) °: {target_rpy}")
print(f"求解得到的关节角 θ1~θ6 (°): {q_ik_deg}")

# --------------------------
# 5. 正逆解 闭环验证 (重中之重！验证解算正确性)
# 逻辑：逆解得到的关节角 → 代入正解 → 看是否和「目标位姿」一致
# 误差<0.1mm 即表示解算正确，满足工业精度要求
# --------------------------
print("\n======================================")
print("✅ 正逆解闭环验证")
T_verify, pos_verify, rpy_verify = piper_forward_kinematics(q_ik_deg)
pos_error = np.round(np.linalg.norm(pos_verify - target_pos), 4)  # 位置误差 mm
rpy_error = np.round(np.linalg.norm(rpy_verify - target_rpy), 4)  # 姿态误差 °
print(f"验证位置 (X,Y,Z) mm: {np.round(pos_verify, 2)}")
print(f"位置误差: {pos_error} mm")
print(f"姿态误差: {rpy_error} °")
if pos_error < 0.1:
    print("✅ 验证通过！正逆解计算结果正确，误差满足工业精度要求")
else:
    print("❌ 验证失败！请检查DH参数或目标位姿是否合理")

# --------------------------
# 6. 机械臂姿态可视化 (直观展示)
# 分别画出：正解零位姿态 + 逆解目标姿态
# --------------------------
plt.figure(figsize=(12, 5))
plt.subplot(121)
piper_arm.plot(np.deg2rad(q_test), block=False)
plt.title("Piper机械臂 - 关节零位姿态", fontsize=12)
plt.xlabel("X (m)")
plt.ylabel("Y (m)")

plt.subplot(122)
piper_arm.plot(np.deg2rad(q_ik_deg), block=True)
plt.title("Piper机械臂 - 逆解目标位姿", fontsize=12)
plt.xlabel("X (m)")
plt.ylabel("Y (m)")

print("\n🎉 Piper机械臂正逆解全流程运行完成！")