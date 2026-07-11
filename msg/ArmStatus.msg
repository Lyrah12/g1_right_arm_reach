# ArmStatus.msg — 机械臂状态反馈
#
# 由 reach_node 发布到 /arm_status，
# 供视觉模块、上层决策节点订阅。

# --- 状态机 ---
string  state              # "IDLE"     — 等待目标，机械臂静止
                           # "MOVING"   — 正在向目标运动
                           # "REACHED"  — 已到达目标（位置误差 < tolerance）
                           # "STUCK"    — 无法收敛（奇异点 / 超出工作空间）
                           # "ERROR"    — 内部故障

# --- 误差信息 ---
float64  position_error    # 当前末端到目标的欧氏距离（米）

# --- 当前末端位姿（world / pelvis 坐标系）---
geometry_msgs/Pose  current_ee_pose

# --- 当前目标信息 ---
string   active_object_id  # 当前正在抓取 / 到达的物体 ID（无则为空）
