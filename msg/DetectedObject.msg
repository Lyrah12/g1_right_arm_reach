# DetectedObject.msg — 视觉识别模块输出的单个检测物体
#
# 由视觉模块发布到 /detected_object 或 /detected_objects，
# 供 reach_node / grasp_action_server 订阅。

string   object_id        # 物体唯一 ID，用于追踪（如 "obj_001"）
string   object_class     # 物体类别（如 "cup", "bottle", "pen"）
string   grasp_type       # 推荐抓取策略（如 "top_grasp", "side_grasp", "pinch"）
float64  confidence       # 检测置信度 [0.0, 1.0]，-1 表示未提供

# 推荐的抓取位姿（在视觉坐标系下，通常为 camera_link）
geometry_msgs/PoseStamped grasp_pose

# 物体包围盒尺寸（可选，用于碰撞避免和抓取规划）
float64  bbox_x           # 包围盒 x 方向长度（m），-1 表示未提供
float64  bbox_y           # 包围盒 y 方向长度（m）
float64  bbox_z           # 包围盒 z 方向长度（m）
