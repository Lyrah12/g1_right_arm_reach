# DetectedObjectArray.msg — 视觉识别模块输出的多物体列表
#
# 用于一次性发送场景中检测到的所有物体，
# 由视觉模块发布到 /detected_objects。

std_msgs/Header  header              # 时间戳 + frame_id
DetectedObject[] objects             # 检测到的物体列表（按置信度降序排列）
