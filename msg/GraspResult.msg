# GraspResult.msg — 抓取操作结果
#
# 由 reach_node / grasp_action_server 在抓取完成后发布到 /grasp_result。

string   object_id         # 对应 DetectedObject.object_id
bool     success           # true = 抓取成功，false = 抓取失败
float64  final_error       # 抓取结束时末端到目标的距离（m）
string   error_msg         # 失败原因（成功时为空）
