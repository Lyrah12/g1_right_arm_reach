#ifndef G1_RIGHT_ARM_REACH__ROBOT_MODEL_HPP_
#define G1_RIGHT_ARM_REACH__ROBOT_MODEL_HPP_

#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>

#include <Eigen/Dense>

#include <string>
#include <vector>

namespace g1_right_arm_reach {

/** Reduced Pinocchio model containing only the five G1 right-arm joints. */
class RobotModel {
public:
    RobotModel(const std::string& urdf_path,
               const std::vector<std::string>& active_joint_names,
               const Eigen::VectorXd& q_ref_full = Eigen::VectorXd(),
               const std::string& ee_frame_name = "right_revo2_tcp");

    /** Build reduced-coordinate indices. For the 23-DOF G1, arm contains all 5 joints. */
    void buildSubsetIndices(const std::vector<std::string>& arm_joint_names,
                            const std::vector<std::string>& wrist_joint_names = {});

    /** Column index in arm_joint_indices_ / the task Jacobian for a joint name, or -1. */
    int armJointColumn(const std::string& joint_name) const;

    void updateKinematics();
    Eigen::Vector3d getEndEffectorPosition() const;
    Eigen::Vector3d getWristPosition() const { return getEndEffectorPosition(); }
    Eigen::Vector3d getRightShoulderPosition() const;
    Eigen::Matrix3d getHandOrientation() const;

    void setNeutralConfig();
    void clampToLimits();
    void printSummary() const;

    int nq() const { return reduced_model_.nq; }
    int nv() const { return reduced_model_.nv; }
    int coordinateIndex(const std::string& joint_name) const;

    pinocchio::Model full_model_, reduced_model_;
    pinocchio::Data full_data_, reduced_data_;
    pinocchio::FrameIndex ee_frame_id_{0}, ee_frame_id_full_{0};

    std::vector<pinocchio::JointIndex> active_joint_ids_full_, locked_joint_ids_full_;
    Eigen::VectorXd q_min_, q_max_, dq_max_, tau_max_;
    Eigen::VectorXd q_, dq_, q_ref_full_;
    std::vector<std::string> active_joint_names_, arm_joint_names_;
    std::vector<int> arm_joint_indices_, wrist_joint_indices_;

private:
    std::vector<pinocchio::JointIndex> identifyJointsToLock(
        const std::vector<std::string>& active_names) const;
    pinocchio::FrameIndex findEndEffectorFrame(const pinocchio::Model& model,
                                                const std::string& requested) const;
    void extractJointLimits();
    std::string ee_frame_name_;
};

}  // namespace g1_right_arm_reach

#endif
