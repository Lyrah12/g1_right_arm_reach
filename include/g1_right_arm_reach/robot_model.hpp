/** @file robot_model.hpp
 *  @brief G1 right-arm model: URDF → reduced 7-DOF Pinocchio model.
 *
 *  RobotModel is the single source of truth for kinematic structure.
 *  Supports Arm-Hand decoupling via buildSubsetIndices():
 *    arm_joints   = [shoulder_pitch, shoulder_roll, shoulder_yaw, elbow]
 *    wrist_joints = [wrist_roll, wrist_pitch, wrist_yaw]
 *  EE frame = wrist_roll_link (origin, position control target).
 *  Hand frame = rubber_hand (distal, orientation observation).
 */

#ifndef G1_RIGHT_ARM_REACH__ROBOT_MODEL_HPP_
#define G1_RIGHT_ARM_REACH__ROBOT_MODEL_HPP_

#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/parsers/urdf.hpp>

#include <Eigen/Dense>

#include <string>
#include <vector>
#include <memory>

// ============================================================================

namespace g1_right_arm_reach {

// ============================================================================
// MODULE: RobotModel — reduced right-arm model and FK
// ============================================================================

/**
 * @brief Core robot model manager for the G1 right arm.
 *
 * Loads the full G1 URDF, then builds a reduced model with only the
 * right arm's 7 active joints.  All other joints (left arm, legs, waist) are
 * locked at their reference configuration and replaced by FIXED_JOINT frames
 * in the reduced model.
 *
 * Supports arm/wrist decoupling via arm_joint_indices_ / wrist_joint_indices_.
 */
class RobotModel {
public:
    // --- Construction ---

    /// Load URDF, build reduced 7-DOF right-arm model.
    /// @param urdf_path          Absolute path to G1 URDF.
    /// @param active_joint_names 7 right-arm joint names.
    /// @param q_ref_full         Full-model reference config (empty = neutral).
    RobotModel(const std::string& urdf_path,
               const std::vector<std::string>& active_joint_names,
               const Eigen::VectorXd& q_ref_full = Eigen::VectorXd());

    /// Map arm/wrist joint names to reduced-model indices after construction.
    void buildSubsetIndices(const std::vector<std::string>& arm_joint_names,
                            const std::vector<std::string>& wrist_joint_names);

    // --- Kinematics ---

    void updateKinematics();
    Eigen::Vector3d getEndEffectorPosition() const;       ///< EE (wrist origin) position
    Eigen::Vector3d getWristPosition() const;              ///< Alias, = EE position
    Eigen::Vector3d getRightShoulderPosition() const;      ///< Shoulder joint origin (world)
    Eigen::Matrix3d getHandOrientation() const;            ///< Rubber_hand orientation (distal)

    void setNeutralConfig();
    void clampToLimits();
    void printSummary() const;

    int nq() const { return reduced_model_.nq; }
    int nv() const { return reduced_model_.nv; }

    // --- Data (public for cooperating classes) ---

    pinocchio::Model full_model_, reduced_model_;
    pinocchio::Data   full_data_, reduced_data_;

    pinocchio::FrameIndex ee_frame_id_, ee_frame_id_full_, hand_frame_id_;

    std::vector<pinocchio::JointIndex> active_joint_ids_full_, locked_joint_ids_full_;

    Eigen::VectorXd q_min_, q_max_, dq_max_;
    Eigen::VectorXd q_, dq_;
    Eigen::VectorXd q_ref_full_;

    std::vector<std::string> active_joint_names_;
    std::vector<int> arm_joint_indices_, wrist_joint_indices_;

private:
    std::vector<pinocchio::JointIndex> identifyJointsToLock(
        const std::vector<std::string>& active_names) const;
    pinocchio::FrameIndex findEndEffectorFrame(const pinocchio::Model& model) const;
    void extractJointLimits();
    pinocchio::FrameIndex findFrameByName(const pinocchio::Model& model,
                                          const std::string& name) const;
};

}  // namespace g1_right_arm_reach

#endif  // G1_RIGHT_ARM_REACH__ROBOT_MODEL_HPP_
