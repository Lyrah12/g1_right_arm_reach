/**
 * @file robot_model.cpp
 * Implements RobotModel: URDF loading, reduced-model building, FK, and joint limits.
 */

#include "g1_right_arm_reach/robot_model.hpp"

#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/model.hpp>

#include <iostream>
#include <algorithm>
#include <unordered_set>

namespace g1_right_arm_reach {

// ============================================================================
// Constructor
// ============================================================================

RobotModel::RobotModel(
    const std::string& urdf_path,
    const std::vector<std::string>& active_joint_names,
    const Eigen::VectorXd& q_ref_full)
    : active_joint_names_(active_joint_names)
{
    // 1. Load full model from URDF
    pinocchio::urdf::buildModel(urdf_path, full_model_);
    full_data_ = pinocchio::Data(full_model_);

    // 2. Reference configuration
    q_ref_full_ = (q_ref_full.size() == 0) ? pinocchio::neutral(full_model_) : q_ref_full;

    // 3. Identify active vs locked joints
    locked_joint_ids_full_ = identifyJointsToLock(active_joint_names_);
    for (pinocchio::JointIndex jid = 1; jid < full_model_.joints.size(); ++jid) {
        if (std::find(locked_joint_ids_full_.begin(), locked_joint_ids_full_.end(), jid)
            == locked_joint_ids_full_.end()) {
            active_joint_ids_full_.push_back(jid);
        }
    }

    // 4. Build reduced model (5-DOF right arm)
    reduced_model_ = pinocchio::buildReducedModel(full_model_, locked_joint_ids_full_, q_ref_full_);
    reduced_data_ = pinocchio::Data(reduced_model_);

    // 5. Locate EE frames
    ee_frame_id_full_ = findEndEffectorFrame(full_model_);   // full model
    ee_frame_id_      = findEndEffectorFrame(reduced_model_); // reduced model

    // 6. Extract joint limits & init config
    extractJointLimits();
    q_ = pinocchio::neutral(reduced_model_);
    dq_ = Eigen::VectorXd::Zero(reduced_model_.nv);
    updateKinematics();

    std::cout << "[RobotModel] Full: " << full_model_.nq << " DOF → Reduced: "
              << reduced_model_.nq << " DOF (active=" << active_joint_ids_full_.size()
              << ", locked=" << locked_joint_ids_full_.size()
              << "), EE='" << full_model_.frames[ee_frame_id_full_].name << "'\n";
}

// ============================================================================
// Joint locking
// ============================================================================

std::vector<pinocchio::JointIndex> RobotModel::identifyJointsToLock(
    const std::vector<std::string>& active_names) const
{
    std::unordered_set<pinocchio::JointIndex> keep;
    for (const auto& name : active_names) {
        if (full_model_.existJointName(name))
            keep.insert(full_model_.getJointId(name));
        else
            std::cerr << "[RobotModel] WARNING: joint '" << name << "' not found\n";
    }

    std::vector<pinocchio::JointIndex> lock_list;
    for (pinocchio::JointIndex jid = 1; jid < full_model_.joints.size(); ++jid)
        if (keep.find(jid) == keep.end()) lock_list.push_back(jid);
    return lock_list;
}

// ============================================================================
// EE frame lookup — unified for full & reduced models
// ============================================================================

pinocchio::FrameIndex RobotModel::findEndEffectorFrame(const pinocchio::Model& model) const
{
    // Exact match, right-side only (most specific first)
    for (const auto& hint : {"right_wrist_roll_rubber_hand", "right_rubber_hand",
                              "right_palm", "right_hand", "right_gripper"}) {
        for (pinocchio::FrameIndex fid = 0; fid < model.nframes; ++fid)
            if (model.frames[fid].name == hint) return fid;
    }

    // Substring match: must contain "right" + "hand", not "left"
    for (pinocchio::FrameIndex fid = 0; fid < model.nframes; ++fid) {
        std::string lo = model.frames[fid].name;
        std::transform(lo.begin(), lo.end(), lo.begin(), ::tolower);
        if (lo.find("right") != std::string::npos &&
            lo.find("hand") != std::string::npos &&
            lo.find("left") == std::string::npos) return fid;
    }

    // Fallback: last frame
    std::cerr << "[RobotModel] WARNING: EE frame not found by name, using last frame\n";
    return model.nframes - 1;
}

// ============================================================================
// Kinematics update
// ============================================================================

void RobotModel::updateKinematics()
{
    pinocchio::forwardKinematics(reduced_model_, reduced_data_, q_);
    pinocchio::updateFramePlacements(reduced_model_, reduced_data_);
}

Eigen::Vector3d RobotModel::getEndEffectorPosition() const
{
    return reduced_data_.oMf[ee_frame_id_].translation();
}

// ============================================================================
// Joint limits
// ============================================================================

void RobotModel::extractJointLimits()
{
    q_min_ = reduced_model_.lowerPositionLimit;
    q_max_ = reduced_model_.upperPositionLimit;
    dq_max_ = reduced_model_.velocityLimit;

    const double kDefaultPos = 2.0 * M_PI;
    const double kDefaultVel = 1.5;
    for (int i = 0; i < reduced_model_.nv; ++i) {
        if (q_min_(i) < -1e10)  q_min_(i) = -kDefaultPos;
        if (q_max_(i) > 1e10)   q_max_(i) =  kDefaultPos;
        if (dq_max_(i) < 0 || dq_max_(i) > 1e10) dq_max_(i) = kDefaultVel;
    }
}

void RobotModel::clampToLimits()
{
    q_ = q_.cwiseMax(q_min_).cwiseMin(q_max_);
}

// ============================================================================
// Helpers
// ============================================================================

void RobotModel::setNeutralConfig()
{
    q_ = pinocchio::neutral(reduced_model_);
    dq_.setZero();
}

void RobotModel::printSummary() const
{
    std::cout << "\n========== G1 Right Arm Model Summary ==========\n"
              << "Full: " << full_model_.nq << " DOF  →  Reduced: "
              << reduced_model_.nq << " DOF\n\nActive joints:\n";
    for (int i = 0; i < reduced_model_.nv; ++i) {
        std::string jname = (i < static_cast<int>(active_joint_names_.size()))
                            ? active_joint_names_[i]
                            : reduced_model_.names[static_cast<size_t>(i) + 1];
        std::cout << "  [" << i << "] " << jname
                  << "  [" << q_min_(i) << ", " << q_max_(i) << "] rad"
                  << "  v_max=" << dq_max_(i) << " rad/s\n";
    }
    std::cout << "\nEE: '" << reduced_model_.frames[ee_frame_id_].name
              << "'  pos=" << getEndEffectorPosition().transpose()
              << "\n=================================================\n\n";
}

}  // namespace g1_right_arm_reach
