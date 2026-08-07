#include "g1_right_arm_reach/robot_model.hpp"

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/model.hpp>
#include <pinocchio/parsers/urdf.hpp>

#include <algorithm>
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <unordered_set>

namespace g1_right_arm_reach {

RobotModel::RobotModel(const std::string& urdf_path,
                       const std::vector<std::string>& active_joint_names,
                       const Eigen::VectorXd& q_ref_full,
                       const std::string& ee_frame_name)
    : active_joint_names_(active_joint_names), ee_frame_name_(ee_frame_name)
{
    pinocchio::urdf::buildModel(urdf_path, full_model_);
    full_data_ = pinocchio::Data(full_model_);
    q_ref_full_ = q_ref_full.size() == full_model_.nq
        ? q_ref_full : pinocchio::neutral(full_model_);

    for (const auto& name : active_joint_names_) {
        if (!full_model_.existJointName(name)) {
            throw std::runtime_error("active joint missing from URDF: " + name);
        }
        const auto jid = full_model_.getJointId(name);
        if (full_model_.joints[jid].nq() != 1 || full_model_.joints[jid].nv() != 1) {
            throw std::runtime_error("only scalar right-arm joints are supported: " + name);
        }
        active_joint_ids_full_.push_back(jid);
    }

    locked_joint_ids_full_ = identifyJointsToLock(active_joint_names_);
    reduced_model_ = pinocchio::buildReducedModel(
        full_model_, locked_joint_ids_full_, q_ref_full_);
    reduced_data_ = pinocchio::Data(reduced_model_);

    ee_frame_id_full_ = findEndEffectorFrame(full_model_, ee_frame_name_);
    ee_frame_id_ = findEndEffectorFrame(reduced_model_, ee_frame_name_);
    ee_frame_name_ = reduced_model_.frames[ee_frame_id_].name;

    q_ = pinocchio::neutral(reduced_model_);
    dq_ = Eigen::VectorXd::Zero(reduced_model_.nv);
    extractJointLimits();
    buildSubsetIndices(active_joint_names_);
    updateKinematics();
}

std::vector<pinocchio::JointIndex> RobotModel::identifyJointsToLock(
    const std::vector<std::string>& active_names) const
{
    std::unordered_set<pinocchio::JointIndex> keep;
    for (const auto& name : active_names) {
        keep.insert(full_model_.getJointId(name));
    }
    std::vector<pinocchio::JointIndex> result;
    for (pinocchio::JointIndex jid = 1; jid < full_model_.joints.size(); ++jid) {
        if (keep.count(jid) == 0) result.push_back(jid);
    }
    return result;
}

pinocchio::FrameIndex RobotModel::findEndEffectorFrame(
    const pinocchio::Model& model, const std::string& requested) const
{
    const std::vector<std::string> candidates{
        requested, "right_revo2_tcp", "right_base_link", "right_wrist_roll_link"};
    for (const auto& name : candidates) {
        if (!name.empty() && model.existFrame(name)) return model.getFrameId(name);
    }
    throw std::runtime_error("no right-arm end-effector frame found");
}

int RobotModel::coordinateIndex(const std::string& joint_name) const
{
    if (!reduced_model_.existJointName(joint_name)) return -1;
    const auto jid = reduced_model_.getJointId(joint_name);
    if (reduced_model_.joints[jid].nq() != 1) return -1;
    return reduced_model_.joints[jid].idx_q();
}

void RobotModel::buildSubsetIndices(const std::vector<std::string>& arm_joint_names,
                                    const std::vector<std::string>& wrist_joint_names)
{
    arm_joint_indices_.clear();
    wrist_joint_indices_.clear();
    arm_joint_names_ = arm_joint_names;
    for (const auto& name : arm_joint_names) {
        const int index = coordinateIndex(name);
        if (index < 0) throw std::runtime_error("arm joint missing from reduced model: " + name);
        arm_joint_indices_.push_back(index);
    }
    for (const auto& name : wrist_joint_names) {
        const int index = coordinateIndex(name);
        if (index < 0) throw std::runtime_error("wrist joint missing from reduced model: " + name);
        wrist_joint_indices_.push_back(index);
    }
}

int RobotModel::armJointColumn(const std::string& joint_name) const
{
    for (size_t k = 0; k < arm_joint_names_.size(); ++k) {
        if (arm_joint_names_[k] == joint_name) return static_cast<int>(k);
    }
    return -1;
}

void RobotModel::extractJointLimits()
{
    constexpr double pi = 3.14159265358979323846;
    q_min_ = reduced_model_.lowerPositionLimit;
    q_max_ = reduced_model_.upperPositionLimit;
    dq_max_ = reduced_model_.velocityLimit;
    tau_max_ = reduced_model_.effortLimit;
    for (int i = 0; i < reduced_model_.nq; ++i) {
        if (!std::isfinite(q_min_(i))) q_min_(i) = -pi;
        if (!std::isfinite(q_max_(i))) q_max_(i) = pi;
        if (!std::isfinite(dq_max_(i)) || dq_max_(i) <= 0.0) dq_max_(i) = 2.0;
        if (!std::isfinite(tau_max_(i)) || tau_max_(i) <= 0.0) tau_max_(i) = 25.0;
    }
}

void RobotModel::updateKinematics()
{
    pinocchio::forwardKinematics(reduced_model_, reduced_data_, q_, dq_);
    pinocchio::updateFramePlacements(reduced_model_, reduced_data_);
}

Eigen::Vector3d RobotModel::getEndEffectorPosition() const
{
    return reduced_data_.oMf[ee_frame_id_].translation();
}

Eigen::Matrix3d RobotModel::getHandOrientation() const
{
    return reduced_data_.oMf[ee_frame_id_].rotation();
}

Eigen::Vector3d RobotModel::getRightShoulderPosition() const
{
    if (!reduced_model_.existJointName("right_shoulder_pitch_joint")) {
        return Eigen::Vector3d::Zero();
    }
    const auto jid = reduced_model_.getJointId("right_shoulder_pitch_joint");
    return reduced_data_.oMi[jid].translation();
}

void RobotModel::setNeutralConfig()
{
    q_ = pinocchio::neutral(reduced_model_);
    dq_.setZero();
    updateKinematics();
}

void RobotModel::clampToLimits()
{
    for (int i = 0; i < q_.size(); ++i) {
        q_(i) = std::max(q_min_(i), std::min(q_max_(i), q_(i)));
    }
}

void RobotModel::printSummary() const
{
    std::cout << "[RobotModel] " << reduced_model_.nq << " active DOF, EE='"
              << ee_frame_name_ << "'\n";
    for (const auto& name : active_joint_names_) {
        std::cout << "  " << name << " -> q[" << coordinateIndex(name) << "]\n";
    }
}

}  // namespace g1_right_arm_reach
