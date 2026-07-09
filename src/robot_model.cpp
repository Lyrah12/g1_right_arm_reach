/**
 * @file robot_model.cpp
 *
 * Implements the RobotModel class: loads the G1 URDF, builds a reduced
 * model locking all non-right-arm joints, locates the end-effector frame
 * in both full and reduced models, and provides FK and config helpers.
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
// MODULE: Constructor / Initialization
// ============================================================================

// Construct the robot model: load URDF, lock inactive joints, locate
// end-effector frames, extract joint limits, initialize to neutral.
//
// 1. Load full model from URDF
// 2. Set reference configuration (neutral or user-supplied)
// 3. Identify which joints to lock vs keep active
// 4. Build reduced model (for FK) with only active joints
// 5. Locate EE frame in full model (for Jacobian) and reduced model (for FK)
// 6. Extract joint limits and initialize config to neutral
RobotModel::RobotModel(
    const std::string& urdf_path,
    const std::vector<std::string>& active_joint_names,
    const Eigen::VectorXd& q_ref_full)
    : active_joint_names_(active_joint_names)
{
    // --- 1. Load full model from URDF ---
    std::cout << "[RobotModel] Loading URDF from: " << urdf_path << std::endl;
    pinocchio::urdf::buildModel(urdf_path, full_model_);
    full_data_ = pinocchio::Data(full_model_);
    std::cout << "[RobotModel] Full model loaded: "
              << full_model_.nq << " position DOF, "
              << full_model_.nv << " velocity DOF" << std::endl;

    // --- 2. Set reference configuration ---
    if (q_ref_full.size() == 0) {
        q_ref_full_ = pinocchio::neutral(full_model_);
    } else {
        q_ref_full_ = q_ref_full;
    }

    // --- 3. Identify active vs locked joints in the FULL model ---
    std::vector<pinocchio::JointIndex> joints_to_lock = identifyJointsToLock(active_joint_names_);
    locked_joint_ids_full_ = joints_to_lock;

    // Collect active joint IDs (all non-root joints minus locked ones)
    for (pinocchio::JointIndex jid = 1; jid < full_model_.joints.size(); ++jid) {
        if (std::find(joints_to_lock.begin(), joints_to_lock.end(), jid) == joints_to_lock.end()) {
            active_joint_ids_full_.push_back(jid);
        }
    }
    std::cout << "[RobotModel] Active joints in full model: " << active_joint_ids_full_.size()
              << " — Locked: " << joints_to_lock.size() << std::endl;

    // --- 4. Build reduced model (for FK only) ---
    reduced_model_ = pinocchio::buildReducedModel(full_model_, joints_to_lock, q_ref_full_);
    reduced_data_ = pinocchio::Data(reduced_model_);

    std::cout << "[RobotModel] Reduced model: "
              << reduced_model_.nq << " position DOF, "
              << reduced_model_.nv << " velocity DOF" << std::endl;

    // --- 5. Find end-effector frame in FULL model (for Jacobian) ---
    ee_frame_id_full_ = findEndEffectorFrameFull();
    std::cout << "[RobotModel] EE frame (full model): '"
              << full_model_.frames[ee_frame_id_full_].name
              << "' (ID=" << ee_frame_id_full_ << ")" << std::endl;

    // --- 6. Find end-effector frame in REDUCED model (for FK display) ---
    ee_frame_id_ = findEndEffectorFrame();

    // --- 5. Extract joint limits ---
    extractJointLimits();

    // --- 6. Initialize to neutral config ---
    q_ = pinocchio::neutral(reduced_model_);
    dq_ = Eigen::VectorXd::Zero(reduced_model_.nv);
    updateKinematics();
}

// ============================================================================
// MODULE: Model Building and Joint Selection
// ============================================================================

// Determine which full-model joints to lock so only the specified
// active joints remain free.  Joint ID 0 (universe) is always kept.
// Returns the list of joint indices to pass to buildReducedModel.
std::vector<pinocchio::JointIndex> RobotModel::identifyJointsToLock(
    const std::vector<std::string>& active_names) const
{
    // Build a set of joint IDs to keep
    std::unordered_set<pinocchio::JointIndex> keep_set;
    for (const auto& name : active_names) {
        if (full_model_.existJointName(name)) {
            keep_set.insert(full_model_.getJointId(name));
        } else {
            std::cerr << "[RobotModel] WARNING: Joint '" << name
                      << "' not found in full model!" << std::endl;
        }
    }

    // Collect all joints EXCEPT:
    // - the universe joint (ID 0, always kept)
    // - joints in the keep_set
    std::vector<pinocchio::JointIndex> lock_list;
    for (pinocchio::JointIndex jid = 1; jid < full_model_.joints.size(); ++jid) {
        if (keep_set.find(jid) == keep_set.end()) {
            lock_list.push_back(jid);
        }
    }

    return lock_list;
}

// Search the REDUCED model for the right-hand end-effector frame name,
// falling back through wrist joint parent lookup to the last frame.
//
// 1. Try exact/case-insensitive match against known right-hand name patterns
// 2. If right_wrist_roll_joint exists, use the frame whose parent is that joint
// 3. Fallback: return the last frame in the reduced model
pinocchio::FrameIndex RobotModel::findEndEffectorFrame() const
{
    // Priority 1: Search for frame names containing known patterns
    const std::vector<std::string> hints = {
        "right_rubber_hand",
        "right_palm",
        "right_hand",
        "right_gripper",
        "right_wrist_roll",
        "rubber_hand",
        "palm",
        "hand",
        "end_effector",
        "ee",
    };

    for (const auto& hint : hints) {
        for (pinocchio::FrameIndex fid = 0; fid < reduced_model_.nframes; ++fid) {
            const std::string& fname = reduced_model_.frames[fid].name;
            // Case-insensitive contains search
            auto it = std::search(
                fname.begin(), fname.end(),
                hint.begin(), hint.end(),
                [](char c1, char c2) {
                    return std::tolower(c1) == std::tolower(c2);
                });
            if (it != fname.end()) {
                return fid;
            }
        }
    }

    // Priority 2: Find the last joint in the right arm chain.
    // In the URDF, right_wrist_roll_joint's child link is the terminal link.
    if (reduced_model_.existJointName("right_wrist_roll_joint")) {
        pinocchio::JointIndex wrist_id =
            reduced_model_.getJointId("right_wrist_roll_joint");
        // Get the child body of this joint
        // The frame for this body is usually at the end of the frames list
        // Search for frames whose parent joint is the wrist joint
        for (pinocchio::FrameIndex fid = 0; fid < reduced_model_.nframes; ++fid) {
            if (reduced_model_.frames[fid].parentJoint == wrist_id) {
                return fid;
            }
        }
    }

    // Priority 3: Last frame in the model (often the terminal body frame)
    std::cerr << "[RobotModel] WARNING: Could not identify end-effector frame by name. "
              << "Using last frame as fallback." << std::endl;
    return reduced_model_.nframes - 1;
}

// Search the FULL model for the right-hand end-effector frame.
// Only matches RIGHT-side frames to avoid false positives on the left arm.
//
// 1. Exact match against known right-hand frame names (most specific first)
// 2. Case-insensitive substring match for frames containing both "right" and "hand"
// 3. Fallback: last frame in the full model
pinocchio::FrameIndex RobotModel::findEndEffectorFrameFull() const
{
    // Priority: search for RIGHT-side frames only, with more specific first
    // "rubber_hand" alone would match left_wrist_roll_rubber_hand before right!
    const std::vector<std::string> hints = {
        "right_rubber_hand",
        "right_wrist_roll_rubber_hand",
        "right_palm",
        "right_hand",
        "right_gripper",
    };

    for (const auto& hint : hints) {
        for (pinocchio::FrameIndex fid = 0; fid < full_model_.nframes; ++fid) {
            const std::string& fname = full_model_.frames[fid].name;
            if (fname == hint) {
                return fid;
            }
        }
    }

    // Fallback: case-insensitive match for right-side frames
    for (pinocchio::FrameIndex fid = 0; fid < full_model_.nframes; ++fid) {
        const std::string& fname = full_model_.frames[fid].name;
        std::string lower = fname;
        std::transform(lower.begin(), lower.end(), lower.begin(), ::tolower);
        if (lower.find("right") != std::string::npos &&
            lower.find("hand") != std::string::npos) {
            return fid;
        }
    }

    std::cerr << "[RobotModel] WARNING: Could not find RIGHT EE frame in full model."
              << " Using last frame as fallback." << std::endl;
    return full_model_.nframes - 1;
}

// ============================================================================
// MODULE: Kinematics Update
// ============================================================================

// Run forward kinematics and update frame placements on the REDUCED model.
// Must be called after any change to q_ before reading EE position.
void RobotModel::updateKinematics()
{
    pinocchio::forwardKinematics(reduced_model_, reduced_data_, q_);
    pinocchio::updateFramePlacements(reduced_model_, reduced_data_);
}

// Run forward kinematics on the FULL model (used when the full Jacobian is
// needed, e.g. for frame Jacobian via Pinocchio's analytical method).
void RobotModel::updateKinematicsFull()
{
    Eigen::VectorXd q_full(full_model_.nv);
    buildFullConfig(q_full);
    pinocchio::forwardKinematics(full_model_, full_data_, q_full);
    pinocchio::updateFramePlacements(full_model_, full_data_);
}

// Build a full-model configuration vector: start from the reference config
// and overwrite the active joint positions with the current reduced-model q_.
void RobotModel::buildFullConfig(Eigen::VectorXd& q_full) const
{
    // Start from neutral (locked joints at zero/neutral)
    q_full = q_ref_full_;

    // Overwrite active joints with current q_
    for (size_t i = 0; i < active_joint_ids_full_.size() && i < static_cast<size_t>(q_.size()); ++i) {
        int full_idx = static_cast<int>(full_model_.joints[active_joint_ids_full_[i]].idx_q());
        if (full_idx >= 0 && full_idx < q_full.size()) {
            q_full(full_idx) = q_(static_cast<int>(i));
        }
    }
}

// ============================================================================
// MODULE: Helpers
// ============================================================================

// Read joint limits from the reduced model (preserved from the URDF).
// Replace infinite bounds with reasonable defaults (2*pi rad pos, 1.5 rad/s vel).
void RobotModel::extractJointLimits()
{
    int nv = reduced_model_.nv;
    q_min_.resize(nv);
    q_max_.resize(nv);
    dq_max_.resize(nv);

    // The reduced model preserves the joint limits from the original URDF
    q_min_ = reduced_model_.lowerPositionLimit;
    q_max_ = reduced_model_.upperPositionLimit;
    dq_max_ = reduced_model_.velocityLimit;

    // If limits are infinite, set reasonable defaults
    const double default_pos_limit = 2.0 * M_PI;
    const double default_vel_limit = 1.5;  // rad/s
    for (int i = 0; i < nv; ++i) {
        if (q_min_(i) < -1e10) q_min_(i) = -default_pos_limit;
        if (q_max_(i) > 1e10)  q_max_(i) = default_pos_limit;
        if (dq_max_(i) < 0 || dq_max_(i) > 1e10) dq_max_(i) = default_vel_limit;
    }
}

// Return the 3-DOF end-effector position from the reduced model's last FK.
Eigen::Vector3d RobotModel::getEndEffectorPosition() const
{
    return reduced_data_.oMf[ee_frame_id_].translation();
}

// Return the full 6-DOF end-effector pose (SE3) from the reduced model.
pinocchio::SE3 RobotModel::getEndEffectorPose() const
{
    // Copy the SE3 object from oMf
    return pinocchio::SE3(reduced_data_.oMf[ee_frame_id_].rotation(),
                          reduced_data_.oMf[ee_frame_id_].translation());
}

// Reset the current configuration to the neutral (zero) pose.
void RobotModel::setNeutralConfig()
{
    q_ = pinocchio::neutral(reduced_model_);
    dq_.setZero();
}

// Clamp each joint position to its [q_min, q_max] range (hard safety).
void RobotModel::clampToLimits()
{
    q_ = q_.cwiseMax(q_min_).cwiseMin(q_max_);
}

// ============================================================================
// MODULE: Debug / Diagnostics
// ============================================================================

// Print a formatted summary of the model: DOF counts, active joint names
// with limits, EE frame name, and current EE position.
void RobotModel::printSummary() const
{
    std::cout << "\n========== G1 Right Arm Model Summary ==========\n";
    std::cout << "Full model:    " << full_model_.nq << " pos DOF, "
              << full_model_.nv << " vel DOF\n";
    std::cout << "Reduced model: " << reduced_model_.nq << " pos DOF, "
              << reduced_model_.nv << " vel DOF\n";
    std::cout << "\nActive joints:\n";
    for (int i = 0; i < reduced_model_.nv; ++i) {
        std::string jname = (i < static_cast<int>(active_joint_names_.size()))
                            ? active_joint_names_[i]
                            : reduced_model_.names[static_cast<size_t>(i) + 1];
        std::cout << "  [" << i << "] " << jname
                  << "  limits: [" << q_min_(i) << ", " << q_max_(i) << "] rad"
                  << "  vel_limit: " << dq_max_(i) << " rad/s\n";
    }
    std::cout << "\nEnd-effector frame: '" << reduced_model_.frames[ee_frame_id_].name
              << "' (ID=" << ee_frame_id_ << ")\n";
    std::cout << "Current EE position: " << getEndEffectorPosition().transpose() << "\n";
    std::cout << "=================================================\n\n";
}

}  // namespace g1_right_arm_reach
