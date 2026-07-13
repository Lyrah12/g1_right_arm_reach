/** @file robot_model.hpp
 *  @brief Loads the G1 URDF, builds a reduced 5-DOF right-arm model, and
 *         provides forward kinematics and joint-limit queries.
 *
 *  RobotModel is the single source of truth for kinematic structure.  All
 *  other modules (Kinematics, PIDController) hold references to it.
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
 * Loads the full 23-DOF G1 URDF, then builds a reduced model with only the
 * right arm's 5 active joints.  All other joints (left arm, legs, waist) are
 * locked at their reference configuration and replaced by FIXED_JOINT frames
 * in the reduced model.
 */
class RobotModel {
public:
    // ------------------------------------------------------------------
    // Construction
    // ------------------------------------------------------------------

    /**
     * @brief Construct and load the robot model from a URDF file.
     * @param urdf_path          Absolute path to the G1 URDF file.
     * @param active_joint_names Names of the 5 right-arm joints to keep active.
     * @param q_ref_full         Optional full-model reference configuration.
     *                           If size 0, uses pinocchio::neutral(full_model_)
     *                           (all zeros).
     */
    RobotModel(const std::string& urdf_path,
               const std::vector<std::string>& active_joint_names,
               const Eigen::VectorXd& q_ref_full = Eigen::VectorXd());

    // ------------------------------------------------------------------
    // Public interface — kinematics and state
    // ------------------------------------------------------------------

    /// Update FK and frame placements on the reduced model.
    /// Must be called after modifying q_ and before reading EE position.
    void updateKinematics();

    /// Get end-effector position (xyz) from the reduced model.
    Eigen::Vector3d getEndEffectorPosition() const;

    /// Reset q_ to neutral and zero dq_.
    void setNeutralConfig();

    /// Clamp each joint in q_ to [q_min, q_max].
    void clampToLimits();

    /// Print model summary to stdout.
    void printSummary() const;


    // ------------------------------------------------------------------
    // Getters
    // ------------------------------------------------------------------

    /// Number of active DOF in the reduced model.
    int nq() const { return reduced_model_.nq; }
    int nv() const { return reduced_model_.nv; }

    // ------------------------------------------------------------------
    // Data members (public, read-only for cooperating classes)
    // ------------------------------------------------------------------

    pinocchio::Model full_model_;        ///< Full 23-DOF model
    pinocchio::Data   full_data_;        ///< Full-model data buffer

    pinocchio::Model reduced_model_;     ///< Reduced 5-DOF right-arm model
    pinocchio::Data   reduced_data_;     ///< Reduced-model data buffer

    pinocchio::FrameIndex ee_frame_id_;       ///< EE frame in reduced model
    pinocchio::FrameIndex ee_frame_id_full_;  ///< EE frame in full model

    std::vector<pinocchio::JointIndex> active_joint_ids_full_;  ///< Active joints in full model
    std::vector<pinocchio::JointIndex> locked_joint_ids_full_;  ///< Locked joints in full model

    Eigen::VectorXd q_min_;   ///< Position lower limits (5-DOF)
    Eigen::VectorXd q_max_;   ///< Position upper limits (5-DOF)
    Eigen::VectorXd dq_max_;  ///< Velocity limits (5-DOF)

    std::vector<std::string> active_joint_names_;  ///< Active joint names (reduced ordering)

    Eigen::VectorXd q_;        ///< Current config (5-DOF)
    Eigen::VectorXd dq_;       ///< Current velocities (5-DOF)

    Eigen::VectorXd q_ref_full_;     ///< Full-model reference config for buildReducedModel (23-DOF)

    // NOTE: Full-model FK (q_full_actual_, updateKinematicsFullActual, etc.) was removed.

private:
    /// Identify which full-model joints to lock (all except active ones).
    std::vector<pinocchio::JointIndex> identifyJointsToLock(
        const std::vector<std::string>& active_names) const;

    /// Find right-hand EE frame in the given model (works for both full & reduced).
    pinocchio::FrameIndex findEndEffectorFrame(const pinocchio::Model& model) const;

    /// Extract joint limits from reduced model.
    void extractJointLimits();
};

}  // namespace g1_right_arm_reach

#endif  // G1_RIGHT_ARM_REACH__ROBOT_MODEL_HPP_
