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

    /**
     * @brief Update forward kinematics and frame placements.
     *
     * Must be called after modifying q_ before reading end-effector position
     * or computing Jacobians.
     */
    void updateKinematics();

    /**
     * @brief Get end-effector position in world frame.
     * @return 3D position vector (x, y, z).
     * @pre updateKinematics() must have been called.
     */
    Eigen::Vector3d getEndEffectorPosition() const;

    /**
     * @brief Get the full SE(3) pose of the end-effector.
     * @pre updateKinematics() must have been called.
     */
    pinocchio::SE3 getEndEffectorPose() const;

    /**
     * @brief Initialize q_ to the neutral (zero) configuration.
     */
    void setNeutralConfig();

    /**
     * @brief Clamp the current configuration q_ to the joint limits.
     */
    void clampToLimits();

    /**
     * @brief Print model summary (joint names, DOF, limits) to stdout.
     */
    void printSummary() const;

    /// Build the full 23-DOF configuration from the 5-DOF q_ plus locked joints
    /// set to their neutral values.
    void buildFullConfig(Eigen::VectorXd& q_full) const;

    /// Update FK on the FULL model (used internally for Jacobian computation).
    void updateKinematicsFull();

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

    /// End-effector frame ID in the REDUCED model index (for FK display).
    pinocchio::FrameIndex ee_frame_id_;

    /// End-effector frame ID in the FULL model index (for Jacobian).
    pinocchio::FrameIndex ee_frame_id_full_;

    /// Indices of the 5 active joints in the FULL model.
    std::vector<pinocchio::JointIndex> active_joint_ids_full_;

    /// Indices of the locked joints in the FULL model.
    std::vector<pinocchio::JointIndex> locked_joint_ids_full_;

    Eigen::VectorXd q_min_;   ///< Joint position limits (lower, 5-DOF)
    Eigen::VectorXd q_max_;   ///< Joint position limits (upper, 5-DOF)
    Eigen::VectorXd dq_max_;  ///< Joint velocity limits (5-DOF)

    /// Active joint names in reduced-model ordering.
    std::vector<std::string> active_joint_names_;

    Eigen::VectorXd q_;        ///< Current configuration (5-DOF, reduced ordering)
    Eigen::VectorXd dq_;       ///< Current joint velocities (5-DOF)

    /// Full-model reference configuration (size = full_model_.nq).
    /// All locked joints are frozen at these values.
    Eigen::VectorXd q_ref_full_;

private:
    // ------------------------------------------------------------------
    // Private helpers
    // ------------------------------------------------------------------

    /**
     * @brief Identify joints to lock (all except the 5 right-arm joints).
     */
    std::vector<pinocchio::JointIndex> identifyJointsToLock(
        const std::vector<std::string>& active_names) const;

    /**
     * @brief Find the end-effector frame in the reduced model.
     *
     * Searches frame names for "rubber_hand", "palm", "hand", or uses the
     * last body in the arm kinematic chain as a fallback.
     */
    pinocchio::FrameIndex findEndEffectorFrame() const;

    /**
     * @brief Find the end-effector frame in the FULL model.
     */
    pinocchio::FrameIndex findEndEffectorFrameFull() const;

    /**
     * @brief Extract joint limits from the URDF model.
     */
    void extractJointLimits();
};

}  // namespace g1_right_arm_reach

#endif  // G1_RIGHT_ARM_REACH__ROBOT_MODEL_HPP_
