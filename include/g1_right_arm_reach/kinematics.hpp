/** @file kinematics.hpp
 *  @brief Jacobian computation, damped pseudoinverse, null-space projection,
 *         and Lie-group integration for the G1 right arm's reduced model.
 *
 *  Kinematics operates on the RobotModel's reduced 5-DOF representation and
 *  provides the linear-algebra building blocks used by PIDController.
 */

#ifndef G1_RIGHT_ARM_REACH__KINEMATICS_HPP_
#define G1_RIGHT_ARM_REACH__KINEMATICS_HPP_

#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/spatial/se3.hpp>

#include <Eigen/Dense>

#include "g1_right_arm_reach/robot_model.hpp"

// ============================================================================

namespace g1_right_arm_reach {

// ============================================================================
// MODULE: Kinematics — Jacobian, pseudoinverse, null-space, integration
// ============================================================================

/**
 * @brief Kinematics utilities for the G1 right arm's reduced model.
 *
 * Provides Jacobian computation (position and full), damped least-squares
 * pseudoinverse, null-space projection for redundancy resolution, and
 * Lie-group configuration integration.  All methods operate on the 5-DOF
 * reduced model referenced through robot_.
 */
class Kinematics {
public:
    // ------------------------------------------------------------------
    // Construction
    // ------------------------------------------------------------------

    /**
     * @brief Construct kinematics helper bound to a RobotModel.
     * @param robot Reference to the RobotModel whose reduced model is used.
     */
    explicit Kinematics(RobotModel& robot);

    // ------------------------------------------------------------------
    // Jacobian computation
    // ------------------------------------------------------------------

    /**
     * @brief Compute the 3xN translational Jacobian at the end-effector.
     *
     * The Jacobian maps joint velocities (dq) to end-effector translational
     * velocity (v_ee) in LOCAL_WORLD_ALIGNED frame:
     * @f$ v_{ee} = J_{pos} \cdot dq @f$
     *
     * @param[out] J_pos  3 × nv translational Jacobian matrix.
     *                    Must be pre-allocated with correct dimensions.
     * @pre updateKinematics() must have been called on the robot model.
     */
    void computePositionJacobian(Eigen::Matrix<double, 3, Eigen::Dynamic>& J_pos);

    /**
     * @brief Compute the full 6xN Jacobian at the end-effector.
     *
     * @param[out] J_full  6 × nv Jacobian (3 translational + 3 rotational rows).
     *                     Must be pre-allocated and ZERO-INITIALIZED.
     * @param ref_frame    Reference frame for the Jacobian expression.
     * @pre updateKinematics() must have been called on the robot model.
     */
    void computeFullJacobian(
        Eigen::Matrix<double, 6, Eigen::Dynamic>& J_full,
        pinocchio::ReferenceFrame ref_frame = pinocchio::LOCAL_WORLD_ALIGNED);

    // ------------------------------------------------------------------
    // Configuration integration
    // ------------------------------------------------------------------

    /**
     * @brief Integrate the current configuration with a velocity step.
     *
     * Uses Pinocchio's Lie-group integration.  NEVER use q += dq * dt for
     * spherical joints — this method handles the manifold correctly.
     *
     * @param dq  Joint velocity vector (size nv).
     * @param dt  Time step in seconds.
     */
    void integrate(const Eigen::VectorXd& dq, double dt);

    // ------------------------------------------------------------------
    // Pseudoinverse and null-space
    // ------------------------------------------------------------------

    /**
     * @brief Compute damped least-squares pseudoinverse of the position Jacobian.
     *
     * @f$ J^{+} = J^{T} \cdot (J \cdot J^{T} + \lambda I)^{-1} @f$
     *
     * @param J_pos      3×N position Jacobian.
     * @param damping    Damping factor λ.
     * @param[out] J_pinv  N×3 pseudoinverse.
     */
    void dampedPseudoInverse(
        const Eigen::Matrix<double, 3, Eigen::Dynamic>& J_pos,
        double damping,
        Eigen::Matrix<double, Eigen::Dynamic, 3>& J_pinv) const;

    /**
     * @brief Compute null-space projector for redundancy resolution.
     *
     * @f$ N = I - J^{+} \cdot J @f$
     *
     * @param J_pos    3×N position Jacobian.
     * @param damping  Damping factor λ.
     * @param[out] N   N×N null-space projector.
     */
    void nullSpaceProjector(
        const Eigen::Matrix<double, 3, Eigen::Dynamic>& J_pos,
        double damping,
        Eigen::MatrixXd& N) const;

    // ------------------------------------------------------------------
    // Data members
    // ------------------------------------------------------------------

    RobotModel& robot_;  ///< Reference to the reduced robot model
};

}  // namespace g1_right_arm_reach

#endif  // G1_RIGHT_ARM_REACH__KINEMATICS_HPP_
