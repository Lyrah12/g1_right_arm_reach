/** @file kinematics.hpp
 *  @brief Numerical Jacobians, damped pseudoinverse, Lie-group integration.
 *
 *  Provides subset Jacobians for Arm-Hand decoupling:
 *    computeArmPositionJacobian()      — 3×4 position Jacobian (wrist origin × arm joints)
 *    computeWristOrientationJacobian() — 3×N orientation Jacobian (hand × wrist joints)
 *
 *  The subset Jacobians perturb only the relevant joints, enabling independent
 *  position and orientation control loops.
 */

#ifndef G1_RIGHT_ARM_REACH__KINEMATICS_HPP_
#define G1_RIGHT_ARM_REACH__KINEMATICS_HPP_

#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "g1_right_arm_reach/robot_model.hpp"

namespace g1_right_arm_reach {

class Kinematics {
public:
    explicit Kinematics(RobotModel& robot);

    // --- Subset Jacobians (Arm-Hand decoupling) ---

    /// 3×4 position Jacobian: wrist origin w.r.t. arm joints only (perturbs arm_joint_indices_).
    void computeArmPositionJacobian(Eigen::Matrix<double, 3, Eigen::Dynamic>& J_arm);

    /// 3×N orientation Jacobian: hand frame angular velocity w.r.t. wrist joints only.
    /// N = wrist_joint_indices_.size() (3 for the 5010 wrist).
    void computeWristOrientationJacobian(Eigen::Matrix<double, 3, Eigen::Dynamic>& J_wrist);

    /// 3×N position Jacobian (all active joints).
    void computePositionJacobian(Eigen::Matrix<double, 3, Eigen::Dynamic>& J_pos);

    // --- Utilities ---

    /// Gravity torque τ_g = g(q) via Pinocchio RNEA.
    void computeGravityTorque(Eigen::VectorXd& tau_g);

    /// Orientation error as axis-angle: log(q_des * q_curr⁻¹). Zero when aligned.
    static Eigen::Vector3d orientationError(const Eigen::Quaterniond& q_des,
                                            const Eigen::Quaterniond& q_curr);

    /// Lie-group integration: q ← integrate(q, dq·dt).
    void integrate(const Eigen::VectorXd& dq, double dt);

    /// Damped pseudoinverse (3×N): J⁺ = Jᵀ·(J·Jᵀ + λI)⁻¹.
    void dampedPseudoInverse(
        const Eigen::Matrix<double, 3, Eigen::Dynamic>& J_pos, double damping,
        Eigen::Matrix<double, Eigen::Dynamic, 3>& J_pinv) const;

    Eigen::Quaterniond getEEOrientation() const;
    Eigen::Vector3d getRightShoulderPosition() const;

    RobotModel& robot_;
};

}  // namespace g1_right_arm_reach

#endif  // G1_RIGHT_ARM_REACH__KINEMATICS_HPP_
