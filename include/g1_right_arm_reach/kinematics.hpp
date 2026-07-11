/** @file kinematics.hpp
 *  Numerical position Jacobian, damped pseudoinverse, and Lie-group integration.
 */

#ifndef G1_RIGHT_ARM_REACH__KINEMATICS_HPP_
#define G1_RIGHT_ARM_REACH__KINEMATICS_HPP_

#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>

#include <Eigen/Dense>

#include "g1_right_arm_reach/robot_model.hpp"

namespace g1_right_arm_reach {

class Kinematics {
public:
    explicit Kinematics(RobotModel& robot);

    /// Compute 3×N position Jacobian via central-difference numerical diff.
    void computePositionJacobian(Eigen::Matrix<double, 3, Eigen::Dynamic>& J_pos);

    /// Lie-group integration: q ← integrate(q, dq * dt).
    void integrate(const Eigen::VectorXd& dq, double dt);

    /// Damped pseudoinverse: J⁺ = Jᵀ·(J·Jᵀ + λI)⁻¹.
    void dampedPseudoInverse(
        const Eigen::Matrix<double, 3, Eigen::Dynamic>& J_pos,
        double damping,
        Eigen::Matrix<double, Eigen::Dynamic, 3>& J_pinv) const;

    RobotModel& robot_;
};

}  // namespace g1_right_arm_reach

#endif  // G1_RIGHT_ARM_REACH__KINEMATICS_HPP_
