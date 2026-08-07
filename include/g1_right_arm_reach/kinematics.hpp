#ifndef G1_RIGHT_ARM_REACH__KINEMATICS_HPP_
#define G1_RIGHT_ARM_REACH__KINEMATICS_HPP_

#include "g1_right_arm_reach/robot_model.hpp"

#include <Eigen/Dense>
#include <Eigen/Geometry>

namespace g1_right_arm_reach {

/** Numerical task kinematics for [TCP x,y,z, tool-axis roll]. */
class Kinematics {
public:
    explicit Kinematics(
        RobotModel& robot,
        const Eigen::Vector3d& tool_axis_local = Eigen::Vector3d::UnitZ());

    void computeTaskJacobian(Eigen::Matrix<double, 4, Eigen::Dynamic>& jacobian);
    void computePositionJacobian(Eigen::Matrix<double, 3, Eigen::Dynamic>& jacobian);
    void computeArmPositionJacobian(Eigen::Matrix<double, 3, Eigen::Dynamic>& jacobian);
    void computeGravityTorque(Eigen::VectorXd& tau_g);

    Eigen::Quaterniond getEEOrientation() const;
    Eigen::Vector3d getRightShoulderPosition() const;

    /** TCP twist around its calibrated local tool axis, wrapped to [-pi, pi]. */
    static double toolRoll(const Eigen::Quaterniond& zero_orientation,
                           const Eigen::Quaterniond& current_orientation,
                           const Eigen::Vector3d& tool_axis_local = Eigen::Vector3d::UnitZ());
    static double wrapAngle(double angle);
    static Eigen::Vector3d orientationError(const Eigen::Quaterniond& desired,
                                            const Eigen::Quaterniond& current);

    void integrate(const Eigen::VectorXd& dq, double dt);
    void dampedPseudoInverse(
        const Eigen::Matrix<double, 3, Eigen::Dynamic>& jacobian, double damping,
        Eigen::Matrix<double, Eigen::Dynamic, 3>& pseudo_inverse) const;
    const Eigen::Vector3d& toolAxisLocal() const { return tool_axis_local_; }

    RobotModel& robot_;

private:
    Eigen::Vector3d tool_axis_local_{Eigen::Vector3d::UnitZ()};
};

}  // namespace g1_right_arm_reach

#endif
