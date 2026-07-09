/**
 * @file test_kinematics.cpp
 * @brief Unit tests for FK and Jacobian correctness.
 *
 * Tests:
 *   - FK returns consistent positions for known configurations
 *   - Jacobian maps joint velocity to end-effector velocity correctly
 *   - Integration preserves joint limits
 *   - Damped pseudoinverse satisfies J * J^+ ≈ I
 */

#include <gtest/gtest.h>

#include "g1_right_arm_reach/robot_model.hpp"
#include "g1_right_arm_reach/kinematics.hpp"

#include <pinocchio/algorithm/joint-configuration.hpp>

#include <Eigen/Dense>
#include <string>
#include <vector>

using namespace g1_right_arm_reach;

// ============================================================================
// MODULE: Test Fixture — SetUp, member vars
// ============================================================================
class KinematicsTest : public ::testing::Test {
protected:
    void SetUp() override {
        // NOTE: Set G1_TEST_URDF_PATH environment variable before running tests
        const char* urdf_env = std::getenv("G1_TEST_URDF_PATH");
        if (!urdf_env) {
            GTEST_SKIP() << "G1_TEST_URDF_PATH environment variable not set. "
                         << "Set it to the G1 URDF file path to run kinematics tests.";
        }
        urdf_path_ = urdf_env;

        active_joints_ = {
            "right_shoulder_pitch_joint",
            "right_shoulder_roll_joint",
            "right_shoulder_yaw_joint",
            "right_elbow_joint",
            "right_wrist_roll_joint"
        };

        robot_ = std::make_unique<RobotModel>(urdf_path_, active_joints_);
        kin_ = std::make_unique<Kinematics>(*robot_);
    }

    std::string urdf_path_;
    std::vector<std::string> active_joints_;
    std::unique_ptr<RobotModel> robot_;
    std::unique_ptr<Kinematics> kin_;
};

// ============================================================================
// MODULE: Forward Kinematics Tests — FK consistency and FK changes
// ============================================================================

TEST_F(KinematicsTest, ForwardKinematicsConsistency) {
    // Neutral config → FK should give a valid position
    robot_->setNeutralConfig();
    robot_->updateKinematics();
    Eigen::Vector3d pos_neutral = robot_->getEndEffectorPosition();

    // All values should be finite
    EXPECT_TRUE(pos_neutral.allFinite());

    // Position should be reasonable (not at origin, not NaN)
    double norm = pos_neutral.norm();
    EXPECT_GT(norm, 0.01);  // arm has non-zero length
    EXPECT_LT(norm, 2.0);   // arm span < 2m
}

TEST_F(KinematicsTest, ForwardKinematicsChanges) {
    robot_->setNeutralConfig();
    robot_->updateKinematics();
    Eigen::Vector3d pos1 = robot_->getEndEffectorPosition();

    // Move shoulder pitch by 30 degrees
    robot_->q_(0) += 0.5236;  // 30 degrees in radians
    robot_->updateKinematics();
    Eigen::Vector3d pos2 = robot_->getEndEffectorPosition();

    // Position should change
    double dist = (pos2 - pos1).norm();
    EXPECT_GT(dist, 0.01);  // meaningful change
}

// ============================================================================
// MODULE: Jacobian Tests — velocity mapping
// ============================================================================

TEST_F(KinematicsTest, JacobianVelocityMapping) {
    robot_->setNeutralConfig();
    robot_->updateKinematics();

    const int nv = robot_->reduced_model_.nv;

    // Compute position Jacobian
    Eigen::Matrix<double, 3, Eigen::Dynamic> J_pos(3, nv);
    kin_->computePositionJacobian(J_pos);

    // J * dq should equal the frame velocity
    Eigen::VectorXd dq = Eigen::VectorXd::Zero(nv);
    dq(0) = 1.0;  // move only shoulder pitch

    // Use finite difference to verify
    double eps = 1e-6;
    Eigen::VectorXd q_plus = robot_->q_ + dq * eps;
    robot_->q_ = q_plus;
    robot_->updateKinematics();
    Eigen::Vector3d pos_plus = robot_->getEndEffectorPosition();

    // Reset
    robot_->setNeutralConfig();
    robot_->updateKinematics();
    Eigen::Vector3d pos = robot_->getEndEffectorPosition();

    // Finite-difference velocity
    Eigen::Vector3d vel_fd = (pos_plus - pos) / eps;

    // J * dq predicted velocity
    Eigen::Vector3d vel_jacobian = J_pos * dq;

    // Should be close (within 1%)
    EXPECT_NEAR(vel_fd.x(), vel_jacobian.x(), std::abs(vel_jacobian.x()) * 0.01 + 1e-4);
    EXPECT_NEAR(vel_fd.y(), vel_jacobian.y(), std::abs(vel_jacobian.y()) * 0.01 + 1e-4);
    EXPECT_NEAR(vel_fd.z(), vel_jacobian.z(), std::abs(vel_jacobian.z()) * 0.01 + 1e-4);
}

// ============================================================================
// MODULE: Joint Limits & Integration Tests
// ============================================================================

TEST_F(KinematicsTest, JointLimitsPopulated) {
    const int nv = robot_->reduced_model_.nv;
    EXPECT_EQ(robot_->q_min_.size(), nv);
    EXPECT_EQ(robot_->q_max_.size(), nv);

    for (int i = 0; i < nv; ++i) {
        EXPECT_LT(robot_->q_min_(i), robot_->q_max_(i))
            << "Joint " << i << " has invalid limits";
        EXPECT_GT(robot_->q_max_(i) - robot_->q_min_(i), 0.01)
            << "Joint " << i << " range too small";
    }
}

TEST_F(KinematicsTest, LieGroupIntegration) {
    robot_->setNeutralConfig();
    Eigen::VectorXd q_before = robot_->q_;

    Eigen::VectorXd dq = Eigen::VectorXd::Zero(robot_->reduced_model_.nv);
    dq(0) = 0.5;  // move shoulder pitch at 0.5 rad/s
    double dt = 0.1;

    kin_->integrate(dq, dt);

    // q should have changed
    double change = (robot_->q_ - q_before).norm();
    EXPECT_GT(change, 0.01);

    // q should still be within limits
    for (int i = 0; i < robot_->reduced_model_.nv; ++i) {
        EXPECT_GE(robot_->q_(i), robot_->q_min_(i) - 1e-6);
        EXPECT_LE(robot_->q_(i), robot_->q_max_(i) + 1e-6);
    }
}

// ============================================================================
// MODULE: Pseudoinverse Tests
// ============================================================================

TEST_F(KinematicsTest, DampedPseudoInverseProperties) {
    robot_->setNeutralConfig();
    robot_->updateKinematics();

    const int nv = robot_->reduced_model_.nv;
    Eigen::Matrix<double, 3, Eigen::Dynamic> J_pos(3, nv);
    kin_->computePositionJacobian(J_pos);

    // Compute pseudoinverse
    Eigen::Matrix<double, Eigen::Dynamic, 3> J_pinv(nv, 3);
    kin_->dampedPseudoInverse(J_pos, 1e-4, J_pinv);

    // J * J^+ should be close to identity (3x3)
    Eigen::Matrix3d J_Jpinv = J_pos * J_pinv;
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            double expected = (i == j) ? 1.0 : 0.0;
            EXPECT_NEAR(J_Jpinv(i, j), expected, 0.1)
                << "J * J^+ should approximate identity";
        }
    }
}

// ============================================================================
// MODULE: Main
// ============================================================================
int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
