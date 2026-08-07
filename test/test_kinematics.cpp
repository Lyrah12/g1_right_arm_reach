#include "g1_right_arm_reach/kinematics.hpp"
#include "g1_right_arm_reach/robot_model.hpp"

#include <gtest/gtest.h>

#include <Eigen/Geometry>

#include <cstdlib>
#include <memory>
#include <string>
#include <vector>

using namespace g1_right_arm_reach;

class KinematicsTest : public ::testing::Test {
protected:
    void SetUp() override
    {
        const char* path = std::getenv("G1_TEST_URDF_PATH");
        if (!path) GTEST_SKIP() << "Set G1_TEST_URDF_PATH to g1_23dof_revo2.urdf";
        joints_ = {"right_shoulder_pitch_joint", "right_shoulder_roll_joint",
                   "right_shoulder_yaw_joint", "right_elbow_joint",
                   "right_wrist_roll_joint"};
        robot_ = std::make_unique<RobotModel>(path, joints_);
        robot_->buildSubsetIndices(joints_);
        kinematics_ = std::make_unique<Kinematics>(*robot_);
    }

    std::vector<std::string> joints_;
    std::unique_ptr<RobotModel> robot_;
    std::unique_ptr<Kinematics> kinematics_;
};

TEST_F(KinematicsTest, ReducedModelIsFiveDof)
{
    EXPECT_EQ(robot_->nq(), 5);
    EXPECT_EQ(robot_->nv(), 5);
    EXPECT_EQ(robot_->arm_joint_indices_.size(), 5U);
    EXPECT_TRUE(robot_->getEndEffectorPosition().allFinite());
}

TEST_F(KinematicsTest, FourDimensionalTaskJacobianMatchesPositionDifference)
{
    Eigen::Matrix<double, 4, Eigen::Dynamic> jacobian;
    kinematics_->computeTaskJacobian(jacobian);
    ASSERT_EQ(jacobian.rows(), 4);
    ASSERT_EQ(jacobian.cols(), 5);
    EXPECT_TRUE(jacobian.allFinite());

    const Eigen::VectorXd initial = robot_->q_;
    const Eigen::Vector3d initial_position = robot_->getEndEffectorPosition();
    constexpr double epsilon = 1.0e-6;
    robot_->q_(0) += epsilon;
    robot_->updateKinematics();
    const Eigen::Vector3d finite_difference =
        (robot_->getEndEffectorPosition() - initial_position) / epsilon;
    EXPECT_TRUE(finite_difference.isApprox(jacobian.block<3, 1>(0, 0), 2.0e-4));
    robot_->q_ = initial;
    robot_->updateKinematics();
}

TEST_F(KinematicsTest, WristRollAffectsToolRollRow)
{
    Eigen::Matrix<double, 4, Eigen::Dynamic> jacobian;
    kinematics_->computeTaskJacobian(jacobian);
    const int wrist_column = 4;
    EXPECT_GT(std::abs(jacobian(3, wrist_column)), 0.5);
}

TEST(KinematicsMathTest, ToolRollUsesCalibratedLocalZAxis)
{
    const Eigen::Quaterniond zero = Eigen::Quaterniond::Identity();
    const Eigen::Quaterniond current(
        Eigen::AngleAxisd(0.7, Eigen::Vector3d::UnitZ()));
    EXPECT_NEAR(Kinematics::toolRoll(zero, current), 0.7, 1.0e-9);
    EXPECT_NEAR(Kinematics::toolRoll(current, current), 0.0, 1.0e-9);
}

TEST_F(KinematicsTest, JointAndEffortLimitsAreFinite)
{
    EXPECT_EQ(robot_->tau_max_.size(), 5);
    for (int i = 0; i < 5; ++i) {
        EXPECT_LT(robot_->q_min_(i), robot_->q_max_(i));
        EXPECT_GT(robot_->dq_max_(i), 0.0);
        EXPECT_GT(robot_->tau_max_(i), 0.0);
    }
}
