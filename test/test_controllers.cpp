#include "g1_right_arm_reach/kinematics.hpp"
#include "g1_right_arm_reach/pid_controller.hpp"
#include "g1_right_arm_reach/robot_model.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <cstdlib>
#include <memory>
#include <string>
#include <vector>

using namespace g1_right_arm_reach;

namespace {
constexpr double kControlDt = 0.02;  // Frozen 50 Hz high-level controller.
}

class ControllerTest : public ::testing::Test {
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
        controller_ = std::make_unique<PIDController>(
            *robot_, *kinematics_, 150.0, 0.0, 0.0, 1.0e-3, 0.005, 0.0);
        controller_->setRollGains(10.0, 0.0, 0.0);
        controller_->setGravityCompensation(false);
        controller_->setJointDamping(0.0);
        controller_->setTorqueSafety(1.0, 100.0, 10000.0);
        zero_ = kinematics_->getEEOrientation();
    }

    std::vector<std::string> joints_;
    std::unique_ptr<RobotModel> robot_;
    std::unique_ptr<Kinematics> kinematics_;
    std::unique_ptr<PIDController> controller_;
    Eigen::Quaterniond zero_;
};

TEST_F(ControllerTest, ProducesFiveFiniteJointTorques)
{
    const Eigen::Vector3d target = robot_->getEndEffectorPosition()
        + Eigen::Vector3d(0.01, -0.01, 0.01);
    const auto result = controller_->computeTask(
        target, 0.1, robot_->q_, robot_->dq_, kControlDt, zero_);
    EXPECT_EQ(result.torque_command.size(), 5);
    EXPECT_EQ(result.task_torque.size(), 5);
    EXPECT_TRUE(result.torque_command.allFinite());
    EXPECT_TRUE(result.task_torque.allFinite());
    EXPECT_GT(result.position_error, 0.0);
    EXPECT_NEAR(result.orientation_error, 0.1, 1.0e-6);
}

TEST_F(ControllerTest, ExactTaskIsReportedConverged)
{
    const auto result = controller_->computeTask(
        robot_->getEndEffectorPosition(), 0.0,
        robot_->q_, robot_->dq_, kControlDt, zero_);
    EXPECT_TRUE(result.success);
    EXPECT_NEAR(result.position_error, 0.0, 1.0e-9);
    EXPECT_NEAR(result.orientation_error, 0.0, 1.0e-9);
}

TEST_F(ControllerTest, RollCommandChangesTaskTorqueSign)
{
    const Eigen::Vector3d position = robot_->getEndEffectorPosition();
    controller_->reset();
    const auto positive = controller_->computeTask(
        position, 0.2, robot_->q_, robot_->dq_, kControlDt, zero_);
    controller_->reset();
    const auto negative = controller_->computeTask(
        position, -0.2, robot_->q_, robot_->dq_, kControlDt, zero_);
    EXPECT_LT(positive.task_torque.dot(negative.task_torque), 0.0);
}

TEST_F(ControllerTest, PositionTaskDoesNotTorqueWrist)
{
    const Eigen::Vector3d target = robot_->getEndEffectorPosition()
        + Eigen::Vector3d(0.02, -0.01, 0.015);
    const auto result = controller_->computeTask(
        target, 0.0, robot_->q_, robot_->dq_, kControlDt, zero_);
    const int wrist = robot_->armJointColumn("right_wrist_roll_joint");
    ASSERT_GE(wrist, 0);
    ASSERT_LT(wrist, result.task_torque.size());
    EXPECT_NEAR(result.task_torque(wrist), 0.0, 1.0e-10);
}

TEST_F(ControllerTest, RollTaskOnlyTorquesWrist)
{
    const auto result = controller_->computeTask(
        robot_->getEndEffectorPosition(), 0.2,
        robot_->q_, robot_->dq_, kControlDt, zero_);
    const int wrist = robot_->armJointColumn("right_wrist_roll_joint");
    ASSERT_GE(wrist, 0);
    for (int k = 0; k < result.task_torque.size(); ++k) {
        if (k == wrist) {
            EXPECT_GT(std::abs(result.task_torque(k)), 1.0e-6);
        } else {
            EXPECT_NEAR(result.task_torque(k), 0.0, 1.0e-10);
        }
    }
}

TEST_F(ControllerTest, JointDampingDoesNotBrakeCommandedTaskVelocity)
{
    controller_->setJointDamping(2.0);
    const int wrist = robot_->armJointColumn("right_wrist_roll_joint");
    ASSERT_GE(wrist, 0);

    Eigen::VectorXd arm_velocity = Eigen::VectorXd::Zero(joints_.size());
    arm_velocity(wrist) = 0.5;
    Eigen::Matrix<double, 4, Eigen::Dynamic> jacobian;
    kinematics_->computeTaskJacobian(jacobian);
    const Eigen::Matrix<double, 4, 1> matching_reference_velocity =
        jacobian * arm_velocity;

    Eigen::VectorXd dq = robot_->dq_;
    dq(robot_->coordinateIndex("right_wrist_roll_joint")) = arm_velocity(wrist);
    const auto result = controller_->computeTask(
        robot_->getEndEffectorPosition(), 0.0,
        robot_->q_, dq, kControlDt, zero_, matching_reference_velocity);

    // With zero tracking error and a matching moving reference, the task-space
    // derivative is zero. Joint damping must not add a counter-torque in the
    // controlled roll direction.
    EXPECT_NEAR(result.torque_command.norm(), 0.0, 1.0e-8);
}

TEST_F(ControllerTest, MovingReferenceVelocityProducesFeedforwardTorque)
{
    // The reference-velocity feedforward flows through the D term
    // (wrench += kd * (v_ref - v_task)). The shared SetUp leaves kd at 0, which
    // would make this test vacuous; give it a nonzero gain so a moving
    // reference actually produces task torque.
    controller_->setKd(5.0);
    const Eigen::Vector3d position = robot_->getEndEffectorPosition();
    controller_->reset();
    const auto stationary = controller_->computeTask(
        position, 0.0, robot_->q_, robot_->dq_, kControlDt, zero_);

    Eigen::Matrix<double, 4, 1> reference_velocity;
    reference_velocity << 0.10, -0.08, 0.06, 0.20;
    controller_->reset();
    const auto moving = controller_->computeTask(
        position, 0.0, robot_->q_, robot_->dq_, kControlDt, zero_,
        reference_velocity);

    EXPECT_GT(moving.task_torque.norm(), stationary.task_torque.norm() + 1.0e-6);
}

TEST_F(ControllerTest, TorqueSafetyAppliesDeratedUrdfLimits)
{
    controller_->setWrenchLimits(10000.0, 10000.0);
    controller_->setTorqueSafety(0.10, 3.0, 10000.0);
    const auto result = controller_->computeTask(
        robot_->getEndEffectorPosition() + Eigen::Vector3d(1.0, 1.0, 1.0),
        3.0, robot_->q_, robot_->dq_, kControlDt, zero_);
    for (int k = 0; k < result.torque_command.size(); ++k) {
        const int index = robot_->arm_joint_indices_[k];
        const double expected_limit = std::min(3.0, 0.10 * robot_->tau_max_(index));
        EXPECT_LE(std::abs(result.torque_command(k)), expected_limit + 1.0e-9);
    }
}

TEST_F(ControllerTest, ResetMakesFirstOutputDeterministic)
{
    const Eigen::Vector3d target = robot_->getEndEffectorPosition()
        + Eigen::Vector3d(0.02, 0.0, 0.0);
    controller_->reset();
    const auto first = controller_->computeTask(
        target, 0.1, robot_->q_, robot_->dq_, kControlDt, zero_);
    controller_->reset();
    const auto second = controller_->computeTask(
        target, 0.1, robot_->q_, robot_->dq_, kControlDt, zero_);
    EXPECT_TRUE(first.torque_command.isApprox(second.torque_command, 1.0e-12));
}
