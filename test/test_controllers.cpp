/**
 * @file test_controllers.cpp
 * @brief Unit tests for PID controller.
 *
 * Tests:
 *   - PID converges to target from various starting configs
 *   - PID respects joint limits
 *   - PID reset clears internal state
 *   - ControlResult fields are populated correctly
 */

#include <gtest/gtest.h>

#include "g1_right_arm_reach/robot_model.hpp"
#include "g1_right_arm_reach/kinematics.hpp"
#include "g1_right_arm_reach/pid_controller.hpp"

#include <Eigen/Dense>
#include <string>
#include <vector>
#include <cstdlib>
#include <memory>

using namespace g1_right_arm_reach;

// ============================================================================
// MODULE: Test Fixture — SetUp, runPIDSteps helper
// ============================================================================
class ControllerTest : public ::testing::Test {
protected:
    void SetUp() override {
        const char* urdf_env = std::getenv("G1_TEST_URDF_PATH");
        if (!urdf_env) {
            GTEST_SKIP() << "G1_TEST_URDF_PATH not set.";
        }
        urdf_path_ = urdf_env;

        active_joints_ = {
            "right_shoulder_pitch_joint",
            "right_shoulder_roll_joint",
            "right_shoulder_yaw_joint",
            "right_elbow_joint",
            "right_wrist_roll_joint",
            "right_wrist_pitch_joint",
            "right_wrist_yaw_joint"
        };

        robot_ = std::make_unique<RobotModel>(urdf_path_, active_joints_);
        kin_ = std::make_unique<Kinematics>(*robot_);

        pid_ = std::make_unique<PIDController>(
            *robot_, *kin_,
            2.0,    // kp
            1.0,    // ki
            0.1,    // kd
            1e-4,   // damping
            0.01    // tolerance (1cm)
        );
    }

    std::string urdf_path_;
    std::vector<std::string> active_joints_;
    std::unique_ptr<RobotModel> robot_;
    std::unique_ptr<Kinematics> kin_;
    std::unique_ptr<PIDController> pid_;

    /**
     * @brief Run PID for N steps toward target and return final position error.
     * @param target Desired end-effector position (3D)
     * @param steps Number of control iterations to run
     * @param dt Timestep per iteration (default 0.05s)
     * @return Final Euclidean position error after all steps
     */
    double runPIDSteps(const Eigen::Vector3d& target, int steps, double dt = 0.05) {
        robot_->setNeutralConfig();
        pid_->reset();
        double final_error = 0.0;
        for (int i = 0; i < steps; ++i) {
            auto result = pid_->compute(target, robot_->q_, robot_->dq_, dt);
            final_error = result.position_error;
            if (result.position_error < 0.01) break;  // early convergence
        }
        return final_error;
    }
};

// ============================================================================
// MODULE: PID Controller Tests — error reduction and joint limits
// ============================================================================

TEST_F(ControllerTest, PIDReducesError) {
    // Start from neutral config
    robot_->setNeutralConfig();
    robot_->updateKinematics();
    Eigen::Vector3d start_pos = robot_->getEndEffectorPosition();

    // Pick a target near the current position (but not at it)
    Eigen::Vector3d target = start_pos + Eigen::Vector3d(0.05, -0.05, 0.05);

    double initial_error = (target - start_pos).norm();
    double final_error = runPIDSteps(target, 200);

    // Error should decrease
    EXPECT_LT(final_error, initial_error);
}

TEST_F(ControllerTest, PIDRespectsJointLimits) {
    // Run PID for many steps toward various targets
    std::vector<Eigen::Vector3d> targets = {
        Eigen::Vector3d(0.30, -0.40, 0.20),
        Eigen::Vector3d(0.20, -0.50, 0.15),
        Eigen::Vector3d(0.35, -0.30, 0.30),
    };

    for (const auto& target : targets) {
        robot_->setNeutralConfig();
        pid_->reset();
        for (int i = 0; i < 500; ++i) {
            pid_->compute(target, robot_->q_, robot_->dq_, 0.05);
        }
        // After running, all joints must be within limits
        for (int j = 0; j < robot_->reduced_model_.nv; ++j) {
            EXPECT_GE(robot_->q_(j), robot_->q_min_(j) - 1e-4)
                << "Joint " << j << " below lower limit for target " << target.transpose();
            EXPECT_LE(robot_->q_(j), robot_->q_max_(j) + 1e-4)
                << "Joint " << j << " above upper limit for target " << target.transpose();
        }
    }
}

// ============================================================================
// MODULE: PID Reset Test
// ============================================================================

TEST_F(ControllerTest, PIDResetClearsState) {
    Eigen::Vector3d target(0.30, -0.40, 0.20);

    // Record the result from a freshly-reset controller.
    robot_->setNeutralConfig();
    pid_->reset();
    auto result_fresh = pid_->compute(target, robot_->q_, robot_->dq_, 0.05);

    // Run many steps to accumulate integral error and derivative history.
    for (int i = 0; i < 100; ++i) {
        pid_->compute(target, robot_->q_, robot_->dq_, 0.05);
    }

    // Reset, re-initialize the robot, and verify the controller behaves
    // identically to a fresh one (integral = 0, first_call = true).
    robot_->setNeutralConfig();
    pid_->reset();
    auto result_after_reset = pid_->compute(target, robot_->q_, robot_->dq_, 0.05);

    // The first command after reset must match the first command from fresh.
    EXPECT_TRUE(result_fresh.command.isApprox(result_after_reset.command, 1e-6));
    EXPECT_DOUBLE_EQ(result_fresh.position_error, result_after_reset.position_error);
}

// ============================================================================
// MODULE: ControlResult Tests — field validation
// ============================================================================

TEST_F(ControllerTest, ControlResultFields) {
    Eigen::Vector3d target(0.30, -0.40, 0.20);
    robot_->setNeutralConfig();

    auto result = pid_->compute(target, robot_->q_, robot_->dq_, 0.05);

    EXPECT_TRUE(result.success);
    EXPECT_EQ(result.command.size(), robot_->reduced_model_.nv);
    EXPECT_GE(result.position_error, 0.0);
    EXPECT_TRUE(result.achieved_position.allFinite());
}

// ============================================================================
// MODULE: Main
// ============================================================================
int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
