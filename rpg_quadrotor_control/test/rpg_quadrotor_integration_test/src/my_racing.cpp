#include "rpg_quadrotor_integration_test/rpg_quadrotor_integration_test.h"

#include <gtest/gtest.h>
#include <vector>

#include <autopilot/autopilot_states.h>
#include <polynomial_trajectories/polynomial_trajectory_settings.h>
#include <quadrotor_common/control_command.h>
#include <quadrotor_common/geometry_eigen_conversions.h>
#include <std_msgs/Bool.h>
#include <trajectory_generation_helper/heading_trajectory_helper.h>
#include <trajectory_generation_helper/polynomial_trajectory_helper.h>
#include <Eigen/Dense>
#include <nav_msgs/Path.h>


// trajectory
#include <polynomial_trajectories/minimum_snap_trajectories.h>
#include <polynomial_trajectories/polynomial_trajectories_common.h>
#include <polynomial_trajectories/polynomial_trajectory.h>
#include <polynomial_trajectories/polynomial_trajectory_settings.h>
#include <quadrotor_common/trajectory_point.h>
#include <quadrotor_common/trajectory.h>


namespace rpg_quadrotor_integration_test {

  QuadrotorIntegrationTest::QuadrotorIntegrationTest()
  : executing_trajectory_(false),
  sum_position_error_squared_(0.0),
  max_position_error_(0.0),
  sum_thrust_direction_error_squared_(0.0),
  max_thrust_direction_error_(0.0) {
    ros::NodeHandle nh;

    arm_pub_ = nh.advertise<std_msgs::Bool>("bridge/arm", 1);

    measure_tracking_timer_ =
    nh_.createTimer(ros::Duration(1.0 / kExecLoopRate_),
      &QuadrotorIntegrationTest::measureTracking, this);
  }

  QuadrotorIntegrationTest::~QuadrotorIntegrationTest() {}

  void QuadrotorIntegrationTest::measureTracking(const ros::TimerEvent& time) {
    if (executing_trajectory_) {
    // Position error
      const double position_error =
      autopilot_helper_.getCurrentPositionError().norm();
      sum_position_error_squared_ += pow(position_error, 2.0);
      if (position_error > max_position_error_) {
        max_position_error_ = position_error;
      }

    // Thrust direction error
      const Eigen::Vector3d ref_thrust_direction =
      autopilot_helper_.getCurrentReferenceOrientation() *
      Eigen::Vector3d::UnitZ();
      const Eigen::Vector3d thrust_direction =
      autopilot_helper_.getCurrentOrientationEstimate() *
      Eigen::Vector3d::UnitZ();

      const double thrust_direction_error =
      acos(ref_thrust_direction.dot(thrust_direction));
      sum_thrust_direction_error_squared_ += pow(thrust_direction_error, 2.0);
      if (thrust_direction_error > max_thrust_direction_error_) {
        max_thrust_direction_error_ = thrust_direction_error;
      }
    }
  }

  void QuadrotorIntegrationTest::run() {
    ros::Rate command_rate(kExecLoopRate_);

  // Make sure everything is up and running
  // Wait for Autopilot feedback with assert
  // ASSERT_TRUE(autopilot_helper_.waitForAutopilotFeedback(10.0, kExecLoopRate_))
  //     << "Did not receive autopilot feedback within 10 seconds.";

    autopilot_helper_.waitForAutopilotFeedback(10.0, kExecLoopRate_);

    ros::Duration(3.0).sleep();

  // Arm bridge
    std_msgs::Bool arm_msg;
    arm_msg.data = true;
    arm_pub_.publish(arm_msg);


    ros::NodeHandle n;
    ros::Publisher path_pub = n.advertise<nav_msgs::Path>("path", 1000);


  // Takeoff for real
    autopilot_helper_.sendStart();

  //wait for autopilot to go to hover
    autopilot_helper_.waitForSpecificAutopilotState(
      autopilot::States::HOVER, 10.0, kExecLoopRate_);

  executing_trajectory_ = true;  // Start measuring errors

//   // Send pose command
  const Eigen::Vector3d position_cmd = Eigen::Vector3d(-1, -1.25, 4.0);
  // const double heading_cmd = -3.14/4;
  const double heading_cmd = 0;

  autopilot_helper_.sendPoseCommand(position_cmd, heading_cmd);

  autopilot_helper_.waitForSpecificAutopilotState(
    autopilot::States::TRAJECTORY_CONTROL, 10.0, kExecLoopRate_);

  autopilot_helper_.waitForSpecificAutopilotState(
    autopilot::States::HOVER, 10.0, kExecLoopRate_);

  ros::Duration(50).sleep();



  // Generate trajectory, sample it and send it as reference states
  const double max_vel = 3*2.0; // 2.0
  const double max_thrust = 3*20.0; // 15.0
  const double max_roll_pitch_rate = 3*1; // 0.5
  quadrotor_common::TrajectoryPoint end_state;
  end_state.position = position_cmd;
  end_state.heading = heading_cmd;

std::vector<Eigen::Vector3d> way_points;

// for ring trajectory, use the first waypoint as well

// way_points.push_back(Eigen::Vector3d(-1, -1.25, 4.0));
way_points.push_back(Eigen::Vector3d(9, 6.25, 2.0));
way_points.push_back(Eigen::Vector3d(9, -3.5, 2.0));
way_points.push_back(Eigen::Vector3d(-3.5, -6, 4.0));
way_points.push_back(Eigen::Vector3d(-3.5, -6, 2.0));
way_points.push_back(Eigen::Vector3d(4, -1, 2.0));
way_points.push_back(Eigen::Vector3d(-2.5, 6.5, 4.0));

way_points.push_back(Eigen::Vector3d(-1, -1.25, 4.0));
way_points.push_back(Eigen::Vector3d(9, 6.25, 2.0));
way_points.push_back(Eigen::Vector3d(9, -3.5, 2.0));
way_points.push_back(Eigen::Vector3d(-3.5, -6, 4.0));
way_points.push_back(Eigen::Vector3d(-3.5, -6, 2.0));
way_points.push_back(Eigen::Vector3d(4, -1, 2.0));
way_points.push_back(Eigen::Vector3d(-2.5, 6.5, 4.0));

way_points.push_back(Eigen::Vector3d(-1, -1.25, 4.0));
way_points.push_back(Eigen::Vector3d(9, 6.25, 2.0));
way_points.push_back(Eigen::Vector3d(9, -3.5, 2.0));
way_points.push_back(Eigen::Vector3d(-3.5, -6, 4.0));
way_points.push_back(Eigen::Vector3d(-3.5, -6, 2.0));
way_points.push_back(Eigen::Vector3d(4, -1, 2.0));
way_points.push_back(Eigen::Vector3d(-2.5, 6.5, 4.0));



Eigen::VectorXd initial_ring_segment_times(int(way_points.size())+1); 
initial_ring_segment_times <<  1, 1, 1, 0.5, 1, 1, 1,
                               1, 1, 1, 0.5, 1, 1, 1,
                               1, 1, 1, 0.5, 1, 1, 1;
initial_ring_segment_times = 2*initial_ring_segment_times;


polynomial_trajectories::PolynomialTrajectorySettings
  ring_trajectory_settings;
ring_trajectory_settings.continuity_order = 4;
Eigen::VectorXd minimization_weights(5);
minimization_weights << 0.0, 1.0, 1.0, 1.0, 1.0; // first was zero
ring_trajectory_settings.minimization_weights = minimization_weights;
ring_trajectory_settings.polynomial_order = 11;
ring_trajectory_settings.way_points = way_points;

quadrotor_common::TrajectoryPoint my_start, my_end;
my_start = end_state;
my_end = end_state;




quadrotor_common::Trajectory ring_traj = trajectory_generation_helper::
  polynomials::generateMinimumSnapTrajectory(
    initial_ring_segment_times, my_start, my_end, 
      ring_trajectory_settings, kExecLoopRate_);

// quadrotor_common::Trajectory ring_traj = trajectory_generation_helper::
// polynomials::generateMinimumSnapRingTrajectory(
//   initial_ring_segment_times, ring_trajectory_settings, max_vel,
//   max_thrust, max_roll_pitch_rate, kExecLoopRate_);

// quadrotor_common::Trajectory ring_traj = trajectory_generation_helper::
//   polynomials::generateMinimumSnapRingTrajectory(
//     initial_ring_segment_times, ring_trajectory_settings,kExecLoopRate_);

polynomial_trajectories::PolynomialTrajectorySettings
  enter_trajectory_settings = ring_trajectory_settings;
enter_trajectory_settings.way_points.clear();



// trajectory_generation_helper::heading::addConstantHeadingRate(0.0, 0,
//   &ring_traj);
trajectory_generation_helper::heading::addHeadingAlongTrajectory(0.0, 0,
  &ring_traj);

nav_msgs::Path traj_path_msg = ring_traj.toRosPath();
path_pub.publish(traj_path_msg);

autopilot_helper_.sendTrajectory(ring_traj);

autopilot_helper_.waitForSpecificAutopilotState(
  autopilot::States::TRAJECTORY_CONTROL, 20.0, kExecLoopRate_);


  // Before trajectory finishes force autopilot to hover
while (autopilot_helper_.getCurrentTrajectoryExecutionLeftDuration() >
  ros::Duration(0.2)) {
  ros::spinOnce();
  command_rate.sleep();
}
  executing_trajectory_ = false;  // Stop measuring errors

  autopilot_helper_.sendForceHover();

  // Land
  autopilot_helper_.sendLand();

  autopilot_helper_.waitForSpecificAutopilotState(
    autopilot::States::LAND, 5.0, kExecLoopRate_);

  autopilot_helper_.waitForSpecificAutopilotState(
    autopilot::States::OFF, 35.0, kExecLoopRate_);

  ros::Duration(0.2).sleep();

  autopilot_helper_.sendOff();

  

}

int main(int argc, char** argv) {
  
  ros::init(argc, argv, "rpg_quadrotor_integration_test");
  rpg_quadrotor_integration_test::QuadrotorIntegrationTest rpg_quadrotor_integration_test;
  rpg_quadrotor_integration_test.run();

  return 0;
}
