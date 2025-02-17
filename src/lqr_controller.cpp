#include <algorithm>
#include <geometry_msgs/msg/detail/point_stamped__struct.hpp>
#include <geometry_msgs/msg/detail/pose_stamped__struct.hpp>
#include <geometry_msgs/msg/detail/twist__struct.hpp>
#include <iterator>
#include <nav_msgs/msg/detail/path__struct.hpp>
#include <rclcpp/time.hpp>
#include <string>
#include <iostream>
#include <fstream>
#include <limits>
#include <memory>
#include <vector>
#include <utility>
#include <cmath>

#include "lqr_controller/lqr_controller.hpp"
#include "lqr_controller/problem.hpp"

#include "angles/angles.h"
#include "nav2_core/exceptions.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav2_costmap_2d/costmap_filters/filter_values.hpp"

#include "lqr_controller/Tool.h"

using nav2_util::declare_parameter_if_not_declared;
using nav2_util::geometry_utils::euclidean_distance;
using rcl_interfaces::msg::ParameterType;
namespace lqr_controller
{
void LqrController::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  auto node = parent.lock();
  node_ = parent;
  if (!node) {
    throw nav2_core::PlannerException("Unable to lock node!");
  }

  costmap_ros_ = costmap_ros;
  costmap_ = costmap_ros_->getCostmap();
  tf_buffer_ = tf;
  plugin_name_ = name;
  logger_ = node->get_logger();
  clock_ = node->get_clock();
  

  global_path_pub_ = node->create_publisher<nav_msgs::msg::Path>("received_global_plan", 1);
  lqr_path_pub_ = node->create_publisher<nav_msgs::msg::Path>("lqr_path", 1);
  target_pub_ = node->create_publisher<geometry_msgs::msg::PoseStamped>("tracking_target", 10);
  // initialize collision checker and set costmap
  collision_checker_ = std::make_unique<nav2_costmap_2d::
  FootprintCollisionChecker<nav2_costmap_2d::Costmap2D *>>(costmap_);
  collision_checker_->setCostmap(costmap_);
  transform_tolerance_ = tf2::durationFromSec(0.5);

}

void LqrController::cleanup()
{
  RCLCPP_INFO(
    logger_,
    "Cleaning up controller: %s of type"
    " lqr_controller::LqrController",
    plugin_name_.c_str());
  global_path_pub_.reset();
  lqr_path_pub_.reset();
  target_pub_.reset();
  target_arc_pub_.reset();
  cusp_pub_.reset();
}

void LqrController::activate()
{
  RCLCPP_INFO(
    logger_,
    "Activating controller: %s of type "
    "LQR_controller::LqrController",
    plugin_name_.c_str());
  global_path_pub_->on_activate();
  lqr_path_pub_->on_activate();
  target_pub_->on_activate();
  // Remove these lines if publishers aren't needed
  
  // target_arc_pub_->on_activate();
  // cusp_pub_->on_activate();
  
  // Add callback for dynamic parameters
  auto node = node_.lock();
  dyn_params_handler_ = node->add_on_set_parameters_callback(
    std::bind(
      &LqrController::dynamicParametersCallback,
      this, std::placeholders::_1));
}

void LqrController::deactivate()
{
  RCLCPP_INFO(
    logger_,
    "Deactivating controller: %s of type "
    "lqr_controller::LqrController",
    plugin_name_.c_str());
  global_path_pub_->on_deactivate();
  lqr_path_pub_->on_deactivate();
  target_pub_->on_deactivate();
  target_arc_pub_->on_deactivate();
  cusp_pub_->on_deactivate();
  dyn_params_handler_.reset();
}

void LqrController::setPlan(const nav_msgs::msg::Path & path)
{
  global_plan_ = path;
}

double get_yaw(const geometry_msgs::msg::PoseStamped & pose) {
  tf2::Quaternion q(pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w);
  tf2::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
  yaw = fmod(yaw + 3 * M_PI, 2 * M_PI) - M_PI;
  return yaw;
}

// bool LqrController::transformPose(
//   const std::string frame,
//   const geometry_msgs::msg::PoseStamped & in_pose,
//   geometry_msgs::msg::PoseStamped & out_pose) const
// {
//   if (in_pose.header.frame_id == frame) {
//     out_pose = in_pose;
//     return true;
//   }

//   try {
//     tf_buffer_->transform(in_pose, out_pose, frame, transform_tolerance_);
//     out_pose.header.frame_id = frame;
//     return true;
//   } catch (tf2::TransformException & ex) {
//     RCLCPP_ERROR(logger_, "Exception in transformPose: %s", ex.what());
//   }
//   return false;
// }

// nav_msgs::msg::Path LqrController::transformGlobalPlan(
//   const geometry_msgs::msg::PoseStamped & pose)
// {
//   if (global_plan_.poses.empty()) {
//     throw nav2_core::PlannerException("Received plan with zero length");
//   }

//   // let's get the pose of the robot in the frame of the plan
//   geometry_msgs::msg::PoseStamped robot_pose;
//   if (!transformPose(global_plan_.header.frame_id, pose, robot_pose)) {
//     throw nav2_core::PlannerException("Unable to transform robot pose into global plan's frame");
//   }

//   // We'll discard points on the plan that are outside the local costmap
//   double max_costmap_extent = getCostmapMaxExtent();

//   auto closest_pose_upper_bound =
//     nav2_util::geometry_utils::first_after_integrated_distance(
//     global_plan_.poses.begin(), global_plan_.poses.end(), max_robot_pose_search_dist_);

//   // First find the closest pose on the path to the robot
//   // bounded by when the path turns around (if it does) so we don't get a pose from a later
//   // portion of the path
//   auto transformation_begin =
//     nav2_util::geometry_utils::min_by(
//     global_plan_.poses.begin(), closest_pose_upper_bound,
//     [&robot_pose](const geometry_msgs::msg::PoseStamped & ps) {
//       return euclidean_distance(robot_pose, ps);
//     });

//   // Find points up to max_transform_dist so we only transform them.
//   auto transformation_end = std::find_if(
//     transformation_begin, global_plan_.poses.end(),
//     [&](const auto & pose) {
//       return euclidean_distance(pose, robot_pose) > max_costmap_extent;
//     });

//   // Lambda to transform a PoseStamped from global frame to local
//   auto transformGlobalPoseToLocal = [&](const auto & global_plan_pose) {
//       geometry_msgs::msg::PoseStamped stamped_pose, transformed_pose;
//       stamped_pose.header.frame_id = global_plan_.header.frame_id;
//       stamped_pose.header.stamp = robot_pose.header.stamp;
//       stamped_pose.pose = global_plan_pose.pose;
//       transformPose(costmap_ros_->getBaseFrameID(), stamped_pose, transformed_pose);
//       transformed_pose.pose.position.z = 0.0;
//       return transformed_pose;
//     };

//   // Transform the near part of the global plan into the robot's frame of reference.
//   nav_msgs::msg::Path transformed_plan;
//   std::transform(
//     transformation_begin, transformation_end,
//     std::back_inserter(transformed_plan.poses),
//     transformGlobalPoseToLocal);
//   transformed_plan.header.frame_id = costmap_ros_->getBaseFrameID();
//   transformed_plan.header.stamp = robot_pose.header.stamp;

//   // Remove the portion of the global plan that we've already passed so we don't
//   // process it on the next iteration (this is called path pruning)
//   global_plan_.poses.erase(begin(global_plan_.poses), transformation_begin);
//   global_path_pub_->publish(transformed_plan);

//   if (transformed_plan.poses.empty()) {
//     throw nav2_core::PlannerException("Resulting plan has 0 poses in it.");
//   }

//   return transformed_plan;
// }
nav_msgs::msg::Path LqrController::grep_path_in_local_costmap(const geometry_msgs::msg::PoseStamped & robot_pose){
  nav_msgs::msg::Path local_path;
  local_path.header = global_plan_.header;
  double costmap_radius = costmap_->getSizeInMetersX()/2;
  for (auto & pose : global_plan_.poses) {
    if(std::hypot(pose.pose.position.x - robot_pose.pose.position.x, pose.pose.position.y - robot_pose.pose.position.y)>costmap_radius){
      break;
    }else{
      local_path.poses.push_back(pose);
    }
  }
  return local_path;
}

int Find_target_index(vehicleState state, nav_msgs::msg::Path &local_path){
  double min = abs(sqrt(pow(state.x - local_path.poses[0].pose.position.x, 2) + pow(state.y - local_path.poses[0].pose.position.y, 2)));
  int index = 0;
  for (size_t i = 0; i < local_path.poses.size(); i++)
  {
    double d = abs(sqrt(pow(state.x - local_path.poses[i].pose.position.x, 2) + pow(state.y - local_path.poses[i].pose.position.y, 2)));
    if (d < min)
    {
      min = d;
      index = i;
    }
  }

  //索引到终点前，当（机器人与下一个目标点的距离Lf）小于（当前目标点到下一个目标点距离L)时，索引下一个目标点
  if (((size_t)index + 1) < local_path.poses.size())
  {
    double current_x = local_path.poses[index].pose.position.x; double current_y = local_path.poses[index].pose.position.y;
    double next_x = local_path.poses[index + 1].pose.position.x; double next_y = local_path.poses[index + 1].pose.position.y;
    double L_ = abs(sqrt(pow(next_x - current_x, 2) + pow(next_y - current_y, 2)));
    double L_1 = abs(sqrt(pow(state.x - next_x, 2) + pow(state.y - next_y, 2)));
    //ROS_INFO("L is %f,Lf is %f",L,Lf);
    if (L_1 < L_)
    {
      index += 1;
    }
  }


  return index;
}

// TODO: must finish this function, this function is to receive global plan 
// then compute velocity to actuate the base
geometry_msgs::msg::TwistStamped LqrController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & speed,
  nav2_core::GoalChecker * goal_checker)
{
  std::lock_guard<std::mutex> lock_reinit(mutex_);
  nav2_costmap_2d::Costmap2D * costmap = costmap_ros_->getCostmap();
  std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*(costmap->getMutex()));

  geometry_msgs::msg::TwistStamped cmd_vel;

  // Eigen::VectorXd x_r = VectorXd::Zero(3);

  nav_msgs::msg::Path local_plan = grep_path_in_local_costmap(pose);

  


  // int nbSteps = local_plan.poses.size();
  // Eigen::VectorXd X_r = VectorXd::Zero(nbSteps * 3);
  // for(unsigned int i = 0; i < local_plan.poses.size(); i++) {
  //   x_r << local_plan.poses[i].pose.position.x, local_plan.poses[i].pose.position.y, get_yaw(local_plan.poses[i]);
  //   X_r.segment(i * 3, 3) = x_r;
  // }
  
  // problem_ = std::make_shared<Problem>(nbSteps, X_r);
  lqr_controller_ = std::make_shared<LQR>();

  // Eigen::VectorXd x_init = Eigen::VectorXd::Zero(3);
  // x_init << pose.pose.position.x, pose.pose.position.y, get_yaw(pose);

  // double K;
  // std::vector<std::vector<double>> path;
  std::vector<waypoint> wps;
  for(size_t i=0;i<global_plan_.poses.size();i++){
    // std::vector<double> temp;
    // temp.push_back(local_plan.poses[i].pose.position.x);
    // temp.push_back(local_plan.poses[i].pose.position.y);
    // temp.push_back(get_yaw(local_plan.poses[i]));
    // path.push_back(temp);
    waypoint wp;
    wp.ID = i;
    wp.x = global_plan_.poses[i].pose.position.x;
    wp.y = global_plan_.poses[i].pose.position.y;
    wp.yaw = get_yaw(global_plan_.poses[i]);
    wps.push_back(wp);
  }

  robot_state_ = vehicleState{pose.pose.position.x,pose.pose.position.y,get_yaw(pose), speed.linear.x,kesi_};

  RCLCPP_INFO(logger_,"pos %f %f",pose.pose.position.x,pose.pose.position.y);

  // lqr 
  // get closest point
  int target_index = Find_target_index(robot_state_,global_plan_);
  // std::cout << "get point " << std::endl;
  waypoint Point = wps[target_index];

  geometry_msgs::msg::PoseStamped target_pose;
  target_pose.header.stamp = rclcpp::Time();
  target_pose.header.frame_id = "map";
  target_pose.pose = global_plan_.poses[target_index].pose;
  target_pub_->publish(target_pose);

  // std::cout << "path len: " << global_plan_.poses.size()  << " target index: " << target_index << " Point: " << "[" <<Point.x << "," << Point.y << "," << Point.yaw << "]" << std::endl;

  double K = cal_K(wps,target_index);
  // std::cout << "K: " << K << std::endl;
  double kesi = atan2(L_ * K, 1);
  // std::cout << "kesi: " << kesi << std::endl;
  double Q[5] = {10.0,1.0,1,1,1};
  double R[2] = {1,1};

  U U_r;

  double distance_to_goal = std::hypot(pose.pose.position.x - global_plan_.poses[global_plan_.poses.size()-1].pose.position.x,pose.pose.position.y - global_plan_.poses[global_plan_.poses.size()-1].pose.position.y);
  double Kp = 0.5;
  U_r.v = Kp * distance_to_goal;

  if(goal_checker->isGoalReached(pose.pose, global_plan_.poses.back().pose , speed)){
    U_r.v = 0;
  }
  U_r.v = std::clamp(U_r.v,-0.50,0.5);
  U_r.kesi = kesi;

  dt_ = 0.05;
  L_ = 0.1;

  lqr_controller_->initial(L_, dt_, robot_state_, Point, U_r, Q, R);

  // std::cout << "compute u" << std::endl;
  U control = lqr_controller_->cal_vel();//计算输入[v, kesi]
  // std::cout << "u completed" << std::endl;
  if(U_r.v==0)control.v = 0;

  // std::cout << "u kesi: " << U_r.kesi << std::endl;
  
  cmd_vel.twist.linear.x = control.v;
  cmd_vel.twist.angular.z = control.v*tan(control.kesi)/L_;

  cmd_vel.twist.linear.x = std::clamp(cmd_vel.twist.linear.x,-0.5,0.5);
  // cmd_vel.twist.angular.z = std::clamp(cmd_vel.twist.angular.z,-1.0,1.0);

  // auto [X_vec, U] = problem_->lqrSolve(x_init);

  // visualize lqr path
  // nav_msgs::msg::Path lqr_path;
  // for(size_t i=0;i<X_vec.size();i++){
  //   geometry_msgs::msg::PoseStamped pose_stamped;
  //   pose_stamped.pose.position.x = X_vec[i][0];
  //   pose_stamped.pose.position.y = X_vec[i][1];
  //   pose_stamped.pose.orientation = tf2::toMsg(tf2::Quaternion(tf2::Vector3(0, 0, 1), X_vec[i][2]));
  //   pose_stamped.header.frame_id = "map";
  //   lqr_path.poses.push_back(pose_stamped);
    
  //   // printf("%f %f %f \n",X_vec[i][0],X_vec[i][1],X_vec[i][2]);
  // }
  // lqr_path.header.stamp = clock_->now();
  // lqr_path.header.frame_id = "map";
  // lqr_path_pub_->publish(lqr_path);


  // cmd_vel.twist.linear.x = U.row(0)(0);
  // cmd_vel.twist.angular.z = U.row(0)(1);
  // end of lqr controller code
  return cmd_vel;
}

double LqrController::costAtPose(const double & x, const double & y)
{
  unsigned int mx, my;

  if (!costmap_->worldToMap(x, y, mx, my)) {
    RCLCPP_FATAL(
      logger_,
      "The dimensions of the costmap is too small to fully include your robot's footprint, "
      "thusly the robot cannot proceed further");

    throw nav2_core::PlannerException(
            "LqrController: Dimensions of the costmap are too small "
            "to encapsulate the robot footprint at current speeds!");
  }

  unsigned char cost = costmap_->getCost(mx, my);
  return static_cast<double>(cost);
}

void LqrController::setSpeedLimit(
  const double & speed_limit,
  const bool & percentage)
{
  if (speed_limit == nav2_costmap_2d::NO_SPEED_LIMIT) {
    // Restore default value
    
  } else {
    if (percentage) {
      // Speed limit is expressed in % from maximum speed of robot
    } else {
      // Speed limit is expressed in absolute value
    }
  }
}

rcl_interfaces::msg::SetParametersResult LqrController::dynamicParametersCallback(
  std::vector<rclcpp::Parameter> parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  std::lock_guard<std::mutex> lock_reinit(mutex_);
  for (auto parameter : parameters) {
    const auto & type = parameter.get_type();
    const auto & name = parameter.get_name();
    RCLCPP_INFO(logger_,"changing parameter: %s",name.c_str());
    if (type == ParameterType::PARAMETER_DOUBLE) {
      
    } else if (type == ParameterType::PARAMETER_BOOL) {
     
    }
  }

  

  result.successful = true;
  return result;
}

}  

// Register this controller as a nav2_core plugin
PLUGINLIB_EXPORT_CLASS(
  lqr_controller::LqrController,
  nav2_core::Controller)