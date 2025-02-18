#include <algorithm>
#include <atomic>
#include <geometry_msgs/msg/detail/point_stamped__struct.hpp>
#include <geometry_msgs/msg/detail/pose_stamped__struct.hpp>
#include <geometry_msgs/msg/detail/twist__struct.hpp>
#include <iterator>
#include <nav_msgs/msg/detail/path__struct.hpp>
#include <rclcpp/clock.hpp>
#include <rclcpp/logging.hpp>
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
#include <algorithm>

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

nav_msgs::msg::Path LqrController::grep_path_in_local_costmap(const geometry_msgs::msg::PoseStamped & robot_pose){
  nav_msgs::msg::Path local_path;
  local_path.header = global_plan_.header;
  double costmap_radius = std::max(costmap_->getSizeInMetersX(),costmap_->getSizeInMetersY())/2;
  int index = Find_target_index(robot_pose,global_plan_);

  // fill local plan on the front
  for (int i=index; i>0;--i) {
    auto pose = global_plan_.poses[i];
    if(std::hypot(pose.pose.position.x - robot_pose.pose.position.x, pose.pose.position.y - robot_pose.pose.position.y)>costmap_radius){
      break;
    }else{
      local_path.poses.insert(local_path.poses.begin(),(pose));
    }
  }
  // fill local 
  for(size_t i=index;i<global_plan_.poses.size();++i){
    auto pose = global_plan_.poses[i];
    if(std::hypot(pose.pose.position.x - robot_pose.pose.position.x, pose.pose.position.y - robot_pose.pose.position.y)>costmap_radius){
      break;
    }else{
      local_path.poses.push_back(pose);
    }
  }

  local_path.header.frame_id = costmap_ros_->getGlobalFrameID();
  local_path.header.stamp = rclcpp::Time();
  return local_path;
}

int LqrController::Find_target_index(const geometry_msgs::msg::PoseStamped & state, nav_msgs::msg::Path &local_path){
  double min = abs(sqrt(pow(state.pose.position.x - local_path.poses[0].pose.position.x, 2) + pow(state.pose.position.y - local_path.poses[0].pose.position.y, 2)));
  int index = 0;
  for (size_t i = 0; i < local_path.poses.size(); i++)
  {
    double d = abs(sqrt(pow(state.pose.position.x - local_path.poses[i].pose.position.x, 2) + pow(state.pose.position.y - local_path.poses[i].pose.position.y, 2)));
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
    double L_1 = abs(sqrt(pow(state.pose.position.x - next_x, 2) + pow(state.pose.position.y - next_y, 2)));
    //ROS_INFO("L is %f,Lf is %f",L,Lf);
    if (L_1 < L_)
    {
      index += 1;
    }
  }


  return index;
}

void LqrController::remove_duplicated_points(vector<waypoint>& points){
  for(size_t i=0;i<points.size()-1;i++){
    if(points[i].x == points[i+1].x && points[i].y == points[i+1].y && points[i+1].yaw == points[i+1].yaw){
      points.erase(points.begin()+i);
    }
  }
}

vector<double> LqrController::get_speed_profile(vehicleState state,float fv_max,float bv_max,float v_min,float max_lateral_accel,vector<waypoint>& wp,vector<double>& curvature_list){
  vector<double> sp(wp.size());
  double kp = 0.5;// TODO: parameterize it
  for (size_t i = 0; i < wp.size()-1; i++)
  {
    // get next point on from or back
    bool backward_motion = false;
    Matrix4x4 T_map_pn,T_map_pnp1;
    T_map_pn << cos(wp[i].yaw), -sin(wp[i].yaw) ,0 , wp[i].x,
                sin(wp[i].yaw), cos(wp[i].yaw), 0 , wp[i].y,
                0, 0, 1, 0,
                0, 0, 0, 1;
    T_map_pnp1 << cos(wp[i+1].yaw), -sin(wp[i+1].yaw) ,0 , wp[i+1].x,
                sin(wp[i+1].yaw), cos(wp[i+1].yaw), 0 , wp[i+1].y,
                0, 0, 1, 0,
                0, 0, 0, 1;
    Matrix4x4 T_pn_pnp1 = T_map_pn.inverse() * T_map_pnp1;
    float next_dir = atan2(T_pn_pnp1(1,3), T_pn_pnp1(0,3));
    if(next_dir < -M_PI_2 || next_dir > M_PI_2){
      backward_motion = true;
    }
    // curvature constraint
    double K = cal_K(wp, i);
    if(isnan(K)){
      cerr << "K is nan, index: " << i << endl;
    }
    curvature_list.push_back(K);
    double max_v_curvature = std::sqrt(max_lateral_accel / std::abs(curvature_list[i]));

    // goal constrain
    double distance_to_goal = std::hypot(state.x - wp.back().x, state.y - wp.back().y);
    double max_v_distance = kp*distance_to_goal;
    max_v_distance = std::max((double)v_min, max_v_distance);
    sp.back() = max_v_distance; // set last point speed profile as dynamic

    // get speed
    if(!backward_motion){
      sp[i] = std::min((double)fv_max, std::min(max_v_curvature,max_v_distance));   // forward motion profile
    }else{
      sp[i] = -std::min((double)abs(bv_max), std::min(max_v_curvature,max_v_distance)); // backward motion profile
    }
    
  }
  return sp;
}

bool LqrController::transformPose(
  const std::string frame,
  const geometry_msgs::msg::PoseStamped & in_pose,
  geometry_msgs::msg::PoseStamped & out_pose) const
{
  if (in_pose.header.frame_id == frame) {
    out_pose = in_pose;
    return true;
  }

  try {
    tf_buffer_->transform(in_pose, out_pose, frame, transform_tolerance_);
    out_pose.header.frame_id = frame;
    return true;
  } catch (tf2::TransformException & ex) {
    RCLCPP_ERROR(logger_, "Exception in transformPose: %s", ex.what());
  }
  return false;
}

// TODO: must finish this function, this function is to receive global plan 
// then compute velocity to actuate the base
geometry_msgs::msg::TwistStamped LqrController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & speed,
  nav2_core::GoalChecker * /*goal_checker*/)
{
  std::lock_guard<std::mutex> lock_reinit(mutex_);
  nav2_costmap_2d::Costmap2D * costmap = costmap_ros_->getCostmap();
  std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*(costmap->getMutex()));

  geometry_msgs::msg::TwistStamped cmd_vel;
  static double last_control_time = rclcpp::Clock().now().seconds();
  // get robot coordinate in map frame, due to the argument pose is in odom frame
  geometry_msgs::msg::PoseStamped global_pose;
  transformPose(global_plan_.header.frame_id,pose,global_pose);

  // crop path inside costmap and publish it
  nav_msgs::msg::Path local_plan = grep_path_in_local_costmap(global_pose);
  // if robot is not properly localized, use global plan instead
  if(local_plan.poses.size() == 0){
    RCLCPP_ERROR(logger_,"local plan empty, trying to find waypoint via global plan.");
    local_plan = global_plan_;
  }
  lqr_path_pub_->publish(local_plan);

  // declare LQR controller
  lqr_controller_ = std::make_shared<LQR>();
  
  // prepare lqr controller 
  std::vector<waypoint> wps;
  robot_state_ = vehicleState{global_pose.pose.position.x,global_pose.pose.position.y,get_yaw(global_pose), speed.linear.x,kesi_};

  // get target point to track and publish 
  int target_index = Find_target_index(global_pose,local_plan);
  geometry_msgs::msg::PoseStamped target_pose;
  target_pose.header.stamp = rclcpp::Time();
  target_pose.header.frame_id = "map";
  target_pose.pose = local_plan.poses[target_index].pose;
  target_pub_->publish(target_pose);

  // get waypoints list
  for(size_t i=0;i<local_plan.poses.size();i++){
    waypoint wp;
    wp.ID = i;
    wp.x = local_plan.poses[i].pose.position.x;
    wp.y = local_plan.poses[i].pose.position.y;
    wp.yaw = get_yaw(local_plan.poses[i]);
    wps.push_back(wp);
  }
  waypoint Point = wps[target_index];
  // remove duplicated points to ensure cusp can be identified correctly
  remove_duplicated_points(wps);

  // compute curvature and apply constraints to speed
  vector<double> k_list;
  // TODO: speed constraints made to be parameters
  vector<double> sp = get_speed_profile(robot_state_,0.5,0.3,0.1,0.20,wps,k_list);
  // get curvature of tracking point
  double K = k_list[target_index];
  double kesi = atan2(L_ * K, 1);   // reference steer angle

  // TODO: these matrix should be made as parameters
  double Q[5] = {8.0,1.0,1,1,1};
  double R[2] = {1,1};

  // reference input
  U U_r;
  U_r.v = sp[target_index];   // apply reference speed
  U_r.v = std::clamp(U_r.v,-0.50,0.5);
  U_r.kesi = kesi*(sp[target_index]>0?1:-1);

  // TODO: make L parameter
  dt_ = rclcpp::Clock().now().seconds() - last_control_time;
  L_ = 0.1;

  last_control_time = rclcpp::Clock().now().seconds();

  lqr_controller_->initial(L_, dt_, robot_state_, Point, U_r, Q, R);

  // std::cout << "compute u" << std::endl;
  U control = lqr_controller_->cal_vel();//计算输入[v, kesi]
  // std::cout << "u completed" << std::endl;
  if(U_r.v==0)control.v = 0;

  // std::cout << "u kesi: " << U_r.kesi << std::endl;
  
  cmd_vel.twist.linear.x = control.v;
  cmd_vel.twist.angular.z = control.v*tan(control.kesi)/L_;

  cmd_vel.twist.linear.x = std::clamp(cmd_vel.twist.linear.x,-0.5,0.5);
  cmd_vel.twist.angular.z = std::clamp(cmd_vel.twist.angular.z,-2.0,2.0);


  // cmd_vel.twist.angular.z = 0;
  // cmd_vel.twist.linear.x = 0;

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