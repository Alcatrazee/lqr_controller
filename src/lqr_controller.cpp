#include <algorithm>
#include <chrono>
#include <geometry_msgs/msg/detail/point_stamped__struct.hpp>
#include <geometry_msgs/msg/detail/pose_stamped__struct.hpp>
#include <geometry_msgs/msg/detail/twist__struct.hpp>
#include <nav2_util/geometry_utils.hpp>
#include <nav_msgs/msg/detail/path__struct.hpp>
#include <numeric>
#include <rclcpp/clock.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/time.hpp>
#include <std_msgs/msg/detail/float32_multi_array__struct.hpp>
#include <std_msgs/msg/detail/u_int64_multi_array__struct.hpp>
#include <string>
#include <memory>
#include <vector>
#include <cmath>
#include "lqr_controller/lqr_controller.hpp"
#include "nav2_core/exceptions.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav2_costmap_2d/costmap_filters/filter_values.hpp"
#include "lqr_controller/Tool.h"
#include <algorithm>
#include <angles/angles.h>
#include <geometry_msgs/msg/polygon_stamped.hpp>

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

  // speed related
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".max_fvx", rclcpp::ParameterValue(0.50));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".max_bvx", rclcpp::ParameterValue(0.5));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".max_wz", rclcpp::ParameterValue(1.00));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".max_linear_accel", rclcpp::ParameterValue(0.10));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".max_lateral_accel", rclcpp::ParameterValue(0.5));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".max_w_accel", rclcpp::ParameterValue(0.4));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".dead_band_speed", rclcpp::ParameterValue(0.1));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".approach_velocity_scaling_dist", rclcpp::ParameterValue(1.0));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".use_output_filter", rclcpp::ParameterValue(false));  
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".use_obstacle_stopping", rclcpp::ParameterValue(true));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".patient_encounter_obst", rclcpp::ParameterValue(10.0));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".path_obst_stop_dist", rclcpp::ParameterValue(1.0));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".path_obst_slow_dist", rclcpp::ParameterValue(2.0));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".vehicle_L", rclcpp::ParameterValue(0.10));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".robot_search_pose_dist", rclcpp::ParameterValue(10));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".inversion_xy_tolerance", rclcpp::ParameterValue(0.10));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".latteral_err_penalty", rclcpp::ParameterValue(8.0));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".latteral_err_dot_penalty", rclcpp::ParameterValue(1.0));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".angle_err_penalty", rclcpp::ParameterValue(1.0));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".angle_err_dot_penalty", rclcpp::ParameterValue(1.0));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".v_err_penalty", rclcpp::ParameterValue(1.0));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".w_effort_penalty", rclcpp::ParameterValue(1.0));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".acc_effort_penalty", rclcpp::ParameterValue(1.0));

  node->get_parameter(plugin_name_ + ".max_fvx", max_fvx_);
  node->get_parameter(plugin_name_ + ".max_bvx", max_bvx_);
  node->get_parameter(plugin_name_ + ".max_wz", max_wz_);
  node->get_parameter(plugin_name_ + ".max_linear_accel", max_lin_acc_);
  node->get_parameter(plugin_name_ + ".max_lateral_accel", max_lateral_accel_);
  node->get_parameter(plugin_name_ + ".max_w_accel", max_w_acc_);
  node->get_parameter(plugin_name_ + ".dead_band_speed", dead_band_speed_);
  node->get_parameter(plugin_name_ + ".approach_velocity_scaling_dist", approach_velocity_scaling_dist_);
  node->get_parameter(plugin_name_ + ".use_obstacle_stopping", use_obstacle_stopping_);
  node->get_parameter(plugin_name_ + ".use_output_filter", use_output_filter_);
  node->get_parameter(plugin_name_ + ".patient_encounter_obst", obstacle_timeout_);
  node->get_parameter(plugin_name_ + ".path_obst_stop_dist", obst_stop_dist_);
  node->get_parameter(plugin_name_ + ".path_obst_slow_dist", obst_slow_dist_);
  node->get_parameter(plugin_name_ + ".vehicle_L", vehicle_L_);
  node->get_parameter(plugin_name_ + ".robot_search_pose_dist", robot_search_pose_dist_);
  node->get_parameter(plugin_name_ + ".inversion_xy_tolerance", inversion_xy_tolerance_);
  node->get_parameter(plugin_name_ + ".latteral_err_penalty", Q_[0]);
  node->get_parameter(plugin_name_ + ".latteral_err_dot_penalty", Q_[1]);
  node->get_parameter(plugin_name_ + ".angle_err_penalty", Q_[2]);
  node->get_parameter(plugin_name_ + ".angle_err_dot_penalty", Q_[3]);
  node->get_parameter(plugin_name_ + ".v_err_penalty", Q_[4]);
  node->get_parameter(plugin_name_ + ".w_effort_penalty", R_[0]);
  node->get_parameter(plugin_name_ + ".acc_effort_penalty", R_[1]);
  
  obst_speed_control_k_ = (max_fvx_ - dead_band_speed_)/(obst_slow_dist_ - obst_stop_dist_);
  obst_speed_control_b_ = max_fvx_ - obst_speed_control_k_*obst_slow_dist_;
  min_lin_deacc_ = pow(max_fvx_,2)/(2*approach_velocity_scaling_dist_);
  RCLCPP_INFO(logger_,"computed linear deaccleration %lf",min_lin_deacc_);

  global_path_pub_ = node->create_publisher<nav_msgs::msg::Path>("received_global_plan", 1);
  lqr_path_pub_ = node->create_publisher<nav_msgs::msg::Path>("lqr_path", 1);
  target_pub_ = node->create_publisher<geometry_msgs::msg::PoseStamped>("tracking_target", 10);
  cusp_pub_ = node->create_publisher<geometry_msgs::msg::PointStamped>("cusp", 10);
  collision_polygon_pub_ = node->create_publisher<geometry_msgs::msg::PolygonStamped>("collision_polygon", 10);
  error_code_pub_ = node->create_publisher<std_msgs::msg::UInt64MultiArray>("error_code", 10);
  debug_pub_ = node->create_publisher<std_msgs::msg::Float32MultiArray>("lqr_debug", 10);

  // initialize collision checker and set costmap
  collision_checker_ = std::make_unique<nav2_costmap_2d::FootprintCollisionChecker<nav2_costmap_2d::Costmap2D *>>(costmap_);
  collision_checker_->setCostmap(costmap_);
  transform_tolerance_ = tf2::durationFromSec(0.5);
  max_steer_rate_ = 0.5;

  encounter_obst_moment_logged_ = false;
  lqr_controller_ = std::make_shared<LQR>();
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
  collision_polygon_pub_.reset();
  error_code_pub_.reset();
  debug_pub_.reset();
  // target_arc_pub_.reset();
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
  collision_polygon_pub_->on_activate();
  error_code_pub_->on_activate();
  debug_pub_->on_activate();
  cusp_pub_->on_activate();
  
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
  collision_polygon_pub_->on_deactivate();
  error_code_pub_->on_deactivate();
  debug_pub_->on_deactivate();
  cusp_pub_->on_deactivate();
  dyn_params_handler_.reset();
}

void LqrController::removeDuplicatedPathPoint(nav_msgs::msg::Path & path)
{
  auto start_time = std::chrono::high_resolution_clock::now();
  for(size_t i=1;i<path.poses.size();i++){
    if(path.poses[i].pose.position.x == path.poses[i-1].pose.position.x &&
      path.poses[i].pose.position.y == path.poses[i-1].pose.position.y &&
      path.poses[i].pose.position.z == path.poses[i-1].pose.position.z &&
      path.poses[i].pose.orientation.x == path.poses[i-1].pose.orientation.x &&
      path.poses[i].pose.orientation.y == path.poses[i-1].pose.orientation.y &&
      path.poses[i].pose.orientation.z == path.poses[i-1].pose.orientation.z){
        
      path.poses.erase(path.poses.begin() + i);
      RCLCPP_INFO(logger_, "Removed duplicate pose");
    }
  }
  auto end_time = std::chrono::high_resolution_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::nanoseconds>(end_time - start_time);
  RCLCPP_DEBUG(logger_, "Time taken to remove duplicates: %ld nanoseconds", duration.count());
}

vector<int> LqrController::find_cusp(const nav_msgs::msg::Path & path){
  vector<int> cusp_indexs;
  // check if path is long enough to contain cusp
  cusp_indexs.push_back(0);
  if(path.poses.size()>3){
    // find cusp based on vector of each points
    for(size_t i=1;i<=path.poses.size()-2;++i){ 
      Eigen::Vector2d vec_ab,vec_bc;
      vec_ab << path.poses[i].pose.position.x - path.poses[i-1].pose.position.x, path.poses[i].pose.position.y - path.poses[i-1].pose.position.y;
      vec_bc << path.poses[i+1].pose.position.x - path.poses[i].pose.position.x, path.poses[i+1].pose.position.y - path.poses[i].pose.position.y;
      double dot_product = vec_ab(0)*vec_bc(0) + vec_ab(1)*vec_bc(1);
      if(dot_product<0.0){
        cusp_indexs.push_back(i);
      }
    }
  }
  cusp_indexs.push_back(path.poses.size()-1);
  return cusp_indexs;
}

// 线性插值函数
double LqrController::lerp(double a, double b, double t) {
  return a + t * (b - a);
}

// 角度插值函数（考虑周期性）
double LqrController::angle_lerp(double a, double b, double t) {
  double delta = atan2(sin(b - a), cos(b - a)); // 计算最短角度差
  return a + t * delta;
}

// 从四元数提取 yaw 角（theta）TODO: reuse get_yaw()
double LqrController::get_yaw_from_quaternion(const geometry_msgs::msg::Quaternion& quat) {
  tf2::Quaternion tf_quat;
  tf2::fromMsg(quat, tf_quat);
  double roll, pitch, yaw;
  tf2::Matrix3x3(tf_quat).getRPY(roll, pitch, yaw);
  return yaw;
}

// 重新采样路径
std::vector<geometry_msgs::msg::PoseStamped> LqrController::resample_path(const std::vector<geometry_msgs::msg::PoseStamped>& poses, size_t num_samples) {
  std::vector<geometry_msgs::msg::PoseStamped> resampled_poses;
  if (poses.empty()) {
      return resampled_poses;
  }

  // Step 1: 计算累积弧长
  std::vector<double> lengths;
  lengths.push_back(0);
  for (size_t i = 1; i < poses.size(); ++i) {
      double dx = poses[i].pose.position.x - poses[i - 1].pose.position.x;
      double dy = poses[i].pose.position.y - poses[i - 1].pose.position.y;
      double segment_length = std::hypot(dx, dy);
      lengths.push_back(lengths.back() + segment_length);
  }

  double total_length = lengths.back();

  // Step 2: 均匀采样
  double step_length = total_length / (num_samples - 1);

  for (size_t i = 0; i < num_samples; ++i) {
      double target_length = i * step_length;

      // 找到目标弧长所在的段
      size_t index = 0;
      while (index < lengths.size() - 1 && lengths[index + 1] < target_length) {
          index++;
      }

      // 插值生成新点
      double t = (target_length - lengths[index]) / (lengths[index + 1] - lengths[index]);
      double x = lerp(poses[index].pose.position.x, poses[index + 1].pose.position.x, t);
      double y = lerp(poses[index].pose.position.y, poses[index + 1].pose.position.y, t);
      double theta = angle_lerp(
          get_yaw_from_quaternion(poses[index].pose.orientation),
          get_yaw_from_quaternion(poses[index + 1].pose.orientation),
          t
      );
      // RCLCPP_INFO(logger_, "yaw i: %f, yaw i+1: %f, t: %f theta:%f", get_yaw_from_quaternion(poses[index].pose.orientation),
      //                                                     get_yaw_from_quaternion(poses[index + 1].pose.orientation),
      //                                                     t,theta);
      // 创建新的 PoseStamped
      geometry_msgs::msg::PoseStamped pose_stamped;
      pose_stamped.header = poses[index].header; // 保留原始 header
      pose_stamped.pose.position.x = x;
      pose_stamped.pose.position.y = y;
      pose_stamped.pose.position.z = 0.0; // 假设 2D 路径

      // 将 theta 转换为四元数
      tf2::Quaternion quat;
      quat.setRPY(0, 0, theta); // 设置 roll=0, pitch=0, yaw=theta
      pose_stamped.pose.orientation = tf2::toMsg(quat);

      resampled_poses.push_back(pose_stamped);
  }

  return resampled_poses;
}

void LqrController::setPlan(const nav_msgs::msg::Path & path)
{
  RCLCPP_INFO(logger_,"setting new plan");
  // get global path
  global_plan_ = path;
  // remove dulicated point so that we can use the path to find cusp index
  removeDuplicatedPathPoint(global_plan_);
  // get cusp index list
  cusp_index_ = find_cusp(global_plan_);
  // cut segament based on cusp index
  path_segment_.clear();
  for(size_t i=0;i<cusp_index_.size()-1;i++){
    nav_msgs::msg::Path path_segment,unsmoothed_path;
    for(int j=cusp_index_[i];j<=cusp_index_[i+1];j++){
      unsmoothed_path.poses.push_back(global_plan_.poses[j]);
    }
    path_segment.poses = resample_path(unsmoothed_path.poses, unsmoothed_path.poses.size());
    path_segment.header.frame_id = global_plan_.header.frame_id;
    path_segment.header.stamp = clock_->now();
    path_segment_.push_back(path_segment);
  }
  current_tracking_path_segment_ = 0;
  encounter_obst_moment_logged_ = false;
}

nav_msgs::msg::Path LqrController::grep_path_in_local_costmap(
  const geometry_msgs::msg::PoseStamped & robot_pose,int &path_offset)
{
  nav_msgs::msg::Path local_path;
  local_path.header = global_plan_.header;

  double costmap_radius = std::max(costmap_->getSizeInMetersX(),costmap_->getSizeInMetersY())/2;
  const double circumscribed_radius = costmap_ros_->getLayeredCostmap()->getCircumscribedRadius();
  int index = Find_target_index(robot_pose,path_segment_[current_tracking_path_segment_]);

  // fill local plan on the back to make curvature smooth
  int count = 0;
  for (int i=index; i>0;--i) {
    auto pose = path_segment_[current_tracking_path_segment_].poses[i];
    double distance = nav2_util::geometry_utils::euclidean_distance(pose,robot_pose);
    if(distance>(costmap_radius - 2 * circumscribed_radius)){
      break;
    }else{
      local_path.poses.insert(local_path.poses.begin(),(pose));
      path_offset = i;
      count++;
      if(count > robot_search_pose_dist_) {
        break;
      }
    }
  }

  // fill local plan on the front
  for(size_t i=index;i<path_segment_[current_tracking_path_segment_].poses.size();++i){
    auto pose = path_segment_[current_tracking_path_segment_].poses[i];
    double distance = nav2_util::geometry_utils::euclidean_distance(pose,robot_pose);
    if(distance>(costmap_radius - circumscribed_radius)){
      break;
    }else{
      local_path.poses.push_back(pose);
    }
  }

  local_path.header.frame_id = path_segment_[current_tracking_path_segment_].header.frame_id;
  local_path.header.stamp = rclcpp::Time();
  return local_path;
}

int LqrController::Find_target_index(
  const geometry_msgs::msg::PoseStamped & state,
  nav_msgs::msg::Path &local_path)
{
  double min = abs(sqrt(pow(state.pose.position.x - local_path.poses[0].pose.position.x, 2) + pow(state.pose.position.y - local_path.poses[0].pose.position.y, 2)));
  int index = 0;
  for (size_t i = 0; i < local_path.poses.size(); i++)
  {
    double d = euclidean_distance(state,local_path.poses[i]);
    if (d < min)
    {
      min = d;
      index = i;
    }
  }
  // check if the next point is closer, if so, move to the next point
  if (((size_t)index + 1) < local_path.poses.size())
  {
    double L_1 = euclidean_distance(state,local_path.poses[index + 1]);
    double L_ = euclidean_distance(local_path.poses[index],local_path.poses[index+1]);
    if (L_1 < L_)
    {
      index += 1;
    }
  }
  return index;
}

vector<double> LqrController::get_path_obst_distance(const nav_msgs::msg::Path &path,const geometry_msgs::msg::PoseStamped &robot_pose)
{
  vector<double> distance_list;
  nav_msgs::msg::Path obst_free_path;
  const double circumscribed_radius = costmap_ros_->getLayeredCostmap()->getCircumscribedRadius();
  
  // find obstacle index in path
  int obst_index = -1;
  
  for(size_t i = 0; i < path.poses.size(); i++){
    // push back pose
    // check if pose is outside of costmap, if pose is outside of costmap, it will return wrong cost
    double check_pose_distance = nav2_util::geometry_utils::euclidean_distance(path.poses[i],robot_pose) + 2 * circumscribed_radius;
    // RCLCPP_INFO(logger_,"checking  (%lf,%lf) dist:%f",path.poses[i].pose.position.x,path.poses[i].pose.position.y,check_pose_distance);
    if( check_pose_distance >= min(costmap_->getSizeInMetersX(),costmap_->getSizeInMetersY())/2){
      // RCLCPP_INFO(logger_,"current checking pose (%lf,%lf) is out of range",path.poses[i].pose.position.x,path.poses[i].pose.position.y);
      break;
    }
    
    obst_free_path.poses.push_back(path.poses[i]);
    double yaw = get_yaw_from_quaternion(path.poses[i].pose.orientation);
    // check ith footprint cost, if cost is equal to lethal, then that is the obstacle
    double footprint_cost = collision_checker_->footprintCostAtPose(path.poses[i].pose.position.x, path.poses[i].pose.position.y, yaw,costmap_ros_->getRobotFootprint());
    if (footprint_cost == static_cast<double>(nav2_costmap_2d::NO_INFORMATION) && costmap_ros_->getLayeredCostmap()->isTrackingUnknown())
    {
      RCLCPP_WARN(logger_, "Footprint cost is unknown, collision check failed");
    }else if(footprint_cost > static_cast<double>(nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE)){
      obst_index = i;
      publish_collision_polygon(path.poses[i]);
      break;
    }
  }

  // find cost
  RCLCPP_DEBUG(logger_, "Obstacle index: %d", obst_index);
  if(obst_index!=-1){
    for(uint32_t i = 0; i < path.poses.size(); i++){
      double distance = 0;
      if(i<(uint32_t)obst_index){
        distance = nav2_util::geometry_utils::calculate_path_length(obst_free_path, i);
      }
      distance_list.push_back(distance);
    }
  }else{
    for(uint32_t i = 0; i < path.poses.size(); i++){
      distance_list.push_back(-1);
    }
  }
  return distance_list;
}

void LqrController::remove_duplicated_points(vector<waypoint>& points){
  for(size_t i=0;i<points.size()-1;i++){
    if(points[i].x == points[i+1].x && points[i].y == points[i+1].y && points[i+1].yaw == points[i+1].yaw){
      points.erase(points.begin()+i);
    }
  }
}

vector<vector<int>> LqrController::find_curve(vector<vector<double>>&K_list,double bound){
  vector<vector<int>> result;
    int n = K_list.size();
    int start_index = -1; // 当前连续段的起始索引，-1表示未开始

    for (int i = 0; i < n; ++i) {
        if (abs(K_list[i][0]) > bound) {
            // 如果当前元素符合条件且未开始记录，则标记起始索引
            if (start_index == -1) {
                start_index = i;
            }
        } else {
            // 如果当前元素不符合条件但之前有连续段，则保存该段
            if (start_index != -1) {
                vector<int> segment;
                for (int j = start_index; j < i; ++j) {
                    segment.push_back(j);
                }
                result.push_back(segment);
                start_index = -1;
            }
        }
    }

    // 处理末尾可能剩余的连续段
    if (start_index != -1) {
        vector<int> segment;
        for (int j = start_index; j < n; ++j) {
            segment.push_back(j);
        }
        result.push_back(segment);
    }
    return result;
}

// 内部优化的前向优化（从指定点向前）
void LqrController::internalForwardOptimize(vector<double>& speeds, const vector<double>& distances, 
                             double deacc_max, int start_index, int end_index) {
    for (int i = start_index - 1; i >= end_index; --i) {
        double v_next = speeds[i + 1];
        double distance = distances[i];
        double v_max = sqrt(v_next * v_next + 2 * deacc_max * distance);
        speeds[i] = min(speeds[i], v_max);
    }
}

// 内部优化的后向优化（从指定点向后）
void LqrController::internalBackwardOptimize(vector<double>& speeds, const vector<double>& distances, 
                              double acc_max, int start_index, int end_index) {
    for (int i = start_index + 1; i <= end_index; ++i) {
        double v_prev = speeds[i - 1];
        double distance = distances[i - 1];
        double v_max = sqrt(v_prev * v_prev + 2 * acc_max * distance);
        speeds[i] = min(speeds[i], v_max);
    }
}

// 优化弯道内部的速度
void LqrController::optimizeCurveInternalSpeeds(vector<double>& speeds, const vector<double>& distances, 
                                double acc_max, double deacc_max, 
                                const vector<vector<int>>& curve_indices, 
                                const vector<int>& fixed_points) {
    for (size_t curve_idx = 0; curve_idx < curve_indices.size(); ++curve_idx) {
        const auto& curve = curve_indices[curve_idx];
        if (curve.empty()) continue;

        // 获取当前弯道的固定点
        int fixed_index = fixed_points[curve_idx];
        int curve_start = curve.front();  // 弯道起点
        int curve_end = curve.back();    // 弯道终点

        // 从固定点向前优化到弯道起点
        internalForwardOptimize(speeds, distances, deacc_max, fixed_index, curve_start);

        // 从固定点向后优化到弯道终点
        internalBackwardOptimize(speeds, distances, acc_max, fixed_index, curve_end);
    }
}

// 前向优化（从指定点向前）
void LqrController::forwardOptimize(vector<double>& speeds, const vector<double>& distances, 
                     double deacc_max, int start_index) {
    for (int i = start_index - 1; i >= 0; --i) {
        double v_next = speeds[i + 1];
        double distance = distances[i];
        double v_max = sqrt(v_next * v_next + 2 * deacc_max * distance);
        speeds[i] = min(speeds[i], v_max);

        // 如果当前点与前一个点的加速度已经在范围内，则停止优化
        if (i > 0) {
            double acc = (speeds[i] - speeds[i - 1]) / distances[i - 1];
            if (abs(acc) <= deacc_max) {
                break;
            }
        }
    }
}

// 后向优化（从指定点向后）
void LqrController::backwardOptimize(vector<double>& speeds, const vector<double>& distances, 
                      double acc_max, int start_index) {
    for (size_t i = start_index + 1; i < speeds.size(); ++i) {
        double v_prev = speeds[i - 1];
        double distance = distances[i - 1];
        double v_max = sqrt(v_prev * v_prev + 2 * acc_max * distance);
        speeds[i] = min(speeds[i], v_max);

        // 如果当前点与后一个点的减速度已经在范围内，则停止优化
        if (i < speeds.size() - 1) {
            double deacc = (speeds[i] - speeds[i + 1]) / distances[i];
            if (abs(deacc) <= acc_max) {
                break;
            }
        }
    }
}

// 优化速度列表
void LqrController::optimizeCurveSpeeds(vector<double>& speeds, const vector<double>& distances, 
                         double acc_max, double deacc_max, 
                         const vector<vector<int>>& curve_indices) {
    for (const auto& curve : curve_indices) {
        if (curve.empty()) continue;

        // 对弯道的第一个点进行前向优化
        int first_index = curve.front();
        forwardOptimize(speeds, distances, deacc_max, first_index);

        // 对弯道的最后一个点进行后向优化
        int last_index = curve.back();
        backwardOptimize(speeds, distances, acc_max, last_index);
    }
}

int LqrController::findMinAbsIndex(const std::vector<double>& max_v_curvature_list, const std::vector<int>& curve_index) {
  double minAbsValue = std::numeric_limits<double>::max();
  int minIndex = -1;

  for (int index : curve_index) {
      double absValue = std::abs(max_v_curvature_list[index]);
      if (absValue < minAbsValue) {
          minAbsValue = absValue;
          minIndex = index;
      }
  }

  return minIndex;
}

void LqrController::publish_collision_polygon(const geometry_msgs::msg::PoseStamped& pose){
  Matrix4x4 T_map_collision_pose;
  double yaw = get_yaw_from_quaternion(pose.pose.orientation);
  T_map_collision_pose << cos(yaw),-sin(yaw),0,pose.pose.position.x,
                         sin(yaw),cos(yaw),0,pose.pose.position.y,
                         0,0,1,0,
                         0,0,0,1;
  auto polygon = costmap_ros_->getRobotFootprintPolygon();
  geometry_msgs::msg::PolygonStamped collision_polygon;
  for(auto polygon_point : polygon.points){
    Eigen::Vector4d point;
    point << polygon_point.x, polygon_point.y, 0, 1;
    auto transformed_point = T_map_collision_pose * point;
    geometry_msgs::msg::Point32 transformed_point_msg;
    transformed_point_msg.x = transformed_point(0);
    transformed_point_msg.y = transformed_point(1);
    collision_polygon.polygon.points.push_back(transformed_point_msg);
  }
  collision_polygon.header.frame_id = pose.header.frame_id;
  collision_polygon.header.stamp = rclcpp::Time();
  collision_polygon_pub_->publish(collision_polygon);
}

bool LqrController::determin_backward(vehicleState &state,vector<waypoint>& wp){
  bool backward_motion = false;
  Matrix4x4 T_map_pn,T_map_pnp1;
  float next_dir = 0;
  if(wp.size()>1){
    T_map_pn << cos(wp[0].yaw), -sin(wp[0].yaw) ,0 , wp[0].x,
              sin(wp[0].yaw), cos(wp[0].yaw), 0 , wp[0].y,
              0, 0, 1, 0,
              0, 0, 0, 1;
    T_map_pnp1 << cos(wp[1].yaw), -sin(wp[1].yaw) ,0 , wp[1].x,
              sin(wp[1].yaw), cos(wp[1].yaw), 0 , wp[1].y,
              0, 0, 1, 0,
              0, 0, 0, 1;
  }else{
    T_map_pn << cos(state.yaw), -sin(state.yaw) ,0 , state.x,
                sin(state.yaw), cos(state.yaw), 0 , state.y,
                0, 0, 1, 0,
                0, 0, 0, 1;
    T_map_pnp1 << cos(wp[0].yaw), -sin(wp[0].yaw) ,0 , wp[0].x,
                  sin(wp[0].yaw), cos(wp[0].yaw), 0 , wp[0].y,
                  0, 0, 1, 0,
                  0, 0, 0, 1;
  }
  Matrix4x4 T_pn_pnp1 = T_map_pn.inverse() * T_map_pnp1;
  next_dir = atan2(T_pn_pnp1(1,3), T_pn_pnp1(0,3));          
  if(next_dir < -M_PI_2 || next_dir > M_PI_2){
    backward_motion = true;
  }
  return backward_motion;
}

vector<double> LqrController::optimizeCurveConstraints(
  const vector<waypoint>& wp,
  vector<double>& max_v_curvature_list,
   vector<vector<double>>& curvature_list,
  double max_lin_acc_,
  double min_lin_deacc_)
{
  vector<int> index_set;
  vector<vector<int>> curve_index_list = find_curve(curvature_list, 0.1);

  if (!curve_index_list.empty()) {
    for (const auto& curve_index : curve_index_list) {
      index_set.push_back(findMinAbsIndex(max_v_curvature_list, curve_index));
    }

    vector<double> distance_list;
    for (size_t i = 1; i < wp.size(); i++) {
      distance_list.push_back(hypot(wp[i].x - wp[i - 1].x, wp[i].y - wp[i - 1].y));
    }

    if (!index_set.empty()) {
      optimizeCurveInternalSpeeds(max_v_curvature_list, distance_list, max_lin_acc_, min_lin_deacc_, curve_index_list, index_set);
      optimizeCurveSpeeds(max_v_curvature_list, distance_list, max_lin_acc_, min_lin_deacc_, curve_index_list);
    }
  }

  return max_v_curvature_list;
}

vector<double> LqrController::get_speed_profile(vehicleState &state,
                          float fv_max,
                          float bv_max,
                          float v_min,
                          float max_lateral_accel,
                          vector<waypoint>& wp,
                          vector<vector<double>>& curvature_list,
                          vector<double> &distance_to_obst,
                          int path_offset){
  vector<double> sp(wp.size());
  vector<double> max_v_curvature_list(wp.size());
  vector<double> max_v_goal_list(wp.size());
  vector<double> max_v_obstacle_list(wp.size());
 
  // get next point on from or backward direction
  bool backward_motion = determin_backward(state,wp);
  
  // goal constraint
  for (size_t i = 0; i < wp.size(); i++)
  { 
    // goal constrain, the closer to the goal, the slower, with square root of 2 times desired deacc and distance
    double distance_to_goal = nav2_util::geometry_utils::calculate_path_length(path_segment_[current_tracking_path_segment_],i+path_offset);
    double max_v_distance = 0;
    if(wp[i].x == global_plan_.poses.back().pose.position.x &&
      wp[i].y == global_plan_.poses.back().pose.position.y &&
      wp[i].yaw == get_yaw_from_quaternion(global_plan_.poses.back().pose.orientation))
    {
      max_v_distance = 0;
    }else{
      max_v_distance = std::max(sqrt(2*min_lin_deacc_*distance_to_goal),(double)v_min);
    }
    max_v_goal_list[i] = clamp(max_v_distance,-max_bvx_,max_fvx_);
  }

  // curvature constraint
  for(size_t i=0;i<wp.size();i++){
    // construct curvature constraint
    double K = curvature_list[i][0];
    double max_v_curvature = std::sqrt(max_lateral_accel / std::abs(K));
    max_v_curvature_list[i] = clamp(max_v_curvature,-max_bvx_,max_fvx_);
  }
  if(!max_v_curvature_list.empty())
  {
    max_v_curvature_list = optimizeCurveConstraints(wp,max_v_curvature_list,curvature_list,max_lin_acc_,min_lin_deacc_);
  }

  // obstacle constraint
  // if obstacle in path is in slow region, slow down, if it is in stop region, stop
  for (size_t i = 0; i < wp.size(); i++){
    double max_v_obst = backward_motion?bv_max:fv_max;
    if(use_obstacle_stopping_ == true){
      if(distance_to_obst[i] < obst_stop_dist_ && distance_to_obst[i]>0){
        max_v_obst = 0;
      }else if(distance_to_obst[i] < obst_slow_dist_ && distance_to_obst[i] > obst_stop_dist_){
        max_v_obst = sqrt(2*min_lin_deacc_*(distance_to_obst[i] - obst_stop_dist_));
      }
    }
    max_v_obstacle_list[i] = max_v_obst;
  }

  // get speed profile
  for (size_t i = 0; i < wp.size(); i++){
    if(!backward_motion){
      sp[i] = std::min({(double)fv_max,max_v_curvature_list[i],max_v_goal_list[i],max_v_obstacle_list[i]});   // forward motion profile
    }else{
      sp[i] = -std::min({(double)bv_max,max_v_curvature_list[i],max_v_goal_list[i],max_v_obstacle_list[i]}); // backward motion profile
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

vector<vector<double>> LqrController::get_kappa_d_kappa(vector<waypoint>& wp){
  vector<vector<double>> kappa_d_kappa; // [kappa ,d_kappa; ... ]
  vector<double> kappa_list;
  vector<double> kappa_rate_list;
  // get kappa list
  if(wp.size()>3){
    for(size_t i=0;i<wp.size();i++){
      double K;
      K = cal_K(wp, i);
      kappa_list.push_back(K);
    }
    // get kappa rate
    kappa_rate_list.push_back(0);
    for(size_t i=1;i<wp.size();i++){
      if(i == wp.size()-1){
        kappa_rate_list.push_back(kappa_rate_list[i-1]*2 - kappa_rate_list[i-2]);
      }else{
        double ds = 0;
        ds = hypot(wp[i-1].x - wp[i+1].x,wp[i-1].y - wp[i+1].y);
        double dK = kappa_list[i];
        dK = (kappa_list[i+1] - kappa_list[i-1])/(2*ds);

        double dk_ds = 0;
        if(ds!=0){
          dk_ds = dK/ds;
        }
        kappa_rate_list.push_back(dk_ds);
      }
      // RCLCPP_INFO(logger_,"%ld %ld",i,wp.size());
    }
    for(size_t i=0; i<wp.size(); i++){
      kappa_d_kappa.push_back({kappa_list[i],kappa_rate_list[i]});
    }
  }
  else{  // TODO: add kappa list when wp is less than 3
    for(size_t i=0;i<wp.size();i++)
      kappa_d_kappa.push_back({0,0});
  }
  return kappa_d_kappa;
}

geometry_msgs::msg::PoseStamped LqrController::interpolate_pose(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::PoseStamped & a,
  const geometry_msgs::msg::PoseStamped & b,
  vector<double> & speeds,double &ref_interpolated_speed,
  vector<double> & kappas,double &kappa_interpolated)
{
  geometry_msgs::msg::PoseStamped result;
  result.header = pose.header;  // 使用车辆自身的时间与坐标系

  // 1. 计算投影点坐标
  double dx = b.pose.position.x - a.pose.position.x;
  double dy = b.pose.position.y - a.pose.position.y;
  double len_sq = dx * dx + dy * dy;

  double t = 0.0;
  if (len_sq > 0.0) {
    t = ((pose.pose.position.x - a.pose.position.x) * dx +
         (pose.pose.position.y - a.pose.position.y) * dy) / len_sq;
    t = std::clamp(t, 0.0, 1.0);
  }

  result.pose.position.x = a.pose.position.x + t * dx;
  result.pose.position.y = a.pose.position.y + t * dy;
  result.pose.position.z = a.pose.position.z + t * (b.pose.position.z - a.pose.position.z);

  // 2. 方向取 a 和 b 的 yaw 均值
  double yaw_a = tf2::getYaw(a.pose.orientation);
  double yaw_b = tf2::getYaw(b.pose.orientation);
  double angle_yaw = angle_lerp(yaw_a,yaw_b, t);//yaw_a + t*(yaw_b - yaw_a);
  ref_interpolated_speed = lerp(speeds[0],speeds[1],t);
  kappa_interpolated = lerp(kappas[0],kappas[1],t);
  // RCLCPP_INFO(logger_,"yaw b :%f yaw a :%f, angle_yaw:%f ,t :%f",yaw_b,yaw_a,angle_yaw,t);
  // double avg_yaw = angles::normalize_angle((yaw_a + yaw_b) / 2.0);

  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, angle_yaw);
  result.pose.orientation = tf2::toMsg(q);
//  RCLCPP_INFO(logger_,"robot pose (%f,%f) a (%f,%f) b (%f,%f) result (%f,%f) ",pose.pose.position.x,pose.pose.position.y,a.pose.position.x,a.pose.position.y,b.pose.position.x,b.pose.position.y,result.pose.position.x,result.pose.position.y);
  return result;
}

geometry_msgs::msg::TwistStamped LqrController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & speed,
  nav2_core::GoalChecker * goal_checker)
{
  std::lock_guard<std::mutex> lock_reinit(mutex_);
  nav2_costmap_2d::Costmap2D * costmap = costmap_ros_->getCostmap();
  std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*(costmap->getMutex()));
  std_msgs::msg::UInt64MultiArray ErrCode;

  geometry_msgs::msg::TwistStamped cmd_vel;
  cmd_vel.header.stamp = clock_->now();

  static vector<double> kesi_history;

  static double last_control_time = rclcpp::Clock().now().seconds();
  double now = rclcpp::Clock().now().seconds();
  dt_ = now - last_control_time;

  last_control_time = now;
  // no output in the first cycle, only update last control time
  if(dt_>1.0f){
    // RCLCPP_INFO(logger_,"dt over time %lf",dt_);
    return cmd_vel;
  }
    
    
  // get robot coordinate in map frame, due to the argument pose is in odom frame
  geometry_msgs::msg::PoseStamped global_pose;
  transformPose(global_plan_.header.frame_id,pose,global_pose);

  geometry_msgs::msg::Pose pose_tolerance;
  geometry_msgs::msg::Twist vel_tolerance;
  goal_checker->getTolerances(pose_tolerance, vel_tolerance);

  // check cusp distance and switch current cusp
  double dist_to_cusp_point = nav2_util::geometry_utils::euclidean_distance(global_pose,path_segment_[current_tracking_path_segment_].poses.back());

  if(dist_to_cusp_point<inversion_xy_tolerance_ && (size_t)current_tracking_path_segment_<path_segment_.size()-1){
    current_tracking_path_segment_++;
  }

  geometry_msgs::msg::PointStamped cusp_point;
  cusp_point.point.x = path_segment_[current_tracking_path_segment_].poses.back().pose.position.x;
  cusp_point.point.y = path_segment_[current_tracking_path_segment_].poses.back().pose.position.y;
  cusp_point.header.stamp = clock_->now();
  cusp_point.header.frame_id = path_segment_[current_tracking_path_segment_].header.frame_id;
  cusp_pub_->publish(cusp_point);
  
  int path_offset = 0;
  nav_msgs::msg::Path local_plan = grep_path_in_local_costmap(global_pose,path_offset);
  // RCLCPP_INFO(logger_,"local plan size:%ld  cropped local path size: %ld",path_segment_[current_tracking_path_segment_].poses.size(),local_plan.poses.size());
  // if robot is not properly localized, use global plan instead
  if(local_plan.poses.size() == 0){
    RCLCPP_ERROR(logger_,"local plan empty, trying to find waypoint via global plan.");
    local_plan = global_plan_;
  }
  lqr_path_pub_->publish(local_plan);

  // prepare lqr controller 
  std::vector<waypoint> wps;
  robot_state_ = vehicleState{global_pose.pose.position.x,global_pose.pose.position.y,get_yaw_from_quaternion(global_pose.pose.orientation), speed.linear.x,kesi_};

  // get target point to track and publish 
  int target_index = Find_target_index(global_pose,local_plan);

  // get waypoints list
  for(size_t i=0;i<local_plan.poses.size();i++){
    waypoint wp;
    wp.ID = i;
    wp.x = local_plan.poses[i].pose.position.x;
    wp.y = local_plan.poses[i].pose.position.y;
    wp.yaw = get_yaw_from_quaternion(local_plan.poses[i].pose.orientation);
    wps.push_back(wp);
  }
  // remove duplicated points to ensure cusp can be identified correctly
  // TODO: do it in set plan callback
  remove_duplicated_points(wps);

  // compute curvature and apply constraints to speed
  vector<double> obstacle_distance_list = get_path_obst_distance(local_plan,global_pose);
  vector<vector<double>> kappa_d_kappa_list = get_kappa_d_kappa(wps);
  vector<double> sp = get_speed_profile(robot_state_,max_fvx_,max_bvx_,dead_band_speed_,max_lateral_accel_,wps,kappa_d_kappa_list,obstacle_distance_list,path_offset);

  // interpolate pose for smooth curvature and speed profile
  geometry_msgs::msg::PoseStamped target_pose;
  double ref_velocity = sp[target_index];
  double K = kappa_d_kappa_list[target_index][0];
  if(target_index!=0){
    // get speed profile and kappa to interpolate 
    vector<double> sp_interp({sp[target_index-1],sp[target_index]});
    vector<double> kappa_interp({kappa_d_kappa_list[target_index-1][0],kappa_d_kappa_list[target_index][0]});
    target_pose = interpolate_pose(global_pose,local_plan.poses[target_index-1],local_plan.poses[target_index],sp_interp,ref_velocity,kappa_interp,K);
  }else{
    target_pose.pose = local_plan.poses[0].pose;
  }
  // publish tracking pose
  target_pose.header.stamp = rclcpp::Time();
  target_pose.header.frame_id = global_plan_.header.frame_id;
  target_pub_->publish(target_pose);

  waypoint Point({0,target_pose.pose.position.x,target_pose.pose.position.y,get_yaw_from_quaternion(target_pose.pose.orientation)});//wps[target_index];

  // check obstacle encounter moment(if not logged) and compute stopped duration
  if(obstacle_distance_list[target_index]<=obst_stop_dist_ && sp[target_index] == 0 && obstacle_distance_list[target_index]>0){
    RCLCPP_ERROR(logger_,"obstacle too close, stop!");
    ErrCode.data.push_back(100006);
    if(encounter_obst_moment_logged_ == false){
      encounter_obst_moment_logged_ = true;
      encounter_obst_moment_ = clock_->now().seconds();
    }else{
      double dt_obst = clock_->now().seconds() - encounter_obst_moment_;
      RCLCPP_INFO(logger_,"obstacle dt: %lf",dt_obst);
      if(dt_obst>obstacle_timeout_){
        throw nav2_core::PlannerException("obstacle ahead, waited for too long. goal failed.");
      }
    }
  }else if(obstacle_distance_list[target_index]<=obst_slow_dist_ && obstacle_distance_list[target_index]>0 && sp[target_index] != 0){
    RCLCPP_WARN(logger_,"obstacle closing in %f, slowing down!",obstacle_distance_list[target_index]);
    ErrCode.data.push_back(100005);
    encounter_obst_moment_logged_ = false;
  }else{
    encounter_obst_moment_logged_ = false;
  }

  // set reference input
  double kesi = atan2(vehicle_L_ * K, 1);   // reference steer angle

  U U_r;
  U_r.v = ref_velocity; 
  U_r.kesi = kesi*(U_r.v>=0?1:-1);

  // initialize lqr controller parameters
  lqr_controller_->initial(vehicle_L_, dt_, robot_state_, Point, U_r, Q_, R_);

  Matrix5x1 state;
  U control = lqr_controller_->cal_vel(state);//计算输入[v, kesi]
  std_msgs::msg::Float32MultiArray debug_info;
  for (int i = 0; i < 5; i++)
  {
    debug_info.data.push_back(state(i));
  }
  
  if(U_r.v==0)control.v = 0;    // to avoid non zero speed output after optimization

  // mean filter to smooth kesi
  if(use_output_filter_ == true){
    if(kesi_history.size()==4){
      kesi_history.erase(kesi_history.begin());
      kesi_history.push_back(control.kesi);
      control.kesi = std::accumulate(kesi_history.begin(),kesi_history.end(),0.0)/4;
    }else{
      kesi_history.push_back(control.kesi);
    }
  }
  
  double gain_kesi = (control.kesi - kesi_)/dt_;
  double kesi_gain_valid = clamp(gain_kesi,-max_steer_rate_,max_steer_rate_);

  kesi_ = kesi_ + kesi_gain_valid*dt_;

  double vx = clamp(control.v,-max_bvx_,max_fvx_);
  double az = clamp(control.v*tan(control.kesi)/vehicle_L_,-max_wz_,max_wz_);

  if(use_output_filter_ == true){
    cmd_vel.twist.linear.x = vx*0.5 + last_cmd_vel_.linear.x*0.5;
    cmd_vel.twist.angular.z = az*0.5 + last_cmd_vel_.angular.z*0.5;
    // RCLCPP_INFO(logger_,"filter on");
  }else{
    cmd_vel.twist.linear.x = vx;
    cmd_vel.twist.angular.z = az;
    // RCLCPP_INFO(logger_,"filter off");
  }
  
  debug_info.data.push_back(kesi_);
  debug_info.data.push_back(control.v);
  debug_info.data.push_back(U_r.kesi);
  debug_pub_->publish(debug_info);
  last_cmd_vel_.angular.z = az;
  last_cmd_vel_.linear.x = vx;

  // check if robot has reached the goal
  if((size_t)current_tracking_path_segment_ == path_segment_.size()-1){
    double dist_to_goal = nav2_util::geometry_utils::euclidean_distance(global_pose,global_plan_.poses.back());
    // RCLCPP_INFO(logger_,"dist_to_goal: %f",dist_to_goal);
    if(dist_to_goal< min(pose_tolerance.position.x,pose_tolerance.position.y)){
      cmd_vel.twist.linear.x = 0;
      cmd_vel.twist.angular.z= 0;
      RCLCPP_INFO(logger_,"goal reached -- from controller plugin");
    }
  }

  error_code_pub_->publish(ErrCode);
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
      if(name == plugin_name_ + ".max_fvx"){
        if(parameter.as_double() < 0){
          RCLCPP_WARN(logger_,"parameter should be positive, using absolute value instead.");
          max_fvx_ = std::abs(parameter.as_double());
        }else{
          max_fvx_ = parameter.as_double();
        }
      }else if(name == plugin_name_ + ".max_bvx"){
        if(parameter.as_double() < 0){
          RCLCPP_WARN(logger_,"parameter should be positive, using absolute value instead.");
          max_bvx_ = std::abs(parameter.as_double());
        }else{
          max_bvx_ = parameter.as_double();
        }
      }else if(name == plugin_name_ + ".max_wz"){
        if(parameter.as_double() < 0){
          RCLCPP_WARN(logger_,"parameter should be positive, using absolute value instead.");
          max_wz_ = std::abs(parameter.as_double());
        }else{
          max_wz_ = parameter.as_double();
        }
      }else if(name == plugin_name_ + ".max_linear_accel"){
        if(parameter.as_double() < 0){
          RCLCPP_WARN(logger_,"parameter should be positive, using absolute value instead.");
          max_lin_acc_ = std::abs(parameter.as_double());
        }else{
          max_lin_acc_ = parameter.as_double();
        }
      }
      // else if(name == plugin_name_ + ".min_linear_deaccel"){
      //   if(parameter.as_double() < 0){
      //     RCLCPP_WARN(logger_,"parameter should be positive, using absolute value instead.");
      //     min_lin_deacc_ = std::abs(parameter.as_double());
      //   }else{
      //     min_lin_deacc_ = parameter.as_double();
      //   }
      // }
      else if(name == plugin_name_ + ".max_lateral_accel"){
        if(parameter.as_double() < 0){
          RCLCPP_WARN(logger_,"parameter should be positive, using absolute value instead.");
          max_lateral_accel_ = std::abs(parameter.as_double());
        }else{
          max_lateral_accel_ = parameter.as_double();
        }
      }else if(name == plugin_name_ + ".max_w_accel"){
        if(parameter.as_double() < 0){
          RCLCPP_WARN(logger_,"parameter should be positive, using absolute value instead.");
          max_w_acc_ = std::abs(parameter.as_double());
        }else{
          max_w_acc_ = parameter.as_double();
        }
      }else if(name == plugin_name_ + ".dead_band_speed"){
        if(parameter.as_double() < 0){
          RCLCPP_WARN(logger_,"parameter should be positive, using absolute value instead.");
          dead_band_speed_ = std::abs(parameter.as_double());
        }else{
          dead_band_speed_ = parameter.as_double();
        }
      }else if(name == plugin_name_ + ".approach_velocity_scaling_dist"){
        if(parameter.as_double() < 0){
          RCLCPP_WARN(logger_,"parameter should be positive, using absolute value instead.");
          approach_velocity_scaling_dist_ = std::abs(parameter.as_double());
        }else{
          approach_velocity_scaling_dist_ = parameter.as_double();
        }
        min_lin_deacc_ = pow(max_fvx_,2)/(2*approach_velocity_scaling_dist_);
      }else if(name == plugin_name_ + ".patient_encounter_obst"){
        if(parameter.as_double() < 0){
          RCLCPP_WARN(logger_,"parameter should be positive, using absolute value instead.");
          obstacle_timeout_ = std::abs(parameter.as_double());
        }else{
          obstacle_timeout_ = parameter.as_double();
        }
      }else if(name == plugin_name_ + ".path_obst_stop_dist"){
        if(parameter.as_double() < 0){
          RCLCPP_WARN(logger_,"parameter should be positive, using absolute value instead.");
          obst_stop_dist_ = std::abs(parameter.as_double());
        }else{
          obst_stop_dist_ = parameter.as_double();
        }
      }else if(name == plugin_name_ + ".path_obst_slow_dist"){
        if(parameter.as_double() < 0){
          RCLCPP_WARN(logger_,"parameter should be positive, using absolute value instead.");
          obst_slow_dist_ = std::abs(parameter.as_double());
        }else{
          obst_slow_dist_ = parameter.as_double();
        }
      }else if(name == plugin_name_ + ".vehicle_L"){
        if(parameter.as_double() < 0){
          RCLCPP_WARN(logger_,"parameter should be positive, using absolute value instead.");
          vehicle_L_ = std::abs(parameter.as_double());
        }else{
          vehicle_L_ = parameter.as_double();
        }
      }else if(name == plugin_name_ + ".inversion_xy_tolerance"){
        if(parameter.as_double() < 0){
          RCLCPP_WARN(logger_,"parameter should be positive, using absolute value instead.");
          inversion_xy_tolerance_ = std::abs(parameter.as_double());
        }else{
          inversion_xy_tolerance_ = parameter.as_double();
        }
      }else if(name == plugin_name_ + ".latteral_err_penalty"){
        if(parameter.as_double() < 0){
          RCLCPP_WARN(logger_,"parameter should be positive, using absolute value instead.");
          Q_[0] = std::abs(parameter.as_double());
        }else{
          Q_[0] = parameter.as_double();
        }
      }else if(name == plugin_name_ + ".latteral_err_dot_penalty"){
        if(parameter.as_double() < 0){
          RCLCPP_WARN(logger_,"parameter should be positive, using absolute value instead.");
          Q_[1] = std::abs(parameter.as_double());
        }else{
          Q_[1] = parameter.as_double();
        }
      }else if(name == plugin_name_ + ".angle_err_penalty"){
        if(parameter.as_double() < 0){
          RCLCPP_WARN(logger_,"parameter should be positive, using absolute value instead.");
          Q_[2] = std::abs(parameter.as_double());
        }else{
          Q_[2] = parameter.as_double();
        }
      }else if(name == plugin_name_ + ".angle_err_dot_penalty"){
        if(parameter.as_double() < 0){
          RCLCPP_WARN(logger_,"parameter should be positive, using absolute value instead.");
          Q_[3] = std::abs(parameter.as_double());
        }else{
          Q_[3] = parameter.as_double();
        }
      }else if(name == plugin_name_ + ".v_err_penalty"){
        if(parameter.as_double() < 0){
          RCLCPP_WARN(logger_,"parameter should be positive, using absolute value instead.");
          Q_[4] = std::abs(parameter.as_double());
        }else{
          Q_[4] = parameter.as_double();
        }
      }else if(name == plugin_name_ + ".w_effort_penalty"){
        if(parameter.as_double() < 0){
          RCLCPP_WARN(logger_,"parameter should be positive, using absolute value instead.");
          R_[0] = std::abs(parameter.as_double());
        }else{
          R_[0] = parameter.as_double();
        }
      }else if(name == plugin_name_ + ".acc_effort_penalty"){
        if(parameter.as_double() < 0){
          RCLCPP_WARN(logger_,"parameter should be positive, using absolute value instead.");
          R_[1] = std::abs(parameter.as_double());
        }else{
          R_[1] = parameter.as_double();
        }
      }
      obst_speed_control_k_ = (max_fvx_ - dead_band_speed_)/(obst_slow_dist_ - obst_stop_dist_);
      obst_speed_control_b_ = max_fvx_ - obst_speed_control_k_*obst_slow_dist_;
      RCLCPP_INFO(logger_,"parameter %s changed to %f",parameter.get_name().c_str(),abs(parameter.as_double()));
    } else if (type == ParameterType::PARAMETER_BOOL) {
      if(name == plugin_name_ + ".use_obstacle_stopping"){
        use_obstacle_stopping_ = parameter.as_bool();
      }else if(name == plugin_name_ + ".use_output_filter"){
        use_output_filter_ = parameter.as_bool();
      }
      // RCLCPP_INFO(logger_,"parameter %s changed to %d",parameter.get_name().c_str(),parameter.as_bool());
    } else if (type == ParameterType::PARAMETER_INTEGER) {
      if(name == plugin_name_ + ".robot_search_pose_dist"){
        use_obstacle_stopping_ = parameter.as_int();
      }
      RCLCPP_INFO(logger_,"parameter %s changed to %ld",parameter.get_name().c_str(),parameter.as_int());
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