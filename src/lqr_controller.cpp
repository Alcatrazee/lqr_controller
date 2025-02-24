#include <algorithm>
#include <geometry_msgs/msg/detail/point_stamped__struct.hpp>
#include <geometry_msgs/msg/detail/pose_stamped__struct.hpp>
#include <geometry_msgs/msg/detail/twist__struct.hpp>
#include <iterator>
#include <nav2_util/geometry_utils.hpp>
#include <nav_msgs/msg/detail/path__struct.hpp>
#include <rclcpp/clock.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/time.hpp>
#include <string>
#include <iostream>
#include <memory>
#include <vector>
#include <cmath>
#include "lqr_controller/lqr_controller.hpp"
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

  // speed related
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".max_fvx", rclcpp::ParameterValue(0.50));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".max_bvx", rclcpp::ParameterValue(0.5));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".max_wz", rclcpp::ParameterValue(1.00));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".max_linear_accel", rclcpp::ParameterValue(0.50));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".max_lateral_accel", rclcpp::ParameterValue(0.5));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".max_w_accel", rclcpp::ParameterValue(0.4));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".dead_band_speed", rclcpp::ParameterValue(0.1));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".approach_velocity_scaling_dist", rclcpp::ParameterValue(1.0));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".approach_velocity_gain", rclcpp::ParameterValue(1.0));
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
  node->get_parameter(plugin_name_ + ".approach_velocity_gain", approach_v_gain_);
  node->get_parameter(plugin_name_ + ".use_obstacle_stopping", use_obstacle_stopping_);
  node->get_parameter(plugin_name_ + ".patient_encounter_obst", obstacle_timeout_);
  node->get_parameter(plugin_name_ + ".path_obst_stop_dist", obst_stop_dist_);
  node->get_parameter(plugin_name_ + ".path_obst_slow_dist", obst_slow_dist_);
  node->get_parameter(plugin_name_ + ".vehicle_L", vehicle_L_);
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

  global_path_pub_ = node->create_publisher<nav_msgs::msg::Path>("received_global_plan", 1);
  lqr_path_pub_ = node->create_publisher<nav_msgs::msg::Path>("lqr_path", 1);
  target_pub_ = node->create_publisher<geometry_msgs::msg::PoseStamped>("tracking_target", 10);
  cusp_pub_ = node->create_publisher<geometry_msgs::msg::PointStamped>("cusp", 10);

  // initialize collision checker and set costmap
  collision_checker_ = std::make_unique<nav2_costmap_2d::FootprintCollisionChecker<nav2_costmap_2d::Costmap2D *>>(costmap_);
  collision_checker_->setCostmap(costmap_);
  transform_tolerance_ = tf2::durationFromSec(0.5);

  encounter_obst_moment_logged_ = false;
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
  cusp_pub_->on_activate();
  // Remove these lines if publishers aren't needed
  
  // target_arc_pub_->on_activate();
  
  
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

void LqrController::removeDuplicatedPathPoint(nav_msgs::msg::Path & path)
{
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

void LqrController::setPlan(const nav_msgs::msg::Path & path)
{
  // get global path
  global_plan_ = path;
  // remove dulicated point so that we can use the path to find cusp index
  removeDuplicatedPathPoint(global_plan_);
  // get cusp index list
  cusp_index_ = find_cusp(global_plan_);
  // cut segament based on cusp index
  path_segment_.clear();
  for(size_t i=0;i<cusp_index_.size()-1;i++){
    nav_msgs::msg::Path path_segment;
    for(int j=cusp_index_[i];j<=cusp_index_[i+1];j++){
      path_segment.poses.push_back(global_plan_.poses[j]);
    }
    path_segment.header.frame_id = global_plan_.header.frame_id;
    path_segment.header.stamp = clock_->now();
    path_segment_.push_back(path_segment);
  }
  current_tracking_path_segment_ = 0;
  encounter_obst_moment_logged_ = false;
  // int cusp_num = 
}

double get_yaw(const geometry_msgs::msg::PoseStamped & pose) {
  tf2::Quaternion q(pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w);
  tf2::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
  yaw = fmod(yaw + 3 * M_PI, 2 * M_PI) - M_PI;
  return yaw;
}

nav_msgs::msg::Path LqrController::grep_path_in_local_costmap(const geometry_msgs::msg::PoseStamped & robot_pose,int &path_offset){
  nav_msgs::msg::Path local_path;
  local_path.header = global_plan_.header;
  double costmap_radius = std::max(costmap_->getSizeInMetersX(),costmap_->getSizeInMetersY())/2;
  const double circumscribed_radius = costmap_ros_->getLayeredCostmap()->getCircumscribedRadius();
  int index = Find_target_index(robot_pose,path_segment_[current_tracking_path_segment_]);

  // fill local plan on the front
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
      if(count > 3)
        break;
    }
  }
  // local_path.poses.insert(local_path.poses.begin(),path_segment_[current_tracking_path_segment_].poses[index]);
  // path_offset = index;

  // fill local 
  for(size_t i=index;i<path_segment_[current_tracking_path_segment_].poses.size();++i){
    auto pose = path_segment_[current_tracking_path_segment_].poses[i];
    double distance = nav2_util::geometry_utils::euclidean_distance(pose,robot_pose);
    if(distance>(costmap_radius - circumscribed_radius)){
    // if(std::hypot(pose.pose.position.x - robot_pose.pose.position.x, pose.pose.position.y - robot_pose.pose.position.y)>costmap_radius){
      break;
    }else{
      local_path.poses.push_back(pose);
    }
  }

  local_path.header.frame_id = path_segment_[current_tracking_path_segment_].header.frame_id;
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

vector<double> LqrController::get_path_obst_distance(const nav_msgs::msg::Path &path,const geometry_msgs::msg::PoseStamped &robot_pose)
{
  vector<double> distance_list;
  uint32_t path_length = path.poses.size();
  nav_msgs::msg::Path obst_free_path;
  // find obstacle index in path
  int obst_index = -1;
  const double circumscribed_radius = costmap_ros_->getLayeredCostmap()->getCircumscribedRadius();
  // RCLCPP_INFO(logger_, "circumscribed_radius radius: %f circum %f ",circumscribed_radius,costmap_ros_->getLayeredCostmap()->getCircumscribedRadius());
  for(size_t i = 0; i < path_length; i++){
    // push back pose
    // check if pose is outside of costmap, if pose is outside of costmap, it will return wrong cost
    double check_pose_distance = nav2_util::geometry_utils::euclidean_distance(path.poses[i],robot_pose) + 2 * circumscribed_radius;
    // RCLCPP_INFO(logger_,"checking  (%lf,%lf) dist:%f",path.poses[i].pose.position.x,path.poses[i].pose.position.y,check_pose_distance);
    if( check_pose_distance >= min(costmap_->getSizeInMetersX(),costmap_->getSizeInMetersY())/2){
      // RCLCPP_INFO(logger_,"current checking pose (%lf,%lf) is out of range",path.poses[i].pose.position.x,path.poses[i].pose.position.y);
      break;
    }
    
    obst_free_path.poses.push_back(path.poses[i]);
    double yaw = get_yaw(path.poses[i]);
    // check ith footprint cost, if cost is equal to lethal, then that is the obstacle
    double footprint_cost = collision_checker_->footprintCostAtPose(path.poses[i].pose.position.x, path.poses[i].pose.position.y, yaw,costmap_ros_->getRobotFootprint());
    // RCLCPP_INFO(logger_,"cost at %ld is %f",i,footprint_cost);
    if (footprint_cost == static_cast<double>(nav2_costmap_2d::NO_INFORMATION) && costmap_ros_->getLayeredCostmap()->isTrackingUnknown())
    {
      RCLCPP_WARN(logger_, "Footprint cost is unknown, collision check failed");
    }else if(footprint_cost == static_cast<double>(nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE) || 
              footprint_cost == static_cast<double>(nav2_costmap_2d::LETHAL_OBSTACLE)){
      obst_index = i;
      // RCLCPP_INFO(logger_, "found obstacle in pose (%f,%f) ",obst_free_path.poses.back().pose.position.x, obst_free_path.poses.back().pose.position.y);
      break;
    }
  }

  // find cost
  if(obst_index!=-1){
    for(uint32_t i = 0; i < path_length; i++){
      double distance = 0;
      if(i<(uint32_t)obst_index){
        distance = nav2_util::geometry_utils::calculate_path_length(obst_free_path, i);
      }
      distance_list.push_back(distance);
    }
  }else{
    for(uint32_t i = 0; i < path_length; i++){
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

vector<double> LqrController::get_speed_profile(vehicleState /*state*/,float fv_max,float /*bv_max*/,float v_min,float max_lateral_accel,vector<waypoint>& wp,vector<double>& curvature_list,vector<double> &distance_to_obst,int path_offset){
  vector<double> sp(wp.size());

  for (size_t i = 0; i < wp.size(); i++)
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
    double K = 0;
    if(wp.size()>3){
      if(i >= wp.size()-3){
        K = curvature_list[i-1] + (curvature_list[i-1]-curvature_list[i-2]);
        RCLCPP_INFO(logger_,"last curvature is %f %f %f",K,curvature_list[i-1],curvature_list[i-2]);
      }else{
        K = cal_K(wp, i);
      }
      
    }

    if(isnan(K)){
      cerr << "K is nan, index: " << i << endl;
    }
    curvature_list.push_back(K);
    double max_v_curvature = std::sqrt(max_lateral_accel / std::abs(curvature_list[i]));

    // goal constrain 
    // TODO: rewrite here to compute distance in path length
    // double distance_to_goal = std::hypot(wp[i].x - path_segment_[current_tracking_path_segment_].poses.back().pose.position.x,
    //                                    wp[i].y - path_segment_[current_tracking_path_segment_].poses.back().pose.position.y);
    double distance_to_goal = nav2_util::geometry_utils::calculate_path_length(path_segment_[current_tracking_path_segment_],i+path_offset);
    double max_v_distance = std::max(approach_v_gain_*distance_to_goal,(double)v_min);
    // sp.back() = max_v_distance; // set last point speed profile as dynamic

    // RCLCPP_INFO(logger_, "%ldth (%lf,%lf)distance to goal(%lf,%lf): %f",i,wp[i].x,wp[i].y,
    //   path_segment_[current_tracking_path_segment_].poses.back().pose.position.x,
    //   path_segment_[current_tracking_path_segment_].poses.back().pose.position.y,distance_to_goal);

    // obstacle constraint
    
    double max_v_obst = fv_max;
    if(distance_to_obst[i] < obst_stop_dist_ && distance_to_obst[i]>0){
      max_v_obst = 0;
    }else if(distance_to_obst[i] < obst_slow_dist_ && distance_to_obst[i] > obst_stop_dist_){
      max_v_obst = obst_speed_control_k_*distance_to_obst[i]+obst_speed_control_b_;
    }
    
    // RCLCPP_INFO(logger_, "distance to obst: %ld %f speed:%f",i,distance_to_obst[i],max_v_obst);

    // get speed
    if(!backward_motion){
      sp[i] = std::min({(double)fv_max,max_v_curvature,max_v_distance,max_v_obst});   // forward motion profile
    }else{
      sp[i] = -std::min({(double)fv_max,max_v_curvature,max_v_distance,max_v_obst}); // backward motion profile
    }
    // RCLCPP_INFO(logger_, "speed profile:%ld %f %f %f",i, sp[i],K,distace_to_obst[i]);
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
  nav2_core::GoalChecker * goal_checker)
{
  std::lock_guard<std::mutex> lock_reinit(mutex_);
  nav2_costmap_2d::Costmap2D * costmap = costmap_ros_->getCostmap();
  std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*(costmap->getMutex()));

  geometry_msgs::msg::TwistStamped cmd_vel;
  static double last_control_time = rclcpp::Clock().now().seconds();
  // get robot coordinate in map frame, due to the argument pose is in odom frame
  geometry_msgs::msg::PoseStamped global_pose;
  transformPose(global_plan_.header.frame_id,pose,global_pose);

  geometry_msgs::msg::Pose pose_tolerance;
  geometry_msgs::msg::Twist vel_tolerance;
  goal_checker->getTolerances(pose_tolerance, vel_tolerance);

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
  // 
  int path_offset = 0;
  nav_msgs::msg::Path local_plan = grep_path_in_local_costmap(global_pose,path_offset);
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
  vector<double> obstacle_distance_list = get_path_obst_distance(local_plan,global_pose);
  vector<double> sp = get_speed_profile(robot_state_,max_fvx_,max_bvx_,dead_band_speed_,max_lateral_accel_,wps,k_list,obstacle_distance_list,path_offset);
  if(obstacle_distance_list[target_index]<=obst_stop_dist_ && sp[target_index] == 0){
    RCLCPP_ERROR(logger_,"obstacle too close, stop!");
    if(encounter_obst_moment_logged_ == false){
      encounter_obst_moment_logged_ = true;
      encounter_obst_moment_ = clock_->now().seconds();
    }else{
      double dt = clock_->now().seconds() - encounter_obst_moment_;
      RCLCPP_INFO(logger_,"obstacle dt: %lf",dt);
      if(dt>obstacle_timeout_){
        throw nav2_core::PlannerException("obstacle ahead, waited for too long. goal failed.");
      }
    }
  }else if(obstacle_distance_list[target_index]<=obst_slow_dist_ && obstacle_distance_list[target_index]>0 && sp[target_index] != 0){
    RCLCPP_WARN(logger_,"obstacle closing in %f, slowing down!",obstacle_distance_list[target_index]);
    encounter_obst_moment_logged_ = false;
  }else{
    encounter_obst_moment_logged_ = false;
  }
  // get curvature of tracking point
  double K = k_list[target_index];
  double kesi = atan2(vehicle_L_ * K, 1);   // reference steer angle

  // reference input
  U U_r;
  U_r.v = sp[target_index];   // apply reference speed
  U_r.kesi = kesi*(sp[target_index]>=0?1:-1);

  dt_ = rclcpp::Clock().now().seconds() - last_control_time;

  last_control_time = rclcpp::Clock().now().seconds();

  lqr_controller_->initial(vehicle_L_, dt_, robot_state_, Point, U_r, Q_, R_);

  // std::cout << "compute u" << std::endl;
  U control = lqr_controller_->cal_vel();//计算输入[v, kesi]
  // std::cout << "u completed" << std::endl;
  if(U_r.v==0)control.v = 0;

  // std::cout << "u kesi: " << U_r.kesi << std::endl;
  
  cmd_vel.twist.linear.x = control.v;
  cmd_vel.twist.angular.z = control.v*tan(control.kesi)/vehicle_L_;
  kesi_ = control.kesi;
  RCLCPP_INFO(logger_,"v:%.2f kesi:%.2f vx:%.2f vyaw:%.2f K:%.2f",control.v,kesi_,cmd_vel.twist.linear.x,cmd_vel.twist.angular.z,k_list[target_index]);


  if((size_t)current_tracking_path_segment_ == path_segment_.size()-1){
    double dist_to_goal = nav2_util::geometry_utils::euclidean_distance(global_pose,global_plan_.poses.back());
    // RCLCPP_INFO(logger_,"dist_to_goal: %f",dist_to_goal);
    if(dist_to_goal< min(pose_tolerance.position.x,pose_tolerance.position.y)/2){
      cmd_vel.twist.linear.x = 0;
      cmd_vel.twist.angular.z= 0;
      RCLCPP_INFO(logger_,"goal reached -- from controller plugin");
    }
  }
  
  // cmd_vel.twist.linear.x = 0;
  // cmd_vel.twist.angular.z= 0;

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