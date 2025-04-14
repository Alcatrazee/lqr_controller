#ifndef LQR_CONTROLLER_HPP
#define LQR_CONTROLLER_HPP

#include <geometry_msgs/msg/detail/twist__struct.hpp>
#include <std_msgs/msg/detail/float32_multi_array__struct.hpp>
#include <std_msgs/msg/detail/u_int64_multi_array__struct.hpp>
#include <string>
#include <vector>
#include <memory>
#include <algorithm>
#include <mutex>
// #include "type_define.hpp"
#include "problem.hpp"
#include "nav2_costmap_2d/footprint_collision_checker.hpp"
#include "nav2_core/controller.hpp"
#include "rclcpp/rclcpp.hpp"
#include "pluginlib/class_loader.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "nav2_util/odometry_utils.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include <nav_msgs/msg/path.hpp>
#include <lqr_controller/LQR.hpp>
#include <vector>
#include <geometry_msgs/msg/polygon_stamped.hpp>
#include <std_msgs/msg/u_int64_multi_array.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>

namespace lqr_controller
{
/**
 * @class lqr_controller::LqrController
 * @brief lqr controller plugin
 */
using MatrixXd = Eigen::MatrixXd;
typedef std::vector<VectorXd, Eigen::aligned_allocator<VectorXd>> VecOfVectorXd;
typedef std::vector<MatrixXd, Eigen::aligned_allocator<MatrixXd>> VecOfMatrixXd;
class LqrController : public nav2_core::Controller
{
public:
  /**
   * @brief Constructor for lqr controller 
   */
  LqrController() = default;

  // LqrController(MatrixXd& A, MatrixXd& B, MatrixXd& Q, MatrixXd& R, MatrixXd& QT, MatrixXd& H, std::vector<VectorXd>& r, int T) : A_(A), B_(B), Q_(Q), R_(R), QT_(QT), H_(H), r_(r), T_(T) {
  //   init();
  // }

  /**
   * @brief Destrructor for lqr controller 
   */
  ~LqrController() override = default;

  /**
   * @brief Configure controller state machine
   * @param parent WeakPtr to node
   * @param name Name of plugin
   * @param tf TF buffer
   * @param costmap_ros Costmap2DROS object of environment
   */
  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;
  /**
   * @brief Cleanup controller state machine
   */
  void cleanup() override;

  /**
   * @brief Activate controller state machine
   */
  void activate() override;

  /**
   * @brief Deactivate controller state machine
   */
  void deactivate() override;

  /**
   * @brief Compute the best command given the current pose and velocity, with possible debug information
   *
   * Same as above computeVelocityCommands, but with debug results.
   * If the results using MatrixXd = Eigen::MatrixXd;
typedef std::vector<VectorXd, Eigen::aligned_allocator<VectorXd>> VecOfVectorXd;
typedef std::vector<MatrixXd, Eigen::aligned_allocator<MatrixXd>> VecOfMatrixXd;pointer is not null, additional information about the twists
   * evaluated will be in results after the call.
   *
   * @param pose      Current robot pose
   * @param velocity  Current robot velocity
   * @param goal_checker   Ptr to the goal checker for this task in case useful in computing commands
   * @return          Best command
   */
  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & /*speed */,
    nav2_core::GoalChecker * /*goal_checker*/) override;

  vector<double> get_speed_profile(vehicleState state,
    float fv_max,
    float bv_max,
    float v_min,
    float max_lateral_accel,
    vector<waypoint>& wp,
    vector<vector<double>>& curvature_list,
    vector<double>& distance_to_obst,
    int path_offset);
  
  vector<double> get_path_obst_distance(const nav_msgs::msg::Path &path,const geometry_msgs::msg::PoseStamped &robot_pose);
  double get_yaw(const geometry_msgs::msg::PoseStamped & pose);
  geometry_msgs::msg::PoseStamped interpolate_pose(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::PoseStamped & a,
    const geometry_msgs::msg::PoseStamped & b);

  /**
   * @brief nav2_core setPlan - Sets the global plan
   * @param path The global plan
   */
    void setPlan(const nav_msgs::msg::Path & path) override;

  nav_msgs::msg::Path grep_path_in_local_costmap(const geometry_msgs::msg::PoseStamped & robot_pose,int &path_offset);
  void remove_duplicated_points(vector<waypoint>& points);
  int Find_target_index(const geometry_msgs::msg::PoseStamped & state, nav_msgs::msg::Path &local_path);
  /**
   * @brief Cost at a point
   * @param x Pose of pose x
   * @param y Pose of pose y
   * @return Cost of pose in costmap
   */
  double costAtPose(const double & x, const double & y);

  /**
   * @brief Limits the maximum linear speed of the robot.
   * @param speed_limit expressed in absolute value (in m/s)
   * or in percentage from maximum robot speed.
   * @param percentage Setting speed limit in percentage if true
   * or in absolute values in false case.
   */
  void setSpeedLimit(const double & speed_limit, const bool & percentage) override;
  void removeDuplicatedPathPoint(nav_msgs::msg::Path & path);
  /**
   * @brief Callback executed when a parameter change is detected
   * @param event ParameterEvent message
   */
  rcl_interfaces::msg::SetParametersResult
  dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters);
  bool transformPose(
    const std::string frame,
    const geometry_msgs::msg::PoseStamped & in_pose,
    geometry_msgs::msg::PoseStamped & out_pose) const;
  void optimizeSpeedsDP(vector<double>& speeds, const vector<double>& distances, 
    double acc_max, double deacc_max, const unordered_set<int>& fixed_indices);
    int findMinAbsIndex(const std::vector<double>& max_v_curvature_list, const std::vector<int>& curve_index);
    vector<vector<int>> find_curve(vector<vector<double>>&K_list,double bound);
  vector<int> find_cusp(const nav_msgs::msg::Path & path);
  vector<vector<double>> get_kappa_d_kappa(vector<waypoint>& wp);

  void publish_collision_polygon(const geometry_msgs::msg::PoseStamped& pose);
  void forwardOptimize(vector<double>& speeds, const vector<double>& distances, 
    double deacc_max, int start_index);
  void backwardOptimize(vector<double>& speeds, const vector<double>& distances, 
    double acc_max, int start_index);
  void optimizeCurveSpeeds(vector<double>& speeds, const vector<double>& distances, 
    double acc_max, double deacc_max, 
    const vector<vector<int>>& curve_indices);

  void optimizeCurveInternalSpeeds(vector<double>& speeds, const vector<double>& distances, 
    double acc_max, double deacc_max, 
    const vector<vector<int>>& curve_indices, 
    const vector<int>& fixed_points);
  
  void internalBackwardOptimize(vector<double>& speeds, const vector<double>& distances, 
    double acc_max, int start_index, int end_index);
  
  void internalForwardOptimize(vector<double>& speeds, const vector<double>& distances, 
    double deacc_max, int start_index, int end_index);
  

    double lerp(double a, double b, double t);
    double angle_lerp(double a, double b, double t);
    double get_yaw_from_quaternion(const geometry_msgs::msg::Quaternion& quat) ;
    std::vector<geometry_msgs::msg::PoseStamped> resample_path(const std::vector<geometry_msgs::msg::PoseStamped>& poses, size_t num_samples);
    /**
   * @brief crop path within costmap
   * @param 
   */

  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::string plugin_name_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  nav2_costmap_2d::Costmap2D * costmap_;
  rclcpp::Logger logger_ {rclcpp::get_logger("LqrController")};
  rclcpp::Clock::SharedPtr clock_;

  tf2::Duration transform_tolerance_;
  int current_tracking_path_segment_ = 0;
  vector<int> cusp_index_;
  vector<nav_msgs::msg::Path> path_segment_;
  double encounter_obst_moment_; // moment of stopping due to obstacle
  bool encounter_obst_moment_logged_; // whether encounter obstacle moment is logged
  double obstacle_timeout_;
  double max_fvx_,max_bvx_,max_wz_;
  double max_lin_acc_,max_lateral_accel_,max_w_acc_,min_lin_deacc_;
  double dead_band_speed_;
  double approach_velocity_scaling_dist_,approach_v_gain_;
  bool use_obstacle_stopping_,use_output_filter_;
  double obst_speed_control_k_,obst_speed_control_b_;
  double obst_stop_dist_,obst_slow_dist_;
  double vehicle_L_;
  double inversion_xy_tolerance_;
  double Q_[5];
  double R_[2];
  int robot_search_pose_dist_;
  double max_steer_rate_;
  geometry_msgs::msg::Twist last_cmd_vel_;


  nav_msgs::msg::Path global_plan_;
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>> global_path_pub_;
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>> lqr_path_pub_;
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PoseStamped>>  target_pub_;
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PointStamped>> cusp_pub_;
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>> target_arc_pub_;
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PolygonStamped>> collision_polygon_pub_;
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::UInt64MultiArray>> error_code_pub_;
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float32MultiArray>> debug_pub_;
  std::unique_ptr<nav2_costmap_2d::FootprintCollisionChecker<nav2_costmap_2d::Costmap2D *>>
  collision_checker_;

  // Dynamic parameters handler
  std::mutex mutex_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr dyn_params_handler_;

  std::shared_ptr<Problem> problem_;
  std::shared_ptr<LQR> lqr_controller_;

  double L_;
  double dt_;
  vehicleState robot_state_;
  double kesi_;
};
}  // namespace lqr_controller

#endif  