#ifndef LQR_CONTROLLER_HPP
#define LQR_CONTROLLER_HPP

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

  vector<double> get_speed_profile(vehicleState state,float fv_max,float bv_max,float v_min,float max_lateral_accel,vector<waypoint>& wp,vector<double>& curvature_list);

  /**
   * @brief nav2_core setPlan - Sets the global plan
   * @param path The global plan
   */
    void setPlan(const nav_msgs::msg::Path & path) override;

  nav_msgs::msg::Path grep_path_in_local_costmap(const geometry_msgs::msg::PoseStamped & robot_pose);
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

  vector<int> find_cusp(const nav_msgs::msg::Path & path);
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

  nav_msgs::msg::Path global_plan_;
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>> global_path_pub_;
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>> lqr_path_pub_;
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PoseStamped>>  target_pub_;
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PointStamped>> cusp_pub_;
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>> target_arc_pub_;
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