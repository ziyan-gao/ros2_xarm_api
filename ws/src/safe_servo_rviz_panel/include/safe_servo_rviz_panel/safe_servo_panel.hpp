#pragma once

#include <array>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_srvs/srv/trigger.hpp>

class QLabel;
class QSlider;
class QDoubleSpinBox;
class QSpinBox;
class QPushButton;
class QTimer;
class QCheckBox;

namespace safe_servo_rviz_panel
{
class SafeServoPanel : public rviz_common::Panel
{
  Q_OBJECT
public:
  explicit SafeServoPanel(QWidget * parent = nullptr);
  void onInitialize() override;
private Q_SLOTS:
  void publishConfig();
  void publishMotionSpeed();
  void publishRandomLoadingConfig();
  void resetFault();
  void startPalletDetection();
  void applyPalletConfig();
  void useConfiguredPalletPose();
  void lockPallet();
  void clearPallet();
  void clearPlacedObstacles();
  void openGripper();
  void closeGripper();
  void setObservationPose();
  void planObservation();
  void planPrePlace();
  void executeMotionPlan();
  void cancelMotion();
  void resetMotionCoordinator();
  void startPickup();
  void abortPickup();
  void resetPickup();
  void startPlace();
  void abortPlace();
  void resetPlace();
  void startRandomLoading();
  void abortRandomLoading();
  void resetRandomLoading();
  void setContinuousRandomLoading(bool enabled);
private:
  void setGripper(bool close);
  QDoubleSpinBox * force_threshold_;
  QSlider * motion_speed_;
  QLabel * motion_speed_label_;
  QPushButton * reset_button_;
  QLabel * state_label_;
  std::array<QLabel *, 7> tcp_telemetry_;
  QSpinBox * pallet_marker_id_;
  QSpinBox * pallet_samples_;
  std::array<QDoubleSpinBox *, 7> pallet_config_;
  std::array<QDoubleSpinBox *, 3> pre_place_pose_;
  QCheckBox * rotate_item_90_;
  QCheckBox * keep_eef_perpendicular_;
  QCheckBox * add_placed_item_obstacle_;
  bool pallet_config_loaded_{false};
  QLabel * pallet_state_label_;
  QLabel * manual_operations_label_;
  QLabel * motion_state_label_;
  QLabel * pickup_state_label_;
  QLabel * place_state_label_;
  QLabel * random_loading_state_label_;
  QSlider * com_bound_ratio_;
  QLabel * com_bound_ratio_label_;
  QCheckBox * continuous_random_loading_;
  QCheckBox * random_add_placed_item_obstacle_;
  QLabel * placed_obstacles_label_;
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr config_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr motion_speed_config_pub_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr reset_client_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr servo_status_sub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pallet_config_pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr pallet_status_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr pallet_config_state_sub_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr pallet_start_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr pallet_lock_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr pallet_use_config_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr pallet_clear_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr clear_placed_obstacles_client_;
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr set_gripper_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr save_observation_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr plan_observation_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr plan_pre_place_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr execute_motion_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr cancel_motion_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr reset_motion_client_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr motion_status_sub_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr start_pickup_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr abort_pickup_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr reset_pickup_client_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr pickup_status_sub_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr start_place_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr abort_place_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr reset_place_client_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr place_status_sub_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr start_random_loading_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr abort_random_loading_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr reset_random_loading_client_;
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr continuous_random_loading_client_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr random_loading_config_pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr random_loading_status_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr planning_scene_status_sub_;
};
}
