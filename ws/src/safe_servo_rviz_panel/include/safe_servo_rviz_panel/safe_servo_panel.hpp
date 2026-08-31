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
  void updateSpeedLabel(int value);
  void toggleExecution();
  void publishTarget();
  void resetFault();
  void updateTouchMode(bool checked);
  void startPalletDetection();
  void lockPallet();
  void clearPallet();
  void startItemDetection();
  void clearItemDetection();
private:
  QSlider * speed_;
  QLabel * speed_label_;
  std::array<QDoubleSpinBox *, 6> bounds_;
  std::array<QDoubleSpinBox *, 4> target_;
  std::array<double, 4> active_target_;
  QDoubleSpinBox * force_threshold_;
  QPushButton * execute_button_;
  QPushButton * reset_button_;
  QLabel * state_label_;
  QSpinBox * pallet_marker_id_;
  QSpinBox * pallet_samples_;
  std::array<QDoubleSpinBox *, 7> pallet_config_;
  QLabel * pallet_state_label_;
  QCheckBox * touch_mode_;
  QSpinBox * item_marker_id_;
  QSpinBox * item_samples_;
  std::array<QDoubleSpinBox *, 2> item_tolerances_;
  QLabel * item_state_label_;
  QTimer * command_timer_;
  bool executing_;
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr config_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr command_pub_;
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr enable_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr reset_client_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pallet_config_pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr pallet_status_sub_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr pallet_start_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr pallet_lock_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr pallet_clear_client_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr item_config_pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr item_status_sub_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr item_start_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr item_clear_client_;
};
}
