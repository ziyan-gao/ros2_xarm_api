#pragma once
#include <array>
#include <vector>
#include <QMap>
#include <rviz_common/panel.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <QElapsedTimer>
#include <QJsonObject>
class QCheckBox;
class QLabel;
class QPlainTextEdit;
class QPushButton;
class QSlider;
namespace safe_servo_rviz_panel
{
class TaskTestPanel : public rviz_common::Panel
{
  Q_OBJECT
public:
  explicit TaskTestPanel(QWidget * parent = nullptr);
  void onInitialize() override;
private:
  void refresh();
  void restartNode(size_t index);
  void request(size_t index);
  void setMode(size_t index, bool enabled);
  std::vector<QPushButton *> restart_buttons_;
  std::vector<QLabel *> restart_labels_;
  std::vector<rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr> restart_clients_;
  QJsonObject restart_status_;
  QMap<QString, QElapsedTimer> restart_received_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr restart_subscription_;
  QCheckBox * confirm_;
  std::array<QCheckBox *, 3> modes_;
  std::array<QPushButton *, 14> buttons_;
  std::array<QLabel *, 4> nodes_;
  QSlider * speed_;
  QLabel * reply_;
  QPlainTextEdit * details_;
  QElapsedTimer received_;
  QElapsedTimer pending_;
  QElapsedTimer reply_received_;
  QJsonObject status_;
  rclcpp::Node::SharedPtr node_;
  std::array<rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr, 14> clients_;
  std::array<rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr, 3> mode_clients_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr speed_pub_;
};
}
