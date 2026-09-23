#pragma once
#include <rviz_common/panel.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <QElapsedTimer>
#include <QImage>
#include <QJsonObject>

class QComboBox;
class QLabel;
class QPlainTextEdit;
class QPushButton;
class QResizeEvent;

namespace safe_servo_rviz_panel
{
class TopFaceDebugPanel : public rviz_common::Panel
{
  Q_OBJECT
public:
  explicit TopFaceDebugPanel(QWidget * parent = nullptr);
  void onInitialize() override;
protected:
  void resizeEvent(QResizeEvent * event) override;
private:
  void command(const QString & action);
  void status(const QJsonObject & object);
  void updateControls();
  void showImage();
  QComboBox * targets_;
  QLabel * preview_;
  QPlainTextEdit * details_;
  QPushButton * capture_;
  QPushButton * segment_;
  QPushButton * save_;
  QPushButton * clear_;
  QImage image_;
  QElapsedTimer received_;
  QElapsedTimer command_pending_;
  QJsonObject last_status_;
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr commands_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr status_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr preview_sub_;
};
}
