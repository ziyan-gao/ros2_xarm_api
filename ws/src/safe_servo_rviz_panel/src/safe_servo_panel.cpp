#include "safe_servo_rviz_panel/safe_servo_panel.hpp"

#include <QDoubleSpinBox>
#include <QCheckBox>
#include <QFormLayout>
#include <QLabel>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QPushButton>
#include <QScrollArea>
#include <QSlider>
#include <QSpinBox>
#include <QTimer>
#include <QVBoxLayout>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/display_context.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

namespace safe_servo_rviz_panel
{
SafeServoPanel::SafeServoPanel(QWidget * parent)
: rviz_common::Panel(parent), executing_(false)
{
  active_target_ = {270.0, 20.0, 70.0, 0.0};
  // Keep the dock's minimum size bounded. Without a scroll area the combined
  // servo and pallet controls can consume the whole RViz window, leaving Ogre
  // a zero-sized render surface and causing a GL vertex-buffer crash.
  auto * root_layout = new QVBoxLayout(this);
  root_layout->setContentsMargins(0, 0, 0, 0);
  auto * scroll = new QScrollArea(this);
  scroll->setWidgetResizable(true);
  scroll->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
  auto * content = new QWidget(scroll);
  auto * layout = new QVBoxLayout(content);
  scroll->setWidget(content);
  root_layout->addWidget(scroll);
  speed_label_ = new QLabel("Speed: 20%", this);
  speed_ = new QSlider(Qt::Horizontal, this);
  speed_->setRange(1, 100);
  speed_->setValue(20);
  layout->addWidget(speed_label_);
  layout->addWidget(speed_);
  connect(speed_, &QSlider::valueChanged, this, &SafeServoPanel::updateSpeedLabel);

  auto * form = new QFormLayout();
  const char * names[] = {"X min (mm)", "X max (mm)", "Y min (mm)",
    "Y max (mm)", "Z min (mm)", "Z max (mm)"};
  const double defaults[] = {160, 390, -360, 360, 50, 80};
  for (size_t i = 0; i < bounds_.size(); ++i) {
    bounds_[i] = new QDoubleSpinBox(this);
    bounds_[i]->setRange(-2000.0, 2000.0);
    bounds_[i]->setDecimals(1);
    bounds_[i]->setValue(defaults[i]);
    form->addRow(names[i], bounds_[i]);
  }
  layout->addLayout(form);

  auto * target_form = new QFormLayout();
  const char * target_names[] = {
    "Target X (mm)", "Target Y (mm)", "Target Z (mm)", "Target yaw (deg)"};
  const double target_defaults[] = {270, 20, 70, 0};
  for (size_t i = 0; i < target_.size(); ++i) {
    target_[i] = new QDoubleSpinBox(this);
    target_[i]->setRange(i == 3 ? -180.0 : -2000.0, i == 3 ? 180.0 : 2000.0);
    target_[i]->setDecimals(1);
    target_[i]->setValue(target_defaults[i]);
    target_form->addRow(target_names[i], target_[i]);
  }
  force_threshold_ = new QDoubleSpinBox(this);
  force_threshold_->setRange(0.1, 200.0);
  force_threshold_->setDecimals(1);
  force_threshold_->setSuffix(" N");
  force_threshold_->setValue(15.0);
  target_form->addRow("Force threshold", force_threshold_);
  touch_mode_ = new QCheckBox(
    "Touch-to-grasp mode (Z-only; ignore X/Y limits)", this);
  target_form->addRow(touch_mode_);
  layout->addLayout(target_form);
  connect(touch_mode_, &QCheckBox::toggled,
    this, &SafeServoPanel::updateTouchMode);

  auto * apply = new QPushButton("Apply safety configuration", this);
  layout->addWidget(apply);
  connect(apply, &QPushButton::clicked, this, &SafeServoPanel::publishConfig);
  execute_button_ = new QPushButton("Execute target", this);
  layout->addWidget(execute_button_);
  connect(execute_button_, &QPushButton::clicked, this, &SafeServoPanel::toggleExecution);
  command_timer_ = new QTimer(this);
  command_timer_->setInterval(50);
  connect(command_timer_, &QTimer::timeout, this, &SafeServoPanel::publishTarget);
  reset_button_ = new QPushButton("Reset fault", this);
  state_label_ = new QLabel("Controller: idle", this);
  layout->addWidget(reset_button_);
  layout->addWidget(state_label_);
  connect(reset_button_, &QPushButton::clicked, this, &SafeServoPanel::resetFault);

  auto * pallet_group = new QGroupBox("Pallet localization", this);
  auto * pallet_layout = new QVBoxLayout(pallet_group);
  auto * pallet_form = new QFormLayout();
  pallet_marker_id_ = new QSpinBox(this);
  pallet_marker_id_->setRange(0, 49);
  pallet_marker_id_->setValue(49);
  pallet_form->addRow("Pallet marker ID", pallet_marker_id_);
  pallet_samples_ = new QSpinBox(this);
  pallet_samples_->setRange(5, 200);
  pallet_samples_->setValue(30);
  pallet_form->addRow("Stable samples", pallet_samples_);
  const char * pallet_names[] = {
    "Position tolerance (mm)", "Angle tolerance (deg)",
    "Pallet +X length (mm)", "Pallet +Y length (mm)",
    "Marker roll offset (deg)", "Marker pitch offset (deg)",
    "Marker yaw offset (deg)"};
  const double pallet_defaults[] = {5.0, 2.0, 1200.0, 1000.0, 0.0, 0.0, 0.0};
  for (size_t i = 0; i < pallet_config_.size(); ++i) {
    pallet_config_[i] = new QDoubleSpinBox(this);
    pallet_config_[i]->setDecimals(1);
    pallet_config_[i]->setRange(i >= 4 ? -180.0 : 0.1,
      i >= 4 ? 180.0 : 3000.0);
    pallet_config_[i]->setValue(pallet_defaults[i]);
    pallet_form->addRow(pallet_names[i], pallet_config_[i]);
  }
  pallet_layout->addLayout(pallet_form);
  auto * pallet_buttons = new QHBoxLayout();
  auto * detect_pallet = new QPushButton("Detect pallet", this);
  auto * lock_pallet = new QPushButton("Accept && lock", this);
  auto * clear_pallet = new QPushButton("Clear", this);
  pallet_buttons->addWidget(detect_pallet);
  pallet_buttons->addWidget(lock_pallet);
  pallet_buttons->addWidget(clear_pallet);
  pallet_layout->addLayout(pallet_buttons);
  pallet_state_label_ = new QLabel("Pallet: UNLOCALIZED", this);
  pallet_state_label_->setWordWrap(true);
  pallet_layout->addWidget(pallet_state_label_);
  layout->addWidget(pallet_group);
  connect(detect_pallet, &QPushButton::clicked,
    this, &SafeServoPanel::startPalletDetection);
  connect(lock_pallet, &QPushButton::clicked,
    this, &SafeServoPanel::lockPallet);
  connect(clear_pallet, &QPushButton::clicked,
    this, &SafeServoPanel::clearPallet);

  auto * item_group = new QGroupBox("Incoming item detection", this);
  auto * item_layout = new QVBoxLayout(item_group);
  auto * item_form = new QFormLayout();
  item_marker_id_ = new QSpinBox(this);
  item_marker_id_->setRange(-1, 49);
  item_marker_id_->setSpecialValueText("Any recognized marker");
  item_marker_id_->setValue(-1);
  item_form->addRow("Expected item marker", item_marker_id_);
  item_samples_ = new QSpinBox(this);
  item_samples_->setRange(5, 200);
  item_samples_->setValue(30);
  item_form->addRow("Stable samples", item_samples_);
  const char * item_tolerance_names[] = {
    "Position tolerance (mm)", "Angle tolerance (deg)"};
  const double item_tolerance_defaults[] = {5.0, 2.0};
  for (size_t i = 0; i < item_tolerances_.size(); ++i) {
    item_tolerances_[i] = new QDoubleSpinBox(this);
    item_tolerances_[i]->setRange(0.1, i == 0 ? 100.0 : 45.0);
    item_tolerances_[i]->setDecimals(1);
    item_tolerances_[i]->setValue(item_tolerance_defaults[i]);
    item_form->addRow(item_tolerance_names[i], item_tolerances_[i]);
  }
  item_layout->addLayout(item_form);
  auto * item_buttons = new QHBoxLayout();
  auto * detect_item = new QPushButton("Detect incoming item", this);
  auto * clear_item = new QPushButton("Clear item", this);
  item_buttons->addWidget(detect_item);
  item_buttons->addWidget(clear_item);
  item_layout->addLayout(item_buttons);
  item_state_label_ = new QLabel("Item: IDLE", this);
  item_state_label_->setWordWrap(true);
  item_layout->addWidget(item_state_label_);
  layout->addWidget(item_group);
  connect(detect_item, &QPushButton::clicked,
    this, &SafeServoPanel::startItemDetection);
  connect(clear_item, &QPushButton::clicked,
    this, &SafeServoPanel::clearItemDetection);
  layout->addStretch();
}

void SafeServoPanel::onInitialize()
{
  auto abstraction = getDisplayContext()->getRosNodeAbstraction().lock();
  if (!abstraction) {
    state_label_->setText("RViz ROS node unavailable");
    return;
  }
  node_ = abstraction->get_raw_node();
  config_pub_ = node_->create_publisher<std_msgs::msg::Float64MultiArray>(
    "/safe_servo/config", 10);
  command_pub_ = node_->create_publisher<std_msgs::msg::Float64MultiArray>(
    "/servo_command", 10);
  enable_client_ = node_->create_client<std_srvs::srv::SetBool>("/safe_servo/enable");
  reset_client_ = node_->create_client<std_srvs::srv::Trigger>("/safe_servo/reset_fault");
  pallet_config_pub_ = node_->create_publisher<std_msgs::msg::Float64MultiArray>(
    "/pallet_localization/config", 10);
  pallet_start_client_ = node_->create_client<std_srvs::srv::Trigger>(
    "/pallet_localization/start");
  pallet_lock_client_ = node_->create_client<std_srvs::srv::Trigger>(
    "/pallet_localization/lock");
  pallet_clear_client_ = node_->create_client<std_srvs::srv::Trigger>(
    "/pallet_localization/clear");
  pallet_status_sub_ = node_->create_subscription<std_msgs::msg::String>(
    "/pallet_localization/status", 10,
    [this](const std_msgs::msg::String::SharedPtr msg) {
      pallet_state_label_->setText(
        QString("Pallet: %1").arg(QString::fromStdString(msg->data)));
    });
  item_config_pub_ = node_->create_publisher<std_msgs::msg::Float64MultiArray>(
    "/item_localization/config", 10);
  item_start_client_ = node_->create_client<std_srvs::srv::Trigger>(
    "/item_localization/start");
  item_clear_client_ = node_->create_client<std_srvs::srv::Trigger>(
    "/item_localization/clear");
  item_status_sub_ = node_->create_subscription<std_msgs::msg::String>(
    "/item_localization/status", 10,
    [this](const std_msgs::msg::String::SharedPtr msg) {
      item_state_label_->setText(
        QString("Item: %1").arg(QString::fromStdString(msg->data)));
    });
}

void SafeServoPanel::updateSpeedLabel(int value)
{
  speed_label_->setText(QString("Speed: %1%").arg(value));
}

void SafeServoPanel::publishConfig()
{
  std_msgs::msg::Float64MultiArray msg;
  msg.data.push_back(static_cast<double>(speed_->value()) / 100.0);
  for (auto * bound : bounds_) {
    msg.data.push_back(bound->value());
  }
  msg.data.push_back(force_threshold_->value());
  msg.data.push_back(touch_mode_->isChecked() ? 1.0 : 0.0);
  config_pub_->publish(msg);
}

void SafeServoPanel::updateTouchMode(bool checked)
{
  target_[0]->setEnabled(!checked);
  target_[1]->setEnabled(!checked);
  target_[3]->setEnabled(!checked);
  for (size_t i = 0; i < 4; ++i) {
    bounds_[i]->setEnabled(!checked);
  }
  execute_button_->setText(checked ? "Execute Z touch descent" : "Execute target");
  state_label_->setText(checked ?
    "Touch mode: X/Y/orientation will be locked at enable" :
    "Controller: idle");
}

void SafeServoPanel::toggleExecution()
{
  if (!executing_) {
    if (!enable_client_->service_is_ready()) {
      state_label_->setText("Controller service unavailable");
      return;
    }
    const char * axes[] = {"X", "Y", "Z"};
    for (size_t i = touch_mode_->isChecked() ? 2 : 0; i < 3; ++i) {
      const double value = target_[i]->value();
      const double lower = bounds_[2 * i]->value();
      const double upper = bounds_[2 * i + 1]->value();
      if (lower >= upper) {
        state_label_->setText(QString("Invalid %1 boundary").arg(axes[i]));
        return;
      }
      if (value < lower || value > upper) {
        state_label_->setText(
          QString("Target %1=%2 outside [%3, %4]; execution rejected")
          .arg(axes[i]).arg(value).arg(lower).arg(upper));
        return;
      }
    }
    publishConfig();
    for (size_t i = 0; i < target_.size(); ++i) {
      active_target_[i] = target_[i]->value();
      target_[i]->setEnabled(false);
    }
    // Config and enable are handled by the same executor. Defer enable so the
    // controller cannot validate its current pose against stale boundaries.
    QTimer::singleShot(150, this, [this]() {
      auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
      request->data = true;
      enable_client_->async_send_request(request, [this](
        rclcpp::Client<std_srvs::srv::SetBool>::SharedFuture future) {
        const auto response = future.get();
        state_label_->setText(QString::fromStdString(response->message));
        if (response->success) {
          executing_ = true;
          publishTarget();
          command_timer_->start();
        execute_button_->setText("Stop command");
        } else {
          for (auto * control : target_) {
            control->setEnabled(true);
          }
        }
      });
    });
  } else {
    executing_ = false;
    command_timer_->stop();
    execute_button_->setText(touch_mode_->isChecked() ?
      "Execute Z touch descent" : "Execute target");
    for (auto * control : target_) {
      control->setEnabled(true);
    }
    auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
    request->data = false;
    enable_client_->async_send_request(request, [this](
      rclcpp::Client<std_srvs::srv::SetBool>::SharedFuture future) {
      state_label_->setText(QString::fromStdString(future.get()->message));
    });
  }
}

void SafeServoPanel::publishTarget()
{
  std_msgs::msg::Float64MultiArray msg;
  msg.data = {
    active_target_[0], active_target_[1], active_target_[2],
    0.0, 0.0, active_target_[3]};
  command_pub_->publish(msg);
}

void SafeServoPanel::resetFault()
{
  if (!reset_client_->service_is_ready()) {
    state_label_->setText("Reset service unavailable");
    return;
  }
  executing_ = false;
  command_timer_->stop();
  execute_button_->setText(touch_mode_->isChecked() ?
    "Execute Z touch descent" : "Execute target");
  for (auto * control : target_) {
    control->setEnabled(true);
  }
  auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
  reset_client_->async_send_request(request, [this](
    rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
    state_label_->setText(QString::fromStdString(future.get()->message));
  });
}

void SafeServoPanel::startPalletDetection()
{
  if (!pallet_start_client_->service_is_ready()) {
    pallet_state_label_->setText("Pallet localization service unavailable");
    return;
  }
  std_msgs::msg::Float64MultiArray config;
  config.data = {
    static_cast<double>(pallet_marker_id_->value()),
    static_cast<double>(pallet_samples_->value()),
    pallet_config_[0]->value(), pallet_config_[1]->value(),
    pallet_config_[2]->value(), pallet_config_[3]->value(),
    pallet_config_[4]->value(), pallet_config_[5]->value(),
    pallet_config_[6]->value()};
  pallet_config_pub_->publish(config);
  // Configuration and service callbacks share the RViz executor; defer the
  // request very briefly so the configuration is applied first.
  QTimer::singleShot(100, this, [this]() {
    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    pallet_start_client_->async_send_request(request, [this](
      rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
      pallet_state_label_->setText(QString::fromStdString(future.get()->message));
    });
  });
}

void SafeServoPanel::lockPallet()
{
  if (!pallet_lock_client_->service_is_ready()) {
    pallet_state_label_->setText("Pallet localization service unavailable");
    return;
  }
  auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
  pallet_lock_client_->async_send_request(request, [this](
    rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
    pallet_state_label_->setText(QString::fromStdString(future.get()->message));
  });
}

void SafeServoPanel::clearPallet()
{
  if (!pallet_clear_client_->service_is_ready()) {
    pallet_state_label_->setText("Pallet localization service unavailable");
    return;
  }
  auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
  pallet_clear_client_->async_send_request(request, [this](
    rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
    pallet_state_label_->setText(QString::fromStdString(future.get()->message));
  });
}

void SafeServoPanel::startItemDetection()
{
  if (!item_start_client_->service_is_ready()) {
    item_state_label_->setText("Incoming-item localization service unavailable");
    return;
  }
  std_msgs::msg::Float64MultiArray config;
  config.data = {
    static_cast<double>(item_marker_id_->value()),
    static_cast<double>(item_samples_->value()),
    item_tolerances_[0]->value(), item_tolerances_[1]->value()};
  item_config_pub_->publish(config);
  QTimer::singleShot(100, this, [this]() {
    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    item_start_client_->async_send_request(request, [this](
      rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
      item_state_label_->setText(QString::fromStdString(future.get()->message));
    });
  });
}

void SafeServoPanel::clearItemDetection()
{
  if (!item_clear_client_->service_is_ready()) {
    item_state_label_->setText("Incoming-item localization service unavailable");
    return;
  }
  auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
  item_clear_client_->async_send_request(request, [this](
    rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
    item_state_label_->setText(QString::fromStdString(future.get()->message));
  });
}

}

PLUGINLIB_EXPORT_CLASS(safe_servo_rviz_panel::SafeServoPanel, rviz_common::Panel)
