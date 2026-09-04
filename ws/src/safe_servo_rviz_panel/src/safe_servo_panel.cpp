#include "safe_servo_rviz_panel/safe_servo_panel.hpp"

#include <QDoubleSpinBox>
#include <QCheckBox>
#include <QFormLayout>
#include <QLabel>
#include <QJsonDocument>
#include <QJsonObject>
#include <QGroupBox>
#include <QPushButton>
#include <QScrollArea>
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
: rviz_common::Panel(parent)
{
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
  auto * tcp_group = new QGroupBox("Current TCP and force", this);
  auto * tcp_layout = new QVBoxLayout(tcp_group);
  const char * tcp_names[] = {"X", "Y", "Z", "Yaw", "Fx", "Fy", "Fz"};
  for (size_t i = 0; i < tcp_telemetry_.size(); ++i) {
    tcp_telemetry_[i] = new QLabel(QString("%1: --").arg(tcp_names[i]), tcp_group);
    tcp_telemetry_[i]->setAlignment(Qt::AlignCenter);
    tcp_telemetry_[i]->setMinimumWidth(72);
    tcp_telemetry_[i]->setStyleSheet(
      "QLabel { border: 1px solid #64748b; border-radius: 4px; padding: 5px; "
      "background: #1e293b; color: #e2e8f0; font-weight: bold; }");
    tcp_layout->addWidget(tcp_telemetry_[i]);
  }
  layout->addWidget(tcp_group);

  auto * target_form = new QFormLayout();
  target_form->setRowWrapPolicy(QFormLayout::WrapAllRows);
  target_form->setFieldGrowthPolicy(QFormLayout::AllNonFixedFieldsGrow);
  force_threshold_ = new QDoubleSpinBox(this);
  force_threshold_->setRange(0.1, 200.0);
  force_threshold_->setDecimals(1);
  force_threshold_->setSuffix(" N");
  force_threshold_->setValue(15.0);
  target_form->addRow("Pickup descent force threshold", force_threshold_);
  layout->addLayout(target_form);
  connect(force_threshold_, qOverload<double>(&QDoubleSpinBox::valueChanged),
    this, [this](double) {publishConfig();});
  reset_button_ = new QPushButton("Reset fault", this);
  state_label_ = new QLabel("Controller: idle", this);
  layout->addWidget(reset_button_);
  layout->addWidget(state_label_);
  connect(reset_button_, &QPushButton::clicked, this, &SafeServoPanel::resetFault);

  auto * pallet_group = new QGroupBox("Pallet localization", this);
  auto * pallet_layout = new QVBoxLayout(pallet_group);
  auto * pallet_form = new QFormLayout();
  pallet_form->setRowWrapPolicy(QFormLayout::WrapAllRows);
  pallet_form->setFieldGrowthPolicy(QFormLayout::AllNonFixedFieldsGrow);
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
  auto * pallet_buttons = new QVBoxLayout();
  auto * detect_pallet = new QPushButton("Detect pallet", this);
  auto * lock_pallet = new QPushButton("Accept, save && lock", this);
  auto * apply_pallet = new QPushButton("Apply/save values", this);
  auto * use_configured_pallet = new QPushButton("Load saved pallet && lock", this);
  auto * clear_pallet = new QPushButton("Clear", this);
  pallet_buttons->addWidget(detect_pallet);
  pallet_buttons->addWidget(lock_pallet);
  pallet_buttons->addWidget(clear_pallet);
  pallet_layout->addLayout(pallet_buttons);
  pallet_layout->addWidget(apply_pallet);
  pallet_layout->addWidget(use_configured_pallet);
  pallet_state_label_ = new QLabel("Pallet: UNLOCALIZED", this);
  pallet_state_label_->setWordWrap(true);
  pallet_layout->addWidget(pallet_state_label_);
  layout->addWidget(pallet_group);
  connect(detect_pallet, &QPushButton::clicked,
    this, &SafeServoPanel::startPalletDetection);
  connect(lock_pallet, &QPushButton::clicked,
    this, &SafeServoPanel::lockPallet);
  connect(apply_pallet, &QPushButton::clicked,
    this, &SafeServoPanel::applyPalletConfig);
  connect(use_configured_pallet, &QPushButton::clicked,
    this, &SafeServoPanel::useConfiguredPalletPose);
  connect(clear_pallet, &QPushButton::clicked,
    this, &SafeServoPanel::clearPallet);

  auto * item_group = new QGroupBox("Incoming item detection", this);
  auto * item_layout = new QVBoxLayout(item_group);
  auto * item_form = new QFormLayout();
  item_form->setRowWrapPolicy(QFormLayout::WrapAllRows);
  item_form->setFieldGrowthPolicy(QFormLayout::AllNonFixedFieldsGrow);
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
  auto * item_buttons = new QVBoxLayout();
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

  auto * waypoint_group = new QGroupBox("Taught motion waypoints", this);
  auto * waypoint_layout = new QVBoxLayout(waypoint_group);
  auto * waypoint_buttons = new QVBoxLayout();
  auto * save_observation = new QPushButton("Save observation", this);
  auto * save_intermediate = new QPushButton("Save intermediate", this);
  waypoint_buttons->addWidget(save_observation);
  waypoint_buttons->addWidget(save_intermediate);
  waypoint_layout->addLayout(waypoint_buttons);
  auto * reload_waypoints = new QPushButton("Reload waypoint file", this);
  waypoint_layout->addWidget(reload_waypoints);
  waypoint_state_label_ = new QLabel(
    "Waypoints: waiting for storage node", this);
  waypoint_state_label_->setWordWrap(true);
  waypoint_layout->addWidget(waypoint_state_label_);
  layout->addWidget(waypoint_group);
  connect(save_observation, &QPushButton::clicked,
    this, &SafeServoPanel::saveObservationWaypoint);
  connect(save_intermediate, &QPushButton::clicked,
    this, &SafeServoPanel::saveIntermediateWaypoint);
  connect(reload_waypoints, &QPushButton::clicked,
    this, &SafeServoPanel::reloadWaypoints);

  auto * motion_group = new QGroupBox("MoveIt waypoint motion", this);
  auto * motion_layout = new QVBoxLayout(motion_group);
  auto * plan_buttons = new QVBoxLayout();
  auto * plan_observation = new QPushButton("Plan observation", this);
  auto * plan_intermediate = new QPushButton("Plan intermediate", this);
  plan_buttons->addWidget(plan_observation);
  plan_buttons->addWidget(plan_intermediate);
  motion_layout->addLayout(plan_buttons);
  auto * motion_buttons = new QVBoxLayout();
  auto * execute_motion = new QPushButton("Execute latest plan", this);
  auto * cancel_motion = new QPushButton("Cancel motion", this);
  motion_buttons->addWidget(execute_motion);
  motion_buttons->addWidget(cancel_motion);
  motion_layout->addLayout(motion_buttons);
  auto * reset_motion = new QPushButton("Reset motion coordinator", this);
  motion_layout->addWidget(reset_motion);
  motion_state_label_ = new QLabel("Motion coordinator: unavailable", this);
  motion_state_label_->setWordWrap(true);
  motion_layout->addWidget(motion_state_label_);
  layout->addWidget(motion_group);
  connect(plan_observation, &QPushButton::clicked,
    this, &SafeServoPanel::planObservation);
  connect(plan_intermediate, &QPushButton::clicked,
    this, &SafeServoPanel::planIntermediate);
  connect(execute_motion, &QPushButton::clicked,
    this, &SafeServoPanel::executeMotionPlan);
  connect(cancel_motion, &QPushButton::clicked,
    this, &SafeServoPanel::cancelMotion);
  connect(reset_motion, &QPushButton::clicked,
    this, &SafeServoPanel::resetMotionCoordinator);

  auto * pickup_group = new QGroupBox("PickAndPlace cycle", this);
  auto * pickup_layout = new QVBoxLayout(pickup_group);
  auto * pickup_buttons = new QVBoxLayout();
  auto * start_pickup = new QPushButton("Start PickAndPlace", this);
  auto * abort_pickup = new QPushButton("Abort", this);
  auto * reset_pickup = new QPushButton("Reset", this);
  pickup_buttons->addWidget(start_pickup);
  pickup_buttons->addWidget(abort_pickup);
  pickup_buttons->addWidget(reset_pickup);
  pickup_layout->addLayout(pickup_buttons);
  pickup_state_label_ = new QLabel("PickAndPlace pipeline: unavailable", this);
  pickup_state_label_->setWordWrap(true);
  pickup_layout->addWidget(pickup_state_label_);
  layout->addWidget(pickup_group);
  connect(start_pickup, &QPushButton::clicked,
    this, &SafeServoPanel::startPickup);
  connect(abort_pickup, &QPushButton::clicked,
    this, &SafeServoPanel::abortPickup);
  connect(reset_pickup, &QPushButton::clicked,
    this, &SafeServoPanel::resetPickup);

  auto * place_group = new QGroupBox("Placement target", this);
  auto * place_layout = new QVBoxLayout(place_group);
  auto * pre_place_form = new QFormLayout();
  pre_place_form->setRowWrapPolicy(QFormLayout::WrapAllRows);
  pre_place_form->setFieldGrowthPolicy(QFormLayout::AllNonFixedFieldsGrow);
  const char * pre_place_names[] = {
    "Final object corner X (mm)", "Final object corner Y (mm)",
    "Final object corner Z (mm)"};
  const double pre_place_defaults[] = {600.0, 500.0, 200.0};
  for (size_t i = 0; i < pre_place_pose_.size(); ++i) {
    pre_place_pose_[i] = new QDoubleSpinBox(this);
    pre_place_pose_[i]->setDecimals(1);
    pre_place_pose_[i]->setRange(-3000.0, 3000.0);
    pre_place_pose_[i]->setValue(pre_place_defaults[i]);
    pre_place_form->addRow(pre_place_names[i], pre_place_pose_[i]);
  }
  rotate_item_90_ = new QCheckBox("Rotate item 90 deg about pallet Z", this);
  pre_place_form->addRow(rotate_item_90_);
  keep_tcp_roll_pitch_ = new QCheckBox("Keep current TCP roll/pitch", this);
  keep_tcp_roll_pitch_->setChecked(false);
  keep_tcp_roll_pitch_->setEnabled(false);
  keep_tcp_roll_pitch_->setToolTip(
    "Object-corner placement derives TCP orientation from the captured grasp");
  pre_place_form->addRow(keep_tcp_roll_pitch_);
  add_placed_item_obstacle_ =
    new QCheckBox("Add placed item as MoveIt obstacle", this);
  add_placed_item_obstacle_->setChecked(true);
  pre_place_form->addRow(add_placed_item_obstacle_);
  place_layout->addLayout(pre_place_form);
  auto * save_place_config = new QPushButton("Apply/save place target", this);
  auto * place_config_buttons = new QVBoxLayout();
  place_config_buttons->addWidget(save_place_config);
  place_layout->addLayout(place_config_buttons);
  place_state_label_ = new QLabel("Place pipeline: unavailable", this);
  place_state_label_->setWordWrap(true);
  place_layout->addWidget(place_state_label_);
  layout->addWidget(place_group);
  connect(save_place_config, &QPushButton::clicked,
    this, &SafeServoPanel::applyPalletConfig);
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
  reset_client_ = node_->create_client<std_srvs::srv::Trigger>("/safe_servo/reset_fault");
  servo_status_sub_ = node_->create_subscription<std_msgs::msg::String>(
    "/safe_servo/status", 10,
    [this](const std_msgs::msg::String::SharedPtr msg) {
      const auto json = QJsonDocument::fromJson(
        QByteArray::fromStdString(msg->data)).object();
      const char * keys[] = {
        "tcp_x_m", "tcp_y_m", "tcp_z_m", "tcp_yaw_deg",
        "force_x_n", "force_y_n", "force_z_n"};
      const char * names[] = {"X", "Y", "Z", "Yaw", "Fx", "Fy", "Fz"};
      const char * units[] = {" mm", " mm", " mm", " deg", " N", " N", " N"};
      for (size_t i = 0; i < tcp_telemetry_.size(); ++i) {
        const auto value = json.value(keys[i]);
        const QString text = value.isDouble()
          ? QString::number(value.toDouble() * (i < 3 ? 1000.0 : 1.0), 'f', 1) + units[i]
          : QString("--");
        tcp_telemetry_[i]->setText(QString("%1: %2").arg(names[i], text));
      }
    });
  pallet_config_pub_ = node_->create_publisher<std_msgs::msg::Float64MultiArray>(
    "/pallet_localization/config", 10);
  pallet_start_client_ = node_->create_client<std_srvs::srv::Trigger>(
    "/pallet_localization/start");
  pallet_lock_client_ = node_->create_client<std_srvs::srv::Trigger>(
    "/pallet_localization/lock");
  pallet_use_config_client_ = node_->create_client<std_srvs::srv::Trigger>(
    "/pallet_localization/use_configured_pose");
  pallet_clear_client_ = node_->create_client<std_srvs::srv::Trigger>(
    "/pallet_localization/clear");
  pallet_status_sub_ = node_->create_subscription<std_msgs::msg::String>(
    "/pallet_localization/status", 10,
    [this](const std_msgs::msg::String::SharedPtr msg) {
      pallet_state_label_->setText(
        QString("Pallet: %1").arg(QString::fromStdString(msg->data)));
    });
  pallet_config_state_sub_ =
    node_->create_subscription<std_msgs::msg::Float64MultiArray>(
    "/pallet_localization/config_state", 10,
    [this](const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
      if (msg->data.size() < 13 || pallet_config_loaded_) {return;}
      pallet_config_loaded_ = true;
      pallet_marker_id_->setValue(static_cast<int>(msg->data[0]));
      pallet_samples_->setValue(static_cast<int>(msg->data[1]));
      for (size_t i = 0; i < pallet_config_.size(); ++i) {
        pallet_config_[i]->setValue(msg->data[i + 2]);
      }
      for (size_t i = 0; i < pre_place_pose_.size(); ++i) {
        pre_place_pose_[i]->setValue(msg->data[i + 9]);
      }
      rotate_item_90_->setChecked(msg->data[12] > 0.5);
      if (msg->data.size() >= 14) {
        keep_tcp_roll_pitch_->setChecked(msg->data[13] > 0.5);
      }
      if (msg->data.size() >= 15) {
        add_placed_item_obstacle_->setChecked(msg->data[14] > 0.5);
      }
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
  save_observation_client_ = node_->create_client<std_srvs::srv::Trigger>(
    "/taught_waypoints/save_observation");
  save_intermediate_client_ = node_->create_client<std_srvs::srv::Trigger>(
    "/taught_waypoints/save_intermediate");
  reload_waypoints_client_ = node_->create_client<std_srvs::srv::Trigger>(
    "/taught_waypoints/reload");
  waypoint_status_sub_ = node_->create_subscription<std_msgs::msg::String>(
    "/taught_waypoints/status", 10,
    [this](const std_msgs::msg::String::SharedPtr msg) {
      waypoint_state_label_->setText(QString::fromStdString(msg->data));
    });
  plan_observation_client_ = node_->create_client<std_srvs::srv::Trigger>(
    "/motion_coordinator/plan_observation");
  plan_intermediate_client_ = node_->create_client<std_srvs::srv::Trigger>(
    "/motion_coordinator/plan_intermediate");
  plan_pre_place_client_ = node_->create_client<std_srvs::srv::Trigger>(
    "/motion_coordinator/plan_pre_place");
  execute_motion_client_ = node_->create_client<std_srvs::srv::Trigger>(
    "/motion_coordinator/execute");
  cancel_motion_client_ = node_->create_client<std_srvs::srv::Trigger>(
    "/motion_coordinator/cancel");
  reset_motion_client_ = node_->create_client<std_srvs::srv::Trigger>(
    "/motion_coordinator/reset");
  motion_status_sub_ = node_->create_subscription<std_msgs::msg::String>(
    "/motion_coordinator/status", 10,
    [this](const std_msgs::msg::String::SharedPtr msg) {
      motion_state_label_->setText(QString::fromStdString(msg->data));
    });
  start_pickup_client_ = node_->create_client<std_srvs::srv::Trigger>(
    "/pick_place_pipeline/start");
  abort_pickup_client_ = node_->create_client<std_srvs::srv::Trigger>(
    "/pick_place_pipeline/abort");
  reset_pickup_client_ = node_->create_client<std_srvs::srv::Trigger>(
    "/pick_place_pipeline/reset");
  pickup_status_sub_ = node_->create_subscription<std_msgs::msg::String>(
    "/pick_place_pipeline/status", 10,
    [this](const std_msgs::msg::String::SharedPtr msg) {
      pickup_state_label_->setText(QString::fromStdString(msg->data));
    });
  start_place_client_ = node_->create_client<std_srvs::srv::Trigger>(
    "/place_pipeline/start");
  abort_place_client_ = node_->create_client<std_srvs::srv::Trigger>(
    "/place_pipeline/abort");
  reset_place_client_ = node_->create_client<std_srvs::srv::Trigger>(
    "/place_pipeline/reset");
  place_status_sub_ = node_->create_subscription<std_msgs::msg::String>(
    "/place_pipeline/status", 10,
    [this](const std_msgs::msg::String::SharedPtr msg) {
      place_state_label_->setText(QString::fromStdString(msg->data));
    });
}

void SafeServoPanel::publishConfig()
{
  std_msgs::msg::Float64MultiArray msg;
  msg.data.push_back(force_threshold_->value());
  config_pub_->publish(msg);
}

void SafeServoPanel::resetFault()
{
  if (!reset_client_->service_is_ready()) {
    state_label_->setText("Reset service unavailable");
    return;
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
  applyPalletConfig();
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

void SafeServoPanel::applyPalletConfig()
{
  std_msgs::msg::Float64MultiArray config;
  config.data = {
    static_cast<double>(pallet_marker_id_->value()),
    static_cast<double>(pallet_samples_->value()),
    pallet_config_[0]->value(), pallet_config_[1]->value(),
    pallet_config_[2]->value(), pallet_config_[3]->value(),
    pallet_config_[4]->value(), pallet_config_[5]->value(),
    pallet_config_[6]->value(),
    pre_place_pose_[0]->value(), pre_place_pose_[1]->value(),
    pre_place_pose_[2]->value(), rotate_item_90_->isChecked() ? 1.0 : 0.0,
    keep_tcp_roll_pitch_->isChecked() ? 1.0 : 0.0,
    add_placed_item_obstacle_->isChecked() ? 1.0 : 0.0};
  pallet_config_pub_->publish(config);
  pallet_state_label_->setText("Pallet: saving configured values");
}

void SafeServoPanel::useConfiguredPalletPose()
{
  if (!pallet_use_config_client_->service_is_ready()) {
    pallet_state_label_->setText("Pallet configured-pose service unavailable");
    return;
  }
  auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
  pallet_use_config_client_->async_send_request(request, [this](
    rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
    pallet_state_label_->setText(QString::fromStdString(future.get()->message));
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

void SafeServoPanel::saveObservationWaypoint()
{
  if (!save_observation_client_->service_is_ready()) {
    waypoint_state_label_->setText("Waypoint storage service unavailable");
    return;
  }
  auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
  save_observation_client_->async_send_request(request, [this](
    rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
    waypoint_state_label_->setText(QString::fromStdString(future.get()->message));
  });
}

void SafeServoPanel::saveIntermediateWaypoint()
{
  if (!save_intermediate_client_->service_is_ready()) {
    waypoint_state_label_->setText("Waypoint storage service unavailable");
    return;
  }
  auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
  save_intermediate_client_->async_send_request(request, [this](
    rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
    waypoint_state_label_->setText(QString::fromStdString(future.get()->message));
  });
}

void SafeServoPanel::reloadWaypoints()
{
  if (!reload_waypoints_client_->service_is_ready()) {
    waypoint_state_label_->setText("Waypoint storage service unavailable");
    return;
  }
  auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
  reload_waypoints_client_->async_send_request(request, [this](
    rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
    waypoint_state_label_->setText(QString::fromStdString(future.get()->message));
  });
}

void SafeServoPanel::planObservation()
{
  if (!plan_observation_client_->service_is_ready()) {
    motion_state_label_->setText("Motion coordinator unavailable");
    return;
  }
  auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
  plan_observation_client_->async_send_request(request, [this](
    rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
    motion_state_label_->setText(QString::fromStdString(future.get()->message));
  });
}

void SafeServoPanel::planIntermediate()
{
  if (!plan_intermediate_client_->service_is_ready()) {
    motion_state_label_->setText("Motion coordinator unavailable");
    return;
  }
  auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
  plan_intermediate_client_->async_send_request(request, [this](
    rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
    motion_state_label_->setText(QString::fromStdString(future.get()->message));
  });
}

void SafeServoPanel::planPrePlace()
{
  if (!plan_pre_place_client_->service_is_ready()) {
    motion_state_label_->setText("Motion coordinator unavailable");
    return;
  }
  applyPalletConfig();
  QTimer::singleShot(100, this, [this]() {
    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    plan_pre_place_client_->async_send_request(request, [this](
      rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
      motion_state_label_->setText(QString::fromStdString(future.get()->message));
    });
  });
}

void SafeServoPanel::executeMotionPlan()
{
  if (!execute_motion_client_->service_is_ready()) {
    motion_state_label_->setText("Motion coordinator unavailable");
    return;
  }
  auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
  execute_motion_client_->async_send_request(request, [this](
    rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
    motion_state_label_->setText(QString::fromStdString(future.get()->message));
  });
}

void SafeServoPanel::cancelMotion()
{
  if (!cancel_motion_client_->service_is_ready()) {
    motion_state_label_->setText("Motion coordinator unavailable");
    return;
  }
  auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
  cancel_motion_client_->async_send_request(request, [this](
    rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
    motion_state_label_->setText(QString::fromStdString(future.get()->message));
  });
}

void SafeServoPanel::resetMotionCoordinator()
{
  if (!reset_motion_client_->service_is_ready()) {
    motion_state_label_->setText("Motion coordinator unavailable");
    return;
  }
  auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
  reset_motion_client_->async_send_request(request, [this](
    rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
    motion_state_label_->setText(QString::fromStdString(future.get()->message));
  });
}

void SafeServoPanel::startPickup()
{
  if (!start_pickup_client_->service_is_ready()) {
    pickup_state_label_->setText("Pickup pipeline unavailable");
    return;
  }
  publishConfig();
  applyPalletConfig();
  QTimer::singleShot(100, this, [this]() {
    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    start_pickup_client_->async_send_request(request, [this](
      rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
      pickup_state_label_->setText(QString::fromStdString(future.get()->message));
    });
  });
}

void SafeServoPanel::abortPickup()
{
  if (!abort_pickup_client_->service_is_ready()) {
    pickup_state_label_->setText("Pickup pipeline unavailable");
    return;
  }
  auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
  abort_pickup_client_->async_send_request(request, [this](
    rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
    pickup_state_label_->setText(QString::fromStdString(future.get()->message));
  });
}

void SafeServoPanel::resetPickup()
{
  if (!reset_pickup_client_->service_is_ready()) {
    pickup_state_label_->setText("Pickup pipeline unavailable");
    return;
  }
  auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
  reset_pickup_client_->async_send_request(request, [this](
    rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
    pickup_state_label_->setText(QString::fromStdString(future.get()->message));
  });
}

void SafeServoPanel::startPlace()
{
  if (!start_place_client_->service_is_ready()) {
    place_state_label_->setText("Place pipeline unavailable");
    return;
  }
  publishConfig();
  applyPalletConfig();
  QTimer::singleShot(100, this, [this]() {
    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    start_place_client_->async_send_request(request, [this](
      rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
      place_state_label_->setText(QString::fromStdString(future.get()->message));
    });
  });
}

void SafeServoPanel::abortPlace()
{
  if (!abort_place_client_->service_is_ready()) {
    place_state_label_->setText("Place pipeline unavailable");
    return;
  }
  auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
  abort_place_client_->async_send_request(request, [this](
    rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
    place_state_label_->setText(QString::fromStdString(future.get()->message));
  });
}

void SafeServoPanel::resetPlace()
{
  if (!reset_place_client_->service_is_ready()) {
    place_state_label_->setText("Place pipeline unavailable");
    return;
  }
  auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
  reset_place_client_->async_send_request(request, [this](
    rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
    place_state_label_->setText(QString::fromStdString(future.get()->message));
  });
}

}

PLUGINLIB_EXPORT_CLASS(safe_servo_rviz_panel::SafeServoPanel, rviz_common::Panel)
