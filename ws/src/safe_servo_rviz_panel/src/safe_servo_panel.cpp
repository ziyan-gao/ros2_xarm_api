#include "safe_servo_rviz_panel/safe_servo_panel.hpp"

#include <QDoubleSpinBox>
#include <QCheckBox>
#include <QFormLayout>
#include <QLabel>
#include <QPainter>
#include <QJsonDocument>
#include <QJsonObject>
#include <QGroupBox>
#include <QMessageBox>
#include <QPushButton>
#include <QScrollArea>
#include <QSignalBlocker>
#include <QSlider>
#include <QSpinBox>
#include <QTimer>
#include <QVBoxLayout>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/display_context.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

namespace
{
class VisibleTextButton : public QPushButton
{
public:
  explicit VisibleTextButton(const QString & caption, QWidget * parent = nullptr)
  : QPushButton(parent), caption_(caption)
  {
    setAccessibleName(caption_);
  }

protected:
  void paintEvent(QPaintEvent * event) override
  {
    QPushButton::paintEvent(event);
    QPainter painter(this);
    painter.setPen(isEnabled() ? QColor(20, 20, 20) : QColor(110, 110, 110));
    painter.setFont(font());
    painter.drawText(rect(), Qt::AlignCenter, caption_);
  }

private:
  QString caption_;
};
}

namespace safe_servo_rviz_panel
{
SafeServoPanel::SafeServoPanel(QWidget * parent)
: rviz_common::Panel(parent)
{
  // Keep the operational dock compact on wide/HiDPI displays. RViz restores
  // the main window at more than 3,000 px on this workstation, otherwise all
  // controls expand into unnecessarily wide rows.
  setMinimumWidth(300);
  setMaximumWidth(560);
  setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Expanding);
  // Keep the dock's minimum size bounded. Without a scroll area the combined
  // servo and pallet controls can consume the whole RViz window, leaving Ogre
  // a zero-sized render surface and causing a GL vertex-buffer crash.
  auto * root_layout = new QVBoxLayout(this);
  root_layout->setContentsMargins(0, 0, 0, 0);
  auto * scroll = new QScrollArea(this);
  scroll->setMinimumWidth(280);
  scroll->setMaximumWidth(550);
  scroll->setWidgetResizable(true);
  scroll->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
  auto * content = new QWidget(scroll);
  content->setMinimumWidth(270);
  content->setMaximumWidth(530);
  content->setStyleSheet(
    "QPushButton { color: rgb(20, 20, 20); "
    "background-color: rgb(245, 245, 245); "
    "border: 1px solid rgb(145, 145, 145); border-radius: 3px; "
    "padding: 6px 10px; min-height: 20px; }"
    "QPushButton:hover { background-color: rgb(225, 235, 245); }"
    "QPushButton:pressed { background-color: rgb(205, 220, 235); }"
    "QPushButton:disabled { color: rgb(110, 110, 110); "
    "background-color: rgb(225, 225, 225); }");
  auto * layout = new QVBoxLayout(content);
  scroll->setWidget(content);
  root_layout->addWidget(scroll);
  auto * tcp_group = new QGroupBox("Current TCP and force", this);
  auto * tcp_layout = new QVBoxLayout(tcp_group);
  const char * tcp_names[] = {"X", "Y", "Z", "Yaw", "Fx", "Fy", "Fz"};
  for (size_t i = 0; i < tcp_telemetry_.size(); ++i) {
    tcp_telemetry_[i] = new QLabel(
      QString::fromLatin1(tcp_names[i]) + QStringLiteral(": --"), tcp_group);
    tcp_telemetry_[i]->setAlignment(Qt::AlignLeft | Qt::AlignVCenter);
    tcp_telemetry_[i]->setMinimumWidth(72);
    tcp_telemetry_[i]->setMinimumHeight(24);
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
  motion_speed_ = new QSlider(Qt::Horizontal, this);
  motion_speed_->setRange(5, 100);
  motion_speed_->setSingleStep(5);
  motion_speed_->setPageStep(10);
  motion_speed_->setValue(30);
  motion_speed_->setToolTip(
    "Controls MoveIt and direct robot-service motion. Safe-servo speed is unchanged.");
  motion_speed_label_ = new QLabel("30% (30 mm/s service, 9% MoveIt)", this);
  auto * motion_speed_layout = new QVBoxLayout();
  motion_speed_layout->addWidget(motion_speed_);
  motion_speed_layout->addWidget(motion_speed_label_);
  target_form->addRow("Non-servo motion speed", motion_speed_layout);
  layout->addLayout(target_form);
  connect(force_threshold_, qOverload<double>(&QDoubleSpinBox::valueChanged),
    this, [this](double) {publishConfig();});
  connect(motion_speed_, &QSlider::valueChanged, this, [this](int value) {
    motion_speed_label_->setText(
      QString("%1% (%1 mm/s service, %2% MoveIt)")
      .arg(value).arg(value * 0.3, 0, 'f', 1));
    publishMotionSpeed();
  });
  reset_button_ = new VisibleTextButton("Reset fault", this);
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
  auto * detect_pallet = new VisibleTextButton("Detect pallet", this);
  auto * lock_pallet = new VisibleTextButton("Accept, save && lock", this);
  auto * apply_pallet = new VisibleTextButton("Apply/save values", this);
  auto * use_configured_pallet = new VisibleTextButton("Load saved pallet && lock", this);
  auto * clear_pallet = new VisibleTextButton("Clear", this);
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

  auto * manual_group = new QGroupBox("Manual operations", this);
  auto * manual_layout = new QVBoxLayout(manual_group);
  auto * open_gripper = new VisibleTextButton("Open gripper", this);
  auto * close_gripper = new VisibleTextButton("Close gripper", this);
  auto * clear_obstacles = new VisibleTextButton("Clear placed obstacles", this);
  manual_layout->addWidget(open_gripper);
  manual_layout->addWidget(close_gripper);
  manual_layout->addWidget(clear_obstacles);
  manual_operations_label_ = new QLabel("Manual controls: ready", this);
  manual_operations_label_->setWordWrap(true);
  manual_layout->addWidget(manual_operations_label_);
  layout->addWidget(manual_group);
  connect(open_gripper, &QPushButton::clicked,
    this, &SafeServoPanel::openGripper);
  connect(close_gripper, &QPushButton::clicked,
    this, &SafeServoPanel::closeGripper);
  connect(clear_obstacles, &QPushButton::clicked,
    this, &SafeServoPanel::clearPlacedObstacles);

  auto * motion_group = new QGroupBox("MoveIt motion", this);
  auto * motion_layout = new QVBoxLayout(motion_group);
  auto * plan_buttons = new QVBoxLayout();
  auto * set_observation = new VisibleTextButton("Set current as observation", this);
  auto * plan_observation = new VisibleTextButton("Plan observation", this);
  set_observation->setToolTip(
    "Overwrite the saved observation pose with the robot's current stationary joint pose");
  plan_buttons->addWidget(set_observation);
  plan_buttons->addWidget(plan_observation);
  motion_layout->addLayout(plan_buttons);
  auto * motion_buttons = new QVBoxLayout();
  auto * execute_motion = new VisibleTextButton("Execute latest plan", this);
  auto * cancel_motion = new VisibleTextButton("Cancel motion", this);
  motion_buttons->addWidget(execute_motion);
  motion_buttons->addWidget(cancel_motion);
  motion_layout->addLayout(motion_buttons);
  auto * reset_motion = new VisibleTextButton("Reset motion coordinator", this);
  motion_layout->addWidget(reset_motion);
  motion_state_label_ = new QLabel("Motion coordinator: unavailable", this);
  motion_state_label_->setWordWrap(true);
  motion_layout->addWidget(motion_state_label_);
  layout->addWidget(motion_group);
  connect(set_observation, &QPushButton::clicked,
    this, &SafeServoPanel::setObservationPose);
  connect(plan_observation, &QPushButton::clicked,
    this, &SafeServoPanel::planObservation);
  connect(execute_motion, &QPushButton::clicked,
    this, &SafeServoPanel::executeMotionPlan);
  connect(cancel_motion, &QPushButton::clicked,
    this, &SafeServoPanel::cancelMotion);
  connect(reset_motion, &QPushButton::clicked,
    this, &SafeServoPanel::resetMotionCoordinator);

  auto * random_loading_group = new QGroupBox("Stable random loading", this);
  auto * random_loading_layout = new QVBoxLayout(random_loading_group);
  auto * start_random_loading = new VisibleTextButton("Random loading", this);
  auto * abort_random_loading = new VisibleTextButton("Abort", this);
  auto * reset_random_loading = new VisibleTextButton("Reset", this);
  com_bound_ratio_ = new QSlider(Qt::Horizontal, this);
  com_bound_ratio_->setRange(1, 100);
  com_bound_ratio_->setSingleStep(1);
  com_bound_ratio_->setPageStep(5);
  com_bound_ratio_->setValue(20);
  com_bound_ratio_->setToolTip(
    "Central fraction of the real item XY footprint that must be fully "
    "contained by the support hull. Larger values are more conservative.");
  com_bound_ratio_label_ = new QLabel(
    "COM bound ratio: 20% of item X/Y", this);
  continuous_random_loading_ = new QCheckBox(
    "Continue when next item is stable", this);
  random_add_placed_item_obstacle_ = new QCheckBox(
    "Register placed items as static obstacles", this);
  random_add_placed_item_obstacle_->setChecked(true);
  start_random_loading->setToolTip(
    "Measure the refined item, select a vertically reachable stable target, "
    "and run PickAndPlace");
  abort_random_loading->setToolTip(
    "Stop the active cycle and discard its uncommitted target");
  reset_random_loading->setToolTip(
    "Clear the virtual pallet, stability state, and Three.js scene");
  continuous_random_loading_->setToolTip(
    "After returning to observation, wait for a stable refined item and "
    "automatically run the next loading cycle");
  random_add_placed_item_obstacle_->setToolTip(
    "After release, retain the measured item as a world collision object "
    "for subsequent MoveIt plans and display it in RViz");
  random_loading_layout->addWidget(com_bound_ratio_label_);
  random_loading_layout->addWidget(com_bound_ratio_);
  random_loading_layout->addWidget(continuous_random_loading_);
  random_loading_layout->addWidget(random_add_placed_item_obstacle_);
  placed_obstacles_label_ = new QLabel(
    "Placed obstacles: waiting for planning scene", this);
  placed_obstacles_label_->setWordWrap(true);
  random_loading_layout->addWidget(placed_obstacles_label_);
  random_loading_layout->addWidget(start_random_loading);
  random_loading_layout->addWidget(abort_random_loading);
  random_loading_layout->addWidget(reset_random_loading);
  random_loading_state_label_ = new QLabel(
    "Stable random loading: unavailable", this);
  random_loading_state_label_->setWordWrap(true);
  random_loading_layout->addWidget(random_loading_state_label_);
  layout->addWidget(random_loading_group);
  connect(start_random_loading, &QPushButton::clicked,
    this, &SafeServoPanel::startRandomLoading);
  connect(abort_random_loading, &QPushButton::clicked,
    this, &SafeServoPanel::abortRandomLoading);
  connect(reset_random_loading, &QPushButton::clicked,
    this, &SafeServoPanel::resetRandomLoading);
  connect(com_bound_ratio_, &QSlider::valueChanged, this, [this](int value) {
    com_bound_ratio_label_->setText(
      QString("COM bound ratio: %1% of item X/Y").arg(value));
    publishRandomLoadingConfig();
  });
  connect(continuous_random_loading_, &QCheckBox::toggled,
    this, &SafeServoPanel::setContinuousRandomLoading);

  auto * pickup_group = new QGroupBox("PickAndPlace cycle", this);
  auto * pickup_layout = new QVBoxLayout(pickup_group);
  auto * pickup_buttons = new QVBoxLayout();
  auto * start_pickup = new VisibleTextButton("Start PickAndPlace", this);
  auto * abort_pickup = new VisibleTextButton("Abort", this);
  auto * reset_pickup = new VisibleTextButton("Reset", this);
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
  rotate_item_90_ = new QCheckBox(
    "Rotate item 90 deg clockwise about pallet Z", this);
  pre_place_form->addRow(rotate_item_90_);
  keep_eef_perpendicular_ =
    new QCheckBox("Keep EEF perpendicular to pallet", this);
  keep_eef_perpendicular_->setChecked(true);
  keep_eef_perpendicular_->setToolTip(
    "Aligns tool Z with the downward pallet normal while preserving placement yaw");
  pre_place_form->addRow(keep_eef_perpendicular_);
  add_placed_item_obstacle_ =
    new QCheckBox("Add placed item as MoveIt obstacle", this);
  add_placed_item_obstacle_->setChecked(true);
  pre_place_form->addRow(add_placed_item_obstacle_);
  place_layout->addLayout(pre_place_form);
  auto * save_place_config = new VisibleTextButton("Apply/save place target", this);
  auto * place_config_buttons = new QVBoxLayout();
  place_config_buttons->addWidget(save_place_config);
  place_layout->addLayout(place_config_buttons);
  place_state_label_ = new QLabel("Place pipeline: unavailable", this);
  place_state_label_->setWordWrap(true);
  place_layout->addWidget(place_state_label_);
  layout->addWidget(place_group);
  connect(save_place_config, &QPushButton::clicked,
    this, &SafeServoPanel::applyPalletConfig);
  connect(random_add_placed_item_obstacle_, &QCheckBox::toggled,
    this, [this](bool enabled) {
      const QSignalBlocker blocker(add_placed_item_obstacle_);
      add_placed_item_obstacle_->setChecked(enabled);
      applyPalletConfig();
    });
  connect(add_placed_item_obstacle_, &QCheckBox::toggled,
    this, [this](bool enabled) {
      const QSignalBlocker blocker(random_add_placed_item_obstacle_);
      random_add_placed_item_obstacle_->setChecked(enabled);
      applyPalletConfig();
    });

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
  motion_speed_config_pub_ =
    node_->create_publisher<std_msgs::msg::Float64MultiArray>(
    "/motion_speed/config", 10);
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
        tcp_telemetry_[i]->setText(
          QString::fromLatin1(names[i]) + QStringLiteral(": ") + text);
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
        keep_eef_perpendicular_->setChecked(msg->data[13] > 0.5);
      }
      if (msg->data.size() >= 15) {
        const bool enabled = msg->data[14] > 0.5;
        const QSignalBlocker place_blocker(add_placed_item_obstacle_);
        const QSignalBlocker random_blocker(random_add_placed_item_obstacle_);
        add_placed_item_obstacle_->setChecked(enabled);
        random_add_placed_item_obstacle_->setChecked(enabled);
      }
    });
  planning_scene_status_sub_ =
    node_->create_subscription<std_msgs::msg::String>(
    "/planning_scene_obstacles/status", 10,
    [this](const std_msgs::msg::String::SharedPtr msg) {
      const auto json = QJsonDocument::fromJson(
        QByteArray::fromStdString(msg->data)).object();
      const bool enabled = json.value("add_placed_item_obstacle").toBool(true);
      const int count = json.value("placed_item_count").toInt(0);
      const QSignalBlocker place_blocker(add_placed_item_obstacle_);
      const QSignalBlocker random_blocker(random_add_placed_item_obstacle_);
      add_placed_item_obstacle_->setChecked(enabled);
      random_add_placed_item_obstacle_->setChecked(enabled);
      placed_obstacles_label_->setText(
        QString("Placed obstacles: %1, registered: %2")
        .arg(enabled ? "enabled" : "disabled").arg(count));
    });
  clear_placed_obstacles_client_ = node_->create_client<std_srvs::srv::Trigger>(
    "/planning_scene_obstacles/clear_placed_items");
  set_gripper_client_ = node_->create_client<std_srvs::srv::SetBool>(
    "/pickup_supervisor/set_gripper");
  save_observation_client_ = node_->create_client<std_srvs::srv::Trigger>(
    "/taught_waypoints/save_observation");
  plan_observation_client_ = node_->create_client<std_srvs::srv::Trigger>(
    "/motion_coordinator/plan_observation");
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
  start_random_loading_client_ = node_->create_client<std_srvs::srv::Trigger>(
    "/random_stable_loading/start");
  abort_random_loading_client_ = node_->create_client<std_srvs::srv::Trigger>(
    "/random_stable_loading/abort");
  reset_random_loading_client_ = node_->create_client<std_srvs::srv::Trigger>(
    "/random_stable_loading/reset_pallet");
  continuous_random_loading_client_ =
    node_->create_client<std_srvs::srv::SetBool>(
    "/random_stable_loading/set_continuous");
  random_loading_config_pub_ =
    node_->create_publisher<std_msgs::msg::Float64MultiArray>(
    "/random_stable_loading/config", 10);
  random_loading_status_sub_ =
    node_->create_subscription<std_msgs::msg::String>(
    "/random_stable_loading/status", 10,
    [this](const std_msgs::msg::String::SharedPtr msg) {
      const auto json = QJsonDocument::fromJson(
        QByteArray::fromStdString(msg->data)).object();
      QString text = QString("Stable random loading: %1")
        .arg(json.value("state").toString("UNKNOWN"));
      if (json.contains("continuous_loading_enabled")) {
        const bool enabled =
          json.value("continuous_loading_enabled").toBool(false);
        const QSignalBlocker blocker(continuous_random_loading_);
        continuous_random_loading_->setChecked(enabled);
        text += QString("\nContinuous: %1%2")
          .arg(enabled ? "enabled" : "disabled")
          .arg(json.value("continuous_run_active").toBool(false) ?
            " (running)" : "");
      }
      if (json.contains("com_bound_ratio")) {
        const int percent = static_cast<int>(
          json.value("com_bound_ratio").toDouble(0.2) * 100.0 + 0.5);
        const QSignalBlocker blocker(com_bound_ratio_);
        com_bound_ratio_->setValue(percent);
        com_bound_ratio_label_->setText(
          QString("COM bound ratio: %1% of item X/Y").arg(percent));
      }
      if (json.contains("stable_candidate_count")) {
        text += QString("\nCandidates stable/vertical/sample: %1/%2/%3")
          .arg(json.value("stable_candidate_count").toInt())
          .arg(json.value("vertical_candidate_count").toInt())
          .arg(json.value("sampled_candidate_count").toInt());
      }
      const auto fault = json.value("fault").toString();
      const auto result = json.value("last_result").toString();
      if (!fault.isEmpty()) {
        text += QString("\n%1").arg(fault);
      } else if (!result.isEmpty()) {
        text += QString("\n%1").arg(result);
      }
      random_loading_state_label_->setText(text);
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

void SafeServoPanel::publishMotionSpeed()
{
  if (!motion_speed_config_pub_) {return;}
  std_msgs::msg::Float64MultiArray msg;
  const double operator_percent = static_cast<double>(motion_speed_->value());
  msg.data.push_back(operator_percent * 0.003);  // MoveIt: 0.015 .. 0.30.
  msg.data.push_back(operator_percent);  // Direct service: 5 .. 100 mm/s.
  motion_speed_config_pub_->publish(msg);
}

void SafeServoPanel::publishRandomLoadingConfig()
{
  if (!random_loading_config_pub_) {return;}
  std_msgs::msg::Float64MultiArray msg;
  msg.data.push_back(static_cast<double>(com_bound_ratio_->value()) / 100.0);
  random_loading_config_pub_->publish(msg);
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
    keep_eef_perpendicular_->isChecked() ? 1.0 : 0.0,
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

void SafeServoPanel::clearPlacedObstacles()
{
  if (!clear_placed_obstacles_client_->service_is_ready()) {
    manual_operations_label_->setText("Obstacle-clear service unavailable");
    return;
  }
  auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
  clear_placed_obstacles_client_->async_send_request(request, [this](
    rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
    manual_operations_label_->setText(QString::fromStdString(future.get()->message));
  });
}

void SafeServoPanel::openGripper()
{
  setGripper(false);
}

void SafeServoPanel::closeGripper()
{
  setGripper(true);
}

void SafeServoPanel::setGripper(bool close)
{
  if (!set_gripper_client_->service_is_ready()) {
    manual_operations_label_->setText("Manual gripper service unavailable");
    return;
  }
  auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
  request->data = close;
  set_gripper_client_->async_send_request(request, [this](
    rclcpp::Client<std_srvs::srv::SetBool>::SharedFuture future) {
    manual_operations_label_->setText(QString::fromStdString(future.get()->message));
  });
}

void SafeServoPanel::setObservationPose()
{
  const auto answer = QMessageBox::question(
    this,
    "Set observation pose",
    "Overwrite the saved observation pose with the robot's current joint pose?\n\n"
    "Make sure the robot is stopped at a safe observation configuration.",
    QMessageBox::Yes | QMessageBox::No,
    QMessageBox::No);
  if (answer != QMessageBox::Yes) {
    return;
  }
  if (!save_observation_client_->service_is_ready()) {
    motion_state_label_->setText("Observation-save service unavailable");
    return;
  }
  auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
  save_observation_client_->async_send_request(request, [this](
    rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
    motion_state_label_->setText(QString::fromStdString(future.get()->message));
  });
}

void SafeServoPanel::planObservation()
{
  if (!plan_observation_client_->service_is_ready()) {
    motion_state_label_->setText("Motion coordinator unavailable");
    return;
  }
  publishMotionSpeed();
  QTimer::singleShot(50, this, [this]() {
    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    plan_observation_client_->async_send_request(request, [this](
      rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
      motion_state_label_->setText(QString::fromStdString(future.get()->message));
    });
  });
}

void SafeServoPanel::planPrePlace()
{
  if (!plan_pre_place_client_->service_is_ready()) {
    motion_state_label_->setText("Motion coordinator unavailable");
    return;
  }
  publishMotionSpeed();
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
  publishMotionSpeed();
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
  publishMotionSpeed();
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

void SafeServoPanel::startRandomLoading()
{
  if (!start_random_loading_client_->service_is_ready()) {
    random_loading_state_label_->setText(
      "Stable random loading service unavailable");
    return;
  }
  publishConfig();
  publishMotionSpeed();
  publishRandomLoadingConfig();
  QTimer::singleShot(100, this, [this]() {
    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    start_random_loading_client_->async_send_request(request, [this](
      rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
      random_loading_state_label_->setText(
        QString::fromStdString(future.get()->message));
    });
  });
}

void SafeServoPanel::setContinuousRandomLoading(bool enabled)
{
  if (!continuous_random_loading_client_ ||
    !continuous_random_loading_client_->service_is_ready())
  {
    random_loading_state_label_->setText(
      "Continuous loading service unavailable");
    const QSignalBlocker blocker(continuous_random_loading_);
    continuous_random_loading_->setChecked(!enabled);
    return;
  }
  auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
  request->data = enabled;
  continuous_random_loading_client_->async_send_request(request, [this](
    rclcpp::Client<std_srvs::srv::SetBool>::SharedFuture future) {
    random_loading_state_label_->setText(
      QString::fromStdString(future.get()->message));
  });
}

void SafeServoPanel::abortRandomLoading()
{
  if (!abort_random_loading_client_->service_is_ready()) {
    random_loading_state_label_->setText(
      "Stable random loading abort service unavailable");
    return;
  }
  auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
  abort_random_loading_client_->async_send_request(request, [this](
    rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
    random_loading_state_label_->setText(
      QString::fromStdString(future.get()->message));
  });
}

void SafeServoPanel::resetRandomLoading()
{
  const auto answer = QMessageBox::question(
    this,
    "Reset stable random loading",
    "Clear the virtual pallet, stability history, and Three.js scene?\n\n"
    "Only continue when the physical pallet is empty.",
    QMessageBox::Yes | QMessageBox::No,
    QMessageBox::No);
  if (answer != QMessageBox::Yes) {
    return;
  }
  if (!reset_random_loading_client_->service_is_ready()) {
    random_loading_state_label_->setText(
      "Stable random loading reset service unavailable");
    return;
  }
  auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
  reset_random_loading_client_->async_send_request(request, [this](
    rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
    random_loading_state_label_->setText(
      QString::fromStdString(future.get()->message));
  });
}

}

PLUGINLIB_EXPORT_CLASS(safe_servo_rviz_panel::SafeServoPanel, rviz_common::Panel)
