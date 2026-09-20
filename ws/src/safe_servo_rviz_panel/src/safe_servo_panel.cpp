#include "safe_servo_rviz_panel/safe_servo_panel.hpp"

#include <QDoubleSpinBox>
#include <QCheckBox>
#include <QComboBox>
#include <QFormLayout>
#include <QLabel>
#include <QPainter>
#include <QJsonDocument>
#include <QJsonArray>
#include <QJsonObject>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QMessageBox>
#include <QPushButton>
#include <QScrollArea>
#include <QSignalBlocker>
#include <QSlider>
#include <QSpinBox>
#include <QTimer>
#include <QVBoxLayout>
#include <QtMath>
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

  auto * box_estimation_group = new QGroupBox("Box dimension estimation", this);
  auto * box_estimation_layout = new QVBoxLayout(box_estimation_group);
  depth_only_estimation_ = new QCheckBox(
    "Use depth only (no marker)", box_estimation_group);
  depth_only_estimation_->setToolTip(
    "Crop the aligned depth cloud by base height and TCP-relative bounds, "
    "then fit the horizontal top plane");
  box_dimensions_label_ = new QLabel(
    "X: -- mm   Y: -- mm\nZ: -- mm", box_estimation_group);
  box_dimensions_label_->setWordWrap(true);
  box_dimensions_label_->setMinimumHeight(44);
  box_estimation_status_label_ = new QLabel(
    "Estimator: waiting for depth", box_estimation_group);
  box_estimation_status_label_->setWordWrap(true);
  box_estimation_status_label_->setMinimumHeight(44);
  box_stability_label_ = new QLabel(
    "Box stability: IDLE", box_estimation_group);
  box_stability_label_->setWordWrap(true);
  box_estimation_layout->addWidget(depth_only_estimation_);
  const char * bound_names[] = {
    "Base Z min", "Base Z max", "TCP X min",
    "TCP X max", "TCP Y min", "TCP Y max"};
  const auto env_millimeters = [](const char * name, int fallback) {
      bool valid = false;
      const double meters = QString::fromUtf8(qgetenv(name)).toDouble(&valid);
      return valid ? qRound(meters * 1000.0) : fallback;
    };
  const int bound_defaults[] = {
    env_millimeters("DEPTH_ONLY_BASE_Z_MIN_M", 70),
    env_millimeters("DEPTH_ONLY_BASE_Z_MAX_M", 300),
    env_millimeters("DEPTH_ONLY_TCP_X_MIN_M", 50),
    env_millimeters("DEPTH_ONLY_TCP_X_MAX_M", 400),
    env_millimeters("DEPTH_ONLY_TCP_Y_MIN_M", -300),
    env_millimeters("DEPTH_ONLY_TCP_Y_MAX_M", 300)};
  const int bound_minimums[] = {-200, -190, -1000, -990, -1000, -1000};
  const int bound_maximums[] = {500, 700, 1000, 1500, 1000, 1000};
  for (size_t i = 0; i < depth_only_bounds_.size(); ++i) {
    auto * row = new QHBoxLayout();
    auto * name = new QLabel(QString::fromLatin1(bound_names[i]), box_estimation_group);
    name->setMinimumWidth(82);
    depth_only_bounds_[i] = new QSlider(Qt::Horizontal, box_estimation_group);
    depth_only_bounds_[i]->setRange(bound_minimums[i], bound_maximums[i]);
    depth_only_bounds_[i]->setSingleStep(1);
    depth_only_bounds_[i]->setPageStep(10);
    depth_only_bounds_[i]->setValue(bound_defaults[i]);
    depth_only_bound_labels_[i] = new QLabel(
      QString("%1 mm").arg(bound_defaults[i]), box_estimation_group);
    depth_only_bound_labels_[i]->setMinimumWidth(58);
    row->addWidget(name);
    row->addWidget(depth_only_bounds_[i], 1);
    row->addWidget(depth_only_bound_labels_[i]);
    box_estimation_layout->addLayout(row);
    connect(depth_only_bounds_[i], &QSlider::valueChanged,
      this, [this, i](int value) {
        depth_only_bound_labels_[i]->setText(QString("%1 mm").arg(value));
        publishDepthOnlyBounds();
      });
  }
  depth_only_height_offset_ = new QSpinBox(box_estimation_group);
  depth_only_height_offset_->setRange(-30, 30);
  depth_only_height_offset_->setSingleStep(1);
  depth_only_height_offset_->setSuffix(" mm");
  depth_only_height_offset_->setValue(
    env_millimeters("DEPTH_ONLY_HEIGHT_OFFSET_M", -20));
  depth_only_height_offset_->setToolTip(
    "Added to the raw depth height before publishing; "
    "170 mm with -20 mm becomes 150 mm");
  auto * height_offset_row = new QHBoxLayout();
  auto * height_offset_name = new QLabel(
    "Height dz offset", box_estimation_group);
  height_offset_name->setMinimumWidth(82);
  height_offset_row->addWidget(height_offset_name);
  height_offset_row->addWidget(depth_only_height_offset_, 1);
  box_estimation_layout->addLayout(height_offset_row);
  connect(depth_only_height_offset_, qOverload<int>(&QSpinBox::valueChanged),
    this, [this](int) {publishDepthOnlyBounds();});
  contact_reference_z_ = new QSpinBox(box_estimation_group);
  contact_reference_z_->setRange(-500, 500);
  contact_reference_z_->setSingleStep(1);
  contact_reference_z_->setSuffix(" mm");
  contact_reference_z_->setValue(
    env_millimeters("OBJECT_CONTACT_REFERENCE_Z_M", 0));
  contact_reference_z_->setToolTip(
    "TCP Z when the tool touches the empty support surface; measured box "
    "height is contact TCP Z minus this value");
  auto * contact_reference_row = new QHBoxLayout();
  auto * contact_reference_name = new QLabel(
    "Empty-table contact Z", box_estimation_group);
  contact_reference_name->setMinimumWidth(82);
  contact_reference_row->addWidget(contact_reference_name);
  contact_reference_row->addWidget(contact_reference_z_, 1);
  box_estimation_layout->addLayout(contact_reference_row);
  connect(contact_reference_z_, qOverload<int>(&QSpinBox::valueChanged),
    this, [this](int) {publishObjectInfoConfig();});
  auto * estimate_object_info = new VisibleTextButton(
    "Estimate Object Info", box_estimation_group);
  object_info_state_label_ = new QLabel(
    "Object info: NOT_OBTAINED", box_estimation_group);
  object_info_state_label_->setWordWrap(true);
  box_estimation_layout->addWidget(estimate_object_info);
  box_estimation_layout->addWidget(object_info_state_label_);
  connect(estimate_object_info, &QPushButton::clicked,
    this, &SafeServoPanel::estimateObjectInfo);
  box_estimation_layout->addWidget(box_dimensions_label_);
  box_estimation_layout->addWidget(box_estimation_status_label_);
  box_estimation_layout->addWidget(box_stability_label_);
  layout->addWidget(box_estimation_group);
  connect(depth_only_estimation_, &QCheckBox::toggled,
    this, &SafeServoPanel::setDepthOnlyEstimation);

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
  motion_speed_->setValue(80);
  motion_speed_->setToolTip(
    "Scales the commissioned MoveIt, joint-transfer, and direct Cartesian "
    "motion envelopes. Safe-servo speed is unchanged.");
  motion_speed_label_ = new QLabel("80% of non-servo motion envelope", this);
  auto * motion_speed_layout = new QVBoxLayout();
  motion_speed_layout->addWidget(motion_speed_);
  motion_speed_layout->addWidget(motion_speed_label_);
  target_form->addRow("Non-servo motion speed", motion_speed_layout);
  layout->addLayout(target_form);
  connect(force_threshold_, qOverload<double>(&QDoubleSpinBox::valueChanged),
    this, [this](double) {publishConfig();});
  connect(motion_speed_, &QSlider::valueChanged, this, [this](int value) {
    motion_speed_label_->setText(
      QString("%1% of non-servo motion envelope").arg(value));
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
  auto * clear_picked_item = new VisibleTextButton("Clear picked item", this);
  auto * recover_ft_sensor = new VisibleTextButton("Recover FT sensor", this);
  clear_picked_item->setToolTip(
    "Remove only the attached item from the MoveIt planning scene; "
    "do not command or open the physical gripper");
  recover_ft_sensor->setToolTip(
    "Disable, clear, enable, and zero the force/torque sensor. "
    "Use only with no item held and no external tool contact.");
  manual_layout->addWidget(open_gripper);
  manual_layout->addWidget(close_gripper);
  manual_layout->addWidget(clear_obstacles);
  manual_layout->addWidget(clear_picked_item);
  manual_layout->addWidget(recover_ft_sensor);
  manual_operations_label_ = new QLabel("Manual controls: ready", this);
  manual_operations_label_->setWordWrap(true);
  manual_layout->addWidget(manual_operations_label_);
  ft_recovery_label_ = new QLabel("FT sensor: ready", this);
  ft_recovery_label_->setWordWrap(true);
  manual_layout->addWidget(ft_recovery_label_);
  layout->addWidget(manual_group);
  connect(open_gripper, &QPushButton::clicked,
    this, &SafeServoPanel::openGripper);
  connect(close_gripper, &QPushButton::clicked,
    this, &SafeServoPanel::closeGripper);
  connect(clear_obstacles, &QPushButton::clicked,
    this, &SafeServoPanel::clearPlacedObstacles);
  connect(clear_picked_item, &QPushButton::clicked,
    this, &SafeServoPanel::clearPickedItem);
  connect(recover_ft_sensor, &QPushButton::clicked,
    this, &SafeServoPanel::recoverFtSensor);

  auto * staging_group = new QGroupBox("Unpacking staging slots", this);
  auto * staging_layout = new QVBoxLayout(staging_group);
  auto * staging_form = new QFormLayout();
  staging_form->setRowWrapPolicy(QFormLayout::WrapAllRows);
  staging_store_slot_ = new QComboBox(this);
  staging_retrieve_slot_ = new QComboBox(this);
  const QStringList staging_corners = {
    "0: (-375, 180, 0) mm", "1: (-125, 180, 0) mm",
    "2: (125, 180, 0) mm", "3: (-375, 430, 0) mm",
    "4: (-125, 430, 0) mm", "5: (125, 430, 0) mm"};
  staging_store_slot_->addItems(staging_corners);
  staging_retrieve_slot_->addItems(staging_corners);
  staging_form->addRow("Store carried item in", staging_store_slot_);
  staging_form->addRow("Retrieve item from", staging_retrieve_slot_);
  staging_layout->addLayout(staging_form);
  auto * store_staging = new VisibleTextButton("Store carried item", this);
  auto * retrieve_staging = new VisibleTextButton("Retrieve staged item", this);
  auto * reset_staging = new VisibleTextButton("Reset staging fault", this);
  store_staging->setToolTip(
    "Requires an attached carried item; aligns its FLB to the selected slot and yaw to 0 deg");
  retrieve_staging->setToolTip(
    "Uses the recorded release TCP pose; no item redetection is performed");
  staging_layout->addWidget(store_staging);
  staging_layout->addWidget(retrieve_staging);
  staging_layout->addWidget(reset_staging);
  staging_state_label_ = new QLabel("Staging slots: unavailable", this);
  staging_state_label_->setWordWrap(true);
  staging_layout->addWidget(staging_state_label_);
  layout->addWidget(staging_group);
  connect(store_staging, &QPushButton::clicked,
    this, &SafeServoPanel::storeInStagingSlot);
  connect(retrieve_staging, &QPushButton::clicked,
    this, &SafeServoPanel::retrieveFromStagingSlot);
  connect(reset_staging, &QPushButton::clicked,
    this, &SafeServoPanel::resetStagingSlots);
  connect(staging_store_slot_, qOverload<int>(&QComboBox::currentIndexChanged),
    this, [this](int index) {
      if (!staging_store_selection_pub_) {return;}
      std_msgs::msg::Int32 message;
      message.data = index;
      staging_store_selection_pub_->publish(message);
    });
  connect(staging_retrieve_slot_, qOverload<int>(&QComboBox::currentIndexChanged),
    this, [this](int index) {
      if (!staging_retrieve_selection_pub_) {return;}
      std_msgs::msg::Int32 message;
      message.data = index;
      staging_retrieve_selection_pub_->publish(message);
    });

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
  com_bound_ratio_->setValue(30);
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

  auto * policy_loading_group = new QGroupBox("Learned policy loading", this);
  auto * policy_loading_layout = new QVBoxLayout(policy_loading_group);
  continuous_policy_loading_ = new QCheckBox(
    "Continue with the next detected item", this);
  policy_repack_planning_ = new QCheckBox(
    "Enable MCTS + A* unpack/repack planning", this);
  policy_simulation_ = new QCheckBox(
    "Automatic simulation (cardboard; no robot commands)", this);
  policy_add_placed_item_obstacle_ = new QCheckBox(
    "Register placed items as static obstacles", this);
  policy_add_placed_item_obstacle_->setChecked(true);
  continuous_policy_loading_->setToolTip(
    "After a successful policy PickAndPlace, estimate and load the next item");
  policy_repack_planning_->setToolTip(
    "When a direct policy placement is blocked, plan and execute a transactional "
    "pack/unpack/repack sequence using the six staging slots.");
  policy_simulation_->setToolTip(
    "Automatically sample cardboard items, solve fixed-descent "
    "pack/unpack/repack trajectories with MoveIt, and animate them in RViz "
    "without execution, gripper, Servo, controller-switch, or force-sensor "
    "commands. Enable continuous loading to keep sampling.");
  policy_add_placed_item_obstacle_->setToolTip(
    "After release, register the placed item as a MoveIt collision object");
  policy_loading_layout->addWidget(continuous_policy_loading_);
  policy_loading_layout->addWidget(policy_repack_planning_);
  policy_loading_layout->addWidget(policy_simulation_);
  policy_loading_layout->addWidget(policy_add_placed_item_obstacle_);
  auto * start_policy_loading = new VisibleTextButton("Policy loading", this);
  auto * plan_policy_loading = new VisibleTextButton("Plan target only", this);
  auto * abort_policy_loading = new VisibleTextButton("Abort", this);
  auto * reset_policy_loading = new VisibleTextButton("Reset", this);
  start_policy_loading->setToolTip(
    "Estimate object information, evaluate the learned policy, and run PickAndPlace");
  plan_policy_loading->setToolTip(
    "Estimate object information and publish the policy target without grasping");
  abort_policy_loading->setToolTip(
    "Stop the active policy cycle and discard its uncommitted target");
  reset_policy_loading->setToolTip(
    "Clear the policy virtual pallet and its Three.js scene");
  policy_loading_layout->addWidget(start_policy_loading);
  policy_loading_layout->addWidget(plan_policy_loading);
  policy_loading_layout->addWidget(abort_policy_loading);
  policy_loading_layout->addWidget(reset_policy_loading);
  policy_loading_state_label_ = new QLabel(
    "Learned policy loading: unavailable", this);
  policy_loading_state_label_->setWordWrap(true);
  policy_loading_layout->addWidget(policy_loading_state_label_);
  layout->addWidget(policy_loading_group);
  connect(start_policy_loading, &QPushButton::clicked,
    this, &SafeServoPanel::startPolicyLoading);
  connect(plan_policy_loading, &QPushButton::clicked,
    this, &SafeServoPanel::planPolicyLoading);
  connect(abort_policy_loading, &QPushButton::clicked,
    this, &SafeServoPanel::abortPolicyLoading);
  connect(reset_policy_loading, &QPushButton::clicked,
    this, &SafeServoPanel::resetPolicyLoading);
  connect(continuous_policy_loading_, &QCheckBox::toggled,
    this, &SafeServoPanel::setContinuousPolicyLoading);
  connect(policy_repack_planning_, &QCheckBox::toggled, this,
    [this](bool enabled) {
      if (!policy_rearrangement_client_ ||
        !policy_rearrangement_client_->service_is_ready())
      {
        policy_loading_state_label_->setText(
          "MCTS/A* configuration service unavailable");
        const QSignalBlocker blocker(policy_repack_planning_);
        policy_repack_planning_->setChecked(!enabled);
        return;
      }
      auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
      request->data = enabled;
      policy_rearrangement_client_->async_send_request(request, [this](
        rclcpp::Client<std_srvs::srv::SetBool>::SharedFuture future) {
        policy_loading_state_label_->setText(
          QString::fromStdString(future.get()->message));
      });
    });
  connect(policy_simulation_, &QCheckBox::toggled, this,
    [this](bool enabled) {
      if (!policy_simulation_client_ ||
        !policy_simulation_client_->service_is_ready())
      {
        policy_loading_state_label_->setText(
          "Policy simulation service unavailable");
        const QSignalBlocker blocker(policy_simulation_);
        policy_simulation_->setChecked(!enabled);
        return;
      }
      auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
      request->data = enabled;
      policy_simulation_client_->async_send_request(request, [this](
        rclcpp::Client<std_srvs::srv::SetBool>::SharedFuture future) {
        const auto result = future.get();
        if (!result->success) {
          const QSignalBlocker blocker(policy_simulation_);
          policy_simulation_->setChecked(!policy_simulation_->isChecked());
        }
        policy_loading_state_label_->setText(
          QString::fromStdString(result->message));
      });
    });

  auto * pickup_group = new QGroupBox("PickAndPlace cycle", this);
  auto * pickup_layout = new QVBoxLayout(pickup_group);
  auto * pickup_buttons = new QVBoxLayout();
  auto * start_pickup = new VisibleTextButton("Start PickAndPlace", this);
  auto * abort_pickup = new VisibleTextButton("Abort", this);
  auto * reset_pickup = new VisibleTextButton("Reset", this);
  pick_only_ = new QCheckBox("Pick only (skip placement)", this);
  pick_only_->setToolTip(
    "Run pickup and retreat only; leave the item attached to the TCP");
  pickup_layout->addWidget(pick_only_);
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
      const QSignalBlocker policy_blocker(policy_add_placed_item_obstacle_);
      add_placed_item_obstacle_->setChecked(enabled);
      policy_add_placed_item_obstacle_->setChecked(enabled);
      applyPalletConfig();
    });
  connect(policy_add_placed_item_obstacle_, &QCheckBox::toggled,
    this, [this](bool enabled) {
      const QSignalBlocker place_blocker(add_placed_item_obstacle_);
      const QSignalBlocker random_blocker(random_add_placed_item_obstacle_);
      add_placed_item_obstacle_->setChecked(enabled);
      random_add_placed_item_obstacle_->setChecked(enabled);
      applyPalletConfig();
    });
  connect(add_placed_item_obstacle_, &QCheckBox::toggled,
    this, [this](bool enabled) {
      const QSignalBlocker blocker(random_add_placed_item_obstacle_);
      const QSignalBlocker policy_blocker(policy_add_placed_item_obstacle_);
      random_add_placed_item_obstacle_->setChecked(enabled);
      policy_add_placed_item_obstacle_->setChecked(enabled);
      applyPalletConfig();
    });

  auto * test_group = new QGroupBox("Single-item PickAndPlace tests (real robot)", this);
  auto * test_layout = new QVBoxLayout(test_group);
  auto * test_help = new QLabel(
    "Start with an empty pallet and slot 0. Disable/reset both loaders. "
    "Manual buttons run one step. Random test reuses one item, or two with floor-only mode, choosing valid operations "
    "and empty slots, and stops on any fault. Keep the workspace clear.", this);
  test_help->setWordWrap(true);
  test_layout->addWidget(test_help);
  test_confirm_ = new QCheckBox("Enable deliberate real-robot test commands", this);
  test_layout->addWidget(test_confirm_);
  test_two_items_ = new QCheckBox("Two-item floor-only test (no stacking)", this);
  test_two_items_->setEnabled(false);
  test_two_items_->setToolTip("Enable on an empty/reset test. Load item 1, present item 2 and start again. Then randomly pack/unpack/repack either item.");
  test_layout->addWidget(test_two_items_);
  connect(test_two_items_, &QCheckBox::toggled, this, [this](bool checked) {
    if (!test_two_items_client_ || !test_two_items_client_->service_is_ready()) {
      const QSignalBlocker blocker(test_two_items_);
      test_two_items_->setChecked(!checked);
      return;
    }
    test_mode_pending_ = true;
    test_two_items_->setEnabled(false);
    test_pack_unpack_only_->setEnabled(false);
    test_buttons_[6]->setEnabled(false);
    auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
    request->data = checked;
    test_two_items_client_->async_send_request(request,
      [this, checked](rclcpp::Client<std_srvs::srv::SetBool>::SharedFuture future) {
        bool success = false;
        QString message;
        try {
          auto result = future.get();
          success = result->success;
          message = QString::fromStdString(result->message);
        } catch (const std::exception & error) {message = error.what();}
        QMetaObject::invokeMethod(this, [this, checked, success, message]() {
          test_mode_pending_ = false;
          const QSignalBlocker blocker(test_two_items_);
          test_two_items_->setChecked(success ? checked : !checked);
          test_status_label_->setText(message);
        }, Qt::QueuedConnection);
      });
  });
  test_pack_unpack_only_ = new QCheckBox("Random test: pack / unpack only (no repack)", this);
  test_pack_unpack_only_->setEnabled(false);
  test_pack_unpack_only_->setToolTip(
    "Pack a new item once if needed, then alternate pallet -> random empty slot "
    "and recorded slot -> random pallet target. Manual buttons are unchanged.");
  test_layout->addWidget(test_pack_unpack_only_);
  connect(test_pack_unpack_only_, &QCheckBox::toggled, this, [this](bool checked) {
    if (!test_mode_client_ || !test_mode_client_->service_is_ready()) {
      const QSignalBlocker blocker(test_pack_unpack_only_);
      test_pack_unpack_only_->setChecked(!checked);
      test_status_label_->setText("Test mode service unavailable; setting not changed");
      return;
    }
    test_mode_pending_ = true;
    test_pack_unpack_only_->setEnabled(false);
    test_buttons_[6]->setEnabled(false);
    auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
    request->data = checked;
    test_mode_client_->async_send_request(request,
      [this, checked](rclcpp::Client<std_srvs::srv::SetBool>::SharedFuture future) {
        bool success = false;
        QString message;
        try {
          const auto result = future.get();
          success = result->success;
          message = QString::fromStdString(result->message);
        } catch (const std::exception & error) {
          message = QString("Test mode request failed: %1").arg(error.what());
        }
        QMetaObject::invokeMethod(this, [this, checked, success, message]() {
          test_mode_pending_ = false;
          const QSignalBlocker blocker(test_pack_unpack_only_);
          test_pack_unpack_only_->setChecked(success ? checked : !checked);
          test_status_label_->setText(message);
          // A fresh coordinator status re-enables controls and confirms mode.
        }, Qt::QueuedConnection);
      });
  });
  const char * labels[] = {
    "1. New item -> random pallet", "2. Unpack -> test slot -> overhead",
    "3. Recorded slot -> random pallet", "4. Repack on pallet (new random position)",
    "Abort test (keep gripper state)", "Reset test bookkeeping",
    "Start random robustness test", "Stop random test after current step"};
  for (size_t i = 0; i < test_buttons_.size(); ++i) {
    test_buttons_[i] = new VisibleTextButton(labels[i], this);
    test_buttons_[i]->setEnabled(false);
    test_layout->addWidget(test_buttons_[i]);
    connect(test_buttons_[i], &QPushButton::clicked, this, [this, i]() {runPickPlaceTest(i);});
  }
  test_status_label_ = new QLabel("Test coordinator: unavailable", this);
  test_status_label_->setWordWrap(true);
  test_layout->addWidget(test_status_label_);
  layout->addWidget(test_group);
  test_conflicting_groups_ = {random_loading_group, policy_loading_group, pickup_group,
    place_group, staging_group, motion_group, manual_group, pallet_group,
    box_estimation_group, nullptr};
  test_stale_timer_ = new QTimer(this);
  test_stale_timer_->setSingleShot(true);
  connect(test_stale_timer_, &QTimer::timeout, this, [this]() {
    test_two_items_->setEnabled(false);
    test_pack_unpack_only_->setEnabled(false);
    for (auto * button : test_buttons_) {button->setEnabled(false);}
    // Abort remains available as a best-effort request if telemetry was lost.
    test_buttons_[4]->setEnabled(true);
    test_status_label_->setText("Test status stale: do not start another motion. Stop/reconcile first.");
  });
  connect(test_confirm_, &QCheckBox::toggled, this, [this](bool) {
    if (!test_last_status_.isEmpty() && test_stale_timer_->isActive()) {
      updatePickPlaceTest(test_last_status_);
    }
  });
  layout->addStretch();
}

void SafeServoPanel::runPickPlaceTest(size_t index)
{
  if (index == 6 && test_mode_pending_) {return;}
  auto client = test_clients_[index];
  if (!client || !client->service_is_ready()) {
    test_status_label_->setText("Test service unavailable; no request sent");
    return;
  }
  for (size_t i = 0; i < 4; ++i) {test_buttons_[i]->setEnabled(false);}
  test_buttons_[6]->setEnabled(false);
  client->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>(),
    [this](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
      const auto result = future.get();
      const QString text = QString::fromStdString(result->message);
      QMetaObject::invokeMethod(this, [this, text]() {
        test_status_label_->setText(text);
      }, Qt::QueuedConnection);
    });
}

void SafeServoPanel::updatePickPlaceTest(const QString & payload)
{
  const auto json = QJsonDocument::fromJson(payload.toUtf8()).object();
  if (!json.contains("state")) {return;}
  test_last_status_ = payload;
  test_stale_timer_->start(3000);
  const auto state = json.value("state").toString();
  const bool automatic = json.value("random_active").toBool();
  const bool busy = automatic || (state != "IDLE" && state != "READY" && state != "FAULT");
  if (!test_mode_pending_) {
    const QSignalBlocker two_blocker(test_two_items_);
    test_two_items_->setChecked(json.value("two_item_mode").toBool());
    const QSignalBlocker blocker(test_pack_unpack_only_);
    test_pack_unpack_only_->setChecked(json.value("random_pack_unpack_only").toBool());
  }
  test_pack_unpack_only_->setEnabled(!busy && !test_mode_pending_ &&
    json.contains("random_pack_unpack_only"));
  test_two_items_->setEnabled(!busy && !test_mode_pending_ && state == "IDLE" &&
    json.contains("two_item_mode"));
  const auto allowed = json.value("allowed").toObject();
  const auto blocked = json.value("blocked").toObject();
  const char * steps[] = {"pack_new", "unpack", "pack_slot", "repack"};
  for (size_t i = 0; i < 4; ++i) {
    test_buttons_[i]->setEnabled(!automatic && !json.value("two_item_mode").toBool() &&
      test_confirm_->isChecked() && allowed.value(steps[i]).toBool());
    test_buttons_[i]->setToolTip(blocked.value(steps[i]).toString());
  }
  test_buttons_[4]->setEnabled(busy);
  test_buttons_[5]->setEnabled(!busy && test_confirm_->isChecked());
  test_buttons_[6]->setEnabled(!busy && !test_mode_pending_ && test_confirm_->isChecked() &&
    json.value("random_start_allowed").toBool());
  test_buttons_[7]->setEnabled(automatic);
  // Recovery controls are available again after a latched fault.
  for (auto * group : test_conflicting_groups_) {
    if (group) {group->setEnabled(!busy);}
  }
  const auto downstream = json.value("downstream").toObject();
  test_status_label_->setText(
    QString("Step: %1 | Phase: %2 (%3 s)\nLast confirmed location: %4\n"
    "Supervisor: %5 | Motion: %6 | Staging: %7\n%8")
    .arg(json.value("step").toString(), state)
    .arg(json.value("phase_elapsed_sec").toDouble(), 0, 'f', 1)
    .arg(json.value("location").toString(), downstream.value("supervisor").toString(),
      downstream.value("motion").toString(), downstream.value("staging").toString(),
      json.value("fault").toString()) +
    QString("\nRandom test: %1/%2 completed | slot %3\n%4\nSeed: %5")
    .arg(json.value("random_completed").toInt()).arg(json.value("random_max_steps").toInt())
    .arg(json.value("selected_test_slot").toInt()).arg(json.value("random_message").toString())
    .arg(json.value("random_seed").toVariant().toString()));
  if (json.value("two_item_mode").toBool()) {
    QString inventory("\nTwo-item inventory:");
    const auto items = json.value("test_items").toObject();
    for (auto it = items.begin(); it != items.end(); ++it) {
      const auto entry = it.value().toObject();
      inventory += QString("\nItem %1: %2").arg(it.key(), entry.value("location").toString());
      if (entry.value("location").toString() == "slot") {
        inventory += QString(" %1").arg(entry.value("record").toObject().value("slot_id").toInt());
      }
    }
    test_status_label_->setText(test_status_label_->text() + inventory);
  }
}

void SafeServoPanel::onInitialize()
{
  auto abstraction = getDisplayContext()->getRosNodeAbstraction().lock();
  if (!abstraction) {
    state_label_->setText("RViz ROS node unavailable");
    return;
  }
  node_ = abstraction->get_raw_node();
  const char * test_services[] = {"pack_new", "unpack", "pack_slot", "repack", "abort", "reset",
    "start_random", "stop_random"};
  for (size_t i = 0; i < test_clients_.size(); ++i) {
    test_clients_[i] = node_->create_client<std_srvs::srv::Trigger>(
      std::string("/pick_place_test/") + test_services[i]);
  }
  test_mode_client_ = node_->create_client<std_srvs::srv::SetBool>(
    "/pick_place_test/set_random_pack_unpack_only");
  test_two_items_client_ = node_->create_client<std_srvs::srv::SetBool>(
    "/pick_place_test/set_two_item_mode");
  test_status_sub_ = node_->create_subscription<std_msgs::msg::String>(
    "/pick_place_test/status", 10, [this](std_msgs::msg::String::SharedPtr msg) {
      const QString payload = QString::fromStdString(msg->data);
      QMetaObject::invokeMethod(this, [this, payload]() {
        updatePickPlaceTest(payload);
      }, Qt::QueuedConnection);
    });
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
  depth_only_estimation_client_ =
    node_->create_client<std_srvs::srv::SetBool>(
    "/pointcloud_detection/set_depth_only");
  depth_only_bounds_pub_ =
    node_->create_publisher<std_msgs::msg::Float64MultiArray>(
    "/pointcloud_detection/depth_only_bounds", 10);
  object_info_config_pub_ =
    node_->create_publisher<std_msgs::msg::Float64MultiArray>(
    "/object_info_estimation/config", 10);
  estimate_object_info_client_ =
    node_->create_client<std_srvs::srv::Trigger>(
    "/pickup_pipeline/estimate_object_info");
  object_info_status_sub_ =
    node_->create_subscription<std_msgs::msg::String>(
    "/pickup_pipeline/status", 10,
    [this](const std_msgs::msg::String::SharedPtr msg) {
      const auto json = QJsonDocument::fromJson(
        QByteArray::fromStdString(msg->data)).object();
      const bool obtained = json.value("object_info_obtained").toBool(false);
      QString text = QString("Object info: %1 (%2)")
        .arg(obtained ? "OBTAINED" : "NOT_OBTAINED")
        .arg(json.value("state").toString("UNKNOWN"));
      const auto object = json.value("corrected_object").toObject();
      if (!object.isEmpty()) {
        text += QString("\nSize: %1 x %2 x %3 mm")
          .arg(object.value("size_x_m").toDouble() * 1000.0, 0, 'f', 1)
          .arg(object.value("size_y_m").toDouble() * 1000.0, 0, 'f', 1)
          .arg(object.value("size_z_m").toDouble() * 1000.0, 0, 'f', 1);
      }
      object_info_state_label_->setText(text);
    });
  box_diagnostics_sub_ = node_->create_subscription<std_msgs::msg::String>(
    "/pointcloud_detection/diagnostics", 10,
    [this](const std_msgs::msg::String::SharedPtr msg) {
      const auto reports = QJsonDocument::fromJson(
        QByteArray::fromStdString(msg->data)).array();
      QJsonObject selected;
      const QString requested_method = depth_only_estimation_->isChecked() ?
        "depth_only" : "marker_seeded";
      for (const auto & value : reports) {
        const auto report = value.toObject();
        if (report.value("method").toString() == requested_method) {
          selected = report;
          if (report.value("state").toString() == "READY") {break;}
        }
      }
      if (selected.isEmpty() || selected.value("state").toString() != "READY") {
        // Depth frames can briefly contain too few usable points. Retain the
        // last valid status instead of alternating READY/NO_BOX every frame.
        return;
      }
      const auto dimensions = selected.value("estimated_dimensions_m").toArray();
      if (dimensions.size() < 3) {return;}
      box_dimensions_label_->setText(
        QString("X: %1 mm   Y: %2 mm\nZ: %3 mm")
        .arg(dimensions[0].toDouble() * 1000.0, 0, 'f', 1)
        .arg(dimensions[1].toDouble() * 1000.0, 0, 'f', 1)
        .arg(dimensions[2].toDouble() * 1000.0, 0, 'f', 1));
      QString status = QString("Estimator: %1, fit %2%")
        .arg(requested_method == "depth_only" ? "depth only" : "marker seeded")
        .arg(selected.value("confidence").toDouble() * 100.0, 0, 'f', 1);
      if (requested_method == "depth_only") {
        status += QString("\nRaw Z: %1 mm, dz: %2 mm")
          .arg(selected.value("raw_height_m").toDouble() * 1000.0, 0, 'f', 1)
          .arg(selected.value("height_offset_m").toDouble() * 1000.0, 0, 'f', 1);
      } else {
        status += "\nMeasurement ready";
      }
      box_estimation_status_label_->setText(status);
    });
  item_localization_status_sub_ =
    node_->create_subscription<std_msgs::msg::String>(
    "/item_localization/status", 10,
    [this](const std_msgs::msg::String::SharedPtr msg) {
      box_stability_label_->setText(
        QString("Box stability: %1").arg(QString::fromStdString(msg->data)));
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
        const QSignalBlocker policy_blocker(policy_add_placed_item_obstacle_);
        add_placed_item_obstacle_->setChecked(enabled);
        random_add_placed_item_obstacle_->setChecked(enabled);
        policy_add_placed_item_obstacle_->setChecked(enabled);
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
      const int visual_count =
        json.value("placed_item_visual_count").toInt(count);
      const QSignalBlocker place_blocker(add_placed_item_obstacle_);
      const QSignalBlocker random_blocker(random_add_placed_item_obstacle_);
      const QSignalBlocker policy_blocker(policy_add_placed_item_obstacle_);
      add_placed_item_obstacle_->setChecked(enabled);
      random_add_placed_item_obstacle_->setChecked(enabled);
      policy_add_placed_item_obstacle_->setChecked(enabled);
      placed_obstacles_label_->setText(
        QString("Placed items: visualized %1, collision obstacles %2 (%3)")
        .arg(visual_count).arg(count)
        .arg(enabled ? "enabled" : "disabled"));
    });
  clear_placed_obstacles_client_ = node_->create_client<std_srvs::srv::Trigger>(
    "/planning_scene_obstacles/clear_placed_items");
  clear_picked_item_client_ = node_->create_client<std_srvs::srv::Trigger>(
    "/planning_scene_obstacles/clear_picked_item");
  recover_ft_sensor_client_ = node_->create_client<std_srvs::srv::Trigger>(
    "/pickup_supervisor/recover_ft_sensor");
  pickup_supervisor_status_sub_ =
    node_->create_subscription<std_msgs::msg::String>(
    "/pickup_supervisor/status", 10,
    [this](const std_msgs::msg::String::SharedPtr msg) {
      const auto json = QJsonDocument::fromJson(
        QByteArray::fromStdString(msg->data)).object();
      const auto state = json.value("state").toString();
      const bool required = json.value("ft_recovery_required").toBool(false);
      const auto reason = json.value("ft_recovery_reason").toString();
      const int attempt = json.value("ft_recovery_attempt").toInt(0);
      if (state == "RECOVERING_FT") {
        ft_recovery_label_->setText(
          QString("FT sensor: recovering (attempt %1)").arg(attempt));
      } else if (required) {
        ft_recovery_label_->setText(
          QString("FT sensor: RECOVERY REQUIRED - %1").arg(reason));
      } else {
        ft_recovery_label_->setText("FT sensor: ready");
      }
    });
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
  start_pick_only_client_ = node_->create_client<std_srvs::srv::Trigger>(
    "/pick_place_pipeline/start_pick_only");
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
        const int final_pool = json.contains("minimum_z_candidate_count") ?
          json.value("minimum_z_candidate_count").toInt() :
          json.value("sampled_candidate_count").toInt();
        text += QString("\nCandidates stable/vertical/min-Z pool: %1/%2/%3")
          .arg(json.value("stable_candidate_count").toInt())
          .arg(json.value("vertical_candidate_count").toInt())
          .arg(final_pool);
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
  start_policy_loading_client_ = node_->create_client<std_srvs::srv::Trigger>(
    "/policy_loading/start");
  plan_policy_loading_client_ = node_->create_client<std_srvs::srv::Trigger>(
    "/policy_loading/plan");
  abort_policy_loading_client_ = node_->create_client<std_srvs::srv::Trigger>(
    "/policy_loading/abort");
  reset_policy_loading_client_ = node_->create_client<std_srvs::srv::Trigger>(
    "/policy_loading/reset_pallet");
  continuous_policy_loading_client_ =
    node_->create_client<std_srvs::srv::SetBool>(
    "/policy_loading/set_continuous");
  policy_rearrangement_client_ =
    node_->create_client<std_srvs::srv::SetBool>(
    "/policy_loading/set_rearrangement");
  policy_simulation_client_ =
    node_->create_client<std_srvs::srv::SetBool>(
    "/policy_loading/set_simulation");
  policy_loading_status_sub_ =
    node_->create_subscription<std_msgs::msg::String>(
    "/policy_loading/status", 10,
    [this](const std_msgs::msg::String::SharedPtr msg) {
      const auto json = QJsonDocument::fromJson(
        QByteArray::fromStdString(msg->data)).object();
      QString text = QString("Learned policy loading: %1")
        .arg(json.value("state").toString("UNKNOWN"));
      if (json.contains("continuous_loading_enabled")) {
        const bool enabled =
          json.value("continuous_loading_enabled").toBool(false);
        const QSignalBlocker blocker(continuous_policy_loading_);
        continuous_policy_loading_->setChecked(enabled);
        text += QString("\nContinuous: %1%2")
          .arg(enabled ? "enabled" : "disabled")
          .arg(json.value("continuous_run_active").toBool(false) ?
            " (running)" : "");
      }
      if (json.contains("policy_action_index")) {
        text += QString("\nAction/EMS: %1/%2, value: %3")
          .arg(json.value("policy_action_index").toInt())
          .arg(json.value("policy_ems_index").toInt())
          .arg(json.value("policy_predicted_value").toDouble(), 0, 'f', 4);
      }
      if (json.contains("rearrangement_enabled")) {
        const bool enabled = json.value("rearrangement_enabled").toBool(false);
        const QSignalBlocker blocker(policy_repack_planning_);
        policy_repack_planning_->setChecked(enabled);
      }
      if (json.contains("simulation_enabled")) {
        const bool enabled = json.value("simulation_enabled").toBool(false);
        const QSignalBlocker blocker(policy_simulation_);
        policy_simulation_->setChecked(enabled);
        text += QString("\nSimulation: %1")
          .arg(enabled ? "enabled (robot commands blocked)" : "disabled");
        if (json.contains("simulation_fixed_descent_mm")) {
          text += QString(", fixed descent %1 mm")
            .arg(json.value("simulation_fixed_descent_mm").toDouble(),
              0, 'f', 0);
        }
        if (enabled && json.contains("simulation_sample_count")) {
          text += QString("\nAutomatic item source: cardboard, sampled: %1")
            .arg(json.value("simulation_sample_count").toInt());
        }
      }
      if (json.contains("operation_kind")) {
        text += QString("\nOperation %1/%2: %3 from %4")
          .arg(json.value("operation_number").toInt())
          .arg(json.value("operation_count").toInt())
          .arg(json.value("operation_kind").toString())
          .arg(json.value("operation_source").toString());
      }
      const auto corner = json.value("target_corner_mm").toArray();
      if (corner.size() >= 3) {
        text += QString("\nTarget corner: (%1, %2, %3) mm")
          .arg(corner.at(0).toDouble(), 0, 'f', 0)
          .arg(corner.at(1).toDouble(), 0, 'f', 0)
          .arg(corner.at(2).toDouble(), 0, 'f', 0);
      }
      text += policy_repack_planning_->isChecked() ?
        "\nRepack planning: MCTS + A* enabled" :
        "\nRepack planning: disabled";
      const auto fault = json.value("fault").toString();
      const auto result = json.value("last_result").toString();
      if (!fault.isEmpty()) {
        text += QString("\n%1").arg(fault);
      } else if (!result.isEmpty()) {
        text += QString("\n%1").arg(result);
      }
      policy_loading_state_label_->setText(text);
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
  staging_store_selection_pub_ = node_->create_publisher<std_msgs::msg::Int32>(
    "/staging_slots/select_store", 10);
  staging_retrieve_selection_pub_ = node_->create_publisher<std_msgs::msg::Int32>(
    "/staging_slots/select_retrieve", 10);
  staging_store_client_ = node_->create_client<std_srvs::srv::Trigger>(
    "/staging_slots/store");
  staging_retrieve_client_ = node_->create_client<std_srvs::srv::Trigger>(
    "/staging_slots/retrieve");
  staging_reset_client_ = node_->create_client<std_srvs::srv::Trigger>(
    "/staging_slots/reset");
  staging_status_sub_ = node_->create_subscription<std_msgs::msg::String>(
    "/staging_slots/status", 10,
    [this](const std_msgs::msg::String::SharedPtr msg) {
      const auto json = QJsonDocument::fromJson(
        QByteArray::fromStdString(msg->data)).object();
      QString text = QString("Staging slots: %1")
        .arg(json.value("state").toString("UNKNOWN"));
      const auto slot_array = json.value("slots").toArray();
      QStringList occupied;
      for (const auto & value : slot_array) {
        const auto slot = value.toObject();
        const int index = slot.value("slot").toInt();
        const bool is_occupied = slot.value("occupied").toBool();
        const QString base = staging_store_slot_->itemText(index)
          .section(" [", 0, 0);
        const QString suffix = is_occupied
          ? QString(" [occupied: %1]").arg(slot.value("item_id").toString())
          : QString(" [empty]");
        staging_store_slot_->setItemText(index, base + suffix);
        staging_retrieve_slot_->setItemText(index, base + suffix);
        if (is_occupied) {occupied << QString::number(index);}
      }
      text += QString("\nOccupied: %1").arg(
        occupied.isEmpty() ? "none" : occupied.join(", "));
      const auto fault = json.value("fault").toString();
      const auto result = json.value("last_result").toString();
      if (!fault.isEmpty()) {text += "\n" + fault;}
      else if (!result.isEmpty()) {text += "\n" + result;}
      staging_state_label_->setText(text);
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
  msg.data.push_back(operator_percent / 100.0);  // MoveIt: 0.05 .. 1.00.
  msg.data.push_back(operator_percent);  // Shared envelope: 5 .. 100%.
  motion_speed_config_pub_->publish(msg);
}

void SafeServoPanel::publishRandomLoadingConfig()
{
  if (!random_loading_config_pub_) {return;}
  std_msgs::msg::Float64MultiArray msg;
  msg.data.push_back(static_cast<double>(com_bound_ratio_->value()) / 100.0);
  random_loading_config_pub_->publish(msg);
}

void SafeServoPanel::publishDepthOnlyBounds()
{
  if (!depth_only_bounds_pub_) {return;}
  const int z_min = depth_only_bounds_[0]->value();
  const int z_max = depth_only_bounds_[1]->value();
  const int x_min = depth_only_bounds_[2]->value();
  const int x_max = depth_only_bounds_[3]->value();
  const int y_min = depth_only_bounds_[4]->value();
  const int y_max = depth_only_bounds_[5]->value();
  if (z_min >= z_max || x_min >= x_max || y_min >= y_max) {
    box_estimation_status_label_->setText(
      "Invalid crop: every minimum must be below its maximum");
    return;
  }
  std_msgs::msg::Float64MultiArray message;
  message.data = {
    z_min / 1000.0, z_max / 1000.0,
    x_min / 1000.0, x_max / 1000.0,
    y_min / 1000.0, y_max / 1000.0,
    depth_only_height_offset_->value() / 1000.0};
  depth_only_bounds_pub_->publish(message);
}

void SafeServoPanel::publishObjectInfoConfig()
{
  if (!object_info_config_pub_) {return;}
  std_msgs::msg::Float64MultiArray message;
  message.data = {contact_reference_z_->value() / 1000.0};
  object_info_config_pub_->publish(message);
}

void SafeServoPanel::estimateObjectInfo()
{
  if (!estimate_object_info_client_ ||
    !estimate_object_info_client_->service_is_ready())
  {
    object_info_state_label_->setText("Object-info estimator unavailable");
    return;
  }
  publishConfig();
  publishMotionSpeed();
  publishDepthOnlyBounds();
  publishObjectInfoConfig();
  QTimer::singleShot(100, this, [this]() {
    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    estimate_object_info_client_->async_send_request(request, [this](
      rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
      object_info_state_label_->setText(
        QString::fromStdString(future.get()->message));
    });
  });
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

void SafeServoPanel::clearPickedItem()
{
  if (!clear_picked_item_client_->service_is_ready()) {
    manual_operations_label_->setText("Picked-item clear service unavailable");
    return;
  }
  auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
  clear_picked_item_client_->async_send_request(request, [this](
    rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
    manual_operations_label_->setText(QString::fromStdString(future.get()->message));
  });
}

void SafeServoPanel::recoverFtSensor()
{
  const auto answer = QMessageBox::question(
    this,
    "Recover force/torque sensor",
    "Confirm that the gripper holds no item and the tool is not touching "
    "anything. The sensor will be disabled, enabled, and zeroed.",
    QMessageBox::Yes | QMessageBox::No,
    QMessageBox::No);
  if (answer != QMessageBox::Yes) {
    return;
  }
  if (!recover_ft_sensor_client_->service_is_ready()) {
    manual_operations_label_->setText("FT recovery service unavailable");
    return;
  }
  auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
  recover_ft_sensor_client_->async_send_request(request, [this](
    rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
    manual_operations_label_->setText(
      QString::fromStdString(future.get()->message));
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
  const auto client = pick_only_->isChecked()
    ? start_pick_only_client_ : start_pickup_client_;
  if (!client || !client->service_is_ready()) {
    pickup_state_label_->setText("Pickup pipeline unavailable");
    return;
  }
  publishConfig();
  publishMotionSpeed();
  publishObjectInfoConfig();
  applyPalletConfig();
  QTimer::singleShot(100, this, [this, client]() {
    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    client->async_send_request(request, [this](
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

void SafeServoPanel::setDepthOnlyEstimation(bool enabled)
{
  if (!depth_only_estimation_client_ ||
    !depth_only_estimation_client_->service_is_ready())
  {
    box_estimation_status_label_->setText(
      "Estimator mode service unavailable");
    const QSignalBlocker blocker(depth_only_estimation_);
    depth_only_estimation_->setChecked(!enabled);
    return;
  }
  publishDepthOnlyBounds();
  auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
  request->data = enabled;
  depth_only_estimation_client_->async_send_request(request, [this, enabled](
    rclcpp::Client<std_srvs::srv::SetBool>::SharedFuture future) {
    const auto response = future.get();
    if (!response->success) {
      const QSignalBlocker blocker(depth_only_estimation_);
      depth_only_estimation_->setChecked(!enabled);
    }
    box_estimation_status_label_->setText(
      QString::fromStdString(response->message));
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

void SafeServoPanel::startPolicyLoading()
{
  if (!start_policy_loading_client_ ||
    !start_policy_loading_client_->service_is_ready())
  {
    policy_loading_state_label_->setText(
      "Learned policy loading service unavailable");
    return;
  }
  publishConfig();
  publishMotionSpeed();
  QTimer::singleShot(100, this, [this]() {
    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    start_policy_loading_client_->async_send_request(request, [this](
      rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
      policy_loading_state_label_->setText(
        QString::fromStdString(future.get()->message));
    });
  });
}

void SafeServoPanel::planPolicyLoading()
{
  if (!plan_policy_loading_client_ ||
    !plan_policy_loading_client_->service_is_ready())
  {
    policy_loading_state_label_->setText(
      "Policy target-planning service unavailable");
    return;
  }
  publishConfig();
  publishMotionSpeed();
  QTimer::singleShot(100, this, [this]() {
    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    plan_policy_loading_client_->async_send_request(request, [this](
      rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
      policy_loading_state_label_->setText(
        QString::fromStdString(future.get()->message));
    });
  });
}

void SafeServoPanel::setContinuousPolicyLoading(bool enabled)
{
  if (!continuous_policy_loading_client_ ||
    !continuous_policy_loading_client_->service_is_ready())
  {
    policy_loading_state_label_->setText(
      "Continuous policy-loading service unavailable");
    const QSignalBlocker blocker(continuous_policy_loading_);
    continuous_policy_loading_->setChecked(!enabled);
    return;
  }
  auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
  request->data = enabled;
  continuous_policy_loading_client_->async_send_request(request, [this](
    rclcpp::Client<std_srvs::srv::SetBool>::SharedFuture future) {
    policy_loading_state_label_->setText(
      QString::fromStdString(future.get()->message));
  });
}

void SafeServoPanel::abortPolicyLoading()
{
  if (!abort_policy_loading_client_ ||
    !abort_policy_loading_client_->service_is_ready())
  {
    policy_loading_state_label_->setText(
      "Learned policy loading abort service unavailable");
    return;
  }
  auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
  abort_policy_loading_client_->async_send_request(request, [this](
    rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
    policy_loading_state_label_->setText(
      QString::fromStdString(future.get()->message));
  });
}

void SafeServoPanel::resetPolicyLoading()
{
  const auto answer = QMessageBox::question(
    this,
    "Reset learned policy loading",
    "Clear the policy virtual pallet and Three.js scene?\n\n"
    "Only continue when the physical pallet is empty.",
    QMessageBox::Yes | QMessageBox::No,
    QMessageBox::No);
  if (answer != QMessageBox::Yes) {
    return;
  }
  if (!reset_policy_loading_client_ ||
    !reset_policy_loading_client_->service_is_ready())
  {
    policy_loading_state_label_->setText(
      "Learned policy loading reset service unavailable");
    return;
  }
  auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
  reset_policy_loading_client_->async_send_request(request, [this](
    rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
    policy_loading_state_label_->setText(
      QString::fromStdString(future.get()->message));
  });
}

void SafeServoPanel::storeInStagingSlot()
{
  if (!staging_store_client_ || !staging_store_client_->service_is_ready()) {
    staging_state_label_->setText("Staging store service unavailable");
    return;
  }
  std_msgs::msg::Int32 selection;
  selection.data = staging_store_slot_->currentIndex();
  staging_store_selection_pub_->publish(selection);
  publishConfig();
  publishMotionSpeed();
  QTimer::singleShot(100, this, [this]() {
    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    staging_store_client_->async_send_request(request, [this](
      rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
      staging_state_label_->setText(QString::fromStdString(future.get()->message));
    });
  });
}

void SafeServoPanel::retrieveFromStagingSlot()
{
  if (!staging_retrieve_client_ || !staging_retrieve_client_->service_is_ready()) {
    staging_state_label_->setText("Staging retrieval service unavailable");
    return;
  }
  std_msgs::msg::Int32 selection;
  selection.data = staging_retrieve_slot_->currentIndex();
  staging_retrieve_selection_pub_->publish(selection);
  publishConfig();
  publishMotionSpeed();
  QTimer::singleShot(100, this, [this]() {
    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    staging_retrieve_client_->async_send_request(request, [this](
      rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
      staging_state_label_->setText(QString::fromStdString(future.get()->message));
    });
  });
}

void SafeServoPanel::resetStagingSlots()
{
  if (!staging_reset_client_ || !staging_reset_client_->service_is_ready()) {
    staging_state_label_->setText("Staging reset service unavailable");
    return;
  }
  auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
  staging_reset_client_->async_send_request(request, [this](
    rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
    staging_state_label_->setText(QString::fromStdString(future.get()->message));
  });
}

}

PLUGINLIB_EXPORT_CLASS(safe_servo_rviz_panel::SafeServoPanel, rviz_common::Panel)
