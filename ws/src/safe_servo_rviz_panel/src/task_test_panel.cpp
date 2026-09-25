#include "safe_servo_rviz_panel/task_test_panel.hpp"
#include <QCheckBox>
#include <QLabel>
#include <QPlainTextEdit>
#include <QPushButton>
#include <QSlider>
#include <QVBoxLayout>
#include <QGridLayout>
#include <QJsonDocument>
#include <QPointer>
#include <QSignalBlocker>
#include <QTimer>
#include <QScrollArea>
#include <pluginlib/class_list_macros.hpp>
#include <rviz_common/display_context.hpp>

namespace safe_servo_rviz_panel
{
namespace {
const char * restart_nodes[] = {
  "pickup_supervisor", "motion_coordinator", "pickup_pipeline", "place_pipeline",
  "pick_place_pipeline", "random_stable_loading", "policy_loading", "item_localization",
  "waypoint_store", "visualization_node", "box_marker_detector", "depth_box_refinement",
  "top_face_debug", "staging_slots", "planning_scene_obstacles", "pallet_localization",
  "task_test", "servo_node", "safe_servo", "move_group", "xarm_planner_node",
  "ros2_control_node", "robot_state_publisher", "realsense2_camera_node", "cumotion_action_server"
};
const char * steps[] = {"pack_new", "unpack", "pack_slot", "repack"};
const char * task_actions[] = {"pack_new", "unpack", "pack_slot", "repack", "abort", "reset",
  "start_random", "stop_random"};
const char * mode_names[] = {"two_item_mode", "random_pack_unpack_only", "new_item_sam_enabled"};
}
TaskTestPanel::TaskTestPanel(QWidget * parent) : rviz_common::Panel(parent)
{
  auto * layout = new QVBoxLayout(this);
  auto * help = new QLabel("Independent task processes / shared motion control. "
    "Restart reloads task Python code; it keeps inventory and does not retry motion. "
    "Shared application restart controls are below; unavailable backends are explicitly marked.", this);
  help->setWordWrap(true);
  layout->addWidget(help);
  confirm_ = new QCheckBox("Enable deliberate real-robot test commands", this);
  layout->addWidget(confirm_);
  connect(confirm_, &QCheckBox::toggled, this, [this] {refresh();});
  auto * speed_label = new QLabel(this);
  speed_ = new QSlider(Qt::Horizontal, this);
  speed_->setRange(5, 100);
  bool valid = false;
  const int initial = qEnvironmentVariableIntValue("MOTION_SPEED_DEFAULT_PERCENT", &valid);
  speed_->setValue(valid ? qBound(5, initial, 100) : 96);
  speed_label->setText(QString("Motion speed: %1%").arg(speed_->value()));
  connect(speed_, &QSlider::valueChanged, this, [speed_label](int value) {
    speed_label->setText(QString("Motion speed: %1% (applied on start)").arg(value));
  });
  layout->addWidget(speed_label);
  layout->addWidget(speed_);
  const char * mode_labels[] = {"Two-item floor-only mode", "Random: pack/unpack only", "New item: SAM"};
  for (size_t i=0; i<modes_.size(); ++i) {
    modes_[i] = new QCheckBox(mode_labels[i], this);
    layout->addWidget(modes_[i]);
    connect(modes_[i], &QCheckBox::toggled, this, [this, i](bool value) {setMode(i, value);});
  }
  auto * grid = new QGridLayout;
  const char * labels[] = {"Pack new", "Unpack to slot", "Pack from slot", "Repack",
    "Abort (keep vacuum)", "Reset task state (keep items)", "Start random", "Stop random after this task"};
  for (size_t i=0; i<buttons_.size(); ++i) {
    buttons_[i] = new QPushButton(i<8 ? labels[i] : (i<12 ? "Restart task node" : (i==12 ? "Confirm recovered checkpoint" : "Reset motion coordinator")), this);
    connect(buttons_[i], &QPushButton::clicked, this, [this, i] {request(i);});
    if (i<4) {
      grid->addWidget(buttons_[i], i*2, 0);
      nodes_[i] = new QLabel(QString("test_task_%1: unavailable").arg(steps[i]), this);
      nodes_[i]->setWordWrap(true);
      grid->addWidget(nodes_[i], i*2+1, 0, 1, 2);
    } else if (i<8) {
      grid->addWidget(buttons_[i], 8+(i-4)/2, (i-4)%2);
    } else if (i<12) {
      grid->addWidget(buttons_[i], (i-8)*2, 1);
    } else {
      grid->addWidget(buttons_[i], 10+(i-12), 0, 1, 2);
    }
  }
  buttons_[13]->setToolTip("Clear the prepared motion and reset its coordinator; does not move the robot or open the gripper.");
  layout->addLayout(grid);
  auto * scroll = new QScrollArea(this);
  scroll->setWidgetResizable(true);
  scroll->setMinimumHeight(180);
  scroll->setMaximumHeight(320);
  auto * restart_widget = new QWidget(scroll);
  auto * restart_grid = new QGridLayout(restart_widget);
  for (size_t i=0; i<std::size(restart_nodes); ++i) {
    auto * button = new QPushButton(QString("Restart %1").arg(restart_nodes[i]), restart_widget);
    auto * label = new QLabel("Restart backend unavailable", restart_widget);
    label->setWordWrap(true);
    restart_buttons_.push_back(button);
    restart_labels_.push_back(label);
    restart_grid->addWidget(button, int(i)*2, 0);
    restart_grid->addWidget(label, int(i)*2+1, 0);
    connect(button, &QPushButton::clicked, this, [this, i] {restartNode(i);});
  }
  scroll->setWidget(restart_widget);
  layout->addWidget(scroll);
  reply_ = new QLabel("Waiting for task session", this);
  reply_->setWordWrap(true);
  layout->addWidget(reply_);
  details_ = new QPlainTextEdit(this);
  details_->setReadOnly(true);
  layout->addWidget(details_);
  auto * timer = new QTimer(this);
  connect(timer, &QTimer::timeout, this, [this] {refresh();});
  timer->start(250);
  refresh();
}

void TaskTestPanel::onInitialize()
{
  auto abstraction = getDisplayContext()->getRosNodeAbstraction().lock();
  if (!abstraction) {return;}
  node_ = abstraction->get_raw_node();
  for (size_t i=0; i<clients_.size(); ++i) {
    const auto name = i<8 ? std::string("/pick_place_test/")+task_actions[i] :
      (i<12 ? std::string("/test_tasks/")+steps[i-8]+"/restart" : (i==12 ? "/test_tasks/reconcile" : "/motion_coordinator/reset"));
    clients_[i] = node_->create_client<std_srvs::srv::Trigger>(name);
  }
  for (const auto * name : restart_nodes) {
    restart_clients_.push_back(node_->create_client<std_srvs::srv::Trigger>(
      std::string("/test_nodes/")+name+"/restart"));
  }
  const char * names[] = {"set_two_item_mode", "set_random_pack_unpack_only", "set_new_item_sam"};
  for (size_t i=0; i<mode_clients_.size(); ++i) {
    mode_clients_[i] = node_->create_client<std_srvs::srv::SetBool>(std::string("/pick_place_test/")+names[i]);
  }
  speed_pub_ = node_->create_publisher<std_msgs::msg::Float64MultiArray>("/motion_speed/config", 10);
  QPointer<TaskTestPanel> self(this);
  restart_subscription_ = node_->create_subscription<std_msgs::msg::String>("/test_nodes/status", 50,
    [self](std_msgs::msg::String::SharedPtr msg) {
      const auto object = QJsonDocument::fromJson(QByteArray::fromStdString(msg->data)).object();
      if (self) {QMetaObject::invokeMethod(self, [self, object] {
        if (!self) {return;}
        const auto id = object.value("id").toString();
        self->restart_status_.insert(id, object);
        self->restart_received_[id].restart();
        self->refresh();
      }, Qt::QueuedConnection);}
    });
  subscription_ = node_->create_subscription<std_msgs::msg::String>("/pick_place_test/status", 10,
    [self](std_msgs::msg::String::SharedPtr msg) {
      const auto object = QJsonDocument::fromJson(QByteArray::fromStdString(msg->data)).object();
      if (self) {QMetaObject::invokeMethod(self, [self, object] {
        if (!self) {return;}
        self->status_ = object;
        self->received_.restart();
        self->refresh();
      }, Qt::QueuedConnection);}
    });
}

void TaskTestPanel::refresh()
{
  const bool fresh = received_.isValid() && received_.elapsed()<3000;
  const bool pending = pending_.isValid() && pending_.elapsed()<10000;
  for (size_t i=0; i<restart_buttons_.size(); ++i) {
    const QString id = restart_nodes[i];
    const auto data = restart_status_.value(id).toObject();
    const bool monitor_fresh = restart_received_.contains(id) && restart_received_[id].elapsed()<3000;
    const auto blocked = data.value("blocked").toString();
    restart_buttons_[i]->setEnabled(monitor_fresh && !pending && blocked.isEmpty() &&
      !data.value("restarting").toBool() && i<restart_clients_.size() && restart_clients_[i]->service_is_ready());
    restart_buttons_[i]->setToolTip(monitor_fresh ? blocked : "Restart backend not installed for this node");
    restart_labels_[i]->setText(monitor_fresh ?
      QString("PID %1 | %2 | %3\n%4").arg(data.value("pid").toInt())
        .arg(data.value("restarting").toBool() ? "RESTARTING" : (data.value("alive").toBool() ? "RUNNING" : "EXITED"))
        .arg(data.value("message").toString()).arg(blocked) : "Restart backend unavailable");
  }
  const auto state = status_.value("state").toString();
  const bool random = status_.value("random_active").toBool();
  const bool busy = random || status_.value("reset_in_progress").toBool() || (state!="IDLE" && state!="READY" && state!="FAULT");
  const bool enabled = fresh && !pending && confirm_->isChecked();
  const auto nodes = status_.value("task_nodes").toObject();
  for (size_t i=0; i<4; ++i) {
    const auto data = nodes.value(steps[i]).toObject();
    const bool alive = data.value("alive").toBool() && data.value("fresh").toBool();
    buttons_[i]->setEnabled(enabled && !random && !status_.value("two_item_mode").toBool() &&
      status_.value("allowed").toObject().value(steps[i]).toBool() && alive);
    buttons_[i]->setToolTip(status_.value("blocked").toObject().value(steps[i]).toString());
    buttons_[i+8]->setEnabled(fresh && !pending && !busy && nodes.contains(steps[i]) &&
      status_.value("restart_blocked").toString().isEmpty() && !data.value("restarting").toBool());
    buttons_[i+8]->setToolTip(status_.value("restart_blocked").toString());
    nodes_[i]->setText(QString("test_task_%1 | PID %2 | %3\n%4").arg(steps[i])
      .arg(data.value("pid").toInt()).arg(fresh && alive ? data.value("state").toString() : "OFFLINE")
      .arg(data.value("fault").toString()));
  }
  buttons_[4]->setEnabled(bool(node_)); // Best effort stop even when telemetry is stale.
  buttons_[5]->setEnabled(enabled && !busy);
  buttons_[5]->setToolTip(status_.value("recovery_blocked").toString());
  buttons_[6]->setEnabled(enabled && !busy && status_.value("random_start_allowed").toBool());
  buttons_[7]->setEnabled(fresh && random);
  buttons_[12]->setEnabled(enabled && state=="FAULT" && status_.value("restart_blocked").toString().isEmpty());
  const auto motion_state = status_.value("downstream").toObject().value("motion").toString();
  buttons_[13]->setEnabled(fresh && !pending && !busy && !status_.value("reset_in_progress").toBool() &&
    (motion_state=="PREPARED" || motion_state=="IDLE" || motion_state=="SUCCEEDED" || motion_state=="FAULT"));
  speed_->setEnabled(!busy && !pending);
  for (size_t i=0; i<modes_.size(); ++i) {
    QSignalBlocker blocker(modes_[i]);
    modes_[i]->setChecked(status_.value(mode_names[i]).toBool());
    modes_[i]->setEnabled(fresh && !busy && !pending && (i!=0 || state=="IDLE"));
  }
  if (fresh && status_.value("reset_in_progress").toBool()) {
    reply_->setText(status_.value("reset_message").toString());
  } else if (fresh && state=="FAULT" && !pending &&
      (!reply_received_.isValid() || reply_received_.elapsed()>=5000)) {
    const auto reason = status_.value("recovery_blocked").toString();
    reply_->setText(reason.isEmpty() ? "Task fault: Reset task state keeps item records." :
      "Recovery blocked: " + reason + ". Item records are retained.");
  }
  details_->setPlainText(fresh ? QString::fromUtf8(QJsonDocument(status_).toJson(QJsonDocument::Indented)) :
    "Session status stale/unavailable. New motion and restart disabled; Abort remains available.");
}

void TaskTestPanel::request(size_t index)
{
  auto client = clients_[index];
  if (!client || !client->service_is_ready()) {reply_->setText("Service unavailable; nothing sent"); return;}
  pending_.restart();
  QPointer<TaskTestPanel> self(this);
  auto send = [self, client] {
    if (!self) {return;}
    client->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>(),
      [self](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture result) {
        QString text;
        try {text = QString::fromStdString(result.get()->message);}
        catch (const std::exception & e) {text=e.what();}
        if (self) {QMetaObject::invokeMethod(self, [self, text] {
          if (!self) {return;}
          self->reply_->setText(text); self->reply_received_.restart();
          self->pending_.invalidate();
          self->refresh();
        }, Qt::QueuedConnection);}
      });
  };
  if (index<4 || index==6) {
    std_msgs::msg::Float64MultiArray speed;
    speed.data = {speed_->value()/100., double(speed_->value())};
    speed_pub_->publish(speed);
    QTimer::singleShot(100, this, send);
  } else {send();}
  refresh();
}

void TaskTestPanel::restartNode(size_t index)
{
  if (index>=restart_clients_.size()) {return;}
  auto client = restart_clients_[index];
  if (!client->service_is_ready()) {reply_->setText("Restart service unavailable"); return;}
  pending_.restart();
  QPointer<TaskTestPanel> self(this);
  client->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>(),
    [self](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture result) {
      QString text;
      try {text=QString::fromStdString(result.get()->message);}
      catch (const std::exception & e) {text=e.what();}
      if (self) {QMetaObject::invokeMethod(self, [self, text] {
        if (!self) {return;}
        self->reply_->setText(text); self->reply_received_.restart();
        self->pending_.invalidate(); self->refresh();
      }, Qt::QueuedConnection);}
    });
  refresh();
}

void TaskTestPanel::setMode(size_t index, bool enabled)
{
  auto client = mode_clients_[index];
  if (!client || !client->service_is_ready()) {reply_->setText("Mode service unavailable"); refresh(); return;}
  pending_.restart();
  auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
  request->data = enabled;
  QPointer<TaskTestPanel> self(this);
  client->async_send_request(request, [self](rclcpp::Client<std_srvs::srv::SetBool>::SharedFuture result) {
    QString text;
    try {text=QString::fromStdString(result.get()->message);}
    catch (const std::exception & e) {text=e.what();}
    if (self) {QMetaObject::invokeMethod(self, [self, text] {
      if (!self) {return;}
      self->reply_->setText(text); self->reply_received_.restart(); self->pending_.invalidate(); self->refresh();
    }, Qt::QueuedConnection);}
  });
  refresh();
}
}
PLUGINLIB_EXPORT_CLASS(safe_servo_rviz_panel::TaskTestPanel, rviz_common::Panel)
