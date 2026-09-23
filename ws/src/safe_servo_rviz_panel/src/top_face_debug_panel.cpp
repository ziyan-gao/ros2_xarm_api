#include "safe_servo_rviz_panel/top_face_debug_panel.hpp"
#include <QComboBox>
#include <QLabel>
#include <QPlainTextEdit>
#include <QPushButton>
#include <QVBoxLayout>
#include <QGridLayout>
#include <QJsonArray>
#include <QJsonDocument>
#include <QPointer>
#include <QSignalBlocker>
#include <QTimer>
#include <QResizeEvent>
#include <QPixmap>
#include <pluginlib/class_list_macros.hpp>
#include <rviz_common/display_context.hpp>

namespace safe_servo_rviz_panel
{
TopFaceDebugPanel::TopFaceDebugPanel(QWidget * parent) : rviz_common::Panel(parent)
{
  auto * layout = new QVBoxLayout(this);
  auto * title = new QLabel("Top-face inspection test — READ ONLY", this);
  title->setWordWrap(true);
  layout->addWidget(title);
  auto * help = new QLabel("No robot motion, vacuum command, or inventory update. "
    "Place the target in view before capturing. Select the marker ID below.", this);
  help->setWordWrap(true);
  layout->addWidget(help);
  targets_ = new QComboBox(this);
  targets_->setSizeAdjustPolicy(QComboBox::AdjustToMinimumContentsLengthWithIcon);
  layout->addWidget(targets_);
  auto * buttons = new QGridLayout;
  capture_ = new QPushButton("1. Capture / project", this);
  segment_ = new QPushButton("2. Run SAM once", this);
  save_ = new QPushButton("3. Save debug data", this);
  clear_ = new QPushButton("Clear capture", this);
  buttons->addWidget(capture_, 0, 0);
  buttons->addWidget(segment_, 0, 1);
  buttons->addWidget(save_, 1, 0);
  buttons->addWidget(clear_, 1, 1);
  layout->addLayout(buttons);
  preview_ = new QLabel("No capture", this);
  preview_->setAlignment(Qt::AlignCenter);
  preview_->setMinimumSize(240, 180);
  preview_->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Expanding);
  layout->addWidget(preview_, 1);
  details_ = new QPlainTextEdit(this);
  details_->setReadOnly(true);
  details_->setMaximumHeight(180);
  details_->setPlainText("Waiting for top_face_debug node...");
  layout->addWidget(details_);
  connect(capture_, &QPushButton::clicked, this, [this] { command("capture"); });
  connect(segment_, &QPushButton::clicked, this, [this] { command("segment"); });
  connect(save_, &QPushButton::clicked, this, [this] { command("save"); });
  connect(clear_, &QPushButton::clicked, this, [this] { command("clear"); });
  connect(targets_, qOverload<int>(&QComboBox::currentIndexChanged), this,
    [this](int) { updateControls(); });
  auto * timer = new QTimer(this);
  connect(timer, &QTimer::timeout, this, [this] {
    updateControls();
    if (received_.isValid() && received_.elapsed() > 2500) {
      details_->setPlainText("Debugger disconnected/stale. No new command is enabled.");
    }
  });
  timer->start(250);
  updateControls();
}

void TopFaceDebugPanel::onInitialize()
{
  auto abstraction = getDisplayContext()->getRosNodeAbstraction().lock();
  if (!abstraction) { return; }
  node_ = abstraction->get_raw_node();
  commands_ = node_->create_publisher<std_msgs::msg::String>("/top_face_debug/command", 10);
  QPointer<TopFaceDebugPanel> guard(this);
  status_sub_ = node_->create_subscription<std_msgs::msg::String>("/top_face_debug/status", 10,
    [guard](const std_msgs::msg::String::ConstSharedPtr msg) {
      const auto doc = QJsonDocument::fromJson(QByteArray::fromStdString(msg->data));
      if (!guard || !doc.isObject()) { return; }
      const auto object = doc.object();
      QMetaObject::invokeMethod(guard.data(), [guard, object] {
        if (guard) { guard->status(object); }
      }, Qt::QueuedConnection);
    });
  preview_sub_ = node_->create_subscription<sensor_msgs::msg::Image>("/top_face_debug/preview", 2,
    [guard](const sensor_msgs::msg::Image::ConstSharedPtr msg) {
      if (!guard || msg->encoding != "bgr8" || msg->width == 0 || msg->height == 0 ||
        msg->step < 3ULL*msg->width || msg->data.size() < size_t(msg->step)*msg->height) { return; }
      // Copy before leaving the ROS callback; the message buffer is temporary.
      const QImage image = QImage(msg->data.data(), msg->width, msg->height,
        msg->step, QImage::Format_RGB888).rgbSwapped().copy();
      QMetaObject::invokeMethod(guard.data(), [guard, image] {
        if (guard) { guard->image_ = image; guard->showImage(); }
      }, Qt::QueuedConnection);
    });
}

void TopFaceDebugPanel::command(const QString & action)
{
  if (!commands_ || !received_.isValid() || received_.elapsed() > 2500) { return; }
  QJsonObject object;
  object["action"] = action;
  object["target"] = targets_->currentText();
  std_msgs::msg::String msg;
  msg.data = QJsonDocument(object).toJson(QJsonDocument::Compact).toStdString();
  if (action == "clear" || action == "capture") {
    image_ = QImage();
    preview_->setText("Waiting for capture...");
  }
  command_pending_.start();
  commands_->publish(msg);
  updateControls();
}

void TopFaceDebugPanel::status(const QJsonObject & object)
{
  received_.start();
  last_status_ = object;
  const auto selection = targets_->currentText();
  QStringList names;
  for (const auto value : object["targets"].toArray()) { names.append(value.toString()); }
  QStringList old;
  for (int i=0; i<targets_->count(); ++i) { old.append(targets_->itemText(i)); }
  if (names != old) {
    QSignalBlocker blocker(targets_);
    targets_->clear();
    targets_->addItems(names);
    const int index = targets_->findText(selection);
    if (index >= 0) { targets_->setCurrentIndex(index); }
  }
  const QString detail = object["state"].toString() + "\nCaptured: " +
    object["captured_target"].toString() + "\n" + object["message"].toString();
  if (details_->toPlainText() != detail) { details_->setPlainText(detail); }
  if (!object["has_capture"].toBool()) {
    image_ = QImage();
    preview_->setText("No capture");
  }
  updateControls();
}

void TopFaceDebugPanel::updateControls()
{
  const bool online = commands_ && received_.isValid() && received_.elapsed() <= 2500;
  const bool available = online && !last_status_["busy"].toBool() &&
    (!command_pending_.isValid() || command_pending_.elapsed() > 400);
  capture_->setEnabled(available && targets_->count() > 0);
  segment_->setEnabled(available && last_status_["can_segment"].toBool() &&
    targets_->currentText() == last_status_["captured_target"].toString());
  save_->setEnabled(available && last_status_["has_capture"].toBool());
  clear_->setEnabled(online);
}

void TopFaceDebugPanel::showImage()
{
  if (!image_.isNull()) {
    preview_->setPixmap(QPixmap::fromImage(image_).scaled(preview_->size(),
      Qt::KeepAspectRatio, Qt::SmoothTransformation));
  }
}
void TopFaceDebugPanel::resizeEvent(QResizeEvent * event)
{
  rviz_common::Panel::resizeEvent(event);
  showImage();
}
}
PLUGINLIB_EXPORT_CLASS(safe_servo_rviz_panel::TopFaceDebugPanel, rviz_common::Panel)
