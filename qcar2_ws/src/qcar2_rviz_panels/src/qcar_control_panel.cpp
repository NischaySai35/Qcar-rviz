#include "qcar_control_panel.hpp"

#include <cstring>

#include <QApplication>
#include <QFrame>
#include <QHBoxLayout>
#include <QStyle>
#include <QVBoxLayout>

#include <pluginlib/class_list_macros.hpp>
#include <rviz_common/display_context.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction_iface.hpp>

namespace qcar2_rviz_panels
{

namespace
{
// One stylesheet for the whole RViz window.  RViz's own widgets are plain
// Qt, so this is enough to lift it out of the stock grey look without
// touching any RViz code.  The 3D render view is OpenGL and is unaffected.
const char * const kTheme = R"(
QMainWindow, QDockWidget, QWidget { background: #0f1724; color: #e6eef8; font-size: 12px; }
QDockWidget::title { background: #16213a; padding: 5px 8px; font-weight: 600; letter-spacing: 0.5px; }
QToolBar { background: #121c30; border: none; spacing: 4px; padding: 3px; }
QToolButton { background: transparent; border: 1px solid transparent; border-radius: 5px; padding: 4px 8px; }
QToolButton:hover { background: #1d2b4a; border-color: #2c4470; }
QToolButton:checked { background: #1f7a6f; border-color: #37d8c4; }
QMenuBar, QMenu { background: #121c30; }
QMenuBar::item:selected, QMenu::item:selected { background: #1f7a6f; }
QTreeView, QListView, QTableView { background: #0b1220; alternate-background-color: #101a2c; border: 1px solid #22314f; border-radius: 6px; }
QTreeView::item:selected { background: #1f7a6f; }
QHeaderView::section { background: #16213a; border: none; padding: 4px; }
QLineEdit, QComboBox, QSpinBox, QDoubleSpinBox { background: #0b1220; border: 1px solid #2a405f; border-radius: 5px; padding: 3px 6px; selection-background-color: #1f7a6f; }
QPushButton { background: #16213a; border: 1px solid #2a405f; border-radius: 7px; padding: 7px 12px; font-weight: 600; }
QPushButton:hover { background: #1d2b4a; border-color: #37d8c4; }
QPushButton:pressed { background: #1f7a6f; }
QSlider::groove:horizontal { height: 6px; background: #22314f; border-radius: 3px; }
QSlider::sub-page:horizontal { background: #37d8c4; border-radius: 3px; }
QSlider::handle:horizontal { width: 16px; margin: -6px 0; background: #e6eef8; border-radius: 8px; }
QScrollBar:vertical { background: #0b1220; width: 10px; }
QScrollBar::handle:vertical { background: #2a405f; border-radius: 5px; min-height: 24px; }
QScrollBar::add-line, QScrollBar::sub-line { height: 0; }
QStatusBar { background: #121c30; }
QSplitter::handle { background: #16213a; }
QFrame#qcarDivider { background: #22314f; max-height: 1px; }
QLabel#qcarTitle { color: #37d8c4; font-size: 15px; font-weight: 700; letter-spacing: 1px; }
QLabel#qcarSection { color: #a8b9d1; font-size: 10px; font-weight: 700; letter-spacing: 1px; }
QLabel#qcarStatus { color: #a8b9d1; }
QLabel#qcarTile { background: #000000; border: 1px solid #22314f; border-radius: 4px; }
QPushButton[active="true"] { background: #1f7a6f; border-color: #37d8c4; color: white; }
)";

// Topic each tile subscribes to.  All four are the same 5 Hz,
// display-friendly republish from image_preview_throttle.py, BGR8, and all
// published best-effort -- see qcar2_nodes/src/csi.cpp.
const char * const kCameraTopics[4] = {
  "/front/camera/preview", "/rear/camera/preview",
  "/left/camera/preview", "/right/camera/preview",
};

QLabel * sectionLabel(const char * text, QWidget * parent)
{
  auto * label = new QLabel(text, parent);
  label->setObjectName("qcarSection");
  return label;
}

QFrame * divider(QWidget * parent)
{
  auto * line = new QFrame(parent);
  line->setObjectName("qcarDivider");
  line->setFrameShape(QFrame::HLine);
  return line;
}

void setActive(QPushButton * button, bool active)
{
  button->setProperty("active", active);
  button->style()->unpolish(button);
  button->style()->polish(button);
}

QLabel * makeTile(QWidget * parent)
{
  auto * tile = new QLabel(parent);
  tile->setObjectName("qcarTile");
  tile->setMinimumSize(96, 72);
  tile->setScaledContents(true);
  tile->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
  tile->setAlignment(Qt::AlignCenter);
  return tile;
}
}  // namespace

QCarControlPanel::QCarControlPanel(QWidget * parent)
: rviz_common::Panel(parent)
{
  qRegisterMetaType<QImage>("QImage");

  auto * layout = new QVBoxLayout(this);
  layout->setContentsMargins(12, 12, 12, 12);
  layout->setSpacing(8);

  auto * title = new QLabel("QCAR2 CONTROL", this);
  title->setObjectName("qcarTitle");
  layout->addWidget(title);

  // ---- cameras: a 2x2 grid rendered directly by this panel, not through
  // separate rviz_default_plugins/Image displays -- those each carry their
  // own dock title bar and a "Reset / Left-Click: Rotate..." help footer,
  // which is most of what was eating the screen.  No name labels on the
  // tiles themselves either, per the same space constraint: fixed layout is
  // Front top-left, Rear top-right, Left bottom-left, Right bottom-right.
  layout->addWidget(sectionLabel("CAMERAS", this));

  for (int i = 0; i < 4; ++i) {
    tiles_[i] = makeTile(this);
  }
  camera_grid_ = new QGridLayout();
  camera_grid_->setSpacing(3);
  layout->addLayout(camera_grid_);
  connect(this, &QCarControlPanel::frameReady, this, &QCarControlPanel::applyFrame);

  cameras_button_ = new QPushButton(this);
  cameras_button_->setMinimumHeight(36);
  connect(cameras_button_, &QPushButton::clicked, this, &QCarControlPanel::toggleCameras);
  layout->addWidget(cameras_button_);

  camera_status_ = new QLabel("Front camera only", this);
  camera_status_->setObjectName("qcarStatus");
  camera_status_->setWordWrap(true);
  layout->addWidget(camera_status_);

  layout->addWidget(divider(this));

  // ---- voice
  layout->addWidget(sectionLabel("VOICE", this));
  voice_button_ = new QPushButton(this);
  voice_button_->setMinimumHeight(36);
  connect(voice_button_, &QPushButton::clicked, this, &QCarControlPanel::toggleVoice);
  layout->addWidget(voice_button_);

  auto * volume_row = new QHBoxLayout();
  volume_row->setSpacing(8);
  volume_slider_ = new QSlider(Qt::Horizontal, this);
  volume_slider_->setRange(0, 100);
  volume_slider_->setValue(85);
  volume_slider_->setTracking(false);  // publish on release, not every pixel
  connect(volume_slider_, &QSlider::valueChanged, this, &QCarControlPanel::onVolumeChanged);
  connect(volume_slider_, &QSlider::sliderMoved, this, [this](int value) {
      volume_label_->setText(QString("%1 %").arg(value));
    });
  volume_label_ = new QLabel("85 %", this);
  volume_label_->setObjectName("qcarStatus");
  volume_label_->setMinimumWidth(38);
  volume_row->addWidget(volume_slider_, 1);
  volume_row->addWidget(volume_label_);
  layout->addLayout(volume_row);

  layout->addStretch();

  layoutCameraTiles();
  refreshCameraButton();
  refreshVoiceButton();
}

QCarControlPanel::~QCarControlPanel()
{
  if (camera_process_ && camera_process_->state() != QProcess::NotRunning) {
    // SIGTERM to `ros2 launch` makes it SIGINT its children, which is the
    // shutdown path the CSI driver needs to release the camera cleanly.
    camera_process_->terminate();
    camera_process_->waitForFinished(3000);
  }
}

void QCarControlPanel::onInitialize()
{
  applyTheme();

  // RViz's own node, so this panel needs no extra process.
  node_ = getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();

  // Front is always live -- hardware_base.launch.py starts it unconditionally.
  subscribeCamera(kFront, kCameraTopics[kFront]);

  // Voice topics are transient_local: qcar2_announcer.py may start after
  // RViz and must still receive the current setting.
  const auto latched = rclcpp::QoS(1).reliable().transient_local();
  voice_enabled_pub_ = node_->create_publisher<std_msgs::msg::Bool>("/qcar2/voice_enabled", latched);
  voice_volume_pub_ = node_->create_publisher<std_msgs::msg::Int32>("/qcar2/voice_volume", latched);
  publishVoiceState();
}

// Voice on/off and volume survive RViz restarts through the .rviz config.
void QCarControlPanel::save(rviz_common::Config config) const
{
  rviz_common::Panel::save(config);
  config.mapSetValue("voice_on", voice_on_);
  config.mapSetValue("volume", volume_slider_->value());
}

void QCarControlPanel::load(const rviz_common::Config & config)
{
  rviz_common::Panel::load(config);
  bool voice_on = false;
  int volume = 85;
  config.mapGetBool("voice_on", &voice_on);
  config.mapGetInt("volume", &volume);
  voice_on_ = voice_on;
  volume_slider_->setValue(volume);
  volume_label_->setText(QString("%1 %").arg(volume));
  refreshVoiceButton();
  if (voice_enabled_pub_) {
    publishVoiceState();
  }
}

void QCarControlPanel::applyTheme()
{
  if (qApp) {
    qApp->setStyleSheet(kTheme);
  }
}

// ------------------------------------------------------------------ cameras

void QCarControlPanel::refreshCameraButton()
{
  cameras_button_->setText(cameras_on_ ? "HIDE 360° CAMERAS" : "SHOW 360° CAMERAS");
  setActive(cameras_button_, cameras_on_);
}

void QCarControlPanel::subscribeCamera(Tile tile, const std::string & topic)
{
  camera_subs_[tile] = node_->create_subscription<sensor_msgs::msg::Image>(
    topic, rclcpp::SensorDataQoS(),
    [this, tile](sensor_msgs::msg::Image::ConstSharedPtr msg) {
      // Runs on rviz's ROS executor thread.  Only BGR8 is expected --
      // qcar2_nodes/src/csi.cpp always publishes that -- so anything else is
      // dropped rather than mis-rendered.
      if (msg->encoding != "bgr8" || msg->data.empty()) {
        return;
      }
      QImage frame(msg->width, msg->height, QImage::Format_RGB888);
      const uchar * source = msg->data.data();
      for (uint32_t row = 0; row < msg->height; ++row) {
        std::memcpy(frame.scanLine(static_cast<int>(row)),
                    source + static_cast<size_t>(row) * msg->step,
                    static_cast<size_t>(msg->width) * 3);
      }
      // Data above is BGR bytes read as if RGB888; swap channels once here
      // to get true RGB, rather than a Format_BGR888 QImage (added in Qt
      // 5.14 -- this platform ships Qt 5.12).
      Q_EMIT frameReady(static_cast<int>(tile), frame.rgbSwapped());
    });
}

void QCarControlPanel::applyFrame(int tile_index, QImage image)
{
  tiles_[static_cast<size_t>(tile_index)]->setPixmap(QPixmap::fromImage(image));
}

void QCarControlPanel::layoutCameraTiles()
{
  // Reset the grid without deleting the tile widgets (still owned by `this`).
  QLayoutItem * item;
  while ((item = camera_grid_->takeAt(0)) != nullptr) {
    delete item;
  }

  if (cameras_on_) {
    camera_grid_->addWidget(tiles_[kFront], 0, 0);
    camera_grid_->addWidget(tiles_[kRear], 0, 1);
    camera_grid_->addWidget(tiles_[kLeft], 1, 0);
    camera_grid_->addWidget(tiles_[kRight], 1, 1);
    tiles_[kRear]->show();
    tiles_[kLeft]->show();
    tiles_[kRight]->show();
  } else {
    camera_grid_->addWidget(tiles_[kFront], 0, 0, 1, 2);
    tiles_[kRear]->hide();
    tiles_[kLeft]->hide();
    tiles_[kRight]->hide();
  }
}

void QCarControlPanel::toggleCameras()
{
  if (cameras_on_) {
    if (camera_process_ && camera_process_->state() != QProcess::NotRunning) {
      camera_process_->terminate();
    }
    camera_subs_[kRear].reset();
    camera_subs_[kLeft].reset();
    camera_subs_[kRight].reset();
    cameras_on_ = false;
    layoutCameraTiles();
    camera_status_->setText("Front camera only");
    refreshCameraButton();
    return;
  }

  if (!camera_process_) {
    camera_process_ = new QProcess(this);
    camera_process_->setProcessChannelMode(QProcess::ForwardedChannels);
    connect(camera_process_,
      QOverload<int, QProcess::ExitStatus>::of(&QProcess::finished),
      this, &QCarControlPanel::onCameraProcessFinished);
  }
  // RViz was started from the project's ROS environment, so the child
  // inherits it; `ros2 launch` resolves the package from AMENT_PREFIX_PATH.
  camera_process_->start("ros2", {"launch", "qcar2_rviz_gui", "cameras_side.launch.py"});
  subscribeCamera(kRear, kCameraTopics[kRear]);
  subscribeCamera(kLeft, kCameraTopics[kLeft]);
  subscribeCamera(kRight, kCameraTopics[kRight]);
  cameras_on_ = true;
  layoutCameraTiles();
  camera_status_->setText("Starting rear / left / right cameras...\nFirst frames take a few seconds.");
  refreshCameraButton();
}

void QCarControlPanel::onCameraProcessFinished(int exit_code, QProcess::ExitStatus status)
{
  if (cameras_on_) {
    // Died without us asking: surface it instead of leaving frozen frames.
    camera_subs_[kRear].reset();
    camera_subs_[kLeft].reset();
    camera_subs_[kRight].reset();
    cameras_on_ = false;
    layoutCameraTiles();
    camera_status_->setText(
      QString("Side cameras stopped (exit %1%2)")
      .arg(exit_code)
      .arg(status == QProcess::CrashExit ? ", crashed" : ""));
    refreshCameraButton();
  }
}

// -------------------------------------------------------------------- voice

void QCarControlPanel::refreshVoiceButton()
{
  voice_button_->setText(voice_on_ ? "VOICE ON" : "VOICE OFF");
  setActive(voice_button_, voice_on_);
  volume_slider_->setEnabled(voice_on_);
}

void QCarControlPanel::publishVoiceState()
{
  std_msgs::msg::Bool enabled;
  enabled.data = voice_on_;
  voice_enabled_pub_->publish(enabled);
  std_msgs::msg::Int32 volume;
  volume.data = volume_slider_->value();
  voice_volume_pub_->publish(volume);
}

void QCarControlPanel::toggleVoice()
{
  voice_on_ = !voice_on_;
  refreshVoiceButton();
  publishVoiceState();
  Q_EMIT configChanged();
}

void QCarControlPanel::onVolumeChanged(int value)
{
  volume_label_->setText(QString("%1 %").arg(value));
  if (voice_volume_pub_) {
    std_msgs::msg::Int32 volume;
    volume.data = value;
    voice_volume_pub_->publish(volume);
  }
  Q_EMIT configChanged();
}

}  // namespace qcar2_rviz_panels

PLUGINLIB_EXPORT_CLASS(qcar2_rviz_panels::QCarControlPanel, rviz_common::Panel)
