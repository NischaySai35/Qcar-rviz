#pragma once

#include <array>
#include <string>

#include <QGridLayout>
#include <QImage>
#include <QLabel>
#include <QProcess>
#include <QPushButton>
#include <QSlider>

#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/int32.hpp>

namespace qcar2_rviz_panels
{

// Interactive controls docked inside RViz itself.  RViz2 has no scriptable
// widgets, so anything clickable has to be a compiled rviz_common::Panel
// plugin like this one.
class QCarControlPanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  explicit QCarControlPanel(QWidget * parent = nullptr);
  ~QCarControlPanel() override;

  void onInitialize() override;
  void save(rviz_common::Config config) const override;
  void load(const rviz_common::Config & config) override;

private Q_SLOTS:
  void toggleCameras();
  void onCameraProcessFinished(int exit_code, QProcess::ExitStatus status);
  void toggleVoice();
  void onVolumeChanged(int value);
  void applyFrame(int tile_index, QImage image);

private:
  enum Tile { kFront = 0, kRear = 1, kLeft = 2, kRight = 3 };

  void applyTheme();
  void refreshCameraButton();
  void refreshVoiceButton();
  void publishVoiceState();
  void layoutCameraTiles();
  void subscribeCamera(Tile tile, const std::string & topic);

  // ---- camera grid (rendered directly, not through rviz_default_plugins,
  // so there is no per-camera dock title bar / help-text footer eating
  // space -- each is just a bare QLabel showing the latest frame)
  QGridLayout * camera_grid_{nullptr};
  std::array<QLabel *, 4> tiles_{};
  std::array<rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr, 4> camera_subs_{};

  QPushButton * cameras_button_{nullptr};
  QLabel * camera_status_{nullptr};
  QProcess * camera_process_{nullptr};
  bool cameras_on_{false};

  QPushButton * voice_button_{nullptr};
  QSlider * volume_slider_{nullptr};
  QLabel * volume_label_{nullptr};
  // Muted by default: matches qcar2_announcer.py's voice_enabled_default
  // parameter, and means the first VOICE ON click is what triggers the
  // "Mapping/Navigation started" greeting -- see the announcer's
  // _try_announce_start().
  bool voice_on_{false};

  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr voice_enabled_pub_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr voice_volume_pub_;

Q_SIGNALS:
  // Camera subscription callbacks run on rviz's ROS executor thread; Qt
  // widgets may only be touched from the GUI thread.  Emitting a signal
  // connected to applyFrame() (auto connection -> queued, since sender and
  // receiver live on different threads) marshals each frame over safely.
  void frameReady(int tile_index, QImage image);
};

}  // namespace qcar2_rviz_panels
