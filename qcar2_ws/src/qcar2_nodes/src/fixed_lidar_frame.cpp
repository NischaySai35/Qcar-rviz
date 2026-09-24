#include <chrono>
#include <functional>
#include <memory>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/static_transform_broadcaster.h"

using namespace std::chrono_literals;

// base_link -> base_scan is a bolted-down sensor mount: the value below never
// changes.  It used to be published as a DYNAMIC transform on a 100 ms timer
// stamped now(), which silently broke Nav2's obstacle layer.  The costmap
// transforms each scan at that scan's OWN timestamp, and a scan captured
// after the most recent 10 Hz broadcast fails with "extrapolation into the
// future", so the observation is dropped -- no marking AND no raytrace
// clearing, which is why stale obstacles also never went away.  Whether a
// given scan landed inside or outside that 100 ms window was a race, hence
// the costmap sometimes tracking the LiDAR, sometimes lagging, sometimes
// never appearing at all.  The RViz/web overlays never showed the problem
// because they look the transform up at Time() ("latest available"), which
// cannot extrapolate.
//
// A static transform is published once to /tf_static (transient_local, so
// late joiners still get it) and is valid for ALL time, past and future, so
// the lookup can never fail for timing reasons again.
class FixedFrameBroadcaster : public rclcpp::Node
{
public:
  FixedFrameBroadcaster()
  : Node("fixed_lidar_frame")
  {
    tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

    geometry_msgs::msg::TransformStamped t;

    t.header.stamp = this->get_clock()->now();
    //t.header.frame_id = "qbot_platform";
    t.header.frame_id = "base_link";
    t.child_frame_id = "base_scan";
    t.transform.translation.x = 0.1;
    t.transform.translation.y = 0.0;
    t.transform.translation.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, -3.14159265359);

    t.transform.rotation.x = q.x();
    t.transform.rotation.y = q.y();
    t.transform.rotation.z = q.z();
    t.transform.rotation.w = q.w();

    tf_broadcaster_->sendTransform(t);
  }

private:
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_broadcaster_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FixedFrameBroadcaster>());
  rclcpp::shutdown();
  return 0;
}