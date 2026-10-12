// SAFETY SHIELD node -- see shield_core.hpp for what it decides and why.
//
// Runs during navigation.  On every LiDAR scan (no timer in between) it
// updates the blind-zone memory, asks the core what to do, and publishes
//   /qcar2/shield_cmd     Float32MultiArray [mode, speed m/s, steering rad, emergency]
//                         mode 0 = idle (Nav2 drives), 1 = hold (hard
//                         brake / stay), 2 = escape.  qcar2_hardware.cpp
//                         obeys a non-idle command over everything else
//                         (Nav2, the drive pad) and treats a message older
//                         than 150 ms as idle -- the watchdog every command
//                         relay on this car needs.
//   /qcar2/shield_status  String JSON for the web console (state, text, gap)
//   /qcar2/say_brief      what it is doing, for the speaker (urgent, expires ~2 s)
// It is switched on/off by /qcar2/shield_enable (Bool, latched; the console
// button) and starts ON.
#include <chrono>
#include <cmath>
#include <iomanip>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "qcar2_interfaces/msg/motor_commands.hpp"

#include "shield_core.hpp"

using namespace std::chrono_literals;

namespace
{
double yaw_of(double x, double y, double z, double w)
{
    return std::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
}
}  // namespace

class SafetyShield : public rclcpp::Node
{
public:
    SafetyShield() : Node("qcar2_safety_shield"), tf_buffer_(get_clock()), tf_listener_(tf_buffer_)
    {
        shield::Config c;
        c.v_max = declare_parameter("escape_speed", c.v_max);
        c.margin = declare_parameter("margin", c.margin);
        c.keep_gap = declare_parameter("keep_gap", c.keep_gap);
        c.v_max_fast = declare_parameter("escape_speed_fast", c.v_max_fast);
        enabled_ = declare_parameter("enabled", true);
        speak_ = declare_parameter("speak", true);
        base_frame_ = declare_parameter("base_frame", std::string("base_link"));
        core_ = std::make_unique<shield::Core>(c);

        cmd_pub_ = create_publisher<std_msgs::msg::Float32MultiArray>("/qcar2/shield_cmd", 10);
        status_pub_ = create_publisher<std_msgs::msg::String>(
            "/qcar2/shield_status", rclcpp::QoS(1).transient_local());
        // Urgent status: spoken next, dropped if stale (qcar2_announcer.py).
        say_pub_ = create_publisher<std_msgs::msg::String>("/qcar2/say_brief", 10);

        enable_sub_ = create_subscription<std_msgs::msg::Bool>(
            "/qcar2/shield_enable", rclcpp::QoS(1).transient_local(),
            [this](const std_msgs::msg::Bool & msg) {
                if (msg.data == enabled_)
                    return;
                enabled_ = msg.data;
                core_->reset();
                memory_.clear();
                decision_ = shield::Decision();
                RCLCPP_INFO(get_logger(), "Safety shield %s", enabled_ ? "ON" : "OFF");
                say(enabled_ ? "Safety shield on." : "Safety shield off.", true);
                publish_command();
                publish_status(true);
            });
        scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", rclcpp::SensorDataQoS(),
            std::bind(&SafetyShield::on_scan, this, std::placeholders::_1));
        // Navigation remaps the wheel odometry to /odom, mapping does not:
        // listen to both, only one exists at a time.
        for (const char * topic : {"/odom", "/wheel_imu_odom"})
            odom_subs_.push_back(create_subscription<nav_msgs::msg::Odometry>(
                topic, 20, std::bind(&SafetyShield::on_odom, this, std::placeholders::_1)));
        motor_sub_ = create_subscription<qcar2_interfaces::msg::MotorCommands>(
            "/qcar2_motor_speed_cmd", 20, [this](const qcar2_interfaces::msg::MotorCommands & msg) {
                if (!msg.values.empty())
                    last_steer_ = msg.values[0];
                // Nav2 (via the converter) or the drive pad asking to move:
                // the still-object buffer is smaller then -- keep_distance().
                if (msg.values.size() >= 2 && std::abs(msg.values[1]) > 0.02)
                    nav_moving_at_ = now();
            });

        // Keep-alive for the hardware watchdog between scans, the lost-LiDAR
        // check, the status for the console and the speech gate.
        timer_ = create_wall_timer(20ms, std::bind(&SafetyShield::on_timer, this));
        RCLCPP_INFO(get_logger(), "Safety shield ready (%s): margin %.2f m, escape up to %.2f m/s",
                    enabled_ ? "ON" : "OFF", c.margin, c.v_max);
    }

private:
    void on_odom(const nav_msgs::msg::Odometry & msg)
    {
        const auto & p = msg.pose.pose;
        odom_x_ = p.position.x;
        odom_y_ = p.position.y;
        odom_th_ = yaw_of(p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w);
        odom_v_ = msg.twist.twist.linear.x;
        odom_frame_ = msg.header.frame_id;
        odom_at_ = now();
        have_odom_ = true;
    }

    bool lidar_mount(const std::string & frame)
    {
        if (have_mount_)
            return true;
        try
        {
            const auto tf = tf_buffer_.lookupTransform(base_frame_, frame, tf2::TimePointZero);
            const auto & q = tf.transform.rotation;
            mount_x_ = tf.transform.translation.x;
            mount_y_ = tf.transform.translation.y;
            mount_yaw_ = yaw_of(q.x, q.y, q.z, q.w);
            have_mount_ = true;
            RCLCPP_INFO(get_logger(), "LiDAR at (%.3f, %.3f) yaw %.2f in %s", mount_x_, mount_y_, mount_yaw_,
                        base_frame_.c_str());
        }
        catch (const std::exception &)
        {
            return false;
        }
        return true;
    }

    void on_scan(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        last_scan_at_ = now();
        if (!enabled_)
            return;
        if (!lidar_mount(msg->header.frame_id) || !have_odom_)
            return;
        const auto started = std::chrono::steady_clock::now();
        const double t = last_scan_at_.seconds();

        angles_.resize(msg->ranges.size());
        ranges_.resize(msg->ranges.size());
        for (size_t i = 0; i < msg->ranges.size(); ++i)
        {
            angles_[i] = msg->angle_min + msg->angle_increment * static_cast<double>(i) + mount_yaw_;
            const double r = msg->ranges[i];
            // < 0.10: the car's own mast/cables, never a real obstacle.
            ranges_[i] = (std::isfinite(r) && r >= 0.10 && r <= 8.0) ? r : 0.0;
        }
        int remembered = 0;
        auto points = memory_.update(t, odom_x_, odom_y_, odom_th_, mount_x_, mount_y_, angles_, ranges_,
                                     &remembered);
        // Drop the car's own body (anything well inside its outline).
        const auto & c = core_->config();
        std::vector<shield::Point> obstacles;
        obstacles.reserve(points.size());
        for (const auto & p : points)
            if (!(p.x < c.front - 0.02 && p.x > c.rear + 0.02 && std::abs(p.y) < c.half_width - 0.02))
                obstacles.push_back(p);

        // LIVE LIDAR ONLY (operator, 2026-10-09): no camera/depth, no static
        // map -- the LiDAR is fast and accurate; the rest is for mapping.
        const bool nav_driving = (now() - nav_moving_at_).seconds() < 0.5;
        decision_ = core_->update(t, odom_x_, odom_y_, odom_th_, obstacles, {}, odom_v_, last_steer_, nav_driving);
        remembered_ = remembered;
        compute_ms_ = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - started).count();
        publish_command();
        announce();
        publish_status(false);
    }

    void publish_command()
    {
        std_msgs::msg::Float32MultiArray msg;
        const bool on = enabled_ && decision_.mode != shield::Mode::Idle;
        msg.data = {static_cast<float>(on ? static_cast<int>(decision_.mode) : 0),
                    static_cast<float>(on ? decision_.speed : 0.0),
                    static_cast<float>(on ? decision_.steering : 0.0),
                    static_cast<float>(on && decision_.emergency ? 1.0 : 0.0)};
        cmd_pub_->publish(msg);
    }

    void on_timer()
    {
        // LiDAR gone while the shield holds the wheels: hold still rather
        // than keep driving an escape chosen from a stale picture.
        if (enabled_ && decision_.mode == shield::Mode::Escape && (now() - last_scan_at_).seconds() > 0.3)
        {
            decision_.mode = shield::Mode::Hold;
            decision_.speed = 0.0;
            decision_.reason = "no_lidar";
        }
        publish_command();
        flush_speech();
        publish_status(false);
    }

    // ---- speech --------------------------------------------------------

    std::string sides_text() const
    {
        const auto & d = decision_;
        std::vector<std::string> s;
        if (d.side_front) s.push_back("in front");
        if (d.side_back) s.push_back("behind");
        if (d.side_left) s.push_back("on the left");
        if (d.side_right) s.push_back("on the right");
        if (s.empty())
            return "close";
        std::string out = s[0];
        for (size_t i = 1; i < s.size(); ++i)
            out += (i + 1 == s.size() ? " and " : ", ") + s[i];
        return out;
    }

    std::string event_text() const
    {
        const auto & d = decision_;
        if (d.trapped)
        {
            if (d.side_front && d.side_back && d.side_left && d.side_right)
                return "Blocked on all sides. Cannot move. Please move away from me.";
            if (d.side_front && d.side_back)
                return "Obstacles in front and behind. Cannot move. Please move away from me.";
            return "Blocked " + sides_text() + ". Cannot move. Please move away from me.";
        }
        if (moving())
        {
            std::string dir = d.speed > 0.0 ? "forward" : "back";
            if (d.steering > 0.15)
                dir += " left";
            else if (d.steering < -0.15)
                dir += " right";
            return "Obstacle " + sides_text() + ". Moving " + dir + ".";
        }
        return "";
    }

    // The shield has the wheels AND is actually driving them (it keeps one
    // mode while active; its speed eases through zero at the ends).
    bool moving() const
    {
        return decision_.mode != shield::Mode::Idle && std::abs(decision_.speed) > 0.05;
    }

    // An event worth saying: engaged, trapped, freed, changed direction.
    // Only what is worth hearing (live test 2026-10-09: a line on every
    // direction change queued up behind ~2.5 s of speech each and was heard
    // seconds after the moment -- "obstacle behind" 5 s after it left):
    //   * the FIRST reaction after a quiet spell, and
    //   * being TRAPPED (with "please move away").
    // Lines go out on /qcar2/say_brief: the announcer drops them if they
    // could not be spoken within ~2 s.
    void announce()
    {
        const auto t = now();
        if (decision_.mode == shield::Mode::Idle)
        {
            last_event_ = "";
            return;
        }
        if (decision_.trapped)
        {
            if (last_event_ != "trapped")
            {
                last_event_ = "trapped";
                say(event_text(), false);
            }
            last_active_at_ = t;
            return;
        }
        if (moving() && last_event_.empty() && (t - last_active_at_).seconds() > kQuietBeforeSpeaking)
        {
            // Short (under a second to say); the console shows the detail.
            last_event_ = "moving";
            say("Making room.", false);
        }
        if (moving())
            last_active_at_ = t;
    }

    // Newest message wins and goes out at most every 2.5 s: the speaker
    // queue is first-in-first-out at ~2 s a phrase, so anything faster
    // would be describing what happened seconds ago.
    void say(const std::string & text, bool always)
    {
        if (text.empty() || (!speak_ && !always))
            return;
        pending_say_ = text;
        pending_at_ = now();
        flush_speech();
    }

    void flush_speech()
    {
        if (pending_say_.empty())
            return;
        const auto t = now();
        if ((t - pending_at_).seconds() > 1.5)
        {
            pending_say_.clear();                   // out of date already: never say it late
            return;
        }
        if ((t - last_say_at_).seconds() < 2.5)
            return;
        if (pending_say_ == last_said_ && (t - last_say_at_).seconds() < 6.0)
        {
            pending_say_.clear();
            return;
        }
        std_msgs::msg::String msg;
        msg.data = pending_say_;
        say_pub_->publish(msg);
        last_said_ = pending_say_;
        last_say_at_ = t;
        pending_say_.clear();
    }

    // ---- console -------------------------------------------------------

    void publish_status(bool force)
    {
        const auto t = now();
        std::string state;
        if (!enabled_)
            state = "off";
        else if ((t - last_scan_at_).seconds() > 1.0)
            state = "no_lidar";
        else if (decision_.trapped)
            state = "trapped";
        else if (moving())
            state = "escape";
        else if (decision_.mode != shield::Mode::Idle)
            state = "hold";
        else
            state = "idle";
        if (!force && state == last_state_ && (t - last_status_at_).seconds() < 0.2)
            return;
        last_state_ = state;
        last_status_at_ = t;

        std::string text = event_text();
        if (state == "hold")
            text = "Braking -- obstacle " + sides_text() + ".";
        std::ostringstream js;
        js << std::fixed << std::setprecision(3) << "{\"enabled\":" << (enabled_ ? "true" : "false")
           << ",\"state\":\"" << state << "\",\"text\":\"" << text << "\",\"gap\":"
           << std::min(decision_.min_gap, 9.9) << ",\"approach\":" << decision_.max_approach
           << ",\"speed\":" << decision_.speed << ",\"steer\":" << decision_.steering
           << ",\"front\":" << decision_.side_front << ",\"back\":" << decision_.side_back
           << ",\"left\":" << decision_.side_left << ",\"right\":" << decision_.side_right
           << ",\"remembered\":" << remembered_ << ",\"ms\":" << compute_ms_ << "}";
        std_msgs::msg::String msg;
        msg.data = js.str();
        status_pub_->publish(msg);
    }

    std::unique_ptr<shield::Core> core_;
    shield::ScanMemory memory_;
    shield::Decision decision_;
    bool enabled_ = true, speak_ = true;
    std::string base_frame_;

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    bool have_mount_ = false;
    double mount_x_ = 0.0, mount_y_ = 0.0, mount_yaw_ = 0.0;

    bool have_odom_ = false;
    double odom_x_ = 0.0, odom_y_ = 0.0, odom_th_ = 0.0, odom_v_ = 0.0;
    std::string odom_frame_;
    rclcpp::Time odom_at_{0, 0, RCL_ROS_TIME};
    double last_steer_ = 0.0;
    rclcpp::Time nav_moving_at_{0, 0, RCL_ROS_TIME};

    std::vector<double> angles_, ranges_;
    rclcpp::Time last_scan_at_{0, 0, RCL_ROS_TIME};
    int remembered_ = 0;
    double compute_ms_ = 0.0;

    static constexpr double kQuietBeforeSpeaking = 5.0;   // s without shield motion before it speaks again
    rclcpp::Time last_active_at_{0, 0, RCL_ROS_TIME};
    std::string last_event_;
    std::string pending_say_, last_said_;
    rclcpp::Time pending_at_{0, 0, RCL_ROS_TIME};
    rclcpp::Time last_say_at_{0, 0, RCL_ROS_TIME};
    std::string last_state_;
    rclcpp::Time last_status_at_{0, 0, RCL_ROS_TIME};

    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr cmd_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_, say_pub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr enable_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    std::vector<rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr> odom_subs_;
    rclcpp::Subscription<qcar2_interfaces::msg::MotorCommands>::SharedPtr motor_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SafetyShield>());
    rclcpp::shutdown();
    return 0;
}
