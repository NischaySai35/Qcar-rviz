#include "rclcpp/rclcpp.hpp"

#include <algorithm>
#include <atomic>
#include <cmath>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <limits>
#include <nav_msgs/msg/odometry.hpp>

#include "quanser/quanser_messages.h"
#include "quanser/quanser_memory.h"
#include "std_msgs/msg/header.hpp"
#include "std_msgs/msg/bool.hpp"
#include <chrono>
#include <thread>

#include "action_msgs/msg/goal_status.hpp"
#include "action_msgs/msg/goal_status_array.hpp"
#include "quanser/quanser_hid.h"
#include "qcar2_interfaces/msg/boolean_leds.hpp"
#include "qcar2_interfaces/msg/motor_commands.hpp"


using namespace std::chrono_literals;



class Nav2QCarConverter : public rclcpp::Node
{


    public:
    Nav2QCarConverter()
    : Node("nav2_qcar2_command_converter")
    {
    // Runtime-adjustable steering limit, exposed so the GUI's steering
    // slider can set it live via a parameter client -- there is no
    // topic-based equivalent for steering the way /speed_limit exists for
    // speed.
    //
    // 0.5236 rad (30 deg) is the MECHANICAL stop, straight out of
    // qcar2/urdf/QCar2.urdf: both hub_front*_joint carry
    // limit lower="-0.5236" upper="0.5236".  This was previously 0.60, which
    // is 15% PAST the stop -- every command above 0.5236 simply pinned the
    // linkage and produced no extra angle, while the whole stack sized its
    // turning geometry off 0.60 as though it were achievable.  That single
    // wrong number is why the car "never steers enough": see
    // AckermannConstraints.min_turning_r and wz_max in
    // qcar2_slam_and_nav.yaml, both of which are derived from this value and
    // were both asking for arcs the chassis cannot physically produce.
    max_steering_rad_ = this->declare_parameter("max_steering_rad", kSteeringStopRad);
    param_callback_handle_ = this->add_on_set_parameters_callback(
        std::bind(&Nav2QCarConverter::steering_param_callback, this, std::placeholders::_1));

    // configuring command publisher
    command_publisher_  = this->create_publisher<qcar2_interfaces::msg::MotorCommands>("qcar2_motor_speed_cmd", 1);
    // led_publisher_      = this->create_publisher<qcar2_interfaces::msg::BooleanLeds>("qcar2_led_cmd",10);

    // Measured motion, kept ONLY for the diagnostic /qcar2/steering_report
    // below.  The steering centre-bias correction itself moved to
    // qcar2_hardware.cpp -- it is the single point every steering source
    // (this node's Nav2 traffic, and the web console's manual drive during
    // mapping) funnels through on the way to the servo, whereas this node
    // only ever saw Nav2 commands and left manual driving uncorrected.
    // Learning it there also means it persists to disk and applies from the
    // very first command after a fresh launch, not just after Nav2 has been
    // driving for a while -- see qcar2_hardware.cpp::update_steering_bias().
    steering_report_publisher_ = this->create_publisher<geometry_msgs::msg::Vector3>(
        "/qcar2/steering_report", 10);
    odom_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 10,
        [this](const nav_msgs::msg::Odometry &odom) {
            measured_speed_ = odom.twist.twist.linear.x;
            measured_yaw_rate_ = odom.twist.twist.angular.z;
            have_odom_ = true;
        });

    //configure nav2 subscriber
    nav2_subscriber_ = this->create_subscription<geometry_msgs::msg::Twist>("/cmd_vel_nav",1,std::bind(&Nav2QCarConverter::nav2_command_callback, this, std::placeholders::_1));

    // EMERGENCY STOP.  Latched here rather than by having the GUI publish
    // zeroed motor commands directly: this node already republishes at 50 Hz,
    // so a second publisher on qcar2_motor_speed_cmd would just race with it
    // and the winner would be whichever message arrived last.  Gating it at
    // the single writer makes the stop deterministic.  transient_local so the
    // stop survives a converter restart while the GUI holds it engaged.
    estop_subscriber_ = this->create_subscription<std_msgs::msg::Bool>(
        "/qcar2_estop", rclcpp::QoS(1).transient_local(),
        std::bind(&Nav2QCarConverter::estop_callback, this, std::placeholders::_1));

    // MANUAL OVERRIDE.  The web console's arrow pad is usable in navigation
    // mode too now, for the case the operator needs to grab the wheel out
    // of Nav2's hands immediately (about to back into something, etc).  It
    // is routed through here rather than published straight to
    // qcar2_motor_speed_cmd by the GUI -- same reason as the e-stop above:
    // this node is already the single 50 Hz writer to that topic while Nav2
    // is running, and a second writer would just race it.  x/y reuse
    // Vector3 the same way steering_report_ does: this is NOT a Twist, x is
    // a steering ANGLE in radians and y is the raw throttle value, exactly
    // the pair the GUI already sends for mapping-mode manual drive.
    manual_drive_subscriber_ = this->create_subscription<geometry_msgs::msg::Vector3>(
        "/qcar2/manual_drive_cmd", 10,
        [this](const geometry_msgs::msg::Vector3 &cmd) {
            manual_steering_ = cmd.x;
            manual_speed_ = cmd.y;
            last_manual_command_time_ = this->now();
        });
    // Explicit "button is held" flag, set false the instant the operator
    // releases -- so handoff back to Nav2 is immediate on release rather
    // than waiting out kManualCommandTimeout.  That timeout still exists
    // underneath as a dead-man's switch for a dropped browser tab/socket.
    manual_active_subscriber_ = this->create_subscription<std_msgs::msg::Bool>(
        "/qcar2/manual_drive_active", rclcpp::QoS(1),
        [this](const std_msgs::msg::Bool &msg) { manual_active_ = msg.data; });

    // Is behavior_server's BackUp recovery running right now?  Read from the
    // action's own status topic rather than inferred from the Twist -- see
    // the STEERED BACK-UP block in nav2_command_callback().  Same QoS as rcl's
    // action status publisher (reliable + transient_local), so a goal that
    // was already executing when this node subscribed is still seen.
    backup_status_subscriber_ = this->create_subscription<action_msgs::msg::GoalStatusArray>(
        "/backup/_action/status", rclcpp::QoS(10).reliable().transient_local(),
        [this](const action_msgs::msg::GoalStatusArray &msg) {
            bool active = false;
            for (const auto &goal : msg.status_list) {
                if (goal.status == action_msgs::msg::GoalStatus::STATUS_ACCEPTED ||
                    goal.status == action_msgs::msg::GoalStatus::STATUS_EXECUTING) {
                    active = true;
                }
            }
            backup_active_ = active;
        });

    //publishing timer for converted command
    // Match the QCar speed-control loop closely enough for smooth motion
    // without flooding a depth-one motor command subscription at 1 kHz.
    timer_ = this->create_wall_timer(20ms, std::bind(&Nav2QCarConverter::command_plublish, this));

    //publishing timer for converted command
    timer2_ = this->create_wall_timer(33ms, std::bind(&Nav2QCarConverter::led_publish, this));



    }

    private:       
        void nav2_command_callback(const geometry_msgs::msg::Twist &nav2_commands){
            last_command_time_ = this->now();
            nav2_speed = nav2_commands.linear.x;
            // Nav2's Twist.angular.z is a vehicle yaw rate (rad/s), whereas
            // QCar2 expects a front-wheel steering angle (rad).  Convert via
            // the Ackermann bicycle model rather than sending yaw rate to the
            // steering actuator directly.
            //
            // MUST use abs(nav2_speed) here, not the signed value.  MPPI's
            // AckermannConstraints only bounds |wz| <= |vx| / min_turning_r
            // (magnitude); it does not flip the sign of wz to match a
            // reversing vx the way a true bicycle-model rollout would (its
            // internal propagation is yaw' = yaw + wz*dt regardless of vx's
            // sign -- this is a known upstream Nav2 MPPI/Ackermann gap, see
            // ros-navigation/navigation2 issues #4425, #5714, #5806).  So the
            // wz this node receives while BackUp/reversing already encodes
            // the intended turn direction on its own.  Dividing by a
            // negative v on top of that added a second, unwanted sign flip,
            // which is exactly "reverse steers to the opposite side" -- the
            // wheels turned away from the way the car actually needed to go.
            // Using |v| keeps the steering-angle magnitude physically
            // correct (same formula) while leaving direction entirely up to
            // wz's own sign, forward or reverse alike.
            //
            // The divisor is additionally FLOORED at kSteeringRefSpeed.  This
            // conversion divides by speed, so as the car slows the same wz
            // asks for an ever-larger steering angle, and below roughly
            // 0.1 m/s the division blows up: a near-zero wz merely changing
            // sign swings the wheels lock-to-lock.  That is the violent
            // steering jitter seen whenever the car is crawling -- squeezing
            // past someone who stepped in close, or easing into a goal (the
            // general_goal_checker comment in qcar2_slam_and_nav.yaml
            // describes the same mechanism at the goal).  The floor caps how
            // far the angle can be amplified at crawl speed while leaving
            // the formula exact at any normal driving speed, where
            // abs(nav2_speed) is the larger term and wins the max().
            if (std::abs(nav2_speed) > kMinSpeedForSteering) {
                nav2_steering = std::atan(
                    kWheelbaseMeters * nav2_commands.angular.z /
                    std::max(std::abs(nav2_speed), kSteeringRefSpeed));
                double limit = max_steering_rad_.load();
                nav2_steering = std::clamp(nav2_steering, -limit, limit);
            } else {
                // A car cannot rotate in place; hold the wheels straight when
                // Nav2 commands a near-zero-speed heading correction.
                nav2_steering = 0.0;
            }

            // STEERED BACK-UP.  Nav2's BackUp recovery always reverses dead
            // straight (linear.x < 0 with angular.z exactly 0 -- the
            // controller never produces an exact zero), which is the "it
            // straightens the wheels when reversing" complaint: a straight
            // reverse only undoes the last approach, and the car drives
            // straight back into the same spot.  A driver stuck nose-first
            // reverses on OPPOSITE lock -- the middle of a three-point turn --
            // so the car keeps rotating toward where it was trying to go.
            // Same here, only if it was genuinely turning forward within the
            // last kBackupSteerMemory, and still clamped to the live limit.
            //
            // BackUp is identified by its action status, NOT by
            // "angular.z == 0.0".  The controller never produces an exact
            // zero, but velocity_smoother does: its angular deadband (0.01
            // rad/s) snaps every small wz to exactly 0.0.  So whenever MPPI
            // itself chose a near-straight reverse (a Reeds-Shepp reverse
            // segment, or backing away from someone who stepped in close),
            // the old test mistook it for BackUp and flicked the wheels to
            // full opposite lock mid-manoeuvre.
            const rclcpp::Time now = this->now();
            if (nav2_speed > kMinSpeedForSteering && std::abs(nav2_steering) > 0.15) {
                last_forward_steering_ = nav2_steering;
                last_forward_steering_time_ = now;
            }
            if (nav2_speed < -kMinSpeedForSteering && backup_active_ &&
                last_forward_steering_time_.nanoseconds() != 0 &&
                (now - last_forward_steering_time_) < kBackupSteerMemory)
            {
                double limit = max_steering_rad_.load();
                nav2_steering = std::clamp(-std::copysign(limit, last_forward_steering_),
                                           -limit, limit);
            }
        }


        rcl_interfaces::msg::SetParametersResult steering_param_callback(
            const std::vector<rclcpp::Parameter> &parameters){
            rcl_interfaces::msg::SetParametersResult result;
            result.successful = true;
            for (const auto &parameter : parameters) {
                if (parameter.get_name() == "max_steering_rad") {
                    double value = parameter.as_double();
                    if (value <= 0.0 || value > 1.0) {
                        result.successful = false;
                        result.reason = "max_steering_rad must be in (0, 1.0] radians";
                        continue;
                    }
                    max_steering_rad_.store(value);
                }
            }
            return result;
        }

        void estop_callback(const std_msgs::msg::Bool &estop){
            if (estop.data && !estop_engaged_) {
                RCLCPP_WARN(this->get_logger(),
                    "EMERGENCY STOP engaged -- motion commands are held at zero.");
            } else if (!estop.data && estop_engaged_) {
                RCLCPP_INFO(this->get_logger(), "Emergency stop released.");
            }
            estop_engaged_ = estop.data;
        }

        void command_plublish(){

            // While the stop is engaged, discard whatever Nav2 is asking for
            // and keep publishing zero.  Publishing must continue rather than
            // simply returning, because the QCar2 throttle is a latched
            // hardware output -- going silent would leave the last non-zero
            // command applied.
            if (estop_engaged_) {
                nav2_speed = 0.0;
                nav2_steering = 0.0;
                // Bypass the steering slew limiter: an emergency stop must
                // centre the wheels immediately, not ramp them over 240 ms.
                steering_cmd_ = 0.0;
                publish_motor_command();
                return;
            }

            // MANUAL OVERRIDE (web console arrow pad, usable in navigation
            // mode too, not just mapping).  Guarded by an explicit "active"
            // flag -- false the instant the operator releases the button --
            // plus kManualCommandTimeout as a dead-man's switch in case the
            // browser tab or its WebSocket dies mid-hold.  While active,
            // this just substitutes the manual command for Nav2's for THIS
            // publish only: nav2_command_callback() keeps updating
            // nav2_speed/nav2_steering from the live /cmd_vel_nav stream
            // the whole time regardless (it is not gated on manual_active_,
            // and Nav2's own goal is never cancelled), so the instant the
            // override lapses the very next tick already publishes
            // wherever Nav2's plan currently wants to go -- not a stale
            // pre-override command -- which is what "release the button and
            // it carries on to the goal from wherever it ended up" needs.
            bool manual_fresh = manual_active_ &&
                (this->now() - last_manual_command_time_) <= kManualCommandTimeout;
            if (manual_fresh) {
                nav2_speed = manual_speed_;
                nav2_steering = manual_steering_;
            } else {
                manual_active_ = false;

                // WATCHDOG.  This timer republishes the last received command
                // at 50 Hz, so without an expiry the car keeps executing
                // whatever it was last told forever.  The dangerous case:
                // Nav2's BackUp recovery sends a negative linear.x, finishes,
                // and stops publishing -- the QCar then reverses indefinitely
                // with nobody commanding it.  If /cmd_vel goes quiet, coast
                // to a stop.
                if (last_command_time_.nanoseconds() == 0 ||
                    (this->now() - last_command_time_) > kCommandTimeout)
                {
                    nav2_speed = 0.0;
                    nav2_steering = 0.0;
                }
            }

            publish_motor_command();
            publish_steering_report();
        }

        // There is NO steering-angle sensor on this car.  Confirmed from
        // qcar2_hardware.cpp: HIL channel 1000 (steering_angle) is written
        // only, and the read set is analog 0-4, encoders 0-2, digital 0-14 and
        // "other" 3000-3002 (gyro) / 4000-4002 (accel) / 10000 / 14000 -- no
        // steering feedback anywhere in it.  So "are the wheels actually where
        // they were told to be" can only ever be INFERRED, by running the
        // bicycle model backwards on the yaw rate the car achieved.
        //
        // Publishing that inference next to the command is the whole point:
        // it is the only way to tell "the controller chose not to steer" apart
        // from "the controller steered and the car did not respond", which is
        // the question every steering problem on this car has come down to.
        //   x = angle Nav2 asked for, y = angle actually sent to the servo
        //       (before qcar2_hardware.cpp's own centre-bias correction),
        //   z = angle inferred from the measured yaw rate (NaN below the
        //       speed at which the inference is meaningless).  A gap between
        //       y and z now mostly reflects that downstream correction.
        void publish_steering_report(){
            geometry_msgs::msg::Vector3 report;
            report.x = nav2_steering;
            report.y = steering_cmd_;
            report.z = (have_odom_ && std::abs(measured_speed_) > kMinLearnSpeed)
                ? std::atan(kWheelbaseMeters * measured_yaw_rate_ / measured_speed_)
                : std::numeric_limits<double>::quiet_NaN();
            steering_report_publisher_->publish(report);
        }

        void publish_motor_command(){

            // Clamp to the current limit, then slew-rate limit.  The centre-
            // bias correction happens downstream in qcar2_hardware.cpp, not
            // here -- see the constructor comment.
            double target = std::clamp(nav2_steering, -max_steering_rad_.load(), max_steering_rad_.load());

            const double max_step = kMaxSteeringRateRadPerSec * kControlPeriodSeconds;
            steering_cmd_ += std::clamp(target - steering_cmd_, -max_step, max_step);

            //configure publisher for LEDS and motor commands:
            // Populate motor command  message for velocity and steering
            qcar2_interfaces::msg::MotorCommands motor_command;

            // // Create a string array for names, and double array for values
            std::vector<std::string> name;
            std::vector<t_double> val;

            name.push_back("steering_angle");
            name.push_back("motor_throttle");

            val.push_back(steering_cmd_);
            val.push_back(nav2_speed);

            motor_command.motor_names = name;
            motor_command.values = val;


            this->command_publisher_->publish(motor_command);


        }



        void led_publish(){
            
            
            
            if (nav2_speed !=0) {
                //set LEDs for QCar moving
                led_values[8]= 1;
                led_values[9]= 1;
                led_values[10]= 1;
                led_values[11]= 1;
                led_values[12]= 1;
                led_values[13]= 1;

                if(steering_cmd_ > 0.01){
                    led_values[14]= 1;
                    led_values[6]= 1;

                }
                else if(steering_cmd_ <-0.01){
                    led_values[15]= 1;
                    led_values[7]= 1;

                }

            }
            else if (nav2_speed == 0 ){
                led_values[0] = 1;
                led_values[1] = 1;
                led_values[2] = 1;
                led_values[3] = 1;
            }          
            
            
            // Populate LED commands 
            qcar2_interfaces::msg::BooleanLeds led_commands;

            std::vector<std::string> led_name;
            std::vector<bool> led_value_commands;

            
            
            led_name.push_back("left_outside_brake_light");
            led_name.push_back("left_inside_brake_light");
            led_name.push_back("right_inside_brake_light");    
            led_name.push_back("right_outside_brake_light");   
            led_name.push_back("left_reverse_light");          
            led_name.push_back("right_reverse_light");         
            led_name.push_back("left_rear_signal");            
            led_name.push_back("right_rear_signal");           
            led_name.push_back("left_outside_headlight");      
            led_name.push_back("left_middle_headlight");       
            led_name.push_back("left_inside_headlight");       
            led_name.push_back("right_inside_headlight");      
            led_name.push_back("right_middle_headlight");      
            led_name.push_back("right_outside_headlight");     
            led_name.push_back("left_front_signal");           
            led_name.push_back("right_front_signal");

            for(bool index: led_values)
            {
                led_value_commands.push_back(index);
            }

            led_commands.led_names = led_name;
            led_commands.values = led_value_commands;
            // this->led_publisher_->publish(led_commands);
         }


        bool led_values[16] = {0};

        double nav2_speed = 0;
        // Steering angle Nav2 is asking for this cycle...
        double nav2_steering = 0;
        // ...and the rate-limited angle actually sent to the servo.
        double steering_cmd_ = 0;
        // Manual arrow-pad override (web console) -- see command_plublish().
        double manual_steering_ = 0.0;
        double manual_speed_ = 0.0;
        bool manual_active_ = false;
        rclcpp::Time last_manual_command_time_{0, 0, RCL_ROS_TIME};
        // 2x the GUI's 150 ms manual-drive heartbeat.
        const rclcpp::Duration kManualCommandTimeout{std::chrono::milliseconds(300)};
        bool have_odom_ = false;
        double measured_speed_ = 0.0;
        double measured_yaw_rate_ = 0.0;
        // Front hub joints at x=+0.12960, rear wheel joints at x=-0.12765
        // (qcar2/urdf/QCar2.urdf): 0.25725 m between axles.
        static constexpr double kWheelbaseMeters = 0.25725;
        // Mechanical steering stop, from the same URDF (hub joint limits).
        static constexpr double kSteeringStopRad = 0.5236;
        // MUST stay below velocity_smoother's deadband_velocity (0.03 m/s).
        // Raising it to 0.05 was a mistake and a dangerous one: any genuine
        // crawl command -- a low /speed_limit setting from the GUI slider, or
        // the slow-down on a tight turn -- lands between 0.03 and 0.05, and
        // this branch then forced the wheels dead straight while the car was
        // still driving.  The car ignores the planned curve and ploughs
        // straight ahead.  The steering jitter this was meant to address is
        // handled properly by kMaxSteeringRateRadPerSec below, which limits
        // how fast the wheels may move without ever blanking the command.
        static constexpr double kMinSpeedForSteering = 0.015;
        // Floor for the speed the steering conversion divides by -- see
        // nav2_command_callback().  NOT a minimum drive speed: the car still
        // crawls at whatever vx Nav2 asks for, this only stops the derived
        // steering ANGLE from being amplified without bound as vx -> 0.
        static constexpr double kSteeringRefSpeed = 0.10;   // m/s
        // Hard slew limit on the steering servo.  A sign flip in wz asks for
        // an instantaneous swing across the full range; a real steering rack
        // cannot do that and trying to is what is seen (and heard) as jitter.
        // Raised from 2.5: at that rate a full 0.60 rad correction took 240 ms,
        // which is a long time to be pointing the wrong way when the car is
        // trying to claw its way back onto the path.  4.0 rad/s covers the full
        // range in 150 ms while still ruling out instant lock-to-lock slamming.
        static constexpr double kMaxSteeringRateRadPerSec = 4.0;
        // Below this speed the yaw-rate-based steering inference in
        // publish_steering_report() is too noisy to mean anything.
        static constexpr double kMinLearnSpeed = 0.12;   // m/s
        // Publish period of timer_ below (20 ms), i.e. the slew step size.
        static constexpr double kControlPeriodSeconds = 0.020;
        std::atomic<double> max_steering_rad_{kSteeringStopRad};
        rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;

        // Nav2's controller runs at 20 Hz (50 ms).  Was 150 ms (three cycles),
        // but on this ARM board one MPPI optimisation regularly runs long
        // (899 "missed its desired rate" warnings in one auto-mapping run),
        // and every such hiccup zeroed the motor -- which, before the resume
        // fix in qcar2_hardware.cpp, also threw away the throttle the speed
        // loop had built up, so the car never got moving.  300 ms still stops
        // a car whose commands have genuinely ceased within ~10 cm at cruise.
        rclcpp::Time last_command_time_{0, 0, RCL_ROS_TIME};
        const rclcpp::Duration kCommandTimeout{std::chrono::milliseconds(300)};
        // Steered back-up -- see nav2_command_callback().
        double last_forward_steering_ = 0.0;
        rclcpp::Time last_forward_steering_time_{0, 0, RCL_ROS_TIME};
        const rclcpp::Duration kBackupSteerMemory{std::chrono::seconds(3)};
        bool backup_active_ = false;

        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::TimerBase::SharedPtr timer2_;


        rclcpp::Publisher<qcar2_interfaces::msg::MotorCommands>::SharedPtr command_publisher_;
        rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr steering_report_publisher_;
        rclcpp::Publisher<qcar2_interfaces::msg::BooleanLeds>::SharedPtr led_publisher_;
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr nav2_subscriber_;
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr estop_subscriber_;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber_;
        rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr manual_drive_subscriber_;
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr manual_active_subscriber_;
        rclcpp::Subscription<action_msgs::msg::GoalStatusArray>::SharedPtr backup_status_subscriber_;
        bool estop_engaged_ = false;
        
};


int main(int argc, char ** argv)
{

    // Node creation
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Nav2QCarConverter>());
    rclcpp::shutdown();

    return 0;
}
