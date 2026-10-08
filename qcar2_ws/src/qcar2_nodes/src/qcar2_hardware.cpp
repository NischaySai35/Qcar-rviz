#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iomanip>
#include <memory>
#include <map>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include "quanser/hil.h"
#include "quanser/quanser_messages.h"
#include "quanser/quanser_types.h"
#include "quanser/quanser_led.h"
#include "sensor_msgs/msg/battery_state.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"

#include "qcar2_interfaces/msg/motor_commands.hpp"
#include "qcar2_interfaces/msg/boolean_leds.hpp"

#define LED_STRIP_SIZE  33

using namespace std::chrono_literals;
using namespace std::placeholders;

class QCar2 : public rclcpp::Node
{
public:
    QCar2()
    : Node("qcar2")
    {
        t_int result;

        other_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        timer_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

        rclcpp::SubscriptionOptions sub_options;
        sub_options.callback_group = other_cb_group_;

        rclcpp::PublisherOptions pub_options;
        pub_options.callback_group = other_cb_group_;

        auto param_desc = rcl_interfaces::msg::ParameterDescriptor{};

        // Declare Gyro parameters
        param_desc.description = "Gyro configuration.";
        param_desc.additional_constraints = "This gyro parameter is for setting the QCar2 onboard gyroscrope parameters.";
        this->declare_parameter("gyro.fs", gyro_fs, param_desc);
        this->declare_parameter("gyro.rate", gyro_rate, param_desc);
        this->declare_parameter("gyro.bw", gyro_bw, param_desc);
        this->declare_parameter("gyro.ord", gyro_ord, param_desc);

        // Declare Accel parameters
        param_desc.description = "Accelerometer configuration.";
        param_desc.additional_constraints = "This accel parameter is for setting the QCar2 onboard accelerometer parameters.";
        this->declare_parameter("accel.fs", accel_fs, param_desc);
        this->declare_parameter("accel.rate", accel_rate, param_desc);
        this->declare_parameter("accel.bw", accel_bw, param_desc);
        this->declare_parameter("accel.ord", accel_ord, param_desc);

        param_desc.description = "IMU temperature bandwidth.";
        param_desc.additional_constraints = "This option sets the temperature sensor's filter bandwidth. Valid values range from 5 to 4000. The units are Hz.";
        this->declare_parameter("temp_bw", temp_bw, param_desc);

        param_desc.description = "Steering bias.";
        param_desc.additional_constraints = "The QCar 2 chassis sometimes has a bias in the steering so that driving the steering output with zero does not produce a zero angle on the wheels i.e., the car may not drive in a straight line when the steering output is set to zero. To adjust for this bias, the steer_bias option may be used to add a small offset to the steering output to eliminate this bias. The value specified should be in radians and may be positive or negative as appropriate. Suitable values are typically between 0.03 and 0.09.";
        this->declare_parameter("steer_bias", steer_bias, param_desc);

        param_desc.description = "Device_Type.";
        param_desc.additional_constraints = "This parameter allows you to switch between the ID of a physical and virtual QCar2";
        this->declare_parameter("device_type", device_type, param_desc);

        param_desc.description = "LED Strip Color";
        param_desc.additional_constraints = "This parameter allows you to set the LEDs for the QCar2";
        this->declare_parameter("led_color_id", 0, param_desc);


        // Parameters initialization
        try
        {
            parameter_cb = this->add_on_set_parameters_callback(std::bind(&QCar2::set_parameters_callback, this, _1));
        }
        catch (const std::bad_alloc& e)
        {
            RCLCPP_ERROR(this->get_logger(), "Error setting up parameters callback. %s", e.what());
            return;
        }

        // Actually get the parameters
        std::string device_type = this->get_parameter("device_type").as_string();
        std::string uri_param;
        std::string LED_uri_param;

        if (device_type.compare("physical")==0){
            uri_param = "0";
            LED_uri_param ="spi://localhost:1?memsize=420,word=8,baud=3333333,lsb=off,frame=1";
        }
        else if (device_type.compare("virtual")==0){
            uri_param = "0@tcpip://localhost:18960";
            LED_uri_param ="tcpip://localhost:18969";
        }
        else {
            RCLCPP_ERROR(this->get_logger(), "Invalid device type, stop node and input either virtual/physical...");
            return;
        }

        RCLCPP_INFO(this->get_logger(),"Current HIL URI for device is: %s", uri_param.c_str() );
        RCLCPP_INFO(this->get_logger(),"Current LED URI for device is: %s", LED_uri_param.c_str() );

        // Load the persisted steering centre-bias BEFORE the 15 ms speed_controller
        // timer starts below -- that timer writes desired_steering (0.0 at this
        // point) to the servo unconditionally from the very first tick, so this
        // load is what makes the wheels come up straight within ~15 ms of launch
        // instead of only after the car has driven far enough to relearn it.
        load_steering_bias();

        // Open the LED strip device
        result =  aaaf5050_mc_k12_open(LED_uri_param.c_str(), LED_STRIP_SIZE, &led_strip);
        if (result < 0)
        {
            msg_get_error_messageA(NULL, result, error_message, sizeof(error_message));
            RCLCPP_ERROR(this->get_logger(), "LED result message is: %s", error_message);
            return;
        }

        // Open the HIL "card"
        result = hil_open("qcar2", uri_param.c_str(), &card);
        if (result < 0)
        {
            msg_get_error_messageA(NULL, result, error_message, sizeof(error_message));
            RCLCPP_ERROR(this->get_logger(), "hil_open error: %s", error_message);
            return;
        }

        // Actually get the parameters
        std::map<std::string, double> params;
        std::map<std::string, double>::iterator it;

        // Get the Gyro params
        if (this->get_parameters({"gyro"}, params))
        {
            // RCLCPP_INFO(this->get_logger(), "Gyro parameters:");
            for (it = params.begin(); it != params.end(); it++)
            {
                // RCLCPP_INFO(this->get_logger(), "%s: %lf", it->first.c_str(), it->second);

                if (it->first.compare("fs") == 0)
                {
                    //RCLCPP_INFO(this->get_logger(), "This param is \"fs\" so assign to the right var");
                    gyro_fs = it->second;
                }
                else if (it->first.compare("rate") == 0)
                {
                    //RCLCPP_INFO(this->get_logger(), "This param is \"rate\" so assign to the right var");
                    gyro_rate = it->second;
                }
                else if (it->first.compare("bw") == 0)
                {
                    //RCLCPP_INFO(this->get_logger(), "This param is \"bw\" so assign to the right var");
                    gyro_bw = it->second;
                }
                else if (it->first.compare("ord") == 0)
                {
                    //RCLCPP_INFO(this->get_logger(), "This param is \"ord\" so assign to the right var");
                    gyro_ord = it->second;
                }
                // Any other gyro.* parameters won't get in here anyways because ROS will not pass
                // non declared parameters in.
            }
        }

        // Get the Accel params
        if (this->get_parameters({"accel"}, params))
        {
            //RCLCPP_INFO(this->get_logger(), "Accel parameters:");
            for (it = params.begin(); it != params.end(); it++)
            {
                //RCLCPP_INFO(this->get_logger(), "%s: %lf", it->first.c_str(), it->second);

                if (it->first.compare("fs") == 0)
                {
                    //RCLCPP_INFO(this->get_logger(), "This param is \"fs\" so assign to the right var");
                    accel_fs = it->second;
                }
                else if (it->first.compare("rate") == 0)
                {
                    //RCLCPP_INFO(this->get_logger(), "This param is \"rate\" so assign to the right var");
                    accel_rate = it->second;
                }
                else if (it->first.compare("bw") == 0)
                {
                    //RCLCPP_INFO(this->get_logger(), "This param is \"bw\" so assign to the right var");
                    accel_bw = it->second;
                }
                else if (it->first.compare("ord") == 0)
                {
                    //RCLCPP_INFO(this->get_logger(), "This param is \"ord\" so assign to the right var");
                    accel_ord = it->second;
                }
                // Any other accel.* parameters won't get in here anyways because ROS will not pass
                // non declared parameters in.
            }
        }

        temp_bw = this->get_parameter("temp_bw").as_double();
        //RCLCPP_INFO(this->get_logger(), "Parameter temp_bw = %lf", temp_bw);

        steer_bias = this->get_parameter("steer_bias").as_double();
        //RCLCPP_INFO(this->get_logger(), "Parameter temp_bw = %lf", temp_bw);

        std::ostringstream bso_stream;
        bso_stream << "gyro_fs=" << gyro_fs << ";"
                   << "gyro_rate=" << gyro_rate << ";"
                   << "gyro_bw=" << gyro_bw << ";"
                   << "gyro_ord=" << gyro_ord << ";"
                   << "accel_fs=" << accel_fs << ";"
                   << "accel_rate=" << accel_rate << ";"
                   << "accel_bw=" << accel_bw << ";"
                   << "accel_ord=" << accel_ord << ";"
                   << "temp_bw=" << temp_bw << ";"
                   << "steer_bias=" << steer_bias << ";"
                   << "enc0_dir=0;enc1_dir=0;enc2_dir=0";

        //RCLCPP_INFO(this->get_logger(), "bso is \"%s\".", bso_stream.str().c_str());
        result = hil_set_card_specific_options(card, bso_stream.str().c_str(), bso_stream.str().length());
        if (result < 0)
        {
            msg_get_error_messageA(NULL, result, error_message, sizeof(error_message));
            RCLCPP_ERROR(this->get_logger(), "hil_set_card_specific_options error: %s", error_message);
            return;
        }

        result = hil_watchdog_clear(card);
        if (result < 0 && result != -QERR_HIL_WATCHDOG_CLEAR)
        {
            msg_get_error_messageA(NULL, result, error_message, sizeof(error_message));
            RCLCPP_ERROR(this->get_logger(), "hil_watchdog_clear error: %s", error_message);
            return;
        }

        // Create the publishers
        battery_state_publisher_ = this->create_publisher<sensor_msgs::msg::BatteryState>("qcar2_battery", 1, pub_options);
        imu_publisher_ = this->create_publisher<sensor_msgs::msg::Imu>("qcar2_imu", 1, pub_options);
        joint_state_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("qcar2_joint", 1, pub_options);
        // true while the throttle is cut because the wheels are blocked --
        // see drive_throttle().  Latched so a late subscriber sees the state.
        stall_publisher_ = this->create_publisher<std_msgs::msg::Bool>(
            "/qcar2/drive_stalled", rclcpp::QoS(1).transient_local());
        // E-stop: the only stop allowed to skip the smooth setpoint ramp.
        estop_subscriber_ = this->create_subscription<std_msgs::msg::Bool>(
            "/qcar2_estop", rclcpp::QoS(1).transient_local(),
            [this](const std_msgs::msg::Bool &msg) { estop_engaged_ = msg.data; });
        // [front, rear] metres the car can travel before a bumper meets an
        // obstacle, from qcar2_bump_guard.py -- see target_speed().
        clearance_subscriber_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "/qcar2/obstacle_clearance", 10,
            [this](const std_msgs::msg::Float32MultiArray &msg) {
                if (msg.data.size() < 2)
                    return;
                clearance_front_ = msg.data[0];
                clearance_rear_ = msg.data[1];
                clearance_stamp_ns_ = clock_.now().nanoseconds();
            });

        // Create the subscribers
        led_cmd_subscriber_ = this->create_subscription<qcar2_interfaces::msg::BooleanLeds>("qcar2_led_cmd", 1, std::bind(&QCar2::led_command_callback, this, _1), sub_options);
        motor_cmd_subscriber_ = this->create_subscription<qcar2_interfaces::msg::MotorCommands>("qcar2_motor_speed_cmd", 1, std::bind(&QCar2::motor_command_callback, this, _1), sub_options);

        //RCLCPP_INFO(this->get_logger(), "driver_comm_sample_time.count: %ld", driver_comm_sample_time.count());
        timer_ = this->create_wall_timer(driver_comm_sample_time, std::bind(&QCar2::timer_callback, this), timer_cb_group_);

        //timer for controlling speed
        timer_speed_control_ = this->create_wall_timer(15ms, std::bind(&QCar2::speed_controller, this));

        //timer for controlling speed
        timer_led_callback_ = this->create_wall_timer(500ms, std::bind(&QCar2::led_timer, this));

        node_running = true;

    }

    ~QCar2()
    {
        int result;

        // MUST happen before hil_close().  The motor throttle is a latched
        // hardware output: whatever PWM speed_controller() wrote last stays
        // applied inside the HIL card after the process exits.  This HIL API
        // version has no "final outputs" facility, so closing the card does
        // NOT zero it.  That is why Ctrl+C used to shut down the LiDAR and
        // RViz cleanly while the wheels kept turning -- there was no longer
        // any process left to command a stop.
        stop_motors();
        save_steering_bias();

        result = hil_close(card);

        if (result < 0)
            RCLCPP_ERROR(this->get_logger(), "Closing the card with error: %d", result);

        // desired LED color
        t_led_color color[LED_STRIP_SIZE];

        for(int i = 0; i < LED_STRIP_SIZE; i++)
        {
            color[i] = { 0, 0, 0 };
        }

        // Turn off the LED strip when we shut the node down
        aaaf5050_mc_k12_write(led_strip, color, LED_STRIP_SIZE);

        result = aaaf5050_mc_k12_close(led_strip);
        if (result < 0)
            RCLCPP_ERROR(this->get_logger(), "Closing LED strip with error: %i", result);

        node_running = false;
        RCLCPP_INFO(this->get_logger(), "qcar2 exit");
    }

public:

    // Write a hard zero to the steering and throttle channels.  Safe to call
    // more than once and safe to call from a shutdown handler: it only
    // touches the HIL card, no ROS interfaces.  Kept public so main() can
    // invoke it from rclcpp::on_shutdown() the moment Ctrl+C is pressed,
    // rather than waiting for the node to be torn down.
    void stop_motors()
    {
        if (motors_stopped_)
            return;

        // Latch the setpoints at zero too, so a speed_controller() tick that
        // is already in flight on another executor thread cannot re-apply
        // the previous throttle after this write.
        desired_speed = 0;
        desired_steering = 0;
        setpoint_ = 0;
        motor_speed_cmd = 0;
        motors_stopped_ = true;

        t_uint32 channels[2] = {1000, 11000};   // steering_angle, motor_throttle
        t_double buffer[2] = {0.0, 0.0};

        t_error result = hil_write_other(card, channels, 2, buffer);
        if (result < 0)
        {
            msg_get_error_messageA(NULL, result, error_message, sizeof(error_message));
            RCLCPP_ERROR(this->get_logger(), "Stopping motors failed: %s", error_message);
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "Motors commanded to zero.");
        }
    }

    // Public for the same reason as stop_motors() just above: main() calls
    // this from rclcpp::on_shutdown() so the last few seconds of steering
    // correction learned this run are not lost.  Path/load counterpart is
    // private below (steering_bias_path(), load_steering_bias()) since only
    // this node's own constructor needs those.
    void save_steering_bias()
    {
        std::ofstream file(steering_bias_path(), std::ios::trunc);
        if (file)
        {
            file << steering_bias_;
            last_saved_bias_ = steering_bias_;
        }
        else
        {
            RCLCPP_WARN(this->get_logger(),
                "Could not save steering centre-bias to %s", steering_bias_path().c_str());
        }
    }

private:

    void led_timer()
    {
        this->get_parameter("led_color_id",led_color_id);
        if (response != led_color_id)
        {
            LED_Set();
            // RCLCPP_INFO(this->get_logger(),"Setting new LED Value %i",led_color_id);
            response = led_color_id;
        }
    }

    void LED_Set()
    {
        //desired LED color
        t_led_color color[LED_STRIP_SIZE];
        t_led_color color_value  = {0,0,0};

        // color ID selection
        if (led_color_id == 0)
            {color_value = { 255, 0, 0 };}
        if (led_color_id == 1)
            {color_value = { 0, 255, 0 }; }
        if (led_color_id == 2)
            {color_value = { 0, 0, 255 };}
        if (led_color_id == 3)
            {color_value = { 255, 255, 0 };}
        if (led_color_id == 4)
            {color_value = { 0, 255, 255 };}
        if (led_color_id == 5)
            {color_value = { 255, 0, 255 };}

        // { 255, 0, 0 };        /* LED #0: red     */
        // { 0, 255, 0 },        /* LED #1: green   */
        // { 0, 0, 255 },        /* LED #2: blue    */
        // { 255, 255, 0 },      /* LED #3: yellow  */
        // { 0, 255, 255 },      /* LED #4: cyan    */
        // { 255, 0, 255 },      /* LED #5: magenta */

        for(int i = 0; i < LED_STRIP_SIZE; i++)
        {
            color[i] = color_value;
        }

        t_int result;
        auto start_time = clock_.now();
        rclcpp::Duration delta_time = clock_.now()-start_time;
        while ( delta_time.seconds() < 0.2)
        {
            delta_time = clock_.now()-start_time;
            result = aaaf5050_mc_k12_write(led_strip, color, LED_STRIP_SIZE);
            if (result < 0)
            {
                msg_get_error_messageA(NULL, result, error_message, sizeof(error_message));
                RCLCPP_ERROR(this->get_logger(), "Error writing to LED strip error: %s", error_message);
            }
        }
    }

    // Wheel speed MAGNITUDE (m/s) from the motor encoder count over one tick.
    // Resolution is not a concern: 1 m/s is ~118,000 counts/s, so even a
    // 0.05 m/s crawl moves ~88 counts in one 15 ms control period.
    //
    // The encoder does NOT give the direction of travel.  It was assumed to
    // be a signed quadrature count, but the 2026-10-07 navigation speed log
    // shows it reading POSITIVE while the car reversed (throttle -0.084,
    // "speed" +0.32 m/s).  Every consumer that took its sign from it --
    // this loop, the steering-bias learner and wheel_imu_odometry.py --
    // counted reversing as driving forward.  On the first goal (forward)
    // that was harmless; on the next one Hybrid-A* chose reverse, the
    // odometry ran forward while the LiDAR saw the car go back, AMCL kept
    // dragging map->odom to reconcile them ("the whole map shifts"), and
    // the goal failed.  Direction now comes from update_travel_direction().
    void update_encoder_speed(double dt)
    {
        const t_int32 count = motor_encoder_count_.load();
        if (dt > 0.0 && have_encoder_count_)
        {
            const double counts_per_second = (count - previous_encoder_count_) / dt;
            encoder_speed_ = std::abs((counts_per_second/(720.0*4.0))*((13.0*19.0)/(70.0*30.0))*(2.0*M_PI)*0.033);
        }
        previous_encoder_count_ = count;
        have_encoder_count_ = true;
    }

    // +1.0 forward, -1.0 reverse, from what the MOTOR IS BEING DRIVEN to do.
    // A DC motor cannot change direction without passing through rest, so:
    // while the wheels turn, hold the direction; only when they are (nearly)
    // stopped take it from the sign of the throttle now being applied.  The
    // brake drag (reverse throttle while still rolling forward) therefore
    // cannot flip it early.  Published to /qcar2_joint for the odometry.
    void update_travel_direction(double speed_magnitude)
    {
        if (speed_magnitude < kStallSpeed && std::abs(motor_speed_cmd) >= kBreakawayPwm)
            travel_direction_ = (motor_speed_cmd > 0.0) ? 1.0 : -1.0;
    }

    // Persisted steering centre-bias.  This car's steering linkage is
    // physically tilted (damaged), so a commanded angle of 0 does NOT drive
    // straight -- confirmed by the operator, who can feel it pull left every
    // time.  There is no steering-angle sensor on this car (checked: HIL
    // channel 1000 is write-only, and the read set has no feedback for it),
    // so the true wheel angle can only ever be INFERRED, from the gyro yaw
    // rate actually achieved versus the commanded angle while driving.  A
    // single file under $HOME/.qcar2/ persists the last learned value across
    // launches -- without this, every fresh mapping/navigate session started
    // uncorrected again and had to relearn by driving crooked for a while
    // first, which is exactly the "goes slightly left on startup" symptom.
    std::string steering_bias_path() const
    {
        const char * home = std::getenv("HOME");
        return std::string(home ? home : "/tmp") + "/.qcar2_steering_bias";
    }

    void load_steering_bias()
    {
        std::ifstream file(steering_bias_path());
        double value = 0.0;
        if (file >> value && std::isfinite(value))
        {
            steering_bias_ = std::clamp(value, -kSteeringBiasMax, kSteeringBiasMax);
            RCLCPP_INFO(this->get_logger(),
                "Loaded steering centre-bias %.3f rad (%.1f deg) from %s",
                steering_bias_, steering_bias_ * 180.0 / M_PI, steering_bias_path().c_str());
        }
        else
        {
            RCLCPP_INFO(this->get_logger(),
                "No saved steering centre-bias yet (%s) -- starting at 0, will learn while driving.",
                steering_bias_path().c_str());
        }
    }

    // Learn the centre-bias from what the car actually did versus what was
    // asked: d_measured = atan(L * yaw_rate / v).  Deliberately just an
    // offset (a per-side GAIN was tried in the Nav2 command converter and
    // disproved by its own logged data -- see qcar2-nav-tuning-gotchas.md
    // round 9 item 29 -- this car's fault is a constant linkage tilt, not an
    // asymmetric response).  Learns only while genuinely moving forward on an
    // unsaturated command, exactly like the retired converter-side learner.
    void update_steering_bias(double dt, double signed_speed)
    {
        // Wait for the gyro's own zero-offset to be characterised first (see
        // timer_callback()): a few millidegrees/s of uncalibrated gyro bias
        // would otherwise leak straight into the learned steering bias.
        if (!gyro_bias_ready_)
            return;
        // FORWARD ONLY (the same rule as the converter's old learner): the
        // fault is in the linkage so forward data covers reverse too, and
        // reverse is where the speed sign was wrong until 2026-10-07 -- a
        // flipped sign inverts atan(L*w/v) and drags the bias the wrong way.
        if (signed_speed < kMinLearnSpeed)
            return;
        if (std::abs(desired_steering) >= 0.98 * kSteeringStopRad ||
            std::abs(desired_steering) > kMaxLearnSteer)
            return;

        const double yaw_rate = measured_yaw_rate_ - gyro_bias_;
        const double measured = std::atan(kWheelbaseMeters * yaw_rate / signed_speed);
        const double error = measured - (desired_steering + steering_bias_);
        steering_bias_ = std::clamp(
            steering_bias_ + kBiasLearnRate * error * dt, -kSteeringBiasMax, kSteeringBiasMax);

        // Save periodically (not every 15 ms tick) so a fresh launch always
        // starts from a recent value without hammering the disk.
        if (std::abs(steering_bias_ - last_saved_bias_) > 0.002)
        {
            bias_save_countdown_ -= dt;
            if (bias_save_countdown_ <= 0.0)
            {
                save_steering_bias();
                bias_save_countdown_ = kBiasSavePeriodSeconds;
            }
        }
    }

    // SPEED LOOP (rewritten 2026-10-07).
    //
    // The old loop was, in effect, a pure integrator: every 15 ms it added
    // kp*error*0.0047/V to the throttle (the kd term works out to ~0.003*
    // error -- nothing), i.e. ~0.1 throttle per second at a 0.2 m/s error,
    // bounded only by the 0.3 clip.  Throttle could therefore neither rise
    // nor FALL quickly: it wound up whenever the wheels were held (rug edge,
    // glass, a bump) and launched the car when they came free, overshot on
    // every start, and when the setpoint ramp or the obstacle-clearance cap
    // asked it to slow down it kept driving for ~1 s -- into the wall
    // (2026-10-07, twice in one day after the round-14 guards).  Every guard
    // added since (stall ceiling, overspeed cut, resume, breakaway hand-off)
    // patched a symptom of that one structure.
    //
    // Now:  throttle = feed-forward(setpoint) + bounded trim + P term
    //   * FEED-FORWARD cruise_pwm(): the throttle that holds a speed on the
    //     flat, learned from steady driving (cruise_gain_).  The throttle
    //     follows the setpoint ramp directly, so slowing the setpoint slows
    //     the car at once.
    //   * TRIM: a small integrator (+-kTrimMax) for floor/battery variation.
    //     It only integrates while the wheels turn, so it cannot wind up.
    //   * P term: soft when below target, firmer above (a gentle brake drag,
    //     never more than kMaxBrakePwm reverse -- no wheel-locking jolt).
    //   * BREAKAWAY: from rest, throttle starts just below the learned
    //     breakaway value and climbs at kBoostRate until the wheels turn; on
    //     the first moving tick it drops straight to feed-forward.  Pinned at
    //     kStallPwmCeiling for kStallCutSeconds = BLOCKED (latched, published
    //     on /qcar2/drive_stalled for the bump guard).
    //   * HARD CEILING ON THE COMMAND ITSELF.  Every speed limit upstream
    //     (explorer 0.20 m/s, MPPI 0.45) only limited the REQUEST; the old
    //     loop could still WRITE up to 0.3 throttle and only noticed the
    //     speed afterwards.  Now the throttle written to the motor can never
    //     exceed cruise_pwm(target) + kTrimMax (+ kPwmHeadroom), never
    //     kPwmMax, and never kStallPwmCeiling while starting -- a fast
    //     command is not produced in the first place.
    //   * Backup only: measured speed above target + kOverspeedMargin
    //     (a sudden floor change) drops to a brake drag.
    //   * TRACTION: wheel speed rising faster than kMaxWheelAccel is spin;
    //     throttle is cut back that tick and trim pulled down.
    // Returns the signed throttle.
    double drive_throttle(double dt, double measured_speed)
    {
        const double direction = (setpoint_ > 0.0) ? 1.0 : -1.0;
        const double target = std::abs(setpoint_);
        const double speed = std::abs(measured_speed);
        const bool same_direction = (measured_speed * setpoint_) > 0.0;
        // Motion from the raw encoder as well as the filtered channel: the
        // filtered one lags, and every tick of lag at breakaway throttle is
        // a tick of throttle the rolling car does not need.
        const bool encoder_moving = encoder_speed_ >= kStallSpeed &&
                                    travel_direction_ * setpoint_ > 0.0;
        const bool moving = speed >= kStallSpeed || encoder_moving;
        const bool moving_our_way = (moving && same_direction) || encoder_moving;

        if (dt > 0.0)
        {
            const double accel = (speed - std::abs(previous_measured_speed_)) / dt;
            wheel_accel_ += 0.3 * (accel - wheel_accel_);           // the speed channel is noisy
        }
        previous_measured_speed_ = measured_speed;

        // A reverse command (Nav2's BackUp, or the planner choosing reverse)
        // is exactly how the car gets off what it hit: release the latch.
        if (stall_latched_ && direction != stall_direction_)
            set_stall_latched(false);

        const double ff = cruise_pwm(target);
        double throttle;                     // magnitude in the commanded direction
        if (!moving_our_way)
        {
            // At rest (or still rolling the other way after a reversal):
            // breakaway boost, no trim.
            if (was_moving_ || boost_ == 0.0)
                boost_ = std::max(ff, kStartFraction * breakaway_pwm_);
            boost_ = std::min(boost_ + kBoostRate * dt, kStallPwmCeiling);
            throttle = std::max(boost_, ff);
            trim_ = 0.0;

            if (!moving && throttle >= kStallPwmCeiling - 1e-6)
            {
                stall_pinned_seconds_ += dt;
                if (!stall_latched_ && stall_pinned_seconds_ >= kStallCutSeconds)
                {
                    stall_direction_ = direction;
                    set_stall_latched(true);
                    RCLCPP_WARN(this->get_logger(),
                        "Drive BLOCKED: %.2f throttle for %.1f s with the wheels not turning "
                        "-- throttle cut until the command reverses or stops.",
                        throttle, stall_pinned_seconds_);
                }
            }
            else
            {
                stall_pinned_seconds_ = 0.0;
            }
            was_moving_ = false;
        }
        else
        {
            if (!was_moving_ && boost_ > 0.0)
            {
                breakaway_pwm_ = std::clamp(0.5 * breakaway_pwm_ + 0.5 * boost_,
                                            kBreakawayPwm, kStallPwmCeiling);
            }
            boost_ = 0.0;
            was_moving_ = true;
            stall_pinned_seconds_ = 0.0;

            const double error = target - speed;
            const bool spinning = wheel_accel_ > kMaxWheelAccel;
            if (!spinning)
                trim_ = std::clamp(trim_ + kTrimKi * error * dt, -kTrimMax, kTrimMax);
            else
                trim_ = std::max(trim_ - kTractionTrimCut, -kTrimMax);
            const double p = (error >= 0.0) ? kTrimKpUp * error : kTrimKpDown * error;
            throttle = ff + trim_ + p;
            if (spinning)
                throttle *= 0.85;

            // Learn the feed-forward from steady driving near the target --
            // never from pushing something (speed far below target).
            if (speed > 0.06 && std::abs(error) < 0.08 && std::abs(wheel_accel_) < 0.15 &&
                throttle > kBreakawayPwm)
            {
                const double gain = std::clamp((throttle - kBreakawayPwm) / speed,
                                               kCruiseGainMin, kCruiseGainMax);
                cruise_gain_ += kCruiseLearnRate * (gain - cruise_gain_);
            }

            // HARD OVERSPEED: whatever the loop thinks, the wheels are too fast.
            if (speed > target + kOverspeedMargin)
            {
                throttle = std::min(throttle, 0.0);
                trim_ = std::min(trim_, 0.0);
                if (!overspeed_)
                    RCLCPP_WARN(this->get_logger(),
                        "OVERSPEED: wheels %.2f m/s, target %.2f -- drive throttle cut.", speed, target);
                overspeed_ = true;
            }
            else
            {
                overspeed_ = false;
            }
        }

        // HARD THROTTLE CEILING tied to the requested speed, and the brake floor.
        throttle = std::clamp(throttle, -kMaxBrakePwm, std::min(kPwmMax, ff + kTrimMax + kPwmHeadroom));
        ff_log_ = ff;
        return direction * throttle;
    }

    // Throttle that holds `speed` (m/s) on the flat, from the learned gain.
    double cruise_pwm(double speed) const
    {
        return std::clamp(kBreakawayPwm + cruise_gain_ * speed, kBreakawayPwm, kStallPwmCeiling);
    }

    // One CSV row per control tick: ~/.ros/qcar2_speed_logs/speed_<time>.csv.
    // Without this the controller's behaviour cannot be reconstructed after a
    // run (the 2026-10-07 crash left no evidence at all).
    void log_speed(const rclcpp::Time & now, double target, double measured_speed)
    {
        if (!speed_log_.is_open())
        {
            if (speed_log_failed_)
                return;
            const char * home = std::getenv("HOME");
            const std::string dir = std::string(home ? home : "/tmp") + "/.ros/qcar2_speed_logs";
            std::error_code ec;
            std::filesystem::create_directories(dir, ec);
            const std::time_t t = std::time(nullptr);
            char stamp[32];
            std::strftime(stamp, sizeof(stamp), "%Y%m%d_%H%M%S", std::localtime(&t));
            const std::string path = dir + "/speed_" + stamp + ".csv";
            speed_log_.open(path);
            if (!speed_log_.is_open())
            {
                speed_log_failed_ = true;
                RCLCPP_WARN(this->get_logger(), "Cannot write speed log %s", path.c_str());
                return;
            }
            RCLCPP_INFO(this->get_logger(), "Speed log: %s", path.c_str());
            speed_log_ << "t,desired,target,setpoint,measured,throttle,ff,trim,boost,cruise_gain,"
                          "breakaway,clear_front,clear_rear,stalled,estop,battery\n";
        }
        speed_log_ << std::fixed << std::setprecision(4)
                   << now.seconds() << ',' << desired_speed << ',' << target << ',' << setpoint_ << ','
                   << measured_speed << ',' << motor_speed_cmd << ',' << ff_log_ << ',' << trim_ << ','
                   << boost_ << ',' << cruise_gain_ << ',' << breakaway_pwm_ << ','
                   << clearance_front_.load() << ',' << clearance_rear_.load() << ','
                   << stall_latched_ << ',' << estop_engaged_.load() << ',' << battery_voltage << '\n';
        if ((now - last_log_flush_).seconds() > 1.0)
        {
            speed_log_.flush();
            last_log_flush_ = now;
        }
    }

    void set_stall_latched(bool latched)
    {
        stall_latched_ = latched;
        stall_pinned_seconds_ = 0.0;
        if (latched)
            motor_speed_cmd = 0.0;
        last_stall_publish_ = rclcpp::Time(0, 0, RCL_SYSTEM_TIME);   // publish now
    }

    void publish_stall_state(const rclcpp::Time & now)
    {
        if ((now - last_stall_publish_).seconds() < 0.2)
            return;
        last_stall_publish_ = now;
        std_msgs::msg::Bool msg;
        msg.data = stall_latched_;
        stall_publisher_->publish(msg);
    }

    // SMOOTH SPEED SETPOINT.  Every source (Nav2 through the converter, the
    // console drive pad, the converter's watchdog zeros) used to set the
    // target speed directly, so every stop was a step to zero throttle and
    // every start a step up.  On this floor's slippery tiles those steps broke
    // traction -- the wheels spun on take-off and skidded on stops -- and
    // each slip went straight into the wheel odometry that Cartographer uses
    // as its motion guess, which smeared the map.  The momentary watchdog
    // zeros (MPPI running late) were also felt as stutters.
    //
    // Now the loop tracks setpoint_, which follows the command at no more
    // than kSetpointAccel / kSetpointDecel.  At the 0.20 m/s mapping speed a
    // full stop takes ~0.3 s and ~3 cm.  Only the EMERGENCY STOP bypasses the
    // ramp.
    //
    // OBSTACLE CLEARANCE.  qcar2_bump_guard.py publishes how far the car can
    // go, along the arc it is steering, before its front / rear bumper meets
    // anything the LiDAR or the RealSense depth can see.  The allowed speed
    // toward it is kClearanceGain * (clearance - kClearanceMargin), so the
    // car eases down as it approaches and comes to rest kClearanceMargin
    // short, instead of driving on until something has to stop it hard.
    // Applied here because every drive source passes through this node:
    // the console's manual drive pad included.  A stale message (guard not
    // running) is ignored.
    double target_speed(const rclcpp::Time & now) const
    {
        double target = desired_speed;
        if ((now.nanoseconds() - clearance_stamp_ns_.load()) * 1e-9 <= kClearanceTimeout)
        {
            const double front = std::max(0.0, kClearanceGain * (clearance_front_ - kClearanceMargin));
            const double rear = std::max(0.0, kClearanceGain * (clearance_rear_ - kClearanceMargin));
            target = std::clamp(target, -rear, front);
            // The cap shrinks in proportion to the room left, so it would
            // approach the margin forever at an ever-slower creep that the
            // motor deadband cannot hold anyway: finish the stop instead.
            if (std::abs(target) < kSetpointStopSpeed)
                target = 0.0;
        }
        return target;
    }

    void update_setpoint(const rclcpp::Time & now, double dt)
    {
        if (estop_engaged_)
        {
            setpoint_ = 0.0;                 // the one hard stop
            return;
        }
        const double target = target_speed(now);
        double next;
        if (target == 0.0 || target * setpoint_ < 0.0 || std::abs(target) < std::abs(setpoint_))
        {
            // Slowing down (or through zero to reverse): toward target, but
            // never past zero in one go.
            const double goal = (target * setpoint_ < 0.0) ? 0.0 : target;
            const double step = kSetpointDecel * dt;
            next = setpoint_ + std::clamp(goal - setpoint_, -step, step);
        }
        else
        {
            const double step = kSetpointAccel * dt;
            next = setpoint_ + std::clamp(target - setpoint_, -step, step);
        }
        // Finish a stop cleanly rather than creeping in the deadband.
        if (target == 0.0 && std::abs(next) < kSetpointStopSpeed)
            next = 0.0;
        setpoint_ = next;
    }

    void speed_controller()
    {
        // Once the stop has been issued the card must not be written again,
        // or this timer would immediately re-apply a non-zero throttle and
        // undo the shutdown stop.
        if (motors_stopped_)
            return;

        auto start_time = clock_.now();
        // time delta calculation
        rclcpp::Duration delta_time = start_time-end_time_;
        const double dt = std::clamp(delta_time.seconds(), 0.0, 0.1);
        update_setpoint(start_time, dt);

        // Neither speed source carries the direction of travel: channel
        // 14000 ("Motor Speed") is a filtered magnitude, and the encoder
        // count reads positive in reverse too -- see update_encoder_speed().
        // Magnitude comes from the encoder (lightly smoothed; 14000's lag
        // let the loop keep pushing after the car was already at speed --
        // offline sim worst-case peak 0.34 vs 0.24 m/s for a 0.20 target),
        // with 14000 as a fallback should the encoder ever read dead while
        // the car is clearly moving.  The sign is update_travel_direction()'s.
        // Measured every tick so both stay continuous through stops.
        update_encoder_speed(delta_time.seconds());
        const double channel_magnitude =
            std::abs((joint_speed_measured/(720.0*4.0))*((13.0*19.0)/(70.0*30.0))*(2.0*M_PI)*0.033);
        encoder_speed_filtered_ += 0.5 * (encoder_speed_ - encoder_speed_filtered_);
        const double speed_magnitude =
            (channel_magnitude > 0.10 && encoder_speed_filtered_ < 0.02)
                ? channel_magnitude : encoder_speed_filtered_;
        update_travel_direction(speed_magnitude);
        const double measured_speed = speed_magnitude * travel_direction_;
        travel_direction_shared_.store(travel_direction_);

        if (setpoint_ != 0)
        {
            command_is_zero_ = false;
            update_steering_bias(delta_time.seconds(), measured_speed);
            motor_speed_cmd = drive_throttle(dt, measured_speed);
        }
        else
        {
            motor_speed_cmd = 0;
            was_moving_ = false;
            boost_ = 0.0;
            trim_ = 0.0;
            overspeed_ = false;
            stall_pinned_seconds_ = 0.0;
            previous_measured_speed_ = 0.0;
            wheel_accel_ = 0.0;
            if (!command_is_zero_)
            {
                command_is_zero_ = true;
                command_zero_since_ = start_time;
            }
            // A deliberate pause (not the converter's momentary watchdog
            // zeros) is a new intent: allow driving in the blocked direction
            // again.
            if (stall_latched_ &&
                (start_time - command_zero_since_).seconds() > kStallReleaseZeroSeconds)
                set_stall_latched(false);
        }
        if (stall_latched_)
            motor_speed_cmd = 0.0;
        publish_stall_state(start_time);
        log_speed(start_time, target_speed(start_time), measured_speed);

        // motor channel mapping
        std::map<std::string, int> motor_channel_map {{"steering_angle", 1000},
                                                    {"motor_throttle", 11000}};

        size_t motor_commands_size = motor_channel_map.size();

        //values for writting command to HIL device
        t_uint32 *channels = new t_uint32[motor_commands_size];
        t_double *buffer = new t_double[motor_commands_size];
        t_uint32 num_channels = 2;

        channels[0] = 1000;
        channels[1] = 11000;

        // Corrected here, at the single point every steering command from
        // every source (manual drive, Nav2) funnels through on its way to
        // the servo -- see update_steering_bias() above.
        buffer[0] = std::clamp(desired_steering - steering_bias_, -kSteeringStopRad, kSteeringStopRad);
        buffer[1] = motor_speed_cmd;

        t_error result;

        result = hil_write_other(card, channels, num_channels, buffer);
        if (result < 0)
        {
            msg_get_error_messageA(NULL, result, error_message, sizeof(error_message));
            RCLCPP_ERROR(this->get_logger(), "hil_write_other error: %s", error_message);
        }

        delete[] channels;
        delete[] buffer;

        // RCLCPP_INFO(this->get_logger(),"Measured Linear Speed is %f", measured_speed);
        // RCLCPP_INFO(this->get_logger(),"Speed Error is %f", speed_error);
        // RCLCPP_INFO(this->get_logger(),"Command is %f", motor_speed_cmd);
        // RCLCPP_INFO(this->get_logger(),"Time is %f", delta_time.seconds());

        end_time_ = start_time;
    }

    void timer_callback()
    {
        t_error result;
        rclcpp::Time hil_read_time;

        t_uint32 AIChannels[] = { 0,   // analog input 0
                                  1,   // analog inptu 1
                                  2,   // battery voltage
                                  3,   // electronics current
                                  4};  // motor current

        t_double AIBuffer[ARRAY_LENGTH(AIChannels)];

        t_uint32 ENChannels[] = {0,     // motor encoder
                                 1,     // user encoder 0
                                 2};    // user encoder 1

        t_int32 ENBuffer[ARRAY_LENGTH(ENChannels)];

        t_uint32 DIChannels[] = { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, // bi-directional digital input
                                  11,   // user button 0
                                  12,   // user button 1
                                  13,   // user button 2
                                  14};  // over current condition

        t_boolean DIBuffer[ARRAY_LENGTH(DIChannels)];

        t_uint32 OIChannels[] = {3000, 3001, 3002,  // angular velocity (from gyroscope)
                                 4000, 4001, 4002,  // lineaear acceleration (from accelerometer)
                                 10000,             // IMU temperature
                                 14000};            // Motor Speed

        t_double OIBuffer[ARRAY_LENGTH(OIChannels)];

        // Read from QCar2 via HIL APIs
        result = hil_read(card,
                        AIChannels, ARRAY_LENGTH(AIChannels),
                        ENChannels, ARRAY_LENGTH(ENChannels),
                        DIChannels, ARRAY_LENGTH(DIChannels),
                        OIChannels, ARRAY_LENGTH(OIChannels),
                        AIBuffer,
                        ENBuffer,
                        DIBuffer,
                        OIBuffer
                        );
        if (result < 0)
        {
            msg_get_error_messageA(NULL, result, error_message, sizeof(error_message));
            RCLCPP_ERROR(this->get_logger(), "hil_read error: %s", error_message);
            return;
        }

        hil_read_time = this->get_clock()->now();

        /* Battery State message */
        auto battery_state = sensor_msgs::msg::BatteryState();
        battery_state.voltage = AIBuffer[2];
        battery_voltage = AIBuffer[2];
        battery_state.temperature = std::numeric_limits<double>::quiet_NaN();
        battery_state.current = std::numeric_limits<double>::quiet_NaN();
        battery_state.charge = std::numeric_limits<double>::quiet_NaN();
        battery_state.capacity = std::numeric_limits<double>::quiet_NaN();
        battery_state.design_capacity = 7.0;    // ??
        battery_state.percentage = std::numeric_limits<double>::quiet_NaN();
        battery_state.power_supply_status = sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_DISCHARGING;
        battery_state.power_supply_health = sensor_msgs::msg::BatteryState::POWER_SUPPLY_HEALTH_GOOD;
        battery_state.power_supply_technology = sensor_msgs::msg::BatteryState::POWER_SUPPLY_TECHNOLOGY_LIPO;
        battery_state.present = true;

        battery_state.header.stamp = hil_read_time;
        battery_state.header.frame_id = "base_link";
        battery_state_publisher_->publish(battery_state);

        /* Imu message */
        auto imu = sensor_msgs::msg::Imu();
        imu.linear_acceleration.x = OIBuffer[3];
        imu.linear_acceleration.y = OIBuffer[4];
        imu.linear_acceleration.z = OIBuffer[5];
        imu.angular_velocity.x = OIBuffer[0];
        imu.angular_velocity.y = OIBuffer[1];
        imu.angular_velocity.z = OIBuffer[2];
        measured_yaw_rate_ = OIBuffer[2];
        // Zero-offset calibration, gated on the car being stationary so it
        // never runs mid-drive.  joint_speed_measured is raw channel 14000
        // counts/s; 200 counts is ~0.0017 m/s by the conversion in
        // speed_controller(), i.e. still stationary.  Runs at 1 kHz, so 300
        // samples is 0.3 s -- long before anyone could realistically start
        // driving after launch.
        if (!gyro_bias_ready_ && std::abs(joint_speed_measured) < 200.0)
        {
            gyro_bias_sum_ += OIBuffer[2];
            if (++gyro_bias_samples_ >= 300)
            {
                gyro_bias_ = gyro_bias_sum_ / gyro_bias_samples_;
                gyro_bias_ready_ = true;
            }
        }

        imu.header.stamp = hil_read_time;
        imu.header.frame_id = "base_link";
        imu_publisher_->publish(imu);

        /* Joint State message */
        auto joint_state = sensor_msgs::msg::JointState();
        joint_state.position.clear();
        joint_state.position.push_back(ENBuffer[0]);
        joint_state.velocity.clear();
        // SIGNED: channel 14000's magnitude with the direction the motor is
        // being driven in (update_travel_direction()) -- neither 14000 nor
        // the encoder count carries the sign by itself.
        joint_state.velocity.push_back(std::abs(OIBuffer[7]) * travel_direction_shared_.load());
        joint_speed_measured = OIBuffer[7];
        // Signed quadrature count; the speed controller reads it to recover
        // the direction of travel that channel 14000 above cannot supply.
        motor_encoder_count_.store(ENBuffer[0]);
        joint_state.effort.clear();
        joint_state.effort.push_back(AIBuffer[4]);

        joint_state.header.stamp = hil_read_time;
        joint_state.header.frame_id = "base_link";
        joint_state_publisher_->publish(joint_state);
    }

    void led_command_callback(const qcar2_interfaces::msg::BooleanLeds &led_commands)
    {
        size_t led_commands_size = led_commands.led_names.size();
        std::map<std::string, int> led_channel_map {{"left_outside_brake_light",    11},
                                                    {"left_inside_brake_light",     12},
                                                    {"right_inside_brake_light",    13},
                                                    {"right_outside_brake_light",   14},
                                                    {"left_reverse_light",          15},
                                                    {"right_reverse_light",         16},
                                                    {"left_rear_signal",            17},
                                                    {"right_rear_signal",           18},
                                                    {"left_outside_headlight",      19},
                                                    {"left_middle_headlight",       20},
                                                    {"left_inside_headlight",       21},
                                                    {"right_inside_headlight",      22},
                                                    {"right_middle_headlight",      23},
                                                    {"right_outside_headlight",     24},
                                                    {"left_front_signal",           25},
                                                    {"right_front_signal",          26}};
        std::map<std::string, int>::iterator it;

        if (led_commands_size > led_channel_map.size())
        {
            RCLCPP_WARN(this->get_logger(), "In %s - size of BooleanLeds message must be between 0 and %ld, but received size is: %ld...ignore", __FUNCTION__, led_channel_map.size(), led_commands_size);
            return;
        }

        t_uint32 *channels = new t_uint32[led_commands_size];
        t_boolean *buffer = new t_boolean[led_commands_size];
        t_uint32 num_channels = 0;
        t_boolean valid_name = true;

        for (unsigned int i = 0; i < led_commands_size; i++)
        {
            it = led_channel_map.find(led_commands.led_names[i]);
            if (it == led_channel_map.end())
            {
                RCLCPP_WARN(this->get_logger(), "In %s - BooleanLeds message led_name %s is invalid...ignoring", __FUNCTION__, led_commands.led_names[i].c_str());
                valid_name = false;
                break;
            }

            channels[num_channels] = it->second;
            buffer[num_channels] = led_commands.values[i];
            num_channels++;
        }

        if (valid_name)
        {
            t_error result;
            result = hil_write_digital(card, channels, num_channels, buffer);
            if (result < 0)
            {
                msg_get_error_messageA(NULL, result, error_message, sizeof(error_message));
                RCLCPP_ERROR(this->get_logger(), "hil_write_digital error: %s", error_message);
            }
        }

        delete[] channels;
        delete[] buffer;
    }

    void motor_command_callback(const qcar2_interfaces::msg::MotorCommands &motor_commands)
    {
        size_t motor_commands_size = motor_commands.motor_names.size();
        std::map<std::string, int> motor_channel_map {{"steering_angle", 1000},
                                                      {"motor_throttle", 11000}};
        std::map<std::string, int>::iterator it;

        if (motor_commands_size > motor_channel_map.size())
        {
            RCLCPP_WARN(this->get_logger(), "In %s - size of MotorCommands message must be between 0 and %ld, but received size is: %ld...ignore", __FUNCTION__, motor_channel_map.size(), motor_commands_size);
            return;
        }

        for (unsigned int i = 0; i < motor_commands_size; i++)
        {
            it = motor_channel_map.find(motor_commands.motor_names[i]);
            if (it == motor_channel_map.end())
            {
                RCLCPP_WARN(this->get_logger(), "In %s - MotorCommands message command_name %s is invalid...ignoring", __FUNCTION__, motor_commands.motor_names[i].c_str());
                break;
            }

            if (i == 0)
                desired_steering = motor_commands.values[0];

            if (i == 1)
                desired_speed = motor_commands.values[1];
        }
    }

    rcl_interfaces::msg::SetParametersResult set_parameters_callback(const std::vector<rclcpp::Parameter> & parameters)
    {
        rcl_interfaces::msg::SetParametersResult result;

        result.successful = true;

        // Loop through the parameters....can happen if set_parameters_atomically() is called
        for (const auto & parameter : parameters)
        {
            if (parameter.get_name().compare(0, 5, "gyro.") == 0)
            {
                if (node_running)
                {
                    result.successful = false;
                    result.reason = "Cannot change this parameter while node is running.";
                }
            }
            else if (parameter.get_name().compare(0, 6, "accel.") == 0)
            {
                if (node_running)
                {
                    result.successful = false;
                    result.reason = "Cannot change this parameter while node is running.";
                }
            }
            else if (parameter.get_name().compare("temp_bw") == 0)
            {
                if (node_running)
                {
                    result.successful = false;
                    result.reason = "Cannot change this parameter while node is running.";
                }
                else
                {
                    if ((parameter.as_double() < min_temp_bw) || (parameter.as_double() > max_temp_bw))
                    {
                        std::ostringstream error_stream;

                        error_stream << "temp_bw must be between " << min_temp_bw << " and " << max_temp_bw << ".";

                        result.successful = false;
                        result.reason = error_stream.str();
                    }
                }
            }
            else if (parameter.get_name().compare("steer_bias") == 0)
            {
                if (node_running)
                {
                    result.successful = false;
                    result.reason = "Cannot change this parameter while node is running.";
                }
            }
            else if (parameter.get_name().compare("device_type") == 0)
            {
                if (node_running)
                {
                    result.successful = false;
                    result.reason = "This parameter cannot change while node is running.";
                }
            }
            else if (parameter.get_name().compare("led_color_id") == 0)
            {
                if(node_running == false)
                {
                this->get_parameter("led_color_id",led_color_id);
                RCLCPP_INFO(this->get_logger(),"New LED ID is %i", led_color_id);
                LED_Set();
                }
            }
            else
            {
                result.successful = false;
                result.reason = "The parameter is invalid.";
            }
        }

        return result;
    }

    rclcpp::CallbackGroup::SharedPtr other_cb_group_;
    rclcpp::CallbackGroup::SharedPtr timer_cb_group_;

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr timer_led_callback_;

    //LED ID selector
    int led_color_id;
    int response = -1;


    // speed controller parameters
    double desired_rotation_speed = 0;
    double speed_error = 0;
    double kp = 20;
    double kd = 0.1;
    double ki = 0.01;
    double km = 0.0047; // v/rad/s
    double joint_speed_measured = 0.0;
    // Signed motor encoder count, written by timer_callback() at 1 kHz and
    // read by speed_controller() at ~67 Hz on a different executor thread.
    std::atomic<t_int32> motor_encoder_count_{0};
    t_int32 previous_encoder_count_{0};
    bool have_encoder_count_{false};
    double travel_direction_{1.0};
    // Copy for timer_callback()'s /qcar2_joint publish (another thread).
    std::atomic<double> travel_direction_shared_{1.0};
    // Raw, unfiltered speed MAGNITUDE from the encoder count over one control
    // tick -- detects the wheels starting to turn without channel 14000's lag.
    double encoder_speed_{0.0};
    double encoder_speed_filtered_{0.0};

    // Steering centre-bias learner/persistence -- see load_steering_bias(),
    // update_steering_bias() above.
    double steering_bias_{0.0};
    double last_saved_bias_{0.0};
    double bias_save_countdown_{0.0};
    double gyro_bias_{0.0};
    double gyro_bias_sum_{0.0};
    int gyro_bias_samples_{0};
    bool gyro_bias_ready_{false};
    double measured_yaw_rate_{0.0};
    static constexpr double kWheelbaseMeters = 0.25725;   // front/rear axle spacing, from QCar2.urdf
    static constexpr double kSteeringStopRad = 0.5236;    // mechanical stop, from QCar2.urdf hub joint limits
    static constexpr double kSteeringBiasMax = 0.15;      // rad, ~9 deg -- a physical fault, not a wide range
    static constexpr double kBiasLearnRate = 0.5;
    static constexpr double kMinLearnSpeed = 0.12;        // m/s
    static constexpr double kMaxLearnSteer = 0.45;        // rad, below the stop -- a saturated command carries no gain information
    static constexpr double kBiasSavePeriodSeconds = 5.0;

    double battery_voltage = 0;
    double desired_speed= 0;
    double desired_steering = 0;
    double prior_speed_error =0;
    double motor_speed_cmd = 0;
    // Throttle below which the motor does not turn (the "~|0.03|" deadband
    // noted in speed_controller()); the floor for every start.
    static constexpr double kBreakawayPwm = 0.03;
    // SPEED LOOP -- see drive_throttle().
    // breakaway_pwm_: throttle at which the wheels last broke free from rest.
    // cruise_gain_: extra throttle (above kBreakawayPwm) per m/s while
    // cruising steadily.  The defaults are deliberately low guesses (0.2 m/s
    // ~ 0.07 throttle); both are re-learned within seconds of driving.
    double breakaway_pwm_ = 0.05;
    double cruise_gain_ = 0.20;
    double boost_ = 0.0;                  // breakaway throttle while at rest
    double trim_ = 0.0;                   // bounded feed-forward correction
    bool was_moving_ = false;
    bool overspeed_ = false;
    double ff_log_ = 0.0;
    static constexpr double kStartFraction = 0.8;             // fresh start just below breakaway
    static constexpr double kBoostRate = 0.08;                // throttle/s climb while the wheels are held
    static constexpr double kCruiseGainMin = 0.05;
    static constexpr double kCruiseGainMax = 0.5;
    static constexpr double kCruiseLearnRate = 0.01;          // per 15 ms tick (~1.5 s time constant)
    static constexpr double kTrimMax = 0.03;                  // trim authority, throttle
    static constexpr double kTrimKi = 0.15;                   // throttle per (m/s * s)
    static constexpr double kTrimKpUp = 0.10;                 // throttle per m/s below target
    static constexpr double kTrimKpDown = 0.30;               // throttle per m/s above target (brake drag)
    static constexpr double kTractionTrimCut = 0.005;         // per spinning tick
    // HARD THROTTLE CEILING while rolling: cruise_pwm(target) + kTrimMax +
    // kPwmHeadroom, never above kPwmMax.  At the 0.20 m/s mapping speed and
    // the default gain that is 0.10 throttle -- ~0.35 m/s on the flat even if
    // everything else in the loop were wrong.  This is what makes a surge
    // impossible rather than detected.
    static constexpr double kPwmHeadroom = 0.0;
    static constexpr double kPwmMax = 0.12;                   // absolute, any speed
    static constexpr double kStallSpeed = 0.03;               // m/s: below this the wheels are "not turning"
    // Max throttle while the wheels are held -- above what breaks static
    // friction on this floor (~0.06-0.09) but no more.  Was 0.15.
    static constexpr double kStallPwmCeiling = 0.10;
    static constexpr double kStallCutSeconds = 1.5;           // pinned that long = blocked
    static constexpr double kStallReleaseZeroSeconds = 1.0;   // a real stop, not watchdog zeros
    static constexpr double kOverspeedMargin = 0.08;          // m/s: backup cut only, see drive_throttle()
    bool stall_latched_ = false;
    double stall_direction_ = 1.0;
    double stall_pinned_seconds_ = 0.0;
    bool command_is_zero_ = true;
    rclcpp::Time command_zero_since_{0, 0, RCL_SYSTEM_TIME};
    rclcpp::Time last_stall_publish_{0, 0, RCL_SYSTEM_TIME};
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr stall_publisher_;
    static constexpr double kMaxWheelAccel = 1.2;             // m/s^2: faster = tyres spinning
    static constexpr double kMaxBrakePwm = 0.08;              // gentle drag only, no reverse jolt
    double previous_measured_speed_ = 0.0;
    double wheel_accel_ = 0.0;
    // Per-tick CSV -- see log_speed().
    std::ofstream speed_log_;
    bool speed_log_failed_ = false;
    rclcpp::Time last_log_flush_{0, 0, RCL_SYSTEM_TIME};

    // Smooth setpoint + obstacle clearance -- see update_setpoint().
    static constexpr double kSetpointAccel = 0.40;            // m/s^2
    static constexpr double kSetpointDecel = 0.70;            // m/s^2
    static constexpr double kSetpointStopSpeed = 0.02;        // m/s: below this a stop is finished
    static constexpr double kClearanceGain = 1.2;             // 1/s: allowed speed per metre of room
    static constexpr double kClearanceMargin = 0.08;          // m: comes to rest this far short
    static constexpr double kClearanceTimeout = 0.5;          // s: older = guard not running
    double setpoint_ = 0.0;
    std::atomic<bool> estop_engaged_{false};
    std::atomic<double> clearance_front_{100.0};
    std::atomic<double> clearance_rear_{100.0};
    std::atomic<int64_t> clearance_stamp_ns_{0};
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr estop_subscriber_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr clearance_subscriber_;
    // Set once the shutdown stop has been written; blocks any further motor
    // writes so the zero cannot be overwritten during teardown.
    std::atomic<bool> motors_stopped_{false};
    rclcpp::Clock clock_;
    rclcpp::Time end_time_;
    rclcpp::TimerBase::SharedPtr timer_speed_control_;

    t_aaaf5050_mc_k12 led_strip;

    rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr battery_state_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_publisher_;

    rclcpp::Subscription<qcar2_interfaces::msg::BooleanLeds>::SharedPtr led_cmd_subscriber_;
    rclcpp::Subscription<qcar2_interfaces::msg::MotorCommands>::SharedPtr motor_cmd_subscriber_;

    char error_message[512];
    t_card card;

    // parameters change callback
    rclcpp::Node::OnSetParametersCallbackHandle::SharedPtr parameter_cb;

    // Sample time to get data from QCar2
    std::chrono::milliseconds driver_comm_sample_time{1};

    bool node_running = false;

    const t_double min_temp_bw = 5.0;
    const t_double max_temp_bw = 4000.0;

    // parameters
    t_double gyro_fs   = 250.0;
    t_double gyro_rate = 500.0;
    t_double gyro_bw   = 125.0;
    t_double gyro_ord  = 3.0;

    t_double accel_fs   = 16.0;
    t_double accel_rate = 1000.0;
    t_double accel_bw   = 250.0;
    t_double accel_ord  = 3.0;

    t_double temp_bw = 4000;
    t_double steer_bias = 0.05;
    std::string device_type = "physical";
};

int main(int argc, char * argv[])
{
    // Initialize the ROS environment
    rclcpp::init(argc, argv);

    // Instantiate the node
    std::shared_ptr<QCar2> qcars_node = std::make_shared<QCar2>();

    // Stop the wheels the instant Ctrl+C is received, instead of waiting for
    // the executor to unwind and the node to be destroyed.  The throttle is a
    // latched hardware output, so any gap here is a gap in which the car is
    // still driving with nothing controlling it.
    //
    // save_steering_bias() lives here too, NOT only in ~QCar2(): this
    // shared_ptr is also captured by this very lambda, and rclcpp shutdown
    // ordering does not guarantee the destructor runs before the process
    // actually exits (observed directly: ~QCar2()'s own save call did not
    // reliably reach disk in testing). on_shutdown callbacks are the
    // documented, deterministic hook for "do this on Ctrl+C".
    rclcpp::on_shutdown([qcars_node]() {
        qcars_node->stop_motors();
        qcars_node->save_steering_bias();
    });

    // Get a multi-threaded executor
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(qcars_node);

    RCLCPP_INFO(qcars_node->get_logger(), "Starting qcar2 loop...");
    executor.spin();
    RCLCPP_INFO(qcars_node->get_logger(), "qcar2 loop ended.\n");

    // Shutdown and exit
    rclcpp::shutdown();
    return 0;
}