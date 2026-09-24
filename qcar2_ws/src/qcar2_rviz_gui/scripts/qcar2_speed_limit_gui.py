#!/usr/bin/python3
# Hardcoded, not `env python3`: see image_preview_throttle.py for why --
# `env python3` can resolve to pyenv's Python 3.7 instead of the system
# 3.8 rclpy's C extension needs, depending on shell PATH state.
"""Runtime navigation control panel: speed/steering limits, e-stop, stop all.

RViz2 has no built-in slider or button widgets (custom panels need a compiled
C++ plugin), so this is a small standalone window instead -- same pattern as
qcar2_drive_gui.py.

Controls:

* Speed slider -- publishes to Nav2's built-in /speed_limit topic, which
  controller_server applies live to FollowPath's configured vx_max. No
  relaunch needed.

* Steering slider -- Nav2 has no topic-based equivalent to /speed_limit for
  steering, so this sets the max_steering_rad parameter directly on
  nav2_qcar_command_convert via a parameter client. That node clamps every
  Ackermann-converted steering command to it live.

* EMERGENCY STOP -- stops the car's motion only. The LiDAR keeps spinning and
  every node stays alive, so nothing has to be re-launched to resume. It
  cancels the running Nav2 goal (so the behavior tree stops issuing commands)
  and latches /qcar2_estop, which nav2_qcar_command_convert honours by
  publishing zero throttle regardless of what Nav2 asks for. The stop is
  gated inside the converter rather than by publishing zeroed motor commands
  from here, because the converter already republishes at 50 Hz and a second
  publisher would just race with it.

* STOP ALL -- full graceful shutdown via scripts/stop.sh, which SIGINTs the
  nodes so the LiDAR spin-down and the motor-zeroing shutdown hooks actually
  run. Never SIGKILL: that is what leaves the LiDAR spinning.
"""

import os
import subprocess
import tkinter as tk

import rclpy
from action_msgs.srv import CancelGoal
from nav2_msgs.msg import SpeedLimit
from rcl_interfaces.msg import Parameter as ParameterMsg
from rcl_interfaces.msg import ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool


BACKGROUND, PANEL = '#0b1220', '#121e32'
TEXT, MUTED, ACCENT = '#edf4ff', '#a8b9d1', '#37d8c4'
STOP, WARN = '#ef5266', '#ffbd69'

# Keep in step with FollowPath.vx_max in qcar2_slam_and_nav.yaml; used only to
# show the operator the resulting m/s next to the percentage.
NAV_MAX_SPEED = 0.36
# Keep in step with kSteeringStopRad in nav2_qcar_command_convert.cpp.  This
# is the MECHANICAL stop from qcar2/urdf/QCar2.urdf (hub joint limits,
# +/-0.5236 rad = 30 deg); commanding past it pins the linkage and adds no
# angle, so the slider must not offer more.
DEFAULT_MAX_STEERING_RAD = 0.52


class NavControlGui(Node):
    """Speed limit slider plus emergency-stop and shutdown controls."""

    def __init__(self):
        super().__init__('qcar2_speed_limit_gui')
        self.publisher = self.create_publisher(SpeedLimit, '/speed_limit', 1)
        # transient_local so the converter still sees an engaged stop if it
        # restarts, and so a converter that starts later picks it up.
        self.estop_publisher = self.create_publisher(
            Bool, '/qcar2_estop',
            QoSProfile(depth=1,
                       reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL))
        # Empty goal_info in a CancelGoal request means "cancel all goals".
        self.cancel_client = self.create_client(
            CancelGoal, '/navigate_to_pose/_action/cancel_goal')
        # No topic-based equivalent to /speed_limit exists for steering, so
        # the slider sets nav2_qcar_command_convert's max_steering_rad
        # parameter directly.
        self.steering_param_client = self.create_client(
            SetParameters, '/nav2_qcar2_command_converter/set_parameters')

        self.estopped = False
        self.root = tk.Tk()
        self.root.title('QCar2 • Nav Control')
        self.root.geometry('420x560')
        self.root.configure(bg=BACKGROUND)
        self.percent = tk.DoubleVar(value=100.0)
        self.steering_rad = tk.DoubleVar(value=DEFAULT_MAX_STEERING_RAD)
        self._build()
        self._publish_speed(100.0)
        self._publish_estop(False)
        self._set_steering_limit(DEFAULT_MAX_STEERING_RAD)

    def _build(self):
        tk.Label(self.root, text='NAV CONTROL', bg=BACKGROUND, fg=TEXT,
                 font=('TkDefaultFont', 15, 'bold')).pack(pady=(16, 10))

        speed_panel = tk.Frame(self.root, bg=PANEL, highlightbackground='#2a405f',
                               highlightthickness=1)
        speed_panel.pack(fill='x', padx=20)
        tk.Label(speed_panel, text='SPEED LIMIT', bg=PANEL, fg=MUTED,
                 font=('TkDefaultFont', 9, 'bold')).pack(anchor='w', padx=14, pady=(10, 0))
        self.value_label = tk.Label(speed_panel, text='100 %  (0.36 m/s)', bg=PANEL,
                                    fg=ACCENT, font=('TkDefaultFont', 15, 'bold'))
        self.value_label.pack(pady=(2, 0))
        self.scale = tk.Scale(speed_panel, variable=self.percent, from_=10, to=150,
                              resolution=5, orient='horizontal', length=330,
                              showvalue=False, bg=PANEL, fg=TEXT, troughcolor=BACKGROUND,
                              highlightthickness=0, activebackground=ACCENT,
                              command=self._on_slide)
        self.scale.pack(pady=(4, 4))
        tk.Button(speed_panel, text='Reset to 100%', command=lambda: self.scale.set(100),
                  bg=BACKGROUND, fg=TEXT, activebackground='#2a405f', relief='flat',
                  font=('TkDefaultFont', 9, 'bold'), padx=10, pady=4).pack(pady=(0, 12))

        steer_panel = tk.Frame(self.root, bg=PANEL, highlightbackground='#2a405f',
                               highlightthickness=1)
        steer_panel.pack(fill='x', padx=20, pady=(12, 0))
        tk.Label(steer_panel, text='MAX STEERING ANGLE', bg=PANEL, fg=MUTED,
                 font=('TkDefaultFont', 9, 'bold')).pack(anchor='w', padx=14, pady=(10, 0))
        self.steering_label = tk.Label(
            steer_panel, text=f'{DEFAULT_MAX_STEERING_RAD:.2f} rad', bg=PANEL,
            fg=ACCENT, font=('TkDefaultFont', 15, 'bold'))
        self.steering_label.pack(pady=(2, 0))
        self.steering_scale = tk.Scale(
            steer_panel, variable=self.steering_rad, from_=0.20, to=0.52,
            resolution=0.02, orient='horizontal', length=330,
            showvalue=False, bg=PANEL, fg=TEXT, troughcolor=BACKGROUND,
            highlightthickness=0, activebackground=ACCENT,
            command=self._on_steering_slide)
        self.steering_scale.pack(pady=(4, 4))
        tk.Button(steer_panel, text=f'Reset to {DEFAULT_MAX_STEERING_RAD:.2f} rad',
                  command=lambda: self.steering_scale.set(DEFAULT_MAX_STEERING_RAD),
                  bg=BACKGROUND, fg=TEXT, activebackground='#2a405f', relief='flat',
                  font=('TkDefaultFont', 9, 'bold'), padx=10, pady=4).pack(pady=(0, 12))

        self.status = tk.Label(self.root, text='RUNNING  •  navigation active',
                               bg=BACKGROUND, fg=ACCENT, font=('TkDefaultFont', 11, 'bold'))
        self.status.pack(pady=(14, 8))

        self.estop_button = tk.Button(
            self.root, text='EMERGENCY STOP', command=self._toggle_estop,
            bg=STOP, fg='white', activebackground='#c83f50', activeforeground='white',
            relief='flat', font=('TkDefaultFont', 13, 'bold'), pady=14)
        self.estop_button.pack(fill='x', padx=20)
        tk.Label(self.root, text='Stops motion only. LiDAR and all nodes keep running.',
                 bg=BACKGROUND, fg=MUTED, font=('TkDefaultFont', 8)).pack(pady=(4, 0))

        tk.Button(self.root, text='STOP ALL  (shut everything down)',
                  command=self._stop_all, bg=WARN, fg='#0a1a2d',
                  activebackground='#ffe2ae', relief='flat',
                  font=('TkDefaultFont', 11, 'bold'), pady=10).pack(fill='x', padx=20, pady=(14, 0))
        tk.Label(self.root, text='Graceful shutdown: spins the LiDAR down and zeroes the motors.',
                 bg=BACKGROUND, fg=MUTED, font=('TkDefaultFont', 8)).pack(pady=(4, 0))

    def _on_slide(self, value):
        self._publish_speed(float(value))

    def _on_steering_slide(self, value):
        self._set_steering_limit(float(value))

    def _publish_speed(self, percent):
        self.value_label.configure(
            text=f'{percent:.0f} %  ({NAV_MAX_SPEED * percent / 100.0:.2f} m/s)')
        message = SpeedLimit()
        message.header.stamp = self.get_clock().now().to_msg()
        message.percentage = True
        message.speed_limit = percent
        self.publisher.publish(message)

    def _set_steering_limit(self, radians):
        self.steering_label.configure(text=f'{radians:.2f} rad')
        if not self.steering_param_client.service_is_ready():
            # The slider still shows the intended value; it will be applied
            # once the converter's parameter service comes up (retried on the
            # next slider move) rather than blocking the GUI on it here.
            self.get_logger().warn(
                'nav2_qcar_command_convert parameter service unavailable; '
                'steering limit not applied yet.')
            return
        request = SetParameters.Request()
        parameter = ParameterMsg()
        parameter.name = 'max_steering_rad'
        parameter.value = ParameterValue(
            type=ParameterType.PARAMETER_DOUBLE, double_value=radians)
        request.parameters = [parameter]
        self.steering_param_client.call_async(request)

    def _publish_estop(self, engaged):
        message = Bool()
        message.data = engaged
        self.estop_publisher.publish(message)

    def _toggle_estop(self):
        self.estopped = not self.estopped
        self._publish_estop(self.estopped)
        if self.estopped:
            self._cancel_nav_goal()
            self.status.configure(text='EMERGENCY STOP  •  motion held at zero', fg=STOP)
            self.estop_button.configure(text='RELEASE EMERGENCY STOP', bg=ACCENT, fg='#0a1a2d')
        else:
            self.status.configure(text='RUNNING  •  navigation active', fg=ACCENT)
            self.estop_button.configure(text='EMERGENCY STOP', bg=STOP, fg='white')

    def _cancel_nav_goal(self):
        """Cancel any running navigate_to_pose goal so the BT stops commanding."""
        if not self.cancel_client.service_is_ready():
            # The stop still holds: the converter gates on /qcar2_estop
            # regardless of whether the goal could be cancelled.
            self.get_logger().warn(
                'navigate_to_pose cancel service unavailable; e-stop still engaged.')
            return
        self.cancel_client.call_async(CancelGoal.Request())

    def _stop_all(self):
        # Engage the stop first so the car is not moving during teardown.
        self.estopped = True
        self._publish_estop(True)
        self._cancel_nav_goal()
        self.status.configure(text='SHUTTING DOWN  •  stopping all nodes', fg=WARN)
        self.root.update_idletasks()
        script = os.path.join(
            os.path.expanduser('~'), 'Desktop', 'Qcar-rviz', 'scripts', 'stop.sh')
        if os.path.isfile(script):
            subprocess.Popen([script])
        else:
            self.get_logger().error(f'stop script not found: {script}')

    def _tick(self):
        rclpy.spin_once(self, timeout_sec=0.0)
        self.root.after(50, self._tick)

    def _close(self):
        self.root.destroy()
        rclpy.shutdown()

    def run(self):
        self.root.protocol('WM_DELETE_WINDOW', self._close)
        self.root.after(50, self._tick)
        self.root.mainloop()


def main():
    rclpy.init()
    gui = NavControlGui()
    try:
        gui.run()
    finally:
        if rclpy.ok():
            gui.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
