#!/usr/bin/python3
# Hardcoded, not `env python3`: see image_preview_throttle.py for why --
# `env python3` can resolve to pyenv's Python 3.7 instead of the system
# 3.8 rclpy's C extension needs, depending on shell PATH state.
"""Responsive hold-to-drive controller for safe QCar2 mapping sessions."""

import math
import tkinter as tk

import rclpy
from qcar2_interfaces.msg import MotorCommands
from rclpy.node import Node
from sensor_msgs.msg import BatteryState


BACKGROUND, PANEL, PANEL_ALT = '#0b1220', '#121e32', '#1c2c46'
TEXT, MUTED, ACCENT, STOP = '#edf4ff', '#a8b9d1', '#37d8c4', '#ef5266'
FORWARD, REVERSE, TURN = '#3acfc0', '#a77cf2', '#6597ee'


class DriveGui(Node):
    """Publish a combined speed/steering command while controls are held."""

    KEY_TO_CONTROL = {
        'w': 'forward', 'up': 'forward', 's': 'reverse', 'down': 'reverse',
        'a': 'left', 'left': 'left', 'd': 'right', 'right': 'right',
    }

    def __init__(self):
        super().__init__('qcar2_drive_gui')
        self.publisher = self.create_publisher(MotorCommands, '/qcar2_motor_speed_cmd', 10)
        self.create_subscription(BatteryState, '/qcar2_battery', self._battery, 10)
        self.held, self.command, self.armed = set(), (0.0, 0.0), True
        self.battery_voltage = math.nan
        self.root = tk.Tk()
        self.root.title('QCar2 • Drive Console')
        self.root.geometry('840x650')
        self.root.minsize(740, 560)
        self.root.configure(bg=BACKGROUND)
        self.root.protocol('WM_DELETE_WINDOW', self._close)
        self.root.bind('<KeyPress>', self._key_down)
        self.root.bind('<KeyRelease>', self._key_up)
        self.root.bind('<FocusOut>', lambda _event: self._release_all())
        self.speed, self.steering = tk.DoubleVar(value=0.42), tk.DoubleVar(value=0.45)
        self.status = tk.StringVar(value='READY  •  Choose a direction and hold to drive')
        self.battery = tk.StringVar(value='Battery  •  waiting for QCar')
        self._build()

    def _build(self):
        header = tk.Frame(self.root, bg=BACKGROUND)
        header.pack(fill='x', padx=28, pady=(23, 13))
        tk.Label(header, text='QCar2  /  DRIVE', bg=BACKGROUND, fg=TEXT,
                 font=('TkDefaultFont', 23, 'bold')).pack(anchor='w')
        tk.Label(header, text='Hold a control to move. Diagonal controls drive and steer together.',
                 bg=BACKGROUND, fg=MUTED, font=('TkDefaultFont', 10)).pack(anchor='w', pady=(2, 0))
        status_frame = self._panel(self.root)
        status_frame.pack(fill='x', padx=28, pady=(0, 13))
        self.status_label = tk.Label(status_frame, textvariable=self.status, bg=PANEL, fg=ACCENT,
                                     font=('TkDefaultFont', 11, 'bold'), pady=11)
        self.status_label.pack(side='left', padx=15)
        tk.Label(status_frame, textvariable=self.battery, bg=PANEL, fg=MUTED,
                 font=('TkDefaultFont', 10)).pack(side='right', padx=15)
        content = tk.Frame(self.root, bg=BACKGROUND)
        content.pack(fill='both', expand=True, padx=28)
        content.columnconfigure(0, weight=3)
        content.columnconfigure(1, weight=2)
        content.rowconfigure(0, weight=1)
        drive, controls = self._panel(content), self._panel(content)
        drive.grid(row=0, column=0, sticky='nsew', padx=(0, 8))
        controls.grid(row=0, column=1, sticky='nsew', padx=(8, 0))
        tk.Label(drive, text='DRIVE PAD', bg=PANEL, fg=TEXT,
                 font=('TkDefaultFont', 12, 'bold')).pack(anchor='w', padx=18, pady=(16, 3))
        tk.Label(drive, text='W / ↑ forward    S / ↓ reverse    A / ← left    D / → right',
                 bg=PANEL, fg=MUTED, font=('TkDefaultFont', 9)).pack(anchor='w', padx=18)
        pad = tk.Frame(drive, bg=PANEL)
        pad.pack(padx=18, pady=(10, 14), expand=True)
        for index in range(3):
            pad.columnconfigure(index, weight=1)
            pad.rowconfigure(index, weight=1)
        self._hold_button(pad, '↖\nFWD LEFT', 0, 0, ('forward', 'left'), FORWARD)
        self._hold_button(pad, '↑\nFORWARD', 0, 1, ('forward',), FORWARD)
        self._hold_button(pad, 'FWD RIGHT\n↗', 0, 2, ('forward', 'right'), FORWARD)
        self._hold_button(pad, '←\nSTEER LEFT', 1, 0, ('left',), TURN)
        tk.Button(pad, text='■\nSTOP', command=self._release_all, width=12, height=3,
                  bg='#ffbd69', fg='#0a1a2d', activebackground='#ffe2ae', relief='flat',
                  font=('TkDefaultFont', 9, 'bold')).grid(row=1, column=1, padx=5, pady=5, sticky='nsew')
        self._hold_button(pad, 'STEER RIGHT\n→', 1, 2, ('right',), TURN)
        self._hold_button(pad, '↙\nREV LEFT', 2, 0, ('reverse', 'left'), REVERSE)
        self._hold_button(pad, '↓\nREVERSE', 2, 1, ('reverse',), REVERSE)
        self._hold_button(pad, 'REV RIGHT\n↘', 2, 2, ('reverse', 'right'), REVERSE)
        tk.Label(controls, text='DRIVE LIMITS', bg=PANEL, fg=TEXT,
                 font=('TkDefaultFont', 12, 'bold')).pack(anchor='w', padx=18, pady=(16, 8))
        # 0.55 m/s is deliberately only a modest increase: faster motion
        # reduces scan overlap and makes indoor SLAM easier to lose.
        self._scale(controls, 'Speed', self.speed, 0.04, 0.55, 0.01, 'm/s')
        # 0.52 is the mechanical stop (URDF hub joint limit 0.5236 rad).
        self._scale(controls, 'Steering', self.steering, 0.05, 0.52, 0.05, 'rad')
        tk.Label(controls, text='Keyboard supports combinations:\nhold W + D for forward-right,\nor S + A for reverse-left.\n\nStraight arrow = motion.\nSide arrow = steering only.',
                 bg=PANEL_ALT, fg=MUTED, justify='left', padx=13, pady=13,
                 font=('TkDefaultFont', 9)).pack(fill='x', padx=18, pady=(9, 12))
        footer = tk.Frame(self.root, bg=BACKGROUND)
        footer.pack(fill='x', padx=28, pady=18)
        tk.Button(footer, text='EMERGENCY STOP  [SPACE]', command=self._emergency_stop,
                  bg=STOP, fg='white', activebackground='#c83f50', activeforeground='white', relief='flat',
                  font=('TkDefaultFont', 11, 'bold'), padx=17, pady=11).pack(side='left')
        tk.Button(footer, text='Reset & arm', command=self._arm, bg=PANEL_ALT, fg=TEXT,
                  activebackground='#2a405f', activeforeground=TEXT, relief='flat',
                  font=('TkDefaultFont', 10, 'bold'), padx=15, pady=11).pack(side='left', padx=9)
        tk.Label(footer, text='Release every held key/button to stop', bg=BACKGROUND, fg=MUTED,
                 font=('TkDefaultFont', 9)).pack(side='right')

    @staticmethod
    def _panel(parent):
        return tk.Frame(parent, bg=PANEL, highlightbackground='#2a405f', highlightthickness=1)

    def _scale(self, parent, label, variable, low, high, step, units):
        row = tk.Frame(parent, bg=PANEL)
        row.pack(fill='x', padx=18, pady=7)
        value = tk.StringVar(value=f'{variable.get():.2f} {units}')
        variable.trace_add('write', lambda *_: value.set(f'{variable.get():.2f} {units}'))
        tk.Label(row, text=label, bg=PANEL, fg=MUTED, font=('TkDefaultFont', 9)).pack(anchor='w')
        tk.Label(row, textvariable=value, bg=PANEL, fg=TEXT, font=('TkDefaultFont', 10, 'bold')).pack(anchor='w')
        tk.Scale(row, variable=variable, from_=low, to=high, resolution=step, orient='horizontal',
                 showvalue=False, length=190, bg=PANEL, fg=TEXT, troughcolor=PANEL_ALT,
                 highlightthickness=0, activebackground=ACCENT).pack(anchor='w')

    def _hold_button(self, parent, label, row, col, controls, color):
        button = tk.Button(parent, text=label, width=12, height=3, bg=color, fg='#0a1a2d',
                           activebackground='#edf4ff', relief='flat', font=('TkDefaultFont', 9, 'bold'))
        button.grid(row=row, column=col, padx=5, pady=5, sticky='nsew')
        token = f'button-{row}-{col}'
        button.bind('<ButtonPress-1>', lambda _event: self._hold(token, controls))
        button.bind('<ButtonRelease-1>', lambda _event: self._unhold(token))

    def _hold(self, token, controls):
        if not self.armed:
            self.status.set('STOPPED  •  Press “Reset & arm” before driving')
            return
        self.held = {entry for entry in self.held if entry[0] != token}
        self.held.update((token, control) for control in controls)
        self._update_command()

    def _unhold(self, token):
        self.held = {entry for entry in self.held if entry[0] != token}
        self._update_command()

    def _release_all(self):
        self.held.clear()
        self._update_command()
        self._publish(0.0, 0.0)

    def _update_command(self):
        controls = {control for _, control in self.held}
        speed = self.speed.get() * (int('forward' in controls) - int('reverse' in controls))
        steering = self.steering.get() * (int('left' in controls) - int('right' in controls))
        if not self.armed:
            speed = steering = 0.0
        self.command = (speed, steering)
        if speed or steering:
            motion = 'FORWARD' if speed > 0 else 'REVERSE' if speed < 0 else 'STEERING'
            turn = ' LEFT' if steering > 0 else ' RIGHT' if steering < 0 else ''
            self.status.set(f'DRIVING  •  {motion}{turn}  •  release to stop')
        elif self.armed:
            self.status.set('READY  •  Choose a direction and hold to drive')

    def _key_down(self, event):
        key = event.keysym.lower()
        if key in ('space', 'escape'):
            self._emergency_stop()
        elif key in self.KEY_TO_CONTROL:
            self._hold(f'key-{key}', (self.KEY_TO_CONTROL[key],))

    def _key_up(self, event):
        key = event.keysym.lower()
        if key in self.KEY_TO_CONTROL:
            self._unhold(f'key-{key}')

    def _emergency_stop(self, *_event):
        self.armed, self.held, self.command = False, set(), (0.0, 0.0)
        self.status.set('EMERGENCY STOP  •  Commands are locked at zero')
        self.status_label.configure(fg=STOP)
        self._publish(0.0, 0.0)

    def _arm(self):
        self.armed = True
        self.status_label.configure(fg=ACCENT)
        self._update_command()

    def _battery(self, message):
        self.battery_voltage = message.voltage

    def _publish(self, speed, steering):
        message = MotorCommands()
        message.motor_names, message.values = ['steering_angle', 'motor_throttle'], [steering, speed]
        self.publisher.publish(message)

    def _tick(self):
        self._publish(*(self.command if self.armed else (0.0, 0.0)))
        if math.isfinite(self.battery_voltage):
            self.battery.set(f'Battery  •  {self.battery_voltage:.1f} V')
        rclpy.spin_once(self, timeout_sec=0.0)
        self.root.after(50, self._tick)

    def _close(self):
        self._release_all()
        self.root.destroy()
        rclpy.shutdown()

    def run(self):
        self.root.after(50, self._tick)
        self.root.mainloop()


def main():
    rclpy.init()
    gui = DriveGui()
    try:
        gui.run()
    finally:
        if rclpy.ok():
            gui._release_all()
            gui.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
