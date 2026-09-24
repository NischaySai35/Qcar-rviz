"""
Brings up all 4 QCar2 CSI cameras (front / rear / left / right) at once,
each in its own ROS namespace, so together they give full 360-degree
coverage around the car.

Topics published (sensor_msgs/Image):
  /front/camera/csi_image
  /rear/camera/csi_image
  /left/camera/csi_image
  /right/camera/csi_image

Hardware camera_num mapping (fixed by the Quanser CSI driver):
  0 = right, 1 = rear, 2 = front, 3 = left
"""
from launch import LaunchDescription
from launch_ros.actions import Node

CAMERAS = [
    ('front', 2),
    ('rear', 1),
    ('left', 3),
    ('right', 0),
]


def generate_launch_description():
    nodes = []
    for name, cam_num in CAMERAS:
        nodes.append(
            Node(
                package='qcar2_nodes',
                executable='csi',
                name=f'csi_{name}',
                namespace=name,
                # 820x410 (the driver's own default) is only a valid native
                # mode at 120fps, NOT 30fps - requesting an invalid
                # width/height/rate combo makes the camera fail to open
                # ("video format is not supported"). 820x616 @ 80fps is one
                # of the CSI driver's documented native modes.
                #
                # additional_env DISPLAY: see hardware_base.launch.py's
                # front_camera_node for the full root-cause writeup. Short
                # version: Quanser's video_capture_open() needs Argus/
                # EGLStream, which needs a real GPU-backed X display; running
                # this over SSH/remote desktop means the inherited DISPLAY is
                # a forwarded/virtual one, not the local seat0 display the
                # Tegra GPU is actually attached to. Forcing it just for this
                # process (not the whole launch) is what actually fixes the
                # camera without touching DISPLAY for anything else, such as
                # the Tk GUI windows, that already renders fine over SSH.
                additional_env={'DISPLAY': ':0', 'XAUTHORITY': '/home/nvidia/.Xauthority'},
                parameters=[{
                    'camera_num': cam_num,
                    'device_type': 'physical',
                    'frame_width': 820,
                    'frame_height': 616,
                    'frame_rate': 80.0,
                }],
                respawn=True,
                respawn_delay=2.0,
                output='screen',
            )
        )
    return LaunchDescription(nodes)
