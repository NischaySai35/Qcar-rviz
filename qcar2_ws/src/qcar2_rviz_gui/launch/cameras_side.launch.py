"""
Rear / left / right CSI cameras plus a 5 Hz RViz preview of each.

Started and stopped by the "SHOW 360 CAMERAS" button in the in-RViz control
panel (qcar2_rviz_panels).  The FRONT camera is deliberately not here: it is
already running from hardware_base.launch.py, and the Quanser driver cannot
open the same camera_num twice.

Hardware camera_num mapping (fixed by the Quanser CSI driver):
  0 = right, 1 = rear, 2 = front, 3 = left

Each camera needs DISPLAY=:0 -- see hardware_base.launch.py for why.  The
raw streams are 820x616 @ 80 fps, far too much to push into three RViz Image
windows over a forwarded desktop, so each is throttled to a preview topic:
  /rear/camera/preview  /left/camera/preview  /right/camera/preview
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

CAMERAS = [
    ('rear', 1),
    ('left', 3),
    ('right', 0),
]


def generate_launch_description():
    # start_cameras:=false starts ONLY the preview relays. Used when the side
    # cameras are already running -- mapping with object detection launches
    # all four itself -- because the Quanser driver cannot open a camera twice,
    # but the browser console still needs the /<name>/camera/preview topics.
    start_cameras = LaunchConfiguration('start_cameras')
    nodes = [DeclareLaunchArgument(
        'start_cameras', default_value='true',
        description='Start the csi camera nodes, or only the preview relays')]
    for name, cam_num in CAMERAS:
        nodes.append(Node(
            package='qcar2_nodes',
            executable='csi',
            name=f'csi_{name}',
            namespace=name,
            condition=IfCondition(start_cameras),
            additional_env={'DISPLAY': ':0', 'XAUTHORITY': '/home/nvidia/.Xauthority'},
            parameters=[{
                'camera_num': cam_num,
                'device_type': 'physical',
                'frame_width': 820,
                'frame_height': 616,
                'frame_rate': 80.0,
            }],
            output='screen',
        ))
        nodes.append(Node(
            package='qcar2_rviz_gui',
            executable='image_preview_throttle.py',
            name=f'{name}_camera_preview',
            output='screen',
            parameters=[{
                'input_topic': f'/{name}/camera/csi_image',
                'output_topic': f'/{name}/camera/preview',
                'preview_hz': 5.0,
            }],
        ))
    return LaunchDescription(nodes)
