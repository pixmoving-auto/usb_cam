# Copyright 2023 Pixmoving, Inc. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
from ament_index_python import get_package_share_directory

import launch
from launch.actions import DeclareLaunchArgument
from launch.actions import OpaqueFunction
from launch.actions import SetLaunchConfiguration
from launch.conditions import IfCondition
from launch.conditions import UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.actions import LoadComposableNodes
from launch_ros.descriptions import ComposableNode
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import yaml


def generate_launch_description():
    launch_arguments = []
    def add_launch_arg(name: str, default_value=None):
        launch_arguments.append(DeclareLaunchArgument(name, default_value=default_value))
    # 加载参数   ----------------------------------------------------------------------------------------
    add_launch_arg("camera_container_name", "camera_preprocessor_container")
    
    parameters={
        "video_device": "/dev/video0",
        "framerate": 30.0,
        "io_method": "mmap",
        "frame_id": "test_camera",
        # "pixel_format": "yuyv2rgb",  
        "pixel_format": "uyvy2rgb",  
        "image_width": 1920,
        "image_height": 1080,
        "camera_name": "test_camera",
        # reusing same camera intrinsics only for demo, should really be unique for camera2"
        "camera_info_url": "file:///home/pixbus/pix/robobus/autoware-robobus/test/camera_info_3mm.yaml",
        "brightness": -1,
        "contrast": -1,
        "saturation": -1,
        "sharpness": -1,
        "gain": -1,
        "auto_white_balance": True,
        "white_balance": 4000,
        "autoexposure": True,
        "exposure": 100,
        "autofocus": False,
        "focus": -1,
        "rect_color": False
    }
    
    parameters["video_device"] = os.path.realpath(parameters["video_device"])
    
    # 创建组件节点----------------------------------------------------------------------------------------
    component_node = ComposableNode(
        package="usb_cam",
        plugin="usb_cam::UsbCamNode",
        name="usb_cam_node",
        namespace="/sensing/camera/top_12mm",
        parameters=[parameters],
    )
    
    
    
    # 加载组件节点-----------------------------------------------------------------------------------------
    camera_perception_loader = LoadComposableNodes(
        composable_node_descriptions=[
            component_node,
            ],
        target_container=LaunchConfiguration('camera_container_name'),
    )

    return launch.LaunchDescription([
        *launch_arguments,
        camera_perception_loader,
        ]
    )