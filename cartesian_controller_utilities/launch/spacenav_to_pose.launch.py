################################################################################
# Copyright 2022 FZI Research Center for Information Technology
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice,
# this list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright notice,
# this list of conditions and the following disclaimer in the documentation
# and/or other materials provided with the distribution.
#
# 3. Neither the name of the copyright holder nor the names of its
# contributors may be used to endorse or promote products derived from this
# software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
################################################################################

# -----------------------------------------------------------------------------
# \file    spacenav_to_pose.launch.py
#
# \author  Stefan Scherzinger <scherzin@fzi.de>
# \date    2022/11/10
#
# -----------------------------------------------------------------------------

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    spacenav_node = Node(
        package="spacenav",
        executable="spacenav_node",
        parameters=[
            {"zero_when_static": True},
            {"static_count_threshold": 30},
            {"linear_scale/x": 0.5},
            {"linear_scale/y": 0.5},
            {"linear_scale/z": 0.5},
            {"angular_scale/x": 1.0},
            {"angular_scale/y": 1.0},
            {"angular_scale/z": 1.0},
        ],
        output="both",
    )

    converter_node = Node(
        package="cartesian_controller_utilities",
        executable="pose.py",
        parameters=[
            {"twist_topic": "/spacenav/twist"},
            {"pose_topic": "/target_frame"},
            {"frame_id": "base_link"},
            {"end_effector": "tool0"},
            {"publishing_rate": 100},
        ],
        output="both",
    )

    return LaunchDescription([spacenav_node, converter_node])
