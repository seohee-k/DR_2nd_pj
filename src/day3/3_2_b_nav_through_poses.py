#!/usr/bin/env python3

# Copyright 2022 Clearpath Robotics, Inc.
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
#
# @author Roni Kreinin (rkreinin@clearpathrobotics.com)

import rclpy

from turtlebot4_navigation.turtlebot4_navigator import TurtleBot4Directions, TurtleBot4Navigator


def main():
    rclpy.init()

    navigator = TurtleBot4Navigator()

    # Start on dock
    if not navigator.getDockedStatus():
        navigator.info('Docking before intialising pose')
        navigator.dock()

    # Set initial pose
    initial_pose = navigator.getPoseStamped([0.0, 0.0], TurtleBot4Directions.NORTH)
    navigator.setInitialPose(initial_pose)

    # Wait for Nav2
    navigator.waitUntilNav2Active()

    # Set goal poses
    goal_pose = []
    goal_pose.append(navigator.getPoseStamped([-1.55069, 0.0668084], TurtleBot4Directions.WEST))
    goal_pose.append(navigator.getPoseStamped([-0.761671, -0.852567], TurtleBot4Directions.SOUTH))
    goal_pose.append(navigator.getPoseStamped([0.0343325, -1.96793], TurtleBot4Directions.EAST))
    goal_pose.append(navigator.getPoseStamped([-0.711899, -0.0612125], TurtleBot4Directions.NORTH))
    # goal_pose.append(navigator.getPoseStamped([9.0, -1.0], TurtleBot4Directions.WEST))
    # goal_pose.append(navigator.getPoseStamped([9.0, 1.0], TurtleBot4Directions.SOUTH))
    # goal_pose.append(navigator.getPoseStamped([-1.0, 1.0], TurtleBot4Directions.EAST))

# Position(-1.55069, 0.0668084, 0), Orientation(0, 0, -0.962154, 0.272507) = Angle: -2.5896
# Position(-0.761671, -0.852567, 0), Orientation(0, 0, -0.488283, 0.872686) = Angle: -1.02024
# Position(0.0343325, -1.96793, 0), Orientation(0, 0, 0.109659, 0.993969) = Angle: 0.21976
# Position(-0.711899, -0.0612125, 0), Orientation(0, 0, 0.298801, 0.954315) = Angle: 0.606873


    # Undock
    navigator.undock()

    # Navigate through poses
    navigator.startThroughPoses(goal_pose)

    # Finished navigating, dock
    navigator.dock()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
