# Copyright 2019 Intelligent Robotics Lab
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

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Get the launch directory
    #example_dir = get_package_share_directory('planning')
    example_dir ='/root/experimental_robotics/src/planning/'
    namespace = LaunchConfiguration('namespace')

    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Namespace')

    stdout_linebuf_envvar = SetEnvironmentVariable(
        'RCUTILS_CONSOLE_STDOUT_LINE_BUFFERED', '1')

    plansys2_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('plansys2_bringup'),
            'launch',
            'plansys2_bringup_launch_monolithic.py')),
        launch_arguments={
          'model_file': example_dir + '/pddl/prova.pddl',
          'namespace': namespace
          }.items())
          
    #slam toolbox launch 
    # Ottieni il percorso del file di lancio di slam_toolbox
    slam_toolbox_launch_dir = os.path.join(
        get_package_share_directory('slam_toolbox'),
        'launch'
    )

    # Include il file di lancio di slam_toolbox
    slam_toolbox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(slam_toolbox_launch_dir, 'online_sync_launch.py')
        )
    )      
    '''
    nav2_bringup_launch_dir = os.path.join(
        get_package_share_directory('nav2_bringup'),
        'launch'
    )

    # Include the 'navigation_launch.py' launch file from the nav2_bringup package
    nav2_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_launch_dir, 'navigation_launch.py')
        )
    )'''
    
    nav2_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('nav2_bringup'),
            'launch',
            'navigation_launch.py')),
        launch_arguments={
            'autostart': 'true',
            'params_file': os.path.join(example_dir, 'params', 'nav2_params.yaml')
        }.items())


    
    
    # Specify the actions
    move_cmd = Node(
        package='planning',
        executable='move_node',
        name='move_node',
        namespace=namespace,
        output='screen',
        parameters=[])

    detect_cmd = Node(
        package='planning',
        executable='detect_node',
        name='detect_node',
        namespace=namespace,
        output='screen',
        parameters=[])

    move_in_order_cmd = Node(
        package='planning',
        executable='move_in_order_node',
        name='move_in_order_node',
        namespace=namespace,
        output='screen',
        parameters=[])   # Create the launch description and populate
    ld = LaunchDescription()

    # Set environment variables
    ld.add_action(stdout_linebuf_envvar)
    ld.add_action(declare_namespace_cmd)

    # Declare the launch options
    ld.add_action(plansys2_cmd)
    
    # Add slam_toolbox launch
    ld.add_action(slam_toolbox_launch)
    #ld.add_action(nav2_cmd)
    #ld.add_action(move_cmd)
    #ld.add_action(detect_cmd)
    ld.add_action(move_in_order_cmd)

    return ld
