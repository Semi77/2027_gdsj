"""复用安装的Nav2 Humble配置，以低速和独立速度话题启动导航。"""
from pathlib import Path
import sys
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node


def navigation_parameters(defaults):
    """@brief 覆盖Nav2默认参数；defaults为系统安装包提供的完整配置。"""
    for value in defaults.values():
        if isinstance(value, dict) and 'ros__parameters' in value:
            value['ros__parameters']['use_sim_time'] = False
    amcl = defaults['amcl']['ros__parameters']
    amcl.update(base_frame_id='base_link', laser_max_range=4.0, laser_min_range=0.05,
                min_particles=300, max_particles=1000, update_min_d=0.02, update_min_a=0.05,
                transform_tolerance=0.5, sigma_hit=0.10)
    controller = defaults['controller_server']['ros__parameters']
    controller.update(controller_frequency=10.0, min_y_velocity_threshold=0.0)
    controller['progress_checker'].update(required_movement_radius=0.05, movement_time_allowance=20.0)
    controller['general_goal_checker'].update(xy_goal_tolerance=0.06, yaw_goal_tolerance=0.15)
    controller['FollowPath'].update(max_vel_x=0.08, max_vel_theta=0.4, max_speed_xy=0.08,
        min_speed_theta=0.0, acc_lim_x=0.2, decel_lim_x=-0.2, acc_lim_theta=0.6,
        decel_lim_theta=-0.6, vx_samples=8, vy_samples=1, vtheta_samples=12,
        sim_time=1.5, trans_stopped_velocity=0.015, xy_goal_tolerance=0.06,
        debug_trajectory_details=False)
    # 车体长25厘米、宽20厘米，额外留2厘米外形余量，雷达偏移暂按零处理。
    for key in ('local_costmap', 'global_costmap'):
        costmap = defaults[key][key]['ros__parameters']
        costmap.update(robot_base_frame='base_link',
            footprint='[[0.125,0.10],[0.125,-0.10],[-0.125,-0.10],[-0.125,0.10]]',
            footprint_padding=0.02, resolution=0.05)
        costmap.pop('robot_radius', None)
        costmap['inflation_layer'].update(inflation_radius=0.25, cost_scaling_factor=4.0)
        for plugin in ('voxel_layer','obstacle_layer'):
            if plugin in costmap:
                costmap[plugin]['scan'].update(raytrace_max_range=4.0, obstacle_max_range=3.8,
                    raytrace_min_range=0.05, obstacle_min_range=0.05)
    defaults['planner_server']['ros__parameters']['GridBased']['allow_unknown'] = False
    behavior = defaults['behavior_server']['ros__parameters']
    behavior.update(robot_base_frame='base_link', max_rotational_vel=0.4,
                    min_rotational_vel=0.08, rotational_acc_lim=0.6)
    smoother = defaults['velocity_smoother']['ros__parameters']
    smoother.update(max_velocity=[0.08,0.0,0.4], min_velocity=[-0.05,0.0,-0.4],
                    max_accel=[0.2,0.0,0.6], max_decel=[-0.2,0.0,-0.6], velocity_timeout=0.2)
    return defaults


def generate_launch_description():
    """@brief 启动地图定位、导航和坐标发布，速度只能进入自主控制专用话题。"""
    folder = Path(__file__).resolve().parent
    share = Path(get_package_share_directory('nav2_bringup'))
    defaults = yaml.safe_load((share/'params/nav2_params.yaml').read_text())
    defaults['map_server']['ros__parameters']['yaml_filename'] = '/workspace/ros2_ws/maps/console_map.yaml'
    configured = folder.parent/'logs/navigation_params.yaml'
    configured.write_text(yaml.safe_dump(navigation_parameters(defaults)), encoding='utf-8')
    nodes = [('nav2_map_server','map_server'), ('nav2_amcl','amcl'),
             ('nav2_controller','controller_server'), ('nav2_smoother','smoother_server'),
             ('nav2_planner','planner_server'), ('nav2_behaviors','behavior_server'),
             ('nav2_bt_navigator','bt_navigator'), ('nav2_waypoint_follower','waypoint_follower'),
             ('nav2_velocity_smoother','velocity_smoother')]
    actions = [ExecuteProcess(cmd=[sys.executable, str(folder/'navigation_tf.py')], output='screen')]
    for package,name in nodes:
        remaps = [('cmd_vel','nav_cmd_vel_raw')] if name in ('controller_server','behavior_server') else []
        if name == 'velocity_smoother':
            remaps = [('cmd_vel','nav_cmd_vel_raw'), ('cmd_vel_smoothed','nav_cmd_vel')]
        actions.append(Node(package=package, executable=name, name=name,
                            parameters=[str(configured)], remappings=remaps, output='screen'))
    for suffix,names in [('localization',['map_server','amcl']),
                         ('navigation',[name for _,name in nodes[2:]])]:
        actions.append(Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
            name='lifecycle_manager_'+suffix, output='screen',
            parameters=[{'use_sim_time':False, 'autostart':True, 'node_names':names}]))
    return LaunchDescription(actions)
