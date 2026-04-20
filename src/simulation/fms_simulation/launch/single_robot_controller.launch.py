"""Launch configuration for single robot controller.launch."""

# import os
# from ament_index_python.packages import get_package_share_directory
# from launch import LaunchDescription
# from launch.substitutions import LaunchConfiguration, NotSubstitution
# from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, LogInfo
# from launch.conditions import IfCondition
# from launch_ros.actions import Node

# def generate_launch_description():

#     print("single_robot_controller")

#     launch_dir = get_package_share_directory('fms_server')

#     is_pure_rotation = LaunchConfiguration('is_pure_rotation')
#     is_pure_rotation_arg = DeclareLaunchArgument('is_pure_rotation', default_value='true')

#     max_v = LaunchConfiguration('max_v')
#     max_v_arg = DeclareLaunchArgument('max_v', default_value='1.0')

#     max_w = LaunchConfiguration('max_w')
#     max_w_arg = DeclareLaunchArgument('max_w', default_value='0.5')

#     robot_model = LaunchConfiguration('robot_model')
#     robot_model_arg = DeclareLaunchArgument('robot_model', default_value='robot_model_0')

#     param_file = LaunchConfiguration('param_file')  ##########
#     param_file_arg = DeclareLaunchArgument('param_file', default_value='1.5') ##########

#     run_stage = LaunchConfiguration('run_stage')
#     run_stage_arg = DeclareLaunchArgument('run_stage', default_value='false')


#     return LaunchDescription([
#         is_pure_rotation_arg, max_v_arg, max_w_arg,run_stage_arg,
#         Node(
#             package='robot_manager', executable='fms_single_ctrl_node', name='robot_manager1', output='both',
#             parameters=[{'robot_name': 'carter1',  'is_pure_rotation': is_pure_rotation, 'max_v': max_v, 'max_w': max_w, 'run_stage': run_stage}],
#         ),
#         Node(
#             package='robot_manager', executable='fms_single_ctrl_node', name='robot_manager2', output='both',
#             parameters=[{'robot_name': 'carter2',  'is_pure_rotation': is_pure_rotation, 'max_v': max_v, 'max_w': max_w, 'run_stage': run_stage}],
#         ),
#         Node(
#             package='robot_manager', executable='fms_single_ctrl_node', name='robot_manager3', output='both',
#             parameters=[{'robot_name': 'carter3',  'is_pure_rotation': is_pure_rotation, 'max_v': max_v, 'max_w': max_w, 'run_stage': run_stage}],
#         ),
#         Node(
#             package='robot_manager', executable='fms_single_ctrl_node', name='robot_manager4', output='both',
#             parameters=[{'robot_name': 'carter4',  'is_pure_rotation': is_pure_rotation, 'max_v': max_v, 'max_w': max_w, 'run_stage': run_stage}],
#         ),
#         Node(
#             package='robot_manager', executable='fms_single_ctrl_node', name='robot_manager5', output='both',
#             parameters=[{'robot_name': 'carter5',  'is_pure_rotation': is_pure_rotation, 'max_v': max_v, 'max_w': max_w, 'run_stage': run_stage}],
#         ),
#         Node(
#             package='robot_manager', executable='fms_single_ctrl_node', name='robot_manager6', output='both',
#             parameters=[{'robot_name': 'carter6',  'is_pure_rotation': is_pure_rotation, 'max_v': max_v, 'max_w': max_w, 'run_stage': run_stage}],
#         ),
#         Node(
#             package='robot_manager', executable='fms_single_ctrl_node', name='robot_manager7', output='both',
#             parameters=[{'robot_name': 'carter7',  'is_pure_rotation': is_pure_rotation, 'max_v': max_v, 'max_w': max_w, 'run_stage': run_stage}],
#         ),
#         Node(
#             package='robot_manager', executable='fms_single_ctrl_node', name='robot_manager8', output='both',
#             parameters=[{'robot_name': 'carter8',  'is_pure_rotation': is_pure_rotation, 'max_v': max_v, 'max_w': max_w, 'run_stage': run_stage}],
#         ),
#         Node(
#             package='robot_manager', executable='fms_single_ctrl_node', name='robot_manager9', output='both',
#             parameters=[{'robot_name': 'carter9',  'is_pure_rotation': is_pure_rotation, 'max_v': max_v, 'max_w': max_w, 'run_stage': run_stage}],
#         ),
#         Node(
#             package='robot_manager', executable='fms_single_ctrl_node', name='robot_manager10', output='both',
#             parameters=[{'robot_name': 'carter10', 'is_pure_rotation': is_pure_rotation, 'max_v': max_v, 'max_w': max_w, 'run_stage': run_stage}],
#         ),
#         Node(
#             package='robot_manager', executable='fms_single_ctrl_node', name='robot_manager11', output='both',
#             parameters=[{'robot_name': 'carter11', 'is_pure_rotation': is_pure_rotation, 'max_v': max_v, 'max_w': max_w, 'run_stage': run_stage}],
#         ),
#         Node(
#             package='robot_manager', executable='fms_single_ctrl_node', name='robot_manager12', output='both',
#             parameters=[{'robot_name': 'carter12', 'is_pure_rotation': is_pure_rotation, 'max_v': max_v, 'max_w': max_w, 'run_stage': run_stage}],
#         ),
#         Node(
#             package='robot_manager', executable='fms_single_ctrl_node', name='robot_manager13', output='both',
#             parameters=[{'robot_name': 'carter13', 'is_pure_rotation': is_pure_rotation, 'max_v': max_v, 'max_w': max_w, 'run_stage': run_stage}],
#         ),
#         Node(5
#             package='robot_manager', executable='fms_single_ctrl_node', name='robot_manager14', output='both',
#             parameters=[{'robot_name': 'carter14', 'is_pure_rotation': is_pure_rotation, 'max_v': max_v, 'max_w': max_w, 'run_stage': run_stage}],
#         ),
#         Node(
#             package='robot_manager', executable='fms_single_ctrl_node', name='robot_manager15', output='both',
#             parameters=[{'robot_name': 'carter15', 'is_pure_rotation': is_pure_rotation, 'max_v': max_v, 'max_w': max_w, 'run_stage': run_stage}],
#         ),
#         Node(
#             package='robot_manager', executable='fms_single_ctrl_node', name='robot_manager16', output='both',
#             parameters=[{'robot_name': 'carter16', 'is_pure_rotation': is_pure_rotation, 'max_v': max_v, 'max_w': max_w, 'run_stage': run_stage}],
#         ),
#         Node(
#             package='robot_manager', executable='fms_single_ctrl_node', name='robot_manager17', output='both',
#             parameters=[{'robot_name': 'carter17', 'is_pure_rotation': is_pure_rotation, 'max_v': max_v, 'max_w': max_w, 'run_stage': run_stage}],
#         ),
#         Node(
#             package='robot_manager', executable='fms_single_ctrl_node', name='robot_manager18', output='both',
#             parameters=[{'robot_name': 'carter18', 'is_pure_rotation': is_pure_rotation, 'max_v': max_v, 'max_w': max_w, 'run_stage': run_stage}],
#         ),
#         Node(
#             package='robot_manager', executable='fms_single_ctrl_node', name='robot_manager19', output='both',
#             parameters=[{'robot_name': 'carter19', 'is_pure_rotation': is_pure_rotation, 'max_v': max_v, 'max_w': max_w, 'run_stage': run_stage}],
#         ),
#         Node(
#             package='robot_manager', executable='fms_single_ctrl_node', name='robot_manager20', output='both',
#             parameters=[{'robot_name': 'carter20', 'is_pure_rotation': is_pure_rotation, 'max_v': max_v, 'max_w': max_w, 'run_stage': run_stage}],
#         ),
#         Node(
#             package='robot_manager', executable='fms_single_ctrl_node', name='robot_manager21', output='both',
#             parameters=[{'robot_name': 'carter21', 'is_pure_rotation': is_pure_rotation, 'max_v': max_v, 'max_w': max_w, 'run_stage': run_stage}],
#         ),
#         Node(
#             package='robot_manager', executable='fms_single_ctrl_node', name='robot_manager22', output='both',
#             parameters=[{'robot_name': 'carter22', 'is_pure_rotation': is_pure_rotation, 'max_v': max_v, 'max_w': max_w, 'run_stage': run_stage}],
#         ),
#         Node(
#             package='robot_manager', executable='fms_single_ctrl_node', name='robot_manager23', output='both',
#             parameters=[{'robot_name': 'carter23', 'is_pure_rotation': is_pure_rotation, 'max_v': max_v, 'max_w': max_w, 'run_stage': run_stage}],
#         ),
#         Node(
#             package='robot_manager', executable='fms_single_ctrl_node', name='robot_manager24', output='both',
#             parameters=[{'robot_name': 'carter24', 'is_pure_rotation': is_pure_rotation, 'max_v': max_v, 'max_w': max_w, 'run_stage': run_stage}],
#         ),
#         Node(
#             package='robot_manager', executable='fms_single_ctrl_node', name='robot_manager25', output='both',
#             parameters=[{'robot_name': 'carter25', 'is_pure_rotation': is_pure_rotation, 'max_v': max_v, 'max_w': max_w, 'run_stage': run_stage}],
#         ),
#         Node(
#             package='robot_manager', executable='fms_single_ctrl_node', name='robot_manager26', output='both',
#             parameters=[{'robot_name': 'carter26', 'is_pure_rotation': is_pure_rotation, 'max_v': max_v, 'max_w': max_w, 'run_stage': run_stage}],
#         ),
#         Node(
#             package='robot_manager', executable='fms_single_ctrl_node', name='robot_manager27', output='both',
#             parameters=[{'robot_name': 'carter27', 'is_pure_rotation': is_pure_rotation, 'max_v': max_v, 'max_w': max_w, 'run_stage': run_stage}],
#         ),
#         Node(
#             package='robot_manager', executable='fms_single_ctrl_node', name='robot_manager28', output='both',
#             parameters=[{'robot_name': 'carter28', 'is_pure_rotation': is_pure_rotation, 'max_v': max_v, 'max_w': max_w, 'run_stage': run_stage}],
#         ),
#         Node(
#             package='robot_manager', executable='fms_single_ctrl_node', name='robot_manager29', output='both',
#             parameters=[{'robot_name': 'carter29', 'is_pure_rotation': is_pure_rotation, 'max_v': max_v, 'max_w': max_w, 'run_stage': run_stage}],
#         ),
#         Node(
#             package='robot_manager', executable='fms_single_ctrl_node', name='robot_manager30', output='both',
#             parameters=[{'robot_name': 'carter30', 'is_pure_rotation': is_pure_rotation, 'max_v': max_v, 'max_w': max_w, 'run_stage': run_stage}],
#         ),
#         Node(
#             package='robot_manager', executable='fms_single_ctrl_node', name='robot_manager31', output='both',
#             parameters=[{'robot_name': 'carter31', 'is_pure_rotation': is_pure_rotation, 'max_v': max_v, 'max_w': max_w, 'run_stage': run_stage}],
#         ),
#         Node(
#             package='robot_manager', executable='fms_single_ctrl_node', name='robot_manager32', output='both',
#             parameters=[{'robot_name': 'carter32', 'is_pure_rotation': is_pure_rotation, 'max_v': max_v, 'max_w': max_w, 'run_stage': run_stage}],
#         ),
#         Node(
#             package='robot_manager', executable='fms_single_ctrl_node', name='robot_manager33', output='both',
#             parameters=[{'robot_name': 'carter33', 'is_pure_rotation': is_pure_rotation, 'max_v': max_v, 'max_w': max_w, 'run_stage': run_stage}],
#         ),
#         Node(
#             package='robot_manager', executable='fms_single_ctrl_node', name='robot_manager34', output='both',
#             parameters=[{'robot_name': 'carter34', 'is_pure_rotation': is_pure_rotation, 'max_v': max_v, 'max_w': max_w, 'run_stage': run_stage}],
#         ),
#         Node(
#             package='robot_manager', executable='fms_single_ctrl_node', name='robot_manager35', output='both',
#             parameters=[{'robot_name': 'carter35', 'is_pure_rotation': is_pure_rotation, 'max_v': max_v, 'max_w': max_w, 'run_stage': run_stage}],
#         ),
#         Node(
#             package='robot_manager', executable='fms_single_ctrl_node', name='robot_manager36', output='both',
#             parameters=[{'robot_name': 'carter36', 'is_pure_rotation': is_pure_rotation, 'max_v': max_v, 'max_w': max_w, 'run_stage': run_stage}],
#         ),
#         Node(
#             package='robot_manager', executable='fms_single_ctrl_node', name='robot_manager37', output='both',
#             parameters=[{'robot_name': 'carter37', 'is_pure_rotation': is_pure_rotation, 'max_v': max_v, 'max_w': max_w, 'run_stage': run_stage}],
#         ),
#         Node(
#             package='robot_manager', executable='fms_single_ctrl_node', name='robot_manager38', output='both',
#             parameters=[{'robot_name': 'carter38', 'is_pure_rotation': is_pure_rotation, 'max_v': max_v, 'max_w': max_w, 'run_stage': run_stage}],
#         ),
#         Node(
#             package='robot_manager', executable='fms_single_ctrl_node', name='robot_manager39', output='both',
#             parameters=[{'robot_name': 'carter39', 'is_pure_rotation': is_pure_rotation, 'max_v': max_v, 'max_w': max_w, 'run_stage': run_stage}],
#         ),
#         Node(
#             package='robot_manager', executable='fms_single_ctrl_node', name='robot_manager40', output='both',
#             parameters=[{'robot_name': 'carter40', 'is_pure_rotation': is_pure_rotation, 'max_v': max_v, 'max_w': max_w, 'run_stage': run_stage}],
#         ),
#         Node(
#             package='robot_manager', executable='fms_single_ctrl_node', name='robot_manager41', output='both',
#             parameters=[{'robot_name': 'carter41', 'is_pure_rotation': is_pure_rotation, 'max_v': max_v, 'max_w': max_w, 'run_stage': run_stage}],
#         ),import os
# from ament_index_python.packages import get_package_share_directory
# from launch import LaunchDescription
# from launch.substitutions import LaunchConfiguration, NotSubstitution
# from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, LogInfo
# from launch.conditions import IfCondition
# from launch_ros.actions import Node

# def generate_launch_description():

#     print("single_robot_controller")

#     launch_dir = get_package_share_directory('fms_server')

#     is_pure_rotation = LaunchConfiguration('is_pure_rotation')
#     is_pure_rotation_arg = DeclareLaunchArgument('is_pure_rotation', default_value='true')

#     max_v = LaunchConfiguration('max_v')
#     max_v_arg = DeclareLaunchArgument('max_v', default_value='1.0')

#     max_w = LaunchConfiguration('max_w')
#     max_w_arg = DeclareLaunchArgument('max_w', default_value='0.5')

#     robot_model = LaunchConfiguration('robot_model')
#     robot_model_arg = DeclareLaunchArgument('robot_model', default_value='robot_model_0')

#     param_file = LaunchConfiguration('param_file')  ##########
#     param_file_arg = DeclareLaunchArgument('param_file', default_value='1.5') ##########

#     run_stage = LaunchConfiguration('run_stage')
#     run_stage_arg = DeclareLaunchArgument('run_stage', default_value='false')


#     return LaunchDescription([
#         is_pure_rotation_arg, max_v_arg, max_w_arg,run_stage_arg,
#         Node(
#             package='robot_manager', executable='fms_single_ctrl_node', name='robot_manager1', output='both',
#             parameters=[{'robot_name': 'carter1',  'is_pure_rotation': is_pure_rotation, 'max_v': max_v, 'max_w': max_w, 'run_stage': run_stage}],
#         ),
#         Node(
#             package='robot_manager', executable='fms_single_ctrl_node', name='robot_manager2', output='both',
#             parameters=[{'robot_name': 'carter2',  'is_pure_rotation': is_pure_rotation, 'max_v': max_v, 'max_w': max_w, 'run_stage': run_stage}],
#         ),
#         Node(
#             package='robot_manager', executable='fms_single_ctrl_node', name='robot_manager3', output='both',
#             parameters=[{'robot_name': 'carter3',  'is_pure_rotation': is_pure_rotation, 'max_v': max_v, 'max_w': max_w, 'run_stage': run_stage}],
#         ),
#         Node(
#             package='robot_manager', executable='fms_single_ctrl_node', name='robot_manager4', output='both',
#             parameters=[{'robot_name': 'carter4',  'is_pure_rotation': is_pure_rotation, 'max_v': max_v, 'max_w': max_w, 'run_stage': run_stage}],
#         ),
#         Node(
#             package='robot_manager', executable='fms_single_ctrl_node', name='robot_manager5', output='both',
#             parameters=[{'robot_name': 'carter5',  'is_pure_rotation': is_pure_rotation, 'max_v': max_v, 'max_w': max_w, 'run_stage': run_stage}],
#         ),
#         Node(
#             package='robot_manager', executable='fms_single_ctrl_node', name='robot_manager6', output='both',
#             parameters=[{'robot_name': 'carter6',  'is_pure_rotation': is_pure_rotation, 'max_v': max_v, 'max_w': max_w, 'run_stage': run_stage}],
#         ),
#         Node(
#             package='robot_manager', executable='fms_single_ctrl_node', name='robot_manager7', output='both',
#             parameters=[{'robot_name': 'carter7',  'is_pure_rotation': is_pure_rotation, 'max_v': max_v, 'max_w': max_w, 'run_stage': run_stage}],
#         ),
#         Node(
#             package='robot_manager', executable='fms_single_ctrl_node', name='robot_manager8', output='both',
#             parameters=[{'robot_name': 'carter8',  'is_pure_rotation': is_pure_r5otation, 'max_v': max_v, 'max_w': max_w, 'run_stage': run_stage}],
#         ),
#         Node(
#             package='robot_manager', executable='fms_single_ctrl_node', name='robot_manager9', output='both',
#             parameters=[{'robot_name': 'carter9',  'is_pure_rotation': is_pure_rotation, 'max_v': max_v, 'max_w': max_w, 'run_stage': run_stage}],
#         ),
#         Node(
#             package='robot_manager', executable='fms_single_ctrl_node', name='robot_manager10', output='both',
#             parameters=[{'robot_name': 'carter10', 'is_pure_rotation': is_pure_rotation, 'max_v': max_v, 'max_w': max_w, 'run_stage': run_stage}],
#         ),
#         Node(
#             package='robot_manager', executable='fms_single_ctrl_node', name='robot_manager11', output='both',
#             parameters=[{'robot_name': 'carter11', 'is_pure_rotation': is_pure_rotation, 'max_v': max_v, 'max_w': max_w, 'run_stage': run_stage}],
#         ),
#         Node(
#             package='robot_manager', executable='fms_single_ctrl_node', name='robot_manager12', output='both',
#             parameters=[{'robot_name': 'carter12', 'is_pure_rotation': is_pure_rotation, 'max_v': max_v, 'max_w': max_w, 'run_stage': run_stage}],
#         ),
#         Node(
#             package='robot_manager', executable='fms_single_ctrl_node', name='robot_manager13', output='both',
#             parameters=[{'robot_name': 'carter13', 'is_pure_rotation': is_pure_rotation, 'max_v': max_v, 'max_w': max_w, 'run_stage': run_stage}],
#         ),
#         Node(
#             package='robot_manager', executable='fms_single_ctrl_node', name='robot_manager14', output='both',
#             parameters=[{'robot_name': 'carter14', 'is_pure_rotation': is_pure_rotation, 'max_v': max_v, 'max_w': max_w, 'run_stage': run_stage}],
#         ),
#         Node(
#             package='robot_manager', executable='fms_single_ctrl_node', name='robot_manager15', output='both',
#             parameters=[{'robot_name': 'carter15', 'is_pure_rotation': is_pure_rotation, 'max_v': max_v, 'max_w': max_w, 'run_stage': run_stage}],
#         ),
#         Node(
#             package='robot_manager', executable='fms_single_ctrl_node', name='robot_manager16', output='both',
#             parameters=[{'robot_name': 'carter16', 'is_pure_rotation': is_pure_rotation, 'max_v': max_v, 'max_w': max_w, 'run_stage': run_stage}],
#         ),
#         Node(
#             package='robot_manager', executable='fms_single_ctrl_node', name='robot_manager17', output='both',
#             parameters=[{'robot_name': 'carter17', 'is_pure_rotation': is_pure_rotation, 'max_v': max_v, 'max_w': max_w, 'run_stage': run_stage}],
#         ),
#         Node(
#             package='robot_manager', executable='fms_single_ctrl_node', name='robot_manager18', output='both',
#             parameters=[{'robot_name': 'carter18', 'is_pure_rotation': is_pure_rotation, 'max_v': max_v, 'max_w': max_w, 'run_stage': run_stage}],
#         ),
#         Node(
#             package='robot_manager', executable='fms_single_ctrl_node', name='robot_manager19', output='both',
#             parameters=[{'robot_name': 'carter19', 'is_pure_rotation': is_pure_rotation, 'max_v': max_v, 'max_w': max_w, 'run_stage': run_stage}],
#         ),
#         Node(
#             package='robot_manager', executable='fms_single_ctrl_node', name='robot_manager20', output='both',
#             parameters=[{'robot_name': 'carter20', 'is_pure_rotation': is_pure_rotation, 'max_v': max_v, 'max_w': max_w, 'run_stage': run_stage}],
#         ),
#         Node(
#             package='robot_manager', executable='fms_single_ctrl_node', name='robot_manager21', output='both',
#             parameters=[{'robot_name': 'carter21', 'is_pure_rotation': is_pure_rotation, 'max_v': max_v, 'max_w': max_w, 'run_stage': run_stage}],
#         ),
#         Node(
#             package='robot_manager', executable='fms_single_ctrl_node', name='robot_manager22', output='both',
#             parameters=[{'robot_name': 'carter22', 'is_pure_rotation': is_pure_rotation, 'max_v': max_v, 'max_w': max_w, 'run_stage': run_stage}],
#         ),
#         Node(
#             package='robot_manager', executable='fms_single_ctrl_node', name='robot_manager23', output='both',
#             parameters=[{'robot_name': 'carter23', 'is_pure_rotation': is_pure_r5obot_manager24', output='both',
#             parameters=[{'robot_name': 'carter24', 'is_pure_rotation': is_pure_rotation, 'max_v': max_v, 'max_w': max_w, 'run_stage': run_stage}],
#         ),
#         Node(
#             package='robot_manager', executable='fms_single_ctrl_node', name='robot_manager25', output='both',
#             parameters=[{'robot_name': 'carter25', 'is_pure_rotation': is_pure_rotation, 'max_v': max_v, 'max_w': max_w, 'run_stage': run_stage}],
#         ),
#         Node(
#             package='robot_manager', executable='fms_single_ctrl_node', name='robot_manager26', output='both',
#             parameters=[{'robot_name': 'carter26', 'is_pure_rotation': is_pure_rotation, 'max_v': max_v, 'max_w': max_w, 'run_stage': run_stage}],
#         ),
#         Node(
#             package='robot_manager', executable='fms_single_ctrl_node', name='robot_manager27', output='both',
#             parameters=[{'robot_name': 'carter27', 'is_pure_rotation': is_pure_rotation, 'max_v': max_v, 'max_w': max_w, 'run_stage': run_stage}],
#         ),
#         Node(
#             package='robot_manager', executable='fms_single_ctrl_node', name='robot_manager28', output='both',
#             parameters=[{'robot_name': 'carter28', 'is_pure_rotation': is_pure_rotation, 'max_v': max_v, 'max_w': max_w, 'run_stage': run_stage}],
#         ),
#         Node(
#             package='robot_manager', executable='fms_single_ctrl_node', name='robot_manager29', output='both',
#             parameters=[{'robot_name': 'carter29', 'is_pure_rotation': is_pure_rotation, 'max_v': max_v, 'max_w': max_w, 'run_stage': run_stage}],
#         ),
#         Node(
#             package='robot_manager', executable='fms_single_ctrl_node', name='robot_manager30', output='both',
#             parameters=[{'robot_name': 'carter30', 'is_pure_rotation': is_pure_rotation, 'max_v': max_v, 'max_w': max_w, 'run_stage': run_stage}],
#         ),
#         Node(
#             package='robot_manager', executable='fms_single_ctrl_node', name='robot_manager31', output='both',
#             parameters=[{'robot_name': 'carter31', 'is_pure_rotation': is_pure_rotation, 'max_v': max_v, 'max_w': max_w, 'run_stage': run_stage}],
#         ),
#         Node(
#             package='robot_manager', executable='fms_single_ctrl_node', name='robot_manager32', output='both',
#             parameters=[{'robot_name': 'carter32', 'is_pure_rotation': is_pure_rotation, 'max_v': max_v, 'max_w': max_w, 'run_stage': run_stage}],
#         ),
#         Node(
#             package='robot_manager', executable='fms_single_ctrl_node', name='robot_manager33', output='both',
#             parameters=[{'robot_name': 'carter33', 'is_pure_rotation': is_pure_rotation, 'max_v': max_v, 'max_w': max_w, 'run_stage': run_stage}],
#         ),
#         Node(
#             package='robot_manager', executable='fms_single_ctrl_node', name='robot_manager34', output='both',
#             parameters=[{'robot_name': 'carter34', 'is_pure_rotation': is_pure_rotation, 'max_v': max_v, 'max_w': max_w, 'run_stage': run_stage}],
#         ),
#         Node(
#             package='robot_manager', executable='fms_single_ctrl_node', name='robot_manager35', output='both',
#             parameters=[{'robot_name': 'carter35', 'is_pure_rotation': is_pure_rotation, 'max_v': max_v, 'max_w': max_w, 'run_stage': run_stage}],
#         ),
#         Node(
#             package='robot_manager', executable='fms_single_ctrl_node', name='robot_manager36', output='both',
#             parameters=[{'robot_name': 'carter36', 'is_pure_rotation': is_pure_rotation, 'max_v': max_v, 'max_w': max_w, 'run_stage': run_stage}],
#         ),
#         Node(
#             package='robot_manager', executable='fms_single_ctrl_node', name='robot_manager37', output='both',
#             parameters=[{'robot_name': 'carter37', 'is_pure_rotation': is_pure_rotation, 'max_v': max_v, 'max_w': max_w, 'run_stage': run_stage}],
#         ),
#         Node(
#             package='robot_manager', executable='fms_single_ctrl_node', name='robot_manager38', output='both',
#             parameters=[{'robot_name': 'carter38', 'is_pure_rotation': is_pure_rotation, 'max_v': max_v, 'max_w': max_w, 'run_stage': run_stage}],
#         ),
#         Node(
#             package='robot_manager', executable='fms_single_ctrl_node', name='robot_manager39', output='both',
#             parameters=[{'robot_name': 'carter39', 'is_pure_rotation': is_pure_rotation, 'max_v': max_v, 'max_w': max_w, 'run_stage': run_stage}],
#         ),
#         Node(
#             package='robot_manager', executable='fms_single_ctrl_node', name='robot_manager40', output='both',
#             parameters=[{'robot_name': 'carter40', 'is_pure_rotation': is_pure_rotation, 'max_v': max_v, 'max_w': max_w, 'run_stage': run_stage}],
#         ),
#         Node(
#             package='robot_manager', executable='fms_single_ctrl_node', name='robot_manager41', output='both',
#             parameters=[{'robot_name': 'carter41', 'is_pure_rotation': is_pure_rotation, 'max_v': max_v, 'max_w': max_w, 'run_stage': run_stage}],
#         ),
#         Node(
#             package='robot_manager', executable='fms_single_ctrl_node', name='robot_manager42', output='both',
#             parameters=[{'robot_name': 'carter42', 'is_pure_rotation': is_pure_rotation, 'max_v': max_v, 'max_w': max_w, 'run_stage': run_stage}],
#         ),
#         Node(5
#             package='robot_manager', executable='fms_single_ctrl_node', name='robot_manager43', output='both',
#             parameters=[{'robot_name': 'carter43', 'is_pure_rotation': is_pure_rotation, 'max_v': max_v, 'max_w': max_w, 'run_stage': run_stage}],
#         ),
#         Node(
#             package='robot_manager', executable='fms_single_ctrl_node', name='robot_manager44', output='both',
#             parameters=[{'robot_name': 'carter44', 'is_pure_rotation': is_pure_rotation, 'max_v': max_v, 'max_w': max_w, 'run_stage': run_stage}],
#         ),
#         Node(
#             package='robot_manager', executable='fms_single_ctrl_node', name='robot_manager45', output='both',
#             parameters=[{'robot_name': 'carter45', 'is_pure_rotation': is_pure_rotation, 'max_v': max_v, 'max_w': max_w, 'run_stage': run_stage}],
#         ),
#         Node(
#             package='robot_manager', executable='fms_single_ctrl_node', name='robot_manager46', output='both',
#             parameters=[{'robot_name': 'carter46', 'is_pure_rotation': is_pure_rotation, 'max_v': max_v, 'max_w': max_w, 'run_stage': run_stage}],
#         ),
#         Node(
#             package='robot_manager', executable='fms_single_ctrl_node', name='robot_manager47', output='both',
#             parameters=[{'robot_name': 'carter47', 'is_pure_rotation': is_pure_rotation, 'max_v': max_v, 'max_w': max_w, 'run_stage': run_stage}],
#         ),
#         Node(
#             package='robot_manager', executable='fms_single_ctrl_node', name='robot_manager48', output='both',
#             parameters=[{'robot_name': 'carter48', 'is_pure_rotation': is_pure_rotation, 'max_v': max_v, 'max_w': max_w, 'run_stage': run_stage}],
#         ),
#         Node(
#             package='robot_manager', executable='fms_single_ctrl_node', name='robot_manager49', output='both',
#             parameters=[{'robot_name': 'carter49', 'is_pure_rotation': is_pure_rotation, 'max_v': max_v, 'max_w': max_w, 'run_stage': run_stage}],
#         ),
#         Node(
#             package='robot_manager', executable='fms_single_ctrl_node', name='robot_manager50', output='both',
#             parameters=[{'robot_name': 'carter50', 'is_pure_rotation': is_pure_rotation, 'max_v': max_v, 'max_w': max_w, 'run_stage': run_stage}],
#         ),
#         # Node(
#         #     package='robot_manager', executable='fms_single_ctrl_node', name='robot_manager51', output='both',
#         #     parameters=[{'robot_name': 'carter51', 'is_pure_rotation': is_pure_rotation, 'max_v': max_v, 'max_w': max_w, 'run_stage': run_stage}],
#         # ),
#         # Node(
#         #     package='robot_manager', executable='fms_single_ctrl_node', name='robot_manager52', output='both',
#         #     parameters=[{'robot_name': 'carter52', 'is_pure_rotation': is_pure_rotation, 'max_v': max_v, 'max_w': max_w, 'run_stage': run_stage}],
#         # ),
#         # Node(
#         #     package='robot_manager', executable='fms_single_ctrl_node', name='robot_manager53', output='both',
#         #     parameters=[{'robot_name': 'carter53', 'is_pure_rotation': is_pure_rotation, 'max_v': max_v, 'max_w': max_w, 'run_stage': run_stage}],
#         # ),
#         # Node(
#         #     package='robot_manager', executable='fms_single_ctrl_node', name='robot_manager54', output='both',
#         #     parameters=[{'robot_name': 'carter54', 'is_pure_rotation': is_pure_rotation, 'max_v': max_v, 'max_w': max_w, 'run_stage': run_stage}],
#         # ),
#         # Node(
#         #     package='robot_manager', executable='fms_single_ctrl_node', name='robot_manager55', output='both',
#         #     parameters=[{'robot_name': 'carter55', 'is_pure_rotation': is_pure_rotation, 'max_v': max_v, 'max_w': max_w, 'run_stage': run_stage}],
#         # ),
#         # Node(
#         #     package='robot_manager', executable='fms_single_ctrl_node', name='robot_manager56', output='both',
#         #     parameters=[{'robot_name': 'carter56', 'is_pure_rotation': is_pure_rotation, 'max_v': max_v, 'max_w': max_w, 'run_stage': run_stage}],
#         # ),
#         # Node(5
#         #     package='robot_manager', executable='fms_single_ctrl_node', name='robot_manager57', output='both',
#         #     parameters=[{'robot_name': 'carter57', 'is_pure_rotation': is_pure_rotation, 'max_v': max_v, 'max_w': max_w, 'run_stage': run_stage}],
#         # ),
#         # Node(
#         #     package='robot_manager', executable='fms_single_ctrl_node', name='robot_manager58', output='both',
#         #     parameters=[{'robot_name': 'carter58', 'is_pure_rotation': is_pure_rotation, 'max_v': max_v, 'max_w': max_w, 'run_stage': run_stage}],
#         # ),
#         # Node(
#         #     package='robot_manager', executable='fms_single_ctrl_node', name='robot_manager59', output='both',
#         #     parameters=[{'robot_name': 'carter59', 'is_pure_rotation': is_pure_rotation, 'max_v': max_v, 'max_w': max_w, 'run_stage': run_stage}],
#         # ),
#         # Node(
#         #     package='robot_manager', executable='fms_single_ctrl_node', name='robot_manager60', output='both',
#         #     parameters=[{'robot_name': 'carter60', 'is_pure_rotation': is_pure_rotation, 'max_v': max_v, 'max_w': max_w, 'run_stage': run_stage}],
#         # ),
#         # Node(
#         #     package='robot_manager', executable='fms_single_ctrl_node', name='robot_manager61', output='both',
#         #     parameters=[{'robot_name': 'carter61', 'is_pure_rotation': is_pure_rotation, 'max_v': max_v, 'max_w': max_w, 'run_stage': run_stage}],
#         # ),
#         # Node(
#         #     package='robot_manager', executable='fms_single_ctrl_node', name='robot_manager62', output='both',
#         #     parameters=[{'robot_name': 'carter62', 'is_pure_rotation': is_pure_rotation, 'max_v': max_v, 'max_w': max_w, 'run_stage': run_stage}],
#         # ),
#         # Node(
#         #     package='robot_manager', executable='fms_single_ctrl_node', name='robot_manager63', output='both',
#         #     parameters=[{'robot_name': 'carter63', 'is_pure_rotation': is_pure_rotation, 'max_v': max_v, 'max_w': max_w, 'run_stage': run_stage}],
#         # ),
#         # Node(
#         #     package='robot_manager', executable='fms_single_ctrl_node', name='robot_manager64', output='both',
#         #     parameters=[{'robot_name': 'carter64', 'is_pure_rotation': is_pure_rotation, 'max_v': max_v, 'max_w': max_w, 'run_stage': run_stage}],
#         # ),
#         # Node(
#         #     package='robot_manager', executable='fms_single_ctrl_node', name='robot_manager65', output='both',
#         #     parameters=[{'robot_name': 'carter65', 'is_pure_rotation': is_pure_rotation, 'max_v': max_v, 'max_w': max_w, 'run_stage': run_stage}],
#         # ),
#         # Node(
#         #     package='robot_manager', executable='fms_single_ctrl_node', name='robot_manager66', output='both',
#         #     parameters=[{'robot_name': 'carter66', 'is_pure_rotation': is_pure_rotation, 'max_v': max_v, 'max_w': max_w, 'run_stage': run_stage}],
#         # ),
#         # Node(
#         #     package='robot_manager', executable='fms_single_ctrl_node', name='robot_manager67', output='both',
#         #     parameters=[{'robot_name': 'carter67', 'is_pure_rotation': is_pure_rotation, 'max_v': max_v, 'max_w': max_w, 'run_stage': run_stage}],
#         # ),
#         # Node(
#         #     package='robot_manager', executable='fms_single_ctrl_node', name='robot_manager68', output='both',
#         #     parameters=[{'robot_name': 'carter68', 'is_pure_rotation': is_pure_rotation, 'max_v': max_v, 'max_w': max_w, 'run_stage': run_stage}],
#         # ),
#         # Node(
#         #     package='robot_manager', executable='fms_single_ctrl_node', name='robot_manager69', output='both',
#         #     parameters=[{'robot_name': 'carter69', 'is_pure_rotation': is_pure_rotation, 'max_v': max_v, 'max_w': max_w, 'run_stage': run_stage}],
#         # ),
#         # Node(
#         #     package='robot_manager', executable='fms_single_ctrl_node', name='robot_manager70', output='both',
#         #     parameters=[{'robot_name': 'carter70', 'is_pure_rotation': is_pure_rotation, 'max_v': max_v, 'max_w': max_w, 'run_stage': run_stage}],
#         # ),
#         # Node(
#         #     package='robot_manager', executable='fms_single_ctrl_node', name='robot_manager71', output='both',
#         #     parameters=[{'robot_name': 'carter71', 'is_pure_rotation5': is_pure_rotation, 'max_v': max_v, 'max_w': max_w, 'run_stage': run_stage}],
#         # ),
#         # Node(
#         #     package='robot_manager', executable='fms_single_ctrl_node', name='robot_manager72', output='both',
#         #     parameters=[{'robot_name': 'carter72', 'is_pure_rotation': is_pure_rotation, 'max_v': max_v, 'max_w': max_w, 'run_stage': run_stage}],
#         # ),
#         # Node(
#         #     package='robot_manager', executable='fms_single_ctrl_node', name='robot_manager73', output='both',
#         #     parameters=[{'robot_name': 'carter73', 'is_pure_rotation': is_pure_rotation, 'max_v': max_v, 'max_w': max_w, 'run_stage': run_stage}],
#         # ),
#         # Node(
#         #     package='robot_manager', executable='fms_single_ctrl_node', name='robot_manager74', output='both',
#         #     parameters=[{'robot_name': 'carter74', 'is_pure_rotation': is_pure_rotation, 'max_v': max_v, 'max_w': max_w, 'run_stage': run_stage}],
#         # ),
#         # Node(
#         #     package='robot_manager', executable='fms_single_ctrl_node', name='robot_manager75', output='both',
#         #     parameters=[{'robot_name': 'carter75', 'is_pure_rotation': is_pure_rotation, 'max_v': max_v, 'max_w': max_w, 'run_stage': run_stage}],
#         # ),
#         # Node(
#         #     package='robot_manager', executable='fms_single_ctrl_node', name='robot_manager76', output='both',
#         #     parameters=[{'robot_name': 'carter76', 'is_pure_rotation': is_pure_rotation, 'max_v': max_v, 'max_w': max_w, 'run_stage': run_stage}],
#         # ),
#         # Node(
#         #     package='robot_manager', executable='fms_single_ctrl_node', name='robot_manager77', output='both',
#         #     parameters=[{'robot_name': 'carter77', 'is_pure_rotation': is_pure_rotation, 'max_v': max_v, 'max_w': max_w, 'run_stage': run_stage}],
#         # ),
#         # Node(
#         #     package='robot_manager', executable='fms_single_ctrl_node', name='robot_manager78', output='both',
#         #     parameters=[{'robot_name': 'carter78', 'is_pure_rotation': is_pure_rotation, 'max_v': max_v, 'max_w': max_w, 'run_stage': run_stage}],
#         # ),
#     ])
#         Node(
#             package='robot_manager', executable='fms_single_ctrl_node', name='robot_manager44', output='both',
#             parameters=[{'robot_name': 'carter44', 'is_pure_rotation': is_pure_rotation, 'max_v': max_v, 'max_w': max_w, 'run_stage': run_stage}],
#         ),
#         Node(
#             package='robot_manager', executable='fms_single_ctrl_node', name='robot_manager45', output='both',
#             parameters=[{'robot_name': 'carter45', 'is_pure_rotation': is_pure_rotation, 'max_v': max_v, 'max_w': max_w, 'run_stage': run_stage}],
#         ),
#         Node(
#             package='robot_manager', executable='fms_single_ctrl_node', name='robot_manager46', output='both',
#             parameters=[{'robot_name': 'carter46', 'is_pure_rotation': is_pure_rotation, 'max_v': max_v, 'max_w': max_w, 'run_stage': run_stage}],
#         ),
#         Node(
#             package='robot_manager', executable='fms_single_ctrl_node', name='robot_manager47', output='both',
#             parameters=[{'robot_name': 'carter47', 'is_pure_rotation': is_pure_rotation, 'max_v': max_v, 'max_w': max_w, 'run_stage': run_stage}],
#         ),
#         Node(
#             package='robot_manager', executable='fms_single_ctrl_node', name='robot_manager48', output='both',
#             parameters=[{'robot_name': 'carter48', 'is_pure_rotation': is_pure_rotation, 'max_v': max_v, 'max_w': max_w, 'run_stage': run_stage}],
#         ),
#         Node(
#             package='robot_manager', executable='fms_single_ctrl_node', name='robot_manager49', output='both',
#             parameters=[{'robot_name': 'carter49', 'is_pure_rotation': is_pure_rotation, 'max_v': max_v, 'max_w': max_w, 'run_stage': run_stage}],
#         ),
#         Node(
#             package='robot_manager', executable='fms_single_ctrl_node', name='robot_manager50', output='both',
#             parameters=[{'robot_name': 'carter50', 'is_pure_rotation': is_pure_rotation, 'max_v': max_v, 'max_w': max_w, 'run_stage': run_stage}],
#         ),
#         # Node(
#         #     package='robot_manager', executable='fms_single_ctrl_node', name='robot_manager51', output='both',
#         #     parameters=[{'robot_name': 'carter51', 'is_pure_rotation': is_pure_rotation, 'max_v': max_v, 'max_w': max_w, 'run_stage': run_stage}],
#         # ),
#         # Node(
#         #     package='robot_manager', executable='fms_single_ctrl_node', name='robot_manager52', output='both',
#         #     parameters=[{'robot_name': 'carter52', 'is_pure_rotation': is_pure_rotation, 'max_v': max_v, 'max_w': max_w, 'run_stage': run_stage}],
#         # ),
#         # Node(
#         #     package='robot_manager', executable='fms_single_ctrl_node', name='robot_manager53', output='both',
#         #     parameters=[{'robot_name': 'carter53', 'is_pure_rotation': is_pure_rotation, 'max_v': max_v, 'max_w': max_w, 'run_stage': run_stage}],
#         # ),
#         # Node(
#         #     package='robot_manager', executable='fms_single_ctrl_node', name='robot_manager54', output='both',
#         #     parameters=[{'robot_name': 'carter54', 'is_pure_rotation': is_pure_rotation, 'max_v': max_v, 'max_w': max_w, 'run_stage': run_stage}],
#         # ),
#         # Node(
#         #     package='robot_manager', executable='fms_single_ctrl_node', name='robot_manager55', output='both',
#         #     parameters=[{'robot_name': 'carter55', 'is_pure_rotation': is_pure_rotation, 'max_v': max_v, 'max_w': max_w, 'run_stage': run_stage}],
#         # ),
#         # Node(
#         #     package='robot_manager', executable='fms_single_ctrl_node', name='robot_manager56', output='both',
#         #     parameters=[{'robot_name': 'carter56', 'is_pure_rotation': is_pure_rotation, 'max_v': max_v, 'max_w': max_w, 'run_stage': run_stage}],
#         # ),
#         # Node(
#         #     package='robot_manager', executable='fms_single_ctrl_node', name='robot_manager57', output='both',
#         #     parameters=[{'robot_name': 'carter57', 'is_pure_rotation': is_pure_rotation, 'max_v': max_v, 'max_w': max_w, 'run_stage': run_stage}],
#         # ),
#         # Node(
#         #     package='robot_manager', executable='fms_single_ctrl_node', name='robot_manager58', output='both',
#         #     parameters=[{'robot_name': 'carter58', 'is_pure_rotation': is_pure_rotation, 'max_v': max_v, 'max_w': max_w, 'run_stage': run_stage}],
#         # ),
#         # Node(
#         #     package='robot_manager', executable='fms_single_ctrl_node', name='robot_manager59', output='both',
#         #     parameters=[{'robot_name': 'carter59', 'is_pure_rotation': is_pure_rotation, 'max_v': max_v, 'max_w': max_w, 'run_stage': run_stage}],
#         # ),
#         # Node(
#         #     package='robot_manager', executable='fms_single_ctrl_node', name='robot_manager60', output='both',
#         #     parameters=[{'robot_name': 'carter60', 'is_pure_rotation': is_pure_rotation, 'max_v': max_v, 'max_w': max_w, 'run_stage': run_stage}],
#         # ),
#         # Node(
#         #     package='robot_manager', executable='fms_single_ctrl_node', name='robot_manager61', output='both',
#         #     parameters=[{'robot_name': 'carter61', 'is_pure_rotation': is_pure_rotation, 'max_v': max_v, 'max_w': max_w, 'run_stage': run_stage}],
#         # ),
#         # Node(
#         #     package='robot_manager', executable='fms_single_ctrl_node', name='robot_manager62', output='both',
#         #     parameters=[{'robot_name': 'carter62', 'is_pure_rotation': is_pure_rotation, 'max_v': max_v, 'max_w': max_w, 'run_stage': run_stage}],
#         # ),
#         # Node(
#         #     package='robot_manager', executable='fms_single_ctrl_node', name='robot_manager63', output='both',
#         #     parameters=[{'robot_name': 'carter63', 'is_pure_rotation': is_pure_rotation, 'max_v': max_v, 'max_w': max_w, 'run_stage': run_stage}],
#         # ),
#         # Node(
#         #     package='robot_manager', executable='fms_single_ctrl_node', name='robot_manager64', output='both',
#         #     parameters=[{'robot_name': 'carter64', 'is_pure_rotation': is_pure_rotation, 'max_v': max_v, 'max_w': max_w, 'run_stage': run_stage}],
#         # ),
#         # Node(
#         #     package='robot_manager', executable='fms_single_ctrl_node', name='robot_manager65', output='both',
#         #     parameters=[{'robot_name': 'carter65', 'is_pure_rotation': is_pure_rotation, 'max_v': max_v, 'max_w': max_w, 'run_stage': run_stage}],
#         # ),
#         # Node(
#         #     package='robot_manager', executable='fms_single_ctrl_node', name='robot_manager66', output='both',
#         #     parameters=[{'robot_name': 'carter66', 'is_pure_rotation': is_pure_rotation, 'max_v': max_v, 'max_w': max_w, 'run_stage': run_stage}],
#         # ),
#         # Node(
#         #     package='robot_manager', executable='fms_single_ctrl_node', name='robot_manager67', output='both',
#         #     parameters=[{'robot_name': 'carter67', 'is_pure_rotation': is_pure_rotation, 'max_v': max_v, 'max_w': max_w, 'run_stage': run_stage}],
#         # ),
#         # Node(
#         #     package='robot_manager', executable='fms_single_ctrl_node', name='robot_manager68', output='both',
#         #     parameters=[{'robot_name': 'carter68', 'is_pure_rotation': is_pure_rotation, 'max_v': max_v, 'max_w': max_w, 'run_stage': run_stage}],
#         # ),
#         # Node(
#         #     package='robot_manager', executable='fms_single_ctrl_node', name='robot_manager69', output='both',
#         #     parameters=[{'robot_name': 'carter69', 'is_pure_rotatio5n': is_pure_rotation, 'max_v': max_v, 'max_w': max_w, 'run_stage': run_stage}],
#         # ),
#         # Node(
#         #     package='robot_manager', executable='fms_single_ctrl_node', name='robot_manager70', output='both',
#         #     parameters=[{'robot_name': 'carter70', 'is_pure_rotation': is_pure_rotation, 'max_v': max_v, 'max_w': max_w, 'run_stage': run_stage}],
#         # ),
#         # Node(
#         #     package='robot_manager', executable='fms_single_ctrl_node', name='robot_manager71', output='both',
#         #     parameters=[{'robot_name': 'carter71', 'is_pure_rotation': is_pure_rotation, 'max_v': max_v, 'max_w': max_w, 'run_stage': run_stage}],
#         # ),
#         # Node(
#         #     package='robot_manager', executable='fms_single_ctrl_node', name='robot_manager72', output='both',
#         #     parameters=[{'robot_name': 'carter72', 'is_pure_rotation': is_pure_rotation, 'max_v': max_v, 'max_w': max_w, 'run_stage': run_stage}],
#         # ),
#         # Node(
#         #     package='robot_manager', executable='fms_single_ctrl_node', name='robot_manager73', output='both',
#         #     parameters=[{'robot_name': 'carter73', 'is_pure_rotation': is_pure_rotation, 'max_v': max_v, 'max_w': max_w, 'run_stage': run_stage}],
#         # ),
#         # Node(
#         #     package='robot_manager', executable='fms_single_ctrl_node', name='robot_manager74', output='both',
#         #     parameters=[{'robot_name': 'carter74', 'is_pure_rotation': is_pure_rotation, 'max_v': max_v, 'max_w': max_w, 'run_stage': run_stage}],
#         # ),
#         # Node(
#         #     package='robot_manager', executable='fms_single_ctrl_node', name='robot_manager75', output='both',
#         #     parameters=[{'robot_name': 'carter75', 'is_pure_rotation': is_pure_rotation, 'max_v': max_v, 'max_w': max_w, 'run_stage': run_stage}],
#         # ),
#         # Node(
#         #     package='robot_manager', executable='fms_single_ctrl_node', name='robot_manager76', output='both',
#         #     parameters=[{'robot_name': 'carter76', 'is_pure_rotation': is_pure_rotation, 'max_v': max_v, 'max_w': max_w, 'run_stage': run_stage}],
#         # ),
#         # Node(
#         #     package='robot_manager', executable='fms_single_ctrl_node', name='robot_manager77', output='both',
#         #     parameters=[{'robot_name': 'carter77', 'is_pure_rotation': is_pure_rotation, 'max_v': max_v, 'max_w': max_w, 'run_stage': run_stage}],
#         # ),
#         # Node(
#         #     package='robot_manager', executable='fms_single_ctrl_node', name='robot_manager78', output='both',
#         #     parameters=[{'robot_name': 'carter78', 'is_pure_rotation': is_pure_rotation, 'max_v': max_v, 'max_w': max_w, 'run_stage': run_stage}],
#         # ),
#     ])

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node

def launch_setup(context):
    nr = int(context.launch_configurations['nr_of_robots'])
    is_pure_rotation = LaunchConfiguration('is_pure_rotation')
    max_v            = LaunchConfiguration('max_v')
    max_w            = LaunchConfiguration('max_w')
    run_stage        = LaunchConfiguration('run_stage')

    common_params = {
        'is_pure_rotation': is_pure_rotation,
        'max_v': max_v,
        'max_w': max_w,
        'run_stage': run_stage,
    }

    nodes = []
    for i in range(1, nr + 1):
        robot_name = f'carter{i}'
        node_name  = f'robot_manager{i}'
        nodes.append(
            Node(
                package='robot_manager',
                executable='fms_single_ctrl_node',
                name=node_name,
                output='both',
                parameters=[{'robot_name': robot_name, **common_params}],
            )
        )
    return nodes

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('nr_of_robots',     default_value='8'),
        DeclareLaunchArgument('is_pure_rotation', default_value='true'),
        DeclareLaunchArgument('max_v',            default_value='1.0'),
        DeclareLaunchArgument('max_w',            default_value='0.5'),
        DeclareLaunchArgument('robot_model',      default_value='robot_model_0'),
        DeclareLaunchArgument('param_file',       default_value='1.5'),
        DeclareLaunchArgument('run_stage',        default_value='false'),
        OpaqueFunction(function=launch_setup),
    ])

