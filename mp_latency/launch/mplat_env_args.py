from launch import LaunchDescription
from launch_ros.actions import Node
import os

# using env vars to set parms for test, default values if not set
test_duration = '20'    # duration of test run, in seconds
if 'RTI_MPL_DURATION' in os.environ:
    test_duration = os.environ['RTI_MPL_DURATION']

pub_frequency = '10'    # pubs per second from HEAD node (float32 value)
if 'RTI_MPL_PUBFREQ' in os.environ:
    pub_frequency = os.environ['RTI_MPL_PUBFREQ']
    
rel_type = 'BE'         # reliability type (REL=reliable, anything else is best effort)
if 'RTI_MPL_RELY' in os.environ:
    rel_type = os.environ['RTI_MPL_RELY']

my_node = 0             # starting node ID
pub_time_modulo = '0'   # time-of-day modulus (in mS) to coordinate pub in TEST nodes (to be implemented)
rmw_type = os.environ.get('RMW_IMPLEMENTATION') # RMW type used for test (for TAIL node results)

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='mp_latency',
            namespace='ipc_lat',
            executable='ipchead',
            output='screen',
            # HEAD node args are: test_duration, pub_frequency, rel_type, my_node, to_node
            arguments=[test_duration, pub_frequency, rel_type, '0', '1'],
            name='head'
        ),
        Node(
            package='mp_latency',
            namespace='ipc_lat',
            executable='ipcnode',
            output='screen',
            # TEST node args are: test_duration, rel_type, from_node, my_node, to_node, pub_time_modulo
            arguments=[test_duration, rel_type, '0', '1', '2', pub_time_modulo],
            name='node1'
        ),
        Node(
            package='mp_latency',
            namespace='ipc_lat',
            executable='ipcnode',
            output='screen',
            # TEST node args are: test_duration, rel_type, from_node, my_node, to_node, pub_time_modulo
            arguments=[test_duration, rel_type, '1', '2', '3', pub_time_modulo],
            name='node2'
        ),
        Node(
            package='mp_latency',
            namespace='ipc_lat',
            executable='ipcnode',
            output='screen',
            # TEST node args are: test_duration, rel_type, from_node, my_node, to_node, pub_time_modulo
            arguments=[test_duration, rel_type, '2', '3', '4', pub_time_modulo],
            name='node3'
        ),
        Node(
            package='mp_latency',
            namespace='ipc_lat',
            executable='ipctail',
            output='screen',
            # TAIL node args are: test_duration, rel_type, from_node, my_node, rmw_type
            arguments=[test_duration, rel_type, '3', '4', rmw_type],
            name='tail',
        )
    ])
