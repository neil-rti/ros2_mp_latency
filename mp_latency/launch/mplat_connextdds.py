from launch import LaunchDescription
from launch_ros.actions import Node
import os

# Set test values here (will need to get cmdLine values or somesuch)
test_duration = '60'    # duration of test run, in seconds
pub_frequency = '30'    # pubs per second from HEAD node (float32 value)
rel_type = 'REL'         # reliability type (BE=best effort, anything else is reliable.)
my_node = 0             # starting node ID
rmw_type = os.environ.get('RMW_IMPLEMENTATION') # RMW type used for test (for TAIL node results)

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='mp_latency',
            namespace='ipc_lat',
            executable='ipctail',
            output='screen',
            # TAIL args: testDur, REL, pubFreq, totalNodes, fromTopic, rmwType, myCfgName
            arguments=[test_duration, rel_type, pub_frequency, '5', 'pt_profile_topic_3_4', rmw_type, 'h-5s-t'],
            name='tail',
        ),
        Node(
            package='mp_latency',
            namespace='ipc_lat',
            executable='ipcnode',
            output='screen',
            # TEST args: testDur, REL, myNodeId, fromTopic, toTopic
            arguments=[test_duration, rel_type, '3', 'pt_profile_topic_2_3', 'pt_profile_topic_3_4'],
            name='node3'
        ),
        Node(
            package='mp_latency',
            namespace='ipc_lat',
            executable='ipcnode',
            output='screen',
            # TEST args: testDur, REL, myNodeId, fromTopic, toTopic
            arguments=[test_duration, rel_type, '2', 'pt_profile_topic_1_2', 'pt_profile_topic_2_3'],
            name='node2'
        ),
        Node(
            package='mp_latency',
            namespace='ipc_lat',
            executable='ipcnode',
            output='screen',
            # TEST args: testDur, REL, myNodeId, fromTopic, toTopic
            arguments=[test_duration, rel_type, '1', 'pt_profile_topic_0_1', 'pt_profile_topic_1_2'],
            name='node1'
        ),
        Node(
            package='mp_latency',
            namespace='ipc_lat',
            executable='ipchead',
            output='screen',
            # HEAD args: testDur, REL, pubFreq, myNodeId, toTopic
            arguments=[test_duration, rel_type, pub_frequency, '0', 'pt_profile_topic_0_1'],
            name='head'
        )
    ])
