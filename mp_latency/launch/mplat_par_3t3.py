# This is a fan-out example of: 1 HEAD --> 3 THRU --> 3 TAIL nodes.
from launch import LaunchDescription
from launch_ros.actions import Node
import os

# Set test values here (will need to get cmdLine values or somesuch)
test_duration = '60'    # duration of test run, in seconds
pub_frequency = '10'    # pubs per second from HEAD node (float32 value)
rel_type = 'REL'         # reliability type (BE=best effort, anything else is reliable.)
my_node = 0             # starting node ID
data_type_suffix = '1kb'    # edit this to change data size in test
exe_head = 'ipchead_' + data_type_suffix
exe_thru = 'ipcthru_' + data_type_suffix
exe_tail = 'ipctail_' + data_type_suffix
rmw_type = os.environ.get('RMW_IMPLEMENTATION') # RMW type used for test (for TAIL node results)

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='mp_latency',
            namespace='ipc_lat',
            executable=exe_tail,
            output='screen',
            # TAIL args: testDur, REL, pubFreq, totalNodes, fromTopic, rmwType, myCfgName
            arguments=[test_duration, rel_type, pub_frequency, '3', 'pt_profile_topic_1_2', rmw_type, 'h-3p-t1'],
            name='tail1',
        ),
        Node(
            package='mp_latency',
            namespace='ipc_lat',
            executable=exe_tail,
            output='screen',
            # TAIL args: testDur, REL, pubFreq, totalNodes, fromTopic, rmwType, myCfgName
            arguments=[test_duration, rel_type, pub_frequency, '3', 'pt_profile_topic_1_3', rmw_type, 'h-3p-t2'],
            name='tail2',
        ),
        Node(
            package='mp_latency',
            namespace='ipc_lat',
            executable=exe_tail,
            output='screen',
            # TAIL args: testDur, REL, pubFreq, totalNodes, fromTopic, rmwType, myCfgName
            arguments=[test_duration, rel_type, pub_frequency, '3', 'pt_profile_topic_1_4', rmw_type, 'h-3p-t3'],
            name='tail3',
        ),
        Node(
            package='mp_latency',
            namespace='ipc_lat',
            executable=exe_thru,
            output='screen',
            # TEST args: testDur, REL, myNodeId, fromTopic, toTopic
            arguments=[test_duration, rel_type, '3', 'pt_profile_topic_0_1', 'pt_profile_topic_1_4'],
            name='node3'
        ),
        Node(
            package='mp_latency',
            namespace='ipc_lat',
            executable=exe_thru,
            output='screen',
            # TEST args: testDur, REL, myNodeId, fromTopic, toTopic
            arguments=[test_duration, rel_type, '2', 'pt_profile_topic_0_1', 'pt_profile_topic_1_3'],
            name='node2'
        ),
        Node(
            package='mp_latency',
            namespace='ipc_lat',
            executable=exe_thru,
            output='screen',
            # TEST args: testDur, REL, myNodeId, fromTopic, toTopic
            arguments=[test_duration, rel_type, '1', 'pt_profile_topic_0_1', 'pt_profile_topic_1_2'],
            name='node1'
        ),
        Node(
            package='mp_latency',
            namespace='ipc_lat',
            executable=exe_head,
            output='screen',
            # HEAD args: testDur, REL, pubFreq, myNodeId, toTopic
            arguments=[test_duration, rel_type, pub_frequency, '0', 'pt_profile_topic_0_1'],
            name='head'
        )
    ])
