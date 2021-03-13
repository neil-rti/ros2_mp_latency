# This is a fan-out example of: 1 SOURCE --> 3 WORK --> 3 SINK nodes.
from launch import LaunchDescription
from launch_ros.actions import Node
import os

# Set test values here (will need to get cmdLine values or somesuch)
test_duration = '60'    # duration of test run, in seconds
pub_frequency = '10'    # pubs per second from SOURCE node (float32 value)
rel_type = 'REL'         # reliability type (BE=best effort, anything else is reliable.)
my_node = 0             # starting node ID
data_type_suffix = '1kb'    # edit this to change data size in test
exe_source = 'ipcsource_' + data_type_suffix
exe_work = 'ipcwork_' + data_type_suffix
exe_sink = 'ipcsink_' + data_type_suffix
rmw_type = os.environ.get('RMW_IMPLEMENTATION') # RMW type used for test (for SINK node results)

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='mp_latency',
            namespace='ipc_lat',
            executable=exe_sink,
            output='screen',
            # SINK args: testDur, REL, pubFreq, totalNodes, fromTopic, rmwType, myCfgName
            arguments=[test_duration, rel_type, pub_frequency, '3', 'pt_profile_topic_1_2', rmw_type, 'h-3p-t1'],
            name='sink1',
        ),
        Node(
            package='mp_latency',
            namespace='ipc_lat',
            executable=exe_sink,
            output='screen',
            # SINK args: testDur, REL, pubFreq, totalNodes, fromTopic, rmwType, myCfgName
            arguments=[test_duration, rel_type, pub_frequency, '3', 'pt_profile_topic_1_3', rmw_type, 'h-3p-t2'],
            name='sink2',
        ),
        Node(
            package='mp_latency',
            namespace='ipc_lat',
            executable=exe_sink,
            output='screen',
            # SINK args: testDur, REL, pubFreq, totalNodes, fromTopic, rmwType, myCfgName
            arguments=[test_duration, rel_type, pub_frequency, '3', 'pt_profile_topic_1_4', rmw_type, 'h-3p-t3'],
            name='sink3',
        ),
        Node(
            package='mp_latency',
            namespace='ipc_lat',
            executable=exe_work,
            output='screen',
            # WORK args: testDur, REL, myNodeId, fromTopic, toTopic
            arguments=[test_duration, rel_type, '3', 'pt_profile_topic_0_1', 'pt_profile_topic_1_4'],
            name='proc3'
        ),
        Node(
            package='mp_latency',
            namespace='ipc_lat',
            executable=exe_work,
            output='screen',
            # WORK args: testDur, REL, myNodeId, fromTopic, toTopic
            arguments=[test_duration, rel_type, '2', 'pt_profile_topic_0_1', 'pt_profile_topic_1_3'],
            name='proc2'
        ),
        Node(
            package='mp_latency',
            namespace='ipc_lat',
            executable=exe_work,
            output='screen',
            # WORK args: testDur, REL, myNodeId, fromTopic, toTopic
            arguments=[test_duration, rel_type, '1', 'pt_profile_topic_0_1', 'pt_profile_topic_1_2'],
            name='proc1'
        ),
        Node(
            package='mp_latency',
            namespace='ipc_lat',
            executable=exe_source,
            output='screen',
            # SOURCE args: testDur, REL, pubFreq, myNodeId, toTopic
            arguments=[test_duration, rel_type, pub_frequency, '0', 'pt_profile_topic_0_1'],
            name='source'
        )
    ])
