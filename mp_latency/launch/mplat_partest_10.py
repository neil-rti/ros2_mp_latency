from launch import LaunchDescription
from launch_ros.actions import Node
import os

# Set test values here (will need to get cmdLine values or somesuch)
test_duration = '60'    # duration of test run, in seconds
pub_frequency = '10'    # pubs per second from HEAD node (float32 value)
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
            arguments=[test_duration, rel_type, pub_frequency, '3', 'pt_profile_topic_1_2', rmw_type, 'h-3p-t1'],
            name='tail1',
        ),
        Node(
            package='mp_latency',
            namespace='ipc_lat',
            executable='ipctail',
            output='screen',
            # TAIL args: testDur, REL, pubFreq, totalNodes, fromTopic, rmwType, myCfgName
            arguments=[test_duration, rel_type, pub_frequency, '3', 'pt_profile_topic_1_3', rmw_type, 'h-3p-t2'],
            name='tail2',
        ),
        Node(
            package='mp_latency',
            namespace='ipc_lat',
            executable='ipctail',
            output='screen',
            # TAIL args: testDur, REL, pubFreq, totalNodes, fromTopic, rmwType, myCfgName
            arguments=[test_duration, rel_type, pub_frequency, '3', 'pt_profile_topic_1_4', rmw_type, 'h-3p-t3'],
            name='tail3',
        ),
        Node(
            package='mp_latency',
            namespace='ipc_lat',
            executable='ipctail',
            output='screen',
            # TAIL args: testDur, REL, pubFreq, totalNodes, fromTopic, rmwType, myCfgName
            arguments=[test_duration, rel_type, pub_frequency, '3', 'pt_profile_topic_1_5', rmw_type, 'h-3p-t4'],
            name='tail4',
        ),
        Node(
            package='mp_latency',
            namespace='ipc_lat',
            executable='ipctail',
            output='screen',
            # TAIL args: testDur, REL, pubFreq, totalNodes, fromTopic, rmwType, myCfgName
            arguments=[test_duration, rel_type, pub_frequency, '3', 'pt_profile_topic_1_6', rmw_type, 'h-3p-t5'],
            name='tail5',
        ),
        Node(
            package='mp_latency',
            namespace='ipc_lat',
            executable='ipctail',
            output='screen',
            # TAIL args: testDur, REL, pubFreq, totalNodes, fromTopic, rmwType, myCfgName
            arguments=[test_duration, rel_type, pub_frequency, '3', 'pt_profile_topic_1_7', rmw_type, 'h-3p-t6'],
            name='tail6',
        ),
        Node(
            package='mp_latency',
            namespace='ipc_lat',
            executable='ipctail',
            output='screen',
            # TAIL args: testDur, REL, pubFreq, totalNodes, fromTopic, rmwType, myCfgName
            arguments=[test_duration, rel_type, pub_frequency, '3', 'pt_profile_topic_1_8', rmw_type, 'h-3p-t7'],
            name='tail7',
        ),
        Node(
            package='mp_latency',
            namespace='ipc_lat',
            executable='ipctail',
            output='screen',
            # TAIL args: testDur, REL, pubFreq, totalNodes, fromTopic, rmwType, myCfgName
            arguments=[test_duration, rel_type, pub_frequency, '3', 'pt_profile_topic_1_9', rmw_type, 'h-3p-t8'],
            name='tail8',
        ),
        Node(
            package='mp_latency',
            namespace='ipc_lat',
            executable='ipctail',
            output='screen',
            # TAIL args: testDur, REL, pubFreq, totalNodes, fromTopic, rmwType, myCfgName
            arguments=[test_duration, rel_type, pub_frequency, '3', 'pt_profile_topic_1_10', rmw_type, 'h-3p-t9'],
            name='tail9',
        ),
        Node(
            package='mp_latency',
            namespace='ipc_lat',
            executable='ipctail',
            output='screen',
            # TAIL args: testDur, REL, pubFreq, totalNodes, fromTopic, rmwType, myCfgName
            arguments=[test_duration, rel_type, pub_frequency, '3', 'pt_profile_topic_1_11', rmw_type, 'h-3p-t10'],
            name='tail10',
        ),
        Node(
            package='mp_latency',
            namespace='ipc_lat',
            executable='ipcnode',
            output='screen',
            # TEST args: testDur, REL, myNodeId, fromTopic, toTopic
            arguments=[test_duration, rel_type, '10', 'pt_profile_topic_0_1', 'pt_profile_topic_1_11'],
            name='node10'
        ),
        Node(
            package='mp_latency',
            namespace='ipc_lat',
            executable='ipcnode',
            output='screen',
            # TEST args: testDur, REL, myNodeId, fromTopic, toTopic
            arguments=[test_duration, rel_type, '9', 'pt_profile_topic_0_1', 'pt_profile_topic_1_10'],
            name='node9'
        ),
        Node(
            package='mp_latency',
            namespace='ipc_lat',
            executable='ipcnode',
            output='screen',
            # TEST args: testDur, REL, myNodeId, fromTopic, toTopic
            arguments=[test_duration, rel_type, '8', 'pt_profile_topic_0_1', 'pt_profile_topic_1_9'],
            name='node8'
        ),
        Node(
            package='mp_latency',
            namespace='ipc_lat',
            executable='ipcnode',
            output='screen',
            # TEST args: testDur, REL, myNodeId, fromTopic, toTopic
            arguments=[test_duration, rel_type, '7', 'pt_profile_topic_0_1', 'pt_profile_topic_1_8'],
            name='node7'
        ),
        Node(
            package='mp_latency',
            namespace='ipc_lat',
            executable='ipcnode',
            output='screen',
            # TEST args: testDur, REL, myNodeId, fromTopic, toTopic
            arguments=[test_duration, rel_type, '6', 'pt_profile_topic_0_1', 'pt_profile_topic_1_7'],
            name='node6'
        ),
        Node(
            package='mp_latency',
            namespace='ipc_lat',
            executable='ipcnode',
            output='screen',
            # TEST args: testDur, REL, myNodeId, fromTopic, toTopic
            arguments=[test_duration, rel_type, '5', 'pt_profile_topic_0_1', 'pt_profile_topic_1_6'],
            name='node5'
        ),
        Node(
            package='mp_latency',
            namespace='ipc_lat',
            executable='ipcnode',
            output='screen',
            # TEST args: testDur, REL, myNodeId, fromTopic, toTopic
            arguments=[test_duration, rel_type, '4', 'pt_profile_topic_0_1', 'pt_profile_topic_1_5'],
            name='node4'
        ),

        Node(
            package='mp_latency',
            namespace='ipc_lat',
            executable='ipcnode',
            output='screen',
            # TEST args: testDur, REL, myNodeId, fromTopic, toTopic
            arguments=[test_duration, rel_type, '3', 'pt_profile_topic_0_1', 'pt_profile_topic_1_4'],
            name='node3'
        ),
        Node(
            package='mp_latency',
            namespace='ipc_lat',
            executable='ipcnode',
            output='screen',
            # TEST args: testDur, REL, myNodeId, fromTopic, toTopic
            arguments=[test_duration, rel_type, '2', 'pt_profile_topic_0_1', 'pt_profile_topic_1_3'],
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
