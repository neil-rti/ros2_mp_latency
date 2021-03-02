from launch import LaunchDescription
from launch_ros.actions import Node
import os

# using env vars to set parms for test, default values if not set
test_duration = '60'    # duration of test run, in seconds
if 'RTI_MPL_DURATION' in os.environ:
    test_duration = os.environ['RTI_MPL_DURATION']

pub_frequency = '10'    # pubs per second from HEAD node (float32 value)
if 'RTI_MPL_PUBFREQ' in os.environ:
    pub_frequency = os.environ['RTI_MPL_PUBFREQ']
    
rel_type = 'REL'         # reliability type (REL=reliable, anything else is best effort)
if 'RTI_MPL_RELY' in os.environ:
    rel_type = os.environ['RTI_MPL_RELY']

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
            arguments=[test_duration, rel_type, pub_frequency, '22', 'pt_profile_topic_20_21', rmw_type, 'h-20s-t'],
            name='tail',
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
            arguments=[test_duration, rel_type, '3', 'pt_profile_topic_2_3', 'pt_profile_topic_3_4'],
            name='node3'
        ),

        Node(
            package='mp_latency',
            namespace='ipc_lat',
            executable='ipcnode',
            output='screen',
            # TEST args: testDur, REL, myNodeId, fromTopic, toTopic
            arguments=[test_duration, rel_type, '4', 'pt_profile_topic_3_4', 'pt_profile_topic_4_5'],
            name='node4'
        ),
        Node(
            package='mp_latency',
            namespace='ipc_lat',
            executable='ipcnode',
            output='screen',
            # TEST args: testDur, REL, myNodeId, fromTopic, toTopic
            arguments=[test_duration, rel_type, '5', 'pt_profile_topic_4_5', 'pt_profile_topic_5_6'],
            name='node5'
        ),
        Node(
            package='mp_latency',
            namespace='ipc_lat',
            executable='ipcnode',
            output='screen',
            # TEST args: testDur, REL, myNodeId, fromTopic, toTopic
            arguments=[test_duration, rel_type, '6', 'pt_profile_topic_5_6', 'pt_profile_topic_6_7'],
            name='node6'
        ),
        Node(
            package='mp_latency',
            namespace='ipc_lat',
            executable='ipcnode',
            output='screen',
            # TEST args: testDur, REL, myNodeId, fromTopic, toTopic
            arguments=[test_duration, rel_type, '7', 'pt_profile_topic_6_7', 'pt_profile_topic_7_8'],
            name='node7'
        ),
        Node(
            package='mp_latency',
            namespace='ipc_lat',
            executable='ipcnode',
            output='screen',
            # TEST args: testDur, REL, myNodeId, fromTopic, toTopic
            arguments=[test_duration, rel_type, '8', 'pt_profile_topic_7_8', 'pt_profile_topic_8_9'],
            name='node8'
        ),
        Node(
            package='mp_latency',
            namespace='ipc_lat',
            executable='ipcnode',
            output='screen',
            # TEST args: testDur, REL, myNodeId, fromTopic, toTopic
            arguments=[test_duration, rel_type, '9', 'pt_profile_topic_8_9', 'pt_profile_topic_9_10'],
            name='node9'
        ),
        Node(
            package='mp_latency',
            namespace='ipc_lat',
            executable='ipcnode',
            output='screen',
            # TEST args: testDur, REL, myNodeId, fromTopic, toTopic
            arguments=[test_duration, rel_type, '10', 'pt_profile_topic_9_10', 'pt_profile_topic_10_11'],
            name='node10'
        ),
        Node(
            package='mp_latency',
            namespace='ipc_lat',
            executable='ipcnode',
            output='screen',
            # TEST args: testDur, REL, myNodeId, fromTopic, toTopic
            arguments=[test_duration, rel_type, '11', 'pt_profile_topic_10_11', 'pt_profile_topic_11_12'],
            name='node11'
        ),
        Node(
            package='mp_latency',
            namespace='ipc_lat',
            executable='ipcnode',
            output='screen',
            # TEST args: testDur, REL, myNodeId, fromTopic, toTopic
            arguments=[test_duration, rel_type, '12', 'pt_profile_topic_11_12', 'pt_profile_topic_12_13'],
            name='node12'
        ),
        Node(
            package='mp_latency',
            namespace='ipc_lat',
            executable='ipcnode',
            output='screen',
            # TEST args: testDur, REL, myNodeId, fromTopic, toTopic
            arguments=[test_duration, rel_type, '13', 'pt_profile_topic_12_13', 'pt_profile_topic_13_14'],
            name='node13'
        ),
        Node(
            package='mp_latency',
            namespace='ipc_lat',
            executable='ipcnode',
            output='screen',
            # TEST args: testDur, REL, myNodeId, fromTopic, toTopic
            arguments=[test_duration, rel_type, '14', 'pt_profile_topic_13_14', 'pt_profile_topic_14_15'],
            name='node14'
        ),
        Node(
            package='mp_latency',
            namespace='ipc_lat',
            executable='ipcnode',
            output='screen',
            # TEST args: testDur, REL, myNodeId, fromTopic, toTopic
            arguments=[test_duration, rel_type, '15', 'pt_profile_topic_14_15', 'pt_profile_topic_15_16'],
            name='node15'
        ),
        Node(
            package='mp_latency',
            namespace='ipc_lat',
            executable='ipcnode',
            output='screen',
            # TEST args: testDur, REL, myNodeId, fromTopic, toTopic
            arguments=[test_duration, rel_type, '16', 'pt_profile_topic_15_16', 'pt_profile_topic_16_17'],
            name='node16'
        ),
        Node(
            package='mp_latency',
            namespace='ipc_lat',
            executable='ipcnode',
            output='screen',
            # TEST args: testDur, REL, myNodeId, fromTopic, toTopic
            arguments=[test_duration, rel_type, '17', 'pt_profile_topic_16_17', 'pt_profile_topic_17_18'],
            name='node17'
        ),
        Node(
            package='mp_latency',
            namespace='ipc_lat',
            executable='ipcnode',
            output='screen',
            # TEST args: testDur, REL, myNodeId, fromTopic, toTopic
            arguments=[test_duration, rel_type, '18', 'pt_profile_topic_17_18', 'pt_profile_topic_18_19'],
            name='node18'
        ),
        Node(
            package='mp_latency',
            namespace='ipc_lat',
            executable='ipcnode',
            output='screen',
            # TEST args: testDur, REL, myNodeId, fromTopic, toTopic
            arguments=[test_duration, rel_type, '19', 'pt_profile_topic_18_19', 'pt_profile_topic_19_20'],
            name='node19'
        ),
        Node(
            package='mp_latency',
            namespace='ipc_lat',
            executable='ipcnode',
            output='screen',
            # TEST args: testDur, REL, myNodeId, fromTopic, toTopic
            arguments=[test_duration, rel_type, '20', 'pt_profile_topic_19_20', 'pt_profile_topic_20_21'],
            name='node20'
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
