# This is an example of a more verbose in-series test description
# If testing arbitrary lengths of WORK node chains, use mplat_ser_n.py 
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

my_node = 0                 # starting node ID
data_type_suffix = '1kb'    # edit this to change data size in test
exe_source = 'ipcsource_' + data_type_suffix
exe_work = 'ipcwork_' + data_type_suffix
exe_sink = 'ipcsink_' + data_type_suffix
rmw_type = os.environ.get('RMW_IMPLEMENTATION') # RMW type used for test (for TAIL node results)

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='mp_latency',
            namespace='ipc_lat',
            executable=exe_sink,
            output='screen',
            # TAIL args: testDur, REL, pubFreq, totalNodes, fromTopic, rmwType, myCfgName
            arguments=[test_duration, rel_type, pub_frequency, '4', 'pt_profile_topic_2_3', rmw_type, 'h-2s-t'],
            name='sink',
        ),
        Node(
            package='mp_latency',
            namespace='ipc_lat',
            executable=exe_work,
            output='screen',
            # TEST args: testDur, REL, myNodeId, fromTopic, toTopic
            arguments=[test_duration, rel_type, '1', 'pt_profile_topic_0_1', 'pt_profile_topic_1_2'],
            name='node1'
        ),
        Node(
            package='mp_latency',
            namespace='ipc_lat',
            executable=exe_work,
            output='screen',
            # TEST args: testDur, REL, myNodeId, fromTopic, toTopic
            arguments=[test_duration, rel_type, '2', 'pt_profile_topic_1_2', 'pt_profile_topic_2_3'],
            name='node2'
        ),
        Node(
            package='mp_latency',
            namespace='ipc_lat',
            executable=exe_source,
            output='screen',
            # HEAD args: testDur, REL, pubFreq, myNodeId, toTopic
            arguments=[test_duration, rel_type, pub_frequency, '0', 'pt_profile_topic_0_1'],
            name='source'
        )
    ])
