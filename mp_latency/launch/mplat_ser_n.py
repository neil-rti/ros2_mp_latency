from launch import LaunchDescription
from launch_ros.actions import Node
import os

# This is used for configuring serial-chain tests with 0--N nodes.
# Env var are used to set:    RMW, topicNameRoot, thruNodesCount, testDuration, relyType, pubFreq, cfgName
rmw_type = 'rmw_connextdds'                 # rmw selection, default to Connext pro
if 'RMW_IMPLEMENTATION' in os.environ:
    rmw_type = os.environ['RMW_IMPLEMENTATION']

topic_base = 'pt_profile_topic'             # root of the topic names used in testing (suffix with node numbers)
if 'RTI_MPL_TOPIC_NAME' in os.environ:
    topic_base = os.environ['RTI_MPL_TOPIC_NAME']

# Cmdline args are used for:  thruNodesCount, testDuration, relyType, pubFreq, cfgName
thru_nodes_count = '1'
if 'RTI_MPL_THRU_NODES' in os.environ:
    thru_nodes_count = os.environ['RTI_MPL_THRU_NODES']

test_duration = '60'
if 'RTI_MPL_TEST_DURATION' in os.environ:
    test_duration = os.environ['RTI_MPL_TEST_DURATION']

rely_type = 'BE'
if 'RTI_MPL_RELY_TYPE' in os.environ:
    rely_type = os.environ['RTI_MPL_RELY_TYPE']

pub_frequency = '10'
if 'RTI_MPL_PUB_FREQ' in os.environ:
    pub_frequency = os.environ['RTI_MPL_PUB_FREQ']

config_name = 'default'
if 'RTI_MPL_CONFIG_NAME' in os.environ:
    config_name = os.environ['RTI_MPL_CONFIG_NAME']

# build the test per the args
def generate_launch_description():
    ld = LaunchDescription()
    
    # add the tail node
    total_nodes = int(thru_nodes_count) + 2
    # tail_node_topic_name = '{}_{}_{}'.format(topic_base, str(total_nodes-1), str(total_nodes))
    tail_node_topic_name = topic_base + '_' + str(total_nodes-2) + '_' + str(total_nodes-1)
    tail_node = Node(
        package='mp_latency',
        namespace='ipc_lat',
        executable='ipctail',
        output='screen',
        arguments=[test_duration, rely_type, pub_frequency, str(total_nodes), tail_node_topic_name, rmw_type, config_name],
        name='tail'
    )
    ld.add_action(tail_node)

    # add the thru nodes
    for n in range(1, (total_nodes - 1)):
        this_node_name = 'thru{}'.format(str(n))
        from_node_topic_name = '{}_{}_{}'.format(topic_base, str(n-1), str(n))
        to_node_topic_name = '{}_{}_{}'.format(topic_base, str(n), str(n+1))
        
        ld.add_action(
            Node(
                package='mp_latency',
                namespace='ipc_lat',
                executable='ipcthru',
                output='screen',
                arguments=[test_duration, rely_type, str(n), from_node_topic_name, to_node_topic_name],
                name=this_node_name
            )
        )
    
    # add the head node
    head_node_topic_name = '{}_0_1'.format(topic_base)
    ld.add_action(
        Node(
            package='mp_latency',
            namespace='ipc_lat',
            executable='ipchead',
            output='screen',
            arguments=[test_duration, rely_type, pub_frequency, '0', head_node_topic_name, rmw_type],
            name='head'
        )
     )

    return ld

