from launch import LaunchDescription
from launch_ros.actions import Node
import os

# This is used for configuring serial-chain tests with 0--N nodes.
# Env var are used to set:
#   RMW, topicNameRoot, workNodesCount, testDuration, relyType, pubFreq, cfgName, dataSizeSuffix
rmw_type = 'rmw_connextdds'                 # rmw selection, default to Connext pro
if 'RMW_IMPLEMENTATION' in os.environ:
    rmw_type = os.environ['RMW_IMPLEMENTATION']

topic_base = 'pt_profile_topic'             # root of the topic names used in testing (suffix with node numbers)
if 'RTI_MPL_TOPIC_NAME' in os.environ:
    topic_base = os.environ['RTI_MPL_TOPIC_NAME']

work_nodes_count = '1'
if 'RTI_MPL_WORK_NODES' in os.environ:
    work_nodes_count = os.environ['RTI_MPL_WORK_NODES']

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

size_suffix = '100b'
if 'RTI_MPL_SIZE_SUFFIX' in os.environ:
    size_suffix = os.environ['RTI_MPL_SIZE_SUFFIX']

exe_source = 'ipcsource_' + size_suffix
exe_work = 'ipcwork_' + size_suffix
exe_sink = 'ipcsink_' + size_suffix


# build the test per the args
def generate_launch_description():
    ld = LaunchDescription()
    
    # add the sink node
    total_nodes = int(work_nodes_count) + 2
    sink_node_topic_name = topic_base + '_' + str(total_nodes-2) + '_' + str(total_nodes-1)
    sink_node = Node(
        package='mp_latency',
        namespace='ipc_lat',
        executable=exe_sink,
        output='screen',
        arguments=[test_duration, rely_type, pub_frequency, str(total_nodes), sink_node_topic_name, rmw_type, config_name],
        name='sink'
    )
    ld.add_action(sink_node)

    # add the work nodes
    for n in range(1, (total_nodes - 1)):
        this_node_name = 'work{}'.format(str(n))
        from_node_topic_name = '{}_{}_{}'.format(topic_base, str(n-1), str(n))
        to_node_topic_name = '{}_{}_{}'.format(topic_base, str(n), str(n+1))
        
        ld.add_action(
            Node(
                package='mp_latency',
                namespace='ipc_lat',
                executable=exe_work,
                output='screen',
                arguments=[test_duration, rely_type, str(n), from_node_topic_name, to_node_topic_name],
                name=this_node_name
            )
        )
    
    # add the source node
    source_node_topic_name = '{}_0_1'.format(topic_base)
    ld.add_action(
        Node(
            package='mp_latency',
            namespace='ipc_lat',
            executable=exe_source,
            output='screen',
            arguments=[test_duration, rely_type, pub_frequency, '0', source_node_topic_name, rmw_type],
            name='source'
        )
     )

    return ld

