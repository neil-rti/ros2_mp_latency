#!/bin/bash

#############################
# change variables here
# NOTE that total test time = (TEST_DURATION + 5) * set size of next 5 vars,
# for example (60 + 5) * 3 * 3 * 5 * 2 * 9 = 52650 seconds, or 14.6 hours.

DDS_BACKENDS='rmw_connextdds rmw_fastrtps_cpp rmw_cyclonedds_cpp'
F_PUBLISHER_SET='10'
MSG_SIZE_SUFFIX_SET='100b 1kb 10kb 100kb 500kb'
QOS_RELIABILITY_SET='BE REL'
NODE_WORK_COUNT_SET='0'
TEST_DURATION=10
TEST_TOPIC_NAME='pt_profile_topic_'
TEST_CONFIG_NAME='h-t'

# the test will create data files in the current directory 
for QOS_RELIABILITY in $QOS_RELIABILITY_SET
do
    for F_PUBLISHER in $F_PUBLISHER_SET
    do
        for BACKEND in $DDS_BACKENDS
        do
            for NODE_WORK_COUNT in $NODE_WORK_COUNT_SET
            do
                for MSG_SIZE_SUFFIX in $MSG_SIZE_SUFFIX_SET
                do
                    # set env vars for this batch of tests
                    export RMW_IMPLEMENTATION=$BACKEND
                    export RTI_MPL_TEST_DURATION=$TEST_DURATION
                    export RTI_MPL_PUB_FREQ=$F_PUBLISHER 
                    export RTI_MPL_RELY_TYPE=$QOS_RELIABILITY 
                    export RTI_MPL_TOPIC_NAME=$TEST_TOPIC_NAME
                    export RTI_MPL_WORK_NODES=$NODE_WORK_COUNT
                    export RTI_MPL_CONFIG_NAME=$TEST_CONFIG_NAME
                    export RTI_MPL_SIZE_SUFFIX=$MSG_SIZE_SUFFIX

                    # launch the tests
                    timestamp=$(date +%Y-%m-%d_%H-%M-%S)
                    echo "[$timestamp]:"
                    command="ros2 launch ../ros2_mp_latency/mp_latency/launch/mplat_ser_n.py"
                    $command
                    sleep 3
                done
            done
        done
    done
done
