#!/bin/bash

#############################
# change variables here
# NOTE that total test time = (TEST_DURATION + 5) * set size of next 5 vars,
# for example (60 + 5) * 3 * 3 * 5 * 2 * 9 = 52650 seconds, or 14.6 hours.

DDS_BACKENDS='rmw_connextdds rmw_fastrtps_cpp rmw_cyclonedds_cpp'
F_PUBLISHER_SET='1 10 100'
MSG_SIZE_SET='100 1000 10000 100000 500000'
QOS_RELIABILITY_SET='BE REL'
NODE_THRU_COUNT_SET='0 1 2 3 4 5 10 15 20'
TEST_DURATION=60
TEST_TOPIC_NAME='pt_profile_topic_'
TEST_CONFIG_NAME='h-sn-t'

# the test will create data files in the current directory 
for QOS_RELIABILITY in $QOS_RELIABILITY_SET
do
    for F_PUBLISHER in $F_PUBLISHER_SET
    do
        for BACKEND in $DDS_BACKENDS
        do
            for NODE_THRU_COUNT in $NODE_THRU_COUNT_SET
            do
                # set env vars for this batch of tests
                export RMW_IMPLEMENTATION=$BACKEND
                export RTI_MPL_TEST_DURATION=$TEST_DURATION
                export RTI_MPL_PUB_FREQ=$F_PUBLISHER 
                export RTI_MPL_RELY_TYPE=$QOS_RELIABILITY 
                export RTI_MPL_TOPIC_NAME=$TEST_TOPIC_NAME
                export RTI_MPL_THRU_NODES=$NODE_THRU_COUNT
                export RTI_MPL_CONFIG_NAME=$TEST_CONFIG_NAME

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
