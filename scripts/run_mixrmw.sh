#!/bin/bash

#############################
# change variables here
# NOTE that total test time = (TEST_DURATION + 5) * set size of next 5 vars,
# for example (60 + 5) * 3 * 3 * 5 * 2 * 9 = 52650 seconds, or 14.6 hours.

DDS_BACKENDS_PUB='rmw_connextdds rmw_fastrtps_cpp rmw_cyclonedds_cpp'
DDS_BACKENDS_SUB='rmw_connextdds rmw_fastrtps_cpp rmw_cyclonedds_cpp'
F_PUBLISHER_SET='20 40 60 100 200'
# MSG_SIZE_SUFFIX_SET='100b 1kb 4kb 10kb 16kb 32kb 60kb 100kb 256kb 500kb 1mb 2mb 4mb 8mb'
MSG_SIZE_SUFFIX_SET='500kb'
QOS_RELIABILITY_SET='BE REL'
NODE_WORK_COUNT_SET='0'
TEST_DURATION=30
TEST_TOPIC_NAME='pt_profile_topic_'
TEST_CONFIG_NAME='1mix1'

# the test will create data files in the current directory 
for F_PUBLISHER in $F_PUBLISHER_SET
do
    for NODE_WORK_COUNT in $NODE_WORK_COUNT_SET
    do
        for QOS_RELIABILITY in $QOS_RELIABILITY_SET
        do
            for MSG_SIZE_SUFFIX in $MSG_SIZE_SUFFIX_SET
            do
                for BACKENDPUB in $DDS_BACKENDS_PUB
                do
                    for BACKENDSUB in $DDS_BACKENDS_SUB
                    do
                        # set env vars for this batch of tests
                        export RMW_IMPLEMENTATION=$BACKENDPUB
                        export RMW_IMPLEMENTATION_SUB=$BACKENDSUB
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
                        command="ros2 launch ../ros2_mp_latency/mp_latency/launch/mplat_ser2_mixrmw.py"
                        $command
                        sleep 3
                    done
                done
            done
        done
    done
done
