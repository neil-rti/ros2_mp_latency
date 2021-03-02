#!/bin/bash

#############################
# change variables here

DDS_BACKENDS='rmw_connextdds rmw_fastrtps_cpp rmw_cyclonedds_cpp'
F_PUBLISHER_SET='1 10 100'
MSG_SIZE_SET='100 1000 10000 100000 500000'
QOS_RELIABILITY_SET='BE REL'
TEST_DURATION=60
DPP_RANGE_START=1
DPP_RANGE_END=21
DPP_RANGE_STEP_SIZE=2

# the test will create data files in the current directory 
for QOS_RELIABILITY in $QOS_RELIABILITY_SET
do
    for F_PUBLISHER in $F_PUBLISHER_SET
    do
        for BACKEND in $DDS_BACKENDS
        do
            # set env vars for this batch of tests
            export RMW_IMPLEMENTATION=$BACKEND
            export RTI_MPL_DURATION=$TEST_DURATION
            export RTI_MPL_PUBFREQ=$F_PUBLISHER 
            export RTI_MPL_RELY=$QOS_RELIABILITY 
            # launch the tests
            timestamp=$(date +%Y-%m-%d_%H-%M-%S)
            echo "[$timestamp]:"
            command="ros2 launch /mp_lat/mp_latency/launch/mplat_ser1.py"
            $command
            sleep 5

            timestamp=$(date +%Y-%m-%d_%H-%M-%S)
            echo "[$timestamp]: $command" >> log.txt
            command="ros2 launch /mp_lat/mp_latency/launch/mplat_ser2.py"
            $command
            sleep 5

            timestamp=$(date +%Y-%m-%d_%H-%M-%S)
            echo "[$timestamp]: $command" >> log.txt
            command="ros2 launch /mp_lat/mp_latency/launch/mplat_ser5.py"
            $command
            sleep 5

            timestamp=$(date +%Y-%m-%d_%H-%M-%S)
            echo "[$timestamp]: $command" >> log.txt
            command="ros2 launch /mp_lat/mp_latency/launch/mplat_ser10.py"
            $command
            sleep 5

            timestamp=$(date +%Y-%m-%d_%H-%M-%S)
            echo "[$timestamp]: $command" >> log.txt
            command="ros2 launch /mp_lat/mp_latency/launch/mplat_ser20.py"
            $command
            sleep 5

        done
    done
done
