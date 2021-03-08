#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <ctime>
#include "rclcpp/rclcpp.hpp"
#include "IpcTestDefs.hpp"    // define the test size here

using namespace std::chrono_literals;

class IpcHead : public rclcpp::Node
{
  public:
    // HEAD node args are: testDuration, useReliable, pubFreq, myNodeId, toTopic
    IpcHead(uint32_t testDuration, bool useReliable, float pubFreq, uint32_t myNodeId,
        std::string toTopic )
    : Node("ipc_head")
    {
      // init
      sample_number = 0;
      run_for_seconds = testDuration;
      my_node_id = myNodeId;
      pub_delay_ms = (uint32_t)(1000.0f / pubFreq);

      if(useReliable) {
        publisher_ = this->create_publisher<MP_DATA_TYPE>(toTopic, rclcpp::QoS(1).reliable());
      }
      else {
        publisher_ = this->create_publisher<MP_DATA_TYPE>(toTopic, rclcpp::QoS(1).best_effort());
      }
      send_msg_.header.size = (uint32_t)MP_DATA_SIZE;
      send_msg_.header.frequency = pubFreq;

      // init the data array with random
      for(int i=0 ; i<MP_DATA_SIZE ; i++) {
        send_msg_.data[i] = (unsigned char)(rand() & 0xff);
      }

      timespec tstart;
      tspec_get(&tstart);
      start_time_seconds = tstart.tv_sec;

      timer_ = this->create_wall_timer(
          std::chrono::milliseconds(pub_delay_ms),
          std::bind(&IpcHead::timer_callback, this));
    }

  private:
    // timer callback to publish data
    void timer_callback()
    {
      // get the current time
      timespec tstamp;
      tspec_get(&tstamp);
      uint64_t myTime = (tstamp.tv_sec * 1000000000) + tstamp.tv_nsec;

      // put my timestamp into the 'headTime' and 'prevTime' positions in the array
      memcpy(&send_msg_.data[HEAD_TS_OFS], &myTime, sizeof(uint64_t));
      memcpy(&send_msg_.data[PREV_TS_OFS], &myTime, sizeof(uint64_t));
      send_msg_.data[TS_IDX_OFS] = 0; 

      send_msg_.header.tracking_number = sample_number++;
      publisher_->publish(send_msg_);

      // time to quit?
      if((tstamp.tv_sec - start_time_seconds) > run_for_seconds) {
        rclcpp::shutdown();
      }
    }

    rclcpp::Publisher<MP_DATA_TYPE>::SharedPtr publisher_;
    MP_DATA_TYPE send_msg_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::string send_topic_name;
    uint32_t pub_delay_ms;
    uint32_t sample_number;
    uint32_t run_for_seconds;
    uint32_t start_time_seconds;
    uint32_t my_node_id;
};

int main(int argc, char * argv[])
{
  // HEAD node gets 5 args, default values are here, in arg order:
  uint32_t testDuration = 60;           // [1] how long to run the test  (for quitting)
  bool useReliable = false;             // [2] best effort or reliable
  float pubFreq = 1.0;                  // [3] the publication frequency
  uint32_t myNodeId = 1;                // [4] my node's ID number
  std::string toTopic = "fromHead";     // [5] topic I publish
  
  // NOTE that extra values can be passed by the ROS2 launch system
  if(argc > 5) {
    testDuration = strtoul(argv[1], NULL, 10);
    useReliable = (!(strcmp(argv[2], "REL"))); // anything but "REL" == best_effort
    pubFreq = strtof(argv[3], NULL);
    myNodeId = strtoul(argv[4], NULL, 10);
    toTopic = argv[5];
  }
  else {
    fprintf(stdout, "Running with default args:\n  testDuration(%u), relType(%s), pubFreq(%1.0f), myNodeId(%u), toTopic(%s)\n",
      testDuration, useReliable?"REL":"BE", pubFreq, myNodeId, toTopic.c_str());
  }

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<IpcHead>(testDuration, useReliable, pubFreq, myNodeId, toTopic));
  rclcpp::shutdown();
  return 0;
}