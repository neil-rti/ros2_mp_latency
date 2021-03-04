#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <ctime>
#include "rclcpp/rclcpp.hpp"
#include "IpcTestDefs.hpp"    // define the test size here

using namespace std::chrono_literals;
using std::placeholders::_1;

class IpcThru : public rclcpp::Node
{
  public:
    // TEST node args testDuration, useReliable, myNodeId, fromTopic, toTopic
    IpcThru(uint32_t testDuration, bool useReliable, uint32_t myNodeId, 
      std::string fromTopic, std::string toTopic)
    : Node("ipc_node")
    {
      // init the topic names per the node number
      node_number = myNodeId;
      run_for_seconds = testDuration;
      timespec tstart;
      tspec_get(&tstart);
      start_time_seconds = tstart.tv_sec;

      if(useReliable) {
        publisher_ = this->create_publisher<MP_DATA_TYPE>(toTopic, rclcpp::QoS(1).reliable());
        subscription_ = this->create_subscription<MP_DATA_TYPE>(
          fromTopic, rclcpp::QoS(1).reliable(), std::bind(&IpcThru::topic_callback, this, _1));
      }
      else {
        publisher_ = this->create_publisher<MP_DATA_TYPE>(toTopic, rclcpp::QoS(1).best_effort());
        subscription_ = this->create_subscription<MP_DATA_TYPE>(
          fromTopic, rclcpp::QoS(1).best_effort(), std::bind(&IpcThru::topic_callback, this, _1));
      }

      // create timer to check when to quit
      timer_ = this->create_wall_timer(
          std::chrono::milliseconds(654),
          std::bind(&IpcThru::timer_callback, this));

    }

  private:
    // callback for received data; immediately publish to the next node
    void topic_callback(const MP_DATA_TYPE::SharedPtr msg)
    {
      // get the current time to calc node-to-node latency
      uint64_t myTime = tstamp_get();

  // debug
//  fprintf(stderr, "M%u:", node_number);
//  for(int32_t d=0 ; d<40 ; d++) {
//    fprintf(stderr, " %02x", msg->data[TS_NODE_BASE + d]);
//  }
//  fprintf(stderr, "\n");


      // copy the received message to the send message
      send_msg_ = *msg;


      // get the time difference from previous node, and put into next-open slot in array
      uint64_t prevTime = 0;
      memcpy(&prevTime, &send_msg_.data[PREV_TS_OFS],sizeof(uint64_t));
      uint64_t timeDiff = myTime - prevTime;
      if(timeDiff > 0xffffffff) timeDiff = 0xffffffff;  // saturate if over 32 bits (4 seconds)
      uint32_t myDiff = (uint32_t)(timeDiff & 0xffffffff);
  
  // fprintf(stderr, "myTime: %lu, prevTime: %lu, myDiff: %u, myNode: %u\n", myTime, prevTime, myDiff, node_number);

#if 1 // this uses an index in the array
      uint32_t nextSlot = (send_msg_.data[TS_IDX_OFS] * sizeof(uint32_t)) + TS_NODE_BASE;
      memcpy(&send_msg_.data[nextSlot], &myDiff, sizeof(uint32_t));
      send_msg_.data[TS_IDX_OFS]++;
#else // this uses a fixed offset based on my node number
 //     uint32_t nextSlot = ((node_number - 1) * 4) + TS_NODE_BASE;
 //     memcpy(&send_msg_.data[nextSlot], &myDiff, 4);
#endif


      // now get another current timestamp into the 'prevTime' position (to send to the next node)
      myTime = tstamp_get();
      memcpy(&send_msg_.data[PREV_TS_OFS], &myTime, sizeof(uint64_t));

      publisher_->publish(send_msg_);
    }

    // timer callback to check when to quit
    void timer_callback()
    {
      // get the current time
      timespec tstamp;
      tspec_get(&tstamp);

      // time to quit?
      if((tstamp.tv_sec - start_time_seconds) > run_for_seconds) {
        rclcpp::shutdown();
      }
    }

    rclcpp::Subscription<MP_DATA_TYPE>::SharedPtr subscription_;
    rclcpp::Publisher<MP_DATA_TYPE>::SharedPtr publisher_;
    MP_DATA_TYPE send_msg_;
    rclcpp::TimerBase::SharedPtr timer_;
    int node_number;
    uint32_t run_for_seconds;
    uint32_t start_time_seconds;
};

int main(int argc, char * argv[])
{
  // TEST node gets 5 args, default values are here, in arg order:
  uint32_t testDuration = 60;           // [1] how long to run the test  (for quitting)
  bool useReliable = false;             // [2] best effort or reliable
  uint32_t myNodeId = 1;                // [3] my node's ID number
  std::string fromTopic = "fromHead";   // [4] topic I subscribe
  std::string toTopic = "toTail";       // [5] topic I publish
  
  // NOTE that extra values can be passed by the ROS2 launch system
  if(argc > 5) {
    testDuration = strtoul(argv[1], NULL, 10);
    useReliable = (!(strcmp(argv[2], "REL"))); // anything but "REL" == best_effort
    myNodeId = strtoul(argv[3], NULL, 10);
    fromTopic = argv[4];
    toTopic = argv[5];
  }
  else {
    fprintf(stdout, "Using default args:\n  testDuration(%u), relType(%s), myNodeId(%u), fromTopic(%s), toTopic(%s)\n",
      testDuration, useReliable?"REL":"BE", myNodeId, fromTopic.c_str(), toTopic.c_str());
  }

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<IpcThru>(testDuration, useReliable, myNodeId, fromTopic, toTopic));
  rclcpp::shutdown();
  return 0;
}