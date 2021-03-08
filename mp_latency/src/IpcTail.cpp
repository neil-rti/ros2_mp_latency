#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <ctime>
#include "rclcpp/rclcpp.hpp"
#include "IpcTestDefs.hpp"    // define the test size here

using namespace std::chrono_literals;
using std::placeholders::_1;
#define HISTO_BIN_NS          (1000)      // use 1uS bin sizes
#define HISTO_MAX             (100000)    // at 1uS per bin = 100mS
#define HISTO_BIN_PRINT       (10)        // at 1uS per bin = 10uS print resolution
#define SKIP_FIRST_N_SAMPLES  (5)         // skip first 5 samples for startup noise reduction

class IpcTail : public rclcpp::Node
{
  public:
    // TAIL node args are: testDuration, useReliable, pubFreq, totalNodes, fromTopic, rmwType
    IpcTail(uint32_t testDuration, bool useReliable, float pubFreq, 
      uint32_t totalNodes, std::string fromTopic, std::string rmwType, std::string myConfig)
    : Node("ipc_tail")
    {
      // init values
      run_for_seconds = testDuration;
      my_nodes_count = totalNodes;
      rmw_type = rmwType;
      my_config = myConfig;
      pub_freq = pubFreq;
      timespec tstart;
      tspec_get(&tstart);
      start_time_seconds = tstart.tv_sec;
      histo_bin_count = HISTO_MAX;                  // bins total
      histo_bin_width_ns = HISTO_BIN_NS;            // histo bin width in nS
      histo_bin_printres = HISTO_BIN_PRINT;         // 10 bin resolution when reporting
      skip_first_n_samples = SKIP_FIRST_N_SAMPLES;  // skip first samples for noise reduction
      statsFileName = "statsAll.csv";
      reliability_type = "best_effort";
      if(useReliable) reliability_type = "reliable";

      // allocate memory for histogram bins:
      uint32_t mem_to_alloc = my_nodes_count * histo_bin_count;
      if(!(histo_bins = (uint32_t *) calloc(mem_to_alloc, sizeof(uint32_t)))) {
        fprintf(stderr, "cannot allocate %u bytes of memory (%s:%d)\n", mem_to_alloc, __FILE__, __LINE__ );
        rclcpp::shutdown();
      }
      sampleCount = 0;

      // open an output data file and print a header
      // data file name format: test_RMW_SIZEb_REL_DURs_CFGNAME_TSTAMP(sec).csv
      std::string relUsed;
      useReliable ? relUsed = "REL" : relUsed = "BE";
      dataFileName = 
          "test_" 
          + rmwType + "+"
          + std::to_string(MP_DATA_SIZE) + "b_"
          + relUsed + "_"
          + std::to_string(testDuration) + "s_"
          + myConfig + "_"
          + std::to_string(tstart.tv_sec);
      
      std::string testLogFileName = dataFileName + "_log.csv";

      fp = fopen(testLogFileName.c_str(), "w");
      if(fp == NULL) {
          fprintf(stderr, "FILE OPEN ERROR (%s:%d)", __FILE__, __LINE__);
          rclcpp::shutdown();
      }
      // Top file info
      fprintf(fp, "DataSize,%u,Nodes,%d,REL,%s,TestDuration,%u,PubFreqHz,%1.3f,RMW,%s,ConfigName,%s,StartTime,%lu\n",
        (uint32_t)MP_DATA_SIZE,
        my_nodes_count,
        useReliable?"Reliable":"BestEffort",
        testDuration, pubFreq, rmwType.c_str(), myConfig.c_str(),
        tstart.tv_sec
      );

      // column headers
      fprintf(fp, "Test#, SumOfLat(nS), ");
      for(uint32_t i=1 ; i<my_nodes_count ; i++) {
        fprintf(fp, " [%2d:%2d], ", i-1, i);
      }
      fprintf(fp, "          StartTime,             EndTime,    End-Start\n");

      // create subscriber to the named topic
      if(useReliable) {
        subscription_ = this->create_subscription<MP_DATA_TYPE>(
          fromTopic, rclcpp::QoS(1).reliable(), std::bind(&IpcTail::topic_callback, this, _1));
      }
      else {
        subscription_ = this->create_subscription<MP_DATA_TYPE>(
          fromTopic, rclcpp::QoS(1).best_effort(), std::bind(&IpcTail::topic_callback, this, _1));
      }

      // create timer to check when to quit
      timer_ = this->create_wall_timer(
          std::chrono::milliseconds(654),
          std::bind(&IpcTail::timer_callback, this));

    }

  private:
    // callback for received data; calc the latencies
    void topic_callback(const MP_DATA_TYPE::SharedPtr msg)
    {
      if(skip_first_n_samples) {
        skip_first_n_samples--;
      }
      else {
        // get the current time
        uint64_t myTime = tstamp_get();
        sampleCount++;

        // get the time diff from previous node
        uint64_t prevTime = 0;
        memcpy(&prevTime, &msg->data[PREV_TS_OFS],sizeof(uint64_t));
        uint64_t myDiff = myTime - prevTime;

        // get the headTime from the received data
        uint64_t headTime = 0;
        memcpy(&headTime, &msg->data[HEAD_TS_OFS],sizeof(uint64_t));      

        // add to histogram array for each node and total
        // Total is in column 0, each node is in [1 to N]
        uint64_t ipcSumLat = myDiff;
        for(uint32_t i=0 ; i<my_nodes_count-2 ; i++) {
          uint32_t nodeLat;
          memcpy(&nodeLat, &msg->data[TS_NODE_BASE + (i * 4)], 4);

          // put into histo bins.  If above range, put into topmost bin.
          uint32_t binIdx = (nodeLat / histo_bin_width_ns);
          if(binIdx > (histo_bin_count-1)) {
            binIdx = (histo_bin_count-1);
          }
          histo_bins[(binIdx * my_nodes_count) + (i+1)]++;
          ipcSumLat += (uint64_t)nodeLat;
        }

        // update the sum latency histo bin (column 0), if above range, put into topmost bin.
        uint32_t binIdx = (ipcSumLat / histo_bin_width_ns);
        if(binIdx > (histo_bin_count-1)) {
          binIdx = (histo_bin_count-1);
        }
        histo_bins[(binIdx * my_nodes_count)]++;

        // add my latency to the last column
        binIdx = (myDiff / histo_bin_width_ns);
        if(binIdx > (histo_bin_count-1)) {
          binIdx = (histo_bin_count-1);
        }
        histo_bins[(binIdx * my_nodes_count) + (my_nodes_count-1)]++;

        // print stuff to file:
        fprintf(fp, "%5d, %12lu, ", 
          msg->header.tracking_number,    // test #
          ipcSumLat);                     // sum of latencies

        // print the sum of, and each node's (except my own) latency
        for(uint32_t i=0 ; i<my_nodes_count-2 ; i++) {
          uint32_t nodeLat;
          memcpy(&nodeLat, &msg->data[TS_NODE_BASE + (i * 4)], sizeof(uint32_t)); 
          fprintf(fp, "%8u, ", nodeLat);
        }
        // print my link's latency
        fprintf(fp, "%8lu, ", myDiff);

        // print the start and end timestamps and their difference
        fprintf(fp, "%lu, %lu,  %11lu\n", headTime, myTime, (myTime - headTime));
        //fflush(fp);
      }
    }

    /** ----------------------------------------------------------------
     * append_stats_file()
     * calc stats on histo_bins and append data to common file.
    **/
    void append_stats_file(void)
    {

      fp = fopen(statsFileName.c_str(), "a");
      if(fp == NULL) {
          fprintf(stderr, "FILE OPEN ERROR (%s:%d)", __FILE__, __LINE__);
          return;
      }

      // basic info to enable sorting in spreadsheet: RMW type, REL, SIZE, nodes, config
      fprintf(fp, "%s,", dataFileName.c_str()); 
      fprintf(fp, "%s,%s,%u,%u,%1.1f,%s", rmw_type.c_str(), reliability_type.c_str(), 
          MP_DATA_SIZE, my_nodes_count, pub_freq, my_config.c_str());

      // find min/mean/max and count for the sum[0] and each node
      uint32_t tMin[50] = {0};
      uint32_t tMax[50] = {0};
      uint32_t tCnt[50] = {0};
      uint64_t tSum[50] = {0};
      for (uint32_t j = 0; j <= histo_bin_count; j++) {
        for(uint32_t i=0 ; i < my_nodes_count ; i++) {
          tSum[i] += (histo_bins[(j * my_nodes_count) + i]) * j;    // accumulate sample count x time(in uS)
          if(histo_bins[(j * my_nodes_count) + i]) {
            if(tMin[i] == 0) {
              tMin[i] = j;
            }
            tMax[i] = j;
            tCnt[i] += histo_bins[(j * my_nodes_count) + i];
          }
        }
      }

      // calc mean and StdDev for each
      for(uint32_t i=0 ; i< my_nodes_count ; i++) {
        double tMean = (double)tSum[i] / tCnt[i];
        // find standard deviation
        double tVar = 0;
        for (uint32_t j = 0; j < histo_bin_count; j++) {
          if (histo_bins[(j * my_nodes_count) + i]) {
            tVar += (pow(i - tMean, 2) * histo_bins[(j * my_nodes_count) + i]);
          }
        }
        double stdDev = sqrt(tVar / tCnt[i]);
        // Idx, Min, Mean, Max, stdDev, Count
        fprintf(fp, ",%u,%u,%5.2f,%u,%5.3f,%u", 
            i, tMin[i], tMean, tMax[i], stdDev, tCnt[i]);
      }

      fprintf(fp, "\n"); 
      fclose(fp);
    }

    // -----------------------------------------------------------------------
    // create a histogram output file (.csv)
    // 
    void write_histo_file()
    {
      std::string histoFileName = dataFileName + "_histo.csv";
      fp = fopen(histoFileName.c_str(), "w");
      if(fp == NULL) {
          fprintf(stderr, "FILE OPEN ERROR (%s:%d)", __FILE__, __LINE__);
          return;
      }
      fprintf(fp, "filename,%s", dataFileName.c_str());

      // first: find the highest bin with a nonzero value; suppress prints after that bin
      uint32_t hiBin = histo_bin_count;
      bool inloop=true;
      while(inloop) {
        hiBin--;
        uint32_t bIdx = (hiBin * my_nodes_count);
        for(uint32_t i=0 ; i<my_nodes_count ; i++) {
          if(histo_bins[bIdx + i]) {
            inloop=false;
          }
        }
        if(hiBin == 0) {
          inloop = false;
        }
      }
      fprintf(fp, ",hiBin,%u\n", hiBin);

      // print out the bins per the histo_bin_printres setting
      uint32_t binSum[50] = {0};  // this could be better.
      uint32_t binI = 0;
      // top-level info
      fprintf(fp, "TestsRun,%u,PrintBinsNSec,%u,DataBinsNSec,%u\n",
          sampleCount, histo_bin_printres * histo_bin_width_ns, histo_bin_width_ns);
      // table column headers
      fprintf(fp, "uSecs,    Sum");
      for(uint32_t i=0 ; i<my_nodes_count-1 ; i++) {
        fprintf(fp, ",    [%d]", i);
      }
      fprintf(fp, "\n");

      // table column data
      uint32_t maxPrint = hiBin + histo_bin_printres;
      if(maxPrint > histo_bin_count) maxPrint = histo_bin_count;
      for (uint32_t j = 0; j < maxPrint; j++) {
        for(uint32_t i=0 ; i< my_nodes_count ; i++) {
          if (histo_bins[(j * my_nodes_count) + i]) {
            binSum[i] += histo_bins[(j * my_nodes_count) + i];
          }
        }
        binI++;
        if (binI >= histo_bin_printres) {
          fprintf(fp, "%5d", j - histo_bin_printres + 1);
          for(uint32_t i=0 ; i<my_nodes_count ; i++) {
            fprintf(fp, ", %6u", binSum[i]);
            binSum[i] = 0;
          }
          fprintf(fp, "\n");
          binI = 0;
        }
      }
      fprintf(fp, "\n");
      fclose(fp);
      return;
    }

    // -----------------------------------------------------------------------
    // timer callback to check when to quit
    void timer_callback()
    {
      // get the current time
      timespec tstamp;
      tspec_get(&tstamp);

      // time to quit?
      if((tstamp.tv_sec - start_time_seconds) > run_for_seconds) {
        // close the log file
        fflush(fp);
        fclose(fp);

        // create a histogram/summary file, and append stats to commmon file.
        IpcTail::append_stats_file();
        histo_bin_printres = 50;      // sum together 10 bins when reporting.
        IpcTail::write_histo_file();

        free(histo_bins);
        rclcpp::shutdown();
      }
    }

    rclcpp::Subscription<MP_DATA_TYPE>::SharedPtr subscription_;
    rclcpp::TimerBase::SharedPtr timer_;
    FILE *fp;
    uint32_t run_for_seconds;
    uint32_t start_time_seconds;
    uint32_t my_nodes_count;
    uint32_t *histo_bins;
    uint32_t histo_bin_count;
    uint32_t histo_bin_width_ns;
    uint32_t histo_bin_printres;     // number of bins to sum-together when reporting
    uint32_t sampleCount;
    uint32_t skip_first_n_samples;
    std::string statsFileName;
    std::string dataFileName;
    std::string rmw_type;
    std::string reliability_type;
    std::string my_config;
    float pub_freq;
};

int main(int argc, char * argv[])
{
  // TAIL node gets 7 args, default values are here, in arg order:
  uint32_t testDuration = 60;           // [1] how long to run the test  (for quitting)
  bool useReliable = false;             // [2] best effort or reliable
  float pubFreq = 1.0;                  // [3] the publication frequency (for report)
  uint32_t totalNodes = 3;              // [4] how many links are in my measurement chain (+1 for sum)
  std::string fromTopic = "toTail";     // [5] topic I subscribe to
  std::string rmwType = "unknown";      // [6] RMW type used in test (for report)
  std::string myConfig = "defaultCfg";  // [7] Name of this test configuration (and this tail end)
 
  // NOTE that extra values can be passed by the ROS2 launch system
  if(argc >= 7) {
    testDuration = strtoul(argv[1], NULL, 10);
    useReliable = (!(strcmp(argv[2], "REL"))); // anything but "REL" == best_effort
    pubFreq = strtof(argv[3], NULL);
    totalNodes = strtoul(argv[4], NULL, 10);
    fromTopic = argv[5];
    rmwType  = argv[6];
    myConfig = argv[7];
  }
  else {
    fprintf(stdout, "Running with default args:\n  testDuration(%u), relType(%s), pubFreq(%1.0f), nodeCount(%u), fromTopic(%s), rmwType(%s), myConfig(%s)\n",
      testDuration, useReliable?"REL":"BE", pubFreq, totalNodes, fromTopic.c_str(), rmwType.c_str(), myConfig.c_str());
  }

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<IpcTail>(testDuration, useReliable, pubFreq, totalNodes, fromTopic, rmwType, myConfig));
  rclcpp::shutdown();
  return 0;
}