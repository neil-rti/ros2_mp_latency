/** --------------------------------------------------------------
 * IpcTestDefs.hpp
 * for IPC test app
 **/
#ifndef __IPCTEST_HPP__
#define __IPCTEST_HPP__

// MP_DATA_SIZE is a compile-time option
#if MP_DATA_SIZE == 100
#include "irobot_interfaces_plugin/msg/stamped100b.hpp"
#define MP_DATA_TYPE  irobot_interfaces_plugin::msg::Stamped100b
#elif MP_DATA_SIZE == 1024
#include "irobot_interfaces_plugin/msg/stamped1kb.hpp"
#define MP_DATA_TYPE  irobot_interfaces_plugin::msg::Stamped1kb
#elif MP_DATA_SIZE == 4096
#include "irobot_interfaces_plugin/msg/stamped4kb.hpp"
#define MP_DATA_TYPE  irobot_interfaces_plugin::msg::Stamped4kb
#elif MP_DATA_SIZE == 10240
#include "irobot_interfaces_plugin/msg/stamped10kb.hpp"
#define MP_DATA_TYPE  irobot_interfaces_plugin::msg::Stamped10kb
#elif MP_DATA_SIZE == 16384
#include "irobot_interfaces_plugin/msg/stamped16kb.hpp"
#define MP_DATA_TYPE  irobot_interfaces_plugin::msg::Stamped16kb
#elif MP_DATA_SIZE == 32768
#include "irobot_interfaces_plugin/msg/stamped32kb.hpp"
#define MP_DATA_TYPE  irobot_interfaces_plugin::msg::Stamped32kb
#elif MP_DATA_SIZE == 61440
#include "irobot_interfaces_plugin/msg/stamped60kb.hpp"
#define MP_DATA_TYPE  irobot_interfaces_plugin::msg::Stamped60kb
#elif MP_DATA_SIZE == 102400
#include "irobot_interfaces_plugin/msg/stamped100kb.hpp"
#define MP_DATA_TYPE  irobot_interfaces_plugin::msg::Stamped100kb
#elif MP_DATA_SIZE == 262144
#include "irobot_interfaces_plugin/msg/stamped256kb.hpp"
#define MP_DATA_TYPE  irobot_interfaces_plugin::msg::Stamped256kb
#elif MP_DATA_SIZE == 512000
#include "irobot_interfaces_plugin/msg/stamped500kb.hpp"
#define MP_DATA_TYPE  irobot_interfaces_plugin::msg::Stamped500kb
#elif MP_DATA_SIZE == 1048576
#include "irobot_interfaces_plugin/msg/stamped1mb.hpp"
#define MP_DATA_TYPE  irobot_interfaces_plugin::msg::Stamped1mb
#elif MP_DATA_SIZE == 2097152
#include "irobot_interfaces_plugin/msg/stamped2mb.hpp"
#define MP_DATA_TYPE  irobot_interfaces_plugin::msg::Stamped2mb
#elif MP_DATA_SIZE == 4194304
#include "irobot_interfaces_plugin/msg/stamped4mb.hpp"
#define MP_DATA_TYPE  irobot_interfaces_plugin::msg::Stamped4mb
#elif MP_DATA_SIZE == 8388608
#include "irobot_interfaces_plugin/msg/stamped8mb.hpp"
#define MP_DATA_TYPE  irobot_interfaces_plugin::msg::Stamped8mb
#else
#error MP_DATA_SIZE not defined or unsupported size
#endif

/* The above test payloads are BYTE ARRAYS; these are used to carry
   timing information as the data samples make their way through the 
   test network -- kind of like a 'tracert': each node adds their ID
   and interval timing to the data array before sending the data.
   The following defines where each node should place its data 
*/
#define SOURCE_TS_OFS     (0)     // starting timestamp (from SOURCE node)
#define SOURCE_TS_LEN     (8)     // 8-byte timestamp
#define PREV_TS_OFS     (SOURCE_TS_OFS + SOURCE_TS_LEN)     // timestamp of previous node (for diff timing)
#define PREV_TS_LEN     (8)
#define TS_IDX_OFS      (PREV_TS_OFS + PREV_TS_LEN)     // index into array of 
#define TS_IDX_LEN      (4)     // its only 1 byte, but keep 4-byte alignment
#define TS_NODE_BASE    (TS_IDX_OFS + TS_IDX_LEN)       // base of node-timing array


// tstamp_get(): utility timestamp getter, returns uint64_t
uint64_t tstamp_get(void)
{
    timespec tStamp;
#ifdef WIN32
    timespec_get(&tStamp, TIME_UTC);
#else
    clock_gettime(CLOCK_REALTIME, &tStamp);
#endif
    return ((tStamp.tv_sec * 1000000000) + tStamp.tv_nsec);
}

// tspec_get(): updates *timespec with current time
void tspec_get(timespec *tStamp)
{
#ifdef WIN32
    timespec_get(tStamp, TIME_UTC);
#else
    clock_gettime(CLOCK_REALTIME, tStamp);
#endif
    return;
}

#endif  // __IPCTEST_HPP__
