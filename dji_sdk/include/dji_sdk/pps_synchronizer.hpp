#pragma once
#include <string>
#include <cmath>
#include <drone_pps/include/drone_pps.hpp>
#include <dji_telemetry.hpp>
#include <ros/ros.h>

namespace DJISDK
{

class Synchronizer
{
public:
  Synchronizer(const std::string& pps_dev_path, pps::Handler::CreationStatus& creation_status_out):
  pps_handler_{pps_dev_path, creation_status_out},
  pulse_arrived_since_prev_flag_{false}
  {}

  bool getSystemTime
  (
    const DJI::OSDK::Telemetry::SyncTimestamp& fc_hardsync_stamp,
    const DJI::OSDK::Telemetry::TimeStamp& fc_package_stamp,
    ros::Time& system_time_out
  )
  {
    bool new_pulse_arrived{false};
    timespec last_rising_edge_system_time;
    const bool pps_fetch_ok{pps_handler_.getLastRisingEdgeTime(last_rising_edge_system_time, new_pulse_arrived)};
    pulse_arrived_since_prev_flag_ |= new_pulse_arrived;

    timespec fc_time;
    if (fc_hardsync_stamp.flag && pulse_arrived_since_prev_flag_)
    {
      pulse_arrived_since_prev_flag_  = false;
      in_use_rising_edge_time_.system = last_rising_edge_system_time;

      getFCTimespec(fc_hardsync_stamp, in_use_rising_edge_time_.fc_hardsync);
      getFCTimespec(fc_package_stamp, in_use_rising_edge_time_.fc_package);

      ROS_INFO_STREAM("hardsync flag received");
      ROS_INFO_STREAM("fc_hardsync_stamp                {time2p5ms: " << fc_hardsync_stamp.time2p5ms << ", time1ns: " << fc_hardsync_stamp.time1ns << "}");
      ROS_INFO_STREAM("fc_package_stamp                 {time_ms: " << fc_package_stamp.time_ms << ", time_ns: " << fc_package_stamp.time_ns << "}");
      ROS_INFO_STREAM("in_use_rising_edge_time_ hardsync{sec: " << in_use_rising_edge_time_.fc_hardsync.tv_sec << ", nsec: " << in_use_rising_edge_time_.fc_hardsync.tv_nsec << "}");
      ROS_INFO_STREAM("in_use_rising_edge_time_ package {sec: " << in_use_rising_edge_time_.fc_package.tv_sec << ", nsec: " << in_use_rising_edge_time_.fc_package.tv_nsec << "}");
      ROS_INFO_STREAM("last_rising_edge_system_time     {sec: " << last_rising_edge_system_time.tv_sec << ", nsec: " << last_rising_edge_system_time.tv_nsec << "}");

      fc_time = in_use_rising_edge_time_.fc_hardsync;
    }
    else
      getFCTimespec(fc_hardsync_stamp, fc_time);

    timespec system_time;
    pps::getSystemTime(fc_time, in_use_rising_edge_time_.fc_hardsync, in_use_rising_edge_time_.system, system_time);
    system_time_out.sec   = static_cast<uint32_t>(system_time.tv_sec);
    system_time_out.nsec  = static_cast<uint32_t>(system_time.tv_nsec);
    return pps_fetch_ok;
  }

  void getSystemTime(const DJI::OSDK::Telemetry::TimeStamp& fc_package_stamp, ros::Time& system_time_out)
  {
    timespec fc_time, system_time;
    getFCTimespec(fc_package_stamp, fc_time);
    pps::getSystemTime(fc_time, in_use_rising_edge_time_.fc_package, in_use_rising_edge_time_.system, system_time);
    system_time_out.sec   = static_cast<uint32_t>(system_time.tv_sec);
    system_time_out.nsec  = static_cast<uint32_t>(system_time.tv_nsec);
  }

private:
  pps::Handler pps_handler_;
  struct
  {
    timespec system;
    timespec fc_hardsync;
    timespec fc_package;
  } in_use_rising_edge_time_;

  struct
  {
    uint32_t package_time_us;
    uint32_t hardsync_time2p5ms;
  } prev_fc_;
  
  bool pulse_arrived_since_prev_flag_;

  void getFCTimespec(const DJI::OSDK::Telemetry::TimeStamp& fc_package_stamp, timespec& fc_package_time)
  {
    /**
     * NOTE: after checking, it is evident that the field named 'time_ns' in the DJI::OSDK::Telemetry::TimeStamp-
     * struct contains a _micro_ second offset, not a _nano_ second offset. This has been found by 
     * comparing 1000000 * DJI::OSDK::Telemetry::TimeStamp::time_ms field to 1000 * DJI::OSDK::Telemetry::TimeStamp::time_ns field.
     * Both yield pretty much the same output, but DJI::OSDK::Telemetry::TimeStamp::time_ns has some added precision and is
     * thus used.
    */
    static constexpr uint64_t NSECS_PER_USEC{1000};
    pps::nsecs2timespec(
      NSECS_PER_USEC * getOverflowCompensated(fc_package_stamp.time_ns, prev_fc_.package_time_us),
      fc_package_time
    );
    prev_fc_.package_time_us = fc_package_stamp.time_ns;
  }

  void getFCTimespec(const DJI::OSDK::Telemetry::SyncTimestamp& fc_hardsync_stamp, timespec& fc_hardsync_time)
  {
    static constexpr uint64_t NSECS_PER_2P5MSECS{2500000};
    pps::nsecs2timespec(
      NSECS_PER_2P5MSECS * getOverflowCompensated(fc_hardsync_stamp.time2p5ms, prev_fc_.hardsync_time2p5ms),
      fc_hardsync_time
    );
    prev_fc_.hardsync_time2p5ms = fc_hardsync_stamp.time2p5ms;
  }

  static uint64_t getOverflowCompensated(
    const uint32_t curr,
    const uint32_t prev
  )
  {
    const uint64_t overflow{curr < prev};
    const uint64_t out{static_cast<uint64_t>(curr) + overflow * (static_cast<uint64_t>((std::numeric_limits<uint32_t>::max() - prev)) + 1ul)};
    ROS_INFO_STREAM_COND(overflow, "[overflow] {prev: " << prev << ", curr: " << curr << "} -> out: " << out);
    return out;
  }
};
  
} // namespace DJISDK
