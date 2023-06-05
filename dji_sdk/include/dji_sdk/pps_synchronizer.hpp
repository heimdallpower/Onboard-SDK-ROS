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
  alignment_exists_{false},
  pulse_arrived_since_prev_flag_{false}
  {}

  bool getSystemTime
  (
    const DJI::OSDK::Telemetry::SyncTimestamp& stamp_HARDSYNC_FC,
    const DJI::OSDK::Telemetry::TimeStamp& stamp_PACKAGE_FC,
    ros::Time& time_SYSTEM_out
  )
  {
    bool new_pulse_arrived{false};
    timespec last_rising_edge_time_SYSTEM;
    const bool pps_fetch_ok{pps_handler_.getLastRisingEdgeTime(last_rising_edge_time_SYSTEM, new_pulse_arrived)};
    pulse_arrived_since_prev_flag_ |= new_pulse_arrived;

    timespec time_HARDSYNC_FC;
    if (pps_fetch_ok && stamp_HARDSYNC_FC.flag && pulse_arrived_since_prev_flag_)
    {
      alignment_exists_               = true;
      pulse_arrived_since_prev_flag_  = false;
      in_use_rising_edge_time_.SYSTEM = last_rising_edge_time_SYSTEM;

      getFCTimespec(stamp_HARDSYNC_FC, in_use_rising_edge_time_.HARDSYNC_FC);
      getFCTimespec(stamp_PACKAGE_FC, in_use_rising_edge_time_.PACKAGE_FC);

      time_HARDSYNC_FC = in_use_rising_edge_time_.HARDSYNC_FC;
    }
    else
      getFCTimespec(stamp_HARDSYNC_FC, time_HARDSYNC_FC);

    timespec time_SYSTEM;
    pps::getSystemTime(time_HARDSYNC_FC, in_use_rising_edge_time_.HARDSYNC_FC, in_use_rising_edge_time_.SYSTEM, time_SYSTEM);
    time_SYSTEM_out.sec   = static_cast<uint32_t>(time_SYSTEM.tv_sec);
    time_SYSTEM_out.nsec  = static_cast<uint32_t>(time_SYSTEM.tv_nsec);
    return alignment_exists_;
  }

  bool getSystemTime(const DJI::OSDK::Telemetry::TimeStamp& stamp_PACKAGE_FC, ros::Time& time_SYSTEM_out)
  {
    timespec time_PACKAGE_FC, time_SYSTEM;
    getFCTimespec(stamp_PACKAGE_FC, time_PACKAGE_FC);
    pps::getSystemTime(time_PACKAGE_FC, in_use_rising_edge_time_.PACKAGE_FC, in_use_rising_edge_time_.SYSTEM, time_SYSTEM);
    time_SYSTEM_out.sec   = static_cast<uint32_t>(time_SYSTEM.tv_sec);
    time_SYSTEM_out.nsec  = static_cast<uint32_t>(time_SYSTEM.tv_nsec);
    return alignment_exists_;
  }

private:
  pps::Handler pps_handler_;
  struct
  {
    timespec SYSTEM;
    timespec HARDSYNC_FC;
    timespec PACKAGE_FC;
  } in_use_rising_edge_time_;

  struct
  {
    uint32_t package_time_us;
    uint32_t hardsync_time2p5ms;
  } prev_fc_;
  
  bool alignment_exists_;
  bool pulse_arrived_since_prev_flag_;

  void getFCTimespec(const DJI::OSDK::Telemetry::TimeStamp& stamp_PACKAGE_FC, timespec& time_PACKAGE_FC)
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
      NSECS_PER_USEC * getOverflowCompensated(stamp_PACKAGE_FC.time_ns, prev_fc_.package_time_us),
      time_PACKAGE_FC
    );
    prev_fc_.package_time_us = stamp_PACKAGE_FC.time_ns;
  }

  void getFCTimespec(const DJI::OSDK::Telemetry::SyncTimestamp& stamp_HARDSYNC_FC, timespec& time_HARDSYNC_FC)
  {
    static constexpr uint64_t NSECS_PER_2P5MSECS{2500000};
    pps::nsecs2timespec(
      NSECS_PER_2P5MSECS * getOverflowCompensated(stamp_HARDSYNC_FC.time2p5ms, prev_fc_.hardsync_time2p5ms),
      time_HARDSYNC_FC
    );
    prev_fc_.hardsync_time2p5ms = stamp_HARDSYNC_FC.time2p5ms;
  }

  static uint64_t getOverflowCompensated(
    const uint32_t curr,
    const uint32_t prev
  )
  {
    const uint64_t overflow{curr < prev};
    const uint64_t out{static_cast<uint64_t>(curr) + overflow * (static_cast<uint64_t>((std::numeric_limits<uint32_t>::max() - prev)) + 1ul)};
    return out;
  }
};
  
} // namespace DJISDK
