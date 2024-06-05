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
    std::chrono::system_clock::time_point last_rising_edge_time_SYSTEM;
    const bool pps_fetch_ok{pps_handler_.getLastAssertTime(last_rising_edge_time_SYSTEM, new_pulse_arrived)};
    pulse_arrived_since_prev_flag_ |= new_pulse_arrived;

    const auto time_HARDSYNC_FC{toChronoNsecs(stamp_HARDSYNC_FC)};
    if (pps_fetch_ok && stamp_HARDSYNC_FC.flag && pulse_arrived_since_prev_flag_)
    {
      alignment_exists_                     = true;
      pulse_arrived_since_prev_flag_        = false;
      in_use_rising_edge_time_.SYSTEM       = last_rising_edge_time_SYSTEM;
      in_use_rising_edge_time_.HARDSYNC_FC  = time_HARDSYNC_FC;
      in_use_rising_edge_time_.PACKAGE_FC   = toChronoNsecs(stamp_PACKAGE_FC);
    }

    const auto time_SYSTEM{pps::getSystemTime(time_HARDSYNC_FC, in_use_rising_edge_time_.HARDSYNC_FC, in_use_rising_edge_time_.SYSTEM)};
    pps::chrono2secnsec(time_SYSTEM, time_SYSTEM_out.sec, time_SYSTEM_out.nsec);
    return alignment_exists_;
  }

  bool getSystemTime(const DJI::OSDK::Telemetry::TimeStamp& stamp_PACKAGE_FC, ros::Time& time_SYSTEM_out)
  {
    const auto time_PACKAGE_FC{toChronoNsecs(stamp_PACKAGE_FC)};
    const auto time_SYSTEM{pps::getSystemTime(time_PACKAGE_FC, in_use_rising_edge_time_.PACKAGE_FC, in_use_rising_edge_time_.SYSTEM)};
    pps::chrono2secnsec(time_SYSTEM, time_SYSTEM_out.sec, time_SYSTEM_out.nsec);
    return alignment_exists_;
  }

private:
  pps::Handler pps_handler_;
  struct
  {
    std::chrono::system_clock::time_point SYSTEM;
    std::chrono::nanoseconds HARDSYNC_FC;
    std::chrono::nanoseconds PACKAGE_FC;
  } in_use_rising_edge_time_;
  
  bool alignment_exists_;
  bool pulse_arrived_since_prev_flag_;

  static std::chrono::nanoseconds toChronoNsecs(const DJI::OSDK::Telemetry::TimeStamp& stamp_PACKAGE_FC)
  {
    /**
     * NOTE: after checking, it is evident that the field named 'time_ns' in the DJI::OSDK::Telemetry::TimeStamp-
     * struct contains a _micro_ second offset, not a _nano_ second offset. This has been found by 
     * comparing 1000000 * DJI::OSDK::Telemetry::TimeStamp::time_ms field to 1000 * DJI::OSDK::Telemetry::TimeStamp::time_ns field.
     * Both yield pretty much the same output, but DJI::OSDK::Telemetry::TimeStamp::time_ns has some added precision and is
     * thus used.
    */
    static constexpr int64_t NSECS_PER_USEC{1000};
    return std::chrono::nanoseconds{NSECS_PER_USEC * static_cast<int64_t>(stamp_PACKAGE_FC.time_ns)};
  }

  static std::chrono::nanoseconds toChronoNsecs(const DJI::OSDK::Telemetry::SyncTimestamp& stamp_HARDSYNC_FC)
  {
    static constexpr int64_t NSECS_PER_2P5MSECS{2500000};
    return std::chrono::nanoseconds{NSECS_PER_2P5MSECS * static_cast<int64_t>(stamp_HARDSYNC_FC.time2p5ms)};
  }
};
  
} // namespace DJISDK
