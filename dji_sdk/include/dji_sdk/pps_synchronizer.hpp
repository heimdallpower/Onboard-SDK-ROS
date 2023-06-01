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
  prev_package_time_us_FC_{0},
  pulse_arrived_{false}
  {}

  bool getSystemTime
  (
    const bool stamp_FC_corresponds_to_rising_edge,
    const DJI::OSDK::Telemetry::TimeStamp& stamp_FC,
    ros::Time& time_SYSTEM_out
  )
  {
    bool new_pulse_arrived{false};
    timespec last_rising_edge_time_SYSTEM;
    const bool pps_fetch_ok{pps_handler_.getLastRisingEdgeTime(last_rising_edge_time_SYSTEM, new_pulse_arrived)};
    pulse_arrived_ |= new_pulse_arrived;

    if (stamp_FC_corresponds_to_rising_edge && pulse_arrived_)
    {
      pulse_arrived_  = false;
      in_use_rising_edge_time_.SYSTEM = last_rising_edge_time_SYSTEM;
      getFCTimespec(stamp_FC, in_use_rising_edge_time_.FC);
    }
    getSystemTime(stamp_FC, time_SYSTEM_out);
    return pps_fetch_ok;
  }

  void getSystemTime(const DJI::OSDK::Telemetry::TimeStamp& stamp_FC, ros::Time& time_SYSTEM_out)
  {
    timespec time_FC, time_SYSTEM;
    getFCTimespec(stamp_FC, time_FC);
    pps::getSystemTime(time_FC, in_use_rising_edge_time_.FC, in_use_rising_edge_time_.SYSTEM, time_SYSTEM);
    time_SYSTEM_out.sec   = static_cast<uint32_t>(time_SYSTEM.tv_sec);
    time_SYSTEM_out.nsec  = static_cast<uint32_t>(time_SYSTEM.tv_nsec);
  }

private:
  pps::Handler pps_handler_;
  struct
  {
    timespec SYSTEM;
    timespec FC;
  } in_use_rising_edge_time_;

  uint32_t prev_package_time_us_FC_;
  bool pulse_arrived_;

  void getFCTimespec(const DJI::OSDK::Telemetry::TimeStamp& stamp_FC, timespec& time_FC_out)
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
      NSECS_PER_USEC * getOverflowCompensated(stamp_FC.time_ns, prev_package_time_us_FC_),
      time_FC_out
    );
    prev_package_time_us_FC_ = stamp_FC.time_ns;
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
