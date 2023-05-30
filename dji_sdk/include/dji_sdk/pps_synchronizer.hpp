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
  pulse_arrived_since_prev_flag_{false},
  in_use_pulse_offset_wrapped_nsecs_{0},
  in_use_pulse_offset_nsec_wraps_nsecs_{0}
  {}

  bool getSystemTime
  (
    const DJI::OSDK::Telemetry::SyncTimestamp& hardsync_timestamp,
    const DJI::OSDK::Telemetry::TimeStamp& package_timestamp,
    ros::Time& system_time_out
  )
  {
    bool new_pulse_arrived{false};
    const bool pps_fetch_ok{pps_handler_.getLastRisingEdgeTime(last_rising_edge_system_ts_, new_pulse_arrived)};
    pulse_arrived_since_prev_flag_ |= new_pulse_arrived;

    if (hardsync_timestamp.flag && pulse_arrived_since_prev_flag_)
    {
      pulse_arrived_since_prev_flag_      = false;
      in_use_rising_edge_system_ts_       = last_rising_edge_system_ts_;

      updateNsecOffsets(hardsync_timestamp.time1ns);
      getHardsyncTimespec(hardsync_timestamp.time2p5ms, in_use_rising_edge_hardsync_ts_);
      getPackageTimespec(package_timestamp, in_use_rising_edge_package_ts_);

      ROS_INFO_STREAM("hardsync flag received");
      ROS_INFO_STREAM("fc       {time2p5ms: " << hardsync_timestamp.time2p5ms << ", time1ns: " << hardsync_timestamp.time1ns << "}");
      ROS_INFO_STREAM("us       {in_use_pulse_offset_wrapped_nsecs: " << in_use_pulse_offset_wrapped_nsecs_ << ", in_use_pulse_offset_nsec_wraps_nsecs: " << in_use_pulse_offset_nsec_wraps_nsecs_ / NSECS_PER_2P5MSECS << "}");
      ROS_INFO_STREAM("hardsync {sec: " << in_use_rising_edge_hardsync_ts_.tv_sec << ", nsec: " << in_use_rising_edge_hardsync_ts_.tv_nsec << "}");
      ROS_INFO_STREAM("package  {sec: " << in_use_rising_edge_package_ts_.tv_sec << ", nsec: " << in_use_rising_edge_package_ts_.tv_nsec << "}");
      ROS_INFO_STREAM("pulse    {sec: " << last_rising_edge_system_ts_.tv_sec << ", nsec: " << last_rising_edge_system_ts_.tv_nsec << "}");
    }
    timespec hardsync_ts;
    getHardsyncTimespec(hardsync_timestamp.time2p5ms, hardsync_ts);

    timespec ts_out;
    pps::getSystemTime(hardsync_ts, in_use_rising_edge_hardsync_ts_, in_use_rising_edge_system_ts_, ts_out);
    system_time_out.sec = static_cast<uint32_t>(ts_out.tv_sec);
    system_time_out.nsec = static_cast<uint32_t>(ts_out.tv_nsec);
    return pps_fetch_ok;
  }

  void getSystemTime(const DJI::OSDK::Telemetry::TimeStamp& package_timestamp, ros::Time& system_time_out)
  {
    timespec package_ts, ts_out;
    getPackageTimespec(package_timestamp, package_ts);
    pps::getSystemTime(package_ts, in_use_rising_edge_package_ts_, in_use_rising_edge_system_ts_, ts_out);
    system_time_out.sec = static_cast<uint32_t>(ts_out.tv_sec);
    system_time_out.nsec = static_cast<uint32_t>(ts_out.tv_nsec);
  }

private:
  static constexpr uint64_t NSECS_PER_MSEC{1000000};
  static constexpr uint64_t NSECS_PER_2P5MSECS{2500000};
  static constexpr int64_t NSEC_WRAP_THRESHOLD{static_cast<int64_t>(NSECS_PER_2P5MSECS / 2)};

  pps::Handler pps_handler_;

  timespec last_rising_edge_system_ts_;
  timespec in_use_rising_edge_system_ts_;
  timespec in_use_rising_edge_hardsync_ts_;
  timespec in_use_rising_edge_package_ts_;
  uint64_t in_use_pulse_offset_wrapped_nsecs_;
  uint64_t in_use_pulse_offset_nsec_wraps_nsecs_;

  bool pulse_arrived_since_prev_flag_;

  static void getPackageTimespec(const DJI::OSDK::Telemetry::TimeStamp& package_timestamp, timespec& package_ts)
  {
    pps::nsecs2timespec(
     NSECS_PER_MSEC * static_cast<uint64_t>(package_timestamp.time_ms), package_ts
    );
  }

  void getHardsyncTimespec(const uint32_t hardsync_time2p5ms, timespec& hardsync_ts) const
  {
    /**
     * Formula from
     * @ref https://developer.dji.com/onboard-sdk/documentation/guides/component-guide-hardware-sync.html
    */
    pps::nsecs2timespec(
      (static_cast<uint64_t>(hardsync_time2p5ms) * NSECS_PER_2P5MSECS) + in_use_pulse_offset_wrapped_nsecs_ + in_use_pulse_offset_nsec_wraps_nsecs_,
      hardsync_ts
    );
  }

  void updateNsecOffsets(const uint32_t hardsync_time1ns)
  {
    /**
     * Formula from
     * @ref https://developer.dji.com/onboard-sdk/documentation/guides/component-guide-hardware-sync.html
    */
    const uint64_t wrapped_nsecs{static_cast<uint64_t>(hardsync_time1ns) % NSECS_PER_2P5MSECS};
    in_use_pulse_offset_nsec_wraps_nsecs_   += NSECS_PER_2P5MSECS * (static_cast<int64_t>(in_use_pulse_offset_wrapped_nsecs_) - static_cast<int64_t>(wrapped_nsecs) >= NSEC_WRAP_THRESHOLD);
    in_use_pulse_offset_wrapped_nsecs_      = wrapped_nsecs;
  }
};
  
} // namespace DJISDK
