#pragma once
#include <string>
#include <cmath>
#include <drone_pps/include/drone_pps.hpp>
#include <dji_telemetry.hpp>

namespace DJISDK
{

class Synchronizer
{
public:
  Synchronizer(const std::string& pps_dev_path, pps::Handler::CreationStatus& creation_status_out):
  pps_handler_{pps_dev_path, creation_status_out},
  pulse_arrived_since_prev_flag_{false}
  {}

  bool getSystemTime(const DJI::OSDK::Telemetry::SyncTimestamp& hardsync_timestamp, const DJI::OSDK::Telemetry::TimeStamp& package_timestamp, ros::Time& system_time_out)
  {
    static constexpr uint32_t scaler{static_cast<uint32_t>(2.5e6)};
    /**
     * Formula from
     * @ref https://developer.dji.com/onboard-sdk/documentation/guides/component-guide-hardware-sync.html
    */
    constexpr uint32_t SCALER{2500000};
    timespec hardsync_ts;
    pps::nsecs2timespec(
      static_cast<uint64_t>(hardsync_timestamp.time1ns % SCALER) + static_cast<uint64_t>(hardsync_timestamp.time2p5ms * SCALER),
      hardsync_ts
    );

    bool new_pulse_arrived{false};
    const bool pps_fetch_ok{pps_handler_.getLastRisingEdgeTime(last_rising_edge_system_ts_, new_pulse_arrived)};
    pulse_arrived_since_prev_flag_ |= new_pulse_arrived;

    if (hardsync_timestamp.flag && pulse_arrived_since_prev_flag_)
    {
      pulse_arrived_since_prev_flag_      = false;
      in_use_rising_edge_system_ts_       = last_rising_edge_system_ts_;
      in_use_rising_edge_hardsync_ts_     = hardsync_ts;
      getPackageTimespec(package_timestamp, in_use_rising_edge_package_ts_);
    }
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
  pps::Handler pps_handler_;

  timespec last_rising_edge_system_ts_;
  timespec in_use_rising_edge_system_ts_;
  timespec in_use_rising_edge_hardsync_ts_;
  timespec in_use_rising_edge_package_ts_;

  bool pulse_arrived_since_prev_flag_;

  static void getPackageTimespec(const DJI::OSDK::Telemetry::TimeStamp& package_timestamp, timespec& package_ts)
  {
    constexpr uint64_t MILLION{1000000};
    pps::nsecs2timespec(
      MILLION * static_cast<uint64_t>(package_timestamp.time_ms) + static_cast<uint64_t>(package_timestamp.time_ns),
      package_ts
    );
  }

};
  
} // namespace DJISDK
