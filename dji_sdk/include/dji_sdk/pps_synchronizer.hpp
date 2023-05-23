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
    const double hardsync_time_s{1e-9 * ((hardsync_timestamp.time1ns % scaler) + (hardsync_timestamp.time2p5ms * scaler))};

    bool new_pulse_arrived{false};
    const bool pps_fetch_ok{pps_handler_.getLastRisingEdgeTime(last_rising_edge_system_time_, new_pulse_arrived)};
    pulse_arrived_since_prev_flag_ |= new_pulse_arrived;

    if (hardsync_timestamp.flag && pulse_arrived_since_prev_flag_)
    {
      pulse_arrived_since_prev_flag_      = false;
      in_use_rising_edge_system_time_     = last_rising_edge_system_time_;
      in_use_rising_edge_hardsync_time_s_ = hardsync_time_s;
      in_use_rising_edge_package_time_s_  = 1e-3 * package_timestamp.time_ms + 1e-9 * package_timestamp.time_ns;
    }
    const timespec t{pps::getSystemTime(hardsync_time_s, in_use_rising_edge_hardsync_time_s_, in_use_rising_edge_system_time_)};
    system_time_out.sec = static_cast<uint32_t>(t.tv_sec);
    system_time_out.nsec = static_cast<uint32_t>(t.tv_nsec);
    
    return pps_fetch_ok;
  }

  void getSystemTime(const DJI::OSDK::Telemetry::TimeStamp& package_timestamp, ros::Time& system_time_out)
  {
    const double package_time_s_{1e-3 * package_timestamp.time_ms + 1e-9 * package_timestamp.time_ns};
    const timespec t{pps::getSystemTime(package_time_s_, in_use_rising_edge_package_time_s_, in_use_rising_edge_system_time_)};
    system_time_out.sec = static_cast<uint32_t>(t.tv_sec);
    system_time_out.nsec = static_cast<uint32_t>(t.tv_nsec);
  }

private:
  pps::Handler pps_handler_;

  timespec last_rising_edge_system_time_;
  timespec in_use_rising_edge_system_time_;
  double in_use_rising_edge_hardsync_time_s_;
  double in_use_rising_edge_package_time_s_;

  bool pulse_arrived_since_prev_flag_;

};
  
} // namespace DJISDK
