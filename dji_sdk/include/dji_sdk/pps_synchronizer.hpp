#pragma once
#include <string>
#include <cmath>
#include <boost/chrono/round.hpp>
#include <drone_pps/include/drone_pps.hpp>
#include <dji_telemetry.hpp>
#include <ros/ros.h>

namespace DJISDK
{

class Synchronizer
{
public:
  Synchronizer
  (
    const std::string& pps_dev_path,
    const double pps_window_half_width_sec,
    pps::Handler::CreationStatus& creation_status_out
  ):
  pps_handler_{pps_dev_path, creation_status_out},
  pps_window_half_width_nsec_{static_cast<boost::chrono::seconds::rep>(pps_window_half_width_sec * S2NS)},
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
    const bool new_pulse_in_expected_window{new_pulse_arrived && isPulseInExpectedWindow(last_rising_edge_time_SYSTEM, in_use_rising_edge_time_.SYSTEM)};
    const bool accept_pulse{!alignment_exists_ || new_pulse_in_expected_window};
    pulse_arrived_since_prev_flag_ |= accept_pulse;
    if (!accept_pulse)
      ROS_ERROR_STREAM("[dji_sdk Synchronizer] Denied PPS pulse.");
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
  static constexpr boost::chrono::seconds::rep S2NS{1000000000ll};

  pps::Handler pps_handler_;
  struct
  {
    std::chrono::system_clock::time_point SYSTEM;
    std::chrono::nanoseconds HARDSYNC_FC;
    std::chrono::nanoseconds PACKAGE_FC;
  } in_use_rising_edge_time_;

  const boost::chrono::seconds::rep pps_window_half_width_nsec_;
  
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

  bool isPulseInExpectedWindow
  (
    const std::chrono::system_clock::time_point& curr_pulse_time,
    const std::chrono::system_clock::time_point& prev_valid_pulse_time
  ) const
  {
    ROS_INFO_STREAM((curr_pulse_time - prev_valid_pulse_time).count() * 1e-9);
    const boost::chrono::nanoseconds diff{(curr_pulse_time - prev_valid_pulse_time).count()};
    const auto diff_nearest_seconds{boost::chrono::round<boost::chrono::seconds>(diff)};
    const auto diff_lag_nsec{diff - boost::chrono::duration_cast<boost::chrono::nanoseconds>(diff_nearest_seconds)};
    const auto diff_num_seconds{diff_nearest_seconds.count()};
    const bool pulse_in_expected_window{std::abs(diff_lag_nsec.count()) < pps_window_half_width_nsec_ * diff_num_seconds};
    return pulse_in_expected_window;
  }
};
  
} // namespace DJISDK
