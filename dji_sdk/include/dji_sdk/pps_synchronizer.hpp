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
  valid_pulse_arrived_since_prev_flag_{false},
  allow_realign_{false}
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

    static std::chrono::system_clock::time_point prev_rising_edge_time_SYSTEM;
    static size_t good_pulsetrain_length_{0u};
    if (new_pulse_arrived)
    {
      constexpr int_least64_t REALIGN_ACCEPTABLE_NSEC_DIFF{static_cast<int_least64_t>(0.0001 * S2NS)};
      constexpr size_t MIN_GOOD_PULSETRAIN_LENGTH{5};

      int_least64_t prev_pulse_diff_num_seconds;
      int_least64_t prev_pulse_diff_lag_nsec;
      getTimeDiff(last_rising_edge_time_SYSTEM, prev_rising_edge_time_SYSTEM, prev_pulse_diff_num_seconds, prev_pulse_diff_lag_nsec);
      const size_t diff_ok{static_cast<size_t>(prev_pulse_diff_num_seconds == 1ll && (std::abs(prev_pulse_diff_lag_nsec) < REALIGN_ACCEPTABLE_NSEC_DIFF))};
      good_pulsetrain_length_ = diff_ok * (good_pulsetrain_length_ + diff_ok);
      ROS_INFO_STREAM("[dji_sdk Synchronizer] good_pulsetrain_length_=" << good_pulsetrain_length_ << ".");

      prev_rising_edge_time_SYSTEM = last_rising_edge_time_SYSTEM;

      boost::chrono::nanoseconds time_since_prev_good_pulse;
      const bool pulse_in_expected_window{isPulseInExpectedWindow(last_rising_edge_time_SYSTEM, in_use_rising_edge_time_.SYSTEM, time_since_prev_good_pulse)};
      const bool do_realign{!pulse_in_expected_window && (allow_realign_ && (good_pulsetrain_length_ >= MIN_GOOD_PULSETRAIN_LENGTH))};
      const bool accept_new_pulse{
        !alignment_exists_ ||
        pulse_in_expected_window ||
        do_realign
      };

      valid_pulse_arrived_since_prev_flag_ |= accept_new_pulse;
      ROS_WARN_STREAM_COND(!pulse_in_expected_window, "[dji_sdk Synchronizer] New pulse outside of permitted window. New pulse came " << time_since_prev_good_pulse.count() * 1e-9 << " secs after previous good pulse.");
      ROS_WARN_STREAM_COND(do_realign, "[dji_sdk Synchronizer] Accepting offset pulse due to sufficiently long good pulsetrain (good_pulsetrain_length_=" << good_pulsetrain_length_ << ").");
      ROS_WARN_STREAM_COND(accept_new_pulse, "[dji_sdk Synchronizer] Accepting new pulse " << time_since_prev_good_pulse.count() * 1e-9 << " secs after previous good pulse.");
    }

    const auto time_HARDSYNC_FC{toChronoNsecs(stamp_HARDSYNC_FC)};
    if (pps_fetch_ok && stamp_HARDSYNC_FC.flag && valid_pulse_arrived_since_prev_flag_)
    {
      alignment_exists_                     = true;
      valid_pulse_arrived_since_prev_flag_  = false;
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

  void setAllowReAlign(const bool allow_realign) { allow_realign_ = allow_realign; }

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
  bool valid_pulse_arrived_since_prev_flag_;
  bool allow_realign_;

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
    const std::chrono::system_clock::time_point& prev_valid_pulse_time,
    boost::chrono::nanoseconds& diff_out
  ) const
  {
    int_least64_t diff_num_seconds;
    int_least64_t diff_lag_nsec;
    diff_out = getTimeDiff(curr_pulse_time, prev_valid_pulse_time, diff_num_seconds, diff_lag_nsec);
    const bool pulse_in_expected_window{std::abs(diff_lag_nsec) < pps_window_half_width_nsec_ * diff_num_seconds};
    return pulse_in_expected_window;
  }

  /**
   * Computers diff_quotient & diff_remainder such that
   * a = b + diff_quotient_sec + (diff_remainder_nsec / 1'000'000'000)
  */
  template<typename TimeType>
  static boost::chrono::nanoseconds getTimeDiff
  (
    const TimeType& a,
    const TimeType& b,
    int_least64_t& diff_quotient_sec,
    int_least64_t& diff_remainder_nsec
  )
  {
    const boost::chrono::nanoseconds diff{(a - b).count()};
    const boost::chrono::seconds tmp{boost::chrono::round<boost::chrono::seconds>(diff)};
    diff_quotient_sec   = tmp.count();
    diff_remainder_nsec = (diff - boost::chrono::duration_cast<boost::chrono::nanoseconds>(tmp)).count();
    return diff;
  }
};
  
} // namespace DJISDK
