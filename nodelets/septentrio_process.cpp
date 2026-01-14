// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief The septentrio_gnss_driver publishes various DO-NOT-USE values in its output, which are usually -2*10^10.
 * Such values are unsuitable for downstream ROS processing. This node filters these values out and replaces them with
 * more suitable values, or throws away whole messages if they would be invalid from the ROS point of view.
 *
 * It also creates alternative message types from the attitude message: compass_msgs/Azimuth (for generic azimuth
 * processing nodes) and sensor_msgs/Imu (for robot_localization). The IMU message contains only ENU heading (and maybe
 * pitch), but the third angle is always reported as zero. Covariance of the invalid angles is set to (2*pi)^2. Also,
 * the angular velocities are added to this IMU message. Accelerations are not used.
 */

#include <algorithm>
#include <functional>
#include <limits>
#include <memory>
#include <string>
#include <utility>

#include <Eigen/Dense>

#ifndef ROS2

#include <compass_msgs/Azimuth.h>
#include <cras_cpp_common/nodelet_utils.hpp>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <gps_common/GPSFix.h>
#include <nmea_msgs/Gpgga.h>
#include <nmea_msgs/Gprmc.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.hpp>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/TimeReference.h>
#include <septentrio_gnss_driver/AIMPlusStatus.h>
#include <septentrio_gnss_driver/AttEuler.h>
#include <septentrio_gnss_driver/AttCovEuler.h>
#include <septentrio_gnss_driver/PosCovGeodetic.h>
#include <septentrio_gnss_driver/PVTGeodetic.h>
#include <septentrio_gnss_driver/RFStatus.h>
#include <septentrio_gnss_driver/VelCovGeodetic.h>
#include <tf2/convert.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#else

#include <compass_interfaces/msg/azimuth.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <gps_msgs/msg/gps_fix.hpp>
#include <nmea_msgs/msg/gpgga.hpp>
#include <nmea_msgs/msg/gprmc.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/time_reference.hpp>
#include <septentrio_gnss_driver/msg/aim_plus_status.hpp>
#include <septentrio_gnss_driver/msg/att_euler.hpp>
#include <septentrio_gnss_driver/msg/att_cov_euler.hpp>
#include <septentrio_gnss_driver/msg/pos_cov_geodetic.hpp>
#include <septentrio_gnss_driver/msg/pvt_geodetic.hpp>
#include <septentrio_gnss_driver/msg/rf_status.hpp>
#include <septentrio_gnss_driver/msg/vel_cov_geodetic.hpp>
#include <tf2/convert.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#endif

#ifndef ROS2
using compass_msgs::Azimuth;
using gps_common::GPSFix;
using nmea_msgs::Gpgga;
using nmea_msgs::Gprmc;
using sensor_msgs::Imu;
using sensor_msgs::NavSatFix;
using sensor_msgs::NavSatStatus;
using sensor_msgs::TimeReference;
using septentrio_gnss_driver::AIMPlusStatus;
using septentrio_gnss_driver::AttCovEulerConstPtr;
using septentrio_gnss_driver::AttCovEuler;
using septentrio_gnss_driver::AttEulerConstPtr;
using septentrio_gnss_driver::AttEuler;
using septentrio_gnss_driver::PosCovGeodetic;
using septentrio_gnss_driver::PVTGeodetic;
using septentrio_gnss_driver::RFStatus;
using septentrio_gnss_driver::VelCovGeodetic;

#define UPDATE_THREAD_NAME this->updateThreadName();

template<typename T> using Publisher = ros::Publisher;
template<typename T> using Subscriber = ros::Subscriber;

#else

using compass_interfaces::msg::Azimuth;
using gps_msgs::msg::GPSFix;
using nmea_msgs::msg::Gpgga;
using nmea_msgs::msg::Gprmc;
using sensor_msgs::msg::Imu;
using sensor_msgs::msg::NavSatFix;
using sensor_msgs::msg::NavSatStatus;
using sensor_msgs::msg::TimeReference;
using septentrio_gnss_driver::msg::AIMPlusStatus;
using septentrio_gnss_driver::msg::AttCovEuler;
using AttCovEulerConstPtr = AttCovEuler::ConstSharedPtr;
using septentrio_gnss_driver::msg::AttEuler;
using AttEulerConstPtr = AttEuler::ConstSharedPtr;
using septentrio_gnss_driver::msg::PosCovGeodetic;
using septentrio_gnss_driver::msg::PVTGeodetic;
using septentrio_gnss_driver::msg::RFStatus;
using septentrio_gnss_driver::msg::VelCovGeodetic;

#define UPDATE_THREAD_NAME

template<typename T> struct Publisher : rclcpp::Publisher<T>::SharedPtr
{
  Publisher() : rclcpp::Publisher<T>::SharedPtr(nullptr) {}
  Publisher(const Publisher& other) :
    rclcpp::Publisher<T>::SharedPtr(static_cast<const rclcpp::Publisher<T>::SharedPtr&>(other)) {}
  Publisher(Publisher&& other) noexcept :
    rclcpp::Publisher<T>::SharedPtr(static_cast<rclcpp::Publisher<T>::SharedPtr&&>(other)) {}
  Publisher& operator=(const Publisher& shared)
  {
    if (this != &shared)
      static_cast<rclcpp::Publisher<T>::SharedPtr&>(*this) =
        static_cast<const rclcpp::Publisher<T>::SharedPtr&>(shared);
    return *this;
  }
  Publisher& operator=(Publisher&& shared) noexcept
  {
    if (this != &shared)
      static_cast<rclcpp::Publisher<T>::SharedPtr&>(*this) = static_cast<rclcpp::Publisher<T>::SharedPtr&&>(shared);
    return *this;
  }

  Publisher(const rclcpp::Publisher<T>::SharedPtr& other) : rclcpp::Publisher<T>::SharedPtr(other) {}  // NOLINT
  template<typename M> void publish(const M& val) const { this->get()->publish(std::move(val)); }
};

template<typename T> using Subscriber = typename rclcpp::Subscription<T>::SharedPtr;

#endif

namespace cras
{
struct SeptentrioProcess :
#ifndef ROS2
  public cras::Nodelet
#else
  public rclcpp::Node
#endif
{
#ifndef ROS2
  void onInit() override
  {
    this->nh = this->getNodeHandle();
    this->pnh = this->getPrivateNodeHandle();

    auto params = this->privateParams();
#else
  explicit SeptentrioProcess(const rclcpp::NodeOptions& options) : Node("cras_septentrio_process", options)
  {
    auto* params = this;
#endif
    // These maximum errors are used instead of do-not-use values or too large errors.
    this->maxPosErr = params->getParam("max_position_error", this->maxPosErr, "m");
    this->maxAltErr = params->getParam("max_altitude_error", this->maxAltErr, "m");
    this->maxAngErr = params->getParam("max_orientation_error", this->maxAngErr, "rad");
    this->maxAngCov = this->maxAngErr * this->maxAngErr;
    this->maxHorzVelErr = params->getParam("max_horizontal_velocity_error", this->maxHorzVelErr, "m/s");
    this->maxVertVelErr = params->getParam("max_vertical_velocity_error", this->maxVertVelErr, "m/s");
    // Max clock bias converted to error in velocity determination
    this->maxClkBiasErr = params->getParam("max_clock_bias_error", this->maxClkBiasErr, "m/s");

    // These thresholds decide acceptable errors for letting the data through marked as valid for navigation.
    this->validPosErrorThreshold = params->getParam(
      "valid_position_error_threshold", this->validPosErrorThreshold, "m");
    this->validHeadingErrorThresholdDeg = params->getParam(
      "valid_heading_error_threshold_deg", this->validHeadingErrorThresholdDeg, "deg");

    // These thresholds tell the minimum covariance that will be present in valid fix, fix_detail and heading messages.
    this->minPosErr = params->getParam("min_position_error", this->minPosErr, "m");
    this->minAltErr = params->getParam("min_altitude_error", this->minAltErr, "m");
    this->minHeadingErrDeg = params->getParam("min_heading_error_deg", this->minHeadingErrDeg, "deg");

    this->publishInvalidFix = params->getParam("publish_invalid_fix", this->publishInvalidFix);
    this->publishInvalidHeading = params->getParam("publish_invalid_heading", this->publishInvalidHeading);

    this->aimPlusStatusPub = this->advertise<AIMPlusStatus>("aimplusstatus", 10);
    this->aimPlusStatusSub = this->subscribe<AIMPlusStatus>(
      "raw/aimplusstatus", 10, &SeptentrioProcess::processAimPlusStatus, this);

    this->attEulerPub = this->advertise<AttEuler>("atteuler", 10);
    this->attEulerSub = this->subscribe<AttEuler>("raw/atteuler", 10, &SeptentrioProcess::processAtt, this);

    this->attCovEulerPub = this->advertise<AttCovEuler>("attcoveuler", 10);
    this->attCovEulerSub = this->subscribe<AttCovEuler>("raw/attcoveuler", 10, &SeptentrioProcess::processAttCov, this);

    this->fixPub = this->advertise<NavSatFix>("fix", 10);
    this->fixSub = this->subscribe<NavSatFix>("raw/fix", 10, &SeptentrioProcess::processFix, this);

    this->fixDetailPub = this->advertise<GPSFix>("fix_detail", 10);
    this->fixDetailSub = this->subscribe<GPSFix>("raw/fix_detail", 10, &SeptentrioProcess::processFixDetail, this);

    this->gpggaPub = this->advertise<Gpgga>("gpgga", 10);
    this->gpggaSub = this->subscribe<Gpgga>("raw/gpgga", 10, &SeptentrioProcess::processGpgga, this);

    this->gprmcPub = this->advertise<Gprmc>("gprmc", 10);
    this->gprmcSub = this->subscribe<Gprmc>("raw/gprmc", 10, &SeptentrioProcess::processGprmc, this);

    this->gpstPub = this->advertise<TimeReference>("gpst", 10);
    this->gpstSub = this->subscribe<TimeReference>("raw/gpst", 10, &SeptentrioProcess::processGpst, this);

    this->posCovGeodeticPub = this->advertise<PosCovGeodetic>("poscovgeodetic", 10);
    this->posCovGeodeticSub = this->subscribe<PosCovGeodetic>(
      "raw/poscovgeodetic", 10, &SeptentrioProcess::processPosCovGeodetic, this);

    this->pvtGeodeticPub = this->advertise<PVTGeodetic>("pvtgeodetic", 10);
    this->pvtGeodeticSub = this->subscribe<PVTGeodetic>(
      "raw/pvtgeodetic", 10, &SeptentrioProcess::processPVTGeodetic, this);

    this->rfStatusPub = this->advertise<RFStatus>("rfstatus", 10);
    this->rfStatusSub = this->subscribe<RFStatus>("raw/rfstatus", 10, &SeptentrioProcess::processRFStatus, this);

    this->velCovGeodeticPub = this->advertise<VelCovGeodetic>("velcovgeodetic", 10);
    this->velCovGeodeticSub = this->subscribe<VelCovGeodetic>(
      "raw/velcovgeodetic", 10, &SeptentrioProcess::processVelCovGeodetic, this);

    this->azimuthPub = this->advertise<Azimuth>("azimuth", 10);
    this->imuPub = this->advertise<Imu>("azimuth_imu", 10);
  }

#ifndef ROS2
  template <typename T>
  ros::Publisher advertise(const std::string& topic, const size_t queueSize)
  {
    return this->nh.advertise<T>(topic, queueSize);
  }

  template <typename T, typename Fn>
  ros::Subscriber subscribe(
    const std::string& topic, const size_t queueSize, const Fn& fn, SeptentrioProcess* self)
  {
    return this->nh.subscribe(topic, queueSize, fn, self);
  }
#else
  template <typename T>
  T getParam(const std::string& name, const T& defaultVal, const std::string& unit = "")
  {
    if (this->has_parameter(name))
      this->undeclare_parameter(name);

    try
    {
      const auto& result = this->declare_parameter<T>(name, defaultVal);
      RCLCPP_INFO(this->get_logger(), "param %s = %s", name.c_str(), std::to_string(result).c_str());
      return result;
    }
    catch (const std::runtime_error& e)
    {
      RCLCPP_WARN(this->get_logger(), "%s", e.what());
      throw;
    }
  }

  template <typename T>
  typename rclcpp::Publisher<T>::SharedPtr advertise(const std::string& topic, const size_t queueSize)
  {
    return this->create_publisher<T>(topic, rclcpp::SystemDefaultsQoS().keep_last(queueSize));
  }

  template <typename T, typename Fn>
  typename rclcpp::Subscription<T>::SharedPtr subscribe(
    const std::string& topic, const size_t queueSize, const Fn& fn, SeptentrioProcess* self)
  {
    return this->create_subscription<T>(
      topic, rclcpp::SystemDefaultsQoS().keep_last(queueSize), std::bind(fn, self, std::placeholders::_1));
  }
#endif

  template<typename T>
  bool fixNan(T& val) const
  {
    if (!std::isnan(val) && val < -1000000000.0)
      val = std::numeric_limits<T>::quiet_NaN();
    return !std::isfinite(val);
  }

  template<typename T>
  bool fixCov(T& val, const T& maxCov) const
  {
    if (std::isnan(val) || val < -1000000000.0)
      val = maxCov;
    return val >= maxCov;
  }

  template<typename T>
  bool fixCov(T& val, const T& maxCov, const T& minCov) const
  {
    bool invalid = fixCov(val, maxCov);
    val = std::max(minCov, val);
    return invalid || val >= maxCov;
  }

  template<typename T>
  bool fixLinCov(T& val, const T& maxVal, const T& minVal = {}) const
  {
    return fixCov(val, maxVal * maxVal, minVal * minVal);
  }

  template<typename T>
  bool fixAngCov(T& val, const T& minCov = {}) const
  {
    return fixCov(val, static_cast<T>(maxAngCov), minCov);
  }

  template<typename T>
#ifndef ROS2
  bool fixLLACov(boost::array<T, 9>& val) const
#else
  bool fixLLACov(std::array<T, 9>& val) const
#endif
  {
    // lat, lon, alt covariance is in meters even for geodetic coordinates
    bool invalid {false};
    invalid |= fixCov(val[0], static_cast<T>(maxPosErr * maxPosErr));
    invalid |= fixCov(val[1], static_cast<T>(maxPosErr * maxPosErr));
    invalid |= fixCov(val[2], static_cast<T>(maxPosErr * maxAltErr));
    invalid |= fixCov(val[3], static_cast<T>(maxPosErr * maxPosErr));
    invalid |= fixCov(val[4], static_cast<T>(maxPosErr * maxPosErr));
    invalid |= fixCov(val[5], static_cast<T>(maxPosErr * maxAltErr));
    invalid |= fixCov(val[6], static_cast<T>(maxPosErr * maxAltErr));
    invalid |= fixCov(val[7], static_cast<T>(maxPosErr * maxAltErr));
    invalid |= fixCov(val[8], static_cast<T>(maxAltErr * maxAltErr));

    const auto maxCov = validPosErrorThreshold * validPosErrorThreshold;
    invalid |= (val[0] > maxCov) || (val[4] > maxCov);

    // New method with eigenvectors
#if 1
    // Inflate the covariance so that it is at least diag((minPosErr^2, minPosErr^2, minAltErr^2)).
    if (minPosErr != 0.0 || minAltErr != 0.0)
    {
      Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>> mat(val.data(), 3, 3);

      Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigensolver(mat);
      if (eigensolver.info() != Eigen::Success)
      {
        invalid = true;
      }
      else
      {
        const auto& E = eigensolver.eigenvectors();
        const auto& L = Eigen::Matrix3d(eigensolver.eigenvalues().asDiagonal());

        if (!(E * L * E.transpose()).isApprox(mat))
        {
#ifndef ROS2
          CRAS_WARN("Cov matrix decomposition is wrong");
#else
          RCLCPP_WARN(this->get_logger(), "Cov matrix decomposition is wrong");
#endif
        }

        // TODO this is not really correct - the eigenvectors are sorted, so we no longer have a correlation between
        //      horizontal and vertical errors and the eigenvectors
        const auto scale0 = (minPosErr * minPosErr) / L(0, 0);
        const auto scale1 = (minPosErr * minPosErr) / L(1, 1);
        const auto scale2 = (minAltErr * minAltErr) / L(2, 2);
        const auto scale = std::max({1.0, scale0, scale1, scale2});
        if (scale > 1.0)
        {
#ifndef ROS2
          CRAS_INFO("Scaling covariance by %.2f", scale);
#else
          auto& clk = *this->get_clock();
          RCLCPP_INFO_THROTTLE(this->get_logger(), clk, 10000, "Scaling covariance by %.2f", scale);
#endif
          mat = E * (scale * L) * E.transpose();
        }
      }
    }
#endif

    // Older method, most probably wrong
#if 0
    // Inflate the covariance so that it is at least diag((minPosErr^2, minPosErr^2, minAltErr^2)).
    if (minPosErr != 0.0 || minAltErr != 0.0)
    {
      const auto scaleN = (minPosErr * minPosErr) / val[0 * 3 + 0];
      const auto scaleE = (minPosErr * minPosErr) / val[1 * 3 + 1];
      const auto scaleA = (minAltErr * minAltErr) / val[2 * 3 + 2];
      const auto scale = std::max({1.0, scaleN, scaleE, scaleA});
      if (scale > 1.0)
        std::transform(val.begin(), val.end(), val.begin(), [scale](double v) { return scale * v; });
    }
#endif

    return invalid;
  }

  void processAimPlusStatus(const AIMPlusStatus& msg) const
  {
    UPDATE_THREAD_NAME
    this->aimPlusStatusPub.publish(msg);
  }

  void processAtt(const AttEulerConstPtr& msg) const
  {
    UPDATE_THREAD_NAME
    // Synchronize atteuler and attcoveuler topics: both come almost at the same time and at low frequency, so the
    // standard message_filters approximate policy synchronizer would introduce very large delays. This is a much
    // more trivial synchronization, but it should work well for the type of data coming from Septentrio.
    if (this->lastAttCov != nullptr && msg->block_header.tow == this->lastAttCov->block_header.tow)
    {
      this->processHeading(msg, this->lastAttCov);
      this->lastAttCov.reset();
    }
    else
      this->lastAtt = msg;
  }

  void processAttCov(const AttCovEulerConstPtr& msg) const
  {
    UPDATE_THREAD_NAME
    if (this->lastAtt != nullptr && msg->block_header.tow == this->lastAtt->block_header.tow)
    {
      this->processHeading(this->lastAtt, msg);
      this->lastAtt.reset();
    }
    else
      this->lastAttCov = msg;
  }

  void processHeading(const AttEulerConstPtr& att, const AttCovEulerConstPtr& cov) const
  {
    UPDATE_THREAD_NAME
    bool invalid {att->error > 0 || cov->error > 0};
    auto outAtt = *att;
    invalid |= fixNan(outAtt.heading);

    auto outCov = *cov;
    invalid |= fixAngCov(outCov.cov_headhead);
    invalid |= (outCov.cov_headhead > validHeadingErrorThresholdDeg * validHeadingErrorThresholdDeg);

    if (!this->publishInvalidHeading && invalid)
      return;

    fixNan(outAtt.heading_dot);
    fixNan(outAtt.roll);
    fixNan(outAtt.roll_dot);
    fixNan(outAtt.pitch);
    fixNan(outAtt.pitch_dot);

    fixAngCov(outCov.cov_headroll);
    fixAngCov(outCov.cov_headpitch);
    fixAngCov(outCov.cov_pitchpitch);
    fixAngCov(outCov.cov_pitchroll);
    fixAngCov(outCov.cov_rollroll);

    // Inflate the covariance so that heading covariance is at least minHeadingErrDeg^2.
    const auto scale = std::max(1.0f, powf(static_cast<float>(minHeadingErrDeg), 2.0f) / outCov.cov_headhead);
    if (scale > 1.0f)
    {
      outCov.cov_headhead = outCov.cov_headhead * scale;
      outCov.cov_headroll = outCov.cov_headroll * scale;
      outCov.cov_headpitch = outCov.cov_headpitch * scale;
      outCov.cov_pitchpitch = outCov.cov_pitchpitch * scale;
      outCov.cov_pitchroll = outCov.cov_pitchroll * scale;
      outCov.cov_rollroll = outCov.cov_rollroll * scale;
    }

    this->attEulerPub.publish(outAtt);
    this->attCovEulerPub.publish(outCov);

    // Create the alternative heading messages

    const auto conv = M_PI * M_PI / (180.0 * 180.0);

    Azimuth az;
    az.header = att->header;
    az.azimuth = outAtt.heading * M_PI / 180.0;
    az.orientation = Azimuth::ORIENTATION_ENU;
    az.reference = Azimuth::REFERENCE_GEOGRAPHIC;
    az.unit = Azimuth::UNIT_RAD;
    az.variance = outCov.cov_headhead * conv;

    this->azimuthPub.publish(az);

    // TODO pitch is referenced w.r.t. the main-aux1 direction, so if attitude offset is 90 deg,
    //      it should probably be roll instead; this should be somehow expressed, while it's currently not
    Imu imu;
    imu.header = att->header;
    imu.orientation_covariance = {
      outCov.cov_rollroll * conv, outCov.cov_pitchroll * conv, outCov.cov_headroll * conv,
      outCov.cov_pitchroll * conv, outCov.cov_pitchpitch * conv, outCov.cov_headpitch * conv,
      outCov.cov_headroll * conv, outCov.cov_headpitch * conv, outCov.cov_headhead * conv
    };
    // Imu message spec says -1 means not measured, 0 means covariance unknown
    imu.angular_velocity_covariance.fill(0);
    imu.linear_acceleration_covariance.fill(-1);

    imu.angular_velocity.x = outAtt.pitch_dot * M_PI / 180.0;
    imu.angular_velocity.y = outAtt.roll_dot * M_PI / 180.0;
    imu.angular_velocity.z = outAtt.heading_dot * M_PI / 180.0;

    // roll angle is not measured
    tf2::Quaternion q;
    q.setRPY(
      0,
      std::isnan(outAtt.pitch) ? 0 : outAtt.pitch * M_PI / 180.0,
      std::isnan(outAtt.heading) ? 0 : outAtt.heading * M_PI / 180.0);
    imu.orientation = tf2::toMsg(q);

    this->imuPub.publish(imu);
  }

  void processFix(const NavSatFix& msg) const
  {
    UPDATE_THREAD_NAME
    auto outMsg = msg;
    bool invalid {false};
    invalid |= fixNan(outMsg.latitude);
    invalid |= fixNan(outMsg.longitude);
    invalid |= fixNan(outMsg.altitude);
    invalid |= fixLLACov(outMsg.position_covariance);

    // Consumers of fix messages should check status and if it is NO_FIX, they should not use the message.
    if (invalid)
    {
      outMsg.status.status = NavSatStatus::STATUS_NO_FIX;
      if (!this->publishInvalidFix)
        return;
    }

    this->fixPub.publish(outMsg);
  }

  void processFixDetail(const GPSFix& msg) const
  {
    UPDATE_THREAD_NAME
    bool invalid {false};
    auto outMsg = msg;
    invalid |= fixNan(outMsg.latitude);
    invalid |= fixNan(outMsg.longitude);
    invalid |= fixNan(outMsg.altitude);
    fixNan(outMsg.track);
    fixNan(outMsg.speed);
    fixNan(outMsg.climb);
    fixNan(outMsg.pitch);
    fixNan(outMsg.roll);
    invalid |= fixNan(outMsg.dip);
    fixNan(outMsg.time);
    fixNan(outMsg.gdop);
    fixNan(outMsg.pdop);
    fixNan(outMsg.hdop);
    fixNan(outMsg.vdop);
    fixNan(outMsg.tdop);
    invalid |= fixLinCov(outMsg.err, maxPosErr, minPosErr);
    invalid |= fixLinCov(outMsg.err_horz, maxPosErr, minPosErr);
    invalid |= fixLinCov(outMsg.err_vert, maxAltErr, minAltErr);
    fixLinCov(outMsg.err_track, 10.0);
    fixLinCov(outMsg.err_speed, 10.0);
    fixLinCov(outMsg.err_climb, 10.0);
#ifndef ROS2
    fixLinCov(outMsg.err_time, ros::Time::now().toSec());
#else
    fixLinCov(outMsg.err_time, this->now().seconds());
#endif
    fixAngCov(outMsg.err_pitch);
    fixAngCov(outMsg.err_roll);
    invalid |= fixAngCov(outMsg.err_dip);
    invalid |= fixLLACov(outMsg.position_covariance);

    if (invalid && !this->publishInvalidFix)
      return;

    this->fixDetailPub.publish(outMsg);
  }

  void processGpgga(const Gpgga& msg) const
  {
    UPDATE_THREAD_NAME
    auto outMsg = msg;
    fixNan(outMsg.lat);
    fixNan(outMsg.lon);
    fixNan(outMsg.alt);
    fixNan(outMsg.hdop);
    fixNan(outMsg.undulation);

    this->gpggaPub.publish(outMsg);
  }

  void processGprmc(const Gprmc& msg) const
  {
    UPDATE_THREAD_NAME
    auto outMsg = msg;
    fixNan(outMsg.lat);
    fixNan(outMsg.lon);
    fixNan(outMsg.speed);
    fixNan(outMsg.track);
    fixNan(outMsg.mag_var);

    this->gprmcPub.publish(outMsg);
  }

  void processPosCovGeodetic(const PosCovGeodetic& msg) const
  {
    UPDATE_THREAD_NAME
    auto outMsg = msg;
#ifndef ROS2
    const auto MAX_BIAS_ERR = ros::Time::now().toSec();
#else
    const auto MAX_BIAS_ERR = this->now().seconds();
#endif
    fixCov(outMsg.cov_latlat, static_cast<float>(maxPosErr * maxPosErr), static_cast<float>(minPosErr * minPosErr));
    fixCov(outMsg.cov_bb, static_cast<float>(MAX_BIAS_ERR * MAX_BIAS_ERR));
    fixCov(outMsg.cov_latlon, static_cast<float>(maxPosErr * maxPosErr), static_cast<float>(minPosErr * minPosErr));
    fixCov(outMsg.cov_lathgt, static_cast<float>(maxPosErr * maxAltErr), static_cast<float>(minPosErr * minAltErr));
    fixCov(outMsg.cov_latb, static_cast<float>(maxPosErr * MAX_BIAS_ERR));
    fixCov(outMsg.cov_lonhgt, static_cast<float>(maxPosErr * maxAltErr), static_cast<float>(minPosErr * minAltErr));
    fixCov(outMsg.cov_lonb, static_cast<float>(maxPosErr * MAX_BIAS_ERR));
    fixCov(outMsg.cov_hb, static_cast<float>(maxAltErr * MAX_BIAS_ERR));

    this->posCovGeodeticPub.publish(outMsg);
  }

  void processPVTGeodetic(const PVTGeodetic& msg) const
  {
    UPDATE_THREAD_NAME
    auto outMsg = msg;
    fixNan(outMsg.latitude);
    fixNan(outMsg.longitude);
    fixNan(outMsg.height);
    fixNan(outMsg.undulation);
    fixNan(outMsg.vn);
    fixNan(outMsg.ve);
    fixNan(outMsg.vu);
    fixNan(outMsg.cog);
    fixNan(outMsg.rx_clk_bias);
    fixNan(outMsg.rx_clk_drift);

    this->pvtGeodeticPub.publish(outMsg);
  }

  void processGpst(const TimeReference& msg) const
  {
    UPDATE_THREAD_NAME
    this->gpstPub.publish(msg);
  }

  void processRFStatus(const RFStatus& msg) const
  {
    UPDATE_THREAD_NAME
    this->rfStatusPub.publish(msg);
  }

  void processVelCovGeodetic(const VelCovGeodetic& msg) const
  {
    UPDATE_THREAD_NAME
    auto outMsg = msg;
    fixCov(outMsg.cov_vnvn, static_cast<float>(maxHorzVelErr * maxHorzVelErr));
    fixCov(outMsg.cov_veve, static_cast<float>(maxHorzVelErr * maxHorzVelErr));
    fixCov(outMsg.cov_vuvu, static_cast<float>(maxVertVelErr * maxVertVelErr));
    fixCov(outMsg.cov_dtdt, static_cast<float>(maxClkBiasErr * maxClkBiasErr));
    fixCov(outMsg.cov_vnve, static_cast<float>(maxHorzVelErr * maxHorzVelErr));
    fixCov(outMsg.cov_vnvu, static_cast<float>(maxHorzVelErr * maxVertVelErr));
    fixCov(outMsg.cov_vndt, static_cast<float>(maxHorzVelErr * maxClkBiasErr));
    fixCov(outMsg.cov_vevu, static_cast<float>(maxHorzVelErr * maxVertVelErr));
    fixCov(outMsg.cov_vedt, static_cast<float>(maxHorzVelErr * maxClkBiasErr));
    fixCov(outMsg.cov_vudt, static_cast<float>(maxVertVelErr * maxClkBiasErr));

    this->velCovGeodeticPub.publish(outMsg);
  }

#ifndef ROS2
  ros::NodeHandle nh;
  ros::NodeHandle pnh;
#endif

  Subscriber<AIMPlusStatus> aimPlusStatusSub;
  Publisher<AIMPlusStatus> aimPlusStatusPub;

  Subscriber<AttEuler> attEulerSub;
  Publisher<AttEuler> attEulerPub;

  Subscriber<AttCovEuler> attCovEulerSub;
  Publisher<AttCovEuler> attCovEulerPub;

  Subscriber<NavSatFix> fixSub;
  Publisher<NavSatFix> fixPub;

  Subscriber<GPSFix> fixDetailSub;
  Publisher<GPSFix> fixDetailPub;

  Subscriber<Gpgga> gpggaSub;
  Publisher<Gpgga> gpggaPub;

  Subscriber<Gprmc> gprmcSub;
  Publisher<Gprmc> gprmcPub;

  Subscriber<TimeReference> gpstSub;
  Publisher<TimeReference> gpstPub;

  Subscriber<PosCovGeodetic> posCovGeodeticSub;
  Publisher<PosCovGeodetic> posCovGeodeticPub;

  Subscriber<PVTGeodetic> pvtGeodeticSub;
  Publisher<PVTGeodetic> pvtGeodeticPub;

  Subscriber<RFStatus> rfStatusSub;
  Publisher<RFStatus> rfStatusPub;

  Subscriber<VelCovGeodetic> velCovGeodeticSub;
  Publisher<VelCovGeodetic> velCovGeodeticPub;

  Publisher<Azimuth> azimuthPub;
  Publisher<Imu> imuPub;

  mutable AttEulerConstPtr lastAtt;
  mutable AttCovEulerConstPtr lastAttCov;

  // These maximum errors are used instead of do-not-use values or too large errors.

  double maxPosErr {40000000};  //!< Maximum error in cartesian position [m]. Default is 40 000 km, Earth circumference.
  double maxAltErr {100000};  //!< Maximum error in altitude [m]. Default is 100 km.
  double maxAngErr {2 * M_PI};  //!< Maximum error in orientation angles [rad].
  double maxAngCov {maxAngErr * maxAngErr};

  double maxHorzVelErr {10.0};  //!< Maximum error in horizontal velocity [m/s].
  double maxVertVelErr {10.0};  //!< Maximum error in vertical velocity [m/s].
  double maxClkBiasErr {10.0};  //!< Maximum error in clock bias converted to velocity error [m/s].

  // These thresholds decide acceptable errors for letting the data through marked as valid for navigation.

  double validPosErrorThreshold {0.15};  //!< Position measurements with larger error (in meters) will be discarded.
  double validHeadingErrorThresholdDeg {5.0};  //!< Heading measurements with larger error will be discarded.

  double minPosErr {0.0};  //!< Minimum error assigned to XY part of fix and fix_detail messages.
  double minAltErr {0.0};  //!< Minimum error assigned to Z part of fix and fix_detail messages.
  double minHeadingErrDeg {0.0};  //!< Minimum error assigned to heading messages (in degrees).

  bool publishInvalidFix {true};  //!< Whether to publish invalid fix messages (with NO_FIX status or NaNs).
  bool publishInvalidHeading {false};  //!< Whether to publish invalid heading messages (with NaNs or high covariance).
};

}

#ifndef ROS2
PLUGINLIB_EXPORT_CLASS(cras::SeptentrioProcess, nodelet::Nodelet)
#else
RCLCPP_COMPONENTS_REGISTER_NODE(cras::SeptentrioProcess)
#endif
