// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief Observation shows that the azimuth (heading) reported by mosaic-H receivers can sometimes intermittently
 * switch by 180 degrees without changing covariance appropriately. This is of course death to all localization
 * algorithms. This filter observes the sequence of headings and compares it to the heading of an IMU. If the GNSS
 * heading differs too much from the IMU heading, the measurement is discarded. The IMU heading does not need to be
 * georeferenced - the initial offset between IMU and GNSS heading is taken into account.
 */

#include <algorithm>
#include <cmath>
#include <memory>
#include <numeric>
#include <string>
#include <utility>
#include <vector>

#include <boost/circular_buffer.hpp>

#ifndef ROS2

#include <cras_cpp_common/nodelet_utils.hpp>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.hpp>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Transform.h>

#else

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2/LinearMath/Matrix3x3.hpp>
#include <tf2/LinearMath/Transform.hpp>

#endif

#ifndef ROS2

using Imu = sensor_msgs::Imu;
using ImuConstPtr = sensor_msgs::ImuConstPtr;
template<typename T> using Publisher = ros::Publisher;
template<typename T> using Subscriber = ros::Subscriber;

#else

using Imu = sensor_msgs::msg::Imu;
using ImuConstPtr = sensor_msgs::msg::Imu::ConstSharedPtr;

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
            static_cast<rclcpp::Publisher<T>::SharedPtr&>(*this) =
                static_cast<rclcpp::Publisher<T>::SharedPtr&&>(shared);
        return *this;
    }

    Publisher(const rclcpp::Publisher<T>::SharedPtr& other) : rclcpp::Publisher<T>::SharedPtr(other) {}  // NOLINT
    template<typename M> void publish(const M& val) const { this->get()->publish(std::move(val)); }
};

template<typename T> using Subscriber = typename rclcpp::Subscription<T>::SharedPtr;

#endif

namespace cras
{

struct GPSAzimuthFilter :
#ifndef ROS2
    public cras::Nodelet
#else
    public rclcpp::Node
#endif
{
#ifndef ROS2
    GPSAzimuthFilter() = default;
    void onInit() override;
#else
    explicit GPSAzimuthFilter(const rclcpp::NodeOptions& options);
#endif
    ~GPSAzimuthFilter() override;

#ifndef ROS2
    ros::NodeHandle nh;
    ros::NodeHandle pnh;
#endif

    Subscriber<Imu> gps_azimuth_subscriber_;
    Subscriber<Imu> imu_subscriber_;
    Publisher<Imu> gps_azimuth_pub_;

    boost::circular_buffer<double> offsets{10};
    ImuConstPtr last_imu_msg;

    double max_msg_timestamp_delta{0.2};  // seconds
    double max_offset_delta{M_PI/2.};  // rad
    int max_wrong_msgs_in_row{10};

    int count_filtered_in_row{0};

    double get_yaw(const ImuConstPtr& msg) const;
    double get_mean_offset() const;
    double get_median_offset() const;

    void gps_callback(const ImuConstPtr& msg);
    void imu_callback(const ImuConstPtr& msg);

#ifndef ROS2
    template <typename T>
    ros::Publisher advertise(const std::string& topic, const size_t queueSize)
    {
        return this->nh.advertise<T>(topic, queueSize);
    }

    template <typename T, typename Fn>
    ros::Subscriber subscribe(
        const std::string& topic, const size_t queueSize, const Fn& fn, GPSAzimuthFilter* self)
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
        const std::string& topic, const size_t queueSize, const Fn& fn, GPSAzimuthFilter* self)
    {
        return this->create_subscription<T>(
            topic, rclcpp::SensorDataQoS().keep_last(queueSize), std::bind(fn, this, std::placeholders::_1));
    }
#endif
};

#ifndef ROS2
void GPSAzimuthFilter::onInit()
{
    this->nh = this->getNodeHandle();
    this->pnh = this->getPrivateNodeHandle();

    const auto params = this->privateParams();
#else
GPSAzimuthFilter::GPSAzimuthFilter(const rclcpp::NodeOptions& options) : Node("GPSAzimuthFilter", options)
{
    auto* params = this;
#endif

    this->max_msg_timestamp_delta = params->getParam("max_msg_timestamp_delta", this->max_msg_timestamp_delta, "s");
    this->max_offset_delta = params->getParam("max_offset_delta", this->max_offset_delta, "rad");
    this->max_wrong_msgs_in_row = params->getParam("max_wrong_msgs_in_row", this->max_wrong_msgs_in_row, "msgs");

    gps_azimuth_subscriber_ = this->subscribe<Imu>("gps/azimuth_imu", 10, &GPSAzimuthFilter::gps_callback, this);
    imu_subscriber_ = this->subscribe<Imu>("imu/data", 10, &GPSAzimuthFilter::imu_callback, this);
    gps_azimuth_pub_ = this->advertise<Imu>("gps/azimuth_imu/filtered", 10);

#ifndef ROS2
    ROS_INFO("Septentrio azimuth filter initialized");
#else
    RCLCPP_INFO(this->get_logger(), "Septentrio azimuth filter initialized");
#endif
}

GPSAzimuthFilter::~GPSAzimuthFilter() = default;

double GPSAzimuthFilter::get_mean_offset() const
{
    if (offsets.empty()) {
        return 0.;
    }
    const double sum = std::accumulate(offsets.begin(), offsets.end(), 0.0);
    return sum / offsets.size();
}

double GPSAzimuthFilter::get_median_offset() const
// TODO: fix for calculating around pi -pi OR 0 2pi
{
    if (offsets.empty()) {
        return 0.;
    }
    std::vector<double> v(offsets.begin(), offsets.end());

    std::sort(v.begin(), v.end());

    const size_t n = v.size();

    // special case if the offsets are around pi and -pi (e.g. something like {2.9, 3.0, 3.1, -3.1, -3.0})
    // in the case above the "median" should be 3.1...
    if (v[0] < -0.8 * M_PI && v[n - 1] > 0.8 * M_PI) {
        const auto first_positive = std::lower_bound(v.begin(), v.end(), 0.0);

        // Copy and reverse the negative part
        std::vector<double> negatives(v.begin(), first_positive);
        std::reverse(negatives.begin(), negatives.end());

        // Erase negatives from original vector
        v.erase(v.begin(), first_positive);

        // Append reversed negatives
        v.insert(v.end(), negatives.begin(), negatives.end());
    }

    if (n % 2 == 0)
        return v[n / 2];
    else
        return v[static_cast<int>(std::floor(n / 2))];
}

double GPSAzimuthFilter::get_yaw(const ImuConstPtr& msg) const
{
    const tf2::Quaternion quat(
        msg->orientation.x,
        msg->orientation.y,
        msg->orientation.z,
        msg->orientation.w);
    const tf2::Matrix3x3 m(quat);

    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    return yaw;
}

void GPSAzimuthFilter::gps_callback(const ImuConstPtr& msg)
{
    if (!last_imu_msg) {
#ifndef ROS2
        ROS_INFO_THROTTLE(1.0, "Waiting for IMU msg to arrive first.");
#else
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000., "Waiting for IMU msg to arrive first.");
#endif
        return;
    }

    // Check that the IMU message is not too old.
#ifndef ROS2
    double timestamp_delta = (msg->header.stamp - last_imu_msg->header.stamp).toSec();
#else
    double timestamp_delta = (rclcpp::Time(msg->header.stamp) - rclcpp::Time(last_imu_msg->header.stamp)).seconds();
#endif
    if (timestamp_delta > max_msg_timestamp_delta) {
#ifndef ROS2
        ROS_WARN("IMU msg too old (%f s > max: %f s)", timestamp_delta, max_msg_timestamp_delta);
#else
        RCLCPP_WARN(this->get_logger(), "IMU msg too old (%f s > max: %f s)", timestamp_delta, max_msg_timestamp_delta);
#endif
        return;
    }

    // Get offsets
    double offset = this->get_yaw(msg) - this->get_yaw(last_imu_msg);
    if (offset > M_PI) {
        offset -= 2 * M_PI;
    } else if (offset < -M_PI) {
        offset += 2 * M_PI;
    }

    double median_offset = get_median_offset();
    // TODO utilize angles package?
    const double offset_delta = std::min(
        std::fabs(offset - median_offset),
        std::fabs(std::fabs(offset - median_offset) - 2 * M_PI));

    // Always push back the first offset (can't decide if wrong or correct).
    if (offsets.empty()) {
        offsets.push_back(offset);
#ifndef ROS2
        ROS_INFO("First message (%ld/%ld) (not republishing yet)", offsets.size(), offsets.capacity());
#else
        RCLCPP_INFO(this->get_logger(),
            "First message (%ld/%ld) (not republishing yet)", offsets.size(), offsets.capacity());
#endif
        return;
    }
    // Filter out if difference between msg offset and median offset is too large
    else if (offset_delta > max_offset_delta)
    {
        count_filtered_in_row++;
#ifndef ROS2
        ROS_ERROR("!!!!!\n\nGPS azimuth msg filtered out (msg offset: %f rad, median offset: %f rad) "
                  "(%d in a row out of max %d)\n\n!!!!!",
                  offset, median_offset, count_filtered_in_row, max_wrong_msgs_in_row);
#else
        RCLCPP_ERROR(this->get_logger(),
            "!!!!!\n\nGPS azimuth msg filtered out (msg offset: %f rad, median offset: %f rad) "
            "(%d in a row out of max %d)\n\n!!!!!",
            offset, median_offset, count_filtered_in_row, max_wrong_msgs_in_row);
#endif
        if (count_filtered_in_row > max_wrong_msgs_in_row) {
#ifndef ROS2
            ROS_WARN("Too many (%d) msgs filtered out in a row, assuming the buffer is full of wrong messages and "
                     "the correct msgs are being filtered => resetting the buffer", count_filtered_in_row);
#else
            RCLCPP_WARN(this->get_logger(),
                "Too many (%d) msgs filtered out in a row, assuming the buffer is full of wrong messages and "
                "the correct msgs are being filtered => resetting the buffer", count_filtered_in_row);
#endif
            offsets.clear();
            count_filtered_in_row = 0;
        }
        return;
    }
    // Else if buffer is not full, fill buffer first
    else if (offsets.size() < offsets.capacity())
    {
        count_filtered_in_row = 0;
        offsets.push_back(offset);
#ifndef ROS2
        ROS_INFO("Filling up buffer (%ld/%ld) (not republishing yet)", offsets.size(), offsets.capacity());
#else
        RCLCPP_INFO(this->get_logger(),
            "Filling up buffer (%ld/%ld) (not republishing yet)", offsets.size(), offsets.capacity());
#endif
        return;
    }
    else
    {
        count_filtered_in_row = 0;
        offsets.push_back(offset);
    }

    // Only if buffer full and the message was not filtered out republish it.
    gps_azimuth_pub_.publish(*msg);
}

void GPSAzimuthFilter::imu_callback(const ImuConstPtr& msg)
{
    last_imu_msg = msg;
}

}

#ifndef ROS2
PLUGINLIB_EXPORT_CLASS(cras::GPSAzimuthFilter, nodelet::Nodelet)
#else
RCLCPP_COMPONENTS_REGISTER_NODE(cras::GPSAzimuthFilter)
#endif
