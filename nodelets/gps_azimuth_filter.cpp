#include "rclcpp/rclcpp.hpp"
#include <boost/circular_buffer.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Transform.h>

namespace cras {
class GPSAzimuthFilter : public rclcpp::Node
{
    public:
        GPSAzimuthFilter();
        explicit GPSAzimuthFilter(const rclcpp::NodeOptions& options);
        ~GPSAzimuthFilter() override;

        void init();
    private:
        std::shared_ptr<rclcpp::Subscription<sensor_msgs::msg::Imu>> gps_azimuth_subscriber_;
        std::shared_ptr<rclcpp::Subscription<sensor_msgs::msg::Imu>> imu_subscriber_;
        std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::Imu>> gps_azimuth_pub_;

        boost::circular_buffer<double> offsets{10};
        sensor_msgs::msg::Imu::SharedPtr last_imu_msg;

        double max_msg_timestamp_delta{0.2}; // seconds
        double max_offset_delta{M_PI/2.}; // rad
        int max_wrong_msgs_in_row{10};

        int count_filtered_in_row{0};

        double get_yaw(const sensor_msgs::msg::Imu::SharedPtr msg);
        double get_mean_offset();
        double get_median_offset();
        
        void gps_callback(const sensor_msgs::msg::Imu::SharedPtr msg);
        void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg);
};

GPSAzimuthFilter::GPSAzimuthFilter() : Node("GPSAzimuthFilter")
{
    this->init();
}

GPSAzimuthFilter::GPSAzimuthFilter(const rclcpp::NodeOptions& options) : Node("GPSAzimuthFilter", options)
{
    this->init();
}

void GPSAzimuthFilter::init()
{
    this->declare_parameter<double>("max_msg_timestamp_delta", this->max_msg_timestamp_delta);
    this->declare_parameter<double>("max_offset_delta", this->max_offset_delta);
    this->declare_parameter<int>("max_wrong_msgs_in_row", this->max_wrong_msgs_in_row);
    this->get_parameter("max_msg_timestamp_delta", this->max_msg_timestamp_delta);
    this->get_parameter("max_offset_delta", this->max_offset_delta);
    this->get_parameter("max_wrong_msgs_in_row", this->max_wrong_msgs_in_row);

    gps_azimuth_subscriber_ = this->create_subscription<sensor_msgs::msg::Imu>(
        "gps/azimuth_imu",
        rclcpp::SensorDataQoS(),
        std::bind(&GPSAzimuthFilter::gps_callback, this, std::placeholders::_1));

    imu_subscriber_ = this->create_subscription<sensor_msgs::msg::Imu>(
        "imu/data", 
        rclcpp::SensorDataQoS(), 
        std::bind(&GPSAzimuthFilter::imu_callback, this, std::placeholders::_1));

    gps_azimuth_pub_ = this->create_publisher<sensor_msgs::msg::Imu>(
        "gps/azimuth_imu/filtered",
        rclcpp::SystemDefaultsQoS());
}

GPSAzimuthFilter::~GPSAzimuthFilter() = default;

double GPSAzimuthFilter::get_mean_offset()
{
    if(offsets.size() == 0) {
        return 0.;
    }
    double sum = std::accumulate(offsets.begin(), offsets.end(), 0);
    return sum/offsets.size();
}

double GPSAzimuthFilter::get_median_offset()
// TODO: fix for calculating around pi -pi OR 0 2pi
{
    if(offsets.size() == 0) {
        return 0.;
    }
    std::vector<double> v(offsets.begin(), offsets.end());

    // for(auto ofs : offsets) {
    //     RCLCPP_INFO(this->get_logger(), std::to_string(ofs).c_str());
    // }
    std::sort(v.begin(), v.end());

    size_t n = v.size();

    // special case if the offsets are around pi and -pi (e.g. something like {2.9, 3.0, 3.1, -3.1, -3.0})
    // in the case above the "median" should be 3.1...
    if (v[0] < -0.8 * M_PI && v[n] > 0.8 * M_PI) {
        auto first_positive = std::lower_bound(v.begin(), v.end(), 0.0);

        // Copy and reverse the negative part
        std::vector<double> negatives(v.begin(), first_positive);
        std::reverse(negatives.begin(), negatives.end());

        // Erase negatives from original vector
        v.erase(v.begin(), first_positive);

        // Append reversed negatives
        v.insert(v.end(), negatives.begin(), negatives.end());
    }
    
    if (n % 2 == 1)
        return v[n / 2];
    else
        return v[(int)std::floor(n / 2)];
}

double GPSAzimuthFilter::get_yaw(const sensor_msgs::msg::Imu::SharedPtr msg)
{
    tf2::Quaternion quat(
        msg->orientation.x,
        msg->orientation.y,
        msg->orientation.z,
        msg->orientation.w);
    tf2::Matrix3x3 m(quat);

    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    return yaw;
}

void GPSAzimuthFilter::gps_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
{   
    if (!last_imu_msg) {
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000., "Waiting for IMU msg to arrive first.");
        return;
    }

    // Check that the IMU message is not too old.
    double timestamp_delta = (rclcpp::Time(msg->header.stamp) - rclcpp::Time(last_imu_msg->header.stamp)).seconds();
    if (timestamp_delta > max_msg_timestamp_delta) {
        RCLCPP_WARN(this->get_logger(), "IMU msg too old (%f s > max: %f s)", timestamp_delta, max_msg_timestamp_delta);
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
    double offset_delta = std::min(std::fabs(offset-median_offset), std::fabs(std::fabs(offset-median_offset)-2*M_PI));
    // RCLCPP_INFO(this->get_logger(), "gps_yaw: %f\nimu_yaw: %f\noffset: %f\nmedian offset: %f\noffset_delta: %f", this->get_yaw(msg), this->get_yaw(last_imu_msg), offset, median_offset, offset_delta);

    // Always push back the first offset (can't decide if wrong or correct).
    if (offsets.empty()) {
        offsets.push_back(offset);
        RCLCPP_INFO(this->get_logger(), "First message (%ld/%ld) (not republishing yet)", offsets.size(), offsets.capacity());
        return;
    }
    // Filter out if difference between msg offset and median offset is too large
    else if (offset_delta > max_offset_delta) {
        count_filtered_in_row++;
        RCLCPP_ERROR(this->get_logger(), "!!!!!\n\nGPS azimuth msg filtered out (msg offset: %f rad, median offset: %f rad) (%d in a row out of max %d)\n\n!!!!!", offset, median_offset, count_filtered_in_row, max_wrong_msgs_in_row);
        if (count_filtered_in_row > max_wrong_msgs_in_row) {
            RCLCPP_WARN(this->get_logger(), "Too many (%d) msgs filtered out in a row, assuming the buffer is full of wrong messages and the correct msgs are being filtered => resetting the buffer", count_filtered_in_row);
            offsets.clear();
            count_filtered_in_row = 0;
        }
        return;
    }
    // Else if buffer is not full, fill buffer first
    else if (offsets.size() < offsets.capacity()) {
        count_filtered_in_row = 0;
        offsets.push_back(offset);
        RCLCPP_INFO(this->get_logger(), "Filling up buffer (%ld/%ld) (not republishing yet)", offsets.size(), offsets.capacity());
        return;
    }
    else {
        count_filtered_in_row = 0;
        offsets.push_back(offset);
    }

    // Only if buffer full and the message was not filtered out republish it.
    gps_azimuth_pub_->publish(*msg);
}

void GPSAzimuthFilter::imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
    last_imu_msg = msg;
}
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(cras::GPSAzimuthFilter)