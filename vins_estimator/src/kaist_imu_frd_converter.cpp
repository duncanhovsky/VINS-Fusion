/*******************************************************
 * KAIST IMU adapter for original VINS-Fusion.
 *
 * Converts FLU-style raw IMU vector channels to FRD by applying
 * D = diag(1, -1, -1). Orientation is invalidated by default because
 * VINS-Fusion only consumes angular velocity and linear acceleration.
 *******************************************************/

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>

#include <string>

namespace {

double axisSign(int index)
{
    return index == 0 ? 1.0 : -1.0;
}

boost::array<double, 9> convertCovariance(const boost::array<double, 9> &input)
{
    boost::array<double, 9> output;
    for (int row = 0; row < 3; ++row)
    {
        for (int col = 0; col < 3; ++col)
        {
            output[row * 3 + col] = axisSign(row) * axisSign(col) * input[row * 3 + col];
        }
    }
    return output;
}

class KaistImuFrdConverter
{
  public:
    explicit KaistImuFrdConverter(const ros::NodeHandle &private_node)
    {
        private_node.param<std::string>("input_topic", input_topic_, "/imu/data_raw");
        private_node.param<std::string>("output_topic", output_topic_, "/imu/data_frd");
        private_node.param<std::string>("output_frame_id", output_frame_id_, "imu_frd");
        private_node.param("invalidate_orientation", invalidate_orientation_, true);

        publisher_ = node_.advertise<sensor_msgs::Imu>(output_topic_, 200);
        subscriber_ = node_.subscribe(input_topic_, 200, &KaistImuFrdConverter::callback, this);

        ROS_INFO_STREAM("KAIST IMU FLU->FRD converter: " << input_topic_ << " -> " << output_topic_);
    }

  private:
    void callback(const sensor_msgs::ImuConstPtr &message)
    {
        sensor_msgs::Imu converted = *message;
        converted.header.frame_id = output_frame_id_;

        converted.angular_velocity.x = message->angular_velocity.x;
        converted.angular_velocity.y = -message->angular_velocity.y;
        converted.angular_velocity.z = -message->angular_velocity.z;

        converted.linear_acceleration.x = message->linear_acceleration.x;
        converted.linear_acceleration.y = -message->linear_acceleration.y;
        converted.linear_acceleration.z = -message->linear_acceleration.z;

        converted.angular_velocity_covariance = convertCovariance(message->angular_velocity_covariance);
        converted.linear_acceleration_covariance = convertCovariance(message->linear_acceleration_covariance);

        if (invalidate_orientation_)
        {
            converted.orientation.x = 0.0;
            converted.orientation.y = 0.0;
            converted.orientation.z = 0.0;
            converted.orientation.w = 1.0;
            converted.orientation_covariance.assign(0.0);
            converted.orientation_covariance[0] = -1.0;
        }

        publisher_.publish(converted);
    }

    ros::NodeHandle node_;
    ros::Subscriber subscriber_;
    ros::Publisher publisher_;
    std::string input_topic_;
    std::string output_topic_;
    std::string output_frame_id_;
    bool invalidate_orientation_{true};
};

} // namespace

int main(int argc, char **argv)
{
    ros::init(argc, argv, "kaist_imu_frd_converter");
    ros::NodeHandle private_node("~");
    KaistImuFrdConverter converter(private_node);
    ros::spin();
    return 0;
}

