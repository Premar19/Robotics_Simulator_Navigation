#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <vector>
#include <limits>
#include <chrono>
#include <functional>
#include <memory>
#include <numeric>
#include <cmath>
#include <string>

using namespace std::chrono_literals;
using std::placeholders::_1;

class ws3 : public rclcpp::Node
{
public:
    ws3()
    : Node("lidar_frame_detector"),
      front_avg_(0.0), front_left_avg_(0.0), front_right_avg_(0.0),
      left_avg_(0.0), right_avg_(0.0)
    {
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/husky/cmd_vel", 10);

        subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/husky/sensors/lidar2d_0/scan", 10,
            std::bind(&ws3::scan_callback, this, _1));

        // 10 Hz control loop
        timer_ = this->create_wall_timer(
            100ms, std::bind(&ws3::timer_callback, this));

        // Parameters (tune as needed)
        safe_front_range_ = 2.0;  // meters
        safe_side_range_ = 1.0;   // meters
        max_speed_ = 1.0;         // m/s


        RCLCPP_INFO(this->get_logger(), "LidarFrameDetector initialized for autonomous driving.");
    }

private:
    // ============= LASER PROCESSING =============
    void scan_callback(const sensor_msgs::msg::LaserScan & msg)
    {
        RCLCPP_INFO(this->get_logger(), "Received scan with %zu ranges", msg.ranges.size());
        const auto &ranges = msg.ranges;
        size_t n = ranges.size();

        // Helper lambda to average over a range of indices
        auto average_range = [&](int start, int end) -> float {
            float sum = 0.0;
            int count = 0;
            for (int i = start; i <= end; ++i) {
                float r = ranges[(i + n) % n];
                if (std::isfinite(r)) {
                    sum += r;
                    count++;
                }
            }
            return (count > 0) ? (sum / count) : msg.range_max;
        };

        // Define angular sectors (assuming 0° = front, increasing CCW)
        int center = static_cast<int>(n / 2);
        int width = 20; // guess to average

        int back_idx        = 0;
        int front_idx       = n / 2;
        int left_idx        = n / 3;
        int right_idx       = (2 * n) / 3;

        // Calculate region averages
        front_avg_       = average_range(front_idx - width, front_idx + width);
        front_left_avg_  = average_range(front_idx - 45 - width, front_idx - 45 + width);
        front_right_avg_ = average_range(front_idx + 45 - width, front_idx + 45 + width);
        left_avg_        = average_range(left_idx - width, left_idx + width);
        right_avg_       = average_range(right_idx - width, right_idx + width);
        RCLCPP_INFO(this->get_logger(), "front average: %.2f", front_avg_);
        RCLCPP_INFO(this->get_logger(), "front-left average: %.2f", front_left_avg_);
        RCLCPP_INFO(this->get_logger(), "front-right average: %.2f", front_right_avg_);
        RCLCPP_INFO(this->get_logger(), "left average: %.2f", left_avg_);
        RCLCPP_INFO(this->get_logger(), "right average: %.2f", right_avg_);
    }

    // ============= CONTROL LOOP =============
    void timer_callback()
    {
        geometry_msgs::msg::Twist cmd;

        // Compute linear velocity (Eq. 1)
        float linear_speed = std::min(((front_avg_ / safe_front_range_) * max_speed_),max_speed_);
        //linear_speed = std::min(linear_speed, max_speed_);
        float angular_speed = 0;

        // Stop if too close to a wall
        //if (front_avg_ < 0.2) {
        //    linear_speed = -1;
            //angular_speed = 0.0;
        //}
        if (front_avg_ < 0.2 && left_avg_ < 0.2){
            linear_speed = -2;
            angular_speed = -2;
        }
        else if (front_avg_ < 0.2 && right_avg_ < 0.2){
            linear_speed = -2;
            angular_speed = 2;
        }

        /*if(front_left_avg_ < 3.0)
        {
            angular_speed = 3.0;
        }
        if(front_right_avg_ < 3.0)
        {
            linear_speed = 0.2;
            angular_speed = -3.0;
        }*/

        // Compute angular velocity (Eq. 2)
        float side_diff = left_avg_ - right_avg_;

        RCLCPP_INFO(this->get_logger(), "The side diff isssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssss %.2f ", side_diff);
        angular_speed = (-side_diff / safe_side_range_) * max_speed_;

        angular_speed = std::clamp(angular_speed, -max_speed_, max_speed_);
        to_return_ = angular_speed;
        //if(side_diff <= 1)
        //{
        //    angular_speed=0;
        //}
        RCLCPP_INFO(this->get_logger(), "The angular speed isssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssss %.2f ", angular_speed);

        // Corner handling
        /*if (front_left_avg_ < 0.7)
            angular_speed -= 1;
        if (front_right_avg_ < 0.7)
            angular_speed += 1;*/

        //linear_speed = 2;
        cmd.linear.x = linear_speed;
        //cmd.angular.z = angular_speed;
        cmd.angular.z = angular_speed;

        publisher_->publish(cmd);

        RCLCPP_INFO(this->get_logger(),
            "cmd: linear=%.2f m/s, angular=%.2f rad/s | front=%.2f, FL=%.2f, FR=%.2f, L=%.2f, R=%.2f",
            cmd.linear.x, cmd.angular.z,
            front_avg_, front_left_avg_, front_right_avg_, left_avg_, right_avg_);
    }

    // ROS objects
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;

    // Laser region averages
    float front_avg_, front_left_avg_, front_right_avg_, left_avg_, right_avg_;

    // Parameters
    float safe_front_range_=1;
    float safe_side_range_ = 0.5;
    float max_speed_ = 5.0;
    float to_return_= 0.0;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ws3>());
    rclcpp::shutdown();
    return 0;
}
