#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <vector>
#include <limits>  // for std::numeric_limits
#include <chrono>
#include <functional>
#include <memory>
#include "geometry_msgs/msg/twist.hpp"
#include <string>
#include <numeric>
#include <cmath>
//ROS 2 Specific includes
//Important: these includes should be reflected in the package.xml and CMakeLists.txt
//#include "rclcpp/rclcpp.hpp"

// TODO: Add or replace with message type needed
#include "std_msgs/msg/u_int32.hpp"
//lidar indexes to be removed: 48 49 50 51 52 73 74 75 76 77 78 79 80 81 82 83 636 637 638 639 640 641 642 643 644 645 646 667 668 669 670 671
using namespace std::chrono_literals;
using std::placeholders::_1;

class LidarFrameDetector : public rclcpp::Node
{
public:
    LidarFrameDetector() : Node("lidar_frame_detector")
    {
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/husky/cmd_vel", 10);
        subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10,
            std::bind(&LidarFrameDetector::scan_callback, this, std::placeholders::_1));

        timer_ = this->create_wall_timer(
          500ms, std::bind(&LidarFrameDetector::timer_callback, this));



    }


private:
    // Private function that will be triggered automatically when messages are received
    // from the topic /hello/world
    // The parameter specifies the expected message type, which must match
    // the type published to the topic
    void scan_callback(const sensor_msgs::msg::LaserScan & msg)
    {
        //Logger used to print details of the message received (printed to console).
        std::vector<int> hit_indexes;
        std::vector<int> indexes;

        for (size_t i = 0; i < msg.ranges.size(); ++i)
        {
            float range = msg.ranges[i];
            //size_t n = range.size();
            // Check if the beam returned a finite distance AND is closer than 1.0m
            if (std::isfinite(range))// && range < 1.0f)
            {
                hit_indexes.push_back(static_cast<int>(i));
            }
            /*if (std::isfinite(range))// && range < 1.0f)
            {
              indexes.push_back(static_cast<int>(i));
            }*/
        }


        // Output the result
        if (!hit_indexes.empty()) {
            RCLCPP_INFO(this->get_logger(), "Beams hitting the frame at indexes:");
            for (int idx : hit_indexes) {
                float range = msg.ranges[idx];
                std::cout << idx << " " << range;
            }
            std::cout << std::endl;
        } else {
            RCLCPP_INFO(this->get_logger(), "No nearby frame detected (all beams far or invalid).");
        }
    }

    void timer_callback()
    {
        auto message = geometry_msgs::msg::Twist();
        message.linear.x = 2.0;
        RCLCPP_INFO(this->get_logger(), "Publishing: msg.linear.x=%f", message.linear.x);
        publisher_->publish(message);
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    //rclcpp::spin(std::make_shared<LidarFrameDetector>());
    rclcpp::spin(std::make_shared<LidarFrameDetector>());
    rclcpp::shutdown();
    return 0;
}

