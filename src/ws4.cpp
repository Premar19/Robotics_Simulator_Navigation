//
// Created by prs49 on 31/10/25.
//
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "ros366_cv_msgs/msg/colour_points.hpp"
#include <vector>
#include <limits>
#include <chrono>
#include <functional>
#include <memory>
#include <numeric>
#include <cmath>
#include <string>
#ifndef WS4_H
#define WS4_H
using namespace std::chrono_literals;
using std::placeholders::_1;


class ws4 : public rclcpp::Node{
public:
    ws4()
    : Node("lidar_frame_detector")
    {
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/husky/cmd_vel", 10);

        subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/husky/sensors/lidar2d_0/scan", 10,
            std::bind(&ws4::scan_callback, this, _1));

		sub_ = this->create_subscription<ros366_cv_msgs::msg::ColourPoints>(
            "/colour_detection", 10,
            std::bind(&ws4::color_callback, this, std::placeholders::_1)
        );
        RCLCPP_INFO(this->get_logger(), "Colour Chase Node Started");
        // 10 Hz control loop
        timer_ = this->create_wall_timer(
            100ms, std::bind(&ws4::timer_callback, this));


        RCLCPP_INFO(this->get_logger(), "LidarFrameDetector initialized for autonomous driving.");

		have_detection_ = false;
		crd=false;cgd=false;cbd=false;
    }
    private:
    // ============= LASER PROCESSING =============
    void scan_callback(const sensor_msgs::msg::LaserScan & msg)
    {
        RCLCPP_INFO(this->get_logger(), "Received scan with %zu ranges", msg.ranges.size());
        const auto &ranges = msg.ranges;
        size_t n = ranges.size();
    }

	void color_callback(const ros366_cv_msgs::msg::ColourPoints & msg)
	{
		last_detection_ = msg;
        have_detection_ = (msg.points.size()>0);
        x=msg.points[0].x;

	}

    // ============= CONTROL LOOP =============
    void timer_callback()
    {

        geometry_msgs::msg::Twist cmd;
        cmd.linear.x=0.0f;
        publisher_->publish(cmd);
        if(crd==false)
        {
            RCLCPP_INFO(this->get_logger(), "Entered Red");
            send_request({0,0,0},max_values[count]);
        }
        /*else if(cgd==false)
        {
            RCLCPP_INFO(this->get_logger(), "Eneterd Green");
            send_request({0,0,0},{0,100,0});

        }
        else if(cbd==false)
        {
            send_request({0,0,0},{100,0,0});

        }
        else
        {
            send_request({0,0,0},{0,0,0});
        }*/

		if (!have_detection_) {
            cmd.angular.z = 0.4;   // turn in place
            publisher_->publish(cmd);
            return;
        }
        else if(have_detection_ && x==320){
            cmd.angular.z = 0.0;
            RCLCPP_INFO(this->get_logger(), "##############Point Found############");
        }
        else
        {
            cmd.angular.z = -(320-x)/100;
            publisher_->publish(cmd);
            RCLCPP_INFO(this->get_logger(), "##############Point Found_________Turning");
        }

		float x = last_detection_.points[0].x;
        float radius = last_detection_.points[0].radius;
        float centre_x = 320;

        if (radius < 25.0f && have_detection_) {   // adjust radius threshold by testing
            cmd.linear.x = 0.2f;
        }
        else{
            cmd.linear.x= 0.0f;
        }
        if(!crd && radius>=25.0f)
        {
            cmd.linear.x=-0.2f;
            crd=true;
            count++;
            RCLCPP_INFO(this->get_logger(), "The count is ",count);
            send_request({0,0,0},max_values[count]);
        }
        /*else if(!cgd && radius>=25.0f)
        {
            cmd.linear.x=-0.2f;
            cgd=true;

        }
        else if(!cbd && radius>=25.0f)
        {
            cmd.linear.x=-0.2f;
            cbd=true;

        }*/


        publisher_->publish(cmd);

    }
    /*
    This function constructs a parameter request to update the upper and lower limits.
    The request is sent to the parameter service advertised by the colour detector node.
    The request is assumed to be successful, and there is no attempt made to read a response.
    If you want to access the response, review the multithreaded executor example for a
    service client provided with lecture 5. BGR
    */
    void send_request(std::vector<long int> min_values, std::vector<long int> max_values)
    {
        auto parameter = rcl_interfaces::msg::Parameter();
        auto request = std::make_shared<rcl_interfaces::srv::SetParameters::Request>();

        auto client = this->create_client<rcl_interfaces::srv::SetParameters>("/colour_detector/set_parameters");

        parameter.name = "lower_limits";
        parameter.value.type = rclcpp::ParameterType::PARAMETER_INTEGER_ARRAY; // specified integer array value
        parameter.value.integer_array_value = min_values; //bgr

        request->parameters.push_back(parameter);

        parameter.name = "upper_limits";
        parameter.value.type = rclcpp::ParameterType::PARAMETER_INTEGER_ARRAY; // specified integer array value
        parameter.value.integer_array_value = max_values; //bgr

        request->parameters.push_back(parameter);

        while (!client->wait_for_service(1s)) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
                return;
            }
            RCLCPP_INFO_STREAM(this->get_logger(), "service not available, waiting again...");
        }
        client->async_send_request(request);  // Send the request without ever trying to read the response.

    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
    rclcpp::Subscription<ros366_cv_msgs::msg::ColourPoints>::SharedPtr sub_;
    ros366_cv_msgs::msg::ColourPoints last_detection_;
    std::vector<std::vector<long int>> max_values = {
        {0, 0, 100},
        {0, 100, 0},
        {100, 0, 0}
    };
    int count=0;
	bool have_detection_;
	bool crd, cgd, cbd;
	float x,y;


};


int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ws4>());
    rclcpp::shutdown();
    return 0;
};



#endif //WS4_H
