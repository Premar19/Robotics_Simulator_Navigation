//
// Created by prs49 on 13/11/25.
//
#include "cv_bridge/cv_bridge.h"
#include <opencv2/opencv.hpp>
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
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
using namespace std::chrono_literals;
using std::placeholders::_1;

class ws5 : public rclcpp::Node{
public:
    ws5()
    : Node("lidar_frame_detector")
    {
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/husky/cmd_vel", 10);

        subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/husky/sensors/lidar2d_0/scan", 10,
            std::bind(&ws5::scan_callback, this, _1));

        img_subscription = this->create_subscription<sensor_msgs::msg::Image>(
        "/husky/sensors/camera_0/depth/image",
        10, std::bind(&ws5::depth_image_callback, this, _1)
        );

        //std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
        //tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        sub_ = this->create_subscription<ros366_cv_msgs::msg::ColourPoints>(
            "/colour_detection", 10,
            std::bind(&ws5::color_callback, this, std::placeholders::_1)
        );
        RCLCPP_INFO(this->get_logger(), "Colour Chase Node Started");
        // 10 Hz control loop
        timer_ = this->create_wall_timer(
            100ms, std::bind(&ws5::timer_callback, this));


        RCLCPP_INFO(this->get_logger(), "LidarFrameDetector initialized for autonomous driving.");
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
    void depth_image_callback(const sensor_msgs::msg::Image & imgMsg)
    {
        //WS6
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
        std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
        tf_buffer_ =
        std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ =
        std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        std::string fromFrame = "base_link";
        std::string toFrame = "target_object";
        geometry_msgs::msg::TransformStamped t;
        // Look up for the transformation between target frames
        /**try {
            t = tf_buffer_->lookupTransform(
            fromFrame, toFrame,
            tf2::TimePointZero);
            RCLCPP_INFO_STREAM(this->get_logger(), "Transform obtained transltion:\n\t"
            << t.transform.translation.x << ", "
            << t.transform.translation.y << ", "
            << t.transform.translation.z << ", "
            << "\nRotation:\n\t"
            << t.transform.rotation.x << ", "
            << t.transform.rotation.y << ", "
            << t.transform.rotation.z << ", "
            << t.transform.rotation.w << ", "
            );
        } catch (const tf2::TransformException & ex) {
            RCLCPP_INFO(
            logger, "Could not transform %s to %s: %s",
            toFrame.c_str(), fromFrame.c_str(), ex.what());
            //return;
        }*/
        //WS5
        /**std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        // Convert ROS Image to CV Image
        rclcpp::Time timestamp = rclcpp::Time(imgMsg.header.stamp);
        geometry_msgs::msg::TransformStamped t;
        // Configure the message header, in particular parent-child frames
        t.header.stamp = timestamp;
        t.header.frame_id = "camera_0_depth_frame";
        t.child_frame_id = "target_object";
        // TODO: Calculate x_offset and y_offset
        // In this coordinate frame, x is forward, y is right and z is up
        // hence the x and y offsets being applied to the y and z coordinates.
        // Additional offsets could be added here later for grasping
        t.transform.translation.x = depth;
        t.transform.translation.y = x_offset;
        t.transform.translation.z = y_offset;
        // We need to supply a rotation, but currently don't care so set it to 0 in Euler coordinates.
        // A function is then used to generate the quaternion for us
        tf2::Quaternion q;
        q.setRPY(0, 0, 0);
        t.transform.rotation.x = q.x();
        t.transform.rotation.y = q.y();
        t.transform.rotation.z = q.z();
        t.transform.rotation.w = q.w();
        // Send the transformation
        tf_broadcaster_->sendTransform(t);
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(imgMsg,
        sensor_msgs::image_encodings::TYPE_32FC1);
        // Extract the depth at the coordinate from colour detection
        // TODO: Obtain X and Y coodinates from colour detection callback and convert to ints
        float depth = cv_ptr    ->image.at<float>(y,x);
        // draw a red circle of radius 10 around the x,y point with line width 2
        cv::cvtColor(cv_ptr->image, cv_ptr->image, cv::COLOR_GRAY2RGB);
        cv::circle(cv_ptr->image, cv::Point(x, y), 10, CV_RGB(0,0,255), 2);
        // Update GUI Window
        cv::imshow(OPENCV_WINDOW, cv_ptr->image);
        cv::waitKey(3);*/
    }
    void timer_callback()
    {
        geometry_msgs::msg::Twist cmd;
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
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr img_subscription;

    const std::string OPENCV_WINDOW = "Depth Perception";
    std::vector<std::vector<long int>> max_values = {
        {0, 0, 100},
        {0, 100, 0},
        {100, 0, 0}
    };
    bool have_detection_;
    float x,y, depth, x_offset, y_offset;

};


int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    rclcpp::spin(std::make_shared<ws5>());
    rclcpp::shutdown();
    return 0;
};




