//
// Created by prs49 on 05/11/25.
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
#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"


using namespace std::chrono_literals;
using std::placeholders::_1;

class assignment : public rclcpp::Node
{
public:
    assignment()
    : Node("lidar_frame_detector"),
      front_avg_(0.0), front_left_avg_(0.0), front_right_avg_(0.0),
      left_avg_(0.0), right_avg_(0.0)
    {
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/husky/cmd_vel", 10);

        subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/husky/sensors/lidar2d_0/scan", 10,
            std::bind(&assignment::scan_callback, this, _1));
        sub_ = this->create_subscription<ros366_cv_msgs::msg::ColourPoints>(
          "/colour_detection", 10,
          std::bind(&assignment::color_callback, this, std::placeholders::_1)
       );
        img_subscription = this->create_subscription<sensor_msgs::msg::Image>(
       "/husky/sensors/camera_0/depth/image",
       10,
       std::bind(&assignment::depth_image_callback, this, _1)
);

        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);




        // 10 Hz control loop
        timer_ = this->create_wall_timer(
            100ms, std::bind(&assignment::timer_callback, this));

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
        int width = 20; // guess to average
        int front_idx       = n / 2;
        int left_idx        = n / 3;
        int right_idx       = (2 * n) / 3;

        // Extract key directional distance readings from the LIDAR scan
        // These are used for naviagtion through the corridor and room detection
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
    void color_callback(const ros366_cv_msgs::msg::ColourPoints & msg)
    {
        last_detection_ = msg;
        have_detection_ = (msg.points.size()>0);
        color_detections=msg.points.size();
        x=msg.points[0].x;
        y = msg.points[0].y;

    }
    void depth_image_callback(const sensor_msgs::msg::Image & imgMsg)
    {
        // TF lookup
        std::string fromFrame = "base_link";
        std::string toFrame   = "target_object";

        geometry_msgs::msg::TransformStamped lookup_tf;

        // Convert to CV image
        cv_bridge::CvImagePtr cv_ptr =
          cv_bridge::toCvCopy(imgMsg, sensor_msgs::image_encodings::TYPE_32FC1);

        // Get depth using detected colour point
        float depth = cv_ptr->image.at<float>(y, x);

        // Broadcast transform
        geometry_msgs::msg::TransformStamped t;
        t.header.stamp = imgMsg.header.stamp;
        t.header.frame_id = "camera_0_depth_frame";
        t.child_frame_id  = "target_object";

        t.transform.translation.x = depth;
        t.transform.translation.y = x_offset;
        t.transform.translation.z = y_offset;

        tf2::Quaternion q;
        q.setRPY(0,0,0);
        t.transform.rotation.x = q.x();
        t.transform.rotation.y = q.y();
        t.transform.rotation.z = q.z();
        t.transform.rotation.w = q.w();
        float depth_val = cv_ptr->image.at<float>(y, x);

        if (std::isfinite(depth_val) && depth_val > 0.0) {
           latest_depth_ = depth_val;
           have_depth_ = true;
        }
        else {
           have_depth_ = false; // depth invalid
        }


        tf_broadcaster_->sendTransform(t);

        // Draw and show
        cv::cvtColor(cv_ptr->image, cv_ptr->image, cv::COLOR_GRAY2RGB);
        cv::circle(cv_ptr->image, cv::Point(x, y), 10, CV_RGB(0,0,255), 2);
        cv::imshow(OPENCV_WINDOW, cv_ptr->image);
        cv::waitKey(3);
    }

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


    // ============= CONTROL LOOP =============
    void timer_callback()
    {
        geometry_msgs::msg::Twist cmd;
       switch(caseno)
       {
          // ==============================
          // STATE 1: CORRIDOR NAVIGATION
          // Uses LIDAR to remain centred between corridor walls
          // Publishes velocity commands to /cmd_vel
          // ==============================
          case 1:{


               // Compute linear velocity
               float linear_speed = std::min(((front_avg_ / safe_front_range_) * max_speed_),max_speed_);
               //linear_speed = std::min(linear_speed, max_speed_);
               float angular_speed = 0;

               if (front_avg_ < 0.2 && left_avg_ < 0.2){
                   linear_speed = -2;
                   angular_speed = -2;
               }
               else if (front_avg_ < 0.2 && right_avg_ < 0.2){
                   linear_speed = -2;
                   angular_speed = 2;
               }


               float side_diff = left_avg_ - right_avg_;

               RCLCPP_INFO(this->get_logger(), "The side diff is %.2f ", side_diff);
               angular_speed = (-side_diff / safe_side_range_) * max_speed_;

               angular_speed = std::clamp(angular_speed, -max_speed_, max_speed_);
               to_return_ = angular_speed;

               cmd.linear.x = linear_speed;

               cmd.angular.z = angular_speed;

               publisher_->publish(cmd);

               RCLCPP_INFO(this->get_logger(),
                   "cmd: linear=%.2f m/s, angular=%.2f rad/s | front=%.2f, FL=%.2f, FR=%.2f, L=%.2f, R=%.2f",
                   cmd.linear.x, cmd.angular.z,
                   front_avg_, front_left_avg_, front_right_avg_, left_avg_, right_avg_);
             // Sum multiple directional distances to estimate when the robot enters a large open space (used to detect room entry)
              float open_space_sum = left_avg_ + right_avg_ + front_left_avg_ + front_right_avg_;
               if((open_space_sum) > 25.0)
               {
                  caseno=2;
                  cmd.linear.x = -1.0;
                  publisher_->publish(cmd);
               }
               break;
           }
          // ==============================
          // STATE 2: ROOM ENTRY AND TARGET SEARCH
          // Triggered when a large open space is detected
          // Uses colour camera to search for the green floor target
          // ==============================
          case 2:
          {
             RCLCPP_INFO(this->get_logger(), "Case 2");
             send_request({0,0,0},{127,255,0});
             cmd.linear.x = 0.0;
             publisher_->publish(cmd);
              if (!have_detection_) {
                  cmd.angular.z = 0.4;   // turn in place
                  publisher_->publish(cmd);
                 RCLCPP_INFO(this->get_logger(), "Turning...........");
                  //return;
              }
              else if(have_detection_ && std::abs(320.0-x)<10){
                  cmd.angular.z = 0.0;
                  //caseno=3;
                  RCLCPP_INFO(this->get_logger(), "##############Point Found############");
                 publisher_->publish(cmd);
                 caseno=3;
              }
              else
              {
                 cmd.linear.x=0.5;
                  cmd.angular.z = -(320-x)/100;
                  publisher_->publish(cmd);
                  RCLCPP_INFO(this->get_logger(), "##############Point Found_________Turning");
              }
              break;
           }
          // ==============================
          // STATE 3: MOVE ONTO THE TARGET PLATFORM
          // Triggered when the target platform has been detected by the color camera
          // Tries to keep the target in the center and move in a straight line towards it
          // ==============================
           case 3:
            {
                // === Local constants for Case 3 (alignment & depth thresholds) ===
                const float CENTER_PIXEL_X      = 320.0f;
                const float CENTER_TOLERANCE    = 10.0f;
                const float ANGLE_SCALE         = 200.0f;

                const float DEPTH_FAR           = 4.0f;   // far distance threshold
                const float DEPTH_MEDIUM        = 2.0f;   // medium distance threshold
                const float DEPTH_NEAR          = 3.0f;   // near stopping threshold

                const float SPEED_FAST          = 1.0f;
                const float SPEED_MEDIUM        = 0.3f;
                const float SPEED_SLOW          = 0.1f;

                RCLCPP_INFO(this->get_logger(), "Case 3");

                if (have_detection_ && have_depth_)
                {
                    float error_x = CENTER_PIXEL_X - x;

                    // rotational correction
                    cmd.angular.z = -(error_x / ANGLE_SCALE);

                    // centred?
                    if (std::abs(error_x) < CENTER_TOLERANCE)
                    {
                        RCLCPP_INFO(this->get_logger(),
                            "Depth to object: %.2f m", latest_depth_);

                        if (latest_depth_ > DEPTH_FAR)
                        {
                            cmd.linear.x = SPEED_FAST;
                        }
                        else if (latest_depth_ > DEPTH_MEDIUM)
                        {
                            cmd.linear.x = SPEED_MEDIUM;
                            RCLCPP_WARN(this->get_logger(),
                                "### Approaching target — slowing down ###");
                        }
                        else if (latest_depth_ <= DEPTH_NEAR)
                        {
                            cmd.linear.x = 0.0;
                            cmd.angular.z = 0.0;

                            RCLCPP_WARN(this->get_logger(),
                                "### Parking distance reached — stopping ###");

                            distance_at_stop_ = latest_depth_;
                            publisher_->publish(cmd);
                            caseno = 4;
                            break;
                        }
                    }
                    else
                    {
                        // not aligned—move slowly & turn
                        cmd.linear.x = SPEED_SLOW;
                    }

                    publisher_->publish(cmd);
                }
                else
                {
                    RCLCPP_WARN(this->get_logger(), "Lost detection in Case 3 — stopping.");
                }

                break;
            }

          // ==============================
          // STATE 4: GET ALL 4 WHEELS ON TARGET
          // Robot slows down when it approaches the near the target to limit any extra distance covered due to its speed and time it takes to stop
          // Uses the remaining distance from depth camera as an indication on how much further it needs to go
          // ==============================
          case 4:
          {
             RCLCPP_INFO(this->get_logger(), "Case 4: Final Approach onto Platform (Calculated).");

             static bool initialized = false;
             static rclcpp::Time start_time;
             const float move_speed = 0.3; // m/s (Slow and controlled)
             const double required_move_distance = distance_at_stop_ + target_platform_penetration_; //total distance to travel to be right on top of the block

             // Calculate time required for the calculated distance
             const double required_time = required_move_distance / move_speed;

             if (!initialized) {
                start_time = this->get_clock()->now();
                initialized = true;
                RCLCPP_INFO(this->get_logger(),
                   "Starting final drive: Moving %.2f m at %.2f m/s for %.2f seconds.",
                   required_move_distance, move_speed, required_time);
             }

             rclcpp::Time current_time = this->get_clock()->now();
             rclcpp::Duration elapsed_time = current_time - start_time;

             // Check if the required time has elapsed
             if (elapsed_time.seconds() < required_time) {
                // Keep driving forward
                cmd.linear.x = move_speed;
                cmd.angular.z = 0.0;
                publisher_->publish(cmd);
                RCLCPP_INFO(this->get_logger(),
                   "Time remaining: %.2f seconds", required_time - elapsed_time.seconds());
             }
             else {
                // Stop, we have moved the required distance
                cmd.linear.x = 0.0;
                cmd.angular.z = 0.0;
                publisher_->publish(cmd);
                RCLCPP_WARN(this->get_logger(), "### Final position reached — STOPPED ON PLATFORM ###");

                // Reset and move to the next state
                initialized = false;
                caseno = 5;
             }

             break;
          }
          // ==============================
          // STATE 5: SEARCH FOR COLOURED CUBE
          // Triggered after the robot has all 4 wheels on the target platform
          // Uses colour camera to search for the purple coloured block
          // ==============================
          case 5:
          {
             RCLCPP_INFO(this->get_logger(), "Case 5: Changing color detector values.");
             send_request({55,0,0},{255,0,255});
             if (!have_detection_) {
                cmd.angular.z = 0.7;   // turn in place
                publisher_->publish(cmd);
                RCLCPP_INFO(this->get_logger(), "Turning...........");
                //return;
             }
             else if((have_detection_ && x==320) || (std::abs(320-x) < 5)){
                cmd.angular.z = 0.0;
                //caseno=3;
                RCLCPP_INFO(this->get_logger(), "##############Point Found############");
                publisher_->publish(cmd);
                caseno=6;
             }
             else
             {
                cmd.angular.z = -(320-x)/200;
                publisher_->publish(cmd);
                RCLCPP_INFO(this->get_logger(), "##############Point Found_________Turning");
             }
             break;
          }
           // ==============================
           // STATE 6: REPORT TRANSFORM SYSTEM SHUTDOWN
           // Reports the transform from the odom to the purple coloured cube
           // Cleanly shuts down the node after completion
           // ==============================
          case 6:
          {
             RCLCPP_INFO(this->get_logger(), "Case 6: Looking up transform from odom to target_object.");

             // Define the source and target frames for the lookup
             std::string fromFrame = "odom"; // The world frame (Source)
             std::string toFrame   = "target_object"; // The detected object frame (Target)
             geometry_msgs::msg::TransformStamped transform_odom_to_object;

             // Look up the transformation from odom to the object
             try {
                transform_odom_to_object = tf_buffer_->lookupTransform(
                   fromFrame, toFrame,
                   tf2::TimePointZero); // Request the latest available transform

                // Log the successfully retrieved transform
                RCLCPP_INFO_STREAM(this->get_logger(),
                   "Transform from odom to target_object obtained:\n\t"
                   << "Translation (x, y, z): "
                   << transform_odom_to_object.transform.translation.x << ", "
                   << transform_odom_to_object.transform.translation.y << ", "
                   << transform_odom_to_object.transform.translation.z
                   << "\n\t"
                   << "Rotation (x, y, z, w): "
                   << transform_odom_to_object.transform.rotation.x << ", "
                   << transform_odom_to_object.transform.rotation.y << ", "
                   << transform_odom_to_object.transform.rotation.z << ", "
                   << transform_odom_to_object.transform.rotation.w
                );
                rclcpp::shutdown();
             }
             catch (const tf2::TransformException & ex) {
                // Handle the exception if the transform is not available
                RCLCPP_ERROR(
                   this->get_logger(),
                   "Could not transform %s to %s: %s",
                   toFrame.c_str(), fromFrame.c_str(), ex.what());
             }

             break;
          }
        }
        }

    // ROS objects
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
    ros366_cv_msgs::msg::ColourPoints last_detection_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr img_subscription;
    rclcpp::Subscription<ros366_cv_msgs::msg::ColourPoints>::SharedPtr sub_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    const std::string OPENCV_WINDOW = "Depth Window";
    bool have_detection_=false;
    bool have_depth_=false;
    float x,y,depth, x_offset, y_offset;
    int color_detections;
    // Laser region averages
    float front_avg_, front_left_avg_, front_right_avg_, left_avg_, right_avg_;
    float final_approach_distance_ = 1.5;
    float distance_at_stop_ = 0.0;
    float target_platform_penetration_ = 0.17;

    // Parameters
    float safe_front_range_=1;
    float safe_side_range_ = 0.5;
    float max_speed_ = 5.0;
    float to_return_= 0.0;
    int caseno=1;
    float latest_depth_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<assignment>());
    rclcpp::shutdown();
    return 0;
}


