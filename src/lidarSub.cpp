// Copyright 2016 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// PHS: Original file obtained from: wget -O subscriber_member_function.cpp https://raw.githubusercontent.com/ros2/examples/humble/rclcpp/topics/lidar_subscriber/member_function.cpp 18 Sept, 14:45
// PHS: File renamed from subscriber_member_function.cpp
// PHS: Renamed class from MinimalSubscriber
// PHS: Additional comments added to document the code
// PHS: Modified topic name used

//https://en.cppreference.com/w/cpp/header/functional
#include <functional> // provides the std::bind() and placeholders
#include "sensor_msgs/msg/laser_scan.hpp"

//https://en.cppreference.com/w/cpp/header/memory
#include <memory> // provides std::make_shared<Class>()

//ROS 2 Specific includes
//https://docs.ros2.org/latest/api/rclcpp/rclcpp_8hpp.html
#include "rclcpp/rclcpp.hpp" // includes most common libraries for ROS 2
//https://index.ros.org/p/std_msgs/
//https://docs.ros.org/en/humble/Concepts/Basic/About-Interfaces.html
//#include "std_msgs/msg/u_int32.hpp" // provides basic Unsigned int32 message type for publishing data

//Namespace specified for simplfication when using placeholders library
using std::placeholders::_1;
//sensor_msgs::msg::LaserScan

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the subscriber. */

//Declaring a new class as a subclass of the ROS 2 Node class
class LidarSubscriber : public rclcpp::Node
{
public:
  //Constructor specifying node name for superclass
  LidarSubscriber()
  : Node("lidar_subscriber")
  {
    // Create the instance of the subscriber that will receive messages
    // of type std_msgs/mgs/UInt32 published to the topic "/hello/world"
    // a queue length of 10 is specified here and a reference is given
    // to the topic_callback method that will process messages that are received.
    subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/husky/sensors/lidar2d_0/scan", 10, std::bind(&LidarSubscriber::topic_callback, this, _1));
  }
  

private:
  // Private function that will be triggered automatically when messages are received
  // from the topic /hello/world
  // The parameter specifies the expected message type, which must match
  // the type published to the topic
  void topic_callback(const sensor_msgs::msg::LaserScan & msg)
  {
        //Logger used to print details of the message received (printed to console).
        std::vector<int> hit_indexes;
        std::vector<int> indexes;

      for (size_t i = 0; i < msg.ranges.size(); ++i)
      {
          float range = msg.ranges[i];
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
          std::cout << idx << " ";
        }
        std::cout << std::endl;
      } else {
        RCLCPP_INFO(this->get_logger(), "No nearby frame detected (all beams far or invalid).");
      }

      /*if(!indexes.empty()){
        RCLCPP_INFO(this->get_logger(), "Beams hitting the frame at indexes:");
        std::cout
      }*/

//lidar indexes to be removed: 48 49 50 51 52 73 74 75 76 77 78 79 80 81 82 83 636 637 638 639 640 641 642 643 644 645 646 667 668 669 670 671
  
  }
  void func()
  {
  	
  }

  //Declaration of private fields used for subscriber
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
};

//Main method defining entry point for program
int main(int argc, char * argv[])
{
  //Initialise ROS 2 for this node
  rclcpp::init(argc, argv);

  //Create the instance of the Node subclass and 
  // start the spinner with a pointer to the instance
  rclcpp::spin(std::make_shared<LidarSubscriber>());

  //When the node is terminated, shut down ROS 2 for this node
  rclcpp::shutdown();
  return 0;
}
