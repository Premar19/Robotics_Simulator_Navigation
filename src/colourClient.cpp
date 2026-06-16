//Based on minimal client example from ROS 2 tutorials#include "rclcpp/rclcpp.hpp"
#include "rcl_interfaces/srv/set_parameters.hpp"
#include "rcl_interfaces/msg/parameter.hpp"
#include "rclcpp/rclcpp.hpp"
#include <chrono>
#include <cstdlib>
#include <memory>
#include <vector>

using namespace std::chrono_literals;
/*
Red: Rhigh>=100
Green: Ghigh>=120
Blue: Bhigh>=100
 */

class ColourClient : public rclcpp::Node
{
  public:
    ColourClient() : Node("colourClient")
    {
      timer_ = this->create_wall_timer(1s, std::bind(&ColourClient::cb_timer, this));
      srand(time(0));
    }

    void cb_timer()
    {
        int r_max = rand() % 150 + 105; // max  105-255
        int g_max = rand() % 150 + 105; // max  105-255
        int b_max = rand() % 150 + 105; // max  105-255

        int r_min = rand() % 100; // min  0-100
        int g_min = rand() % 100; // min  0-100
        int b_min = rand() % 100; // min  0-100

        std::vector<long int> max_values = std::vector<long int>{b_max, g_max, r_max};
        std::vector<long int> min_values = std::vector<long int>{b_min, g_min, r_min};
        send_request(min_values, max_values);
    }

    /*
    This function constructs a parameter request to update the upper and lower limits.
    The request is sent to the parameter service advertised by the colour detector node.
    The request is assumed to be successful, and there is no attempt made to read a response.
    If you want to access the response, review the multithreaded executor example for a
    service client provided with lecture 5.
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
    
  private:
    rclcpp::TimerBase::SharedPtr timer_;
};



int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ColourClient>());
  rclcpp::shutdown();

  return 0;
}
