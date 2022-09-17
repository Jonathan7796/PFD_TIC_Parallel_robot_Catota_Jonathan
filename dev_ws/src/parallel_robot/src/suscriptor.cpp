#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "my_messages/msg/mensajes.hpp" 
#include "std_msgs/msg/float64_multi_array.hpp"

using std::placeholders::_1;

class ControlAction : public rclcpp::Node
{
public:
  ControlAction()
  : Node("effort_commands")
  {
    subscription_ = this->create_subscription<my_messages::msg::Mensajes>("control_actions", 10, std::bind(&ControlAction::topic_callback, this, _1));
      
    publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/effort_controllers/commands",10);
    
  }

private:
  void topic_callback(const my_messages::msg::Mensajes::SharedPtr   msg) //const //const  // CHANGE
  {
        auto message = std_msgs::msg::Float64MultiArray() ;
        
        message.data[0]=msg->posiciones[0]; 
        message.data[1]=msg->posiciones[1];
        message.data[2]=msg->posiciones[2];
        message.data[3]=msg->posiciones[3];
        
     RCLCPP_INFO_STREAM(this->get_logger(), "I heard: '%f'"  <<  message.data[0]);
     RCLCPP_INFO_STREAM(this->get_logger(), "I heard: '%f'"  <<  message.data[1] ) ;
     RCLCPP_INFO_STREAM(this->get_logger(), "I heard: '%f'"  <<  message.data[2] );
     RCLCPP_INFO_STREAM(this->get_logger(), "I heard: '%f'"  <<  message.data[3] );    // CHANGE
     publisher_->publish(message);
  }
   
   rclcpp::Subscription<my_messages::msg::Mensajes>::SharedPtr subscription_;  // CHANGE
   rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControlAction>());
  //rclcpp::shutdown();
  return 0;
}



