// Incluya archivos de encabezado importantes de C++ que proporcionen clase
// plantillas para operaciones útiles.
#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "my_messages/msg/mensajes.hpp"                                            // CHANGE

using namespace std::chrono_literals;

class MinimalPublisher : public rclcpp::Node
{
public:
  MinimalPublisher()
  : Node("minimal_publisher"), count_(0)
  {
    publisher_ = this->create_publisher<my_messages::msg::Mensajes>("topic", 10);  // CHANGE
    timer_ = this->create_wall_timer(
      500ms, std::bind(&MinimalPublisher::timer_callback, this));
  }

private:
  void timer_callback()
  {
    auto message = my_messages::msg::Mensajes();                               // CHANGE

       for( int i = 0; i < 4; i++)
        {
          message.dato3[i]=this->count_++;
          
        }
   // message.dato3[0] = this->count_++;                                                     // CHANGE
    RCLCPP_INFO_STREAM(this->get_logger(), "Publishing: '" << message.dato3[0] << "'");
     RCLCPP_INFO_STREAM(this->get_logger(), "Publishing: '" << message.dato3[1] << "'") ;
      RCLCPP_INFO_STREAM(this->get_logger(), "Publishing: '" << message.dato3[2] << "'");
       RCLCPP_INFO_STREAM(this->get_logger(), "Publishing: '" << message.dato3[3] << "'");
      //  RCLCPP_INFO_STREAM(this->get_logger(), "Publishing: '" << message.dato3[4] << "'");   // CHANGE
   // publisher_->publish(message);
  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<my_messages::msg::Mensajes>::SharedPtr publisher_;             // CHANGE
  size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}