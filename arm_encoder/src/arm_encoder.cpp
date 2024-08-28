#include <arm_encoder/arm_encoder.hpp>
using namespace std::chrono_literals;
namespace arm_encoder
{
  Arm_encoder::Arm_encoder()
  :rclcpp::Node("arm_decoder"),
  velocities{0.0},
  heartbeat(0)
  {
    pub=this->create_publisher<std_msgs::msg::UInt8MultiArray>("can_tx", 10);
    sub= this->create_subscription<sensor_msgs::msg::JointState>(
        "arm_commands", 10, std::bind(&Arm_encoder::callback, this, std::placeholders::_1));
    timer=this->create_wall_timer(
      50ms, std::bind(&Arm_encoder::publisher, this));
      RCLCPP_INFO(this->get_logger(),"Arm_Encoder Started!") ;
    }

    Arm_encoder::~Arm_encoder(){
    RCLCPP_INFO(this->get_logger(),"Arm_Encoder Killed!") ;
    }

    void Arm_encoder::callback(const sensor_msgs::msg::JointState::SharedPtr message)
    {
         std::copy(message->velocity.begin(), message->velocity.end(), velocities.begin());
         heartbeat= 0;
    }

    void Arm_encoder::send()
    {
        for (int i=0; i<7; i++)
        {
            std_msgs::msg::UInt8MultiArray msg;
            float velocity = velocities[i];
            uint8_t data[3];
            data[0] = static_cast<uint8_t>(i + 21);
            half_float::half velocity_half = static_cast<half_float::half>(velocity);
            uint16_t* velocity_bytes = reinterpret_cast<uint16_t*>(&velocity_half);
            std::memcpy(&data[1], velocity_bytes, 2);
            msg.data.assign(data, data + 3);
            pub->publish(msg);
        }
    }

    void Arm_encoder::publisher()
    {
        heartbeat++;
        if (heartbeat>10)
        {
            velocities={0.0,0.0,0.0,0.0,0.0,0.0,0.0};
        }
        send();
    }
  }

  int main(int argc, char** argv)
  {
    rclcpp::init(argc, argv);


    std::shared_ptr<arm_encoder::Arm_encoder> node = std::make_shared<arm_encoder::Arm_encoder>();

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();

    return 0;
  }