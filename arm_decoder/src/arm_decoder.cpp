#include <arm_decoder/arm_decoder.hpp>
using namespace std::chrono_literals;
namespace arm_decoder
{
  Arm_decoder::Arm_decoder()
  :rclcpp::Node("arm_decoder"),
  velocities{0.0},
  efforts{0.0},
  positions{0.0}
  {
    pub=this->create_publisher<sensor_msgs::msg::JointState>("arm_states", 10);
    sub= this->create_subscription<std_msgs::msg::UInt8MultiArray>(
        "can_rx", 10, std::bind(&Arm_decoder::callback, this, std::placeholders::_1));
    timer=this->create_wall_timer(
      50ms, std::bind(&Arm_decoder::publisher, this));
      RCLCPP_INFO(this->get_logger(),"Arm_Decoder Started!") ;
    }

    Arm_decoder::~Arm_decoder(){
    RCLCPP_INFO(this->get_logger(),"Arm_Decoder Killed!") ;
    }

    void Arm_decoder::callback(const std_msgs::msg::UInt8MultiArray::SharedPtr arr)
    {
        int index=static_cast<int>(arr->data[0]);
        if(20<index && 28>index)
        {
          float temp = convertFloat16ToFloat32(arr->data[1], arr->data[2]);
          if (std::isnan(temp)){
            temp=0;
          }
          positions[index-21]=static_cast<float>(temp);
          velocities[index-21]=static_cast<float>(convertFloat16ToFloat32(arr->data[3],arr->data[4]));
          efforts[index-21]=static_cast<float>(convertFloat16ToFloat32(arr->data[5],arr->data[6]));
        }
    }

    void Arm_decoder::publisher()
    {
        sensor_msgs::msg::JointState message;
        message.name={"Rotational1","Rotational2","Rotational3","Rotational4","Rotational5","Rotational6","Slider1"};
        message.header.stamp = this->get_clock()->now();
        message.velocity = std::vector<double>(velocities.begin(), velocities.end());
        message.effort = std::vector<double>(efforts.begin(), efforts.end());
        message.position = std::vector<double>(positions.begin(), positions.end());
        pub->publish(message);
    }

    float Arm_decoder::convertFloat16ToFloat32(uint8_t byte1, uint8_t byte2) {
        uint16_t h = (static_cast<uint16_t>(byte2) << 8) | byte1;
        uint16_t h_exp = (h & 0x7C00) >> 10; 
        uint16_t h_sig = h & 0x03FF;         
        uint32_t f;

        if (h_exp == 0) {
            if (h_sig == 0) {
                f = (h & 0x8000) << 16; 
            } else {
                h_sig <<= 1;
                while ((h_sig & 0x0400) == 0) {
                    h_sig <<= 1;
                    h_exp--;
                }
                h_exp++;
                h_sig &= ~(0x0400);
                f = ((h & 0x8000) << 16) | ((h_exp + 112) << 23) | (h_sig << 13);
            }
        } else if (h_exp == 0x1F) {
            // INF/NaN
            f = ((h & 0x8000) << 16) | 0x7F800000 | (h_sig << 13);
        } else {
            f = ((h & 0x8000) << 16) | ((h_exp + 112) << 23) | (h_sig << 13);
        }

        float result;
        std::memcpy(&result, &f, sizeof(f));
        return result;
    }

  }

  int main(int argc, char** argv)
  {
    rclcpp::init(argc, argv);


    std::shared_ptr<arm_decoder::Arm_decoder> node = std::make_shared<arm_decoder::Arm_decoder>();

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();

    return 0;
  }