#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
using std::placeholders::_1;

class MinimalSubscriber : public rclcpp::Node
{
public:
  MinimalSubscriber()
  : Node("minimal_subscriber")
  {
    // 创建一个订阅者，订阅 "chatter" 话题，队列大小为 10
    // 当收到消息时，调用 topic_callback 函数
    subscription_ = this->create_subscription<std_msgs::msg::String>(
      "chatter", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));
  }

private:
  void topic_callback(const std_msgs::msg::String & msg) const
  {
    // 打印收到的消息
    RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg.data.c_str());
  }
  
  // 声明成员变量
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  // 初始化 ROS 2
  rclcpp::init(argc, argv);
  
  // 创建并运行节点，等待消息
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  
  // 关闭 ROS 2
  rclcpp::shutdown();
  return 0;
}