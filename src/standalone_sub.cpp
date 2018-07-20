#include "sub.h"
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MyTestSub>());
  rclcpp::shutdown();
  return 0;
}
