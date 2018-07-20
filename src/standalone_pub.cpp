#include "pub.h"
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MyTestPub>());
  rclcpp::shutdown();
  return 0;
}
