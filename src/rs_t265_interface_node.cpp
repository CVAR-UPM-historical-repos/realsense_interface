// "Copyright [year] <Copyright Owner>"

#include "rs_t265_interface.hpp"

int main(int argc, char * argv[])
{
  // find_device_with_streams
  std::cout << "Starting sensor_combined listener node... " << std::endl;
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);
  rclcpp::init(argc, argv);
  auto ptr = std::make_shared<RsT265Interface>();
  ptr->setupOdom();
  ptr->setupTf();
  rclcpp::Rate r(200);
  
  while(rclcpp::ok()){
    ptr->runOdom();
    rclcpp::spin_some(ptr);
    r.sleep();
  }

  rclcpp::shutdown();
  return 0;
}
