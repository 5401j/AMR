#include <memory>
#include <string>
#include <iomanip>
#include <vector>
#include <chrono>
#include <time.h>
#include <cmath>
#include <sys/types.h>
#include <unistd.h>

using namespace std;

#include "rclcpp/rclcpp.hpp"
#include "task_interfaces/srv/lidar_task.hpp"

void task(const std::shared_ptr<task_interfaces::srv::LidarTask::Request> request,
          std::shared_ptr<task_interfaces::srv::LidarTask::Response>      response)
{
  if(request->request=="DoTask"){
    cout<<"you call me!!!!"<<endl;
    system("ros2 run pcl_process publish_cloud");
    cout<<"wait......."<<endl;
    // kill(pid,SIGINT);
    // cout<<"you kill me!!!!"<<endl;
    response->response="Done";
    
  }else response->response="Failed";
  
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("LiDAR_task_server");

  rclcpp::Service<task_interfaces::srv::LidarTask>::SharedPtr service =
    node->create_service<task_interfaces::srv::LidarTask>("LiDAR_task", &task);

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Ready to do LiDAR task.");

  rclcpp::spin(node);
  rclcpp::shutdown();
}
