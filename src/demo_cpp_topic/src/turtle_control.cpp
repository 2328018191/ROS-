#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <chrono>
#include "turtlesim/msg/pose.hpp"


using namespace std::chrono_literals;

class turtlecontrol : public rclcpp::Node
{
private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr subscriber_;
    double target_x_=1.0;
    double target_y_=1.0;
    double k_=1.0;
    double max_speed_=3.0;


public:
    explicit turtlecontrol(const std::string &node_name)
        : Node(node_name)
    {
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", 10);
        subscriber_=this->create_subscription<turtlesim::msg::Pose>("/turtle1/pose",10,std::bind(
        &turtlecontrol::on_pose_received,this,std::placeholders::_1));

    }

    void on_pose_received(const turtlesim::msg::Pose::SharedPtr pose)
    {

        auto current_x=pose->x;
        auto current_y=pose->y;
        RCLCPP_INFO(get_logger(),"x=%f,y=%f",current_x,current_y);
        
// 2. 计算当前海龟与目标点的距离差和角度差
        const double dx   = target_x_ - current_x;
        const double dy   = target_y_ - current_y;
        const double distance = std::sqrt(dx * dx + dy * dy);
        const double angle    = std::atan2(dy, dx) - pose->theta;   // 期望朝向 - 当前朝向

// 3. 控制策略
auto msg = geometry_msgs::msg::Twist();

// 距离阈值：0.1 m
if (distance > 0.1)
{
  // 角度阈值：0.2 rad
  if (std::fabs(angle) > 0.2)
  {
    // 先原地转向
    msg.angular.z = (angle > 0) ? 1.0 : -1.0;   // 可按需改为 angle * Kp
  }
  else
  {
    // 朝向已接近，开始前进
    msg.linear.x = k_ * distance;

    // 4. 限制最大线速度
    if (msg.linear.x > max_speed_)
      msg.linear.x = max_speed_;
    else if (msg.linear.x < -max_speed_)
      msg.linear.x = -max_speed_;
  }
}

// 发布速度指令
publisher_->publish(msg);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<turtlecontrol>("turtle_circle");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}