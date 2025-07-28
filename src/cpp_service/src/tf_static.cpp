#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/static_transform_broadcaster.h"

class tf_static:public rclcpp::Node
{
private:
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster>broadcaster;
public:
    tf_static():Node("tf_static")
    {
    broadcaster=std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
    this->publish_tf();

    }

    void publish_tf()

    {
        geometry_msgs::msg::TransformStamped transform;
        transform.header.stamp=this->get_clock()->now();
        transform.header.frame_id="map";
        transform.child_frame_id="target_point";
        transform.transform.translation.x=5.0;
        tf2::Quaternion quat;
        quat.setRPY(0,0,60*M_PI/180);
        transform.transform.rotation=tf2::toMsg(quat);
        broadcaster->sendTransform(transform);
    }
};


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<tf_static>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}


