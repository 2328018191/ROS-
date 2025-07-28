#include "rclcpp/rclcpp.hpp"
#include "chapt/srv/patrol.hpp"
#include <ctime>

using patrol=chapt::srv::Patrol;
using namespace std::chrono_literals;
class turtleclient:public rclcpp::Node
{
    private:
    rclcpp::TimerBase::SharedPtr timer;
    rclcpp::Client<patrol>::SharedPtr patrol_client;


    public:
        turtleclient(const std::string &node_name):Node(node_name)
        {
            srand(time(NULL));

            patrol_client=this->create_client<patrol>("patrol");
            timer=this->create_wall_timer(5s,[&]()->void{
            while (!this->patrol_client->wait_for_service(1s))
            {
                if(!rclcpp::ok())
                {
                    RCLCPP_ERROR(this->get_logger(),"ERROR");
                    return;
                }
                RCLCPP_INFO(this->get_logger(),"等待");
            }
            auto request=std::make_shared<patrol::Request>();
            request->target_x=rand()%15;
            request->target_y=rand()%15;
            this->patrol_client->async_send_request(request,[&]
            (rclcpp::Client<patrol>::SharedFuture result_future)->void{
                auto response =result_future.get();
                if(response->result==patrol::Response::SUCCESS){

                    RCLCPP_INFO(this->get_logger(),"成功");

                }
                if(response->result==patrol::Response::FAIL){
                    RCLCPP_INFO(this->get_logger(),"失败");
            }
            })
            ;})

        ;}

};
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<turtleclient>("turtle_circle");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
