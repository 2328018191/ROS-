#include <rclcpp/rclcpp.hpp>
#include <queue>
#include <chrono>
#include <std_msgs/msg/string.hpp>
#include <curl/curl.h>          // 新增
#include <sstream>              // 新增

using namespace std::chrono_literals;

// 1. 先定义类
class NovelPubNode : public rclcpp::Node
{
public:
  explicit NovelPubNode(const std::string & name)
  : rclcpp::Node(name)
  {
    RCLCPP_INFO(this->get_logger(), "%s 启动!", name.c_str());
  
    publisher_ = this->create_publisher<std_msgs::msg::String>("novel", 10);
    timer_ = this->create_wall_timer(
      5s, [this]() {
        if (!novels_queue_.empty()) {
          auto msg = std_msgs::msg::String();
          msg.data = novels_queue_.front();
          novels_queue_.pop();
          publisher_->publish(msg);
          RCLCPP_INFO(this->get_logger(), "发布: %s", msg.data.c_str());
        } else {
          RCLCPP_INFO(this->get_logger(), "队列为空，暂无内容可发布");
        }
      });
  }
    bool download(const std::string &url)
    {
      RCLCPP_INFO(this->get_logger(), "开始下载：%s", url.c_str());

      CURL *curl = curl_easy_init();
      if (!curl) {
          RCLCPP_ERROR(this->get_logger(), "curl_easy_init 失败");
          return false;
      }
  
      std::string buffer;
      curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
      curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION,
          +[](char *ptr, size_t size, size_t nmemb, void *userdata) -> size_t {
              auto *buf = static_cast<std::string *>(userdata);
              buf->append(ptr, size * nmemb);
              return size * nmemb;
          });
      curl_easy_setopt(curl, CURLOPT_WRITEDATA, &buffer);
  
      CURLcode res = curl_easy_perform(curl);
      if (res != CURLE_OK) {
          RCLCPP_ERROR(this->get_logger(), "下载失败: %s", curl_easy_strerror(res));
          curl_easy_cleanup(curl);
          return false;
      }
  
      std::istringstream stream(buffer);
      std::string line;
      while (std::getline(stream, line))
          if (!line.empty()) novels_queue_.push(line);
  
      RCLCPP_INFO(this->get_logger(), "下载完成，共 %zu 行", novels_queue_.size());
      curl_easy_cleanup(curl);
      return true;
          }

      private:
      rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
      rclcpp::TimerBase::SharedPtr timer_;
      std::queue<std::string> novels_queue_;

};
// 2. 再写 main
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<NovelPubNode>("novel_pub");
  node->download("http://127.0.0.1:8000/local.txt");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}