#include <rclcpp/rclcpp.hpp>
#include <action_msgs/msg/goal_status.hpp>
#include <image_processor/ImageProcessor.hpp>

class ImageProcessorServer : public rclcpp::Node
{
public:
  ImageProcessorServer() : Node("image_processor_server")
  {
    server_ = rclcpp_action::create_server<image_processor::action::ImageProcessor>(
        this,
        "image_processor",
        std::bind(&ImageProcessorServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&ImageProcessorServer::handle_cancel, this, std::placeholders::_1),
        std::bind(&ImageProcessorServer::handle_accepted, this, std::placeholders::_1));
  }

private:
  rclcpp_action::Server<image_processor::action::ImageProcessor>::SharedPtr server_;

  rclcpp_action::GoalResponse handle_goal(
      const rclcpp_action::GoalUUID &uuid,
      std::shared_ptr<const image_processor::action::ImageProcessor::Goal> goal)
  {
    (void)uuid;
    RCLCPP_INFO(this->get_logger(), "Received image processing request.");
    rclcpp_action::GoalResponse response = rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    return response;
  }

  rclcpp_action::CancelResponse handle_cancel(
      const std::shared_ptr<rclcpp_action::ServerGoalHandle<image_processor::action::ImageProcessor>> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Received image processing request cancellation.");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<image_processor::action::ImageProcessor>> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Processing image...");
    auto result = std::make_shared<image_processor::action::ImageProcessor::Result>();
    // Process the image and populate 'result' accordingly.

    goal_handle->succeed(result);
    RCLCPP_INFO(this->get_logger(), "Image processing completed.");
  }
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImageProcessorServer>());
  rclcpp::shutdown();
  return 0;
}
