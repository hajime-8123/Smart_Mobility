#include <rclcpp/rclcpp.hpp>
#include <image_processor/ImageProcessor.hpp>

class ImageProcessorClient : public rclcpp::Node
{
public:
  ImageProcessorClient() : Node("image_processor_client")
  {
    client_ = rclcpp_action::create_client<image_processor::action::ImageProcessor>(
        this,
        "image_processor");
  }

  void send_goal()
  {
    auto goal_msg = image_processor::action::ImageProcessor::Goal();
    // Populate goal_msg with image data.

    auto send_goal_options = rclcpp_action::Client<image_processor::action::ImageProcessor>::SendGoalOptions();
    send_goal_options.goal_response_callback =
        std::bind(&ImageProcessorClient::goal_response_callback, this, std::placeholders::_1);
    send_goal_options.feedback_callback =
        std::bind(&ImageProcessorClient::feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
    send_goal_options.result_callback =
        std::bind(&ImageProcessorClient::result_callback, this, std::placeholders::_1);
    auto goal_handle_future = client_->async_send_goal(goal_msg, send_goal_options);
  }

private:
  rclcpp_action::Client<image_processor::action::ImageProcessor>::SharedPtr client_;

  void goal_response_callback(std::shared_future<rclcpp_action::ClientGoalHandle<image_processor::action::ImageProcessor>> future)
  {
    RCLCPP_INFO(this->get_logger(), "Image processing goal accepted.");
  }

  void feedback_callback(
      rclcpp_action::ClientGoalHandle<image_processor::action::ImageProcessor>::SharedPtr,
      const std::shared_ptr<const image_processor::action::ImageProcessor::Feedback> feedback)
  {
    // Handle feedback, if necessary.
  }

  void result_callback(const rclcpp_action::ClientGoalHandle<image_processor::action::ImageProcessor>::WrappedResult &result)
  {
    switch (result.code)
    {
      case rclcpp_action::ResultCode::SUCCEEDED:
        // Handle the successful result (e.g., display the processed image).
        RCLCPP_INFO(this->get_logger(), "Image processing succeeded.");
        break;
      default:
        RCLCPP_ERROR(this->get_logger(), "Image processing failed.");
        break;
    }
  }
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ImageProcessorClient>();
  node->send_goal();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
