#include "rclcpp/rclcpp.hpp"

class AlliedVisionController : public rclcpp::Node
{
  public:
    AlliedVisionController()
        : Node("alliedvision_controller")
    {
    }

  private:
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AlliedVisionController>());
    rclcpp::shutdown();
    return 0;
}
