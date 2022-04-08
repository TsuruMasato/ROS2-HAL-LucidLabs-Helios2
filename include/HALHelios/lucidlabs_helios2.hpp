#ifndef HAL_LUCIDLABS_HELIOS2
#define HAL_LUCIDLABS_HELIOS2


#include <string>
#include <cstdlib>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/time_source.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/create_timer.hpp"
#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "ArenaApi.h"

namespace hal {

class LucidlabsHelios2 : public rclcpp::Node {
 private:
    Arena::ISystem* pSystem = nullptr;
    std::vector<Arena::DeviceInfo> deviceInfos;
    Arena::IDevice* pDevice = nullptr;
    GenApi::INodeMap* pNodeMap = nullptr;
    GenApi::INodeMap* pStreamNodeMap = nullptr;
    Arena::IImage* pImage = nullptr;
    float offX, offY, offZ;
    std::string output_topic;
    int threshold, accum, exposure, mode;
    bool spatial_filter, flying_filter;
    std::string frame_id;
    rclcpp::Clock::SharedPtr clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);

    void runtime();
    rclcpp::TimerBase::SharedPtr timer;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pc_publisher_;

 public:
    LucidlabsHelios2(const rclcpp::NodeOptions & options);

    ~LucidlabsHelios2();

    void get_point_cloud();
};

}  //namespace hal

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(hal::LucidlabsHelios2)

#endif //HAL_LUCIDLABS_HELIOS2
