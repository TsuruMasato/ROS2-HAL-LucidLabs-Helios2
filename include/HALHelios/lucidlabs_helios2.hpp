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

class LucidlabsHelios2 : public rclcpp::Node, public Arena::IImageCallback {
 private:
   Arena::ISystem* pSystem = nullptr;
   std::vector<Arena::DeviceInfo> deviceInfos;
   Arena::IDevice* pDevice = nullptr;
   GenApi::INodeMap* pNodeMap = nullptr;
   GenApi::INodeMap* pStreamNodeMap = nullptr;
   float offX, offY, offZ;
   float scaleX, scaleY, scaleZ;
   std::string output_topic;
   int confidence_filter_threshold, flying_filter_threshold, accum, exposure, mode, hdr_mode;
   bool confidence_filter, spatial_filter, flying_filter;
   bool structured_cloud, publish_intensity;
   std::string frame_id;
   rclcpp::Clock::SharedPtr clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);

   Arena::IDevice* findDevice();

   Arena::IImage* try_get_image();
   void runtime();
   void OnImage(Arena::IImage* pImage) override;

   template <typename PointT>
   void get_point_cloud(Arena::IImage* pImage);

   rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pc_publisher_;

   enum class cam_model {HELIOS_2, HELIOS_2_PLUS};
   cam_model m;
   Arena::IImageCallback* pCallbackHandler;
   
 public:
   LucidlabsHelios2(const rclcpp::NodeOptions & options);

   ~LucidlabsHelios2();
};

}  //namespace hal

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(hal::LucidlabsHelios2)

#endif //HAL_LUCIDLABS_HELIOS2
