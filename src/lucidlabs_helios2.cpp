#include <iostream>
#include <chrono>
#include <cmath>
#include <string>
#include <functional>
#include <cstdlib>
#include <memory>
#include <cassert>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/time_source.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include <pcl_conversions/pcl_conversions.h>

#include "ArenaApi.h"

#define SYSTEM_TIMEOUT 2000
#define IMAGE_TIMEOUT 2000

using std::placeholders::_1;

class HALHeliosNode : public rclcpp::Node {
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

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pc_publisher_;

 public:
    HALHeliosNode() : Node("hal_lucidlabs_helios2") {
        output_topic = declare_parameter<std::string>("output_topic", "/point_cloud");
        frame_id = declare_parameter<std::string>("frame_id", "camera");
        threshold = declare_parameter<int>("threshold", 0);
        exposure = declare_parameter<int>("exposure", 0);
        mode = declare_parameter<int>("mode", 0);
        accum = declare_parameter<int>("accumulate_frames", 1);
        spatial_filter = declare_parameter<bool>("spatial_filter", true);
        flying_filter = declare_parameter<bool>("flying_filter", true);

        try {
            pSystem = Arena::OpenSystem();
            pSystem->UpdateDevices(SYSTEM_TIMEOUT);
            deviceInfos = pSystem->GetDevices();
            if (deviceInfos.size() == 0) {
                RCLCPP_ERROR(get_logger(), "No camera connected");
                std::abort();
            }

            unsigned int id;
            unsigned int found_id;
            bool found = false;
            for (id = 0; id < deviceInfos.size(); id++) {
                RCLCPP_INFO(get_logger(), "Model name: %s", deviceInfos[id].ModelName().c_str());
                if (deviceInfos[id].ModelName() == "HLT003S-001" // Helios2
                or deviceInfos[id].ModelName() == "HTP003S-001"  // Helios2+
                ) {
                    found = true;
                    found_id = id;
                }
            }

            if (!found) {
                RCLCPP_ERROR(get_logger(), "No Helios 2 Camera found.");
                std::abort();
            }
            try {
                pDevice = pSystem->CreateDevice(deviceInfos[found_id]);
            } catch (GenICam::GenericException &ge) {
                RCLCPP_ERROR(get_logger(), "GenICam CreateDevice - exception thrown: %s", ge.what());
                std::abort();
            }
            pNodeMap = pDevice->GetNodeMap();
            pStreamNodeMap = pDevice->GetTLStreamNodeMap();

            GenApi::CEnumerationPtr pModeSel = pNodeMap->GetNode("Scan3dModeSelector");
            GenApi::CEnumEntryPtr pModeProcessed = pModeSel->GetEntryByName("Processed");
            pModeSel->SetIntValue(pModeProcessed->GetValue());

            Arena::SetNodeValue<bool>(pDevice->GetNodeMap(), "ChunkModeActive", false);

            GenApi::CEnumerationPtr pStreamBufferHandlingMode = pStreamNodeMap->GetNode("StreamBufferHandlingMode");
            GenApi::CEnumEntryPtr pNewestOnly = pStreamBufferHandlingMode->GetEntryByName("NewestOnly");
            pStreamBufferHandlingMode->SetIntValue(pNewestOnly->GetValue());

            GenApi::CEnumerationPtr pPixelMode = pNodeMap->GetNode("PixelFormat");
            GenApi::CEnumEntryPtr pCoord3D_ABC16s = pPixelMode->GetEntryByName("Coord3D_ABC16");
            pPixelMode->SetIntValue(pCoord3D_ABC16s->GetValue());

            /*
            GenApi::CIntegerPtr pDeviceStreamChannelPacketSize = pNodeMap->GetNode("DeviceStreamChannelPacketSize");
            pDeviceStreamChannelPacketSize->SetValue(pDeviceStreamChannelPacketSize->GetMax());
            std::cout << "MTU: " << pDeviceStreamChannelPacketSize->GetMax() << std::endl;
            */

            Arena::SetNodeValue<bool>(pDevice->GetTLStreamNodeMap(), "StreamAutoNegotiatePacketSize", true);
            Arena::SetNodeValue<bool>(pDevice->GetTLStreamNodeMap(), "StreamPacketResendEnable", true);

            /*
            Enumeration: 'ExposureTimeSelector'
                EnumEntry: 'Exp62_5Us'
                EnumEntry: 'Exp250Us'
                EnumEntry: 'Exp1000Us'
            */
            static std::vector<std::string> exposures = {
                "Exp62_5Us",
                "Exp250Us",
                "Exp1000Us"
            };
            GenApi::CEnumerationPtr pExposureTime = pNodeMap->GetNode("ExposureTimeSelector");
            GenApi::CEnumEntryPtr pExp = pExposureTime->GetEntryByName(exposures[exposure].c_str());
            pExposureTime->SetIntValue(pExp->GetValue());
            RCLCPP_INFO(get_logger(), "Exposure setting: %s", exposures[exposure].c_str());

            /*
            Enumeration: 'Scan3dOperatingMode'
                EnumEntry: 'HighSpeedDistance2500mmSingleFreq'
                EnumEntry: 'HighSpeedDistance1250mmSingleFreq'
                EnumEntry: 'HighSpeedDistance625mmSingleFreq'
                EnumEntry: 'Freq90MHz' (Not available)
                EnumEntry: 'Distance8300mmMultiFreq'
                EnumEntry: 'Distance6000mmSingleFreq'
                EnumEntry: 'Distance5000mmMultiFreq'
                EnumEntry: 'Distance4000mmSingleFreq'
                EnumEntry: 'Distance3000mmSingleFreq'
                EnumEntry: 'Distance1250mmSingleFreq'
            */
            static std::vector<std::string> modes = {
               "HighSpeedDistance2500mmSingleFreq",
               "HighSpeedDistance1250mmSingleFreq",
               "HighSpeedDistance625mmSingleFreq",
               "Distance8300mmMultiFreq",
               "Distance6000mmSingleFreq",
               "Distance5000mmMultiFreq",
               "Distance4000mmSingleFreq",
               "Distance3000mmSingleFreq",
               "Distance1250mmSingleFreq"
            };
            GenApi::CEnumerationPtr pOperatingMode = pNodeMap->GetNode("Scan3dOperatingMode");
            GenApi::CEnumEntryPtr pMode = pOperatingMode->GetEntryByName(modes[mode].c_str());
            pOperatingMode->SetIntValue(pMode->GetValue());
            RCLCPP_INFO(get_logger(), "Mode setting: %s", modes[mode].c_str());

            GenApi::CEnumerationPtr pScan3DCoordinateSelector = pNodeMap->GetNode("Scan3dCoordinateSelector");
            GenApi::CEnumEntryPtr pCoordinateA = pScan3DCoordinateSelector->GetEntryByName("CoordinateA");
            GenApi::CEnumEntryPtr pCoordinateB = pScan3DCoordinateSelector->GetEntryByName("CoordinateB");
            GenApi::CEnumEntryPtr pCoordinateC = pScan3DCoordinateSelector->GetEntryByName("CoordinateC");
            GenApi::CFloatPtr pScan3DCoordinateOffset = pNodeMap->GetNode("Scan3dCoordinateOffset");

            Arena::SetNodeValue<bool>(pNodeMap, "Scan3dConfidenceThresholdEnable", true);

            GenApi::CIntegerPtr pThresh = pNodeMap->GetNode("Scan3dConfidenceThresholdMin");
            pThresh->SetValue(threshold);

            GenApi::CBooleanPtr pSpatialFilter = pNodeMap->GetNode("Scan3dSpatialFilterEnable");
            pSpatialFilter->SetValue(spatial_filter);

            GenApi::CBooleanPtr pFlyingFilter = pNodeMap->GetNode("Scan3dFlyingPixelsRemovalEnable");
            pFlyingFilter->SetValue(flying_filter);

            Arena::SetNodeValue<int64_t>(pDevice->GetNodeMap(), "Scan3dImageAccumulation", accum);

            pScan3DCoordinateSelector->SetIntValue(pCoordinateA->GetValue());
            offX = pScan3DCoordinateOffset->GetValue();
            pScan3DCoordinateSelector->SetIntValue(pCoordinateB->GetValue());
            offY = pScan3DCoordinateOffset->GetValue();
            pScan3DCoordinateSelector->SetIntValue(pCoordinateC->GetValue());
            offZ = pScan3DCoordinateOffset->GetValue();

            pDevice->StartStream();
        } catch (GenICam::GenericException &ge) {
            RCLCPP_ERROR(get_logger(), "GenICam exception thrown: %s", ge.what());
            std::abort();
        } catch (std::exception &ex) {
            RCLCPP_ERROR(get_logger(), "Standard exception thrown: %s", ex.what());
            std::abort();
        } catch (...) {
            RCLCPP_ERROR(get_logger(), "Unexpected exception thrown");
            std::abort();
        }

        pc_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(output_topic, 1);

        while (rclcpp::ok()) {
            get_point_cloud();
        }
    }

    ~HALHeliosNode() {
        if (pDevice) {
            pDevice->StopStream();
            pSystem->DestroyDevice(pDevice);
        }
        if (pSystem) {
            Arena::CloseSystem(pSystem);
        }
    }

    void get_point_cloud() {
        std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
        pImage = pDevice->GetImage(IMAGE_TIMEOUT);
        assert(pImage != nullptr);
        pcl::PointCloud<pcl::PointXYZ> cloud;
        sensor_msgs::msg::PointCloud2 msg;
        cloud.reserve(640*480);

        if (pImage->IsIncomplete()) {
            RCLCPP_WARN(get_logger(), "Incomplete frame");
            pDevice->RequeueBuffer(pImage);
            return;
        }

        const uint16_t *data = (uint16_t*)(pImage->GetData());
        uint16_t A, B, C;

        for (int o = 0; o < 640*480; o++) {
            A = data[o*3+0];
            B = data[o*3+1];
            C = data[o*3+2];
            if (A != 0xFFFF && B != 0xFFFF && C != 0xFFFF && (C*0.25f + offZ) > 300.0f/*mm*/) {
                cloud.emplace_back( (A*0.25f+offX)/1000,
                                    (B*0.25f+offY)/1000,
                                    (C*0.25f+offZ)/1000);
            }
        }
        pDevice->RequeueBuffer(pImage);

        pcl::toROSMsg(cloud, msg);
        msg.header.stamp = clock->now();
        msg.header.frame_id = frame_id;
        msg.is_dense = false;
        pc_publisher_->publish(msg);

        std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
        RCLCPP_INFO(get_logger(), "Time difference = %d [ms]", std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count());
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<HALHeliosNode>();
    rclcpp::shutdown();
    return 0;
}
