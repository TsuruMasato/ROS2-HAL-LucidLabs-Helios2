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
#include "rclcpp/duration.hpp"
#include "rclcpp/create_timer.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include <pcl_conversions/pcl_conversions.h>

#include "ArenaApi.h"

#include "HALHelios/lucidlabs_helios2.hpp"

#define SYSTEM_TIMEOUT 2000
#define IMAGE_TIMEOUT 2000

using std::placeholders::_1;

namespace hal {

LucidlabsHelios2::LucidlabsHelios2(const rclcpp::NodeOptions & options) : Node("hal_lucidlabs_helios2", options) {
    output_topic = declare_parameter<std::string>("output_topic", "/point_cloud");
    frame_id = declare_parameter<std::string>("frame_id", "camera");
    exposure = declare_parameter<int>("exposure_level", 0);
    hdr_mode = declare_parameter<int>("hdr_mode", 0);
    mode = declare_parameter<int>("mode", 0);
    accum = declare_parameter<int>("accumulate_frames", 1);
    confidence_filter = declare_parameter<bool>("confidence_filter.enable", true);
    confidence_filter_threshold = declare_parameter<int>("confidence_filter.threshold", 0);
    spatial_filter = declare_parameter<bool>("spatial_filter.enable", true);
    flying_filter = declare_parameter<bool>("flying_filter.enable", true);
    flying_filter_threshold = declare_parameter<int>("flying_filter.threshold", 0);
    structured_cloud = declare_parameter<bool>("structured_cloud", true);
    publish_intensity = declare_parameter<bool>("publish_intensity", true);

    pDevice = findDevice();

    try {
        pNodeMap = pDevice->GetNodeMap();
        pStreamNodeMap = pDevice->GetTLStreamNodeMap();

        /************************/

        GenApi::CEnumerationPtr pModeSel = pNodeMap->GetNode("Scan3dModeSelector");
        GenApi::CEnumEntryPtr pModeProcessed = pModeSel->GetEntryByName("Processed");
        pModeSel->SetIntValue(pModeProcessed->GetValue());

        /************************/

        Arena::SetNodeValue<bool>(pDevice->GetNodeMap(), "ChunkModeActive", false);

        /************************/

        GenApi::CEnumerationPtr pStreamBufferHandlingMode = pStreamNodeMap->GetNode("StreamBufferHandlingMode");
        GenApi::CEnumEntryPtr pNewestOnly = pStreamBufferHandlingMode->GetEntryByName("NewestOnly");
        pStreamBufferHandlingMode->SetIntValue(pNewestOnly->GetValue());

        /************************/

        GenApi::CEnumerationPtr pPixelMode = pNodeMap->GetNode("PixelFormat");        
        GenApi::CEnumEntryPtr pFormat = pPixelMode->GetEntryByName(
            publish_intensity ? "Coord3D_ABCY16" : "Coord3D_ABC16");
        pPixelMode->SetIntValue(pFormat->GetValue());

        /************************/

        Arena::SetNodeValue<bool>(pDevice->GetTLStreamNodeMap(), "StreamAutoNegotiatePacketSize", true);
        Arena::SetNodeValue<bool>(pDevice->GetTLStreamNodeMap(), "StreamPacketResendEnable", true);

        /************************/

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

        /************************/

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

        if (m == cam_model::HELIOS_2 && mode <= 2) {
            RCLCPP_ERROR(get_logger(), "Helios 2 does not support Scan3dOperatingMode=%s", modes[mode].c_str());
            std::abort();
        }

        GenApi::CEnumerationPtr pOperatingMode = pNodeMap->GetNode("Scan3dOperatingMode");
        GenApi::CEnumEntryPtr pMode = pOperatingMode->GetEntryByName(modes[mode].c_str());
        pOperatingMode->SetIntValue(pMode->GetValue());
        RCLCPP_INFO(get_logger(), "Mode setting: %s", modes[mode].c_str());

        /************************/

        GenApi::CBooleanPtr pConfidenceFilter = pNodeMap->GetNode("Scan3dConfidenceThresholdEnable");        
        pConfidenceFilter->SetValue(confidence_filter);
        GenApi::CIntegerPtr pConfThresh = pNodeMap->GetNode("Scan3dConfidenceThresholdMin");
        pConfThresh->SetValue(confidence_filter_threshold);

        /************************/

        GenApi::CBooleanPtr pSpatialFilter = pNodeMap->GetNode("Scan3dSpatialFilterEnable");
        pSpatialFilter->SetValue(spatial_filter);

        /************************/

        GenApi::CBooleanPtr pFlyingFilter = pNodeMap->GetNode("Scan3dFlyingPixelsRemovalEnable");
        pFlyingFilter->SetValue(flying_filter);
        // Maybe not available on Helios2
        GenApi::CIntegerPtr pFlyingThresh = pNodeMap->GetNode("Scan3dFlyingPixelsDistanceThreshold");
        pFlyingThresh->SetValue(flying_filter_threshold);

        /************************/

        static std::vector<std::string> hdr_modes = {
            "LowNoiseHDRX8",
            "LowNoiseHDRX4",
            "StandardHDR",
            "Off"
        };

        if (m == cam_model::HELIOS_2 && hdr_mode != 3) {
            RCLCPP_ERROR(get_logger(), "Helios 2 does not support Scan3dHDRMode");
            std::abort();
        } else if (m == cam_model::HELIOS_2_PLUS) {
            GenApi::CEnumerationPtr pScan3dHDRMode = pNodeMap->GetNode("Scan3dHDRMode");
            GenApi::CEnumEntryPtr pHDRMode = pScan3dHDRMode->GetEntryByName(hdr_modes[hdr_mode].c_str());
            pScan3dHDRMode->SetIntValue(pHDRMode->GetValue());
            RCLCPP_INFO(get_logger(), "HDR Mode setting: %s", hdr_modes[hdr_mode].c_str());
        } 

        if (hdr_mode == 3) {
            Arena::SetNodeValue<int64_t>(pDevice->GetNodeMap(), "Scan3dImageAccumulation", accum);
        }

        /************************/

        GenApi::CEnumerationPtr pScan3DCoordinateSelector = pNodeMap->GetNode("Scan3dCoordinateSelector");
        GenApi::CEnumEntryPtr pCoordinateA = pScan3DCoordinateSelector->GetEntryByName("CoordinateA");
        GenApi::CEnumEntryPtr pCoordinateB = pScan3DCoordinateSelector->GetEntryByName("CoordinateB");
        GenApi::CEnumEntryPtr pCoordinateC = pScan3DCoordinateSelector->GetEntryByName("CoordinateC");
        GenApi::CFloatPtr pScan3DCoordinateOffset = pNodeMap->GetNode("Scan3dCoordinateOffset");
        GenApi::CFloatPtr pScan3DCoordinateScale = pNodeMap->GetNode("Scan3dCoordinateScale");

        pScan3DCoordinateSelector->SetIntValue(pCoordinateA->GetValue());
        offX = pScan3DCoordinateOffset->GetValue();
        scaleX = pScan3DCoordinateScale->GetValue();
        pScan3DCoordinateSelector->SetIntValue(pCoordinateB->GetValue());
        offY = pScan3DCoordinateOffset->GetValue();
        scaleY = pScan3DCoordinateScale->GetValue();
        pScan3DCoordinateSelector->SetIntValue(pCoordinateC->GetValue());
        offZ = pScan3DCoordinateOffset->GetValue();
        scaleZ = pScan3DCoordinateScale->GetValue();

        RCLCPP_INFO(get_logger(), "Offset = %f | %f | %f", offX, offY, offZ);
        RCLCPP_INFO(get_logger(), "Scale  = %f | %f | %f", scaleX, scaleY, scaleZ);

        /************************/

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

    using namespace std::chrono_literals;
    pc_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(output_topic, 1);
    timer = create_wall_timer(10ms, std::bind(&LucidlabsHelios2::runtime, this));
    // TODO: compute time period w.r.t. mode/accumulation/HDR settings
}

Arena::IDevice* LucidlabsHelios2::findDevice() {

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
            if (deviceInfos[id].ModelName() == "HLT003S-001"){
                    m = cam_model::HELIOS_2;
                    found = true;
                    found_id = id;
                    break;
            } else if (deviceInfos[id].ModelName() == "HTP003S-001"){
                m = cam_model::HELIOS_2_PLUS;
                found = true;
                found_id = id;
                break;
            }
        }

        if (!found) {
            RCLCPP_ERROR(get_logger(), "No Helios 2 Camera found.");
            std::abort();
        }
        return pSystem->CreateDevice(deviceInfos[found_id]);
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
}

void LucidlabsHelios2::runtime() {
    if (publish_intensity)
        get_point_cloud<pcl::PointXYZI>();
    else        
        get_point_cloud<pcl::PointXYZ>();
}

LucidlabsHelios2::~LucidlabsHelios2() {
    if (pDevice) {
        pDevice->StopStream();
        pSystem->DestroyDevice(pDevice);
    }
    if (pSystem) {
        Arena::CloseSystem(pSystem);
    }
}

template <typename PointT>
void LucidlabsHelios2::get_point_cloud() {
    try {
        pImage = pDevice->GetImage(15);
    } catch (...) {
        return;
    }
    if (pImage == nullptr) return;
    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
    pcl::PointCloud<PointT> cloud;
    sensor_msgs::msg::PointCloud2 msg;
    cloud.reserve(640*480);

    if (pImage->IsIncomplete()) {
        RCLCPP_WARN(get_logger(), "Incomplete frame");
        pDevice->RequeueBuffer(pImage);
        return;
    }

    const uint16_t *data = (uint16_t*)(pImage->GetData());
    uint16_t A, B, C, Y;
    (void) Y;

    if (structured_cloud){
        cloud.resize(640*480);
        cloud.width = 640;
        cloud.height = 480;
    }

    PointT* p = nullptr;
    bool last_invalid = false;
    for (int o = 0; o < 640*480; o++) {
        if constexpr (std::is_same<PointT,pcl::PointXYZ>::value){
            A = data[o*3+0];
            B = data[o*3+1];
            C = data[o*3+2];
        } else {
            A = data[o*4+0];
            B = data[o*4+1];
            C = data[o*4+2];
            Y = data[o*4+3];
        }
        if (structured_cloud){
            p = &cloud[o];
        } else if (not last_invalid) {
            cloud.emplace_back();
            p = &cloud.back();
        }

        if (A != 0xFFFF && B != 0xFFFF && C != 0xFFFF) {
            assert(p != nullptr);
            p->x = (A*scaleX+offX)/1000;
            p->y = (B*scaleY+offY)/1000;
            p->z = (C*scaleZ+offZ)/1000;
            if constexpr (std::is_same<PointT,pcl::PointXYZI>::value)
                p->intensity = (float)Y;
            last_invalid = false;
        } else {
            assert(p != nullptr);
            p->x = 0;
            p->y = 0;
            p->z = 0;
            if constexpr (std::is_same<PointT,pcl::PointXYZI>::value)
                p->intensity = 0;
            last_invalid = true;
        }
    }
    if ((not structured_cloud) and last_invalid){
        cloud.resize(cloud.size()-1);
    }

    pDevice->RequeueBuffer(pImage);

    begin = std::chrono::steady_clock::now();
    pcl::toROSMsg(cloud, msg);
    msg.header.stamp = clock->now();
    msg.header.frame_id = frame_id;
    msg.is_dense = false;

    begin = std::chrono::steady_clock::now();
    pc_publisher_->publish(msg);
    std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
    RCLCPP_INFO(get_logger(), "Time difference = %d [ms]", std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count());
}

}  // namespace hal
