#include "Heartbeater.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "VmbCPP/VmbCPP.h"

#include <iostream>
#include <sstream>

class AlliedVisionControl : public rclcpp::Node
{
  public:
    struct AlliedVisionControlParams
    {
        std::string camera_id_top; // Camera can only be identified from ID. The API does not support searching by Serial Number.
        bool use_top; // The camera layout is 1 top and 1 bottom.
        bool rotate_top;
        float camera_fps;
        std::string top_intrinsics;
        std::string img_save_path;
        float save_images_fps;
        bool save_top;
    };

    AlliedVisionControl()
        : Node("alliedvision_control"),
          heartbeater(this),
          vmb_system(VmbCPP::VmbSystem::GetInstance())
    {
        rclcpp::QoS qos(2);
        qos.reliable();
        qos.durability_volatile();

        // Set parameters
        this->params.camera_id_top = declare_parameter<std::string>("camera_id_top", "DEV_1AB22C020237");
        this->params.use_top = declare_parameter<bool>("use_top_camera", true);
        this->params.camera_fps = declare_parameter<float>("camera_fps", 2);
        this->params.top_intrinsics =
            declare_parameter<std::string>("top_intrinsics", "/home/anyone/config/allied_top_camera_calib.yml");
        this->params.img_save_path = declare_parameter<std::string>("img_save_path", "/home/anyone/Pictures/hires_images");
        this->params.save_images_fps = declare_parameter<float>("save_images_fps", 0.1);
        this->params.save_top = declare_parameter<bool>("save_top_images", true);

        // Initialize topics and services

        // Initialize timer

        printAllCameraInfo();

        heartbeater.Start();
        RCLCPP_INFO(get_logger(), "The AlliedVision Control node has started...");
    }

    ~AlliedVisionControl()
    {
        this->vmb_system.Shutdown();
        RCLCPP_INFO(get_logger(), "The AlliedVision Control node has stoped.");
    }

  private:
    AlliedVisionControlParams params;
    Heartbeater heartbeater;
    VmbCPP::VmbSystem& vmb_system;
    VmbCPP::CameraPtr camera_ptr;
    VmbCPP::FramePtr frame_ptr;

    // Topics and services
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr top_img_publisher;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr top_cam_info_publisher;

    // Timers
    rclcpp::TimerBase::SharedPtr top_cam_timer;

    /** Print information for all Allied Vision cameras.
     * Copied from VmbCPP example code: ${VmbCPP_DIR}/api/examples/VmbCPP/ListCameras/ListCameras.cpp
     */
    void printAllCameraInfo()
    {
        VmbVersionInfo_t versionInfo;
        this->vmb_system.QueryVersion(versionInfo);
        RCLCPP_INFO_STREAM(get_logger(), "Vmb Version Major: " << versionInfo.major << " Minor: " << versionInfo.minor
                                                               << " Patch: " << versionInfo.patch << std::endl);

        VmbErrorType vmb_err = this->vmb_system.Startup(); // Initialize the Vmb API
        if(vmb_err == VmbErrorType::VmbErrorSuccess)
        {
            VmbCPP::TransportLayerPtrVector transport_layers; // A vector of std::shared_ptr<AVT::VmbAPI::TransportLayer> objects
            vmb_err = this->vmb_system.GetTransportLayers(transport_layers); // Fetch all transport layers
            if(vmb_err == VmbErrorType::VmbErrorSuccess)
            {
                RCLCPP_INFO_STREAM(get_logger(), "TransportLayers found: " << transport_layers.size());
            }

            VmbCPP::InterfacePtrVector interfaces; // A vector of std::shared_ptr<AVT::VmbAPI::Interface> objects
            vmb_err = this->vmb_system.GetInterfaces(interfaces); // Fetch all interfaces
            if(vmb_err == VmbErrorType::VmbErrorSuccess)
            {
                RCLCPP_INFO_STREAM(get_logger(), "Interfaces found: " << interfaces.size());
            }

            VmbCPP::CameraPtrVector cameras; // A vector of std::shared_ptr<AVT::VmbAPI::Camera> objects
            vmb_err = vmb_system.GetCameras(cameras); // Fetch all cameras
            if(vmb_err == VmbErrorType::VmbErrorSuccess)
            {
                RCLCPP_INFO_STREAM(get_logger(), "Cameras found: " << cameras.size());

                // Query all static details of all known cameras and print them out.
                // We don't have to open the cameras for that.
                std::for_each(cameras.begin(), cameras.end(),
                              [&](const VmbCPP::CameraPtr& cam) { RCLCPP_INFO(get_logger(), printCameraInfo(cam).c_str()); });
            }
        }
        else { RCLCPP_ERROR_STREAM(get_logger(), "Failed to start camera. Error: " << vmb_err); }
    }

    /** Print information for one Allied Vision camera.
     * Copied from VmbCPP example code: ${VmbCPP_DIR}/api/examples/VmbCPP/ListCameras/ListCameras.cpp
     */
    std::string printCameraInfo(const VmbCPP::CameraPtr& camera)
    {
        std::string id;
        std::string name;
        std::string model;
        std::string serial;
        std::string interface_id;
        VmbCPP::TransportLayerPtr transport_layer;
        std::string transport_layer_id;
        std::string transport_layer_path;

        std::ostringstream err_ss;

        VmbErrorType vmb_err = camera->GetID(id);
        if(vmb_err != VmbErrorType::VmbErrorSuccess)
        {
            err_ss << "[Could not get camera ID. Error code: " << vmb_err << "]";
            id = err_ss.str();
        }

        vmb_err = camera->GetName(name);
        if(vmb_err != VmbErrorType::VmbErrorSuccess)
        {
            err_ss << "[Could not get camera name. Error code: " << vmb_err << "]";
            name = err_ss.str();
        }

        vmb_err = camera->GetModel(model);
        if(vmb_err != VmbErrorType::VmbErrorSuccess)
        {
            err_ss << "[Could not get camera mode name. Error code: " << vmb_err << "]";
            model = err_ss.str();
        }

        vmb_err = camera->GetSerialNumber(serial);
        if(vmb_err != VmbErrorType::VmbErrorSuccess)
        {
            err_ss << "[Could not get camera serial number. Error code: " << vmb_err << "]";
            serial = err_ss.str();
        }

        vmb_err = camera->GetInterfaceID(interface_id);
        if(vmb_err != VmbErrorType::VmbErrorSuccess)
        {
            err_ss << "[Could not get interface ID. Error code: " << vmb_err << "]";
            interface_id = err_ss.str();
        }

        vmb_err = camera->GetTransportLayer(transport_layer);
        vmb_err = transport_layer->GetID(transport_layer_id);
        if(vmb_err != VmbErrorType::VmbErrorSuccess)
        {
            err_ss << "[Could not get transport layer ID. Error code: " << vmb_err << "]";
            transport_layer_id = err_ss.str();
        }

        vmb_err = transport_layer->GetPath(transport_layer_path);
        if(vmb_err != VmbErrorType::VmbErrorSuccess)
        {
            err_ss << "[Could not get transport layer path. Error code: " << vmb_err << "]";
            transport_layer_path = err_ss.str();
        }

        std::ostringstream ss;
        ss << std::endl
           << "    Camera Name          : " << name << std::endl
           << "    Model Name           : " << model << std::endl
           << "    Camera ID            : " << id << std::endl
           << "    Serial Number        : " << serial << std::endl
           << "    @ Interface ID       : " << interface_id << std::endl
           << "    @ TransportLayer ID  : " << transport_layer_id << std::endl
           << "    @ TransportLayer Path: " << transport_layer_path << std::endl;
        return ss.str();
    }
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AlliedVisionControl>());
    rclcpp::shutdown();
    return 0;
}
