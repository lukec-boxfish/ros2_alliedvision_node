#include "Heartbeater.hpp"
#include "opencv2/core/core.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/distortion_models.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "VmbCPP/VmbCPP.h"

#include <chrono>
#include <iostream>
#include <sstream>

using namespace std::chrono_literals;

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
        // Quality of Service
        rclcpp::QoS qos(2);
        qos.reliable();
        qos.durability_volatile();

        // Image data does not keep samples in the queue.
        rclcpp::QoS img_qos(0);
        img_qos.history(RMW_QOS_POLICY_HISTORY_KEEP_LAST);
        img_qos.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
        img_qos.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);
        img_qos.deadline(rclcpp::Duration(2, 0));
        img_qos.lifespan(rclcpp::Duration(2, 0));
        img_qos.liveliness(RMW_QOS_POLICY_LIVELINESS_AUTOMATIC);
        img_qos.liveliness_lease_duration(rclcpp::Duration(6, 0));

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
        this->top_img_publisher = create_publisher<sensor_msgs::msg::Image>("raw_top_allied_img", img_qos);
        this->top_cam_info_publisher = create_publisher<sensor_msgs::msg::CameraInfo>("raw_top_camera_info", img_qos);

        // Initialize timer
        this->top_cam_timer = create_wall_timer(5ms, std::bind(&AlliedVisionControl::topImgPublisher, this));

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

    void topImgPublisher()
    {
    }

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

    /** Create CameraInfo object from an OpenCV calibration file.
     * Calibration parameters: https://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/CameraInfo.html
     */
    void copyIntrinsics(const cv::FileStorage& fs, sensor_msgs::msg::CameraInfo& info)
    {
        cv::Mat camera_matrix, distortion_coefficients;
        double image_width, image_height;

        fs["image_width"] >> image_width;
        fs["image_height"] >> image_height;
        fs["camera_matrix"] >> camera_matrix;
        fs["distortion_coefficients"] >> distortion_coefficients;

        info.height = image_height;
        info.width = image_width;
        info.distortion_model = sensor_msgs::distortion_models::RATIONAL_POLYNOMIAL;

        // Distortion parameters
        const int distortion_coef_size = distortion_coefficients.size().height * distortion_coefficients.size().width;
        // Rational polynomial support 8 parameters: [k1, k2, p1, p2, k3, k4, k5, k6]
        // https://www.ros.org/reps/rep-0104.html#alternate-distortion-models
        for(int i = 0; (i < distortion_coef_size && i < 8); i++)
        {
            info.d.push_back(distortion_coefficients.at<double>(0, i));
        }

        // Intrinsic camera matrix
        for(int row = 0; row < camera_matrix.size().height; row++)
        {
            for(int col = 0; col < camera_matrix.size().width; col++)
            {
                info.k[(camera_matrix.size().width * row) + col] = camera_matrix.at<double>(row, col);
            }
        }

        // Rectification matrix (stereo cameras only)
        info.r[0] = 1.0;
        info.r[4] = 1.0;
        info.r[8] = 1.0;

        // Projection/camera matrix
        // Assume 0 translation between cameras. This is definitely wrong, use a stereo calib file to override.
        info.p[0] = info.k[0];
        info.p[1] = info.k[1];
        info.p[2] = info.k[2];
        info.p[3] = 0.0;
        info.p[4] = info.k[3];
        info.p[5] = info.k[4];
        info.p[6] = info.k[5];
        info.p[7] = 0.0;
        info.p[8] = info.k[6];
        info.p[9] = info.k[7];
        info.p[10] = info.k[8];
        info.p[11] = 0.0;
    }

    /** Update CameraInfo object from a stereo calibration file.
     */
    void copyStereoExtrinsics(const cv::FileStorage& fs, sensor_msgs::msg::CameraInfo& info, std::string cam_location)
    {
    }
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AlliedVisionControl>());
    rclcpp::shutdown();
    return 0;
}
