#include "cv_bridge/cv_bridge.h"
#include "Heartbeater.hpp"
#include "opencv2/core/core.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/distortion_models.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "VmbCPP/VmbCPP.h"

#include <chrono>
#include <filesystem>
#include <iostream>
#include <sstream>

using namespace std::chrono_literals;

const std::string FRAME_ID_TOP = "front_top_cam_link";
const std::string FRAME_ID_BOTTOM = "front_bottom_cam_link";

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
        std::string top_setting_path;
        bool save_setting;
    };

    AlliedVisionControl()
        : Node("alliedvision_control"),
          heartbeater(this),
          vmb_system(VmbCPP::VmbSystem::GetInstance()),
          pair_timestamp(this->now()),
          is_top_valid(false)
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
        this->params.camera_fps = declare_parameter<float>("camera_fps", 2.0);
        this->params.top_intrinsics =
            declare_parameter<std::string>("top_intrinsics", "/home/anyone/config/allied_top_camera_calib.yml");
        this->params.img_save_path = declare_parameter<std::string>("img_save_path", "/home/anyone/Pictures/hires_images");
        this->params.save_images_fps = declare_parameter<float>("save_images_fps", 0.1);
        this->params.save_top = declare_parameter<bool>("save_top_images", true);
        this->params.top_setting_path =
            declare_parameter<std::string>("top_setting_path", "/home/anyone/config/allied_top_settings.xml");
        this->params.save_setting = declare_parameter<bool>("save_setting", false);

        startCameras();

        // Load camera calibration file
        this->is_top_valid = this->loadCalibrationFile(true);

        // Prepare cameras
        this->prepareCamera();

        // Initialize topics and services
        this->top_img_publisher = create_publisher<sensor_msgs::msg::Image>("/front/top/image", img_qos);
        this->top_cam_info_publisher = create_publisher<sensor_msgs::msg::CameraInfo>("/front/top/camera_info", img_qos);

        // Initialize timer
        this->top_cam_timer = create_wall_timer(1s, std::bind(&AlliedVisionControl::topImgPublisher, this));

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
    VmbCPP::CameraPtr camera_top;
    VmbCPP::FramePtr frame_top;
    rclcpp::Time pair_timestamp; // Image and CameraInfo must have exact matching timestamp.
    sensor_msgs::msg::CameraInfo top_cam_info;
    bool is_top_valid;
    VmbErrorType top_err; // Hold error message from VmbCpp for the top camera.

    // Topics and services
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr top_img_publisher;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr top_cam_info_publisher;

    // Timers
    rclcpp::TimerBase::SharedPtr top_cam_timer;

    void topImgPublisher()
    {
        this->captureImage();
    }

    /** Start the system and print information for all Allied Vision cameras.
     * Copied from VmbCPP example code: ${VmbCPP_DIR}/api/examples/VmbCPP/ListCameras/ListCameras.cpp
     */
    void startCameras()
    {
        VmbVersionInfo_t versionInfo;
        this->vmb_system.QueryVersion(versionInfo);
        RCLCPP_INFO_STREAM(get_logger(), "Vmb Version Major: " << versionInfo.major << " Minor: " << versionInfo.minor
                                                               << " Patch: " << versionInfo.patch << std::endl);

        VmbErrorType vmb_err = this->vmb_system.Startup(); // Initialize the Vmb API
        if(vmb_err == VmbErrorSuccess)
        {
            VmbCPP::TransportLayerPtrVector transport_layers; // A vector of std::shared_ptr<AVT::VmbAPI::TransportLayer> objects
            vmb_err = this->vmb_system.GetTransportLayers(transport_layers); // Fetch all transport layers
            if(vmb_err == VmbErrorSuccess)
            {
                RCLCPP_INFO_STREAM(get_logger(), "TransportLayers found: " << transport_layers.size());
            }

            VmbCPP::InterfacePtrVector interfaces; // A vector of std::shared_ptr<AVT::VmbAPI::Interface> objects
            vmb_err = this->vmb_system.GetInterfaces(interfaces); // Fetch all interfaces
            if(vmb_err == VmbErrorSuccess) { RCLCPP_INFO_STREAM(get_logger(), "Interfaces found: " << interfaces.size()); }

            VmbCPP::CameraPtrVector cameras; // A vector of std::shared_ptr<AVT::VmbAPI::Camera> objects
            vmb_err = vmb_system.GetCameras(cameras); // Fetch all cameras
            if(vmb_err == VmbErrorSuccess)
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
        if(vmb_err != VmbErrorSuccess)
        {
            err_ss << "[Could not get camera ID. Error code: " << vmb_err << "]";
            id = err_ss.str();
        }

        vmb_err = camera->GetName(name);
        if(vmb_err != VmbErrorSuccess)
        {
            err_ss << "[Could not get camera name. Error code: " << vmb_err << "]";
            name = err_ss.str();
        }

        vmb_err = camera->GetModel(model);
        if(vmb_err != VmbErrorSuccess)
        {
            err_ss << "[Could not get camera mode name. Error code: " << vmb_err << "]";
            model = err_ss.str();
        }

        vmb_err = camera->GetSerialNumber(serial);
        if(vmb_err != VmbErrorSuccess)
        {
            err_ss << "[Could not get camera serial number. Error code: " << vmb_err << "]";
            serial = err_ss.str();
        }

        vmb_err = camera->GetInterfaceID(interface_id);
        if(vmb_err != VmbErrorSuccess)
        {
            err_ss << "[Could not get interface ID. Error code: " << vmb_err << "]";
            interface_id = err_ss.str();
        }

        vmb_err = camera->GetTransportLayer(transport_layer);
        vmb_err = transport_layer->GetID(transport_layer_id);
        if(vmb_err != VmbErrorSuccess)
        {
            err_ss << "[Could not get transport layer ID. Error code: " << vmb_err << "]";
            transport_layer_id = err_ss.str();
        }

        vmb_err = transport_layer->GetPath(transport_layer_path);
        if(vmb_err != VmbErrorSuccess)
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
        // TODO: Not support stereo ATM.
        (void)fs;
        (void)info;
        (void)cam_location;
    }

    /** Load the calibration file for one camera.
     */
    bool loadCalibrationFile(const bool is_top)
    {
        std::string cam_position = "top";
        std::string file_path = this->params.top_intrinsics;
        bool is_valid = false;
        sensor_msgs::msg::CameraInfo& cam_info = this->top_cam_info; // Create a reference
        cam_info.header.frame_id = FRAME_ID_TOP;

        if(!is_top) // the bottom camera
        {
            cam_position = "bottom";
            // TODO: Not implemented yet.
        }

        cv::FileStorage fs_top(this->params.top_intrinsics.c_str(), cv::FileStorage::READ);
        if(!fs_top.isOpened())
        {
            RCLCPP_ERROR(get_logger(), "Failed to open the %s camera calibration file. The %s camera is unavailable.",
                         cam_position.c_str(), cam_position.c_str());
        }
        else
        {
            this->copyIntrinsics(fs_top, cam_info);
            is_valid = true;
            RCLCPP_INFO(get_logger(), "Successfully load the %s camera calibration file.", cam_position.c_str());
        }
        return is_valid;
    }

    /** Prepare a single camera.
     */
    void prepareCamera()
    {
        // Prepare top camera
        if(this->params.use_top && this->is_top_valid)
        {
            RCLCPP_INFO(get_logger(), "Starting the top camera...");
            this->top_err = this->vmb_system.GetCameraByID(this->params.camera_id_top.c_str(), this->camera_top);

            if(this->top_err != VmbErrorSuccess)
            {
                RCLCPP_ERROR_STREAM(get_logger(), "Could not start the top camera. ID: " << this->params.camera_id_top
                                                                                         << ". Error: " << this->top_err);
            }

            this->top_err = this->camera_top->Open(VmbAccessModeType::VmbAccessModeFull);
            if(this->top_err != VmbErrorSuccess)
            {
                RCLCPP_ERROR_STREAM(get_logger(), "Could not open the top camera. Error: " << this->top_err);
            }

            // TODO: Should have 2 separate setting files
            const VmbFilePathChar_t* cam_setting_path = VMB_FILE_PATH_LITERAL(this->params.top_setting_path.c_str());
            RCLCPP_INFO_STREAM(get_logger(), "Camera settings path: " << cam_setting_path);
            VmbFeaturePersistSettings_t settings;
            settings.persistType = VmbFeaturePersistNoLUT;
            settings.modulePersistFlags = VmbModulePersistFlagsAll;
            settings.maxIterations = 5;
            settings.loggingLevel = 1;
            if(this->params.save_setting)
            {
                this->top_err = this->camera_top->SaveSettings(cam_setting_path, &settings);
                if(this->top_err != VmbErrorSuccess)
                {
                    RCLCPP_ERROR_STREAM(get_logger(),
                                        "Could not save camera settings to: " << cam_setting_path << " Error: " << this->top_err);
                }
                else { RCLCPP_INFO_STREAM(get_logger(), "Saved camera settings to: " << cam_setting_path); }
            }
            else // load settings
            {
                this->top_err = this->camera_top->LoadSettings(cam_setting_path, &settings);
                if(this->top_err != VmbErrorSuccess)
                {
                    RCLCPP_ERROR_STREAM(get_logger(), "Could not load camera settings from: " << cam_setting_path
                                                                                              << " Error: " << this->top_err);
                }
                else { RCLCPP_INFO(get_logger(), "Successfully load camera settings."); }
            }
        }

        // TODO: Prepare bottom camera
    }

    void captureImage()
    {
        this->pair_timestamp = this->now();
        this->top_err = this->camera_top->AcquireSingleImage(this->frame_top, 5000);
        if(this->top_err != VmbErrorSuccess)
        {
            RCLCPP_ERROR_STREAM(get_logger(), "Could not capture photo from the top camera. Error: " << this->top_err);
        }
        else
        {
            VmbUint32_t buffer_size;
            this->top_err = this->frame_top->GetBufferSize(buffer_size);
            if(this->top_err != VmbErrorSuccess)
            {
                RCLCPP_ERROR_STREAM(get_logger(), "Could not get image buffer size. Error: " << this->top_err);
            }
            // RCLCPP_INFO_STREAM(get_logger(), "Buffer size: " << buffer_size); // 37106688 = 4112 * 3008 * 3

            VmbUchar_t* buffer_ptr = nullptr;
            this->top_err = this->frame_top->GetImage(buffer_ptr);
            if(this->top_err != VmbErrorSuccess)
            {
                RCLCPP_ERROR_STREAM(get_logger(), "Could not get image from the top camera. Error: " << this->top_err);
            }
            // RCLCPP_INFO(get_logger(), "Image is captured.");

            // Publish image
            VmbUint32_t height, width;
            this->frame_top->GetHeight(height);
            this->frame_top->GetWidth(width);
            cv::Mat raw = cv::Mat(height, width, CV_8UC3, (uint8_t*)buffer_ptr);

            sensor_msgs::msg::Image img_msg;
            cv_bridge::CvImage(std_msgs::msg::Header(), "rgb8", raw).toImageMsg(img_msg);
            img_msg.header.frame_id = FRAME_ID_TOP;
            img_msg.header.stamp = this->pair_timestamp;

            this->top_img_publisher->publish(img_msg);

            // Publish camera info
            this->top_cam_info.header.stamp = this->pair_timestamp;
            this->top_cam_info_publisher->publish(this->top_cam_info);
        }
    }
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AlliedVisionControl>());
    rclcpp::shutdown();
    return 0;
}
