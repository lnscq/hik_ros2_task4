#include # include“hik_camera / hik_camera_node.hpp”# include“hik_camera / hik_camera_node.hpp”# include“hik_camera / hik_camera_node.hpp”"“hik_camera / hik_camera_node.hpp   高压泵”hik_camera/hik_camera_node.hpp   高压泵"

using   使用 使用命名空间std::chrono_literals；使用命名空间std::chrono_literals；使用命名空间std::chrono_literals；namespace   名称空间 std::chrono_literals;

HikCameraNode::（（const   常量 rclcpp::NodeOptions & options   选项）const   常量 rclcpp::NodeOptions & options   选项）HikCameraNode（const   常量 rclcpp::NodeOptions & options   选项）(const   常量 rclcpp::NodeOptions & options)
: ： Node   节点： Node： Node   节点("hik_camera_node   “hik_camera_node”", options   选项)，("hik_camera_node   “hik_camera_node”", options   选项)，("hik_camera_node   “hik_camera_node”", options   选项)，Node("hik_camera_node   “hik_camera_node”"   “hik_camera_node”, options   选项),
  camera_handle_(nullptr),
        is_grabbing_(假)is_grabbing_(假)is_grabbing_   is_grabbing_(假)(false   假   假)
{
    // 声明参数
    ——> ——> ——> declare_parameter(“camera_name”、“hik_camera”);declare_parameter(“camera_name”、“hik_camera”);declare_parameter(“camera_name”、“hik_camera”);this   这->declare_parameter("camera_name"   “camera_name”, "   “hik_camera”hik_camera");
    ——> ——> ——> declare_parameter(“camera_ip”、“192.168.1.10”);declare_parameter(“camera_ip”、“192.168.1.10”);declare_parameter(“camera_ip”、“192.168.1.10”);this   这->declare_parameter("camera_ip"   “camera_ip”, "192.168.1.10");
    ——> ——> ——> declare_parameter(“camera_serial”、" ");declare_parameter(“camera_serial”、" ");declare_parameter(“camera_serial”、" ");this   这->declare_parameter("camera_serial"   “camera_serial”, "");
    ——> declare_parameter(“frame_rate”,30.0);this->declare_parameter("   “frame_rate”frame_rate", 30.0);
    ——> ——> ——> declare_parameter(“exposure_time”,10000.0);declare_parameter(“exposure_time”,10000.0);declare_parameter(“exposure_time”,10000.0);this   这->declare_parameter("exposure_time"   “exposure_time”, 10000.0);
    ——> declare_parameter(“获得”,5.0);this->declare_parameter("   “获得”gain", 5.0);
    ——> ——> ——> declare_parameter(“pixel_format”、“BGR8”);declare_parameter(“pixel_format”、“BGR8”);declare_parameter(“pixel_format”、“BGR8”);this   这->declare_parameter("pixel_format"   “pixel_format”, "BGR8");
    ——> declare_parameter(“camera_info_url”、" ");this->declare_parameter("   “camera_info_url”camera_info_url", "");
    
    // 获取参数
    ——> ——> ——> get_parameter(“camera_name”,camera_name_);get_parameter(“camera_name”,camera_name_);get_parameter(“camera_name”,camera_name_);this   这->get_parameter("camera_name"   “camera_name”, camera_name_);
    ——> get_parameter(“camera_ip”,camera_ip_);this->get_parameter("   “camera_ip”camera_ip", camera_ip_);
    ——> ——> ——> get_parameter(“camera_serial”,camera_serial_);get_parameter(“camera_serial”,camera_serial_);get_parameter(“camera_serial”,camera_serial_);this   这->get_parameter("camera_serial"   “camera_serial”, camera_serial_);
    ——> get_parameter(“frame_rate”,frame_rate_);this->get_parameter("   “frame_rate”frame_rate", frame_rate_);
    ——> ——> ——> get_parameter(“exposure_time”,exposure_time_);get_parameter(“exposure_time”,exposure_time_);get_parameter(“exposure_time”,exposure_time_);this   这->get_parameter("exposure_time"   “exposure_time”, exposure_time_);
    ——> get_parameter(“获得”,gain_);this->get_parameter("   “获得”gain", gain_);
    ————> get_parameter(“pixel_format”,pixel_format_);> get_parameter(“pixel_format”,pixel_format_);this   这->get_parameter("pixel_format"   “pixel_format”, pixel_format_);
    ——> get_parameter(“camera_info_url”,camera_info_url_);this->get_parameter("   “camera_info_url”camera_info_url", camera_info_url_);
    
    RCLCPP_INFORCLCPP_INFORCLCPP_INFO(这- > get_logger(),“camera_ip: % s camera_ip_.c_str ());(这- > get_logger(),“camera_ip: % s camera_ip_.c_str ());(this   这->get_logger(), "camera_ip: %s"   “camera_ip: % s”, camera_ip_.c_str());
    RCLCPP_INFORCLCPP_INFORCLCPP_INFO(this   这->get_logger(), "camera_serial: %s"   “camera_serial: % s”, camera_serial_c_str ())；(this   这->get_logger(), "camera_serial: %s"   “camera_serial: % s”, camera_serial_c_str ())；(this   这   这->get_logger(), "camera_serial: %s"   “camera_serial: % s”, camera_serial_.c_str());
    RCLCPP_INFORCLCPP_INFORCLCPP_INFO(this   这->get_logger()，“exposure_time: %f”，exposure_time)；(this   这->get_logger()，“exposure_time: %f”，exposure_time)；(this   这->get_logger(), "exposure_time: %f"   “exposure_time: % f”, exposure_time_);
    RCLCPP_INFORCLCPP_INFO(这- > get_logger(),“获得:% f”,gain_);(this->get_logger(), "   “获得:% f”gain: %f", gain_);
    RCLCPP_INFORCLCPP_INFO（RCLCPP_INFO（this   这->get_logger()，“帧率：%f”，帧率）；this   这->get_logger()，“帧率：%f”，帧率）；(this   这->get_logger(), "frame_rate: %f"   “frame_rate: % f”, frame_rate_);

    initialize_timer_ = Initialize_timer_ = Initialize_timer_ = Initialize_timer_ = this   这->this   这->this   这->this   这->create_wall_timer(
        100ms, std::bind(&HikCameraNode::initialize, this));100ms, std::bind   绑定(&HikCameraNode::initialize, this   这))；
}

void HikCameraNode::initialize   初始化()空白HikCameraNode:初始化()
{
    initialize_timer_->cancel();initialize_timer_ - >取消();

    // 初始化图像传输
    image_transport_ = std::make_shared<image_transport::ImageTransport>(shared_from_this());image_transport = std::make_shared(shared_from_this())；

    // 初始化camera_info_manager
    camera_info_manager_ = std::make_shared<camera_info_manager::CameraInfoManager>(this, camera_name_, camera_info_url_);camera_info_manager_ = std::make_shared(this   这, camera_name_, camera_info_url_)；
    
    // 创建相机发布器
    camera_pub_ = image_transport_->advertiseCamera("image_raw", 10);camera_pub_ = image_transport_->advertiseCamera("image_raw"   “image_raw”, 10)；
    
    // 设置参数回调
    param_callback_handle_ = this->add_on_set_parameters_callback(参数回调函数(
        std::bind(&HikCameraNode::parameters_callback, this, std::placeholders::_1));std::bind   绑定(&HikCameraNode::parameters_callback, this   这, std::placeholders::_1))；

    if (initialize_camera()) {   If (initialize_camera()) {
        RCLCPP_INFO(this->get_logger(), "Camera initialized successfully");RCLCPP_INFO（this   这->get_logger(), “相机初始化成功”）；
        start_grabbing();
    }
    else {   其他{
        RCLCPP_ERROR(this->get_logger(), "Failed to initialize camera");RCLCPP_ERROR（this   这->get_logger()，“初始化相机失败”）；
        rclcpp::shutdown();   rclcpp:关闭();
    }
}

HikCameraNode::~HikCameraNode()
{
    stop_grabbing();
    if (camera_handle_) {   If (camera_handle_) {
        MV_CC_DestroyHandle(camera_handle_);
        camera_handle_ = nullptr;   Camera_handle_ = nullptr；
    }
}


bool HikCameraNode::initialize_camera()bool   保龄球 HikCameraNode: initialize_camera ()
{
    int nRet = MV_OK;
    
    MV_CC_DEVICE_INFO_LIST stDeviceList;
    memset(&stDeviceList, 0, sizeof(MV_CC_DEVICE_INFO_LIST));memset(&stDeviceList, 0, sizeof(MV_CC_DEVICE_INFO_LIST))；
    
    // 枚举设备
    nRet = MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, &stDeviceList);nRet = MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, &stDeviceList)；
    if (MV_OK != nRet) {
        RCLCPP_ERROR(this->get_logger(), "Enum Devices fail! nRet [0x%x]", nRet);
        return false;
    }
    
    if (stDeviceList.nDeviceNum == 0) {
        RCLCPP_ERROR(this->get_logger(), "Find No Devices!");
        return false;
    }
    
    unsigned int nIndex = 0;
    bool found = false;
    
    for (unsigned int i = 0; i < stDeviceList.nDeviceNum; i++) {
        MV_CC_DEVICE_INFO* pDeviceInfo = stDeviceList.pDeviceInfo[i];
        
        if (pDeviceInfo->nTLayerType == MV_GIGE_DEVICE) {
            // GigE设备
            char strIp[64] = {0};
            sprintf(strIp, "%u.%u.%u.%u", 
                    pDeviceInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0xFF,
                    (pDeviceInfo->SpecialInfo.stGigEInfo.nCurrentIp >> 8) & 0xFF,
                    (pDeviceInfo->SpecialInfo.stGigEInfo.nCurrentIp >> 16) & 0xFF,
                    (pDeviceInfo->SpecialInfo.stGigEInfo.nCurrentIp >> 24) & 0xFF);
            
            RCLCPP_INFO(this->get_logger(), "Found camera IP: %s", strIp);
            
            if (camera_ip_ == strIp || (camera_serial_.empty() && i == 0)) {
                nIndex = i;
                found = true;
                RCLCPP_INFO(this->get_logger(), "Selected camera with IP: %s", strIp);
                break;
            }
        } else if (pDeviceInfo->nTLayerType == MV_USB_DEVICE) {
            // USB设备
            if (!camera_serial_.empty()) {
                std::string serial_num = reinterpret_cast<char*>(pDeviceInfo->SpecialInfo.stUsb3VInfo.chSerialNumber);
                if (camera_serial_ == serial_num) {
                    nIndex = i;
                    found = true;
                    RCLCPP_INFO(this->get_logger(), "Selected camera with serial: %s", serial_num.c_str());
                    break;
                }
            }
        }
    }
    
    if (!found && stDeviceList.nDeviceNum > 0) {
        RCLCPP_WARN(this->get_logger(), "Specific camera not found, using first available camera");
        nIndex = 0;
        found = true;
    }
    
    if (!found) {
        RCLCPP_ERROR(this->get_logger(), "Camera not found with IP: %s or serial: %s", 
                    camera_ip_.c_str(), camera_serial_.c_str());
        return false;
    }
    
    // 创建句柄
    nRet = MV_CC_CreateHandle(&camera_handle_, stDeviceList.pDeviceInfo[nIndex]);
    if (MV_OK != nRet) {
        RCLCPP_ERROR(this->get_logger(), "Create Handle fail! nRet [0x%x]", nRet);
        return false;
    }
    
    // 打开设备
    nRet = MV_CC_OpenDevice(camera_handle_);
    if (MV_OK != nRet) {
        RCLCPP_ERROR(this->get_logger(), "Open Device fail! nRet [0x%x]", nRet);
        MV_CC_DestroyHandle(camera_handle_);
        camera_handle_ = nullptr;
        return false;
    }
    
    // 设置相机参数
    return set_camera_parameters();
}

bool HikCameraNode::set_camera_parameters()
{
    int nRet = MV_OK;
    
    // 设置触发模式为连续采集
    nRet = MV_CC_SetEnumValue(camera_handle_, "TriggerMode", 0);
    if (MV_OK != nRet) {
        RCLCPP_ERROR(this->get_logger(), "Set TriggerMode fail! nRet [0x%x]", nRet);
        return false;
    }
    
    // 设置帧率
    nRet = MV_CC_SetFloatValue(camera_handle_, "AcquisitionFrameRate", frame_rate_);
    if (MV_OK != nRet) {
        RCLCPP_WARN(this->get_logger(), "Set FrameRate fail! nRet [0x%x]", nRet);
    }
    
    // 设置曝光时间
    nRet = MV_CC_SetFloatValue(camera_handle_, "ExposureTime", exposure_time_);
    if (MV_OK != nRet) {
        RCLCPP_WARN(this->get_logger(), "Set ExposureTime fail! nRet [0x%x]", nRet);
    }
    
    // 设置增益
    nRet = MV_CC_SetFloatValue(camera_handle_, "Gain", gain_);
    if (MV_OK != nRet) {
        RCLCPP_WARN(this->get_logger(), "Set Gain fail! nRet [0x%x]", nRet);
    }
    
    // 设置像素格式
    if (pixel_format_ == "BGR8") {
        nRet = MV_CC_SetEnumValue(camera_handle_, "PixelFormat", PixelType_Gvsp_BGR8_Packed);
    } else if (pixel_format_ == "RGB8") {
        nRet = MV_CC_SetEnumValue(camera_handle_, "PixelFormat", PixelType_Gvsp_RGB8_Packed);
    } else if (pixel_format_ == "Mono8") {
        nRet = MV_CC_SetEnumValue(camera_handle_, "PixelFormat", PixelType_Gvsp_Mono8);
    } else {
        RCLCPP_WARN(this->get_logger(), "Unsupported pixel format: %s, using BGR8", pixel_format_.c_str());
        nRet = MV_CC_SetEnumValue(camera_handle_, "PixelFormat", PixelType_Gvsp_BGR8_Packed);
    }
    
    if (MV_OK != nRet) {
        RCLCPP_WARN(this->get_logger(), "Set PixelFormat fail! nRet [0x%x]", nRet);
    }
    
    RCLCPP_INFO(this->get_logger(), "Camera parameters set: %.1f FPS, %.1f exposure, %.1f gain, %s format",
                frame_rate_, exposure_time_, gain_, pixel_format_.c_str());
    
    return true;
}

void HikCameraNode::start_grabbing()
{
    int nRet = MV_CC_StartGrabbing(camera_handle_);
    if (MV_OK != nRet) {
        RCLCPP_ERROR(this->get_logger(), "Start Grabbing fail! nRet [0x%x]", nRet);
        return;
    }
    
    is_grabbing_ = true;
    grab_thread_ = std::thread(&HikCameraNode::grab_loop, this);
    RCLCPP_INFO(this->get_logger(), "Start grabbing images");
}

void HikCameraNode::stop_grabbing()
{
    is_grabbing_ = false;
    if (grab_thread_.joinable()) {
        grab_thread_.join();
    }
    
    if (camera_handle_) {
        MV_CC_StopGrabbing(camera_handle_);
    }
    RCLCPP_INFO(this->get_logger(), "Stop grabbing images");
}

void HikCameraNode::grab_loop()
{
    MV_FRAME_OUT stImageInfo;
    memset(&stImageInfo, 0, sizeof(MV_FRAME_OUT));
    int nRet = MV_OK;
    
    while (rclcpp::ok() && is_grabbing_) {
        nRet = MV_CC_GetImageBuffer(camera_handle_, &stImageInfo, 1000);
        if (nRet == MV_OK) {
            // 转换图像数据
            auto msg = std::make_unique<sensor_msgs::msg::Image>();
            msg->header.stamp = this->now();
            msg->header.frame_id = camera_name_ + "_optical_frame";
            msg->height = stImageInfo.stFrameInfo.nHeight;
            msg->width = stImageInfo.stFrameInfo.nWidth;
            
            // 根据像素格式设置编码
            if (stImageInfo.stFrameInfo.enPixelType == PixelType_Gvsp_BGR8_Packed) {
                msg->encoding = "bgr8";
                msg->step = stImageInfo.stFrameInfo.nWidth * 3;
            } else if (stImageInfo.stFrameInfo.enPixelType == PixelType_Gvsp_RGB8_Packed) {
                msg->encoding = "rgb8";
                msg->step = stImageInfo.stFrameInfo.nWidth * 3;
            } else if (stImageInfo.stFrameInfo.enPixelType == PixelType_Gvsp_Mono8) {
                msg->encoding = "mono8";
                msg->step = stImageInfo.stFrameInfo.nWidth;
            } else {
                msg->encoding = "bgr8";
                msg->step = stImageInfo.stFrameInfo.nWidth * 3;
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, 
                                   "Unsupported pixel format: 0x%lx, using bgr8", 
                                   stImageInfo.stFrameInfo.enPixelType);
            }
            
            msg->is_bigendian = 0;
            
            // 复制图像数据
            size_t image_size = stImageInfo.stFrameInfo.nFrameLen;
            msg->data.resize(image_size);
            memcpy(msg->data.data(), stImageInfo.pBufAddr, image_size);
            
            // 发布相机信息和图像
            auto camera_info = camera_info_manager_->getCameraInfo();
            camera_info.header = msg->header;
            camera_pub_.publish(*msg, camera_info);
            
            // 释放图像缓冲区
            MV_CC_FreeImageBuffer(camera_handle_, &stImageInfo);
        } else if (nRet == static_cast<int>(MV_E_NODATA)) {
            continue;
        } else {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, 
                               "Get image buffer failed: 0x%x", nRet);
        }
    }
}

bool HikCameraNode::reconnect_camera()
{
    stop_grabbing();
    
    if (camera_handle_) {
        MV_CC_CloseDevice(camera_handle_);
        MV_CC_DestroyHandle(camera_handle_);
        camera_handle_ = nullptr;
    }
    
    // 等待并重试连接
    for (int i = 0; i < 5; ++i) {
        RCLCPP_INFO(this->get_logger(), "Attempting to reconnect camera (attempt %d/5)", i + 1);
        
        if (initialize_camera()) {
            start_grabbing();
            RCLCPP_INFO(this->get_logger(), "Camera reconnected successfully");
            return true;
        }
        
        std::this_thread::sleep_for(2s);
    }
    
    RCLCPP_ERROR(this->get_logger(), "Failed to reconnect camera");
    return false;
}

rcl_interfaces::msg::SetParametersResult HikCameraNode::parameters_callback(
    const std::vector<rclcpp::Parameter> & parameters)
{
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    
    for (const auto & param : parameters) {
        if (param.get_name() == "exposure_time") {
            exposure_time_ = param.as_double();
            int nRet = MV_CC_SetFloatValue(camera_handle_, "ExposureTime", exposure_time_);
            if (MV_OK == nRet) {
                RCLCPP_INFO(this->get_logger(), "Exposure time set to: %f", exposure_time_);
            } else {
                RCLCPP_WARN(this->get_logger(), "Failed to set exposure time: 0x%x", nRet);
            }
        } else if (param.get_name() == "gain") {
            gain_ = param.as_double();
            int nRet = MV_CC_SetFloatValue(camera_handle_, "Gain", gain_);
            if (MV_OK == nRet) {
                RCLCPP_INFO(this->get_logger(), "Gain set to: %f", gain_);
            } else {
                RCLCPP_WARN(this->get_logger(), "Failed to set gain: 0x%x", nRet);
            }
        } else if (param.get_name() == "frame_rate") {
            frame_rate_ = param.as_double();
            int nRet = MV_CC_SetFloatValue(camera_handle_, "AcquisitionFrameRate", frame_rate_);
            if (MV_OK == nRet) {
                RCLCPP_INFO(this->get_logger(), "Frame rate set to: %f", frame_rate_);
            } else {
                RCLCPP_WARN(this->get_logger(), "Failed to set frame rate: 0x%x", nRet);
            }
        } else if (param.get_name() == "pixel_format") {
            pixel_format_ = param.as_string();
            RCLCPP_WARN(this->get_logger(), "Pixel format change requires camera restart");
        }
    }
    
    return result;
}
