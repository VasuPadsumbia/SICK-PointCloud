#include "Camera.h"
#include <tuple>

Camera& Camera::getInstance() {
    static Camera instance;
    return instance;
}

Camera::Camera()
    : ip_address("192.168.1.10"), device_type("Visionary-S"), stream_port(2114), roi(false),
      left_roi(215), right_roi(409), top_roi(220), bottom_roi(345),
      integration_time(1200), integration_time_color(5000), depth_range(std::make_tuple(680, 697)), setup_updated(false)
  , timestamp(0)
  , device_control(nullptr)
{
    //visualizer = std::make_unique<Visualizer>();
  visualizer = std::make_shared<Visualizer>();
        if (device_type == "Visionary-S")
        {
          stream_port = 2114;
          VisionaryType visionaryType(VisionaryType::eVisionaryS); // Add this line
        }
        else if (device_type == "Visionary-T-mini")
        {
          stream_port = 2116;
          VisionaryType visionaryType(VisionaryType::eVisionaryTMini); // Add this line
       
        }
}

Camera::~Camera() {
    cleanup();
}

void Camera::run(int count, bool image_plot, bool point_cloud_plot, bool PCD, bool PLY, bool save_image, bool save_point_cloud) {
    auto start_time = std::chrono::high_resolution_clock::now();
    initializeStream();
    auto end_time = std::chrono::high_resolution_clock::now();
    std::cout << "Initialization took: " << std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count() << "ms\n";

    try {
        while (count > 0) {
            start_time = std::chrono::high_resolution_clock::now();
            processFrame(image_plot, point_cloud_plot, PCD, PLY, save_image, save_point_cloud);
            end_time = std::chrono::high_resolution_clock::now();
            std::cout << "Processing frame took: " << std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count() << "ms\n";
            --count;
        }
    } catch (const std::exception& e) {
        std::cerr << "Exception: " << e.what() << "\n";
    } catch (...) {
        std::cerr << "Unknown exception\n";
    }
    cleanup();
}

void Camera::setROI(int left_roi, int right_roi, int top_roi, int bottom_roi) {
    roi = true;
    this->left_roi = left_roi;
    this->right_roi = right_roi;
    this->top_roi = top_roi;
    this->bottom_roi = bottom_roi;
}

void Camera::setIntegrationTime(int integration_time, int integration_time_color) {
    this->integration_time = integration_time;
    this->integration_time_color = integration_time_color;
    this->setup_updated = true;
}

void Camera::setDepthRange(std::tuple<float, float> depth_range) {
    this->depth_range = depth_range;
}

ExitCode Camera::initializeStream() {
    
    device_control = std::make_shared<VisionaryControl>(visionaryType);
    if (!device_control->open(ip_address))
    {
      std::cerr << "Failed to open control connection to device.\n";
      return ExitCode::eControlCommunicationError;
    }
    if (!device_control->stopAcquisition())
    {
      std::cerr << "Failed to stop acquisition.\n";
      return ExitCode::eControlCommunicationError;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    device_control->login(IAuthentication::UserLevel::SERVICE, "CUST_SERV");

    if (roi) {
      std::uint8_t acquisitionModeStereo = 0;
      CoLaCommand  setAcquisitionModeStereoCommand =
        CoLaParameterWriter(CoLaCommandType::WRITE_VARIABLE, "acquisitionModeStereo")
          .parameterUSInt(acquisitionModeStereo)
          .build();
      CoLaCommand setAcquisitionModeStereoResponse = device_control->sendCommand(setAcquisitionModeStereoCommand);

        std::cout << "Setting ROI for auto exposure, color and white balance.\n";
      CoLaCommand setAutoExposureROICommand = CoLaParameterWriter(CoLaCommandType::WRITE_VARIABLE, "autoExposureROI")
                                                .parameterUDInt(this->left_roi)
                                                  .parameterUDInt(this->right_roi)
                                                  .parameterUDInt(this->top_roi)
                                                  .parameterUDInt(this->bottom_roi)
                                                .build();
        CoLaCommand setAutoWhiteBalanceROICommand =
          CoLaParameterWriter(CoLaCommandType::WRITE_VARIABLE, "autoWhiteBalanceROI")
            .parameterUDInt(this->left_roi)
            .parameterUDInt(this->right_roi)
            .parameterUDInt(this->top_roi)
            .parameterUDInt(this->bottom_roi)
            .build();
      CoLaCommand setAutoExposureColorROICommand =
          CoLaParameterWriter(CoLaCommandType::WRITE_VARIABLE, "autoExposureColorROI")
            .parameterUDInt(this->left_roi)
            .parameterUDInt(this->right_roi)
            .parameterUDInt(this->top_roi)
            .parameterUDInt(this->bottom_roi)
            .build();
        std::cout << "Sending Command to Camera to set ROI \n";
        CoLaCommand setAutoExposureROIResponse = device_control->sendCommand(setAutoExposureROICommand);
      CoLaCommand setAutoWhiteBalanceROIResponse = device_control->sendCommand(setAutoWhiteBalanceROICommand);
        CoLaCommand setAutoExposureColorROIResponse = device_control->sendCommand(setAutoExposureColorROICommand);

        CoLaCommand getIntegrationTimeUsCommand =
          CoLaParameterWriter(CoLaCommandType::READ_VARIABLE, "integrationTimeUs").build();
        CoLaCommand   getIntegrationTimeUsResponse = device_control->sendCommand(getIntegrationTimeUsCommand);
        std::uint32_t integrationTimeUs            = CoLaParameterReader(getIntegrationTimeUsResponse).readUDInt();
        std::printf("Read integrationTimeUs = %d\n", integrationTimeUs);

        CoLaCommand getIntegrationTimeUsColorCommand =
          CoLaParameterWriter(CoLaCommandType::READ_VARIABLE, "integrationTimeUsColor").build();
        CoLaCommand getIntegrationTimeUsColorResponse = device_control->sendCommand(getIntegrationTimeUsColorCommand);
        std::uint32_t integrationTimeUsColor = CoLaParameterReader(getIntegrationTimeUsColorResponse).readUDInt();
        std::printf("Read integrationTimeUsColor = %d\n", integrationTimeUsColor);

        // tag::invoke_autoExposure[]
        for (uint8_t autoType = 0; autoType < 3;
             autoType++) // 0 = Auto Exposure 3D, 1 = Auto Exposure RGB, 2 = Auto White Balance
        {
          std::printf("Invoke method 'TriggerAutoExposureParameterized' (Param: %d) ...\n", autoType);

          CoLaCommand invokeAutoExposureCommand =
            CoLaParameterWriter(CoLaCommandType::METHOD_INVOCATION, "TriggerAutoExposureParameterized")
              .parameterUInt(1)
              .parameterUSInt(autoType)
              .build();
          CoLaCommand autoExposureResponse = device_control->sendCommand(invokeAutoExposureCommand);

          if (autoExposureResponse.getError() != CoLaError::OK)
          {
            std::printf("ERROR: Invoking 'TriggerAutoExposureParameterized' fails! (autoExposureResponse: %d)\n",
                        CoLaParameterReader(autoExposureResponse).readBool());
          }

          // Wait until auto exposure method is finished
          bool      autoExpParamRunning = true;
          long long startTime           = std::chrono::system_clock::now().time_since_epoch().count();
          long long timeNow             = startTime;
          while (autoExpParamRunning)
          {
            CoLaCommand getAutoExpParamRunningCommand =
              CoLaParameterWriter(CoLaCommandType::READ_VARIABLE, "autoExposureParameterizedRunning").build();
            CoLaCommand autoExpParamRunningResponse = device_control->sendCommand(getAutoExpParamRunningCommand);
            autoExpParamRunning                     = CoLaParameterReader(autoExpParamRunningResponse).readBool();

            timeNow = std::chrono::system_clock::now().time_since_epoch().count();
            if ((timeNow - startTime)
                <= 10000000000) // 10 sec = 10 000 000 000 ns (time after auto exposure method should be finished)
            {
              std::this_thread::sleep_for(std::chrono::seconds(1));
            }
            else
            {
              std::printf("TIMEOUT: auto exposure function (Param: %d) needs longer than expected!\n", autoType);
            }
          }
        }
        // end::invoke_autoExposure[]

        // Read changed integration time values (after auto exposure was triggered)
        getIntegrationTimeUsCommand  = CoLaParameterWriter(CoLaCommandType::READ_VARIABLE, "integrationTimeUs").build();
        getIntegrationTimeUsResponse = device_control->sendCommand(getIntegrationTimeUsCommand);
        integrationTimeUs            = CoLaParameterReader(getIntegrationTimeUsResponse).readUDInt();
        std::printf("Read integrationTimeUs = %d\n", integrationTimeUs);

        getIntegrationTimeUsColorCommand =
          CoLaParameterWriter(CoLaCommandType::READ_VARIABLE, "integrationTimeUsColor").build();
        getIntegrationTimeUsColorResponse = device_control->sendCommand(getIntegrationTimeUsColorCommand);
        integrationTimeUsColor            = CoLaParameterReader(getIntegrationTimeUsColorResponse).readUDInt();
        std::printf("Read integrationTimeUsColor = %d\n", integrationTimeUsColor);

    }
        
    if (this->setup_updated) {
        // tag::set_integrationTime[]
        CoLaCommand setIntegrationTimeUsCommand =
          CoLaParameterWriter(CoLaCommandType::WRITE_VARIABLE, "integrationTimeUs")
            .parameterUDInt(this->integration_time)
            .build();
        CoLaCommand setIntegrationTimeUsResponse = device_control->sendCommand(setIntegrationTimeUsCommand);
        // end::set_integrationTime[]
        std::cout << "Setting integration time to " << this->integration_time << " micro seconds.\n";
        // tag::set_integrationTime[]
        CoLaCommand setIntegrationTimeUsColorCommand =
          CoLaParameterWriter(CoLaCommandType::WRITE_VARIABLE, "integrationTimeUsColor")
            .parameterUDInt(this->integration_time_color)
            .build();
        CoLaCommand setIntegrationTimeUsColorResponse = device_control->sendCommand(setIntegrationTimeUsColorCommand);
        // end::set_integrationTime[]
        std::cout << "Setting color integration time to " << this->integration_time_color << "micro seconds.\n"; 
        this->setup_updated = false;
    }
   
    setTransportProtocol(device_control, "TCP"); // TCP
    setBlobTcpPort(device_control, stream_port);
    device_control->logout();

    return ExitCode::eOk;
}

void Camera::cleanup() {
        device_control->stopAcquisition();
        //std::this_thread::sleep_for(std::chrono::milliseconds(10));
        device_control->close();
}

ExitCode Camera::processFrame(bool image_plot, bool point_cloud_plot, bool PCD, bool PLY, bool save_image, bool save_point_cloud) 
{
  auto start_time = std::chrono::high_resolution_clock::now();
    const std::chrono::milliseconds pollPeriodSpan{pollperiodMs};
    auto lastSnapTime = std::chrono::steady_clock::now();
    pFrameGrabber    = device_control->createFrameGrabber();
    pDataHandler     = device_control->createDataHandler();
    //-----------------------------------------------
    // acquire a single snapshot
    
    // make sure we don't overrun the device
    // (otherwise snapshot requests would be dropped by the device)
    // tag::avoid_overrun[]
    const auto timeSinceLastSnap = std::chrono::steady_clock::now() - lastSnapTime;

    if (timeSinceLastSnap < pollPeriodSpan)
    {
      auto timeToWait = pollPeriodSpan - timeSinceLastSnap;
      std::this_thread::sleep_for(timeToWait);
    }
    // end::avoid_overrun[]

    // now we are not too fast and can trigger a snapshot
    // tag::acquire_snapshots[]
    lastSnapTime = std::chrono::steady_clock::now();
    if (!device_control->stepAcquisition())
    {
      std::cerr << "Failed to trigger a snapshot\n";
      return ExitCode::eControlCommunicationError;
    }
    if (!pFrameGrabber->genGetNextFrame(pDataHandler))
    {
      std::cerr << "Frame timeout for snapshot\n";
      return ExitCode::eFrameTimeout;
    }
    else
    {
      std::cout << "Frame received in snapshot mode, frame #" << pDataHandler->getFrameNum() << "\n";
      if (save_image)
      {
        writeFrame(visionaryType, *pDataHandler, "");
      }
      // Convert data to a point cloud
      pDataHandler->generatePointCloud(point_cloud_ply);
      pDataHandler->transformPointCloud(point_cloud_ply);
    }

    //auto end_time = std::chrono::high_resolution_clock::now();
    //std::chrono::duration<double> elapsed_seconds = end_time - start_time;
    //std::cout << "Point cloud process time: " << elapsed_seconds.count() << "s\n";
    std::uint64_t     timestamp_ms = pDataHandler->getTimestampMS();
    std::time_t       timestamp_s  = timestamp_ms / 1000;
    std::tm           tm           = *std::gmtime(&timestamp_s);
    std::stringstream ss;
    ss << std::put_time(&tm, "%Y-%m-%d %H:%M:%S") << '.' << std::setw(3) << std::setfill('0') << (timestamp_ms % 1000);
    std::cout << "Data Timestamp [YYYY-MM-DD HH:MM:SS.mm] = " << ss.str() << "\n";
    timestamp = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
    if (image_plot)
    {
      visualizer->plotFrameRealTime(*pDataHandler);
    }
    if (save_point_cloud)
    {
      // Write point cloud to PLY
      const std::string framePrefix  = std::to_string(pDataHandler->getFrameNum());
      std::string       plyFilePath  = framePrefix + "-pointcloud.ply";
      const char*       cPlyFilePath = plyFilePath.c_str();
      std::printf("Writing frame to %s\n", cPlyFilePath);
      PointCloudPlyWriter::WriteFormatPLY(cPlyFilePath, point_cloud_ply, pDataHandler->getRGBAMap(), true);
    }
    if (point_cloud_plot)
    {
      const VisionarySData& data = dynamic_cast<const VisionarySData&>(*pDataHandler);
      visualizer->setPointCloudColor(point_cloud_ply, data.getRGBAMap());
      visualizer->visualizePointCloud();
    }
    auto end_time = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed_seconds = end_time - start_time;
    std::cout << "Point cloud process time: " << elapsed_seconds.count() << "s\n";
    return ExitCode::eOk;
}

std::tuple<pcl::PointCloud<pcl::PointXYZ>, 
    Eigen::Vector3d, 
    Eigen::Vector3d> Camera::getContours(
        bool plot, double eps, int min_samples)
{
    visualizer->setDepthRange(depth_range);
    visualizer->setPointCloudColor(point_cloud_ply, pDataHandler->getRGBAMap());
    if (plot) {
      return std::tie(this->contour, this->centroid, this->point_color) = 
          visualizer->processAndVisualizePointCloud(true, eps, min_samples);
    
    } else {
      return std::tie(this->contour, this->centroid, this->point_color) =
        visualizer->processAndVisualizePointCloud(false, eps, min_samples);
    }
}

//std::tuple<std::vector<float>, std::vector<float>, std::tuple<int, int>> Camera::getDepthRange() {
//    visualizer->setPointCloud(point_cloud_ply);
//    visualizer->visualizePointCloud();
//    auto point_1 = visualizer->pick_points[0];
//    auto point_2 = visualizer->pick_points[1];
//    auto depth_range = std::make_tuple(point_1[3], point_2[3]);
//    return std::make_tuple(point_1, point_2, depth_range);
//}
//