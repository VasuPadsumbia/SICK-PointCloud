//
// Copyright (c) 2023,2024 SICK AG, Waldkirch
//
// SPDX-License-Identifier: Unlicense

#include <cinttypes>
#include <cstdint>
#include <cstdio>
#include <memory>
#include <string>

#include <iostream>
#include <sstream>

#include <chrono>
#include <thread>

#include <sick_visionary_cpp_base/CoLaParameterWriter.h>
#include <sick_visionary_cpp_base/FrameGrabber.h>
#include <sick_visionary_cpp_base/PointCloudPlyWriter.h>
#include <sick_visionary_cpp_base/PointXYZ.h>
#include <sick_visionary_cpp_base/VisionaryControl.h>
#include <sick_visionary_cpp_base/VisionaryType.h>

#include "exitcodes.h"
#include "framewrite.h"

static ExitCode runWelcomeDemo(visionary::VisionaryType visionaryType, const std::string& ipAddress)
{
  auto start = std::chrono::high_resolution_clock::now();
  using namespace visionary;
  VisionaryControl visionaryControl(visionaryType);

  if (!visionaryControl.open(ipAddress))
  {
    std::fprintf(stderr, "Failed to open control connection to device.\n");

    return ExitCode::eControlCommunicationError;
  }

  // Stop image acquisition
  if (!visionaryControl.stopAcquisition())
  {
    std::fprintf(stderr, "Failed to stop acquisition.\n");

    return ExitCode::eControlCommunicationError;
  }

  // Wait for the device configuration
  std::this_thread::sleep_for(std::chrono::milliseconds(50));

  // create a frame grabber suitable for the Visionary type used in visionaryControl
  auto pFrameGrabber = visionaryControl.createFrameGrabber();

  // the data handler pointer will later contain the frame data
  auto pDataHandler = visionaryControl.createDataHandler();

  // acquire a single snapshot
  if (!visionaryControl.stepAcquisition())
  {
    std::fprintf(stderr, "Failed to trigger a snapshot\n");

    return ExitCode::eControlCommunicationError;
  }

  
  auto point_cloud_start = std::chrono::high_resolution_clock::now();
  // the snapshot has possibly already arrived, so parameter onlyNewer is false
  if (!pFrameGrabber->genGetNextFrame(pDataHandler))
  {
    std::fprintf(stderr, "Frame timeout for snapshot\n");

    return ExitCode::eFrameTimeout;
  }
  else
  {
    std::printf("Frame received in snapshot mode, frame #%" PRIu32 "\n", pDataHandler->getFrameNum());
    // write the frame to disk
    //writeFrame(visionaryType, *pDataHandler, "");
    // Convert data to a point cloud
    std::vector<PointXYZ> pointCloud;
    pDataHandler->generatePointCloud(pointCloud);
    pDataHandler->transformPointCloud(pointCloud);    
  }
  auto point_cloud_end = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> elapsed_seconds = point_cloud_end - point_cloud_start;
  std::cout << "Point cloud process time: " << elapsed_seconds.count() << "s\n";
  // Stop image acquisition
  if (!visionaryControl.stopAcquisition())
  {
    std::fprintf(stderr, "Failed to stop acquisition.\n");

    return ExitCode::eControlCommunicationError;
  }
  // delete the frame grabber
  //pFrameGrabber.reset();

  visionaryControl.close();
  std::printf("Logout and close.\n");
  auto end = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> execution_time = end - start;
  std::cout << "Welcome Function process time: " << execution_time.count() << "s\n";

  return ExitCode::eOk;
}

int main(int argc, char* argv[])
{
  std::string deviceIpAddr{"192.168.1.10"};
  std::string filePrefix{""};

  visionary::VisionaryType visionaryType(visionary::VisionaryType::eVisionaryS);

  bool showHelpAndExit = false;

  ExitCode exitCode = ExitCode::eOk;

  for (int i = 1; i < argc; ++i)
  {
    std::istringstream argstream(argv[i]);

    if (argstream.get() != '-')
    {
      showHelpAndExit = true;
      exitCode        = ExitCode::eParamError;
      break;
    }
    switch (argstream.get())
    {
      case 'h':
        showHelpAndExit = true;
        break;
      case 'i':
        argstream >> deviceIpAddr;
        break;
      case 'd':
      {
        std::string visionaryTypeName;
        argstream >> visionaryTypeName;
        try
        {
          visionaryType = visionary::VisionaryType::fromString(visionaryTypeName);
        }
        catch (const std::invalid_argument& e)
        {
          // NOLINTNEXTLINE(performance-avoid-endl)
          std::cerr << e.what() << ": '" << visionaryTypeName << "'" << std::endl;
          showHelpAndExit = true;
          exitCode        = ExitCode::eParamError;
        }
        break;
      }
      default:
        showHelpAndExit = true;
        break;
    }
  }

  if (showHelpAndExit)
  {
    std::cout << "\nUsage: " << argv[0] << " [option]*\n";

    std::cout << "where option is one of\n";
    std::cout << "-h              show this help and exit\n";
    std::cout << "-i<IP>          connect to the device with IP address <IP>; default is " << deviceIpAddr << '\n';
    std::cout << "-d<device type> visionary product type; default is '" << visionaryType.toString() << "'\n";

    std::cout << "\nVisionary product types:\n";
    for (const auto& name : visionary::VisionaryType::getNames())
    {
      std::cout << "  " << name << '\n';
    }

    return static_cast<int>(exitCode);
  }
  
  auto start = std::chrono::high_resolution_clock::now();
  runWelcomeDemo(visionaryType, deviceIpAddr);
  
  printf("Welcome function ended!\n");
  auto end = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> execution_time = end - start;

  std::cout << "Point cloud process time: " << execution_time.count() << "s\n";
  std::cout << "exit code " << static_cast<int>(exitCode) << '\n';

  return static_cast<int>(exitCode);
}
