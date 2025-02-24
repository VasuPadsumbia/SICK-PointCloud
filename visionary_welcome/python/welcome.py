#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright (c) 2024 SICK AG, Waldkirch
#
# SPDX-License-Identifier: Unlicense

import argparse
import os
from pickle import TRUE
import time
from time import sleep

from base.python.Control import Control
from base.python.PointCloud.PointCloud import (convertToPointCloud,
                                               convertToPointCloudOptimized,
                                               writePointCloudToPCD,
                                               writePointCloudToPLY)
from base.python.Stream import Streaming
from base.python.Streaming import Data
from base.python.Streaming.BlobServerConfiguration import BlobClientConfig
from shared.python.framewrite import writeFrame
from shared.python.devices_config import get_device_config


def runWelcomeDemo(ip_address: str, cola_protocol: str, control_port: int, device_type: str):
    # Directories to save the output in
    pcl_dir = 'VisionaryToPointCloud'
    img_dir = 'VisionaryImages'
    os.makedirs(pcl_dir, exist_ok=True)
    os.makedirs(img_dir, exist_ok=True)

    # Connect to control channel
    device_control = Control(ip_address, cola_protocol, control_port)
    device_control.open()

    # Stop image acquisition
    device_control.stopStream()

    # Wait for stop command to be complete
    sleep(0.1)

    # Login to the device for access rights to certain methods
    device_control.login(Control.USERLEVEL_SERVICE, 'CUST_SERV')

    # Streaming settings
    streaming_settings = BlobClientConfig(device_control)
    streaming_device = None
    streaming_settings.setTransportProtocol(
        streaming_settings.PROTOCOL_TCP)
    streaming_settings.setBlobTcpPort(2114)
    # Open streaming channel
    streaming_device = Streaming(ip_address, 2114)
    streaming_device.openStream()

    # Logout after configuration
    device_control.logout()

    # Take a snapshot
    device_control.singleStep()
    sensor_data = Data.Data()
    streaming_device.getFrame()
    whole_frame = streaming_device.frame
    sensor_data.read(whole_frame, convertToMM=True)
    print("Data Timestamp [YYYY-MM-DD HH:MM:SS.mm] = %04u-%02u-%02u %02u:%02u:%02u.%03u" % (
        sensor_data.getDecodedTimestamp()))
    if sensor_data.hasDepthMap:
                    frame_number = sensor_data.depthmap.frameNumber
                    print("Data contains depth map data:")
                    print("=== Write PNG file: Frame number: {}".format(frame_number))
                    writeFrame(device_type, sensor_data,
                            os.path.join(img_dir, ""))
                    print("=== Converting image to pointcloud")

                    # Non optimized
                    start_time = time.time()
                    world_coordinates, dist_data = convertToPointCloud(sensor_data.depthmap.distance,
                                                                    sensor_data.depthmap.intensity,
                                                                    sensor_data.depthmap.confidence,
                                                                    sensor_data.cameraParams, sensor_data.xmlParser.stereo)
                    end_time = time.time()
                    execution_time = end_time - start_time
                    print(f"convertToPointCloud took: {execution_time:.3}s")

                    # Optimized
                    is_stereo = True if device_type == "Visionary-S" else False
                    start_time = time.time()
                    point_cloud = convertToPointCloudOptimized(sensor_data.depthmap.distance,
                                                                        sensor_data.depthmap.confidence,
                                                                        sensor_data.cameraParams, is_stereo)
                    end_time = time.time()
                    execution_time = end_time - start_time
                    print(f"convertToPointCloudOptimized took: {execution_time:.3}s")

                    # Write output of the non optimized function to PLY
                    writePointCloudToPLY(os.path.join(
                        pcl_dir, "world_coordinates{}.ply".format(frame_number)), world_coordinates)

                    # Write output of the optimized function to PCD
                    writePointCloudToPCD(os.path.join(
                        pcl_dir, "world_coordinates{}.pcd".format(frame_number)), point_cloud.reshape(-1, point_cloud.shape[-1]))

    # Stop image aqcuisition
    device_control.stopStream()

    # Login to the device for access rights to certain methods
    device_control.login(Control.USERLEVEL_SERVICE, 'CUST_SERV')

    # change one frontend setting and take another image
    if device_type == "Visionary-T Mini":
        device_control.setFramePeriodUs(60000)
        print("\nSet frame period to 60000 micro seconds.\n")
    elif device_type == "Visionary-S":
        device_control.setIntegrationTimeUs(3000)
        print("\nSet integration time to 3000 micor seconds.\n")

    # Logout after configuration
    device_control.logout()

    # Take a snapshot
    device_control.singleStep()
    sensor_data = Data.Data()
    streaming_device.getFrame()
    whole_frame = streaming_device.frame
    sensor_data.read(whole_frame, convertToMM=False)
    print("Data Timestamp [YYYY-MM-DD HH:MM:SS.mm] = %04u-%02u-%02u %02u:%02u:%02u.%03u" % (
        sensor_data.getDecodedTimestamp()))
    if sensor_data.hasDepthMap:
                    frame_number = sensor_data.depthmap.frameNumber
                    print("Data contains depth map data:")
                    print("=== Write PNG file: Frame number: {}".format(frame_number))
                    writeFrame(device_type, sensor_data,
                            os.path.join(img_dir, ""))
                    print("=== Converting image to pointcloud")

                    # Non optimized
                    start_time = time.time()
                    world_coordinates, dist_data = convertToPointCloud(sensor_data.depthmap.distance,
                                                                    sensor_data.depthmap.intensity,
                                                                    sensor_data.depthmap.confidence,
                                                                    sensor_data.cameraParams, sensor_data.xmlParser.stereo)
                    end_time = time.time()
                    execution_time = end_time - start_time
                    print(f"convertToPointCloud took: {execution_time:.3}s")

                    # Optimized
                    is_stereo = True if device_type == "Visionary-S" else False
                    start_time = time.time()
                    point_cloud = convertToPointCloudOptimized(sensor_data.depthmap.distance,
                                                                        sensor_data.depthmap.confidence,
                                                                        sensor_data.cameraParams, is_stereo)
                    end_time = time.time()
                    execution_time = end_time - start_time
                    print(f"convertToPointCloudOptimized took: {execution_time:.3}s")

                    # Write output of the non optimized function to PLY
                    writePointCloudToPLY(os.path.join(
                        pcl_dir, "world_coordinates{}.ply".format(frame_number)), world_coordinates)

                    # Write output of the optimized function to PCD
                    writePointCloudToPCD(os.path.join(
                        pcl_dir, "world_coordinates{}.pcd".format(frame_number)), point_cloud.reshape(-1, point_cloud.shape[-1]))


    # Stop image aqcuisition
    device_control.stopStream()

    # Close streaming channel
    streaming_device.closeStream()

    # Logout and close
    device_control.close()
    print("Logout and close.")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="This welcome demo shows what is possible with the API.")
    parser.add_argument('-i', '--ipAddress', required=False, type=str,
                        default="192.168.1.10", help="The ip address of the device.")
    parser.add_argument('-d', '--device_type', required=False, type=str,
                        default="Visionary-T Mini", choices=["Visionary-S", "Visionary-T Mini"],
                        help="Visionary product type.")
    args = parser.parse_args()

    cola_protocol, control_port, _ = get_device_config(args.device_type)

    runWelcomeDemo(args.ipAddress, cola_protocol,
                   control_port, args.device_type)
