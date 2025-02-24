import os, argparse, json, struct, time, cv2, math
from pickle import TRUE
from time import sleep 
from scipy.optimize import leastsq
from base.python.Control import Control
from shared.python.framewrite import writeFrame
from Layers.Camera.CustomPointCloud import *
#from base.python.PointCloud.PointCloud import convertToPointCloud, convertToPointCloudOptimized
from base.python.Stream import Streaming
from base.python.Streaming import Data
from base.python.Streaming.BlobServerConfiguration import BlobClientConfig
from shared.python.devices_config import get_device_config
from Layers.Camera.Visuals import *

def filter_rectangular_region(point_cloud, left=212, right=528, top=151, bottom=384):
    """
    Filters the point cloud to keep only points within the specified rectangular region.
    """
    # Assuming point_cloud is H x W x 3 (X, Y, Z)
    height, width, _ = point_cloud.shape
    
    # Ensure indices are within bounds
    left = max(0, left)
    right = min(width, right)
    top = max(0, top)
    bottom = min(height, bottom)
    
    # Extract the relevant region
    filtered_points = point_cloud[top:bottom, left:right].reshape(-1, 3)
    
    return filtered_points

def initialize_stream(ip_address, cola_protocol, control_port, device_type, roi=False):
    """
    Initializes the device stream and returns control and streaming objects.
    """
    device_control = Control(ip_address, cola_protocol, control_port)
    device_control.open()
    device_control.stopStream()
    sleep(0.1)
    device_control.login(Control.USERLEVEL_SERVICE, 'CUST_SERV')
    if roi:
        print("Setting ROI for auto exposure, color and white balance.")
        print("//-----------------------------------------------")
        print("Read integration time before autoexposure")
        print(f"Read IntegrationTimeUS: {device_control.getIntegrationTimeUs()}")
        print(f"Read IntegrationTimeUSColor: {device_control.getIntegrationTimeUsColor()}")

        left, right, top, bottom = 212, 528, 151, 384
        device_control.setAutoExposure3DROI(left, right, top, bottom)
        device_control.setAutoExposureColorROI(left, right, top, bottom)
        # NOTE: The user is responisble to make sure that the region he sets the ROI to, is actually white.
        device_control.setAutoWhiteBalanceROI(left, right, top, bottom)
        for i in range(3):
            auto_type = i
            auto_exposure_response = device_control.startAutoExposureParameterized(
                struct.pack(">HB", 1, auto_type))
            if not auto_exposure_response:
                print(
                    f"ERROR: Invoking 'TriggerAutoExposureParameterized' fails! (autoExposureResponse: {auto_exposure_response}")
            # Wait until auto exposure method is finished
            auto_exp_param_running = True
            start_time = time.time()
            time_now = start_time
            while auto_exp_param_running:
                auto_exp_param_running = device_control.getAutoExposureParameterizedRunning()
                time_now = time.time()
                # 10 sec (time after auto exposure method should be finished)
                if (time_now - start_time) <= 10:
                    time.sleep(1)
                else:
                    print(
                        f"TIMEOUT: auto exposure function (Param: {auto_type}) needs longer than expected!")
        print("//-----------------------------------------------")
        print("Read integration time after autoexposure (integration time changed indirectly by autoexposure)")
        print(f"Read IntegrationTimeUS: {device_control.getIntegrationTimeUs()}")
        print(f"Read IntegrationTimeUSColor: {device_control.getIntegrationTimeUsColor()}")

    #device_control.setIntegrationTimeUs(1550)
    #print("\nSet integration time to 1550 micor seconds.\n")
    print("//-----------------------------------------------")
    print("Read integration time after autoexposure (integration time changed indirectly by autoexposure)")
    print(f"Read IntegrationTimeUS: {device_control.getIntegrationTimeUs()}")
    print(f"Read IntegrationTimeUSColor: {device_control.getIntegrationTimeUsColor()}")
    streaming_settings = BlobClientConfig(device_control)
    streaming_settings.setTransportProtocol(streaming_settings.PROTOCOL_TCP)
    streaming_settings.setBlobTcpPort(2114)
    streaming_device = Streaming(ip_address, 2114)
    streaming_device.openStream()
    device_control.logout()
    device_control.singleStep()

    return device_control, streaming_device

def writePointCloudToPCD(filename, wCoordinates):
    with open(filename, 'w') as f:
        for item in wCoordinates:
            f.write(("{} {} {}\n").format(item[0], item[1], item[2]))

def find_closest_point(point_cloud):
    """
    Finds the point in the point cloud with the shortest Euclidean distance from the origin.
    
    :param point_cloud: List of points, each in [X, Y, Z, R, G, B, I] format
    :return: The closest point as a list [X, Y, Z, R, G, B, I]
    """
    # Initialize min distance and closest point
    min_distance = float('inf')
    closest_point = None
    
    for point in point_cloud:
        x, y, z = point[:3]  # Extract X, Y, Z
        distance = math.sqrt(x**2 + y**2 + z**2)  # Compute Euclidean distance
        
        if distance < min_distance:
            min_distance = distance
            closest_point = point
    
    return closest_point

def process_frame(streaming_device, device_type):
    """
    Processes frames from the streaming device and visualizes the point cloud.
    """
    
    sensor_data = Data.Data()
    streaming_device.getFrame()
    whole_frame = streaming_device.frame
    pcl_dir = 'VisionaryToPointCloud'
    os.makedirs(pcl_dir, exist_ok=True)
    
    sensor_data.read(whole_frame, convertToMM=True)
    print("Data Timestamp [YYYY-MM-DD HH:MM:SS.mm] = %04u-%02u-%02u %02u:%02u:%02u.%03u" % (
        sensor_data.getDecodedTimestamp()))
    if sensor_data.hasDepthMap:
        point_map = Visualizer()
        # Depth map visualization
        frame_number = sensor_data.depthmap.frameNumber
        print("Data contains depth map data:")
        print("=== Frame number: {}".format(frame_number))
        point_map.plot_frame_real_time(device_type, sensor_data)
        # Point cloud generation and visualization
        #roi = (223, 578, 197, 418)  # Region of interest in pixel coordinates
        #is_stereo = device_type == "Visionary-S"
        start_time = time.time()
        point_cloud_ply, dist_data = convertToPointCloud(
            sensor_data.depthmap.distance, 
            sensor_data.depthmap.intensity,
            sensor_data.depthmap.confidence, 
            sensor_data.cameraParams, 
            sensor_data.xmlParser.stereo
            )
        execution_time = time.time() - start_time
        print(f"convertToPointCloud took: {execution_time:.3}s")
        #point_cloud = convertToPointCloudOptimized(
        #    sensor_data.depthmap.distance,
        #    sensor_data.depthmap.confidence,
        #    sensor_data.cameraParams,
        #    #depth_range=depth_range, 
        #    #roi=roi,
        #    isStereo=is_stereo
        #)
        
        #point_cloud_roi, dist_data = convertToPointCloudROI(
        #    sensor_data.depthmap.distance,
        #    sensor_data.depthmap.intensity,
        #    sensor_data.depthmap.confidence,
        #    sensor_data.cameraParams,
        #    is_stereo,
        #    roi)
        #print("confidence: ", sensor_data.depthmap.confidence)
        # Filter the point cloud to keep only points within the specified rectangular region
        #point_cloud = filter_rectangular_region(point_cloud, *roi)
        #print("point cloud raw data: ", dist_data)

        # Save to a JSON file
        #writePointCloudToPCD(os.path.join(pcl_dir, f"point_cloud_{frame_number}.pcd"), point_cloud)
        
        depth_range = (548,550) # Adjust based on your specific use case
        point_map.point_cloud = point_cloud_ply
        point_map.depth_range = depth_range
        point_map.type = "ply"
        start_time = time.time()
        point_map.visualize_point_cloud_with_open3d()
        #point_map.process_and_visualize_point_cloud(visualize=True)
        print("Centroid: ", point_map.centroid)
        #print("Contour: ", point_map.contour)
        
        execution_time = time.time() - start_time
        print(f"Contour processing took: {execution_time:.3}s")
    
def cleanup(device_control, streaming_device):
    """
    Cleans up resources and closes connections.
    """
    # Stop image aqcuisition
    device_control.stopStream()
    streaming_device.closeStream()
    device_control.close()
    #cv2.destroyAllWindows()

def runWelcomeDemo(ip_address: str, cola_protocol: str, control_port: int, device_type: str):
    # Record the start time
    start_time = time.time()
    device_control, streaming_device = initialize_stream(
        ip_address, cola_protocol, 
        control_port, device_type, roi=True)
    execution_time = time.time() - start_time
    print(f"Initialization took: {execution_time:.3}s")

    start_time = time.time()
    try:
        process_frame(streaming_device, device_type)
        execution_time = time.time() - start_time
        print(f"Frame processing took: {execution_time:.3}s")
    except KeyboardInterrupt:
        print("Stream interrupted.")
    finally:
        cleanup(device_control, streaming_device)
        print("Stream closed and resources released.")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="This welcome demo shows what is possible with the API.")
    parser.add_argument('-i', '--ipAddress', required=False, type=str,
                        default="192.168.1.10", help="The ip address of the device.")
    parser.add_argument('-d', '--device_type', required=False, type=str,
                        default="Visionary-T Mini", choices=["Visionary-S", "Visionary-T Mini"],
                        help="Visionary product type.")
    args = parser.parse_args()
    start_time = time.time()
    cola_protocol, control_port, _ = get_device_config(args.device_type)

    runWelcomeDemo(args.ipAddress, cola_protocol,
                   control_port, args.device_type)
    execution_time = time.time() - start_time
    print(f"Total execution time: {execution_time:.3}s")