import os, struct, time
from time import sleep 
from base.python.Control import Control
from shared.python.framewrite import writeFrame
from base.python.PointCloud.PointCloud import (convertToPointCloud,
                                               convertToPointCloudOptimized,
                                               writePointCloudToPCD,
                                               writePointCloudToPLY)
#from Layers.Camera.CustomPointCloud import convertToPointCloud
from base.python.Stream import Streaming
from base.python.Streaming import Data
from base.python.Streaming.BlobServerConfiguration import BlobClientConfig
from shared.python.devices_config import get_device_config
from Layers.Camera.Visuals import *
from concurrent.futures import ThreadPoolExecutor
from datetime import datetime, timezone

class Singleton_meta(type):
    _instances = {}
    def __call__(cls, *args, **kwargs):
        if cls not in cls._instances:
            cls._instances[cls] = super(Singleton_meta, cls).__call__(*args, **kwargs)
        return cls._instances[cls]
    
class Camera( metaclass=Singleton_meta):
    """
    Class to handle the camera stream and process frames.

    Args:
        ip_address (str): IP address of the camera device.
        device_type (str): Type of the camera device.
        stream_port (int): Port for streaming the camera data.
        roi (bool): Flag to set the region of interest (ROI) for the camera stream.
        left_roi (int): Left boundary of the ROI.
        right_roi (int): Right boundary of the ROI.
        top_roi (int): Top boundary of the ROI.
        bottom_roi (int): Bottom boundary of the ROI.
        integration_time (int): Integration time for the depth map.
        integration_time_color (int): Integration time for the color image.
    """
    def __init__(self, ip_address="192.168.1.10", device_type="Visionary-S", 
                 stream_port=2114, roi=False, 
                 left_roi=215, right_roi=409,
                 top_roi=220, bottom_roi=345,
                 integration_time=1200, integration_time_color=5000) -> None:
        
        self.ip_address = ip_address
        self.device_type = device_type
        self.stream_port = stream_port
        self.roi = roi
        self.left_roi = left_roi
        self.right_roi = right_roi
        self.top_roi = top_roi
        self.bottom_roi = bottom_roi
        self.integration_time = integration_time
        self.integration_time_color = integration_time_color
        self.cola_protocol, self.control_port, _ = get_device_config(self.device_type)
        self.depth_range = (680, 697)  # Adjust based on your specific use case
        self.device_control = None 
        self.streaming_device = None
        self.point_cloud_pcd = None
        self.point_cloud_ply = None
        self.dist_data = None
        self.Visualizer = Visualizer()
        self.contours = None
        self.centroid = None    
        self.color = None
        self.setup_updated = False
        self.timestamp = None

    def run(self, count=1, image_plot=False, point_cloud_plot=False, PCD=False, PLY=False, save_image=False, save_point_cloud=False):
        """
        Runs the camera stream and processes frames.
        Args:
            count (int): Number of frames to process.
            image_plot (bool): Plot the depth map image.
            point_cloud_plot (bool): Plot the point cloud.
            PCD (bool): Save the point cloud as a PCD file.
            PLY (bool): Save the point cloud as a PLY file.
        """
        start_time = time.time()
        self.device_control, self.streaming_device = self.initialize_stream()
        print(f"Initialization took: {time.time() - start_time:.3}s")

        try:
            while count > 0:
                start_time = time.time()
                self.process_frame(image_plot, point_cloud_plot, PCD, PLY, save_image, save_point_cloud)
                print(f"Processing frame took: {time.time() - start_time:.3}s")
                count -= 1
        except KeyboardInterrupt:
            print("Terminating")
        finally:
            self.cleanup()
    
    
    def set_roi(self, left_roi, right_roi, top_roi, bottom_roi):
        """
        Sets the region of interest (ROI) for the camera stream.
        Args:
            left_roi (int): Left boundary of the ROI.
            right_roi (int): Right boundary of the ROI.
            top_roi (int): Top boundary of the ROI.
            bottom_roi (int): Bottom boundary of the ROI.
        """
        self.roi = True
        self.left_roi = left_roi
        self.right_roi = right_roi
        self.top_roi = top_roi
        self.bottom_roi = bottom_roi
    
    def set_integration_time(self, integration_time, integration_time_color):
        """
        Sets the integration time for the camera stream.
        Args:
            integration_time (int): Integration time for the depth map.
            integration_time_color (int): Integration time for the color image.
        """
        self.integration_time = integration_time
        self.integration_time_color = integration_time_color
        self.setup_updated = True

    def set_depth_range(self, depth_range):
        """
        Sets the depth range for the point cloud.
        Args:
            depth_range (tuple): Tuple (min_depth, max_depth) specifying the depth range.
        """
        self.depth_range = depth_range
    
    def initialize_stream(self):
        """
        Initializes the device stream and returns control and streaming objects.
        """
        device_control = Control(self.ip_address, self.cola_protocol, self.control_port)
        device_control.open()
        device_control.stopStream()
        sleep(0.1)
        device_control.login(Control.USERLEVEL_SERVICE, 'CUST_SERV')
        if self.roi:
            print("Setting ROI for auto exposure, color and white balance.")
            print("//-----------------------------------------------")
            print("Read integration time before autoexposure")
            print(f"Read IntegrationTimeUS: {device_control.getIntegrationTimeUs()}")
            print(f"Read IntegrationTimeUSColor: {device_control.getIntegrationTimeUsColor()}")
            device_control.setAutoExposure3DROI(self.left_roi, self.right_roi, self.top_roi, self.bottom_roi)
            device_control.setAutoExposureColorROI(self.left_roi, self.right_roi, self.top_roi, self.bottom_roi)
            # NOTE: The user is responisble to make sure that the region he sets the ROI to, is actually white.
            device_control.setAutoWhiteBalanceROI(self.left_roi, self.right_roi, self.top_roi, self.bottom_roi)
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

        if device_control.getIntegrationTimeUs() != self.integration_time and self.setup_updated:
            device_control.setIntegrationTimeUs(self.integration_time)
            print(f"Setting integration time to {self.integration_time} micro seconds.")
            self.setup_updated = False

        if device_control.getIntegrationTimeUsColor() != self.integration_time_color and self.setup_updated:
            device_control.setIntegrationTimeUsColor(self.integration_time_color)
            print(f"Setting color integration time to {self.integration_time_color} micro seconds.")
            self.setup_updated = False

        print("//-----------------------------------------------")
        print(f"Read IntegrationTimeUS: {device_control.getIntegrationTimeUs()}")
        print(f"Read IntegrationTimeUSColor: {device_control.getIntegrationTimeUsColor()}")
        streaming_settings = BlobClientConfig(device_control)
        streaming_settings.setTransportProtocol(streaming_settings.PROTOCOL_TCP)
        streaming_settings.setBlobTcpPort(self.stream_port)
        streaming_device = Streaming(self.ip_address, self.stream_port)
        streaming_device.openStream()
        device_control.logout()
        device_control.singleStep()

        self.streaming_device = streaming_device
        self.device_control = device_control
        
    def cleanup(self):
        """
        Cleans up the device stream.
        """
        self.device_control.stopStream()
        sleep(0.1)
        self.streaming_device.closeStream()
        self.device_control.close()
    
    def process_frame(self, image_plot=False, 
                      point_cloud_plot=False, PCD=False, PLY=False, 
                      save_image=False, save_point_cloud=False):
        """
        Processes frames from the streaming device and visualizes the point cloud.
        Args:
            image_plot (bool): Plot the depth map image.
            point_cloud_plot (bool): Plot the point cloud.
            PCD (bool): Save the point cloud as a PCD file.
            PLY (bool): Save the point cloud as a PLY file.
            save_image (bool): Save the depth map image.
            save_point_cloud (bool): Save the point cloud.
        """
        sensor_data = Data.Data()
        self.streaming_device.getFrame()
        whole_frame = self.streaming_device.frame
        # Directories to save the output in
        pcl_dir = 'VisionaryToPointCloud'
        img_dir = 'VisionaryImages'
        os.makedirs(pcl_dir, exist_ok=True)
        os.makedirs(img_dir, exist_ok=True)
        start_time = time.time()
        sensor_data.read(whole_frame, convertToMM=True)
        print("Data Timestamp [YYYY-MM-DD HH:MM:SS.mm] = %04u-%02u-%02u %02u:%02u:%02u.%03u" % (
            sensor_data.getDecodedTimestamp()))
        year, month, day, hour, minute, second, millisecond = sensor_data.getDecodedTimestamp()
        self.timestamp = datetime(year, month, day, hour, minute, second, millisecond*1000, tzinfo=timezone.utc).timestamp()*1000*1000
        #print("Data Timestamp [ms] =", self.timestamp)
        if sensor_data.hasDepthMap:
            # Depth map visualization
            frame_number = sensor_data.depthmap.frameNumber
            print("Data contains depth map data:")
            print("=== Frame number: {}".format(frame_number))
            if image_plot:
                self.Visualizer.plot_frame_real_time(self.device_type, sensor_data)
            if save_image:
                print("=== Write PNG file: Frame number: {}".format(frame_number))
                writeFrame(self.device_type, sensor_data,os.path.join(img_dir, ""))
            # Point cloud generation and visualization
            print("time to read frame: ", time.time() - start_time)
            start_time = time.time()

            if PLY:
                self.point_cloud_ply, self.dist_data = convertToPointCloud(
                    sensor_data.depthmap.distance, 
                    sensor_data.depthmap.intensity,
                    sensor_data.depthmap.confidence, 
                    sensor_data.cameraParams, 
                    sensor_data.xmlParser.stereo
                    )
                if save_point_cloud:
                    writePointCloudToPLY(
                        os.path.join(pcl_dir, 
                                     f"world_coordinates{frame_number}.ply"), 
                                     self.point_cloud_ply)
            elif PCD:
                self.point_cloud_pcd = convertToPointCloudOptimized(
                    sensor_data.depthmap.distance,
                    sensor_data.depthmap.confidence,
                    sensor_data.cameraParams,
                    isStereo=sensor_data.xmlParser.stereo)
                if save_point_cloud:
                    writePointCloudToPCD(
                        os.path.join(pcl_dir, 
                                     f"world_coordinates{frame_number}.pcd"), 
                                     self.point_cloud_pcd.reshape(-1, 
                                                                  self.point_cloud_pcd.shape[-1]))
           
            print(f"convertToPointCloud took: {(time.time() - start_time):.3}s")
            # print("confidence: ", sensor_data.depthmap.confidence)
            # #Filter the point cloud to keep only points within the specified rectangular region
            # print("point cloud raw data: ", dist_data)
            if point_cloud_plot:
                if PLY:
                    self.Visualizer.type = "ply"
                    self.Visualizer.point_cloud = self.point_cloud_ply
                elif PCD:
                    self.Visualizer.type = "pcd"
                    self.Visualizer.point_cloud = self.point_cloud_pcd
                self.Visualizer.visualize_point_cloud_with_open3d()
 
    def get_contours(self, PLY=False, PCD=False, plot=False, eps=15, min_samples=10):   
        """
        Returns the contour of the cardboard box.
        """
        
        self.Visualizer.depth_range = self.depth_range
        if PLY:
            self.Visualizer.point_cloud = self.point_cloud_ply
        elif PCD:
            self.Visualizer.point_cloud = self.point_cloud_pcd  

        #start_time = time.time()
        if plot:
             self.Visualizer.process_and_visualize_point_cloud(visualize=True, eps=eps, min_samples=min_samples)
        else:
            self.Visualizer.process_and_visualize_point_cloud(visualize=False, eps=eps, min_samples=min_samples)
        
        self.contours = self.Visualizer.contour
        self.centroid = self.Visualizer.centroid
        self.color = self.Visualizer.color
        #print(f"Contour processing took: {(time.time() - start_time):.3}s")
    
    def get_depth_range(self, PLY=False, PCD=False):
        """
        Returns the depth range of the point cloud.
        """
        if PLY:
            self.Visualizer.point_cloud = self.point_cloud_ply
            self.Visualizer.type = "ply"
            self.Visualizer.visualize_point_cloud_with_open3d()
        elif PCD:
            self.Visualizer.point_cloud = self.point_cloud_pcd
            self.Visualizer.type = "pcd"
            self.Visualizer.visualize_point_cloud_with_open3d()
        point_1 = self.Visualizer.pick_points[0]
        point_2 = self.Visualizer.pick_points[1]
        depth_range = (point_1[3], point_2[3])
        return point_1, point_2, depth_range