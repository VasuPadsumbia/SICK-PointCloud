import time
from Layers.logger import Logger as log
from Layers.OPC.PLCData import OPCUAClient
from Layers.Camera.Camera import Camera
import threading

class Communication:
    def __init__(self) -> None:
        #Init OPC
        self.opc = OPCUAClient()
        self.stop_event = threading.Event()
        self.output_event = threading.Event()
        self.input_event = threading.Event()

    def worker(self, q1, q2, input_event, output_event, start_event=None):
        self.input_event = input_event
        self.output_event = output_event
        print("Communication worker started.")
        self.opc.connect() # Connect to the OPC UA server
        while not self.stop_event.is_set():
            #print("If stop event is set then break the loop")
            try:

                if self.opc.connected:
                    if not q1.empty():
                        json_data = q1.get()
                        print(json_data)
                        # Dictionary
                        position = (json_data['x'], json_data['y'], json_data['z'], json_data['colour'])
                        time_stamp = json_data['time_stamp']
                        # Dictionary
                        self.opc.data_processing(*position, time_stamp)
                        q1.task_done()
                        self.input_event.set()
                    OPC_Connected = self.opc.get_node_value(self.opc.connect_node_id)
                    q2.put(OPC_Connected)
                    #print("waiting for camera to accept the data")
                    # Wait for camera to accept the data
                    if self.output_event.wait(timeout=1):
                        print("Camera accepted the data")
                        self.output_event.clear()
                else:
                    self.opc.connect()  # Reconnect to the OPC UA server
            except KeyboardInterrupt:
                print("Keyboard interrupt detected. Exiting loop.")
                log().getLogger("Communication").info("Keyboard interrupt detected. Exiting loop.")
                self.opc.disconnect()
                break
            except Exception as e:
                print(f"An error occurred: {e}")
                log().getLogger("Communication").error(e)
                self.opc.disconnect()
                break
        print("Communication worker stopped.")

    def stop_worker(self):
        self.stop_event.set()
        self.opc.disconnect()
        self.output_event.clear()
        self.input_event.clear()


class PositionData:
    def __init__(self) -> None:
        #Init Camera
        self.camera = Camera()
        self.OPC_Connect = False
        self.stop_event = threading.Event()
        self.output_event = threading.Event()
        self.input_event = threading.Event()

    def worker(self, q1, q2, input_event, output_event, start_event=None, count=1):
        self.input_event = input_event
        self.output_event = output_event
        print("PositionData worker started.")
        while not self.stop_event.is_set():
            #print("Waiting for camera to accept the data")
            try:
                if not q2.empty():
                    self.OPC_Connect = q2.get()
                    print(self.OPC_Connect)
                    q2.task_done()
                    self.input_event.set()

                if self.OPC_Connect:
                    while count > 0:
                        # Capture image
                        # If want to set roi then pass the roi as parameter however it takes time for processing
                        # self.camera.set_roi(left, right, top, botom)
                        #self.camera.set_roi(210, 535, 151, 384)
                        # If want to set integration time then pass the integration time as parameter
                        # self.camera.set_integration_time(integration_time, integration_time_color)
                        start_time = time.time()
                        self.camera.initialize_stream()
                        print(f"Camera initialization time: {time.time() - start_time}")
                        try:    
                            start_time = time.time()
                            self.camera.process_frame(PLY=True)
                            print(f"Camera processing time: {time.time() - start_time}")
                            """
                            set the depth range automatically by 
                            finding the minimum and maximum depth values by pressing
                            Shift + Left Mouse Click on the point cloud visualization window
                            """
                            #point_1, point_2, depth_range = self.camera.get_depth_range()
                            #print(f"Depth range processing time: {time.time() - start_time}")
                            #print("Depth range: ", depth_range)
                            #print("Point 1: ", point_1)
                            #print("Point 2: ", point_2)
                            #depth_range = point_1[3]-3, point_1[3]+3
                            # If want to set depth range manually then pass the depth range as parameter
                            self.camera.set_depth_range((548,550))
                            start_time = time.time()
                            self.camera.get_contours(PLY=True, plot=False, eps=25, min_samples=2) 
                            print(f"Contour processing time: {time.time() - start_time}")
                            #print("Contour: ", self.camera.contours)
                            print("Centroid: ", self.camera.centroid)
                            print("Color: ", self.camera.color)
                            # Get position
                            position = self.camera.centroid
                            color = self.camera.color                           
                        except Exception as e:
                            print(e)
                            log().getLogger("PositionData::Camera").error(e)
                        finally:
                            self.camera.cleanup()
                        # Dictionary
                        data = {
                            "x": position[0],
                            "y": position[1],
                            "z": position[2],
                            "colour": color,
                            "time_stamp": self.camera.timestamp
                        }
                        q1.put(data)
                        if self.output_event.wait(timeout=1):
                            self.output_event.clear()
                        count -= 1
            except KeyboardInterrupt:
                print("Keyboard interrupt detected. Exiting loop.")
                log().getLogger("PositionData").info("Keyboard interrupt detected. Exiting loop.")
                break
            except Exception as e:
                print(f"An error occurred: {e}")
                log().getLogger("PositionData").error(e)
                break
            
        print("PositionData worker stopped.")    

    def stop_worker(self):
        self.stop_event.set()
        self.output_event.clear()
        self.input_event.clear()