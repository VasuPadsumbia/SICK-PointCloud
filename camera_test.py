from Layers.Camera.Camera import Camera
from Layers.Camera.features import PointCloudProcessor
import time
from concurrent.futures import ThreadPoolExecutor
import datetime

class CameraConfiguration:

    def __init__(self) -> None:
        self.camera = Camera()
        #self.point_cloud_processor = PointCloudProcessor()

    def get_colour(self, color):
        """
        Get the colour of the cardboard box at the given point.

        Args:
            point (tuple): X, Y, Z coordinates of the point.

        Returns:
            tuple: R, G, B colour values.
        """
        r, g, b = color
        if r > g and r > b:
            if g > 100 and b < 100:
                return 4  # Yellow
            return 1  # Red
        elif g > r and g > b:
            return 2  # Green
        elif b > r and b > g:
            return 3  # Blue
        return 0  # Undefined
    
    def worker(self):
        print("CameraConfiguration worker started.")
        try:
            #self.camera.set_roi(210, 535, 151, 384)
            #self.camera.set_integration_time(1250, 5000)
            start_time_main = time.time()
            self.camera.initialize_stream()
            print(f"Camera initialization time: {time.time() - start_time_main}")
            start_time = time.time()
            self.camera.process_frame(PLY=True)
            print(f"Camera processing time: {time.time() - start_time}")
            self.camera.set_depth_range((548,550))
            self.camera.get_contours(PLY=True, plot=True, eps=25, min_samples=2) 
            print(f"Total time: {time.time() - start_time_main}")
            #print("Contour: ", self.camera.contours)
            print("Centroid: ", self.camera.centroid)
            print("Color: ", self.camera.color)
            print("Colour catagory: ", self.get_colour(self.camera.color))
        except KeyboardInterrupt:
            print("Keyboard interrupt detected. Exiting loop.")
            
        except Exception as e:
            print(f"An error occurred: {e}")
        finally:
            self.camera.cleanup()
        print("CameraConfiguration worker stopped.")

if __name__ == "__main__":
    camera_configuration = CameraConfiguration()
    camera_configuration.worker()
    print("datetime: ", datetime.datetime.now().timestamp())
    #print("type: ", int(datetime.datetime.now()))