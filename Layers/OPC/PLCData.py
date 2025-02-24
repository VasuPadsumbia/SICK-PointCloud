from opcua import Client, ua
import time, random

class Singleton_meta(type):
    _instances = {}
    def __call__(cls, *args, **kwargs):
        if cls not in cls._instances:
            cls._instances[cls] = super(Singleton_meta, cls).__call__(*args, **kwargs)
        return cls._instances[cls]

class OPCUAClient( metaclass=Singleton_meta ):

    def __init__(self, url="opc.tcp://localhost:4841", 
                 x_range=(300, 900), y_range=(-0.5, 136.5), 
                 z_range=(-47, -2), colour_range=(1, 4), 
                 id_start=1):
        if not hasattr(self, 'initialized'):  # Ensure __init__ is only called once
            self.url = url
            self.client = Client(url)
            self.current_id = id_start
            self.start_time = time.time()
            self.x_range = x_range
            self.y_range = y_range
            self.z_range = z_range
            self.colour_range = colour_range
            self.initialized = True
            self.base_node_id = "ns=4;s=|var|CODESYS Control RTE x64 .Application.OPC_UA.newPos"
            self.connect_node_id = "ns=4;s=|var|CODESYS Control RTE x64 .Application.OPC_UA.OPC_Connect"
            self.connected = False
    def connect(self):
        try:
            self.client.connect()
            print("Connected to OPC UA server.")
            self.connected = True
        except Exception as e:
            print(f"Failed to connect to OPC UA server: {e}")
            self.connected = False

    def disconnect(self):
        try:
            if self.client.uaclient._uasocket is not None:
                self.client.disconnect()
                print("Disconnected from OPC UA server.")
                self.connected = False
        except Exception as e:
            print(f"An error occurred during disconnection: {e}")

    @property
    def is_connected(self):
        """
        Check if the client is connected to the OPC UA server.
        
        Returns:
            bool: True if connected, False otherwise.
        """
        try:
            # Attempt to read a known node's value to verify connection
            node = self.client.get_node("ns=0;i=2258")  # Server Status CurrentTime
            node.get_value()
            return True
        except Exception:
            return False
        
    def update_node_value(self, node_id, value, variant_type):
        try:
            node = self.client.get_node(node_id)
            print(f"Updating node {node_id} with value: {value}")
            node.set_value(ua.Variant(value, variant_type))
        except Exception as e:
            print(f"Failed to update node {node_id}: {e}")

    def get_node_value(self, node_id):
        try:
            node = self.client.get_node(node_id)
            value = node.get_value()
            print(f"Value of node {node_id}: {value}")
            return value
        except Exception as e:
            print(f"Failed to get value of node {node_id}: {e}")
            return None

    def get_elapsed_time_in_milliseconds(self):
        milliseconds = int((time.time() - self.start_time) * 1000)
        return milliseconds
        
    def data_processing(self, x_pos, y_pos, z_pos, colour, time_stamp): 
        """
        This method updates the ST_Pos structure in the CODESYS OPC UA server.
        """
        try:
            OPC_Connect = self.get_node_value(self.connect_node_id)
            if not OPC_Connect:
                print("CODESYS not ready to receive data.")
                time.sleep(1)
            else:
                st_pos_values = {
                    "X_Pos": (x_pos, ua.VariantType.Double),
                    "Y_Pos": (y_pos , ua.VariantType.Double),
                    "Z_Pos": (z_pos, ua.VariantType.Double),
                    "timeStamp": (time_stamp, ua.VariantType.Int64),
                    "colour": (colour, ua.VariantType.Int32),
                    "ID": (self.current_id, ua.VariantType.Int16),
                }

                for field, (value, variant_type) in st_pos_values.items():
                    node_id = f"{self.base_node_id}.{field}"
                    self.update_node_value(node_id, value, variant_type)
                print("ST_Pos structure updated successfully.")
                time.sleep(1)
                self.current_id += 1
        
        except KeyboardInterrupt:
            print("Keyboard interrupt detected.")
        
        except PermissionError as pe:
            print(f"Permission error: {pe}")
        
        except Exception as e:
            print(f"An error occurred: {e}")
        
        finally:
            self.disconnect()

    def random_generator(self):
        try:
            while True:
                OPC_Connect = self.get_node_value(self.connect_node_id)
                if not OPC_Connect:
                    print("CODESYS not ready to receive data.")
                    time.sleep(1)
                else:
                    st_pos_values = {
                        "X_Pos": (random.uniform(*self.x_range), ua.VariantType.Double),
                        "Y_Pos": (random.uniform(*self.y_range), ua.VariantType.Double),
                        "Z_Pos": (random.uniform(*self.z_range), ua.VariantType.Double),
                        "timeStamp": (self.get_elapsed_time_in_milliseconds(), ua.VariantType.Int64),
                        "colour": (random.randint(*self.colour_range), ua.VariantType.Int32),
                        "ID": (self.current_id, ua.VariantType.Int16),
                    }

                    for field, (value, variant_type) in st_pos_values.items():
                        node_id = f"{self.base_node_id}.{field}"
                        self.update_node_value(node_id, value, variant_type)

                    print("ST_Pos structure updated successfully.")
                    time.sleep(1)
                    self.current_id += 1

        except KeyboardInterrupt:
            print("Keyboard interrupt detected.")
        except PermissionError as pe:
            print(f"Permission error: {pe}")
        except Exception as e:
            print(f"An error occurred: {e}")
        finally:
            self.disconnect()

#def run_opcua_client(url, x_range, y_range, z_range, colour_range, id_start):
def run_opcua_client():
    #opcua_client = OPCUAClient(url, x_range, y_range, z_range, colour_range, id_start)
    opcua_client = OPCUAClient()
    opcua_client.connect()
    opcua_client.random_generator()
    #opcua_client.data_processing(0, 0, 0, 0)

if __name__ == "__main__":
    #url = input("Enter OPC UA server URL: ")
    #x_range = (int(input("Enter X range start: ")), int(input("Enter X range end: ")))
    #y_range = (float(input("Enter Y range start: ")), float(input("Enter Y range end: ")))
    #z_range = (float(input("Enter Z range start: ")), float(input("Enter Z range end: ")))
    #colour_range = (int(input("Enter colour range start: ")), int(input("Enter colour range end: ")))
    #id_start = int(input("Enter starting ID: "))
    #run_opcua_client(url, x_range, y_range, z_range, colour_range, id_start)
    run_opcua_client()
    """  """