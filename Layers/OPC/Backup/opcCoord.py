from opcua import Client, ua
from datetime import datetime, timezone
import time, random

def update_node_value(client, node_id, value, variant_type):
    """
    Update the value of a node over OPC UA.

    Args:
        client (Client): The OPC UA client.
        node_id (str): The node ID of the variable.
        value: The value to update.
        variant_type: The variant type of the value.
    """
    node = client.get_node(node_id)
    print(f"Updating node {node_id} with value: {value}")
    node.set_value(ua.Variant(value, variant_type))

def get_node_value(client, node_id):
    """
    Get the value of a node over OPC UA.

    Args:
        client (Client): The OPC UA client.
        node_id (str): The node ID of the variable.

    Returns:
        object: The value of the node.
    """
    node = client.get_node(node_id)
    value = node.get_value()
    print(f"Value of node {node_id}: {value}")
    return value

def format_timestamp(dt):
    """
    Format the timestamp in the required format T#49D17H2M47S295MS.

    Args:
        dt (datetime): The datetime object to format.

    Returns:
        str: The formatted timestamp.
    """
    epoch = datetime(1970, 1, 1, tzinfo=timezone.utc)
    delta = dt - epoch
    days = delta.days
    hours, remainder = divmod(delta.seconds, 3600)
    minutes, seconds = divmod(remainder, 60)
    milliseconds = delta.microseconds // 1000
    return f"T#{days}D{hours}H{minutes}M{seconds}S{milliseconds}MS"

def get_elapsed_time_in_milliseconds(start_time):
    """
    Get the elapsed time in milliseconds since the start time.

    Args:
        start_time (float): The start time in seconds since the epoch.

    Returns:
        int: The elapsed time in milliseconds.
    """
    current_time = time.time()
    elapsed_time = current_time - start_time
    milliseconds = int(elapsed_time * 1000)
    return milliseconds

def get_timestamp_in_milliseconds(dt):
    """
    Get the timestamp in milliseconds since the Unix epoch.

    Args:
        dt (datetime): The datetime object to convert.

    Returns:
        int: The timestamp in milliseconds.
    """
    epoch = datetime(1970, 1, 1, tzinfo=timezone.utc)
    delta = dt - epoch
    milliseconds = int(delta.total_seconds() * 1000)
    return milliseconds

def main():
    # OPC UA server URL
    url = "opc.tcp://localhost:4841"
    
    # Record the start time
    start_time = time.time()

    # Connect to the OPC UA server
    client = Client(url)
    try:
        client.connect()

        # Base node ID of the newPos variable in PROGRAM OPC_UA
        base_node_id = "ns=4;s=|var|CODESYS Control RTE x64 .Application.OPC_UA"
        connect_node_id = f"{base_node_id}.OPC_Connect"
        newpos_node_id = f"{base_node_id}.newPos"
        time_node_id = f"{base_node_id}.timeStamp"
        # Initialize ID
        current_id = 1
        try:
            while True:
                # Check if the OPC server is connected and ready to receive data
                OPC_Connect = get_node_value(client, connect_node_id)
                if not OPC_Connect:
                    print("CODESYS not ready to receive data.")
                    time.sleep(1)
                else:
                    # Create values for the ST_Pos structure
                    st_pos_values = {
                        "X_Pos": (random.uniform(300, 900), ua.VariantType.Double),
                        "Y_Pos": (random.uniform(-0.5, 136.5), ua.VariantType.Double),
                        "Z_Pos": (random.uniform(-47, -2), ua.VariantType.Double),
                        "colour": (random.randint(1, 4), ua.VariantType.Int32),
                        "ID": (current_id, ua.VariantType.Int16),
                    }
                    timestamp_lword = int(datetime.now().astimezone(timezone.utc).timestamp()*1000*1000*1000)
                    print ("timestamp: ", datetime.now().astimezone(timezone.utc))
                    update_node_value(client, time_node_id, timestamp_lword, ua.VariantType.UInt64)
                    # Update each field of the ST_Pos structure individually
                    for field, (value, variant_type) in st_pos_values.items():
                        node_id = f"{newpos_node_id}.{field}"
                        update_node_value(client, node_id, value, variant_type)

                    print("ST_Pos structure updated successfully.")
                    time.sleep(1)
                    current_id += 1
        except KeyboardInterrupt:
            print("Keyboard interrupt detected.")
        ## Create values for the ST_Pos structure
        #st_pos_values = {
        #    "X_Pos": (100.0, ua.VariantType.Double),
        #    "Y_Pos": (200.0, ua.VariantType.Double),
        #    "Z_Pos": (300.0, ua.VariantType.Double),
        #    #"timeStamp": (get_timestamp_in_milliseconds(datetime.now(timezone.utc)), ua.VariantType.Int64),
        #    "timeStamp": (get_elapsed_time_in_milliseconds(start_time), ua.VariantType.Int64),
        #    "colour": (1, ua.VariantType.Int32),
        #    "ID": (1, ua.VariantType.Int16),
        #}
#
        ## Update each field of the ST_Pos structure individually
        #for field, (value, variant_type) in st_pos_values.items():
        #    node_id = f"{base_node_id}.{field}"
        #    update_node_value(client, node_id, value, variant_type)
#
        #print("ST_Pos structure updated successfully.")
    except KeyboardInterrupt:
        print("Keyboard interrupt detected.")
    except PermissionError as pe:
        print(f"Permission error: {pe}")
    except Exception as e:
        print(f"An error occurred: {e}")

    finally:
        try:
            if client.uaclient._uasocket is not None:
                client.disconnect()
        except Exception as e:
            print(f"An error occurred during disconnection: {e}")

if __name__ == "__main__":
    main()