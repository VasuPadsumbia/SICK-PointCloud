#include <application_base_modules/PLCData.h>
#include <chrono>
#include <cinttypes>
#include <cstdint>
#include <cstdio>
#include <iostream>
#include <memory>
#include <random>
#include <sstream>
#include <string>
#include <thread>
#include <open62541/client_highlevel.h>
#include <open62541/client_config_default.h>

int main(int argc, char* argv[])
{
  // Default values:
  std::string deviceIpAddr("10.11.36.166");
  int         port = 4841;
  std::string base_node_id("|var|CODESYS Control RTE x64 .Application");
  std::string connect_node_id = base_node_id + ".GVL.OPC_Connect";
  std::string newpos_node_id  = base_node_id + ".OPC_UA.newPos";
  std::string time_node_id    = base_node_id + ".OPC_UA.TimeStamp";

  // Create PLCData instance
  PLCData plc_data(deviceIpAddr, port);

  // Connect to OPC UA server
  plc_data.connect();
 
  // Create random number generator
  std::random_device               rd;
  std::mt19937                     gen(rd());
  std::uniform_real_distribution<> dis_x(300.0, 900.0);
  std::uniform_real_distribution<> dis_y(-0.5, 136.5);
  std::uniform_real_distribution<> dis_z(-47, -15);
  std::uniform_int_distribution<>  dis_color(1, 4);
  //std::uniform_int_distribution<>  dis_id(1, 1000);

  int current_id = 1;
  UA_Variant value;
  UA_Variant_init(&value);
  try
  {
    while (true)
    {
      // Check if the OPC server is connected and ready to receive data
      //std::string opc_connect_str = plc_data->readData(connect_node_id);
      bool        OPC_Connect     = plc_data.readData(connect_node_id);
       //(opc_connect_str == "true");
  
      if (!OPC_Connect)
      {
        std::cout << "CODESYS not ready to receive data." << std::endl;
        std::this_thread::sleep_for(std::chrono::seconds(1));
      }
      else
      {
        // Create values for the ST_Pos structure
        UA_Double x_pos = dis_x(gen);
        UA_Double y_pos  = dis_y(gen);
        UA_Double z_pos = -35;
        UA_Int32  color = dis_color(gen);
        UA_Int16  id     = current_id;
  
        // Get the current timestamp in nanoseconds
        auto     now             = std::chrono::system_clock::now();
        auto     now_ns          = std::chrono::time_point_cast<std::chrono::nanoseconds>(now);
        auto     epoch           = now_ns.time_since_epoch();
        UA_UInt64 timestamp_lword = static_cast<uint64_t>(epoch.count());
  
        std::cout << "timestamp: " << timestamp_lword << std::endl;
        //plc_data.TypeCheck(time_node_id);
        // Update the timestamp node
        plc_data.writeData(time_node_id, timestamp_lword);
        std::cout << "Timestamp updated successfully." << std::endl;
        // Update each field of the ST_Pos structure individually
        plc_data.writeData(newpos_node_id + ".X_Pos", x_pos);
        plc_data.writeData(newpos_node_id + ".Y_Pos", y_pos);
        plc_data.writeData(newpos_node_id + ".Z_Pos", z_pos);
        plc_data.writeData(newpos_node_id + ".colour", color);
        plc_data.writeData(newpos_node_id + ".ID", id);
  
        std::cout << "ST_Pos structure updated successfully." << std::endl;
  
        std::this_thread::sleep_for(std::chrono::seconds(1));
        current_id++;
        if (current_id > 100)
          {
            current_id = 1;
          }
      }
    }
  }
  catch (const std::exception& e)
  {
    std::cerr << "An error occurred: " << e.what() << std::endl;
  }

  // Disconnect from OPC UA server
  plc_data.disconnect();

  return 0;
}