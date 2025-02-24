#include "PLCData.h"
#include <open62541/client_config_default.h>
#include <open62541/client_highlevel.h>
#include <stdexcept>
#include <iostream>
#include <open62541/config.h> // Add this include

PLCData::PLCData(const std::string& ip_address, int port) : ip_address_(ip_address), port_(port), client_(nullptr)
{
//connect();
}

PLCData::~PLCData()
{
  disconnect();
}

void PLCData::connect()
{
  client_ = UA_Client_new();
  UA_ClientConfig* config = UA_Client_getConfig(client_);
  UA_ClientConfig_setDefault(config);

  // Set username and password for authentication
  std::string username = "Somic";
  std::string password = "Somic";

  std::string endpoint = "opc.tcp://" + ip_address_ + ":" + std::to_string(port_);
  std::cout << "username: " << username.c_str() << std::endl;
  std::cout << "password: " << password.c_str() << std::endl;

  UA_StatusCode retval = UA_Client_connectUsername(client_, endpoint.c_str(), username.c_str(), password.c_str());
  if (retval != UA_STATUSCODE_GOOD)
  {
    UA_Client_delete(client_);
    client_ = nullptr;
    std::cerr << "Failed to connect to the OPC UA server. StatusCode: " << retval << std::endl;
  }
  else
  {
    std::cout << "Connected to the OPC UA server" << std::endl;
    isConnected = true;
  }
}

bool PLCData::readData(const std::string& address)
{
  std::cout << "Reading data from address: " << address << std::endl;

  if (!client_)
  {
    isConnected = false;
    throw std::runtime_error("Client is not connected");
  }

  UA_Variant value;
  UA_Variant_init(&value);
  UA_NodeId nodeId = UA_NODEID_STRING_ALLOC(4, address.c_str());

  UA_StatusCode retval = UA_Client_readValueAttribute(client_, nodeId, &value);

  if (retval != UA_STATUSCODE_GOOD)
  {
    UA_Variant_clear(&value);
    UA_NodeId_clear(&nodeId);
    return false; // ✅ Ensure a valid return value on failure
  }

  bool result = false;
  if (UA_Variant_hasScalarType(&value, &UA_TYPES[UA_TYPES_BOOLEAN]))
  {
    result = *(UA_Boolean*)value.data;
  }

  UA_Variant_clear(&value);
  UA_NodeId_clear(&nodeId);

  return result;
}

// Double data type
void PLCData::writeData(const std::string& address, UA_Double data)
{
  if (!client_)
  {
    isConnected = false;
    throw std::runtime_error("Client is not connected");
  }

  UA_Variant value;
  UA_Variant_init(&value);

  UA_Variant_setScalar(&value, &data, &UA_TYPES[UA_TYPES_DOUBLE]);

  UA_NodeId     nodeId = UA_NODEID_STRING_ALLOC(4, address.c_str());
  UA_StatusCode retval = UA_Client_writeValueAttribute(client_, nodeId, &value);

  if (retval != UA_STATUSCODE_GOOD)
  {
    UA_NodeId_clear(&nodeId);
    throw std::runtime_error("Failed to write data to the OPC UA server");
  }

  std::cout << "Data written successfully for address: " << address << std::endl;

  if (value.data)
  {
    std::cout << "UA Variant value: " << *(UA_Double*)value.data << std::endl;
  }

  //UA_Variant_clear(&value);
  UA_NodeId_clear(&nodeId);
}

// String data type
void PLCData::writeData(const std::string& address, const std::string& data)
{
  if (!client_)
  {
    isConnected = false;
    throw std::runtime_error("Client is not connected");
  }

  UA_Variant value;
  UA_Variant_init(&value);

  UA_String uaString = UA_STRING_ALLOC(data.c_str());
  UA_Variant_setScalar(&value, &uaString, &UA_TYPES[UA_TYPES_STRING]);

  UA_NodeId     nodeId = UA_NODEID_STRING_ALLOC(4, address.c_str());
  UA_StatusCode retval = UA_Client_writeValueAttribute(client_, nodeId, &value);

  if (retval != UA_STATUSCODE_GOOD)
  {
    UA_String_clear(&uaString);
    UA_NodeId_clear(&nodeId);
    throw std::runtime_error("Failed to write data to the OPC UA server");
  }

  std::cout << "Data written successfully for address: " << address << std::endl;

  //UA_Variant_clear(&value);
  UA_String_clear(&uaString);
  UA_NodeId_clear(&nodeId);
}

void PLCData::writeData(const std::string& address, UA_UInt64 data)
{
  if (!client_)
  {
    isConnected = false;
    throw std::runtime_error("Client is not connected");
  }

  UA_Variant value;
  UA_Variant_init(&value);
  UA_NodeId nodeId = UA_NODEID_STRING_ALLOC(4, address.c_str());

  UA_Variant_setScalar(&value, &data, &UA_TYPES[UA_TYPES_UINT64]);

  UA_StatusCode retval = UA_Client_writeValueAttribute(client_, nodeId, &value);

  if (retval != UA_STATUSCODE_GOOD)
  {
    UA_Variant_clear(&value); // ✅ Free variant first
    UA_NodeId_clear(&nodeId); // ✅ Free node ID after variant
    throw std::runtime_error("Failed to write data to the OPC UA server");
  }

  std::cout << "Data written successfully for address: " << address << std::endl;

  //UA_Variant_clear(&value); // ✅ Only clear once
  UA_NodeId_clear(&nodeId);
}



// Int32 data type
void PLCData::writeData(const std::string& address, UA_Int32 data)
{
  if (!client_)
  {
    isConnected = false;
    throw std::runtime_error("Client is not connected");
  }

  UA_Variant value;
  UA_Variant_init(&value);

  UA_Variant_setScalar(&value, &data, &UA_TYPES[UA_TYPES_INT32]);

  UA_NodeId     nodeId = UA_NODEID_STRING_ALLOC(4, address.c_str());
  UA_StatusCode retval = UA_Client_writeValueAttribute(client_, nodeId, &value);

  if (retval != UA_STATUSCODE_GOOD)
  {
    UA_NodeId_clear(&nodeId);
    throw std::runtime_error("Failed to write data to the OPC UA server");
  }

  std::cout << "Data written successfully for address: " << address << std::endl;

  if (value.data)
  {
    std::cout << "UA Variant value: " << *(UA_Int32*)value.data << std::endl;
  }

  //UA_Variant_clear(&value);
  UA_NodeId_clear(&nodeId);
}


// Int16 data type
void PLCData::writeData(const std::string& address, UA_Int16 data)
{
  if (!client_)
  {
    isConnected = false;
    throw std::runtime_error("Client is not connected");
  }

  UA_Variant value;
  UA_Variant_init(&value);

  UA_Variant_setScalar(&value, &data, &UA_TYPES[UA_TYPES_INT16]);

  UA_NodeId     nodeId = UA_NODEID_STRING_ALLOC(4, address.c_str());
  UA_StatusCode retval = UA_Client_writeValueAttribute(client_, nodeId, &value);

  if (retval != UA_STATUSCODE_GOOD)
  {
    UA_NodeId_clear(&nodeId);
    throw std::runtime_error("Failed to write data to the OPC UA server");
  }

  std::cout << "Data written successfully for address: " << address << std::endl;

  if (value.data)
  {
    std::cout << "UA Variant value: " << *(UA_Int16*)value.data << std::endl;
  }

  //UA_Variant_clear(&value);
  UA_NodeId_clear(&nodeId);
}

  void PLCData::disconnect()
{
  if (client_)
  {
    isConnected = false;
    UA_Client_disconnect(client_);
    UA_Client_delete(client_);
    client_ = nullptr;
  }
}

void PLCData::TypeCheck(const std::string& address)
{
  if (!client_)
  {
    isConnected = false;
    throw std::runtime_error("Client is not connected");
  }
  UA_Variant value;
  UA_Variant_init(&value);
  UA_NodeId     nodeId = UA_NODEID_STRING(4, const_cast<char*>(address.c_str()));
  UA_StatusCode retval = UA_Client_readValueAttribute(client_, nodeId, &value);
  if (retval != UA_STATUSCODE_GOOD)
  {
    throw std::runtime_error("Failed to read data from the OPC UA server");
  }
  else
  {
    std::cout << "Node expects type: " << value.type->typeName << std::endl;
  }
  UA_Variant_clear(&value);
}

void PLCData::dataProcessing(const std::string& json_data)
{
  try
  {
    auto json_obj = nlohmann::json::parse(json_data);

    // Extract values from JSON
    double      x_pos     = json_obj.at("x").get<double>();
    double      y_pos     = json_obj.at("y").get<double>();
    double      z_pos     = json_obj.at("z").get<double>();
    int         color     = json_obj.at("color").get<std::int32_t>();
    int         id        = json_obj.at("id").get<std::int16_t>();
    std::uint64_t timestamp = json_obj.at("timestamp").get<std::uint64_t>();

    // Write values to OPC UA server
    writeData(newpos_node_id + ".X_Pos", x_pos);
    writeData(newpos_node_id + ".Y_Pos", y_pos);
    writeData(newpos_node_id + ".Z_Pos", z_pos);
    writeData(newpos_node_id + ".colour", color);
    writeData(newpos_node_id + ".ID", id);
    writeData(time_node_id, timestamp);
  }
  catch (const std::exception& e)
  {
    std::cerr << "Failed to process JSON data: " << e.what() << std::endl;
  }
}