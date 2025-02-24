#ifndef PLCDATA_H
#define PLCDATA_H

#include <open62541/client.h>
#include <string>
#include <nlohmann/json.hpp>

class PLCData
{
public:
  PLCData(const std::string& ip_address, int port);
  ~PLCData();

  void        connect();
  bool readData(const std::string& address);
  void        writeData(const std::string& address, UA_Int32 data);
  void        writeData(const std::string& address, const std::string& data);
  void        writeData(const std::string& address, UA_UInt64 data);
  void        writeData(const std::string& address, UA_Double data);
  void        writeData(const std::string& address, UA_Int16 data);
  void        dataProcessing(const std::string& json_data);

  void        disconnect();
  void        TypeCheck(const std::string& address);
  bool        isConnected;

private:
  std::string base_node_id = "|var|CODESYS Control RTE x64 .Application.OPC_UA";
  std::string connect_node_id = base_node_id + ".OPC_Connect";
  std::string newpos_node_id  = base_node_id + ".newPos";
  std::string time_node_id    = base_node_id + ".TimeStamp";

  std::string ip_address_;
  int         port_;
  UA_Client*  client_; // UA_Client is a type from open62541 library
};

#endif // PLCDATA_H
