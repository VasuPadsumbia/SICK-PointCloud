#include <application_base_modules/Camera.h>
#include <application_base_modules/PLCData.h>
#include <boost/interprocess/ipc/message_queue.hpp>
#include <future>
#include <iostream>

namespace bip = boost::interprocess;

class Communication
{
public:
  Communication() : stop_event(false), opc(deviceIpAddr, port)
  {
    std::cout << "Communication instance created." << std::endl;
  }

  void worker(bip::message_queue& mq,
              bip::message_queue& op_connect_queue,
              std::promise<void>& exit_signal,
              std::shared_future<void>& future_obj)
  {
    std::cout << "Communication worker started." << std::endl;
    opc.connect();
    while (future_obj.wait_for(std::chrono::milliseconds(1)) == std::future_status::timeout)
    {
      try
      {
        if (opc.isConnected)
        {
          std::string json_data;
          unsigned int priority;
          bip::message_queue::size_type recvd_size;
          if (mq.try_receive(&json_data, sizeof(json_data), recvd_size, priority))
          {
            opc.dataProcessing(json_data);
          }
          bool OPC_Connect = opc.readData(connect_node_id);
          op_connect_queue.send(&OPC_Connect, sizeof(OPC_Connect), 0);
          if (OPC_Connect)
          {
            std::cout << "CODESYS is ready to receive data." << std::endl;
          }
          else
          {
            std::cout << "CODESYS not ready to receive data." << std::endl;
          }
        }
        else
        {
          try
          {
            opc.connect();
          }
          catch (const std::exception& e)
          {
            std::cerr << "OPC UA Connection failed: " << e.what() << std::endl;
          }
        }
      }
      catch (const std::exception& e)
      {
        std::cerr << "An error occurred: " << e.what() << std::endl;
        opc.disconnect();
        break;
      }
    }
    std::cout << "Communication worker stopped." << std::endl;
  }

  void stopWorker()
  {
    stop_event.store(true);
    opc.disconnect();
  }

private:
  std::atomic<bool> stop_event;
  std::string deviceIpAddr = "10.11.36.166";
  int port = 4841;
  std::string base_node_id = "|var|CODESYS Control RTE x64 .Application";
  std::string connect_node_id = base_node_id + ".GVL.OPC_Connect";
  std::string newpos_node_id = base_node_id + ".OPC_UA.newPos";
  std::string time_node_id = base_node_id + ".OPC_UA.TimeStamp";
  PLCData opc;
};

int main()
{
  std::promise<void> exit_signal;
  std::shared_future<void> future_obj = exit_signal.get_future();
  bip::message_queue mq(bip::open_only, "message_queue");
  bip::message_queue       op_connect_queue(bip::open_only, "op_connect_queue");

  Communication comm;
  comm.worker(mq, op_connect_queue, exit_signal, future_obj);

  return 0;
}
