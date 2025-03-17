//#include <application_base_modules/Camera.h>
#include <application_base_modules/PLCData.h>
#include <boost/interprocess/ipc/message_queue.hpp>
#include <future>
#include <iostream>
#include <chrono>
#include <iomanip>
#include <sstream>

// Existing code
std::stringstream ss;
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
          auto time_start = std::chrono::high_resolution_clock::now();
          std::vector<char> buffer(4096); // Buffer to receive data
          std::string json_data;
          unsigned int priority;
          bip::message_queue::size_type recvd_size;
		 
          if (mq.try_receive(buffer.data(), buffer.size(), recvd_size, priority))
          {
              std::string json_data(buffer.data(), recvd_size);
			  std::cout << "Received data: " << json_data << std::endl;
              opc.dataProcessing(json_data);
			  printTimestamp();
           }
          bool OPC_Connect = opc.readData(connect_node_id);
		  std::cout << "OPC_Connect: " << OPC_Connect << std::endl;
          op_connect_queue.send(&OPC_Connect, sizeof(OPC_Connect), 0);
          if (OPC_Connect)
          {
            std::cout << "CODESYS is ready to receive data." << std::endl;
          }
          else
          {
            std::cout << "CODESYS not ready to receive data." << std::endl;
          }
          auto time_end = std::chrono::high_resolution_clock::now();
          std::chrono::duration<double> elapsed_seconds = time_end - time_start;
          std::cout << "Communication processing time: " << elapsed_seconds.count() << "s\n";
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

  void printTimestamp()
  {
      auto now = std::chrono::system_clock::now();
      auto in_time_t = std::chrono::system_clock::to_time_t(now);
      auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()) % 1000;

      std::stringstream ss;
      ss << std::put_time(std::localtime(&in_time_t), "[%Y-%m-%d %H:%M:%S");
      ss << '.' << std::setw(3) << std::setfill('0') << ms.count() << ']';
      std::cout << ss.str() << std::endl;
  }
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
