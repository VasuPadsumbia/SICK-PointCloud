#include <future>
#include <iostream>
#include <thread>
//#include "PositionData.cpp"
//#include "Communication.cpp"
#include <boost/process.hpp>
#include <boost/interprocess/ipc/message_queue.hpp>
#include <boost/interprocess/shared_memory_object.hpp>
#include <boost/interprocess/mapped_region.hpp>

/*class Communication
{
public:
  Communication() : stop_event(false), opc(deviceIpAddr, port) // Initialize opc with deviceIpAddr and port
  {
    std::cout << "Communication instance created." << std::endl;
  }

  void worker(bip::message_queue& mq,
              std::promise<void>&        exit_signal,
              std::shared_future<void>&  future_obj)
              //std::shared_future<std::string>& data_future,
              //std::promise<bool>&        connect_promise)
  {
    std::cout << "Communication worker started." << std::endl;
    opc.connect(); // Connect to the OPC UA server
    while (future_obj.wait_for(std::chrono::milliseconds(1)) == std::future_status::timeout)
    {
      try
      {
        if (opc.isConnected)
        {
          std::string                   json_data;
          unsigned int                  priority;
          bip::message_queue::size_type recvd_size;
          if (mq.try_receive(&json_data, sizeof(json_data), recvd_size, priority))
          {
            opc.dataProcessing(json_data);
          }
            //if (data_future.valid() && data_future.wait_for(std::chrono::milliseconds(1)) == std::future_status::ready)
            //{
            //  std::string json_data = data_future.get();
            //  opc.dataProcessing(json_data);
            //}
          bool OPC_Connect = opc.readData(connect_node_id);
          if (OPC_Connect)
          {
            std::cout << "CODESYS is ready to receive data." << std::endl;
            //connect_promise.set_value(OPC_Connect);
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
  // Default values:
  std::string deviceIpAddr = "10.11.36.166";
  int         port = 4841;
  std::string base_node_id = "|var|CODESYS Control RTE x64 .Application.OPC_UA";
  std::string connect_node_id = base_node_id + ".OPC_Connect";
  std::string newpos_node_id  = base_node_id + ".newPos";
  std::string time_node_id    = base_node_id + ".TimeStamp";
  PLCData     opc;
  // Create values for the ST_Pos structure
  UA_Double x_pos;
  UA_Double y_pos;
  UA_Double z_pos;
  UA_Int32  color;
  UA_Int16  id;
  
};

class PositionData
{
public:
  PositionData() : stop_event(false), OPC_Connect(false)
  {
    
  }
  pcl::PointCloud<pcl::PointXYZ> contours;
  Eigen::Vector3d centroid;
  Eigen::Vector3d point_color;
  void worker(bip::message_queue&        mq,
              std::promise<void>&        exit_signal,
              std::shared_future<void>&  future_obj)
              //std::promise<std::string>& data_promise,
              //std::shared_future<bool>&  connect_future)
  {
    std::cout << "PositionData worker started." << std::endl;
    while (future_obj.wait_for(std::chrono::milliseconds(1)) == std::future_status::timeout)
    {
      std::cout << "PositionData worker running." << std::endl;
      try
      {
        std::cout << "trying to connect with communication process." << std::endl;
        //if (connect_future.valid()
        //    && connect_future.wait_for(std::chrono::milliseconds(1)) == std::future_status::ready)
        //{
        //  std::cout << "Waiting for OPC connection..." << std::endl;
        //  OPC_Connect = connect_future.get();
        //}
        if (OPC_Connect)
        {
          if (&camera != nullptr)
          {
            std::cout << "Processing frame..." << std::endl;
            // camera.setROI(215, 535, 151, 384);
            camera.initializeStream();
            camera.processFrame(false, false, false, false, false, false);
            std::cout << "Setting depth range..." << std::endl;
            // camera.setDepthRange(std::make_tuple(0.548, 0.550));
            camera.setDepthRange(std::make_tuple(0.555, 0.568));
            std::tie(contours, centroid, point_color) = camera.getContours();
            current_id                                = current_id + 1;
            // create a JSON object
            nlohmann::json json_obj;
            json_obj["x"]         = centroid(0);
            json_obj["y"]         = centroid(1);
            json_obj["z"]         = centroid(2);
            json_obj["color"]     = point_color;
            json_obj["id"]        = current_id;
            json_obj["timestamp"] = camera.getTimestampMS();

            // convert JSON object to string
            std::string json_data = json_obj.dump();
            mq.send(json_data.c_str(), json_data.size(), 0);
            //data_promise.set_value(json_data);
          }
          else
          {
            std::cerr << "Camera instance is null." << std::endl;
          }
        }
      }
      catch (const std::exception& e)
      {
        std::cerr << "An error occurred: " << e.what() << std::endl;
        break;
      }
    }
    std::cout << "PositionData worker stopped." << std::endl;
  }

  void stopWorker()
  {
    stop_event.store(true);
  }

private:
  Camera&           camera     = Camera::getInstance();
  int               current_id = 0;
  std::atomic<bool> stop_event;
  bool              OPC_Connect;
};*/
/*
void start_communication_worker()
{
  std::promise<void>       exit_signal;
  std::shared_future<void> future_obj = exit_signal.get_future();
  bip::message_queue       mq(bip::open_only, "message_queue");

  Communication comm;
  comm.worker(mq, exit_signal, future_obj);
}

void start_positiondata_worker()
{
  std::promise<void>       exit_signal;
  std::shared_future<void> future_obj = exit_signal.get_future();
  bip::message_queue       mq(bip::open_only, "message_queue");

  PositionData pos;
  pos.worker(mq, exit_signal, future_obj);
}*/
namespace bp  = boost::process;
namespace bip = boost::interprocess;

int main(int argc, char* argv[])
{
  /* if (argc > 1)
  {
    std::string arg = argv[1];
    if (arg == "--comm")
    {
      start_communication_worker();
      return 0;
    }
    else if (arg == "--pos")
    {
      start_positiondata_worker();
      return 0;
    }
  }*/
  std::promise<void>        exit_signal;
  std::shared_future<void>  future_obj = exit_signal.get_future();

  bip::message_queue::remove("message_queue");
  bip::message_queue mq(bip::create_only, "message_queue", 100, sizeof(std::string));

  bip::message_queue::remove("op_connect_queue");
  bip::message_queue op_connect_queue(bip::create_only, "op_connect_queue", 100, sizeof(bool));

  //bp::child comm_process(bp::search_path("app"), bp::args = {"--comm"});
  //bp::child pos_process(bp::search_path("app"), bp::args = {"--pos"});

  try
  {
    auto comm_worker_path = bp::search_path("comm_worker");
    auto pos_worker_path  = bp::search_path("pos_worker");

    if (comm_worker_path.empty() || pos_worker_path.empty())
    {
      std::cerr << "Error: comm_worker or pos_worker not found in PATH." << std::endl;
      return 1;
    }

    bp::child comm_process(bp::search_path("comm_worker"));
    bp::child pos_process(bp::search_path("pos_worker"));

    std::this_thread::sleep_for(std::chrono::seconds(10)); // Run for 10 seconds

    comm_process.wait();
    pos_process.wait();
  }
  catch (const boost::process::process_error& e)
  {
    std::cerr << "Process error: " << e.what() << std::endl;
    return 1;
  }
  catch (const std::exception& e)
  {
    std::cerr << "An error occurred: " << e.what() << std::endl;
    return 1;
  }

  bip::message_queue::remove("message_queue");
  bip::message_queue::remove("op_connect_queue");
   
  std::cout << "Processes stopped." << std::endl;

  return 0;
  //bp::child comm_process(bp::search_path("comm_worker"));
  //bp::child pos_process(bp::search_path("pos_worker"));

  //std::this_thread::sleep_for(std::chrono::seconds(10)); // Run for 10 seconds
 
  //std::cout << "Stopping processes..." << std::endl;
  //exit_signal.set_value();
  //comm.stopWorker();
  //pos.stopWorker();

  //comm_process.wait();
  //pos_process.wait();

  //bip::message_queue::remove("message_queue");
  //std::cout << "Processes stopped." << std::endl;

  //return 0;
}
