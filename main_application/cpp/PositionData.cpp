#include <application_base_modules/Camera.h>
#include <boost/interprocess/ipc/message_queue.hpp>
#include <future>
#include <iostream>
#include <nlohmann/json.hpp>
#include <application_base_modules/PLCData.h>

namespace bip = boost::interprocess;
std::stringstream ss;

class PositionData
{
public:
  PositionData() : stop_event(false), OPC_Connect(false) {}

  pcl::PointCloud<pcl::PointXYZ> contours;
  Eigen::Vector3d centroid;
  Eigen::Vector3d point_color;

  void worker(bip::message_queue& mq,
              bip::message_queue& op_connect_queue,
              std::promise<void>& exit_signal,
              std::shared_future<void>& future_obj)
  {
    std::cout << "PositionData worker started." << std::endl;
    while (future_obj.wait_for(std::chrono::milliseconds(1)) == std::future_status::timeout)
    {
      //std::cout << "PositionData worker running." << std::endl;
      try
      {
        bip::message_queue::size_type recvd_size;
        unsigned int                  priority;
        op_connect_queue.try_receive(&OPC_Connect, sizeof(OPC_Connect), recvd_size, priority);

        if (OPC_Connect)
        {
          if (&camera != nullptr)
          {
			auto time_start = std::chrono::high_resolution_clock::now();
            std::cout << "Processing frame..." << std::endl;
            camera.initializeStream();
            camera.processFrame(false, false, false, false, false, false);
            //std::cout << "Setting depth range..." << std::endl;
            camera.setDepthRange(std::make_tuple(0.554, 0.569));
            //camera.setDepthRange(std::make_tuple(0.5065, 0.512));
            std::tie(contours, centroid, point_color) = camera.getContours(false, eps, min_samples);
            current_id = current_id + 1;
            //std::cout << "Centroid: " << centroid << std::endl;
            //std::cout << "Point color: " << point_color << std::endl;
            //std::cout << "Contour size: " << contours.size() << std::endl;

            nlohmann::json json_obj;
            json_obj["x"] = centroid(0)*100;
            json_obj["y"] = centroid(1)*100;
            json_obj["z"] = centroid(2)*100;
            json_obj["color"] = 2;
            json_obj["id"] = current_id;
			std::cout << "time elapsed: " << camera.getTimestampMS() - timestamp << std::endl;
			timestamp = camera.getTimestampMS();
            json_obj["timestamp"] = timestamp;// timestamp_value;
            std::cout << "timestamp: " << timestamp << std::endl;

            std::string json_data = json_obj.dump();
			std::cout << "json_data: " << json_data << std::endl;
			//std::cout << "Json data size: " << json_data.size() << std::endl;
            if (json_data.size() > 4096)
            {
              std::cerr << "Error: JSON data size exceeds message queue limit." << std::endl;
              continue;
            }
			mq.send(json_data.c_str(), json_data.size(), 0);
			printTimestamp();

			if (current_id >= 100)
			{
				current_id = 0;
			}
			auto time_end = std::chrono::high_resolution_clock::now();
			std::chrono::duration<double> elapsed_seconds = time_end - time_start;
			std::cout << "Position Data processing time: " << elapsed_seconds.count() << "s\n";
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
  Camera& camera = Camera::getInstance();
  int current_id = 0;
  std::atomic<bool> stop_event;
  bool OPC_Connect;
  uint64_t timestamp;
  double            eps         = 0.05;
  int               min_samples = 15;

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

  PositionData pos;
  pos.worker(mq, op_connect_queue, exit_signal, future_obj);

  return 0;
}
